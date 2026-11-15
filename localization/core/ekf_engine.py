import logging
import math
import threading
from collections.abc import Sequence
from typing import Any

import numpy as np

from .types import wrap_angle

logger = logging.getLogger(__name__)


class LocalizationEKF:
    """
    5-State Extended Kalman Filter for planar autonomous vehicle localization.

    State vector:
        x = [px, py, theta, v, omega]^T  (5x1)
            px, py : Global Cartesian position (m)
            theta  : Heading / yaw angle in [-pi, pi) (rad)
            v      : Longitudinal linear velocity (m/s)
            omega  : Yaw rate (rad/s)

    Key Mathematical & Engineering Features:
        - Exact Constant Turn Rate and Velocity (CTRV) circular-arc state integration
          with analytical Jacobian G (including d(px,py)/d(omega) second-order terms)
          and smooth Taylor fallback as |omega| -> 0.
        - Manifold-aware angle normalization on S^1 for state and angular innovations.
        - Continuous-to-discrete dynamic process noise Q(theta, dt) projecting physical
          longitudinal and angular acceleration variances into Cartesian coordinates.
        - Pre-allocated workspace matrices for zero-allocation hot-loop execution.
        - Cholesky / closed-form SPD linear solver (eliminating explicit np.linalg.inv).
        - Symmetric Joseph stabilized covariance update guaranteeing P = P^T >= 0.
        - Chi-squared (Mahalanobis NIS) outlier gating with consecutive-rejection recovery.
        - Modular multi-sensor fusion (GPS position, GPS+heading pose, IMU+wheel twist,
          map corridor boundary constraints, and delayed OOSM ring-buffer replay).
    """

    EPS_OMEGA: float = 1e-5

    def __init__(
        self,
        dt: float,
        Q_diag: Sequence[float],
        R_diag: Sequence[float],
        sigma_a: float = 1.5,
        sigma_alpha: float = 0.8,
        use_dynamic_Q: bool = True,
        gate_threshold: float | None = 13.8155,  # 99.9% chi2 for 2 DOF
        max_consecutive_rejections: int = 5,
        history_maxlen: int = 100,
    ) -> None:
        if dt <= 0.0 or not math.isfinite(dt):
            raise ValueError(f"Default dt must be finite and > 0, got {dt}")
        if len(Q_diag) != 5:
            raise ValueError(f"Q_diag must have 5 elements, got {len(Q_diag)}")
        if len(R_diag) < 1:
            raise ValueError("R_diag must have at least 1 element")

        self.dt: float = float(dt)
        self.x: np.ndarray = np.zeros((5, 1), dtype=np.float64)
        self.P: np.ndarray = np.eye(5, dtype=np.float64) * 0.1

        self.Q_base: np.ndarray = np.diag(np.asarray(Q_diag, dtype=np.float64))
        self.Q: np.ndarray = self.Q_base.copy()
        self.R: np.ndarray = np.diag(np.asarray(R_diag, dtype=np.float64))

        self.sigma_a: float = float(sigma_a)
        self.sigma_alpha: float = float(sigma_alpha)
        self.use_dynamic_Q: bool = bool(use_dynamic_Q)

        self.gate_threshold: float | None = gate_threshold
        self.max_consecutive_rejections: int = int(max_consecutive_rejections)
        self.consecutive_rejections: int = 0
        self._channel_rejections: dict[Any, int] = {}
        self.channel_nis: dict[str, float] = {}
        self.total_rejections: int = 0
        self.last_nis: float = 0.0

        # Pre-allocated constant observation matrices (zero-allocation in update loop)
        self.H_pos: np.ndarray = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )
        self.H_pose: np.ndarray = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )
        self.H_twist: np.ndarray = np.array(
            [
                [0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self.H_pos_twist: np.ndarray = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self.H_full: np.ndarray = np.eye(5, dtype=np.float64)
        self.H: np.ndarray = self.H_pos

        # Pre-allocated workspace buffers for predict() and update()
        self._I5: np.ndarray = np.eye(5, dtype=np.float64)
        self._G: np.ndarray = np.eye(5, dtype=np.float64)
        self._Gamma: np.ndarray = np.zeros((5, 2), dtype=np.float64)
        self._Gamma_Qu: np.ndarray = np.zeros((5, 2), dtype=np.float64)
        self._Q_buf: np.ndarray = np.zeros((5, 5), dtype=np.float64)
        self._Qu: np.ndarray = np.diag([self.sigma_a**2, self.sigma_alpha**2])
        self._H_corridor: np.ndarray = np.zeros((1, 5), dtype=np.float64)
        self._R_corridor: np.ndarray = np.zeros((1, 1), dtype=np.float64)
        self._z_corridor: np.ndarray = np.zeros((1, 1), dtype=np.float64)

        # Thread synchronization
        self._lock: threading.RLock = threading.RLock()
        self._current_time: float = 0.0

    def initialize_state(
        self,
        x0: np.ndarray,
        P0: np.ndarray | None = None,
        timestamp: float = 0.0,
    ) -> None:
        """Initialize filter state vector and covariance with angle normalization."""
        with self._lock:
            arr = np.asarray(x0, dtype=np.float64).reshape(5, 1)
            if not np.all(np.isfinite(arr)):
                raise ValueError("Initial state x0 contains non-finite values")
            self.x = arr.copy()
            self.x[2, 0] = float(wrap_angle(self.x[2, 0]))
            if P0 is not None:
                P_arr = np.asarray(P0, dtype=np.float64).reshape(5, 5)
                self.P = 0.5 * (P_arr + P_arr.T)
            self._current_time = float(timestamp)

    def _compute_process_noise(
        self, theta: float, dt: float, omega: float = 0.0
    ) -> np.ndarray:
        """
        Compute continuous-to-discrete process noise covariance Q(theta, omega, dt).
        Projects physical body-frame accelerations along the midpoint circular-arc
        orientation (theta + 0.5 * omega * dt) via Gamma * Qu * Gamma^T and adds
        dt-scaled full-rank baseline regularization for positive-definiteness.
        """
        dt_ratio = dt / self.dt
        if not self.use_dynamic_Q:
            np.multiply(self.Q_base, dt_ratio, out=self._Q_buf)
            return self._Q_buf

        mid_theta = theta + 0.5 * omega * dt
        c = math.cos(mid_theta)
        s = math.sin(mid_theta)
        half_dt2 = 0.5 * dt * dt

        self._Gamma.fill(0.0)
        self._Gamma[0, 0] = half_dt2 * c
        self._Gamma[1, 0] = half_dt2 * s
        self._Gamma[2, 1] = half_dt2
        self._Gamma[3, 0] = dt
        self._Gamma[4, 1] = dt

        np.matmul(self._Gamma, self._Qu, out=self._Gamma_Qu)
        np.matmul(self._Gamma_Qu, self._Gamma.T, out=self._Q_buf)
        self._Q_buf += self.Q_base * dt_ratio
        return self._Q_buf

    def predict(self, dt_actual: float | None = None) -> None:
        """
        Propagate state x and error covariance P forward by dt using exact CTRV
        kinematics (with Taylor series limit for |omega| -> 0).
        """
        with self._lock:
            dt = self.dt if dt_actual is None else float(dt_actual)
            if not math.isfinite(dt) or dt < 0.0:
                raise ValueError(f"dt must be finite and non-negative, got {dt}")
            if dt == 0.0:
                return

            theta = float(self.x[2, 0])
            v = float(self.x[3, 0])
            omega = float(self.x[4, 0])

            G = self._G
            G.fill(0.0)
            G[0, 0] = 1.0
            G[1, 1] = 1.0
            G[2, 2] = 1.0
            G[3, 3] = 1.0
            G[4, 4] = 1.0

            if abs(omega) > self.EPS_OMEGA:
                theta_new = theta + omega * dt
                sin_theta = math.sin(theta)
                cos_theta = math.cos(theta)
                sin_theta_new = math.sin(theta_new)
                cos_theta_new = math.cos(theta_new)
                inv_omega = 1.0 / omega
                v_over_omega = v * inv_omega

                # Exact circular-arc state integration
                dx = v_over_omega * (sin_theta_new - sin_theta)
                dy = v_over_omega * (-cos_theta_new + cos_theta)

                # Analytical CTRV Jacobian elements
                G[0, 2] = v_over_omega * (cos_theta_new - cos_theta)
                G[0, 3] = inv_omega * (sin_theta_new - sin_theta)
                G[0, 4] = (v * dt * inv_omega) * cos_theta_new - (
                    v * inv_omega * inv_omega
                ) * (sin_theta_new - sin_theta)

                G[1, 2] = v_over_omega * (sin_theta_new - sin_theta)
                G[1, 3] = inv_omega * (-cos_theta_new + cos_theta)
                G[1, 4] = (v * dt * inv_omega) * sin_theta_new - (
                    v * inv_omega * inv_omega
                ) * (-cos_theta_new + cos_theta)
            else:
                # Second-order midpoint Taylor expansion for near-zero yaw rate
                mid_theta = theta + 0.5 * omega * dt
                sin_mid = math.sin(mid_theta)
                cos_mid = math.cos(mid_theta)
                half_dt = 0.5 * dt

                dx = v * cos_mid * dt
                dy = v * sin_mid * dt

                G[0, 2] = -v * sin_mid * dt
                G[0, 3] = cos_mid * dt
                G[0, 4] = -v * sin_mid * dt * half_dt

                G[1, 2] = v * cos_mid * dt
                G[1, 3] = sin_mid * dt
                G[1, 4] = v * cos_mid * dt * half_dt

            G[2, 4] = dt

            # Update state on manifold
            self.x[0, 0] += dx
            self.x[1, 0] += dy
            self.x[2, 0] = float(wrap_angle(theta + omega * dt))

            # Update covariance P = G * P * G^T + Q(theta, omega, dt)
            self.Q = self._compute_process_noise(theta, dt, omega)
            P_pred = G @ self.P @ G.T + self.Q
            self.P = 0.5 * (P_pred + P_pred.T)

            self._current_time += dt

    def _select_default_observation_model(
        self, m: int
    ) -> tuple[np.ndarray, tuple[int, ...]]:
        """Select pre-allocated H and angular row indices based on measurement size."""
        if m == 2:
            return self.H_pos, ()
        if m == 3:
            return self.H_pose, (2,)
        if m == 4:
            return self.H_pos_twist, ()
        if m == 5:
            return self.H_full, (2,)
        raise ValueError(f"Unsupported measurement vector dimension: {m}")

    def update(
        self,
        z: np.ndarray,
        R: np.ndarray | None = None,
        H: np.ndarray | None = None,
        gate_threshold: float | None = None,
        angle_indices: Sequence[int] | None = None,
        channel: str | None = None,
    ) -> bool:
        """
        Perform measurement update using Joseph stabilized covariance form,
        Cholesky / Sylvester SPD verification, and chi-squared Mahalanobis gating.

        Returns True if the measurement was accepted and fused, or False if it was
        rejected by input validation, non-SPD innovation covariance, or outlier gating.
        """
        with self._lock:
            z_arr = np.asarray(z, dtype=np.float64)
            if z_arr.ndim == 1:
                z_arr = z_arr.reshape(-1, 1)
            elif z_arr.ndim != 2 or z_arr.shape[1] != 1:
                raise ValueError(f"Measurement z must have shape (m, 1), got {z_arr.shape}")

            if not np.all(np.isfinite(z_arr)):
                logger.warning("Ignoring measurement containing NaN/Inf values.")
                return False

            m = z_arr.shape[0]
            if H is None:
                H_mat, default_angle_idx = self._select_default_observation_model(m)
                if angle_indices is None:
                    angle_indices = default_angle_idx
            else:
                H_mat = np.asarray(H, dtype=np.float64)
                if H_mat.shape != (m, 5):
                    raise ValueError(f"H must have shape ({m}, 5), got {H_mat.shape}")
                if angle_indices is None:
                    angle_indices = ()

            ang_tuple = tuple(angle_indices)
            self.H = H_mat

            if R is not None:
                R_mat = np.asarray(R, dtype=np.float64)
            elif self.R.shape == (m, m):
                R_mat = self.R
            else:
                # Safe fallback preserving existing diagonal variance scale
                base_var = float(self.R[0, 0]) if self.R.size > 0 else 2.25
                R_mat = np.eye(m, dtype=np.float64) * base_var

            # 1. Innovation y = z - H * x with S^1 angle wrapping on angular rows
            y = z_arr - (H_mat @ self.x)
            for idx in ang_tuple:
                y[idx, 0] = float(wrap_angle(y[idx, 0]))

            # 2. Innovation covariance S = H * P * H^T + R
            PHt = self.P @ H_mat.T
            S = H_mat @ PHt + R_mat
            S = 0.5 * (S + S.T)

            # 3. Strict SPD verification and solve for Kalman gain K = P * H^T * S^{-1}
            if m == 1:
                s00 = float(S[0, 0])
                if not math.isfinite(s00) or s00 <= 1e-12:
                    logger.warning("Non-SPD 1x1 innovation variance S; skipping update.")
                    return False
                inv_s00 = 1.0 / s00
                K = PHt * inv_s00
                nis = float(y[0, 0] * y[0, 0] * inv_s00)
            elif m == 2:
                # Closed-form 2x2 Sylvester SPD solver (s00 > 0, s11 > 0, det > 0)
                s00, s01, s11 = float(S[0, 0]), float(S[0, 1]), float(S[1, 1])
                det = s00 * s11 - s01 * s01
                if not (s00 > 1e-12 and s11 > 1e-12 and det > 1e-12):
                    logger.warning("Non-SPD 2x2 innovation covariance S; skipping update.")
                    return False
                inv_det = 1.0 / det
                S_inv = np.array(
                    [[s11 * inv_det, -s01 * inv_det], [-s01 * inv_det, s00 * inv_det]],
                    dtype=np.float64,
                )
                K = PHt @ S_inv
                S_inv_y = S_inv @ y
                nis = max(0.0, float((y.T @ S_inv_y)[0, 0]))
            else:
                try:
                    # Cholesky factorization S = L * L^T guarantees S is strictly SPD
                    L = np.linalg.cholesky(S)
                    z_whitened = np.linalg.solve(L, y)
                    nis = float(np.dot(z_whitened.ravel(), z_whitened.ravel()))
                    K = np.linalg.solve(L.T, np.linalg.solve(L, PHt.T)).T
                except np.linalg.LinAlgError:
                    logger.warning("Non-SPD innovation covariance S (Cholesky failed); skipping update.")
                    return False

            # 4. Normalized Innovation Squared (NIS / squared Mahalanobis distance)
            self.last_nis = nis
            channel_key = (
                channel
                if channel is not None
                else f"dim_{m}_ang_{ang_tuple}"
            )
            self.channel_nis[str(channel_key)] = nis

            active_gate = self.gate_threshold if gate_threshold is None else gate_threshold
            chan_rej = self._channel_rejections.get(channel_key, 0)
            if (
                active_gate is not None
                and nis > active_gate
                and chan_rej < self.max_consecutive_rejections
            ):
                self._channel_rejections[channel_key] = chan_rej + 1
                self.consecutive_rejections = self._channel_rejections[channel_key]
                self.total_rejections += 1
                logger.debug(
                    "Measurement on channel '%s' rejected by chi2 gate (NIS=%.3f > %.3f).",
                    channel_key,
                    nis,
                    active_gate,
                )
                return False

            self._channel_rejections[channel_key] = 0
            self.consecutive_rejections = 0

            # 5. State correction & S^1 yaw normalization
            self.x = self.x + (K @ y)
            self.x[2, 0] = float(wrap_angle(self.x[2, 0]))

            # 6. Symmetric Joseph stabilized covariance update:
            #    P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
            I_KH = self._I5 - (K @ H_mat)
            P_upd = (I_KH @ self.P @ I_KH.T) + (K @ R_mat @ K.T)
            self.P = 0.5 * (P_upd + P_upd.T)

            return True

    def update_gps(
        self,
        z_xy: np.ndarray,
        R_gps: np.ndarray | None = None,
        gate_threshold: float | None = None,
    ) -> bool:
        """Fuse a 2D global position measurement [x, y]^T."""
        return self.update(
            z_xy,
            R=R_gps,
            H=self.H_pos,
            gate_threshold=gate_threshold,
            angle_indices=(),
            channel="gps",
        )

    def update_pose(
        self,
        z_xytheta: np.ndarray,
        R_pose: np.ndarray | None = None,
        gate_threshold: float | None = None,
    ) -> bool:
        """Fuse a 3D planar pose measurement [x, y, theta]^T."""
        return self.update(
            z_xytheta,
            R=R_pose,
            H=self.H_pose,
            gate_threshold=gate_threshold,
            angle_indices=(2,),
            channel="pose",
        )

    def update_imu_odom(
        self,
        z_v_omega: np.ndarray,
        R_prop: np.ndarray | None = None,
        gate_threshold: float | None = None,
    ) -> bool:
        """Fuse a 2D proprioceptive twist measurement [v_wheel, omega_imu]^T."""
        return self.update(
            z_v_omega,
            R=R_prop,
            H=self.H_twist,
            gate_threshold=gate_threshold,
            angle_indices=(),
            channel="imu_odom",
        )

    def constrain_to_corridor(
        self,
        center_xy: np.ndarray,
        heading_ref: float | None = None,
        w_right: float | np.ndarray = 1.1,
        w_left: float | None = None,
        margin: float = 0.05,
        sigma_bound: float = 0.05,
    ) -> bool:
        """
        Enforce physical track-boundary corridor constraint in the local Frenet frame.
        Supports both a single reference point (center_xy, heading_ref, w_right, w_left)
        and full map-matched lookup when center_xy is a (N, 2) centerline array and
        w_right is a (N, 2) track_widths array (eliminating ground-truth oracle leakage).
        Applies a Joseph-stabilized 1D scalar Kalman measurement update along the
        local track normal vector so both state x and covariance P remain consistent.
        """
        with self._lock:
            c_arr = np.asarray(center_xy, dtype=np.float64)
            px = float(self.x[0, 0])
            py = float(self.x[1, 0])

            if c_arr.ndim == 2 and c_arr.shape[0] >= 2:
                # Map-matched projection onto nearest centerline segment to estimated (px, py)
                dists_sq = (c_arr[:, 0] - px) ** 2 + (c_arr[:, 1] - py) ** 2
                idx = int(np.argmin(dists_sq))
                n_pts = c_arr.shape[0]
                prev_idx = (idx - 1) % n_pts
                next_idx = (idx + 1) % n_pts

                # Choose segment (prev->idx or idx->next) with closest orthogonal projection
                best_cx, best_cy = float(c_arr[idx, 0]), float(c_arr[idx, 1])
                best_psi = float(self.x[2, 0]) if heading_ref is None else float(heading_ref)
                best_wr, best_wl = 1.1, 1.1
                widths_arr = np.asarray(w_right, dtype=np.float64)
                has_w_arr = widths_arr.ndim == 2 and widths_arr.shape[0] == n_pts

                min_seg_dist2 = float("inf")
                for i0, i1 in ((prev_idx, idx), (idx, next_idx)):
                    x0_s, y0_s = float(c_arr[i0, 0]), float(c_arr[i0, 1])
                    s_dx = float(c_arr[i1, 0]) - x0_s
                    s_dy = float(c_arr[i1, 1]) - y0_s
                    seg_len2 = s_dx * s_dx + s_dy * s_dy
                    if seg_len2 <= 1e-12:
                        continue
                    t_proj = ((px - x0_s) * s_dx + (py - y0_s) * s_dy) / seg_len2
                    t_clamped = max(0.0, min(1.0, t_proj))
                    proj_x = x0_s + t_clamped * s_dx
                    proj_y = y0_s + t_clamped * s_dy
                    d2 = (px - proj_x) ** 2 + (py - proj_y) ** 2
                    if d2 < min_seg_dist2:
                        min_seg_dist2 = d2
                        best_cx, best_cy = proj_x, proj_y
                        best_psi = math.atan2(s_dy, s_dx)
                        if has_w_arr:
                            best_wr = (1.0 - t_clamped) * float(widths_arr[i0, 0]) + t_clamped * float(widths_arr[i1, 0])
                            best_wl = (1.0 - t_clamped) * float(widths_arr[i0, 1]) + t_clamped * float(widths_arr[i1, 1])
                cx, cy = best_cx, best_cy
                psi_ref = best_psi
                wr_val, wl_val = best_wr, best_wl
            else:
                cx = float(c_arr[0])
                cy = float(c_arr[1])
                psi_ref = float(self.x[2, 0]) if heading_ref is None else float(heading_ref)
                wr_val = float(w_right)
                wl_val = float(w_right) if w_left is None else float(w_left)

            dx = px - cx
            dy = py - cy
            nx = -math.sin(psi_ref)
            ny = math.cos(psi_ref)
            lat_offset = dx * nx + dy * ny

            max_left = max(0.1, wl_val - margin)
            max_right = max(0.1, wr_val - margin)

            if lat_offset > max_left:
                clamped_lat = max_left
            elif lat_offset < -max_right:
                clamped_lat = -max_right
            else:
                return False

            # Joseph-form 1D scalar Kalman update along normal H_corridor = [nx, ny, 0, 0, 0]
            self._H_corridor.fill(0.0)
            self._H_corridor[0, 0] = nx
            self._H_corridor[0, 1] = ny
            self._R_corridor[0, 0] = max(1e-6, float(sigma_bound) ** 2)
            self._z_corridor[0, 0] = cx * nx + cy * ny + clamped_lat

            self.update(
                self._z_corridor,
                R=self._R_corridor,
                H=self._H_corridor,
                gate_threshold=None,
                angle_indices=(),
                channel="corridor",
            )

            # Guarantee strict geometric boundary compliance if residual excess remains
            dx_post = float(self.x[0, 0]) - cx
            dy_post = float(self.x[1, 0]) - cy
            lat_post = dx_post * nx + dy_post * ny
            if lat_post > max_left or lat_post < -max_right:
                lon_post = dx_post * math.cos(psi_ref) + dy_post * math.sin(psi_ref)
                self.x[0, 0] = cx + lon_post * math.cos(psi_ref) + clamped_lat * nx
                self.x[1, 0] = cy + lon_post * math.sin(psi_ref) + clamped_lat * ny

            return True
