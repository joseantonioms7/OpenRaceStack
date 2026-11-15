"""
3D 15-DOF Error-State Kalman Filter (ESKF) on SE(3) x R^6 for Autonomous Vehicles.

State Representation:
    Nominal State (16-D):
        x = [p (3), v (3), q (4), b_a (3), b_omega (3)]^T
            p       : Global Cartesian 3D position [x, y, z] (m)
            v       : Global linear velocity [vx, vy, vz] (m/s)
            q       : Orientation unit quaternion [qw, qx, qy, qz] on SO(3)
            b_a     : Accelerometer sensor bias [b_ax, b_ay, b_az] (m/s^2)
            b_omega : Gyroscope sensor bias [b_wx, b_wy, b_wz] (rad/s)

    Error State (15-DOF):
        delta_x = [delta_p (3), delta_v (3), delta_theta (3), delta_b_a (3), delta_b_omega (3)]^T in R^15
            delta_p     : 3D position error
            delta_v     : 3D velocity error
            delta_theta : 3D Lie algebra tangent-space rotation error in so(3)
            delta_b_a   : Accelerometer bias error
            delta_b_w   : Gyroscope bias error

Key Architectural Features:
    - High-rate strapdown inertial mechanization on SE(3) with closed-form Rodrigues integration.
    - Continuous-to-discrete error state propagation with analytical Jacobians.
    - True Error-State reset and state injection (manifold retraction on SO(3)).
    - Symmetric Joseph stabilized covariance update guaranteeing positive semi-definiteness P >= 0.
    - Chi-squared (NIS) Mahalanobis outlier gating on position, velocity, and orientation updates.
    - Multi-sensor updates: 3D GNSS, Heading/Dual-antenna, Wheel Odometry, 6D LiDAR/Visual Pose.
"""

import math
from collections.abc import Sequence

import numpy as np

from .types import wrap_angle


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """Computes 3x3 skew-symmetric cross-product matrix [v]_x."""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=np.float64)


def quat_to_rot_mat(q: np.ndarray) -> np.ndarray:
    """Converts unit quaternion [qw, qx, qy, qz] to 3x3 rotation matrix R (body to global)."""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return np.array([
        [1.0 - 2.0 * (qy**2 + qz**2), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx**2 + qz**2), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx**2 + qy**2)]
    ], dtype=np.float64)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton quaternion product q = q1 (x) q2."""
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ], dtype=np.float64)


def quat_from_axis_angle(rot_vec: np.ndarray) -> np.ndarray:
    """Converts 3D rotation vector in so(3) to unit quaternion via exponential map."""
    angle = float(np.linalg.norm(rot_vec))
    if angle < 1e-8:
        return np.array([1.0, 0.5 * rot_vec[0], 0.5 * rot_vec[1], 0.5 * rot_vec[2]], dtype=np.float64)
    axis = rot_vec / angle
    half_angle = 0.5 * angle
    s = math.sin(half_angle)
    return np.array([math.cos(half_angle), axis[0] * s, axis[1] * s, axis[2] * s], dtype=np.float64)


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Converts ZYX Euler angles (roll, pitch, yaw) in radians to unit quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ], dtype=np.float64)


def quat_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    """Extracts (roll, pitch, yaw) in radians from unit quaternion [qw, qx, qy, qz]."""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx**2 + qy**2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ErrorStateKalmanFilter3D:
    """
    15-DOF Error-State Extended Kalman Filter for 3D Autonomous Vehicle Localization.
    """

    GRAVITY_ENU: np.ndarray = np.array([0.0, 0.0, -9.80665], dtype=np.float64)

    def __init__(
        self,
        dt_default: float = 0.01,
        sigma_acc_noise: float = 0.15,         # Accelerometer continuous noise (m/s^2 / sqrt(Hz))
        sigma_gyr_noise: float = 0.015,        # Gyroscope continuous noise (rad/s / sqrt(Hz))
        sigma_acc_bias_walk: float = 0.001,    # Accelerometer bias random walk (m/s^3 / sqrt(Hz))
        sigma_gyr_bias_walk: float = 0.0001,   # Gyroscope bias random walk (rad/s^2 / sqrt(Hz))
    ):
        self.dt = float(dt_default)
        self.sigma_an = float(sigma_acc_noise)
        self.sigma_wn = float(sigma_gyr_noise)
        self.sigma_aw = float(sigma_acc_bias_walk)
        self.sigma_ww = float(sigma_gyr_bias_walk)

        # Nominal State: 16-D
        self.p = np.zeros(3, dtype=np.float64)           # [px, py, pz]
        self.v = np.zeros(3, dtype=np.float64)           # [vx, vy, vz]
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # [qw, qx, qy, qz]
        self.b_a = np.zeros(3, dtype=np.float64)         # [b_ax, b_ay, b_az]
        self.b_w = np.zeros(3, dtype=np.float64)         # [b_wx, b_wy, b_wz]

        # 15x15 Error State Covariance
        self.P = np.eye(15, dtype=np.float64) * 0.1
        self.P[0:3, 0:3] *= 0.5    # Position cov
        self.P[3:6, 3:6] *= 0.1    # Velocity cov
        self.P[6:9, 6:9] *= 0.01   # Orientation cov
        self.P[9:12, 9:12] *= 0.01 # Accel bias cov
        self.P[12:15, 12:15] *= 0.001 # Gyro bias cov

        # Preallocated Identity matrices
        self._I15 = np.eye(15, dtype=np.float64)
        self._I3 = np.eye(3, dtype=np.float64)

        # Diagnostics & Health statistics
        self.last_nis: float = 0.0
        self.consecutive_rejections: int = 0
        self.channel_nis: dict[str, float] = {}

    def initialize_state(
        self,
        p0: Sequence[float],
        v0: Sequence[float] | None = None,
        yaw0: float = 0.0,
        pitch0: float = 0.0,
        roll0: float = 0.0,
        P0_diag: Sequence[float] | None = None,
    ) -> None:
        """Initializes 3D nominal state vector and 15x15 error covariance."""
        self.p = np.asarray(p0, dtype=np.float64).flatten()[:3]
        self.v = np.asarray(v0 if v0 is not None else [0.0, 0.0, 0.0], dtype=np.float64).flatten()[:3]
        self.q = quat_from_euler(roll0, pitch0, yaw0)
        self.b_a.fill(0.0)
        self.b_w.fill(0.0)

        if P0_diag is not None:
            diag_arr = np.asarray(P0_diag, dtype=np.float64)
            if len(diag_arr) == 15:
                self.P = np.diag(diag_arr)
        else:
            self.P = np.eye(15, dtype=np.float64) * 0.1
            self.P[0:3, 0:3] *= 0.2
            self.P[3:6, 3:6] *= 0.05
            self.P[6:9, 6:9] *= 0.005
            self.P[9:12, 9:12] *= 0.002
            self.P[12:15, 12:15] *= 0.0002

    def predict_imu(
        self,
        acc_meas: Sequence[float],
        gyr_meas: Sequence[float],
        dt: float | None = None
    ) -> None:
        """
        High-rate inertial propagation using 3D accelerometer and gyroscope observations.
        Propagates nominal 16D state via strapdown mechanization and propagates 15x15 error covariance.
        """
        dt_val = self.dt if dt is None else float(dt)
        if dt_val <= 0.0:
            return

        a_m = np.asarray(acc_meas, dtype=np.float64).flatten()[:3]
        w_m = np.asarray(gyr_meas, dtype=np.float64).flatten()[:3]

        # 1. Unbiased body inertial measurements
        a_b = a_m - self.b_a
        w_b = w_m - self.b_w

        # 2. Nominal strapdown mechanization
        R_b = quat_to_rot_mat(self.q)
        a_g = R_b @ a_b + self.GRAVITY_ENU

        # Position and velocity propagation
        self.p += self.v * dt_val + 0.5 * a_g * (dt_val**2)
        self.v += a_g * dt_val

        # Quaternion integration via exponential map
        delta_q = quat_from_axis_angle(w_b * dt_val)
        self.q = quat_multiply(self.q, delta_q)
        # Normalize unit quaternion to prevent numerical drift
        q_norm = np.linalg.norm(self.q)
        if q_norm > 1e-9:
            self.q /= q_norm

        # 3. Error-State Transition Jacobian F_x (15x15)
        F_x = self._I15.copy()
        dt2 = 0.5 * (dt_val**2)

        skew_ab = skew_symmetric(a_b)
        R_skew_ab = R_b @ skew_ab

        F_x[0:3, 3:6] = self._I3 * dt_val
        F_x[0:3, 6:9] = -0.5 * R_skew_ab * dt2
        F_x[0:3, 9:12] = -0.5 * R_b * dt2

        F_x[3:6, 6:9] = -R_skew_ab * dt_val
        F_x[3:6, 9:12] = -R_b * dt_val

        # Rotation error propagation
        R_w_dt = quat_to_rot_mat(delta_q).T
        F_x[6:9, 6:9] = R_w_dt
        F_x[6:9, 12:15] = -self._I3 * dt_val

        # 4. Discrete Process Noise Covariance Q_d (15x15)
        Q_d = np.zeros((15, 15), dtype=np.float64)
        var_an = (self.sigma_an**2) * dt_val
        var_wn = (self.sigma_wn**2) * dt_val
        var_aw = (self.sigma_aw**2) * dt_val
        var_ww = (self.sigma_ww**2) * dt_val

        Q_d[0:3, 0:3] = self._I3 * (var_an * dt2**2)
        Q_d[3:6, 3:6] = self._I3 * var_an
        Q_d[6:9, 6:9] = self._I3 * var_wn
        Q_d[9:12, 9:12] = self._I3 * var_aw
        Q_d[12:15, 12:15] = self._I3 * var_ww

        # Covariance propagation: P = F_x * P * F_x^T + Q_d
        self.P = F_x @ self.P @ F_x.T + Q_d
        self.P = 0.5 * (self.P + self.P.T)  # Enforce symmetry

    def _inject_error_state_and_reset(self, delta_x: np.ndarray) -> None:
        """
        Manifold state injection and error state reset:
        x_nominal <- x_nominal (+) delta_x, delta_x <- 0.
        """
        # Inject position and velocity
        self.p += delta_x[0:3]
        self.v += delta_x[3:6]

        # Inject orientation on SO(3) via quaternion exponential map
        delta_theta = delta_x[6:9]
        dq = quat_from_axis_angle(delta_theta)
        self.q = quat_multiply(self.q, dq)
        q_norm = np.linalg.norm(self.q)
        if q_norm > 1e-9:
            self.q /= q_norm

        # Inject sensor biases
        self.b_a += delta_x[9:12]
        self.b_w += delta_x[12:15]

    def update_gnss_pos(
        self,
        z_pos: Sequence[float],
        R_pos: np.ndarray,
        gate_threshold: float = 13.8155  # 99.9% chi2 for 3 DOF
    ) -> bool:
        """Fuses 3D absolute GNSS position [x, y, z]."""
        z = np.asarray(z_pos, dtype=np.float64).flatten()[:3]
        H = np.zeros((3, 15), dtype=np.float64)
        H[0:3, 0:3] = self._I3

        # Innovation residual
        nu = z - self.p
        S = H @ self.P @ H.T + R_pos

        # Mahalanobis gating
        try:
            S_inv = np.linalg.inv(S)
            nis = float(nu.T @ S_inv @ nu)
            self.last_nis = nis
            self.channel_nis["gnss_pos"] = nis

            if nis > gate_threshold:
                self.consecutive_rejections += 1
                return False

            self.consecutive_rejections = 0
            # Kalman gain
            K = self.P @ H.T @ S_inv
            delta_x = K @ nu

            # Joseph stabilized covariance update
            I_KH = self._I15 - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ R_pos @ K.T
            self.P = 0.5 * (self.P + self.P.T)

            # State injection
            self._inject_error_state_and_reset(delta_x)
            return True
        except np.linalg.LinAlgError:
            return False

    def update_heading(
        self,
        z_yaw: float,
        var_yaw: float = 0.04**2,
        gate_threshold: float = 9.21  # 99% chi2 for 1 DOF
    ) -> bool:
        """Fuses absolute heading / course-over-ground observation in radians."""
        _, _, current_yaw = quat_to_euler(self.q)
        nu_yaw = float(wrap_angle(z_yaw - current_yaw))

        # Observation matrix H maps delta_theta_z to heading residual
        H = np.zeros((1, 15), dtype=np.float64)
        H[0, 8] = 1.0  # delta_theta_z

        S = float(self.P[8, 8] + var_yaw)
        nis = (nu_yaw**2) / max(S, 1e-9)
        self.last_nis = nis
        self.channel_nis["heading"] = nis

        if nis > gate_threshold:
            return False

        K = self.P @ H.T / S  # (15, 1)
        delta_x = (K * nu_yaw).flatten()

        I_KH = self._I15 - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K * var_yaw @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        self._inject_error_state_and_reset(delta_x)
        return True

    def update_wheel_odometry(
        self,
        v_wheel_forward: float,
        var_v: float = 0.08**2,
        gate_threshold: float = 9.21
    ) -> bool:
        """Fuses forward longitudinal wheel speed in body frame."""
        R_b = quat_to_rot_mat(self.q)
        # Forward axis in global frame is first column of R_b
        fwd_axis = R_b[:, 0]

        v_fwd_est = float(np.dot(self.v, fwd_axis))
        nu = v_wheel_forward - v_fwd_est

        # Observation matrix H: d(v_fwd)/d(delta_v) = fwd_axis^T, d(v_fwd)/d(delta_theta) = (v x fwd_axis)^T
        H = np.zeros((1, 15), dtype=np.float64)
        H[0, 3:6] = fwd_axis
        H[0, 6:9] = np.cross(self.v, fwd_axis)

        S = float(H @ self.P @ H.T + var_v)
        nis = (nu**2) / max(S, 1e-9)
        self.last_nis = nis
        self.channel_nis["wheel"] = nis

        if nis > gate_threshold:
            return False

        K = self.P @ H.T / S
        delta_x = (K * nu).flatten()

        I_KH = self._I15 - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K * var_v @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        self._inject_error_state_and_reset(delta_x)
        return True

    def update_pose_6d(
        self,
        p_meas: Sequence[float],
        q_meas: Sequence[float],
        R_cov_6x6: np.ndarray,
        gate_threshold: float = 22.458  # 99.9% chi2 for 6 DOF
    ) -> bool:
        """Fuses full 6-DOF relative/global pose from LiDAR scan matching or Visual Odometry."""
        z_p = np.asarray(p_meas, dtype=np.float64).flatten()[:3]
        z_q = np.asarray(q_meas, dtype=np.float64).flatten()[:4]

        # Position residual
        nu_p = z_p - self.p

        # Orientation residual delta_theta in so(3)
        # q_diff = q_meas (x) q_est^(-1)
        q_est_inv = np.array([self.q[0], -self.q[1], -self.q[2], -self.q[3]], dtype=np.float64)
        q_diff = quat_multiply(z_q, q_est_inv)
        # Normalize
        q_diff /= max(np.linalg.norm(q_diff), 1e-9)
        # Extract small angle vector
        if q_diff[0] < 0:
            q_diff = -q_diff
        nu_theta = 2.0 * q_diff[1:4]

        nu = np.concatenate([nu_p, nu_theta])  # (6,)

        H = np.zeros((6, 15), dtype=np.float64)
        H[0:3, 0:3] = self._I3
        H[3:6, 6:9] = self._I3

        S = H @ self.P @ H.T + R_cov_6x6
        try:
            S_inv = np.linalg.inv(S)
            nis = float(nu.T @ S_inv @ nu)
            self.last_nis = nis
            self.channel_nis["pose_6d"] = nis

            if nis > gate_threshold:
                return False

            K = self.P @ H.T @ S_inv
            delta_x = K @ nu

            I_KH = self._I15 - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ R_cov_6x6 @ K.T
            self.P = 0.5 * (self.P + self.P.T)

            self._inject_error_state_and_reset(delta_x)
            return True
        except np.linalg.LinAlgError:
            return False

    def get_state_snapshot(self) -> dict[str, np.ndarray]:
        """Returns nominal state snapshot and error covariance."""
        roll, pitch, yaw = quat_to_euler(self.q)
        return {
            "p": self.p.copy(),
            "v": self.v.copy(),
            "q": self.q.copy(),
            "euler_rad": np.array([roll, pitch, yaw], dtype=np.float64),
            "euler_deg": np.degrees([roll, pitch, yaw]),
            "b_a": self.b_a.copy(),
            "b_w": self.b_w.copy(),
            "P": self.P.copy(),
            "P_pos_trace": float(np.trace(self.P[0:3, 0:3])),
            "P_total_trace": float(np.trace(self.P)),
        }
