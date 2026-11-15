"""
Dynamic Raceline Optimization and Speed Profiling Layer for OpenRaceStack.

Provides minimum-curvature raceline optimization within track corridor boundaries,
friction-ellipse coupled speed profiling (forward/backward passes), and lap-time
evaluation across diverse vehicle platforms.
"""

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear

from .dynamics import VehicleParameters


@dataclass
class VehicleLimits:
    """Dynamic vehicle physical constraints for raceline speed profiling."""
    max_accel_mps2: float = 6.0         # Longitudinal traction limit a_x,max (m/s^2)
    max_decel_mps2: float = -9.0        # Longitudinal braking limit a_x,min (m/s^2)
    max_lateral_accel_mps2: float = 11.0 # Lateral tire grip limit a_y,max = mu * g (m/s^2)
    max_velocity_mps: float = 35.0      # Engine / top speed limit (m/s)
    drag_coeff: float = 0.35            # Drag coefficient Cd
    frontal_area: float = 0.10          # Frontal area A (m^2)
    downforce_coeff: float = 0.0        # Downforce coefficient Cl
    mass: float = 3.50                  # Mass m (kg)
    air_density: float = 1.225          # Air density rho (kg/m^3)

    @classmethod
    def from_vehicle_params(cls, params: VehicleParameters, friction_coeff: float = 1.1) -> "VehicleLimits":
        """Constructs speed profiling limits directly from VehicleParameters."""
        g = 9.80665
        return cls(
            max_accel_mps2=params.max_accel,
            max_decel_mps2=params.max_decel,
            max_lateral_accel_mps2=friction_coeff * g,
            max_velocity_mps=45.0 if params.mass < 10.0 else 90.0,
            drag_coeff=params.drag_coeff,
            frontal_area=params.frontal_area,
            downforce_coeff=params.downforce_coeff,
            mass=params.mass,
            air_density=params.air_density,
        )


@dataclass
class RacelineProfile:
    """Comprehensive raceline geometry, velocity profile, and telemetry."""
    points: np.ndarray             # Array of shape (N, 2) coordinates [x, y]
    s: np.ndarray                  # Array of shape (N,) cumulative arc length (m)
    tangent_headings: np.ndarray   # Array of shape (N,) path tangent angles (rad)
    curvatures: np.ndarray         # Array of shape (N,) signed curvatures kappa (1/m)
    velocities: np.ndarray         # Array of shape (N,) optimal velocities (m/s)
    ax: np.ndarray                 # Array of shape (N,) longitudinal accelerations (m/s^2)
    ay: np.ndarray                 # Array of shape (N,) lateral accelerations (m/s^2)
    lap_time_s: float              # Theoretical minimum lap time (s)
    track_length_m: float          # Total circuit length (m)
    max_velocity_mps: float        # Peak top speed (m/s)
    min_velocity_mps: float        # Minimum apex speed (m/s)
    avg_velocity_mps: float        # Mean velocity across lap (m/s)


class RacelineOptimizer:
    """
    Minimum-curvature path optimizer and dynamic speed profile generator.
    """

    @staticmethod
    def compute_path_geometry(
        points: np.ndarray,
        is_closed: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Calculates cumulative arc length s, unit tangents, tangent headings,
        and signed curvature kappa for a 2D path.
        Returns: (s, tangents, headings, curvatures).
        """
        n_pts = len(points)
        diffs = np.zeros_like(points)
        for i in range(n_pts):
            next_idx = (i + 1) % n_pts if is_closed else min(i + 1, n_pts - 1)
            diffs[i] = points[next_idx] - points[i]

        seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
        # Replace zero length with small epsilon
        seg_lengths = np.where(seg_lengths < 1e-6, 1e-6, seg_lengths)

        s = np.zeros(n_pts, dtype=np.float64)
        for i in range(1, n_pts):
            s[i] = s[i - 1] + seg_lengths[i - 1]

        tangents = diffs / seg_lengths[:, None]
        headings = np.arctan2(tangents[:, 1], tangents[:, 0])

        if not is_closed and n_pts > 1:
            tangents[-1] = tangents[-2]
            headings[-1] = headings[-2]

        # 3-point circle curvature calculation: kappa = 4*Area / (a*b*c)
        curvatures = np.zeros(n_pts, dtype=np.float64)
        for i in range(n_pts):
            prev_idx = (i - 1) % n_pts if is_closed else max(0, i - 1)
            next_idx = (i + 1) % n_pts if is_closed else min(n_pts - 1, i + 1)

            if prev_idx == i or next_idx == i:
                curvatures[i] = 0.0
                continue

            p_prev = points[prev_idx]
            p_curr = points[i]
            p_next = points[next_idx]

            # Vector side lengths
            d_a = float(np.hypot(p_curr[0] - p_prev[0], p_curr[1] - p_prev[1]))
            d_b = float(np.hypot(p_next[0] - p_curr[0], p_next[1] - p_curr[1]))
            d_c = float(np.hypot(p_next[0] - p_prev[0], p_next[1] - p_prev[1]))

            # Signed triangle area
            area = 0.5 * ((p_curr[0] - p_prev[0]) * (p_next[1] - p_prev[1]) - (p_curr[1] - p_prev[1]) * (p_next[0] - p_prev[0]))

            denom = d_a * d_b * d_c
            if denom > 1e-8:
                curvatures[i] = 4.0 * area / denom
            else:
                curvatures[i] = 0.0

        return s, tangents, headings, curvatures

    @classmethod
    def optimize_minimum_curvature_path(
        cls,
        centerline: np.ndarray,
        width_right: float = 3.0,
        width_left: float = 3.0,
        safety_margin: float = 0.3,
        is_closed: bool = True,
    ) -> np.ndarray:
        """
        Solves convex quadratic programming problem to find optimal lateral
        deviation alpha along track normals that minimizes sum of squared path curvatures.
        P_opt[i] = C[i] + alpha[i] * n[i]
        """
        n_pts = len(centerline)
        _, _, headings, _ = cls.compute_path_geometry(centerline, is_closed=is_closed)

        # Normal unit vectors n = [-sin(theta), cos(theta)]
        normals = np.column_stack((-np.sin(headings), np.cos(headings)))

        # Bounds on lateral offset alpha: -width_right + margin <= alpha <= width_left - margin
        lb = np.full(n_pts, -width_right + safety_margin, dtype=np.float64)
        ub = np.full(n_pts, width_left - safety_margin, dtype=np.float64)

        # Second difference operator matrix D2 (approximates d^2 P / ds^2)
        # P''_i = P_{i-1} - 2 P_i + P_{i+1}
        # In terms of alpha:
        # P''_x = (C_x'' ) + alpha_{i-1} n_{x, i-1} - 2 alpha_i n_{x, i} + alpha_{i+1} n_{x, i+1}
        kx = np.zeros((n_pts, n_pts), dtype=np.float64)
        ky = np.zeros((n_pts, n_pts), dtype=np.float64)
        bx = np.zeros(n_pts, dtype=np.float64)
        by = np.zeros(n_pts, dtype=np.float64)

        for i in range(n_pts):
            prev_idx = (i - 1) % n_pts if is_closed else max(0, i - 1)
            next_idx = (i + 1) % n_pts if is_closed else min(n_pts - 1, i + 1)

            # Centerline second differences
            c_prev = centerline[prev_idx]
            c_curr = centerline[i]
            c_next = centerline[next_idx]

            bx[i] = -(c_prev[0] - 2.0 * c_curr[0] + c_next[0])
            by[i] = -(c_prev[1] - 2.0 * c_curr[1] + c_next[1])

            kx[i, prev_idx] += normals[prev_idx, 0]
            kx[i, i] -= 2.0 * normals[i, 0]
            kx[i, next_idx] += normals[next_idx, 0]

            ky[i, prev_idx] += normals[prev_idx, 1]
            ky[i, i] -= 2.0 * normals[i, 1]
            ky[i, next_idx] += normals[next_idx, 1]

        # Stack into single least-squares system: min ||K alpha - b||^2 s.t. lb <= alpha <= ub
        k_mat = np.vstack((kx, ky))
        b_vec = np.concatenate((bx, by))

        # Solve box-constrained linear least squares
        res = lsq_linear(k_mat, b_vec, bounds=(lb, ub), lsmr_tol="auto")
        alpha_opt = res.x

        # Reconstruct optimal path coordinates
        opt_path = centerline + alpha_opt[:, None] * normals
        return opt_path

    @classmethod
    def generate_speed_profile(
        cls,
        path: np.ndarray,
        limits: VehicleLimits | None = None,
        is_closed: bool = True,
        n_passes: int = 3,
    ) -> RacelineProfile:
        """
        Generates friction-ellipse limited speed profile using forward/backward
        integration passes across path curvature peaks.
        """
        limits = limits or VehicleLimits()
        n_pts = len(path)
        s, _, headings, curvatures = cls.compute_path_geometry(path, is_closed=is_closed)

        # Segment lengths ds
        ds = np.zeros(n_pts, dtype=np.float64)
        for i in range(n_pts):
            next_idx = (i + 1) % n_pts if is_closed else min(n_pts - 1, i + 1)
            ds[i] = float(np.hypot(path[next_idx, 0] - path[i, 0], path[next_idx, 1] - path[i, 1]))
            if ds[i] < 1e-4:
                ds[i] = 1e-4

        # 1. Apex / Cornering Speed Limit at each point
        # a_y = v^2 * |kappa| <= a_y,max => v_apex = sqrt(a_y,max / |kappa|)
        v_profile = np.full(n_pts, limits.max_velocity_mps, dtype=np.float64)
        for i in range(n_pts):
            k_abs = abs(curvatures[i])
            if k_abs > 1e-4:
                # Aerodynamic downforce increases available lateral grip at higher speeds
                # Iterative convergence for aero downforce: a_y,max = (mu*m*g + 0.5*rho*Cl*A*v^2)/m
                v_corner = math.sqrt(limits.max_lateral_accel_mps2 / k_abs)
                v_profile[i] = min(limits.max_velocity_mps, v_corner)

        # 2. Iterative Backward (Braking) & Forward (Acceleration) Passes
        max_decel_mag = abs(limits.max_decel_mps2)
        max_accel_mag = limits.max_accel_mps2

        for _ in range(n_passes):
            # Backward pass: braking limits into corners
            for step in range(n_pts):
                idx = (n_pts - 1 - step) if not is_closed else (n_pts * 2 - step) % n_pts
                next_idx = (idx + 1) % n_pts if is_closed else min(n_pts - 1, idx + 1)

                v_next = v_profile[next_idx]
                k_curr = abs(curvatures[idx])
                # Friction ellipse reduction: a_x,avail = a_x,max * sqrt(max(0, 1 - (v^2 * k / a_y,max)^2))
                lat_accel_ratio = (v_next**2 * k_curr) / limits.max_lateral_accel_mps2
                lat_factor = math.sqrt(max(0.05, 1.0 - min(0.95, lat_accel_ratio**2)))

                a_brake_avail = max_decel_mag * lat_factor
                v_max_brake = math.sqrt(v_next**2 + 2.0 * a_brake_avail * ds[idx])
                v_profile[idx] = min(v_profile[idx], v_max_brake)

            # Forward pass: traction acceleration limits out of corners
            for step in range(n_pts):
                idx = step if not is_closed else step % n_pts
                prev_idx = (idx - 1) % n_pts if is_closed else max(0, idx - 1)

                v_prev = v_profile[prev_idx]
                k_curr = abs(curvatures[idx])
                lat_accel_ratio = (v_prev**2 * k_curr) / limits.max_lateral_accel_mps2
                lat_factor = math.sqrt(max(0.05, 1.0 - min(0.95, lat_accel_ratio**2)))

                # Aero drag resistance reduction: a_net = a_accel - (0.5 * rho * Cd * A * v^2) / m
                drag_decel = (0.5 * limits.air_density * limits.drag_coeff * limits.frontal_area * v_prev**2) / limits.mass
                a_accel_net = max(0.5, (max_accel_mag * lat_factor) - drag_decel)

                v_max_accel = math.sqrt(v_prev**2 + 2.0 * a_accel_net * ds[prev_idx])
                v_profile[idx] = min(v_profile[idx], v_max_accel)

        # 3. Compute Accelerations and Lap Time
        ax = np.zeros(n_pts, dtype=np.float64)
        ay = np.zeros(n_pts, dtype=np.float64)
        total_time = 0.0

        for i in range(n_pts):
            next_idx = (i + 1) % n_pts if is_closed else min(n_pts - 1, i + 1)
            v_curr = v_profile[i]
            v_next = v_profile[next_idx]
            v_avg = 0.5 * (v_curr + v_next)

            ax[i] = (v_next**2 - v_curr**2) / (2.0 * ds[i]) if ds[i] > 1e-4 else 0.0
            ay[i] = (v_curr**2) * curvatures[i]
            total_time += ds[i] / max(0.5, v_avg)

        track_len = float(s[-1] + ds[-1]) if is_closed else float(s[-1])

        return RacelineProfile(
            points=path,
            s=s,
            tangent_headings=headings,
            curvatures=curvatures,
            velocities=v_profile,
            ax=ax,
            ay=ay,
            lap_time_s=float(total_time),
            track_length_m=track_len,
            max_velocity_mps=float(np.max(v_profile)),
            min_velocity_mps=float(np.min(v_profile)),
            avg_velocity_mps=float(np.mean(v_profile)),
        )
