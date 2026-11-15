"""
Stanley (Stanford) Lateral and Longitudinal Path-Tracking Controller for OpenRaceStack.

Implements the non-linear Stanley front-axle trajectory tracking control law:
    delta(t) = wrap_angle(theta_e(t) + arctan2(k * e_fa(t), v(t) + k_soft))
with steering slew-rate limiting, dynamic front-axle projection, and speed tracking.
"""

import math
from dataclasses import dataclass

import numpy as np

from .types import wrap_angle


@dataclass
class StanleyConfig:
    """Configuration gains and actuator limits for Stanley controller."""
    k_e: float = 1.2                       # Cross-track error gain (1/s)
    k_soft: float = 1.0                    # Softening constant for low-speed regularization (m/s)
    k_yaw: float = 1.0                     # Heading error multiplier
    k_v: float = 2.0                       # Velocity P-controller gain (1/s)
    max_steering_angle_rad: float = math.radians(24.0)   # Steering saturation (+/- 24 deg)
    max_steering_rate_radps: float = math.radians(120.0) # Slew rate limit (rad/s)
    max_accel_mps2: float = 6.0            # Maximum acceleration (m/s^2)
    max_decel_mps2: float = -8.0           # Maximum deceleration (m/s^2)


@dataclass
class StanleyCommand:
    """Actuation demand and diagnostic errors from Stanley controller."""
    steering_angle_rad: float
    acceleration_mps2: float
    cross_track_error_m: float
    heading_error_rad: float
    front_axle_x: float
    front_axle_y: float
    is_saturated: bool = False


class StanleyController:
    """
    Non-linear Stanley Tracking Controller evaluating tracking errors at the front axle.
    """

    def __init__(self, config: StanleyConfig | None = None, wheelbase_m: float = 0.33):
        self.config = config or StanleyConfig()
        self.wheelbase = float(wheelbase_m)
        self._prev_steer = 0.0

    def compute_control(
        self,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_theta: float,
        vehicle_v: float,
        reference_path: np.ndarray,
        reference_headings: np.ndarray | None = None,
        reference_velocities: np.ndarray | None = None,
        dt: float = 0.05,
    ) -> StanleyCommand:
        """
        Computes steering and longitudinal commands to follow reference path.

        Args:
            vehicle_x, vehicle_y: Global position of vehicle rear axle / CG (m).
            vehicle_theta: Inertial heading yaw angle (rad).
            vehicle_v: Longitudinal velocity (m/s).
            reference_path: Array of shape (N, 2) defining track waypoints [x, y].
            reference_headings: Optional array of shape (N,) reference headings.
            reference_velocities: Optional array of shape (N,) target speeds.
            dt: Control sampling step (s).

        Returns:
            StanleyCommand containing actuator commands and tracking errors.
        """
        ref = np.asarray(reference_path, dtype=np.float64)
        if len(ref) < 2:
            raise ValueError("Reference path must contain at least 2 waypoints.")

        # 1. Compute front axle position
        fx = vehicle_x + self.wheelbase * math.cos(vehicle_theta)
        fy = vehicle_y + self.wheelbase * math.sin(vehicle_theta)

        # 2. Find closest point on path to front axle
        dists_sq = (ref[:, 0] - fx) ** 2 + (ref[:, 1] - fy) ** 2
        idx = int(np.argmin(dists_sq))
        px, py = ref[idx, 0], ref[idx, 1]

        # 3. Reference path tangent and heading
        if reference_headings is not None and len(reference_headings) == len(ref):
            ref_heading = float(reference_headings[idx])
        else:
            if idx < len(ref) - 1:
                dx = ref[idx + 1, 0] - px
                dy = ref[idx + 1, 1] - py
            else:
                dx = px - ref[idx - 1, 0]
                dy = py - ref[idx - 1, 1]
            ref_heading = math.atan2(dy, dx)

        # 4. Heading error theta_e = path_heading - vehicle_heading
        theta_e = wrap_angle(ref_heading - vehicle_theta)

        # 5. Signed cross-track error e_fa:
        # Vector from front axle to closest path waypoint: [px - fx, py - fy]
        # Left-pointing track normal: [-sin(ref_heading), cos(ref_heading)]
        # When vehicle is to the right of path, e_fa > 0 commanding positive steer (left)
        dx_to_path = px - fx
        dy_to_path = py - fy
        e_fa = -dx_to_path * math.sin(ref_heading) + dy_to_path * math.cos(ref_heading)

        # 6. Non-linear Stanley steering law
        cross_track_term = math.atan2(self.config.k_e * e_fa, max(0.01, vehicle_v + self.config.k_soft))
        steer_raw = self.config.k_yaw * theta_e + cross_track_term

        # 7. Apply steering slew rate limit
        max_d_steer = self.config.max_steering_rate_radps * dt
        d_steer = float(np.clip(steer_raw - self._prev_steer, -max_d_steer, max_d_steer))
        steer_cmd = self._prev_steer + d_steer

        # 8. Apply steering angle saturation
        steer_sat = float(np.clip(
            steer_cmd,
            -self.config.max_steering_angle_rad,
            self.config.max_steering_angle_rad,
        ))
        is_saturated = abs(steer_sat - steer_raw) > 1e-4
        self._prev_steer = steer_sat

        # 9. Longitudinal speed control
        target_v = float(reference_velocities[idx]) if reference_velocities is not None else vehicle_v
        accel_raw = self.config.k_v * (target_v - vehicle_v)
        accel_cmd = float(np.clip(accel_raw, self.config.max_decel_mps2, self.config.max_accel_mps2))

        return StanleyCommand(
            steering_angle_rad=steer_sat,
            acceleration_mps2=accel_cmd,
            cross_track_error_m=float(e_fa),
            heading_error_rad=float(theta_e),
            front_axle_x=float(fx),
            front_axle_y=float(fy),
            is_saturated=is_saturated,
        )
