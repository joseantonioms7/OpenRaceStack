"""
Adaptive Lookahead Pure Pursuit Path-Tracking Controller for OpenRaceStack.

Implements geometric pure pursuit path tracking with velocity-dependent adaptive
lookahead distance, path interpolation, actuator slew-rate limiting, and closed-loop
speed tracking along 2D racing trajectories and waypoints.
"""

import math
import time
from dataclasses import dataclass

import numpy as np

from .types import wrap_angle


@dataclass
class PurePursuitCommand:
    """Actuator command and diagnostic telemetry from Pure Pursuit tracking."""
    steering_angle_rad: float        # Front wheel steering angle (rad)
    steering_angle_deg: float        # Front wheel steering angle (deg)
    acceleration_mps2: float         # Longitudinal acceleration demand (m/s^2)
    lookahead_distance_m: float      # Dynamic lookahead distance L_d (m)
    target_point: tuple[float, float]# Intersected lookahead point (x, y)
    cross_track_error_m: float       # Lateral offset from path center (m)
    heading_error_rad: float         # Orientation error relative to track tangent (rad)
    target_velocity_mps: float       # Commanded velocity at lookahead point (m/s)
    is_saturated_steering: bool
    is_saturated_accel: bool
    solve_time_ms: float


@dataclass
class PurePursuitConfig:
    """Configuration parameters for Pure Pursuit path tracking."""
    wheelbase_m: float = 0.33                     # Vehicle wheelbase L (m)
    min_lookahead_m: float = 0.8                  # Minimum lookahead distance L_min (m)
    max_lookahead_m: float = 20.0                 # Maximum lookahead distance L_max (m)
    lookahead_gain_s: float = 0.35                # Velocity scaling gain k_dd (s): L_d = k_dd * v + L_min
    max_steering_rad: float = math.radians(24.0)  # Maximum steer angle limit (rad)
    max_steering_rate_radps: float = math.radians(120.0) # Steering slew rate limit (rad/s)
    kp_v: float = 2.0                             # Proportional velocity tracking gain
    max_accel_mps2: float = 4.0                   # Max traction acceleration
    max_decel_mps2: float = -7.0                  # Max braking deceleration


class PurePursuitController:
    """
    High-performance Adaptive Lookahead Pure Pursuit path tracking controller.
    """

    def __init__(self, config: PurePursuitConfig | None = None):
        self.config = config or PurePursuitConfig()
        self.prev_steering: float = 0.0
        self.prev_time: float | None = None

    def reset(self) -> None:
        """Resets controller internal filter memory and rate limiters."""
        self.prev_steering = 0.0
        self.prev_time = None

    def compute_lookahead_distance(self, speed_mps: float) -> float:
        """
        Calculates dynamic lookahead distance as a function of speed:
        L_d = clip(k_dd * speed + L_min, L_min, L_max)
        """
        raw_ld = self.config.lookahead_gain_s * abs(speed_mps) + self.config.min_lookahead_m
        return float(np.clip(raw_ld, self.config.min_lookahead_m, self.config.max_lookahead_m))

    def find_target_point(
        self,
        vehicle_x: float,
        vehicle_y: float,
        reference_path: np.ndarray,
        lookahead_dist: float,
    ) -> tuple[tuple[float, float], int, float, float]:
        """
        Finds the lookahead target point on the reference path.
        Returns: ((target_x, target_y), nearest_idx, cross_track_error, heading_error).
        """
        n_pts = len(reference_path)
        pos = np.array([vehicle_x, vehicle_y], dtype=np.float64)

        # 1. Locate nearest point on path
        deltas = reference_path[:, :2] - pos
        sq_dists = np.sum(deltas**2, axis=1)
        nearest_idx = int(np.argmin(sq_dists))

        # 2. Compute signed cross-track error e_y and path tangent heading
        p_near = reference_path[nearest_idx, :2]
        p_next = reference_path[(nearest_idx + 1) % n_pts, :2]
        segment_vec = p_next - p_near
        seg_len = float(np.linalg.norm(segment_vec))

        if seg_len > 1e-4:
            tangent = segment_vec / seg_len
            tangent_heading = math.atan2(tangent[1], tangent[0])
            normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
            d_pos = pos - p_near
            cte = float(np.dot(d_pos, normal))
        else:
            tangent_heading = 0.0
            cte = float(math.hypot(deltas[nearest_idx, 0], deltas[nearest_idx, 1]))

        # 3. Search forward along path for the lookahead horizon using track arc-length
        # This ensures the target is always L_d ahead along raceline regardless of lateral offset
        is_closed = float(np.linalg.norm(reference_path[0, :2] - reference_path[-1, :2])) < 3.0
        accum_dist = 0.0
        target_point = (float(p_near[0]), float(p_near[1]))
        found_target = False

        for i in range(n_pts):
            curr_idx = (nearest_idx + i) % n_pts if is_closed else min(nearest_idx + i, n_pts - 1)
            next_idx = (curr_idx + 1) % n_pts if is_closed else min(curr_idx + 1, n_pts - 1)
            if curr_idx == next_idx:
                target_point = (float(reference_path[curr_idx, 0]), float(reference_path[curr_idx, 1]))
                found_target = True
                break

            seg = reference_path[next_idx, :2] - reference_path[curr_idx, :2]
            seg_len = float(np.linalg.norm(seg))
            if accum_dist + seg_len >= lookahead_dist:
                remain = lookahead_dist - accum_dist
                ratio = float(np.clip(remain / seg_len, 0.0, 1.0)) if seg_len > 1e-5 else 0.0
                p_interp = reference_path[curr_idx, :2] + ratio * seg
                target_point = (float(p_interp[0]), float(p_interp[1]))
                found_target = True
                break
            accum_dist += seg_len

        if not found_target:
            target_point = (float(reference_path[-1, 0]), float(reference_path[-1, 1]))

        return target_point, nearest_idx, cte, tangent_heading

    def compute_control(
        self,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_theta: float,
        vehicle_v: float,
        reference_path: np.ndarray,
        reference_velocities: np.ndarray | None = None,
        dt: float = 0.05,
    ) -> PurePursuitCommand:
        """
        Computes steering and acceleration commands given vehicle state and reference trajectory.
        """
        t0 = time.perf_counter()

        # Dynamic lookahead distance
        ld = self.compute_lookahead_distance(vehicle_v)

        # Lookahead point and tracking errors
        target_pt, nearest_idx, cte, tangent_heading = self.find_target_point(
            vehicle_x, vehicle_y, reference_path, ld
        )
        heading_err = wrap_angle(vehicle_theta - tangent_heading)

        # Transform target point into body-fixed vehicle frame (x forward, y left)
        dx = target_pt[0] - vehicle_x
        dy = target_pt[1] - vehicle_y
        cos_th = math.cos(vehicle_theta)
        sin_th = math.sin(vehicle_theta)

        local_x = dx * cos_th + dy * sin_th
        local_y = -dx * sin_th + dy * cos_th

        # Actual distance to target point in body frame
        dist_sq = local_x**2 + local_y**2
        if dist_sq < 1e-4:
            raw_steer = 0.0
        else:
            # Curvature kappa = 2 * local_y / dist^2
            curvature = (2.0 * local_y) / dist_sq
            # Steering angle delta = arctan(kappa * L)
            raw_steer = math.atan(curvature * self.config.wheelbase_m)

        # Slew-rate limiting
        max_rate_delta = self.config.max_steering_rate_radps * dt
        delta_change = float(np.clip(raw_steer - self.prev_steering, -max_rate_delta, max_rate_delta))
        steer_rate_limited = self.prev_steering + delta_change

        # Saturation clamping
        steer_saturated = float(np.clip(
            steer_rate_limited,
            -self.config.max_steering_rad,
            self.config.max_steering_rad,
        ))
        is_sat_steer = bool(abs(steer_saturated) >= self.config.max_steering_rad - 1e-4)

        # Update controller state
        self.prev_steering = steer_saturated

        # Speed / Acceleration Tracking
        if reference_velocities is not None and len(reference_velocities) > nearest_idx:
            target_v = float(reference_velocities[nearest_idx])
        else:
            target_v = 15.0

        v_error = target_v - vehicle_v
        raw_accel = self.config.kp_v * v_error
        accel_saturated = float(np.clip(
            raw_accel,
            self.config.max_decel_mps2,
            self.config.max_accel_mps2,
        ))
        is_sat_accel = bool(
            accel_saturated >= self.config.max_accel_mps2 - 1e-4
            or accel_saturated <= self.config.max_decel_mps2 + 1e-4
        )

        solve_time_ms = (time.perf_counter() - t0) * 1000.0

        return PurePursuitCommand(
            steering_angle_rad=steer_saturated,
            steering_angle_deg=math.degrees(steer_saturated),
            acceleration_mps2=accel_saturated,
            lookahead_distance_m=ld,
            target_point=target_pt,
            cross_track_error_m=cte,
            heading_error_rad=heading_err,
            target_velocity_mps=target_v,
            is_saturated_steering=is_sat_steer,
            is_saturated_accel=is_sat_accel,
            solve_time_ms=solve_time_ms,
        )
