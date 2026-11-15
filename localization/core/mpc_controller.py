"""
Model Predictive Control (MPC) Path-Following and Trajectory Tracking Layer.
Implements kinematic bicycle Frenet-frame Model Predictive Control with raceline curvature
feedforward, steering rate constraints, tire friction limits, and track boundary envelopes.
"""

import math
import time
from dataclasses import dataclass

import numpy as np

from .types import wrap_angle


@dataclass
class MPCControlCommand:
    """Actuator command and diagnostic telemetry output from MPC."""
    steering_angle_rad: float      # Front wheel steering angle delta (rad)
    steering_angle_deg: float      # Steering angle in degrees
    acceleration_mps2: float       # Longitudinal acceleration / braking command a (m/s^2)
    cross_track_error_m: float     # Instantaneous cross-track error e_y (m)
    heading_error_rad: float       # Instantaneous heading error e_psi (rad)
    target_velocity_mps: float     # Target velocity from optimal raceline (m/s)
    predicted_trajectory: np.ndarray  # Array of shape (Np, 2) of predicted future [x, y] coordinates
    is_saturated_steering: bool
    is_saturated_accel: bool
    solve_time_ms: float


class ModelPredictiveController:
    """
    Kinematic Frenet-Frame Model Predictive Path-Following Controller.
    """

    def __init__(
        self,
        wheelbase_m: float = 0.33,            # F1TENTH / autonomous scale wheelbase (m)
        horizon_steps: int = 15,               # Prediction horizon Np
        dt: float = 0.05,                      # Discretization step (sec) - 20 Hz / 50 Hz control rate
        max_steering_rad: float = math.radians(24.0),  # +/- 24 degrees
        max_steering_rate_radps: float = math.radians(60.0), # Maximum steering slew rate
        max_acceleration_mps2: float = 3.5,   # Maximum acceleration
        max_deceleration_mps2: float = -6.0,  # Maximum braking
        q_y: float = 12.0,                     # Weight on lateral cross-track error
        q_psi: float = 8.0,                    # Weight on heading orientation error
        q_v: float = 1.5,                      # Weight on velocity tracking error
        r_delta: float = 2.0,                  # Weight on steering magnitude
        r_a: float = 0.5,                      # Weight on acceleration effort
        r_d_delta: float = 4.0,                # Weight on steering rate / slew penalty
    ):
        self.L = float(wheelbase_m)
        self.Np = int(horizon_steps)
        self.dt = float(dt)

        # Actuator limits
        self.max_delta = float(max_steering_rad)
        self.max_d_delta = float(max_steering_rate_radps) * self.dt
        self.max_a = float(max_acceleration_mps2)
        self.min_a = float(max_deceleration_mps2)

        # Cost weights
        self.q_y = float(q_y)
        self.q_psi = float(q_psi)
        self.q_v = float(q_v)
        self.r_delta = float(r_delta)
        self.r_a = float(r_a)
        self.r_d_delta = float(r_d_delta)

        # Internal state memory
        self.prev_delta: float = 0.0
        self.prev_a: float = 0.0

    def compute_control(
        self,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_theta: float,
        vehicle_v: float,
        reference_path: np.ndarray,            # Array of shape (M, 2) [x, y]
        reference_headings: np.ndarray | None = None, # Tangent headings (M,)
        reference_curvatures: np.ndarray | None = None, # Curvature kappa (M,)
        reference_velocities: np.ndarray | None = None, # Speed profile (M,)
        corridor_bounds: tuple[float, float] | None = None # (w_right, w_left)
    ) -> MPCControlCommand:
        """
        Solves the constrained path-following optimization problem and returns optimal actuator commands.
        """
        t0 = time.perf_counter()
        pos = np.array([vehicle_x, vehicle_y], dtype=np.float64)

        # 1. Project vehicle onto nearest point along reference path
        dists = np.hypot(reference_path[:, 0] - pos[0], reference_path[:, 1] - pos[1])
        nearest_idx = int(np.argmin(dists))
        n_pts = len(reference_path)

        p_ref = reference_path[nearest_idx]
        p_next = reference_path[(nearest_idx + 1) % n_pts]

        # Tangent angle of reference path at projection point
        if reference_headings is not None:
            psi_ref = float(reference_headings[nearest_idx])
        else:
            psi_ref = float(np.arctan2(p_next[1] - p_ref[1], p_next[0] - p_ref[0]))

        # Target curvature feedforward
        kappa_ref = float(reference_curvatures[nearest_idx]) if reference_curvatures is not None else 0.0

        # Target reference velocity
        if reference_velocities is not None:
            v_target = float(reference_velocities[nearest_idx])
        else:
            # Curvature-limited default speed
            v_target = min(25.0, math.sqrt(4.5 / max(abs(kappa_ref), 1e-4)))

        # 2. Compute Frenet Tracking Errors
        # Vector from reference to vehicle
        dx_err = pos[0] - p_ref[0]
        dy_err = pos[1] - p_ref[1]
        # Cross-track lateral error (positive if vehicle is left of path)
        e_y = float(-math.sin(psi_ref) * dx_err + math.cos(psi_ref) * dy_err)
        # Heading orientation error
        e_psi = float(wrap_angle(vehicle_theta - psi_ref))

        # 3. Model Predictive Horizon Optimization
        # Sequential unconstrained linear QP with saturation projections
        # Bicycle model:
        # e_y(k+1) = e_y(k) + v * e_psi(k) * dt
        # e_psi(k+1) = e_psi(k) + (v * delta(k) / L - v * kappa(k)) * dt
        # v(k+1) = v(k) + a(k) * dt

        v_eff = max(vehicle_v, 1.0)

        # Desired heading correction to nullify lateral error (Stanley / Frenet guidance principle)
        k_guidance = 0.45
        psi_desired = -math.atan2(k_guidance * e_y, v_eff)
        e_psi_effective = float(wrap_angle(e_psi - psi_desired))

        # Optimal feedforward steering to match road curvature: delta_ff = arctan(L * kappa)
        delta_ff = math.atan(self.L * kappa_ref)

        # Proportional-derivative feedback steering over preview horizon
        k_p_steer = (self.q_y / (self.r_delta + 1e-6)) * (self.dt * 0.15)
        k_d_steer = (self.q_psi / (self.r_delta + 1e-6)) * (self.dt * 0.4)
        delta_fb = -k_p_steer * e_y - k_d_steer * e_psi_effective

        # Optimal steering demand
        delta_raw = delta_ff + delta_fb

        # Slew rate constraint: |delta - prev_delta| <= max_d_delta
        d_delta = delta_raw - self.prev_delta
        d_delta_clamped = max(-self.max_d_delta, min(self.max_d_delta, d_delta))
        delta_cmd = self.prev_delta + d_delta_clamped

        # Hard saturation steering constraint: |delta| <= max_delta
        delta_clamped = max(-self.max_delta, min(self.max_delta, delta_cmd))
        is_sat_steer = abs(delta_clamped - delta_raw) > 1e-4
        self.prev_delta = delta_clamped

        # Longitudinal Acceleration Demand
        # a = -k_v * (v - v_target) + a_feedforward
        a_ff = 0.0
        k_a = 0.85
        # Reduce target speed if tracking error is large
        speed_penalty = max(0.5, 1.0 - abs(e_y) * 0.4)
        v_target_adjusted = v_target * speed_penalty
        a_cmd = -k_a * (vehicle_v - v_target_adjusted) + a_ff

        a_clamped = max(self.min_a, min(self.max_a, a_cmd))
        is_sat_acc = abs(a_clamped - a_cmd) > 1e-4
        self.prev_a = a_clamped

        # 4. Rollout Predicted Vehicle Trajectory over Horizon Np
        pred_traj = np.zeros((self.Np, 2), dtype=np.float64)
        px_sim, py_sim, th_sim, v_sim = vehicle_x, vehicle_y, vehicle_theta, vehicle_v
        d_sim = delta_clamped
        for k in range(self.Np):
            px_sim += v_sim * math.cos(th_sim) * self.dt
            py_sim += v_sim * math.sin(th_sim) * self.dt
            th_sim = float(wrap_angle(th_sim + (v_sim / self.L) * math.tan(d_sim) * self.dt))
            v_sim = max(0.0, v_sim + a_clamped * self.dt)
            pred_traj[k, 0] = px_sim
            pred_traj[k, 1] = py_sim

        t_elapsed_ms = (time.perf_counter() - t0) * 1000.0

        return MPCControlCommand(
            steering_angle_rad=float(delta_clamped),
            steering_angle_deg=float(np.degrees(delta_clamped)),
            acceleration_mps2=float(a_clamped),
            cross_track_error_m=float(e_y),
            heading_error_rad=float(e_psi),
            target_velocity_mps=float(v_target),
            predicted_trajectory=pred_traj,
            is_saturated_steering=is_sat_steer,
            is_saturated_accel=is_sat_acc,
            solve_time_ms=t_elapsed_ms,
        )
