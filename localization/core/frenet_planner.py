"""
Frenet Lattice Trajectory Planner and Dynamic Obstacle Avoidance Layer for OpenRaceStack.

Generates optimal spatio-temporal polynomial trajectories in the Frenet frame (Werling et al. 2010),
sampling candidate lateral maneuvers (quintic polynomials) and longitudinal velocity profiles
(quartic polynomials) for dynamic overtaking, obstacle avoidance, and track boundary adherence.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .types import wrap_angle


@dataclass
class Obstacle:
    """Dynamic or static track obstacle representation."""
    x: float                     # Current Cartesian X position (m)
    y: float                     # Current Cartesian Y position (m)
    vx: float = 0.0              # Longitudinal velocity component (m/s)
    vy: float = 0.0              # Lateral velocity component (m/s)
    radius: float = 0.8          # Collision bounding circle radius (m)
    obstacle_id: str = "obs_0"   # Identifier

    def predict_position(self, t: float) -> tuple[float, float]:
        """Linearly projects obstacle position at future time horizon t (seconds)."""
        return (self.x + self.vx * t, self.y + self.vy * t)


@dataclass
class FrenetState:
    """Vehicle state decomposed in the Frenet coordinate frame."""
    s: float = 0.0               # Arc-length progress along reference raceline (m)
    s_d: float = 0.0             # Longitudinal speed ds/dt (m/s)
    s_dd: float = 0.0            # Longitudinal acceleration d2s/dt2 (m/s^2)
    d: float = 0.0               # Lateral displacement perpendicular to raceline (m)
    d_d: float = 0.0             # Lateral velocity dd/dt (m/s)
    d_dd: float = 0.0            # Lateral acceleration d2d/dt2 (m/s^2)


class QuinticPolynomial:
    """
    1D Quintic Polynomial trajectory: x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5.
    Used for lateral trajectory planning d(t) with specified boundary conditions.
    """

    def __init__(
        self,
        x0: float,
        v0: float,
        a0: float,
        x1: float,
        v1: float,
        a1: float,
        t_horizon: float,
    ):
        self.t_horizon = max(0.1, float(t_horizon))
        self.a0 = float(x0)
        self.a1 = float(v0)
        self.a2 = 0.5 * float(a0)

        t = self.t_horizon
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t

        # Boundary conditions at t = T
        # [T^3,   T^4,   T^5  ] [a3]   [x1 - (a0 + a1*T + a2*T^2)]
        # [3*T^2, 4*T^3, 5*T^4] [a4] = [v1 - (a1 + 2*a2*T)       ]
        # [6*T,   12*T^2,20*T^3] [a5]   [a1 - 2*a2                ]
        b1 = x1 - (self.a0 + self.a1 * t + self.a2 * t2)
        b2 = v1 - (self.a1 + 2.0 * self.a2 * t)
        b3 = a1 - 2.0 * self.a2

        # Analytical closed-form matrix inverse
        inv_2t5 = 1.0 / (2.0 * t5)
        self.a3 = inv_2t5 * (20.0 * t2 * b1 - 8.0 * t3 * b2 + t4 * b3)
        self.a4 = inv_2t5 * (-30.0 * t * b1 + 14.0 * t2 * b2 - 2.0 * t3 * b3)
        self.a5 = inv_2t5 * (12.0 * b1 - 6.0 * t * b2 + t2 * b3)

    def calc_point(self, t: float) -> float:
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

    def calc_first_derivative(self, t: float) -> float:
        return self.a1 + 2.0 * self.a2 * t + 3.0 * self.a3 * t**2 + 4.0 * self.a4 * t**3 + 5.0 * self.a5 * t**4

    def calc_second_derivative(self, t: float) -> float:
        return 2.0 * self.a2 + 6.0 * self.a3 * t + 12.0 * self.a4 * t**2 + 20.0 * self.a5 * t**3

    def calc_third_derivative(self, t: float) -> float:
        return 6.0 * self.a3 + 24.0 * self.a4 * t + 60.0 * self.a5 * t**2

    def calc_integrated_squared_jerk(self) -> float:
        """Exact closed-form integral of squared jerk: int_0^T (d3x/dt3)^2 dt."""
        t = self.t_horizon
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        a3, a4, a5 = self.a3, self.a4, self.a5
        return (
            36.0 * a3**2 * t
            + 144.0 * a3 * a4 * t2
            + 240.0 * a3 * a5 * t3
            + 192.0 * a4**2 * t3
            + 720.0 * a4 * a5 * t4
            + 720.0 * a5**2 * t5
        )


class QuarticPolynomial:
    """
    1D Quartic Polynomial trajectory: s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4.
    Used for longitudinal velocity-keeping mode (specifying target velocity v1 and accel 0).
    """

    def __init__(
        self,
        x0: float,
        v0: float,
        a0: float,
        v1: float,
        a1: float,
        t_horizon: float,
    ):
        self.t_horizon = max(0.1, float(t_horizon))
        self.a0 = float(x0)
        self.a1 = float(v0)
        self.a2 = 0.5 * float(a0)

        t = self.t_horizon
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t

        # Boundary conditions at t = T
        # [3*T^2, 4*T^3 ] [a3] = [v1 - (a1 + 2*a2*T)]
        # [6*T,   12*T^2] [a4]   [a1 - 2*a2         ]
        b1 = v1 - (self.a1 + 2.0 * self.a2 * t)
        b2 = a1 - 2.0 * self.a2

        # Analytical inverse: det = 36*T^4 - 24*T^4 = 12*T^4
        inv_det = 1.0 / (12.0 * t4)
        self.a3 = inv_det * (12.0 * t2 * b1 - 4.0 * t3 * b2)
        self.a4 = inv_det * (-6.0 * t * b1 + 3.0 * t2 * b2)

    def calc_point(self, t: float) -> float:
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4

    def calc_first_derivative(self, t: float) -> float:
        return self.a1 + 2.0 * self.a2 * t + 3.0 * self.a3 * t**2 + 4.0 * self.a4 * t**3

    def calc_second_derivative(self, t: float) -> float:
        return 2.0 * self.a2 + 6.0 * self.a3 * t + 12.0 * self.a4 * t**2

    def calc_third_derivative(self, t: float) -> float:
        return 6.0 * self.a3 + 24.0 * self.a4 * t

    def calc_integrated_squared_jerk(self) -> float:
        """Exact closed-form integral of squared jerk: int_0^T (d3s/dt3)^2 dt."""
        t = self.t_horizon
        t2 = t * t
        t3 = t2 * t
        a3, a4 = self.a3, self.a4
        return 36.0 * a3**2 * t + 144.0 * a3 * a4 * t2 + 192.0 * a4**2 * t3


@dataclass
class FrenetTrajectory:
    """Complete candidate or optimal spatio-temporal trajectory in Cartesian and Frenet frames."""
    t: np.ndarray = field(default_factory=lambda: np.array([]))
    d: np.ndarray = field(default_factory=lambda: np.array([]))
    d_d: np.ndarray = field(default_factory=lambda: np.array([]))
    d_dd: np.ndarray = field(default_factory=lambda: np.array([]))
    s: np.ndarray = field(default_factory=lambda: np.array([]))
    s_d: np.ndarray = field(default_factory=lambda: np.array([]))
    s_dd: np.ndarray = field(default_factory=lambda: np.array([]))
    x: np.ndarray = field(default_factory=lambda: np.array([]))
    y: np.ndarray = field(default_factory=lambda: np.array([]))
    yaw: np.ndarray = field(default_factory=lambda: np.array([]))
    v: np.ndarray = field(default_factory=lambda: np.array([]))
    curvature: np.ndarray = field(default_factory=lambda: np.array([]))
    cost: float = float("inf")
    is_valid: bool = False
    invalidation_reason: str = ""


@dataclass
class FrenetPlannerConfig:
    """Tuning weights and physical constraints for the Frenet Lattice Planner."""
    max_speed_mps: float = 35.0          # Peak velocity limit (m/s)
    max_accel_mps2: float = 6.0          # Max traction acceleration (m/s^2)
    max_decel_mps2: float = -9.0         # Max braking deceleration (m/s^2)
    max_lateral_accel_mps2: float = 11.0 # Max lateral tire grip (m/s^2)
    max_curvature_1pm: float = 0.50      # Maximum path curvature (1/m)
    road_width_left_m: float = 3.5       # Left track corridor boundary (m)
    road_width_right_m: float = 3.5      # Right track corridor boundary (m)
    vehicle_radius_m: float = 0.75       # Vehicle collision envelope radius (m)
    safety_margin_m: float = 0.35        # Additional clearance margin around obstacles (m)
    dt_eval: float = 0.10                # Temporal discretization for trajectory rollout (s)
    horizon_min_s: float = 1.5           # Minimum planning horizon (s)
    horizon_max_s: float = 3.0           # Maximum planning horizon (s)
    horizon_step_s: float = 0.5          # Horizon sampling step (s)
    d_sample_step_m: float = 0.6         # Lateral target sampling step (m)
    v_sample_step_mps: float = 2.0       # Longitudinal velocity target sampling step (m/s)
    n_v_samples: int = 1                 # Number of velocity variations around target (+/-)
    k_jerk_d: float = 1.0                # Lateral jerk cost weight
    k_jerk_s: float = 1.0                # Longitudinal jerk cost weight
    k_time: float = 2.0                  # Planning horizon duration weight
    k_target_d: float = 2.5              # Penalty for lateral offset from optimal raceline
    k_target_v: float = 3.0              # Penalty for velocity error from target


class FrenetLatticePlanner:
    """
    Spatio-temporal Frenet Lattice Trajectory Planner for dynamic obstacle avoidance and overtaking.
    """

    def __init__(self, config: FrenetPlannerConfig | None = None):
        self.config = config or FrenetPlannerConfig()

    @staticmethod
    def cartesian_to_frenet(
        x: float,
        y: float,
        yaw: float,
        v: float,
        reference_path: np.ndarray,
    ) -> FrenetState:
        """
        Projects Cartesian vehicle state onto reference raceline and calculates
        Frenet coordinates (s, s_d, s_dd, d, d_d, d_dd).
        """
        n_pts = len(reference_path)
        pos = np.array([x, y], dtype=np.float64)

        # 1. Nearest waypoint
        deltas = reference_path[:, :2] - pos
        sq_dists = np.sum(deltas**2, axis=1)
        nearest_idx = int(np.argmin(sq_dists))

        # Arc length computation along path
        p_near = reference_path[nearest_idx, :2]
        p_next = reference_path[(nearest_idx + 1) % n_pts, :2]
        tangent = p_next - p_near
        seg_len = float(np.linalg.norm(tangent))

        if seg_len > 1e-4:
            tangent_unit = tangent / seg_len
            theta_ref = math.atan2(tangent_unit[1], tangent_unit[0])
            normal_unit = np.array([-tangent_unit[1], tangent_unit[0]], dtype=np.float64)
            d_pos = pos - p_near
            d = float(np.dot(d_pos, normal_unit))
        else:
            theta_ref = 0.0
            d = float(math.hypot(deltas[nearest_idx, 0], deltas[nearest_idx, 1]))

        # Approximate arc length s
        # Sum segment lengths up to nearest_idx
        diffs = np.diff(reference_path[:, :2], axis=0)
        seg_lens = np.hypot(diffs[:, 0], diffs[:, 1])
        s = float(np.sum(seg_lens[:nearest_idx])) if nearest_idx > 0 else 0.0

        # Orientation error
        delta_yaw = wrap_angle(yaw - theta_ref)
        s_d = max(0.0, v * math.cos(delta_yaw))
        d_d = v * math.sin(delta_yaw)

        return FrenetState(s=s, s_d=s_d, s_dd=0.0, d=d, d_d=d_d, d_dd=0.0)

    def generate_candidate_trajectories(
        self,
        current_state: FrenetState,
        target_velocity: float,
    ) -> list[tuple[QuinticPolynomial, QuarticPolynomial, float]]:
        """
        Samples candidate lateral quintic polynomials and longitudinal quartic polynomials
        across temporal horizons T and target lateral offsets d.
        """
        candidates: list[tuple[QuinticPolynomial, QuarticPolynomial, float]] = []

        cfg = self.config
        horizons = np.arange(cfg.horizon_min_s, cfg.horizon_max_s + 1e-3, cfg.horizon_step_s)

        # Sample lateral target positions d_target within road boundaries
        max_d_left = cfg.road_width_left_m - cfg.safety_margin_m
        max_d_right = -cfg.road_width_right_m + cfg.safety_margin_m
        d_targets = np.arange(max_d_right, max_d_left + 1e-3, cfg.d_sample_step_m)
        # Always include d=0 (optimal raceline)
        if not any(abs(d) < 1e-4 for d in d_targets):
            d_targets = np.sort(np.append(d_targets, 0.0))

        # Velocity variations
        v_variations = [target_velocity]
        for step in range(1, cfg.n_v_samples + 1):
            v_variations.append(max(1.0, target_velocity - step * cfg.v_sample_step_mps))
            v_variations.append(min(cfg.max_speed_mps, target_velocity + step * cfg.v_sample_step_mps))

        for t_horiz in horizons:
            for d_target in d_targets:
                # Lateral quintic polynomial: from (d0, d_d0, d_dd0) to (d_target, 0, 0)
                lat_poly = QuinticPolynomial(
                    x0=current_state.d,
                    v0=current_state.d_d,
                    a0=current_state.d_dd,
                    x1=d_target,
                    v1=0.0,
                    a1=0.0,
                    t_horizon=t_horiz,
                )

                for v_target in v_variations:
                    # Longitudinal quartic polynomial: from (s0, s_d0, s_dd0) to (v_target, 0)
                    lon_poly = QuarticPolynomial(
                        x0=current_state.s,
                        v0=current_state.s_d,
                        a0=current_state.s_dd,
                        v1=v_target,
                        a1=0.0,
                        t_horizon=t_horiz,
                    )
                    candidates.append((lat_poly, lon_poly, float(t_horiz)))

        return candidates

    def evaluate_and_convert_trajectory(
        self,
        lat_poly: QuinticPolynomial,
        lon_poly: QuarticPolynomial,
        t_horiz: float,
        reference_path: np.ndarray,
        target_velocity: float,
        obstacles: list[Obstacle] | None = None,
    ) -> FrenetTrajectory:
        """
        Converts 1D Frenet polynomials into a full Cartesian FrenetTrajectory,
        evaluates safety constraints, checks obstacle collisions, and computes cost.
        """
        cfg = self.config
        dt = cfg.dt_eval
        time_steps = np.arange(0.0, t_horiz + 1e-4, dt)
        n_steps = len(time_steps)

        # 1D profiles
        d = np.array([lat_poly.calc_point(t) for t in time_steps])
        d_d = np.array([lat_poly.calc_first_derivative(t) for t in time_steps])
        d_dd = np.array([lat_poly.calc_second_derivative(t) for t in time_steps])

        s = np.array([lon_poly.calc_point(t) for t in time_steps])
        s_d = np.array([lon_poly.calc_first_derivative(t) for t in time_steps])
        s_dd = np.array([lon_poly.calc_second_derivative(t) for t in time_steps])

        # Cost evaluation
        j_d = lat_poly.calc_integrated_squared_jerk()
        j_s = lon_poly.calc_integrated_squared_jerk()
        cost = (
            cfg.k_jerk_d * j_d
            + cfg.k_jerk_s * j_s
            + cfg.k_time * t_horiz
            + cfg.k_target_d * (d[-1] ** 2)
            + cfg.k_target_v * ((s_d[-1] - target_velocity) ** 2)
        )

        # Kinematic & Boundary Validation
        if np.any(s_d > cfg.max_speed_mps):
            return FrenetTrajectory(cost=cost, is_valid=False, invalidation_reason="Speed limit exceeded")
        if np.any(s_dd > cfg.max_accel_mps2) or np.any(s_dd < cfg.max_decel_mps2):
            return FrenetTrajectory(cost=cost, is_valid=False, invalidation_reason="Longitudinal acceleration limit exceeded")
        if np.any(d > cfg.road_width_left_m - cfg.safety_margin_m) or np.any(d < -cfg.road_width_right_m + cfg.safety_margin_m):
            return FrenetTrajectory(cost=cost, is_valid=False, invalidation_reason="Track boundary infringement")

        # Convert to Cartesian Frame along reference path
        n_ref = len(reference_path)
        # Cumulative distances along reference path
        diffs = np.diff(reference_path[:, :2], axis=0)
        seg_lens = np.hypot(diffs[:, 0], diffs[:, 1])
        s_ref = np.zeros(n_ref, dtype=np.float64)
        s_ref[1:] = np.cumsum(seg_lens)
        total_path_len = s_ref[-1]

        x_coords = np.zeros(n_steps, dtype=np.float64)
        y_coords = np.zeros(n_steps, dtype=np.float64)
        yaw_coords = np.zeros(n_steps, dtype=np.float64)
        v_coords = np.zeros(n_steps, dtype=np.float64)
        curv_coords = np.zeros(n_steps, dtype=np.float64)

        for i, (_t_val, s_val, d_val, sd_val, dd_val) in enumerate(zip(time_steps, s, d, s_d, d_d, strict=False)):
            # Modulo for closed loops or clamp
            s_mod = s_val % total_path_len if total_path_len > 1e-3 else s_val
            ref_idx = int(np.searchsorted(s_ref, s_mod, side="right") - 1)
            ref_idx = max(0, min(n_ref - 2, ref_idx))

            p0 = reference_path[ref_idx, :2]
            p1 = reference_path[ref_idx + 1, :2]
            seg_vec = p1 - p0
            seg_len = float(np.linalg.norm(seg_vec))
            seg_u = seg_vec / seg_len if seg_len > 1e-5 else np.array([1.0, 0.0])

            theta_ref = math.atan2(seg_u[1], seg_u[0])
            normal_u = np.array([-seg_u[1], seg_u[0]])

            # Interpolate reference point
            ds = s_mod - s_ref[ref_idx]
            p_ref = p0 + seg_u * ds

            # Global Cartesian point: P = P_ref + d * normal
            p_cart = p_ref + normal_u * d_val
            x_coords[i] = p_cart[0]
            y_coords[i] = p_cart[1]

            # Orientation
            yaw_coords[i] = wrap_angle(theta_ref + math.atan2(dd_val, max(0.1, sd_val)))
            v_coords[i] = math.hypot(sd_val, dd_val)

        # Dynamic curvature and lateral acceleration check
        # dx, dy derivatives
        dx = np.gradient(x_coords, dt)
        dy = np.gradient(y_coords, dt)
        ddx = np.gradient(dx, dt)
        ddy = np.gradient(dy, dt)
        denom = (dx**2 + dy**2) ** 1.5
        curv_coords = np.where(denom > 1e-4, np.abs(dx * ddy - dy * ddx) / denom, 0.0)

        lat_accel = v_coords**2 * curv_coords
        if np.any(curv_coords > cfg.max_curvature_1pm):
            return FrenetTrajectory(cost=cost, is_valid=False, invalidation_reason="Curvature exceeds vehicle limits")
        if np.any(lat_accel > cfg.max_lateral_accel_mps2):
            return FrenetTrajectory(cost=cost, is_valid=False, invalidation_reason="Lateral acceleration exceeds tire grip")

        # Collision Checking against dynamic/static obstacles
        if obstacles:
            for obs in obstacles:
                for t_val, x_val, y_val in zip(time_steps, x_coords, y_coords, strict=False):
                    obs_pos = obs.predict_position(t_val)
                    dist = math.hypot(x_val - obs_pos[0], y_val - obs_pos[1])
                    min_allowed_dist = cfg.vehicle_radius_m + obs.radius + cfg.safety_margin_m
                    if dist < min_allowed_dist:
                        return FrenetTrajectory(
                            cost=cost,
                            is_valid=False,
                            invalidation_reason=f"Collision predicted with {obs.obstacle_id}",
                        )

        return FrenetTrajectory(
            t=time_steps,
            d=d,
            d_d=d_d,
            d_dd=d_dd,
            s=s,
            s_d=s_d,
            s_dd=s_dd,
            x=x_coords,
            y=y_coords,
            yaw=yaw_coords,
            v=v_coords,
            curvature=curv_coords,
            cost=cost,
            is_valid=True,
        )

    def plan(
        self,
        current_state: FrenetState,
        reference_path: np.ndarray,
        obstacles: list[Obstacle] | None = None,
        target_velocity: float = 20.0,
    ) -> FrenetTrajectory:
        """
        Executes Frenet lattice trajectory planning:
        1. Samples spatio-temporal polynomial candidates.
        2. Validates boundaries, dynamics, and collision horizons.
        3. Returns optimal lowest-cost collision-free trajectory.
        """
        candidates = self.generate_candidate_trajectories(current_state, target_velocity)

        best_traj: FrenetTrajectory | None = None
        min_cost = float("inf")

        for lat_poly, lon_poly, t_horiz in candidates:
            traj = self.evaluate_and_convert_trajectory(
                lat_poly=lat_poly,
                lon_poly=lon_poly,
                t_horiz=t_horiz,
                reference_path=reference_path,
                target_velocity=target_velocity,
                obstacles=obstacles,
            )

            if traj.is_valid and traj.cost < min_cost:
                min_cost = traj.cost
                best_traj = traj

        if best_traj is not None:
            return best_traj

        # Fallback: Emergency Braking Trajectory along current lane
        emergency_t = 2.0
        lat_safe = QuinticPolynomial(
            x0=current_state.d,
            v0=current_state.d_d,
            a0=current_state.d_dd,
            x1=current_state.d,
            v1=0.0,
            a1=0.0,
            t_horizon=emergency_t,
        )
        lon_safe = QuarticPolynomial(
            x0=current_state.s,
            v0=current_state.s_d,
            a0=current_state.s_dd,
            v1=0.0,  # Hard stop
            a1=self.config.max_decel_mps2,
            t_horizon=emergency_t,
        )
        fallback = self.evaluate_and_convert_trajectory(
            lat_safe,
            lon_safe,
            emergency_t,
            reference_path,
            target_velocity=0.0,
            obstacles=None,  # Ignore collision in pure emergency braking fallback
        )
        fallback.is_valid = False
        fallback.invalidation_reason = "Emergency braking fallback triggered"
        return fallback
