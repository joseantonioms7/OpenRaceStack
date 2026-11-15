"""
Vehicle Dynamics and Simulation Layer for OpenRaceStack.

Provides high-fidelity single-track bicycle models (Kinematic & Dynamic) with
Pacejka Magic Formula tire curves, normal load transfer (longitudinal acceleration
and aerodynamic downforce), and Runge-Kutta 4th order (RK4) numerical integration.
Includes vehicle parameter presets for 1:10 F1TENTH, Go-Karts, Dallara AV-21,
and full-size GT racing vehicles.
"""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from .types import wrap_angle


@dataclass(frozen=True)
class VehicleParameters:
    """
    Physical parameters defining race vehicle mass, geometry, aerodynamics,
    and actuator physical constraints.
    """
    mass: float                # Total vehicle mass m (kg)
    wheelbase: float           # Wheelbase L = lf + lr (m)
    lf: float                  # Distance from CG to front axle (m)
    lr: float                  # Distance from CG to rear axle (m)
    Iz: float                  # Yaw moment of inertia around CG Iz (kg*m^2)
    track_width: float = 0.20  # Axle track width (m)
    h_cg: float = 0.08         # Center of gravity height above ground (m)
    wheel_radius: float = 0.05 # Dynamic rolling wheel radius (m)
    drag_coeff: float = 0.35   # Aerodynamic drag coefficient Cd
    frontal_area: float = 0.10 # Frontal cross-sectional area A (m^2)
    downforce_coeff: float = 0.0 # Aerodynamic downforce coefficient Cl
    air_density: float = 1.225 # Air density rho (kg/m^3)
    rolling_resistance: float = 0.015 # Dimensionless coefficient of rolling resistance Crr
    max_steering_angle: float = math.radians(24.0) # Maximum front wheel steer angle (rad)
    max_steering_rate: float = math.radians(120.0) # Maximum steering angular velocity (rad/s)
    max_accel: float = 6.0     # Maximum longitudinal traction acceleration (m/s^2)
    max_decel: float = -8.0    # Maximum longitudinal braking deceleration (m/s^2)


# Standard Vehicle Profiles across racing scales
F1TENTH_PARAMS = VehicleParameters(
    mass=3.50,
    wheelbase=0.33,
    lf=0.158,
    lr=0.172,
    Iz=0.0471,
    track_width=0.20,
    h_cg=0.074,
    wheel_radius=0.052,
    drag_coeff=0.45,
    frontal_area=0.08,
    downforce_coeff=0.0,
    max_steering_angle=math.radians(24.0),
    max_steering_rate=math.radians(120.0),
    max_accel=6.5,
    max_decel=-8.5,
)

GO_KART_PARAMS = VehicleParameters(
    mass=165.0,
    wheelbase=1.045,
    lf=0.55,
    lr=0.495,
    Iz=35.0,
    track_width=0.85,
    h_cg=0.22,
    wheel_radius=0.14,
    drag_coeff=0.65,
    frontal_area=0.55,
    downforce_coeff=0.0,
    max_steering_angle=math.radians(20.0),
    max_steering_rate=math.radians(80.0),
    max_accel=5.5,
    max_decel=-9.0,
)

AV21_PARAMS = VehicleParameters(
    mass=750.0,
    wheelbase=2.97,
    lf=1.55,
    lr=1.42,
    Iz=850.0,
    track_width=1.60,
    h_cg=0.30,
    wheel_radius=0.32,
    drag_coeff=0.92,
    frontal_area=1.45,
    downforce_coeff=2.85, # IndyCar aerodynamic downforce package
    max_steering_angle=math.radians(15.0),
    max_steering_rate=math.radians(45.0),
    max_accel=12.0,
    max_decel=-25.0,
)

PASSENGER_GT_PARAMS = VehicleParameters(
    mass=1350.0,
    wheelbase=2.65,
    lf=1.25,
    lr=1.40,
    Iz=1950.0,
    track_width=1.65,
    h_cg=0.42,
    wheel_radius=0.33,
    drag_coeff=0.38,
    frontal_area=2.10,
    downforce_coeff=0.85,
    max_steering_angle=math.radians(30.0),
    max_steering_rate=math.radians(60.0),
    max_accel=8.0,
    max_decel=-14.0,
)


# ============================================================================
# TIRE MODELS
# ============================================================================

class TireModel(ABC):
    """Abstract base class for lateral tire force modeling."""

    @abstractmethod
    def compute_lateral_force(self, slip_angle_rad: float, normal_force_n: float) -> float:
        """
        Calculates tire lateral force Fy (N) given tire slip angle alpha (rad)
        and normal wheel load Fz (N).
        """
        pass


class LinearTireModel(TireModel):
    """
    Linear cornering stiffness tire model with Coulomb friction saturation.
    Fy = clip(C_alpha * alpha, -mu * Fz, mu * Fz)
    """

    def __init__(self, cornering_stiffness_n_rad: float = 40.0, friction_coeff: float = 1.0):
        self.c_alpha = float(cornering_stiffness_n_rad)
        self.friction_coeff = float(friction_coeff)

    def compute_lateral_force(self, slip_angle_rad: float, normal_force_n: float) -> float:
        if normal_force_n <= 1e-3:
            return 0.0
        max_fy = self.friction_coeff * normal_force_n
        fy_linear = self.c_alpha * slip_angle_rad
        return float(np.clip(fy_linear, -max_fy, max_fy))


class PacejkaTireModel(TireModel):
    """
    Pacejka Magic Formula lateral tire model (Bakker, Nyborg, Pacejka '87 / '96).
    Fy = D * sin(C * arctan(B * alpha - E * (B * alpha - arctan(B * alpha))))
    where:
      D = mu * Fz (peak force)
      C = shape factor (~1.3 - 1.65)
      B = stiffness factor (B = C_alpha / (C * D))
      E = curvature factor (~ -1.0 to 0.0)
    """

    def __init__(
        self,
        cornering_stiffness_n_rad: float = 45.0,
        friction_coeff: float = 1.1,
        shape_c: float = 1.35,
        curvature_e: float = -0.10,
    ):
        self.c_alpha = float(cornering_stiffness_n_rad)
        self.friction_coeff = float(friction_coeff)
        self.shape_c = float(shape_c)
        self.curvature_e = float(curvature_e)

    def compute_lateral_force(self, slip_angle_rad: float, normal_force_n: float) -> float:
        if normal_force_n <= 1e-3:
            return 0.0

        # Peak lateral force D
        d = self.friction_coeff * normal_force_n
        if d <= 1e-6:
            return 0.0

        # Cornering stiffness C_alpha scales with normal load or stays at nominal
        # B * C * D = C_alpha => B = C_alpha / (C * D)
        # Avoid division by zero
        b = self.c_alpha / (self.shape_c * d)

        # Magic Formula evaluation
        bx = b * slip_angle_rad
        inner = bx - self.curvature_e * (bx - math.atan(bx))
        fy = d * math.sin(self.shape_c * math.atan(inner))
        return float(fy)


# ============================================================================
# STATE AND DYNAMICS CLASSES
# ============================================================================

@dataclass
class VehicleDynamicsState:
    """High-fidelity state vector and diagnostic metrics for vehicle dynamics."""
    x: float             # Global inertial X position (m)
    y: float             # Global inertial Y position (m)
    psi: float           # Inertial heading yaw angle (rad)
    vx: float            # Body-fixed longitudinal velocity (m/s)
    vy: float            # Body-fixed lateral velocity (m/s)
    omega_z: float       # Body-fixed yaw rate (rad/s)
    
    # Diagnostic outputs (evaluated at current state)
    speed: float = 0.0           # Total speed norm (m/s)
    beta: float = 0.0            # Vehicle center-of-gravity side-slip angle (rad)
    alpha_f: float = 0.0         # Front tire slip angle (rad)
    alpha_r: float = 0.0         # Rear tire slip angle (rad)
    fz_f: float = 0.0            # Front axle normal load (N)
    fz_r: float = 0.0            # Rear axle normal load (N)
    fy_f: float = 0.0            # Front tire lateral force (N)
    fy_r: float = 0.0            # Rear tire lateral force (N)

    def __post_init__(self):
        self.speed = math.hypot(self.vx, self.vy)
        self.beta = math.atan2(self.vy, self.vx) if abs(self.vx) > 1e-3 else 0.0
        self.psi = wrap_angle(self.psi)


@dataclass
class DynamicControlInput:
    """Actuator commands into vehicle dynamics models."""
    steering_angle: float  # Front wheel steer angle delta (rad)
    acceleration: float    # Longitudinal acceleration demand (m/s^2)


class DynamicBicycleModel:
    """
    Nonlinear Dynamic Single-Track Bicycle Model with Pacejka tires,
    longitudinal dynamic weight transfer, aerodynamic downforce & drag,
    and Runge-Kutta 4th order (RK4) integration.
    
    State vector: x = [x, y, psi, vx, vy, omega_z]
    Control vector: u = [delta, ax]
    """

    def __init__(
        self,
        params: VehicleParameters = F1TENTH_PARAMS,
        front_tire: TireModel | None = None,
        rear_tire: TireModel | None = None,
        low_speed_threshold_mps: float = 0.5,
    ):
        self.params = params
        self.front_tire = front_tire or PacejkaTireModel()
        self.rear_tire = rear_tire or PacejkaTireModel()
        self.low_speed_thresh = float(low_speed_threshold_mps)
        self.g = 9.80665

    def compute_derivatives(
        self,
        state: VehicleDynamicsState,
        control: DynamicControlInput,
    ) -> tuple[np.ndarray, dict[str, float]]:
        """
        Computes state derivatives d/dt [x, y, psi, vx, vy, omega_z].
        Returns (derivatives_vector, diagnostic_dict).
        """
        delta = float(np.clip(
            control.steering_angle,
            -self.params.max_steering_angle,
            self.params.max_steering_angle,
        ))
        ax = float(np.clip(
            control.acceleration,
            self.params.max_decel,
            self.params.max_accel,
        ))

        vx = state.vx
        vy = state.vy
        omega_z = state.omega_z
        psi = state.psi

        # 1. Aerodynamic drag and downforce
        f_aero_drag = (
            0.5 * self.params.air_density
            * self.params.drag_coeff
            * self.params.frontal_area
            * (vx**2) * np.sign(vx)
        )
        f_aero_down = (
            0.5 * self.params.air_density
            * self.params.downforce_coeff
            * self.params.frontal_area
            * (vx**2)
        )

        # 2. Dynamic Normal Load Transfer (Static + Accel transfer + Aero)
        m = self.params.mass
        l_f = self.params.lf
        l_r = self.params.lr
        wb = self.params.wheelbase
        h = self.params.h_cg

        # Longitudinal load transfer: -m*ax*h/L to front, +m*ax*h/L to rear
        f_zf = (m * self.g * l_r - m * ax * h + 0.5 * f_aero_down * l_r) / wb
        f_zr = (m * self.g * l_f + m * ax * h + 0.5 * f_aero_down * l_f) / wb

        # Clamping normal load >= 0 (wheel unweighting / liftoff)
        f_zf = max(0.0, f_zf)
        f_zr = max(0.0, f_zr)

        # 3. Tire slip angles with low-speed singularity regularization
        # As vx -> 0, dynamic tire slip diverges. We blend smoothly with low-speed kinematics
        vx_eff = math.sqrt(vx**2 + (0.35)**2)  # Regularized speed floor for slip computation

        alpha_f = delta - math.atan((vy + l_f * omega_z) / vx_eff)
        alpha_r = -math.atan((vy - l_r * omega_z) / vx_eff)

        # 4. Tire lateral forces
        fy_f = self.front_tire.compute_lateral_force(alpha_f, f_zf)
        fy_r = self.rear_tire.compute_lateral_force(alpha_r, f_zr)

        # 5. Rolling resistance force
        f_roll = m * self.g * self.params.rolling_resistance * np.sign(vx) if abs(vx) > 0.05 else 0.0

        # 6. Equations of Motion in Vehicle Body Frame
        # d(vx)/dt = ax + vy * omega_z - (fy_f * sin(delta) + f_aero_drag + f_roll) / m
        dot_vx = ax + vy * omega_z - (fy_f * math.sin(delta) + f_aero_drag + f_roll) / m

        # d(vy)/dt = -vx * omega_z + (fy_f * cos(delta) + fy_r) / m
        dot_vy = -vx * omega_z + (fy_f * math.cos(delta) + fy_r) / m

        # d(omega_z)/dt = (l_f * fy_f * cos(delta) - l_r * fy_r) / Iz
        dot_omega_z = (l_f * fy_f * math.cos(delta) - l_r * fy_r) / self.params.Iz

        # Kinematic coordinate transformation to Global Inertial Frame
        dot_x = vx * math.cos(psi) - vy * math.sin(psi)
        dot_y = vx * math.sin(psi) + vy * math.cos(psi)
        dot_psi = omega_z

        # Smooth kinematic blending at low speeds (< low_speed_thresh)
        speed = math.hypot(vx, vy)
        if speed < self.low_speed_thresh:
            blend = speed / self.low_speed_thresh
            # Kinematic yaw rate target = vx / L * tan(delta)
            kin_omega_z = (vx / wb) * math.tan(delta)
            dot_omega_z = blend * dot_omega_z + (1.0 - blend) * (kin_omega_z - omega_z) * 10.0
            dot_vy = blend * dot_vy + (1.0 - blend) * 0.0

        derivatives = np.array([dot_x, dot_y, dot_psi, dot_vx, dot_vy, dot_omega_z], dtype=np.float64)
        diagnostics = {
            "alpha_f": alpha_f,
            "alpha_r": alpha_r,
            "fz_f": f_zf,
            "fz_r": f_zr,
            "fy_f": fy_f,
            "fy_r": fy_r,
        }
        return derivatives, diagnostics

    def step_rk4(
        self,
        state: VehicleDynamicsState,
        control: DynamicControlInput,
        dt: float,
    ) -> VehicleDynamicsState:
        """
        Integrates the dynamic single-track equations of motion using
        4th-order Runge-Kutta (RK4) numerical scheme.
        """
        def state_to_vec(s: VehicleDynamicsState) -> np.ndarray:
            return np.array([s.x, s.y, s.psi, s.vx, s.vy, s.omega_z], dtype=np.float64)

        def vec_to_state(v: np.ndarray) -> VehicleDynamicsState:
            return VehicleDynamicsState(
                x=float(v[0]),
                y=float(v[1]),
                psi=float(v[2]),
                vx=float(v[3]),
                vy=float(v[4]),
                omega_z=float(v[5]),
            )

        y0 = state_to_vec(state)

        # k1
        k1, diag = self.compute_derivatives(state, control)

        # k2
        y1 = y0 + 0.5 * dt * k1
        s1 = vec_to_state(y1)
        k2, _ = self.compute_derivatives(s1, control)

        # k3
        y2 = y0 + 0.5 * dt * k2
        s2 = vec_to_state(y2)
        k3, _ = self.compute_derivatives(s2, control)

        # k4
        y3 = y0 + dt * k3
        s3 = vec_to_state(y3)
        k4, _ = self.compute_derivatives(s3, control)

        # Combine RK4 stages
        y_next = y0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        new_state = vec_to_state(y_next)
        new_state.alpha_f = diag["alpha_f"]
        new_state.alpha_r = diag["alpha_r"]
        new_state.fz_f = diag["fz_f"]
        new_state.fz_r = diag["fz_r"]
        new_state.fy_f = diag["fy_f"]
        new_state.fy_r = diag["fy_r"]
        return new_state

    def step_euler(
        self,
        state: VehicleDynamicsState,
        control: DynamicControlInput,
        dt: float,
    ) -> VehicleDynamicsState:
        """First-order Forward Euler integration step (baseline comparison)."""
        derivatives, diag = self.compute_derivatives(state, control)
        new_state = VehicleDynamicsState(
            x=state.x + dt * derivatives[0],
            y=state.y + dt * derivatives[1],
            psi=state.psi + dt * derivatives[2],
            vx=state.vx + dt * derivatives[3],
            vy=state.vy + dt * derivatives[4],
            omega_z=state.omega_z + dt * derivatives[5],
        )
        new_state.alpha_f = diag["alpha_f"]
        new_state.alpha_r = diag["alpha_r"]
        new_state.fz_f = diag["fz_f"]
        new_state.fz_r = diag["fz_r"]
        new_state.fy_f = diag["fy_f"]
        new_state.fy_r = diag["fy_r"]
        return new_state


# ============================================================================
# KINEMATIC BICYCLE MODEL
# ============================================================================

@dataclass
class KinematicVehicleState:
    """State vector for kinematic bicycle model."""
    x: float
    y: float
    psi: float
    v: float

    def __post_init__(self):
        self.psi = wrap_angle(self.psi)


class KinematicBicycleModel:
    """
    Standard kinematic bicycle model with CG side-slip angle beta.
    Suitable for low-to-medium speeds or real-time optimization when tire slip is negligible.
    """

    def __init__(self, params: VehicleParameters = F1TENTH_PARAMS):
        self.params = params

    def compute_derivatives(
        self,
        state: KinematicVehicleState,
        control: DynamicControlInput,
    ) -> np.ndarray:
        delta = float(np.clip(
            control.steering_angle,
            -self.params.max_steering_angle,
            self.params.max_steering_angle,
        ))
        accel = float(np.clip(
            control.acceleration,
            self.params.max_decel,
            self.params.max_accel,
        ))

        # Side slip angle at center of gravity
        # tan(beta) = (lr / (lf + lr)) * tan(delta)
        beta = math.atan((self.params.lr / self.params.wheelbase) * math.tan(delta))

        dot_x = state.v * math.cos(state.psi + beta)
        dot_y = state.v * math.sin(state.psi + beta)
        dot_psi = (state.v * math.cos(beta) / self.params.wheelbase) * math.tan(delta)
        dot_v = accel

        return np.array([dot_x, dot_y, dot_psi, dot_v], dtype=np.float64)

    def step_rk4(
        self,
        state: KinematicVehicleState,
        control: DynamicControlInput,
        dt: float,
    ) -> KinematicVehicleState:
        y0 = np.array([state.x, state.y, state.psi, state.v], dtype=np.float64)

        def to_state(arr: np.ndarray) -> KinematicVehicleState:
            return KinematicVehicleState(x=float(arr[0]), y=float(arr[1]), psi=float(arr[2]), v=float(arr[3]))

        k1 = self.compute_derivatives(state, control)
        k2 = self.compute_derivatives(to_state(y0 + 0.5 * dt * k1), control)
        k3 = self.compute_derivatives(to_state(y0 + 0.5 * dt * k2), control)
        k4 = self.compute_derivatives(to_state(y0 + dt * k3), control)

        y_next = y0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return to_state(y_next)
