"""
Online Parameter Adaptation and Model Validation Layer for OpenRaceStack.

Provides:
- k-step ahead dynamics model validation and prediction error metrics (RMSE vx, vy, omega_z)
- Real-time vehicle parameter adaptation adjusting controller velocity profiles,
  friction ellipse limits, and MPC horizon constraints under changing surface grip (mu)
"""

import math
from dataclasses import dataclass

import numpy as np

from .dynamics import (
    F1TENTH_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    VehicleDynamicsState,
    VehicleParameters,
)
from .frenet_mpc import FrenetMPCConfig
from .sys_id import SysIdEstimate


@dataclass
class ModelPredictionError:
    """Quantitative validation metrics evaluating vehicle model fidelity against telemetry."""
    rmse_vx: float                       # Longitudinal velocity RMSE (m/s)
    rmse_vy: float                       # Lateral velocity RMSE (m/s)
    rmse_omega_z: float                  # Yaw rate RMSE (rad/s)
    rmse_pos_2d: float                   # 2D position error RMSE (m)
    max_pos_error_m: float               # Peak position error (m)
    horizon_steps: int                   # Number of prediction steps evaluated


@dataclass
class AdaptationState:
    """Active vehicle parameter adaptation status and dynamic limits."""
    estimated_mu: float                  # Current road friction estimate
    effective_c_alpha: float             # Current tire cornering stiffness (N/rad)
    max_lateral_accel_mps2: float        # Adapted lateral acceleration ceiling mu * g * safety
    cornering_speed_limit_mps: float     # Speed ceiling for upcoming curvature
    is_grip_degraded: bool               # True if friction dropped below nominal (>25% drop)
    safety_margin: float = 0.85


class ModelPredictionValidator:
    """
    Computes k-step ahead prediction errors comparing model rollouts against telemetry.
    """

    def __init__(self, params: VehicleParameters = F1TENTH_PARAMS):
        self.params = params

    def evaluate_k_step_prediction_error(
        self,
        model: DynamicBicycleModel,
        initial_state: VehicleDynamicsState,
        controls: list[DynamicControlInput],
        ground_truth_states: list[VehicleDynamicsState],
        dt: float = 0.05,
    ) -> ModelPredictionError:
        """
        Rolls out model from initial_state applying controls, comparing against ground truth.
        """
        k = min(len(controls), len(ground_truth_states))
        if k == 0:
            raise ValueError("Control and ground truth state histories must not be empty.")

        pred_state = initial_state
        err_vx = []
        err_vy = []
        err_om = []
        err_pos = []

        for i in range(k):
            pred_state = model.step_rk4(pred_state, controls[i], dt=dt)
            gt = ground_truth_states[i]

            err_vx.append(pred_state.vx - gt.vx)
            err_vy.append(pred_state.vy - gt.vy)
            err_om.append(pred_state.omega_z - gt.omega_z)
            dist = math.hypot(pred_state.x - gt.x, pred_state.y - gt.y)
            err_pos.append(dist)

        err_vx_arr = np.array(err_vx, dtype=np.float64)
        err_vy_arr = np.array(err_vy, dtype=np.float64)
        err_om_arr = np.array(err_om, dtype=np.float64)
        err_pos_arr = np.array(err_pos, dtype=np.float64)

        return ModelPredictionError(
            rmse_vx=float(np.sqrt(np.mean(err_vx_arr**2))),
            rmse_vy=float(np.sqrt(np.mean(err_vy_arr**2))),
            rmse_omega_z=float(np.sqrt(np.mean(err_om_arr**2))),
            rmse_pos_2d=float(np.sqrt(np.mean(err_pos_arr**2))),
            max_pos_error_m=float(np.max(err_pos_arr)),
            horizon_steps=k,
        )


class OnlineAdaptationManager:
    """
    Supervises online parameter changes and reconfigures guidance, speed profiles,
    and MPC friction limits in response to surface degradation.
    """

    def __init__(
        self,
        base_params: VehicleParameters = F1TENTH_PARAMS,
        nominal_mu: float = 1.0,
        safety_margin: float = 0.85,
    ):
        self.base_params = base_params
        self.nominal_mu = float(nominal_mu)
        self.safety_margin = float(safety_margin)
        self.current_state = AdaptationState(
            estimated_mu=nominal_mu,
            effective_c_alpha=50.0,
            max_lateral_accel_mps2=nominal_mu * 9.81 * safety_margin,
            cornering_speed_limit_mps=25.0,
            is_grip_degraded=False,
            safety_margin=safety_margin,
        )

    def update_from_estimate(
        self,
        estimate: SysIdEstimate,
        upcoming_curvature: float = 0.0,
    ) -> AdaptationState:
        """
        Updates internal adaptation state from latest system identification output.
        """
        mu = float(np.clip(estimate.mu_peak, 0.2, 2.5))
        avg_c = 0.5 * (estimate.c_alpha_f + estimate.c_alpha_r)

        # Usable lateral acceleration ceiling under friction ellipse
        max_ay = mu * 9.81 * self.safety_margin

        # Curvature-limited cornering speed: v_max = sqrt(a_y_max / |kappa|)
        curv_abs = max(abs(upcoming_curvature), 1e-4)
        corner_v = math.sqrt(max_ay / curv_abs)

        is_degraded = (mu < 0.75 * self.nominal_mu)

        self.current_state = AdaptationState(
            estimated_mu=mu,
            effective_c_alpha=avg_c,
            max_lateral_accel_mps2=max_ay,
            cornering_speed_limit_mps=corner_v,
            is_grip_degraded=is_degraded,
            safety_margin=self.safety_margin,
        )
        return self.current_state

    def adapt_speed_profile(
        self,
        nominal_speeds: np.ndarray,
        curvatures: np.ndarray,
    ) -> np.ndarray:
        """
        Scales an entire trajectory velocity profile to respect current surface friction limits.
        """
        speeds = np.asarray(nominal_speeds, dtype=np.float64).copy()
        curvs = np.asarray(curvatures, dtype=np.float64)

        max_ay = self.current_state.max_lateral_accel_mps2
        for i in range(len(speeds)):
            k = max(abs(curvs[i]), 1e-4)
            v_limit = math.sqrt(max_ay / k)
            speeds[i] = min(speeds[i], v_limit)

        return speeds

    def adapt_mpc_config(self, base_config: FrenetMPCConfig) -> FrenetMPCConfig:
        """
        Adapts Frenet MPC cost matrices and acceleration bounds to surface friction.
        """
        mu_ratio = self.current_state.estimated_mu / self.nominal_mu
        clamped_ratio = float(np.clip(mu_ratio, 0.3, 1.2))

        # Under low friction, penalize lateral errors more aggressively and clamp braking/accel
        adapted_q_y = base_config.q_y * (1.0 / clamped_ratio)
        adapted_max_accel = base_config.max_accel_mps2 * clamped_ratio
        adapted_max_decel = base_config.max_decel_mps2 * clamped_ratio
        adapted_max_lat = base_config.max_lateral_accel_mps2 * clamped_ratio

        return FrenetMPCConfig(
            wheelbase_m=base_config.wheelbase_m,
            horizon_steps=base_config.horizon_steps,
            dt=base_config.dt,
            max_steering_rad=base_config.max_steering_rad,
            max_steering_rate_radps=base_config.max_steering_rate_radps,
            max_accel_mps2=adapted_max_accel,
            max_decel_mps2=adapted_max_decel,
            max_lateral_accel_mps2=adapted_max_lat,
            q_y=adapted_q_y,
            q_psi=base_config.q_psi,
            q_v=base_config.q_v,
            r_delta=base_config.r_delta,
            r_a=base_config.r_a,
            r_d_delta=base_config.r_d_delta,
            r_d_a=base_config.r_d_a,
        )
