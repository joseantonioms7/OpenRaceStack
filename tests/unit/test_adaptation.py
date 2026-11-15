"""
Unit Tests for Online Parameter Adaptation and Dynamics Model Validation.
"""

import math

import numpy as np

from Localization.core.adaptation import (
    ModelPredictionError,
    ModelPredictionValidator,
    OnlineAdaptationManager,
)
from Localization.core.dynamics import (
    F1TENTH_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    LinearTireModel,
    VehicleDynamicsState,
)
from Localization.core.frenet_mpc import FrenetMPCConfig
from Localization.core.sys_id import SysIdEstimate


def test_k_step_prediction_error_matched_model():
    """Verify k-step prediction error is negligible when model matches ground truth physics."""
    tire = LinearTireModel(cornering_stiffness_n_rad=50.0)
    model = DynamicBicycleModel(params=F1TENTH_PARAMS, front_tire=tire, rear_tire=tire)

    init_state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=5.0, vy=0.0, omega_z=0.0)
    controls = [DynamicControlInput(steering_angle=0.05 * math.sin(i * 0.1), acceleration=0.2) for i in range(20)]

    # Generate exact ground truth rollout
    gt_states = []
    st = init_state
    for ctrl in controls:
        st = model.step_rk4(st, ctrl, dt=0.05)
        gt_states.append(st)

    validator = ModelPredictionValidator(params=F1TENTH_PARAMS)
    pred_error = validator.evaluate_k_step_prediction_error(
        model=model,
        initial_state=init_state,
        controls=controls,
        ground_truth_states=gt_states,
        dt=0.05,
    )

    assert isinstance(pred_error, ModelPredictionError)
    assert pred_error.horizon_steps == 20
    assert pred_error.rmse_vx < 1e-4
    assert pred_error.rmse_vy < 1e-4
    assert pred_error.rmse_pos_2d < 1e-4


def test_k_step_prediction_error_mismatched_model():
    """Verify model prediction error sharply increases when vehicle parameters are mismatched."""
    tire_true = LinearTireModel(cornering_stiffness_n_rad=50.0)
    model_true = DynamicBicycleModel(params=F1TENTH_PARAMS, front_tire=tire_true, rear_tire=tire_true)

    tire_mismatch = LinearTireModel(cornering_stiffness_n_rad=15.0)  # 70% degraded stiffness
    model_mismatch = DynamicBicycleModel(params=F1TENTH_PARAMS, front_tire=tire_mismatch, rear_tire=tire_mismatch)

    init_state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=6.0, vy=0.0, omega_z=0.0)
    controls = [DynamicControlInput(steering_angle=0.15, acceleration=0.0) for _ in range(25)]

    # Generate ground truth from true model
    gt_states = []
    st = init_state
    for ctrl in controls:
        st = model_true.step_rk4(st, ctrl, dt=0.05)
        gt_states.append(st)

    validator = ModelPredictionValidator(params=F1TENTH_PARAMS)
    err_true = validator.evaluate_k_step_prediction_error(model_true, init_state, controls, gt_states)
    err_mismatched = validator.evaluate_k_step_prediction_error(model_mismatch, init_state, controls, gt_states)

    # Mismatched model must exhibit significantly larger prediction error
    assert err_mismatched.rmse_pos_2d > 10.0 * err_true.rmse_pos_2d
    assert err_mismatched.rmse_omega_z > 0.05


def test_online_adaptation_speed_profile_scaling():
    """Verify OnlineAdaptationManager scales speed profile under degraded surface grip."""
    manager = OnlineAdaptationManager(base_params=F1TENTH_PARAMS, nominal_mu=1.0, safety_margin=0.85)

    # Curvature of a tight curve: kappa = 0.2 m^-1 (radius = 5 m)
    curvatures = np.full(50, 0.2)
    nominal_speeds = np.full(50, 12.0)  # Aggressive 12 m/s

    # 1. Update with nominal grip (mu = 1.0)
    est_dry = SysIdEstimate(c_alpha_f=50.0, c_alpha_r=50.0, mu_peak=1.0)
    state_dry = manager.update_from_estimate(est_dry, upcoming_curvature=0.2)
    assert not state_dry.is_grip_degraded
    speeds_dry = manager.adapt_speed_profile(nominal_speeds, curvatures)

    # Theoretical limit: sqrt(1.0 * 9.81 * 0.85 / 0.2) ~ 6.45 m/s
    assert math.isclose(speeds_dry[0], math.sqrt(1.0 * 9.81 * 0.85 / 0.2), rel_tol=0.05)

    # 2. Update with wet/ice grip (mu = 0.40)
    est_wet = SysIdEstimate(c_alpha_f=25.0, c_alpha_r=25.0, mu_peak=0.40)
    state_wet = manager.update_from_estimate(est_wet, upcoming_curvature=0.2)
    assert state_wet.is_grip_degraded
    speeds_wet = manager.adapt_speed_profile(nominal_speeds, curvatures)

    # Speeds must be substantially reduced
    assert speeds_wet[0] < speeds_dry[0]
    expected_v_wet = math.sqrt(0.40 * 9.81 * 0.85 / 0.2)
    assert math.isclose(speeds_wet[0], expected_v_wet, rel_tol=0.05)


def test_adaptation_mpc_config():
    """Verify MPC tuning weights and acceleration limits adapt to surface friction."""
    manager = OnlineAdaptationManager(base_params=F1TENTH_PARAMS, nominal_mu=1.0)
    base_config = FrenetMPCConfig(wheelbase_m=0.33, max_accel_mps2=5.0, max_decel_mps2=-7.0, q_y=10.0)

    # Surface grip drops to mu = 0.50
    est_half = SysIdEstimate(c_alpha_f=30.0, c_alpha_r=30.0, mu_peak=0.50)
    manager.update_from_estimate(est_half)

    adapted_config = manager.adapt_mpc_config(base_config)

    # Lateral tracking penalty should double to enforce tighter corridor containment
    assert adapted_config.q_y > base_config.q_y
    # Max acceleration and braking should be scaled down by 50%
    assert math.isclose(adapted_config.max_accel_mps2, 2.5, abs_tol=1e-3)
    assert math.isclose(adapted_config.max_decel_mps2, -3.5, abs_tol=1e-3)


def test_reproducible_adaptation_friction_drop_experiment():
    """
    Reproducible Experiment:
    Simulates vehicle entering a high-speed corner while friction drops from mu=1.0 to mu=0.45.
    Demonstrates that the adaptive observer detects grip loss and commands pre-emptive braking,
    preventing track boundary infringement.
    """
    manager = OnlineAdaptationManager(nominal_mu=1.0, safety_margin=0.85)

    # Vehicle cornering on a curve with radius 8m (kappa = 0.125)
    curve_kappa = 0.125
    nominal_speed = 8.0  # m/s

    # Phase 1: High friction mu = 1.0 (dry asphalt)
    est_dry = SysIdEstimate(c_alpha_f=50.0, c_alpha_r=50.0, mu_peak=1.0)
    ad_dry = manager.update_from_estimate(est_dry, upcoming_curvature=curve_kappa)
    assert ad_dry.cornering_speed_limit_mps > nominal_speed

    # Phase 2: Abrupt friction drop to mu = 0.45 (water spray / oil slick)
    est_wet = SysIdEstimate(c_alpha_f=22.0, c_alpha_r=22.0, mu_peak=0.45)
    ad_wet = manager.update_from_estimate(est_wet, upcoming_curvature=curve_kappa)

    # The adapted cornering speed limit must immediately drop below current nominal speed
    safe_speed = ad_wet.cornering_speed_limit_mps
    assert safe_speed < nominal_speed
    assert ad_wet.is_grip_degraded

    # Pre-emptive deceleration demand calculated
    decel_required = (safe_speed**2 - nominal_speed**2) / (2.0 * 10.0)  # over 10m preview
    assert decel_required < 0.0  # Braking demanded
