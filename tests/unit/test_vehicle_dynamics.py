"""
Unit Tests for Vehicle Dynamics, Tire Models, and Single-Track Simulation Layer.
"""

import math

import numpy as np
import pytest

from Localization.core.dynamics import (
    AV21_PARAMS,
    F1TENTH_PARAMS,
    GO_KART_PARAMS,
    PASSENGER_GT_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    KinematicBicycleModel,
    KinematicVehicleState,
    LinearTireModel,
    PacejkaTireModel,
    VehicleDynamicsState,
)

# ============================================================================
# TIRE MODEL TESTS
# ============================================================================

def test_linear_tire_model_linear_regime():
    """Verify linear tire model computes Fy = C_alpha * alpha in linear regime."""
    tire = LinearTireModel(cornering_stiffness_n_rad=50.0, friction_coeff=1.0)
    alpha = 0.02  # 0.02 rad (~1.15 deg)
    fz = 100.0   # 100 N
    fy = tire.compute_lateral_force(alpha, fz)

    # In linear regime: Fy = 50 * 0.02 = 1.0 N
    assert math.isclose(fy, 1.0, rel_tol=1e-4)


def test_linear_tire_model_saturation():
    """Verify linear tire model saturates at mu * Fz."""
    tire = LinearTireModel(cornering_stiffness_n_rad=1000.0, friction_coeff=0.9)
    alpha = 0.30  # High slip angle
    fz = 50.0     # Max lateral force = 0.9 * 50 = 45 N
    fy = tire.compute_lateral_force(alpha, fz)

    assert math.isclose(fy, 45.0, rel_tol=1e-4)

    # Verify negative saturation
    fy_neg = tire.compute_lateral_force(-alpha, fz)
    assert math.isclose(fy_neg, -45.0, rel_tol=1e-4)


def test_linear_tire_zero_normal_force():
    """Verify tire force is zero when normal load is zero or negative."""
    tire = LinearTireModel(cornering_stiffness_n_rad=50.0)
    assert tire.compute_lateral_force(0.1, 0.0) == 0.0
    assert tire.compute_lateral_force(0.1, -10.0) == 0.0


def test_pacejka_magic_formula_properties():
    """Verify Pacejka Magic Formula anti-symmetry, zero slip, and saturation."""
    tire = PacejkaTireModel(cornering_stiffness_n_rad=60.0, friction_coeff=1.1, shape_c=1.4)
    fz = 80.0  # N

    # 1. Zero slip -> zero force
    assert math.isclose(tire.compute_lateral_force(0.0, fz), 0.0, abs_tol=1e-6)

    # 2. Anti-symmetry: Fy(-alpha) == -Fy(alpha)
    fy_pos = tire.compute_lateral_force(0.05, fz)
    fy_neg = tire.compute_lateral_force(-0.05, fz)
    assert math.isclose(fy_pos, -fy_neg, rel_tol=1e-5)

    # 3. Peak force does not exceed friction limit mu * Fz
    max_theoretical_fy = 1.1 * fz
    for test_alpha in np.linspace(-0.5, 0.5, 50):
        fy = tire.compute_lateral_force(float(test_alpha), fz)
        assert abs(fy) <= max_theoretical_fy + 1e-4

    # 4. Small-angle initial slope matches cornering stiffness
    d_alpha = 1e-4
    slope = tire.compute_lateral_force(d_alpha, fz) / d_alpha
    assert math.isclose(slope, 60.0, rel_tol=0.05)


# ============================================================================
# VEHICLE PARAMETERS PRESETS
# ============================================================================

@pytest.mark.parametrize("params", [F1TENTH_PARAMS, GO_KART_PARAMS, AV21_PARAMS, PASSENGER_GT_PARAMS])
def test_vehicle_parameter_geometry(params):
    """Verify consistency of wheelbase geometry and physical properties across presets."""
    assert math.isclose(params.wheelbase, params.lf + params.lr, abs_tol=1e-3)
    assert params.mass > 0.0
    assert params.Iz > 0.0
    assert params.wheel_radius > 0.0
    assert params.max_steering_angle > 0.0
    assert params.max_accel > 0.0
    assert params.max_decel < 0.0


# ============================================================================
# DYNAMIC BICYCLE MODEL TESTS
# ============================================================================

def test_dynamic_bicycle_straight_propagation():
    """Verify dynamic bicycle drives straight forward with zero steering and accel."""
    model = DynamicBicycleModel(params=F1TENTH_PARAMS)
    init_state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=5.0, vy=0.0, omega_z=0.0)
    control = DynamicControlInput(steering_angle=0.0, acceleration=0.0)

    state = init_state
    dt = 0.02
    for _ in range(50):  # 1 second rollout
        state = model.step_rk4(state, control, dt)

    # Traveled ~5m along X
    assert math.isclose(state.x, 5.0, abs_tol=0.3)
    # Negligible lateral displacement and heading change
    assert abs(state.y) < 1e-4
    assert abs(state.psi) < 1e-4
    assert abs(state.omega_z) < 1e-4


def test_dynamic_bicycle_turning_response():
    """Verify that applying steer angle generates positive lateral force and yaw rate."""
    model = DynamicBicycleModel(params=F1TENTH_PARAMS)
    init_state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=4.0, vy=0.0, omega_z=0.0)
    # Steer left (+0.1 rad)
    control = DynamicControlInput(steering_angle=0.10, acceleration=0.0)

    state = model.step_rk4(init_state, control, dt=0.05)

    # Positive front slip angle & lateral force
    assert state.fy_f > 0.0
    # Positive yaw rate develops (turning left)
    assert state.omega_z > 0.0
    # Vehicle curves left into positive Y
    assert state.y > 0.0


def test_dynamic_bicycle_weight_transfer():
    """Verify longitudinal acceleration causes rearward normal load transfer."""
    model = DynamicBicycleModel(params=F1TENTH_PARAMS)
    state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=3.0, vy=0.0, omega_z=0.0)

    # Static / zero accel
    _, diag_static = model.compute_derivatives(state, DynamicControlInput(0.0, 0.0))

    # Heavy acceleration
    _, diag_accel = model.compute_derivatives(state, DynamicControlInput(0.0, 4.0))

    # Heavy braking
    _, diag_brake = model.compute_derivatives(state, DynamicControlInput(0.0, -6.0))

    # Under acceleration: rear load increases, front load decreases
    assert diag_accel["fz_r"] > diag_static["fz_r"]
    assert diag_accel["fz_f"] < diag_static["fz_f"]

    # Under braking: front load increases, rear load decreases
    assert diag_brake["fz_f"] > diag_static["fz_f"]
    assert diag_brake["fz_r"] < diag_static["fz_r"]


def test_dynamic_bicycle_low_speed_singularity_protection():
    """Verify low-speed singularity handling when vx is 0.0 m/s without NaN or crash."""
    model = DynamicBicycleModel(params=F1TENTH_PARAMS)
    # State at complete standstill
    standstill = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=0.0, vy=0.0, omega_z=0.0)
    control = DynamicControlInput(steering_angle=0.2, acceleration=2.0)

    derivatives, _ = model.compute_derivatives(standstill, control)
    assert not np.isnan(derivatives).any()
    assert not np.isinf(derivatives).any()

    # Step forward with RK4
    next_state = model.step_rk4(standstill, control, dt=0.05)
    assert not math.isnan(next_state.vx)
    assert not math.isnan(next_state.omega_z)
    assert next_state.vx >= 0.0


def test_dynamic_bicycle_euler_vs_rk4_stability():
    """Verify RK4 provides stable simulation compared to Euler under dynamic cornering."""
    model = DynamicBicycleModel(params=F1TENTH_PARAMS)
    s_rk4 = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=8.0, vy=0.0, omega_z=0.0)
    s_euler = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=8.0, vy=0.0, omega_z=0.0)
    control = DynamicControlInput(steering_angle=0.15, acceleration=1.0)

    dt = 0.05
    for _ in range(10):
        s_rk4 = model.step_rk4(s_rk4, control, dt)
        s_euler = model.step_euler(s_euler, control, dt)

    assert not math.isnan(s_rk4.x)
    assert not math.isnan(s_euler.x)
    # Both curve into positive Y
    assert s_rk4.y > 0.0
    assert s_euler.y > 0.0


# ============================================================================
# KINEMATIC BICYCLE MODEL TESTS
# ============================================================================

def test_kinematic_bicycle_turning_radius():
    """Verify kinematic bicycle follows geometric turning radius R = L / tan(delta)."""
    params = F1TENTH_PARAMS
    kin_model = KinematicBicycleModel(params=params)
    delta = 0.15  # rad
    v = 2.0      # m/s
    state = KinematicVehicleState(x=0.0, y=0.0, psi=0.0, v=v)
    control = DynamicControlInput(steering_angle=delta, acceleration=0.0)

    derivs = kin_model.compute_derivatives(state, control)
    # Yaw rate = v * cos(beta) / L * tan(delta)
    # beta is small, so omega_z ~ v / L * tan(delta)
    expected_omega_z = (v / params.wheelbase) * math.tan(delta)
    assert math.isclose(derivs[2], expected_omega_z, rel_tol=0.05)

    # RK4 rollout over short interval
    next_state = kin_model.step_rk4(state, control, dt=0.05)
    assert next_state.y > 0.0
    assert next_state.psi > 0.0
