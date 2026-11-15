"""
Unit Tests for System Identification Engine (RLS and Batch Parameter Estimation).
"""

import math

import numpy as np
import pytest

from Localization.core.dynamics import (
    F1TENTH_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    LinearTireModel,
    VehicleDynamicsState,
)
from Localization.core.sys_id import (
    BatchSystemIdentifier,
    OnlineVehicleSystemIdentifier,
    RecursiveLeastSquares,
    SysIdEstimate,
)


def test_recursive_least_squares_scalar_convergence():
    """Verify RLS converges to ground truth parameters on synthetic noisy linear problem."""
    # True relationship: y = 2.5 * x1 + 4.0 * x2
    theta_true = np.array([2.5, 4.0])
    rls = RecursiveLeastSquares(n_params=2, forgetting_factor=0.99, initial_covariance=100.0)

    rng = np.random.default_rng(123)
    for _ in range(300):
        phi = rng.uniform(-2.0, 2.0, size=2)
        noise = rng.normal(0.0, 0.05)
        y = float(phi @ theta_true + noise)
        rls.update(phi, y)

    assert math.isclose(rls.theta[0], 2.5, rel_tol=0.05)
    assert math.isclose(rls.theta[1], 4.0, rel_tol=0.05)


@pytest.fixture
def dynamic_bicycle_telemetry():
    """Simulates dynamic bicycle slalom maneuver with known linear tire parameters."""
    c_true = 50.0  # N/rad cornering stiffness
    tire = LinearTireModel(cornering_stiffness_n_rad=c_true, friction_coeff=1.0)
    model = DynamicBicycleModel(params=F1TENTH_PARAMS, front_tire=tire, rear_tire=tire)

    state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=6.0, vy=0.0, omega_z=0.0)
    dt = 0.02
    n_steps = 250

    vx_arr = np.zeros(n_steps)
    vy_arr = np.zeros(n_steps)
    omega_arr = np.zeros(n_steps)
    delta_arr = np.zeros(n_steps)
    ay_arr = np.zeros(n_steps)

    for i in range(n_steps):
        t = i * dt
        # Sinusoidal steering excitation (slalom)
        steer = 0.12 * math.sin(2.0 * math.pi * 1.5 * t)
        ctrl = DynamicControlInput(steering_angle=steer, acceleration=0.0)

        state = model.step_rk4(state, ctrl, dt=dt)

        vx_arr[i] = state.vx
        vy_arr[i] = state.vy
        omega_arr[i] = state.omega_z
        delta_arr[i] = steer
        # Lateral acceleration ay = v * omega_z + dy/dt
        ay_arr[i] = (state.fy_f + state.fy_r) / F1TENTH_PARAMS.mass

    return {
        "vx": vx_arr,
        "vy": vy_arr,
        "omega": omega_arr,
        "delta": delta_arr,
        "ay": ay_arr,
        "c_true": c_true,
    }


def test_online_system_identifier_convergence(dynamic_bicycle_telemetry):
    """Verify online system identifier estimates cornering stiffness within 10% tolerance."""
    data = dynamic_bicycle_telemetry
    identifier = OnlineVehicleSystemIdentifier(
        params=F1TENTH_PARAMS,
        forgetting_factor=0.99,
        min_excitation_speed_mps=1.0,
    )

    n_samples = len(data["vx"])
    latest_estimate = None

    for i in range(n_samples):
        latest_estimate = identifier.update_from_telemetry(
            vx=data["vx"][i],
            vy=data["vy"][i],
            omega_z=data["omega"][i],
            steer_angle=data["delta"][i],
            ay_measured=data["ay"][i],
        )

    assert isinstance(latest_estimate, SysIdEstimate)
    assert latest_estimate.sample_count > 50

    # Average of front and rear stiffness should converge near c_true (50 N/rad)
    avg_c = 0.5 * (latest_estimate.c_alpha_f + latest_estimate.c_alpha_r)
    assert math.isclose(avg_c, data["c_true"], rel_tol=0.15)


def test_batch_system_identifier_accuracy_and_r2(dynamic_bicycle_telemetry):
    """Verify batch least-squares identification achieves high R^2 and parameter accuracy."""
    data = dynamic_bicycle_telemetry
    batch_id = BatchSystemIdentifier(params=F1TENTH_PARAMS)

    estimate = batch_id.identify_lateral_parameters(
        vx_arr=data["vx"],
        vy_arr=data["vy"],
        omega_arr=data["omega"],
        delta_arr=data["delta"],
        ay_arr=data["ay"],
    )

    assert isinstance(estimate, SysIdEstimate)
    assert estimate.sample_count == len(data["vx"])
    assert estimate.r_squared > 0.85

    # Estimated cornering stiffness within 15%
    avg_c = 0.5 * (estimate.c_alpha_f + estimate.c_alpha_r)
    assert math.isclose(avg_c, data["c_true"], rel_tol=0.15)
