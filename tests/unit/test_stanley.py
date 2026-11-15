"""
Unit Tests for Stanley (Stanford) Lateral & Longitudinal Path-Tracking Controller.
"""

import math

import numpy as np
import pytest

from Localization.core.dynamics import (
    F1TENTH_PARAMS,
    DynamicControlInput,
    KinematicBicycleModel,
    KinematicVehicleState,
)
from Localization.core.stanley import StanleyCommand, StanleyConfig, StanleyController


@pytest.fixture
def straight_path():
    """Straight reference path along global X axis (y = 0.0)."""
    x = np.linspace(0.0, 100.0, 201)
    y = np.zeros_like(x)
    return np.column_stack((x, y))


def test_stanley_straight_line_zero_error(straight_path):
    """Verify Stanley outputs zero steering when vehicle is perfectly aligned on path."""
    controller = StanleyController(wheelbase_m=0.33)
    cmd = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=0.0,
        vehicle_theta=0.0,
        vehicle_v=5.0,
        reference_path=straight_path,
        dt=0.05,
    )

    assert isinstance(cmd, StanleyCommand)
    assert math.isclose(cmd.cross_track_error_m, 0.0, abs_tol=1e-5)
    assert math.isclose(cmd.heading_error_rad, 0.0, abs_tol=1e-5)
    assert math.isclose(cmd.steering_angle_rad, 0.0, abs_tol=1e-4)


def test_stanley_cross_track_correction(straight_path):
    """Verify Stanley commands steering toward the path when laterally offset."""
    controller = StanleyController(wheelbase_m=0.33)

    # Vehicle is 0.5m to the right (y = -0.5) with zero heading error
    cmd = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=-0.5,
        vehicle_theta=0.0,
        vehicle_v=5.0,
        reference_path=straight_path,
        dt=0.05,
    )

    # Should steer left (positive steering angle) to return to path
    assert cmd.steering_angle_rad > 0.0
    assert math.isclose(abs(cmd.cross_track_error_m), 0.5, abs_tol=1e-3)


def test_stanley_heading_correction(straight_path):
    """Verify Stanley commands counter-steering when vehicle heading is misaligned."""
    controller = StanleyController(wheelbase_m=0.33)

    # Vehicle on path (y=0), but yawing left (theta = +0.15 rad)
    cmd = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=0.0,
        vehicle_theta=0.15,
        vehicle_v=5.0,
        reference_path=straight_path,
        dt=0.05,
    )

    # Should steer right (negative angle) to correct heading
    assert cmd.steering_angle_rad < 0.0
    assert math.isclose(cmd.heading_error_rad, -0.15, abs_tol=1e-3)


def test_stanley_steering_saturation_and_slew(straight_path):
    """Verify Stanley enforces physical steering angle limits and rate constraints."""
    config = StanleyConfig(
        max_steering_angle_rad=math.radians(20.0),
        max_steering_rate_radps=math.radians(60.0), # max 3 deg per 0.05s
    )
    controller = StanleyController(config=config, wheelbase_m=0.33)

    # Extreme lateral displacement (15 meters off track)
    cmd1 = controller.compute_control(
        vehicle_x=0.0,
        vehicle_y=-15.0,
        vehicle_theta=0.0,
        vehicle_v=5.0,
        reference_path=straight_path,
        dt=0.05,
    )

    # Rate limited on first step
    max_d_steer = math.radians(60.0) * 0.05
    assert abs(cmd1.steering_angle_rad) <= max_d_steer + 1e-5

    # Run multiple steps to let rate ramp up to saturation
    for _ in range(50):
        cmd = controller.compute_control(
            vehicle_x=0.0,
            vehicle_y=-15.0,
            vehicle_theta=0.0,
            vehicle_v=5.0,
            reference_path=straight_path,
            dt=0.05,
        )

    assert math.isclose(cmd.steering_angle_rad, config.max_steering_angle_rad, abs_tol=1e-3)
    assert cmd.is_saturated


def test_stanley_speed_tracking(straight_path):
    """Verify longitudinal P-control tracks target velocities within accel bounds."""
    config = StanleyConfig(max_accel_mps2=5.0, max_decel_mps2=-7.0, k_v=1.5)
    controller = StanleyController(config=config)

    # Target 10 m/s, vehicle at 5 m/s -> positive acceleration demand
    target_vels = np.full(len(straight_path), 10.0)
    cmd_accel = controller.compute_control(
        vehicle_x=5.0,
        vehicle_y=0.0,
        vehicle_theta=0.0,
        vehicle_v=5.0,
        reference_path=straight_path,
        reference_velocities=target_vels,
        dt=0.05,
    )
    assert cmd_accel.acceleration_mps2 > 0.0

    # Target 3 m/s, vehicle at 8 m/s -> braking deceleration demand
    target_slow = np.full(len(straight_path), 3.0)
    cmd_brake = controller.compute_control(
        vehicle_x=5.0,
        vehicle_y=0.0,
        vehicle_theta=0.0,
        vehicle_v=8.0,
        reference_path=straight_path,
        reference_velocities=target_slow,
        dt=0.05,
    )
    assert cmd_brake.acceleration_mps2 < 0.0


def test_stanley_closed_loop_convergence(straight_path):
    """Verify Stanley converges closed-loop with kinematic bicycle model."""
    controller = StanleyController(wheelbase_m=F1TENTH_PARAMS.wheelbase)
    model = KinematicBicycleModel(params=F1TENTH_PARAMS)

    # Initial state: 1.0m lateral offset
    state = KinematicVehicleState(x=0.0, y=1.0, psi=0.0, v=4.0)
    dt = 0.05

    for _ in range(120):  # 6.0 seconds simulation
        cmd = controller.compute_control(
            vehicle_x=state.x,
            vehicle_y=state.y,
            vehicle_theta=state.psi,
            vehicle_v=state.v,
            reference_path=straight_path,
            dt=dt,
        )
        ctrl = DynamicControlInput(steering_angle=cmd.steering_angle_rad, acceleration=cmd.acceleration_mps2)
        state = model.step_rk4(state, ctrl, dt=dt)

    # Final lateral error should be less than 5 cm
    assert abs(state.y) < 0.05
    assert abs(state.psi) < 0.02
