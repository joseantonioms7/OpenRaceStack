"""
Unit Tests for Constrained Frenet-Frame Model Predictive Path Tracking Controller.
"""

import math

import numpy as np
import pytest

from Localization.core.frenet_mpc import (
    FrenetMPCConfig,
    FrenetMPCController,
)


@pytest.fixture
def straight_path():
    """Synthetic straight line path along the X-axis from x=0 to x=250."""
    x = np.linspace(0.0, 250.0, 500)
    y = np.zeros_like(x)
    return np.column_stack((x, y))


def test_frenet_mpc_straight_tracking(straight_path):
    """Verify Frenet MPC commands near-zero steering when vehicle is aligned on path."""
    controller = FrenetMPCController()
    cmd = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=0.0,
        vehicle_theta=0.0,
        vehicle_v=12.0,
        reference_path=straight_path,
    )

    assert abs(cmd.steering_angle_rad) < 1e-3
    assert abs(cmd.cross_track_error_m) < 1e-4
    assert cmd.predicted_trajectory.shape == (controller.config.horizon_steps, 2)
    assert not cmd.is_saturated_steering


def test_frenet_mpc_lateral_correction(straight_path):
    """Verify Frenet MPC commands corrective steering for lateral deviations."""
    controller = FrenetMPCController()

    # Offset to the right (y = -1.5) -> must steer left (+delta)
    cmd_right = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=-1.5,
        vehicle_theta=0.0,
        vehicle_v=10.0,
        reference_path=straight_path,
    )
    assert cmd_right.steering_angle_rad > 0.0
    assert cmd_right.cross_track_error_m < -1.0

    controller.reset()

    # Offset to the left (y = +1.5) -> must steer right (-delta)
    cmd_left = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=1.5,
        vehicle_theta=0.0,
        vehicle_v=10.0,
        reference_path=straight_path,
    )
    assert cmd_left.steering_angle_rad < 0.0
    assert cmd_left.cross_track_error_m > 1.0


def test_frenet_mpc_slew_rate_limiting(straight_path):
    """Verify Frenet MPC strictly enforces maximum steering slew rate constraint."""
    config = FrenetMPCConfig(
        max_steering_rate_radps=math.radians(60.0),
        dt=0.05,
    )
    controller = FrenetMPCController(config)
    controller.prev_delta = 0.0

    max_allowed_step = config.max_steering_rate_radps * config.dt

    # Large abrupt disturbance
    cmd = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=-10.0,
        vehicle_theta=0.0,
        vehicle_v=5.0,
        reference_path=straight_path,
    )

    assert abs(cmd.steering_angle_rad) <= max_allowed_step + 1e-5


def test_frenet_mpc_steering_saturation(straight_path):
    """Verify steering command saturates at max_steering_rad under extreme offset."""
    config = FrenetMPCConfig(
        max_steering_rad=math.radians(18.0),
        max_steering_rate_radps=math.radians(500.0), # High slew to isolate angle saturation
    )
    controller = FrenetMPCController(config)

    cmd = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=-20.0,
        vehicle_theta=0.0,
        vehicle_v=4.0,
        reference_path=straight_path,
    )

    assert abs(cmd.steering_angle_rad) <= config.max_steering_rad + 1e-6
    assert cmd.is_saturated_steering


def test_frenet_mpc_circular_track(circular_track):
    """Verify Frenet MPC follows curved circular track with bounded error."""
    controller = FrenetMPCController()

    # Place vehicle at (100, 0) facing north (pi/2) at 12 m/s
    cmd = controller.compute_control(
        vehicle_x=100.0,
        vehicle_y=0.0,
        vehicle_theta=math.pi / 2,
        vehicle_v=12.0,
        reference_path=circular_track,
    )

    # Must steer left (> 0) on counter-clockwise circle
    assert cmd.steering_angle_rad > 0.0
    assert abs(cmd.cross_track_error_m) < 0.2
    assert cmd.solve_time_ms < 25.0 # Real-time execution guarantee
