"""
Unit Tests for Adaptive Lookahead Pure Pursuit Path Tracking Controller.
"""

import math

import numpy as np
import pytest

from Localization.core.pure_pursuit import (
    PurePursuitConfig,
    PurePursuitController,
)


@pytest.fixture
def straight_path():
    """Synthetic straight line path along the X-axis from x=0 to x=200."""
    x = np.linspace(0.0, 200.0, 400)
    y = np.zeros_like(x)
    return np.column_stack((x, y))


def test_pure_pursuit_adaptive_lookahead():
    """Verify lookahead scales with speed and respects bounds [L_min, L_max]."""
    config = PurePursuitConfig(
        min_lookahead_m=1.0,
        max_lookahead_m=20.0,
        lookahead_gain_s=0.5,
    )
    controller = PurePursuitController(config)

    # At standstill -> L_min
    assert math.isclose(controller.compute_lookahead_distance(0.0), 1.0)

    # At 10 m/s -> 0.5 * 10 + 1.0 = 6.0 m
    assert math.isclose(controller.compute_lookahead_distance(10.0), 6.0)

    # At high speed 50 m/s -> saturated at L_max (20.0 m)
    assert math.isclose(controller.compute_lookahead_distance(50.0), 20.0)


def test_pure_pursuit_on_straight_path(straight_path):
    """Verify that vehicle centered on path commands near-zero steering."""
    controller = PurePursuitController()
    cmd = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=0.0,
        vehicle_theta=0.0,
        vehicle_v=10.0,
        reference_path=straight_path,
        dt=0.05,
    )

    assert abs(cmd.steering_angle_rad) < 1e-4
    assert abs(cmd.cross_track_error_m) < 1e-4
    assert not cmd.is_saturated_steering


def test_pure_pursuit_lateral_correction(straight_path):
    """Verify steering corrective sign for lateral offsets."""
    controller = PurePursuitController()

    # Vehicle offset to the left of path (y = +1.0) facing east
    # Needs to turn right (negative steer) to re-intercept path
    cmd_left = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=1.0,
        vehicle_theta=0.0,
        vehicle_v=8.0,
        reference_path=straight_path,
        dt=0.05,
    )
    assert cmd_left.steering_angle_rad < 0.0
    assert cmd_left.cross_track_error_m > 0.5

    # Reset controller memory
    controller.reset()

    # Vehicle offset to the right of path (y = -1.0) facing east
    # Needs to turn left (positive steer) to re-intercept path
    cmd_right = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=-1.0,
        vehicle_theta=0.0,
        vehicle_v=8.0,
        reference_path=straight_path,
        dt=0.05,
    )
    assert cmd_right.steering_angle_rad > 0.0
    assert cmd_right.cross_track_error_m < -0.5


def test_pure_pursuit_slew_rate_limiting(straight_path):
    """Verify controller enforces actuator maximum steering slew rate."""
    config = PurePursuitConfig(
        max_steering_rate_radps=math.radians(60.0), # 60 deg/s max rate
    )
    controller = PurePursuitController(config)
    dt = 0.05
    max_step_change = config.max_steering_rate_radps * dt

    # Initial zero steering state
    controller.prev_steering = 0.0

    # Demand massive abrupt lateral error
    cmd = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=-20.0,
        vehicle_theta=0.0,
        vehicle_v=5.0,
        reference_path=straight_path,
        dt=dt,
    )

    # Change in steering must not exceed max_step_change
    assert abs(cmd.steering_angle_rad) <= max_step_change + 1e-5


def test_pure_pursuit_steering_saturation(straight_path):
    """Verify steering command is strictly clamped to max_steering_rad."""
    config = PurePursuitConfig(
        min_lookahead_m=0.5,
        max_steering_rad=math.radians(15.0),
        max_steering_rate_radps=math.radians(500.0),  # High rate to isolate angle limit
    )
    controller = PurePursuitController(config)

    # Offset of 0.8m with 0.5m lookahead demands ~30 deg steer, which exceeds 15 deg limit
    cmd = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=-0.8,
        vehicle_theta=0.0,
        vehicle_v=0.1,
        reference_path=straight_path,
        dt=0.1,
    )

    assert abs(cmd.steering_angle_rad) <= config.max_steering_rad + 1e-6
    assert cmd.is_saturated_steering


def test_pure_pursuit_velocity_tracking(straight_path):
    """Verify acceleration output responds to target velocity speed error."""
    controller = PurePursuitController()

    # Under-speed: target 20 m/s, vehicle at 10 m/s -> positive acceleration
    targets = np.full(len(straight_path), 20.0)
    cmd_accel = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=0.0,
        vehicle_theta=0.0,
        vehicle_v=10.0,
        reference_path=straight_path,
        reference_velocities=targets,
    )
    assert cmd_accel.acceleration_mps2 > 0.0

    # Over-speed: target 10 m/s, vehicle at 25 m/s -> braking deceleration
    cmd_decel = controller.compute_control(
        vehicle_x=10.0,
        vehicle_y=0.0,
        vehicle_theta=0.0,
        vehicle_v=25.0,
        reference_path=straight_path,
        reference_velocities=np.full(len(straight_path), 10.0),
    )
    assert cmd_decel.acceleration_mps2 < 0.0


def test_pure_pursuit_circular_track(circular_track):
    """Verify tracking on a circular track with curvature."""
    controller = PurePursuitController()

    # Vehicle on circle (100, 0) facing tangent (pi/2)
    cmd = controller.compute_control(
        vehicle_x=100.0,
        vehicle_y=0.0,
        vehicle_theta=math.pi / 2,
        vehicle_v=15.0,
        reference_path=circular_track,
        dt=0.05,
    )

    # On a counter-clockwise circle, vehicle must steer left (> 0)
    assert cmd.steering_angle_rad > 0.0
    assert abs(cmd.cross_track_error_m) < 0.5
    assert cmd.solve_time_ms >= 0.0
