"""
Unit Tests for Path Guidance and MPC Controller.
"""

import math


def test_mpc_cross_track_error_sign(mpc_controller, circular_track):
    """Verify that cross-track error e_y has expected sign relative to reference path."""
    # Place vehicle at (105, 0) facing north (pi/2) on a 100m radius circle
    # At (100, 0), path tangent points north. Vehicle is at x=105 (to the right of tangent)
    cmd = mpc_controller.compute_control(
        vehicle_x=105.0,
        vehicle_y=0.0,
        vehicle_theta=math.pi / 2,
        vehicle_v=15.0,
        reference_path=circular_track,
    )
    # Distance to path is ~5m
    assert math.isclose(abs(cmd.cross_track_error_m), 5.0, abs_tol=0.2)
    # Output trajectory rollout shape should be (15, 2)
    assert cmd.predicted_trajectory.shape == (15, 2)


def test_mpc_steering_saturation(mpc_controller, circular_track):
    """Verify that steering angle respects actuator saturation limits."""
    # Huge lateral error should demand maximum steering
    cmd = mpc_controller.compute_control(
        vehicle_x=200.0,
        vehicle_y=0.0,
        vehicle_theta=0.0,
        vehicle_v=15.0,
        reference_path=circular_track,
    )
    assert abs(cmd.steering_angle_rad) <= mpc_controller.max_delta + 1e-6
    assert abs(cmd.steering_angle_deg) <= math.degrees(mpc_controller.max_delta) + 0.1
