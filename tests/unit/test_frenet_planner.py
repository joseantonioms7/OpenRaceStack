"""
Unit Tests for Frenet Lattice Trajectory Planner and Dynamic Obstacle Avoidance.
"""

import math

import numpy as np
import pytest

from Localization.core.frenet_planner import (
    FrenetLatticePlanner,
    FrenetState,
    Obstacle,
    QuarticPolynomial,
    QuinticPolynomial,
)


@pytest.fixture
def straight_raceline():
    """Synthetic straight raceline along the X-axis from x=0 to x=300."""
    x = np.linspace(0.0, 300.0, 300)
    y = np.zeros_like(x)
    return np.column_stack((x, y))


def test_quintic_polynomial_boundary_conditions():
    """Verify quintic polynomial satisfies initial and final boundary constraints."""
    x0, v0, a0 = 1.0, 0.5, -0.2
    x1, v1, a1 = -2.0, 0.0, 0.0
    t_horiz = 2.5

    poly = QuinticPolynomial(x0, v0, a0, x1, v1, a1, t_horiz)

    # Initial conditions at t = 0
    assert math.isclose(poly.calc_point(0.0), x0, abs_tol=1e-5)
    assert math.isclose(poly.calc_first_derivative(0.0), v0, abs_tol=1e-5)
    assert math.isclose(poly.calc_second_derivative(0.0), a0, abs_tol=1e-5)

    # Final conditions at t = T
    assert math.isclose(poly.calc_point(t_horiz), x1, abs_tol=1e-5)
    assert math.isclose(poly.calc_first_derivative(t_horiz), v1, abs_tol=1e-5)
    assert math.isclose(poly.calc_second_derivative(t_horiz), a1, abs_tol=1e-5)

    # Integrated jerk must be positive
    jerk_cost = poly.calc_integrated_squared_jerk()
    assert jerk_cost > 0.0


def test_quartic_polynomial_boundary_conditions():
    """Verify quartic polynomial satisfies initial pose and target velocity constraints."""
    s0, v0, a0 = 10.0, 5.0, 1.0
    v1, a1 = 15.0, 0.0
    t_horiz = 3.0

    poly = QuarticPolynomial(s0, v0, a0, v1, a1, t_horiz)

    # Initial conditions at t = 0
    assert math.isclose(poly.calc_point(0.0), s0, abs_tol=1e-5)
    assert math.isclose(poly.calc_first_derivative(0.0), v0, abs_tol=1e-5)
    assert math.isclose(poly.calc_second_derivative(0.0), a0, abs_tol=1e-5)

    # Final velocity at t = T
    assert math.isclose(poly.calc_first_derivative(t_horiz), v1, abs_tol=1e-4)

    # Integrated jerk must be positive
    assert poly.calc_integrated_squared_jerk() > 0.0


def test_cartesian_to_frenet_projection(straight_raceline):
    """Verify projection of vehicle pose onto Frenet coordinates."""
    # Vehicle at (20, 1.5) facing east (yaw=0) with v=10
    state = FrenetLatticePlanner.cartesian_to_frenet(
        x=20.0,
        y=1.5,
        yaw=0.0,
        v=10.0,
        reference_path=straight_raceline,
    )

    assert math.isclose(state.s, 20.0, abs_tol=0.5)
    assert math.isclose(state.d, 1.5, abs_tol=1e-3)
    assert math.isclose(state.s_d, 10.0, abs_tol=1e-3)
    assert math.isclose(state.d_d, 0.0, abs_tol=1e-3)


def test_clear_track_planning(straight_raceline):
    """Verify planner targets optimal raceline (d=0) when track is clear."""
    planner = FrenetLatticePlanner()
    init_state = FrenetState(s=10.0, s_d=15.0, s_dd=0.0, d=0.0, d_d=0.0, d_dd=0.0)

    traj = planner.plan(
        current_state=init_state,
        reference_path=straight_raceline,
        obstacles=[],
        target_velocity=20.0,
    )

    assert traj.is_valid
    # On clear track, trajectory stays on raceline (d ~ 0)
    assert abs(traj.d[-1]) < 0.1
    # Accelerates towards target velocity
    assert traj.v[-1] >= 15.0
    assert len(traj.x) > 0


def test_static_obstacle_avoidance(straight_raceline):
    """Verify planner navigates around an obstacle parked on the raceline."""
    planner = FrenetLatticePlanner()
    init_state = FrenetState(s=10.0, s_d=12.0, s_dd=0.0, d=0.0, d_d=0.0, d_dd=0.0)

    # Stationary obstacle on the centerline at x=24.0 (14m ahead)
    obs = Obstacle(x=24.0, y=0.0, vx=0.0, vy=0.0, radius=0.8, obstacle_id="static_kart")

    traj = planner.plan(
        current_state=init_state,
        reference_path=straight_raceline,
        obstacles=[obs],
        target_velocity=15.0,
    )

    assert traj.is_valid
    # Must deviate from centerline to avoid obstacle
    assert abs(traj.d[-1]) > 0.5

    # Check minimum clearance to obstacle at all trajectory points
    min_dist = float("inf")
    for t_val, x_val, y_val in zip(traj.t, traj.x, traj.y, strict=False):
        obs_x, obs_y = obs.predict_position(t_val)
        dist = math.hypot(x_val - obs_x, y_val - obs_y)
        if dist < min_dist:
            min_dist = dist

    # Clearance must exceed vehicle + obstacle radius
    assert min_dist >= planner.config.vehicle_radius_m + obs.radius


def test_dynamic_obstacle_overtaking(straight_raceline):
    """Verify planner overtakes a slower-moving vehicle ahead."""
    planner = FrenetLatticePlanner()
    init_state = FrenetState(s=10.0, s_d=18.0, s_dd=0.0, d=0.0, d_d=0.0, d_dd=0.0)

    # Slower car 12m ahead moving at 5 m/s on centerline
    moving_car = Obstacle(x=22.0, y=0.0, vx=5.0, vy=0.0, radius=0.8, obstacle_id="slow_car")

    traj = planner.plan(
        current_state=init_state,
        reference_path=straight_raceline,
        obstacles=[moving_car],
        target_velocity=20.0,
    )

    assert traj.is_valid
    # Deviation around moving vehicle
    assert abs(traj.d[-1]) > 0.5


def test_emergency_braking_when_fully_blocked(straight_raceline):
    """Verify emergency braking fallback is engaged when both lanes are blocked."""
    planner = FrenetLatticePlanner()
    init_state = FrenetState(s=10.0, s_d=15.0, s_dd=0.0, d=0.0, d_d=0.0, d_dd=0.0)

    # Wall of obstacles blocking left, center, and right
    wall = [
        Obstacle(x=25.0, y=0.0, radius=2.0, obstacle_id="center"),
        Obstacle(x=25.0, y=2.0, radius=2.0, obstacle_id="left"),
        Obstacle(x=25.0, y=-2.0, radius=2.0, obstacle_id="right"),
    ]

    traj = planner.plan(
        current_state=init_state,
        reference_path=straight_raceline,
        obstacles=wall,
        target_velocity=15.0,
    )

    # Emergency fallback should be triggered
    assert not traj.is_valid
    assert "Emergency" in traj.invalidation_reason
