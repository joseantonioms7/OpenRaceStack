"""
Unit Tests for Raceline Optimization, Dynamic Speed Profiling, and Track Curvatures.
"""

import math

import numpy as np

from Localization.core.dynamics import AV21_PARAMS, F1TENTH_PARAMS
from Localization.core.raceline_optimizer import (
    RacelineOptimizer,
    VehicleLimits,
)


def test_path_geometry_straight_line():
    """Verify arc length, tangents, and zero curvature on a straight line."""
    x = np.linspace(0.0, 100.0, 50)
    y = np.zeros_like(x)
    path = np.column_stack((x, y))

    s, tangents, headings, curvatures = RacelineOptimizer.compute_path_geometry(path, is_closed=False)

    assert math.isclose(s[-1], 100.0, rel_tol=1e-3)
    assert np.allclose(tangents[:, 0], 1.0)
    assert np.allclose(tangents[:, 1], 0.0)
    assert np.allclose(headings, 0.0)
    assert np.allclose(curvatures, 0.0, atol=1e-6)


def test_path_geometry_circle(circular_track):
    """Verify circle of radius 100m computes kappa ~ 1/100 = 0.01 and length 2*pi*R."""
    s, _, _, curvatures = RacelineOptimizer.compute_path_geometry(circular_track, is_closed=True)

    expected_len = 2.0 * np.pi * 100.0
    # Length of all segments
    total_len = s[-1] + np.hypot(circular_track[0, 0] - circular_track[-1, 0], circular_track[0, 1] - circular_track[-1, 1])
    assert math.isclose(total_len, expected_len, rel_tol=0.01)

    # Curvature of 100m radius circle is ~0.01 rad/m
    assert np.allclose(curvatures, 0.01, atol=1e-3)


def test_minimum_curvature_raceline_optimization():
    """Verify raceline optimizer straightens a chicane and respects boundaries."""
    # Create an S-curve / chicane centerline
    s_vals = np.linspace(0.0, 200.0, 100)
    x = s_vals
    y = 2.5 * np.sin(2.0 * np.pi * s_vals / 100.0)
    centerline = np.column_stack((x, y))

    w_right = 3.0
    w_left = 3.0
    margin = 0.2

    opt_path = RacelineOptimizer.optimize_minimum_curvature_path(
        centerline,
        width_right=w_right,
        width_left=w_left,
        safety_margin=margin,
        is_closed=False,
    )

    # 1. Output shape preserved
    assert opt_path.shape == centerline.shape

    # 2. Check that optimized path stays within track corridor bounds
    diff = opt_path - centerline
    offsets = np.hypot(diff[:, 0], diff[:, 1])
    assert np.all(offsets <= w_left - margin + 1e-3)

    # 3. Sum of squared curvatures of optimized raceline is less than centerline
    _, _, _, k_orig = RacelineOptimizer.compute_path_geometry(centerline, is_closed=False)
    _, _, _, k_opt = RacelineOptimizer.compute_path_geometry(opt_path, is_closed=False)

    sum_sq_k_orig = float(np.sum(k_orig**2))
    sum_sq_k_opt = float(np.sum(k_opt**2))

    # Raceline must be significantly flatter (lower total curvature)
    assert sum_sq_k_opt < sum_sq_k_orig


def test_speed_profiling_on_circle(circular_track):
    """Verify speed profiling clamps cornering speed to sqrt(ay_max / kappa)."""
    limits = VehicleLimits(
        max_lateral_accel_mps2=10.0,
        max_velocity_mps=50.0,
        max_accel_mps2=5.0,
        max_decel_mps2=-8.0,
    )

    profile = RacelineOptimizer.generate_speed_profile(circular_track, limits=limits, is_closed=True)

    # On a 100m circle (kappa = 0.01):
    # Theoretical apex speed = sqrt(10.0 / 0.01) = sqrt(1000) ~ 31.62 m/s
    expected_apex_v = math.sqrt(10.0 / 0.01)

    assert math.isclose(profile.avg_velocity_mps, expected_apex_v, rel_tol=0.05)
    assert profile.lap_time_s > 0.0
    assert profile.max_velocity_mps <= limits.max_velocity_mps


def test_speed_profiling_straight_acceleration():
    """Verify vehicle accelerates forward on straight away towards top speed."""
    x = np.linspace(0.0, 300.0, 150)
    y = np.zeros_like(x)
    straight = np.column_stack((x, y))

    limits = VehicleLimits(
        max_accel_mps2=4.0,
        max_velocity_mps=30.0,
    )

    profile = RacelineOptimizer.generate_speed_profile(straight, limits=limits, is_closed=False)

    # Curvatures are zero, velocities should reach top speed
    assert math.isclose(profile.max_velocity_mps, 30.0, rel_tol=1e-3)
    # Velocity at end of straight is higher than at start
    assert profile.velocities[-1] >= profile.velocities[0]


def test_vehicle_limits_preset_conversion():
    """Verify VehicleLimits correctly maps from F1TENTH and AV21 VehicleParameters."""
    lim_f1 = VehicleLimits.from_vehicle_params(F1TENTH_PARAMS, friction_coeff=1.2)
    assert lim_f1.max_accel_mps2 == F1TENTH_PARAMS.max_accel
    assert lim_f1.max_decel_mps2 == F1TENTH_PARAMS.max_decel
    assert math.isclose(lim_f1.max_lateral_accel_mps2, 1.2 * 9.80665, rel_tol=1e-3)

    lim_av21 = VehicleLimits.from_vehicle_params(AV21_PARAMS, friction_coeff=1.5)
    assert lim_av21.mass == 750.0
    assert lim_av21.downforce_coeff > 0.0
    assert lim_av21.max_velocity_mps == 90.0
