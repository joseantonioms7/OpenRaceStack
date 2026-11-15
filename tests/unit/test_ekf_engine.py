"""
Unit Tests for 5-State Planar CTRV LocalizationEKF.
"""

import math

import numpy as np

from Localization.core.ekf_engine import LocalizationEKF
from Localization.core.types import wrap_angle


def test_angle_wrapping():
    """Verify that wrap_angle normalizes angles strictly to [-pi, pi)."""
    assert math.isclose(wrap_angle(0.0), 0.0)
    assert math.isclose(wrap_angle(math.pi), -math.pi, abs_tol=1e-7)
    assert math.isclose(wrap_angle(3 * math.pi), -math.pi, abs_tol=1e-7)
    assert math.isclose(wrap_angle(-3 * math.pi), -math.pi, abs_tol=1e-7)
    assert math.isclose(wrap_angle(math.pi / 2), math.pi / 2)
    assert math.isclose(wrap_angle(-math.pi / 2), -math.pi / 2)


def test_ekf_straight_line_prediction(initialized_ekf):
    """Verify straight line prediction when omega = 0."""
    dt = 0.1
    initialized_ekf.predict(dt)
    # x = [px, py, theta, v, omega]
    # v = 15 m/s, theta = 0 => dx = 15 * 0.1 = 1.5m
    assert math.isclose(initialized_ekf.x[0, 0], 1.5, rel_tol=1e-3)
    assert math.isclose(initialized_ekf.x[1, 0], 0.0, abs_tol=1e-4)
    assert math.isclose(initialized_ekf.x[2, 0], 0.0, abs_tol=1e-4)


def test_ekf_circular_arc_prediction():
    """Verify exact circular-arc integration when omega != 0."""
    ekf = LocalizationEKF(
        dt=0.1,
        Q_diag=[0.01, 0.01, 0.001, 0.01, 0.1],
        R_diag=[1.0, 1.0],
    )
    # v = 10 m/s, omega = 0.5 rad/s => R = v / omega = 20m
    x0 = np.array([[0.0], [0.0], [0.0], [10.0], [0.5]], dtype=np.float64)
    ekf.initialize_state(x0)
    dt = 0.1
    ekf.predict(dt)
    # Expected theta = 0 + 0.5 * 0.1 = 0.05 rad
    assert math.isclose(ekf.x[2, 0], 0.05, rel_tol=1e-4)
    # dy should be positive turning left
    assert ekf.x[1, 0] > 0.0


def test_ekf_covariance_symmetry_and_positive_definiteness(initialized_ekf):
    """Verify that covariance P remains symmetric positive definite after updates."""
    # Run multiple prediction and update cycles
    for _ in range(10):
        initialized_ekf.predict(0.05)
        z_pos = np.array([[initialized_ekf.x[0, 0] + 0.05], [initialized_ekf.x[1, 0] - 0.02]])
        initialized_ekf.update_gps(z_pos, R_gps=np.diag([0.5, 0.5]))

    P = initialized_ekf.P
    # Symmetry check: P == P.T
    np.testing.assert_allclose(P, P.T, atol=1e-10)
    # Positive definiteness check: eigenvalues > 0
    eigenvalues = np.linalg.eigvals(P)
    assert np.all(eigenvalues > 0.0)


def test_ekf_chi2_gating_rejection(initialized_ekf):
    """Verify that outlier measurements exceeding chi2 threshold are rejected."""
    initialized_ekf.predict(0.05)
    # Normal measurement should be accepted
    accepted_normal = initialized_ekf.update_gps(
        np.array([[0.75], [0.0]]),
        R_gps=np.diag([0.5, 0.5]),
        gate_threshold=13.82,
    )
    assert accepted_normal is True

    # Wild outlier (100m spike) should be rejected by chi-squared gate
    outlier_z = np.array([[100.0], [100.0]])
    accepted_outlier = initialized_ekf.update_gps(
        outlier_z,
        R_gps=np.diag([0.5, 0.5]),
        gate_threshold=13.82,
    )
    assert accepted_outlier is False
