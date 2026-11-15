"""
Unit Tests for 3D 15-DOF Error-State Kalman Filter (ESKF) on SE(3) x R^6.
"""

import math

import numpy as np


def test_eskf_initialization(initialized_eskf):
    """Verify nominal state and covariance dimensions."""
    assert initialized_eskf.p.shape == (3,)
    assert initialized_eskf.v.shape == (3,)
    assert initialized_eskf.q.shape == (4,)
    assert initialized_eskf.P.shape == (15, 15)
    # Unit quaternion initial norm
    assert math.isclose(np.linalg.norm(initialized_eskf.q), 1.0, rel_tol=1e-6)


def test_eskf_strapdown_prediction(initialized_eskf):
    """Verify high-rate IMU strapdown forward mechanization."""
    dt = 0.05
    # Pure gravity in z: stationary accelerometer reading [0, 0, 9.80665]
    acc_meas = [0.0, 0.0, 9.80665]
    gyr_meas = [0.0, 0.0, 0.0]

    initialized_eskf.predict_imu(acc_meas, gyr_meas, dt)

    # With v0 = [15.0, 0.0, 0.0], x should advance by 15.0 * 0.05 = 0.75m
    assert math.isclose(initialized_eskf.p[0], 0.75, rel_tol=1e-3)
    # Vertical acceleration should cancel with ENU gravity
    assert math.isclose(initialized_eskf.v[2], 0.0, abs_tol=1e-3)
    # Quaternion should remain normalized
    assert math.isclose(np.linalg.norm(initialized_eskf.q), 1.0, rel_tol=1e-6)


def test_eskf_gnss_update_and_covariance_symmetry(initialized_eskf):
    """Verify 3D GNSS position update and 15x15 covariance symmetry."""
    initialized_eskf.predict_imu([0.0, 0.0, 9.80665], [0.0, 0.0, 0.0], dt=0.05)
    z_gnss = [0.76, 0.01, 0.0]
    R_cov = np.diag([0.25, 0.25, 0.5])

    accepted = initialized_eskf.update_gnss_pos(z_gnss, R_cov)
    assert accepted is True

    # Covariance symmetry: P == P.T
    np.testing.assert_allclose(initialized_eskf.P, initialized_eskf.P.T, atol=1e-10)

    # Positive definiteness
    eigenvalues = np.linalg.eigvals(initialized_eskf.P)
    assert np.all(eigenvalues > 0.0)


def test_eskf_heading_update(initialized_eskf):
    """Verify course-over-ground heading update and gating."""
    # Small nominal heading update (within chi2 gate)
    nominal_yaw = math.radians(2.0)
    accepted = initialized_eskf.update_heading(nominal_yaw, var_yaw=0.04**2)
    assert accepted is True

    snap = initialized_eskf.get_state_snapshot()
    yaw_deg = snap["euler_deg"][2]
    assert yaw_deg > 0.0

    # Large outlier heading jump (e.g. 45 degrees) should be rejected by chi2 gate
    outlier_yaw = math.radians(45.0)
    rejected = initialized_eskf.update_heading(outlier_yaw, var_yaw=0.01**2)
    assert rejected is False
