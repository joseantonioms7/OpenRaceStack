"""
Unit Tests for Geodetic, Frenet, and Dynamic Metrics Evaluator.
"""

import numpy as np

from Localization.utils.metrics import calculate_advanced_metrics


def test_metrics_rmse_and_cep():
    """Verify that 2D RMSE, CEP50, and CEP95 match exact mathematical definitions."""
    n_pts = 100
    true_traj = np.zeros((n_pts, 2), dtype=np.float64)
    true_traj[:, 0] = np.linspace(0, 100, n_pts)

    # Synthetic estimated trajectory with fixed offset +0.3m in x, +0.4m in y (hypot = 0.5m)
    est_traj = true_traj.copy()
    est_traj[:, 0] += 0.3
    est_traj[:, 1] += 0.4

    # Noisy measurements with 1.0m offset
    noisy_traj = true_traj + 1.0

    metrics = calculate_advanced_metrics(
        track_true=true_traj,
        noisy_data=noisy_traj,
        ekf_data=est_traj,
        dt=0.05,
    )

    # Since all errors are exactly 0.5m, RMSE, CEP50, and CEP95 should all be 0.5m
    assert np.isclose(metrics["rmse_ekf"], 0.5, atol=1e-5)
    assert np.isclose(metrics["cep_50"], 0.5, atol=1e-5)
    assert np.isclose(metrics["cep_95"], 0.5, atol=1e-5)
    assert np.isclose(metrics["max_error"], 0.5, atol=1e-5)


def test_metrics_frenet_decomposition():
    """Verify along-track (longitudinal) and cross-track (lateral) decomposition."""
    n_pts = 50
    # Vehicle traveling purely along X axis (heading = 0)
    true_traj = np.column_stack((np.linspace(0, 50, n_pts), np.zeros(n_pts)))

    # Estimate has 0.2m along-track error and 0.4m cross-track error
    est_traj = true_traj.copy()
    est_traj[:, 0] += 0.2  # Along-track
    est_traj[:, 1] += 0.4  # Cross-track

    metrics = calculate_advanced_metrics(
        track_true=true_traj,
        noisy_data=true_traj + 1.0,
        ekf_data=est_traj,
        theta_true_hist=np.zeros(n_pts),
        dt=0.05,
    )

    assert np.isclose(metrics["rmse_lon"], 0.2, atol=1e-5)
    assert np.isclose(metrics["rmse_lat"], 0.4, atol=1e-5)
