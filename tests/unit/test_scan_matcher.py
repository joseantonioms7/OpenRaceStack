"""
Unit Tests for LiDAR Scan Matcher, Point-Cloud Filters, ICP, and 2D NDT.
"""

import math

import numpy as np

from Localization.core.scan_matcher import (
    IterativeClosestPoint2D,
    NormalDistributionsTransform2D,
    StatisticalOutlierFilter,
    VoxelGridFilter,
)


def test_voxel_grid_filter():
    """Verify voxel grid filter downsamples redundant points into grid centroids."""
    # Cluster 10 points within a 0.1m x 0.1m box
    pts_dense = np.random.uniform(0.0, 0.1, size=(10, 2))
    # 1 point far away
    pt_far = np.array([[5.0, 5.0]])
    points = np.vstack((pts_dense, pt_far))

    # Downsample with voxel_size = 0.5m
    downsampled = VoxelGridFilter.filter(points, voxel_size=0.5)

    # Should collapse the 10 clustered points into 1 centroid, leaving 2 points total
    assert len(downsampled) == 2
    assert math.isclose(downsampled[1, 0], 5.0, abs_tol=1e-3)


def test_statistical_outlier_filter():
    """Verify statistical outlier rejection removes isolated noise points."""
    # Dense linear barrier of 50 points
    x = np.linspace(0.0, 10.0, 50)
    y = np.zeros_like(x)
    dense_line = np.column_stack((x, y))

    # Isolated outlier point far from the barrier
    outlier = np.array([[5.0, 20.0]])
    noisy_cloud = np.vstack((dense_line, outlier))

    filtered = StatisticalOutlierFilter.filter(noisy_cloud, k_neighbors=6, std_ratio=1.5)

    # Outlier must be rejected
    assert len(filtered) == len(dense_line)
    assert not np.any(np.isclose(filtered[:, 1], 20.0))


def test_icp_identity_alignment():
    """Verify ICP with zero initial error converges immediately with near-zero fitness."""
    # Synthetic curved obstacle wall
    angles = np.linspace(-math.pi / 4, math.pi / 4, 40)
    target_map = np.column_stack((10.0 * np.cos(angles), 10.0 * np.sin(angles)))
    source_scan = target_map.copy()

    icp = IterativeClosestPoint2D()
    result = icp.align(source_scan, target_map, initial_pose=np.array([0.0, 0.0, 0.0]))

    assert result.converged
    assert result.fitness_score < 1e-4
    assert result.inlier_ratio > 0.95
    assert np.allclose(result.estimated_pose, 0.0, atol=1e-3)
    assert result.covariance.shape == (3, 3)


def test_icp_rigid_body_displacement():
    """Verify ICP recovers known translational and rotational perturbation."""
    # Synthetic racetrack chicane segment
    s = np.linspace(0.0, 100.0, 150)
    x = s
    y = 10.0 / (1.0 + np.exp(-0.2 * (s - 40.0))) - 10.0 / (1.0 + np.exp(-0.2 * (s - 70.0)))
    target_map = np.column_stack((x, y))

    # Perturbation: dx = 0.20m, dy = -0.15m, dtheta = 0.03 rad (~1.7 deg)
    true_dx, true_dy, true_dth = 0.20, -0.15, 0.03
    cos_t, sin_t = math.cos(true_dth), math.sin(true_dth)
    rot = np.array([[cos_t, -sin_t], [sin_t, cos_t]])

    # Generate source scan in local sensor frame
    source_scan = (np.linalg.inv(rot) @ (target_map - np.array([true_dx, true_dy])).T).T

    icp = IterativeClosestPoint2D(max_iterations=40)
    # Provide prior guess near origin
    result = icp.align(source_scan, target_map, initial_pose=np.array([0.0, 0.0, 0.0]))

    assert result.converged
    assert math.isclose(result.estimated_pose[0], true_dx, abs_tol=0.05)
    assert math.isclose(result.estimated_pose[1], true_dy, abs_tol=0.05)
    assert math.isclose(result.estimated_pose[2], true_dth, abs_tol=0.02)


def test_ndt_grid_alignment():
    """Verify 2D NDT builds distribution cells and registers scan against map."""
    # Target map: curved track sector with 200 points
    angles = np.linspace(0.0, 2.0 * math.pi, 200, endpoint=False)
    r = 10.0 + 2.0 * np.sin(3 * angles)
    x = r * np.cos(angles)
    y = r * np.sin(angles)
    target_points = np.column_stack((x, y))

    ndt = NormalDistributionsTransform2D(cell_size=2.0)
    ndt.build_ndt_grid(target_points)
    assert len(ndt.grid_cells) > 0

    # Source scan with slight translation (0.15m, -0.10m)
    true_dx, true_dy = 0.15, -0.10
    source_scan = target_points - np.array([true_dx, true_dy])

    result = ndt.align(source_scan, initial_pose=np.array([0.0, 0.0, 0.0]))

    assert result.converged
    assert result.inlier_ratio > 0.60
    assert math.isclose(result.estimated_pose[0], true_dx, abs_tol=0.05)
    assert math.isclose(result.estimated_pose[1], true_dy, abs_tol=0.05)
