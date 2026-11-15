"""
High-Performance Scan Matching and Point-Cloud Registration Engine for OpenRaceStack.

Provides 2D/3D Iterative Closest Point (ICP) and Normal Distributions Transform (NDT)
scan registration algorithms, voxel grid downsampling, statistical outlier rejection (SOR),
and analytical Hessian covariance estimation for sub-centimeter localization.
"""

import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from .types import wrap_angle


@dataclass
class ScanMatchResult:
    """Telemetry and diagnostic metrics from scan matching alignment."""
    estimated_pose: np.ndarray    # Registered vehicle pose [x, y, theta]
    covariance: np.ndarray        # 3x3 pose uncertainty covariance matrix from Hessian
    fitness_score: float          # Root-mean-square correspondence residual (m)
    inlier_ratio: float           # Fraction of scan points successfully associated [0.0, 1.0]
    iterations: int               # Number of optimization steps to convergence
    converged: bool               # Convergence flag
    execution_time_ms: float      # Computation time in milliseconds


class VoxelGridFilter:
    """Spatial voxel binning downsampling filter for 2D/3D point clouds."""

    @staticmethod
    def filter(points: np.ndarray, voxel_size: float = 0.25) -> np.ndarray:
        """
        Reduces point density by replacing points within each cubic voxel with their centroid.
        """
        if len(points) == 0:
            return np.empty((0, points.shape[1] if points.ndim > 1 else 2), dtype=np.float64)

        v_size = max(0.01, float(voxel_size))
        # Discrete integer voxel keys
        coords = np.floor(points[:, :2] / v_size).astype(np.int64)

        # Unique voxels and inverse index mapping
        unique_coords, inverse_indices = np.unique(coords, axis=0, return_inverse=True)

        n_voxels = len(unique_coords)
        sum_pts = np.zeros((n_voxels, points.shape[1]), dtype=np.float64)
        counts = np.zeros(n_voxels, dtype=np.int64)

        np.add.at(sum_pts, inverse_indices, points)
        np.add.at(counts, inverse_indices, 1)

        centroids = sum_pts / counts[:, None]
        return centroids


class StatisticalOutlierFilter:
    """Statistical Outlier Rejection (SOR) filter to remove sensor noise and dust."""

    @staticmethod
    def filter(
        points: np.ndarray,
        k_neighbors: int = 8,
        std_ratio: float = 1.5,
    ) -> np.ndarray:
        """
        Computes mean distance to k-nearest neighbors and removes points
        whose distance exceeds mean + std_ratio * std_dev.
        """
        if len(points) <= k_neighbors:
            return points

        tree = cKDTree(points[:, :2])
        # Query k+1 distances (first neighbor is point itself)
        dists, _ = tree.query(points[:, :2], k=k_neighbors + 1)
        mean_dists = np.mean(dists[:, 1:], axis=1)

        mu = float(np.mean(mean_dists))
        sigma = float(np.std(mean_dists))
        thresh = mu + std_ratio * sigma

        mask = mean_dists <= thresh
        return points[mask]


class IterativeClosestPoint2D:
    """
    Point-to-Point Iterative Closest Point (ICP) scan registration.
    Solves optimal SE(2) rigid transform using Singular Value Decomposition (SVD).
    """

    def __init__(
        self,
        max_iterations: int = 30,
        tolerance: float = 1e-4,
        max_correspondence_dist: float = 2.5,
    ):
        self.max_iter = max_iterations
        self.tolerance = tolerance
        self.max_corr_dist = max_correspondence_dist

    def align(
        self,
        source_scan: np.ndarray,
        target_map: np.ndarray,
        initial_pose: np.ndarray,
    ) -> ScanMatchResult:
        """
        Aligns source point cloud (sensor frame) to target point cloud (map frame)
        starting from initial_pose [x, y, theta].
        """
        t0 = time.perf_counter()

        if len(source_scan) < 5 or len(target_map) < 5:
            return ScanMatchResult(
                estimated_pose=initial_pose.copy(),
                covariance=np.diag([10.0, 10.0, 1.0]),
                fitness_score=999.0,
                inlier_ratio=0.0,
                iterations=0,
                converged=False,
                execution_time_ms=0.0,
            )

        target_tree = cKDTree(target_map[:, :2])

        # Current transformation estimate
        cur_x = float(initial_pose[0])
        cur_y = float(initial_pose[1])
        cur_th = float(initial_pose[2])

        converged = False
        final_iter = 0
        fitness = 999.0
        inlier_ratio = 0.0

        for it in range(self.max_iter):
            final_iter = it + 1

            # Transform source scan points into map frame using current pose
            cos_th = math.cos(cur_th)
            sin_th = math.sin(cur_th)
            rot = np.array([[cos_th, -sin_th], [sin_th, cos_th]], dtype=np.float64)

            p_trans = (rot @ source_scan[:, :2].T).T + np.array([cur_x, cur_y])

            # Query closest correspondences in target map
            dists, indices = target_tree.query(p_trans)

            # Filter valid correspondences within max distance
            valid_mask = dists < self.max_corr_dist
            n_inliers = int(np.sum(valid_mask))
            inlier_ratio = n_inliers / len(source_scan)

            if n_inliers < 4:
                break

            matched_src = p_trans[valid_mask]
            matched_tgt = target_map[indices[valid_mask], :2]

            fitness = float(np.sqrt(np.mean(dists[valid_mask] ** 2)))

            # Centroids of matched pairs
            mu_s = np.mean(matched_src, axis=0)
            mu_t = np.mean(matched_tgt, axis=0)

            # Cross-covariance matrix H
            h_mat = (matched_src - mu_s).T @ (matched_tgt - mu_t)

            # SVD decomposition: H = U * S * V^T
            u, _, vt = np.linalg.svd(h_mat)
            r_opt = vt.T @ u.T

            # Reflection correction
            if np.linalg.det(r_opt) < 0:
                vt[1, :] *= -1.0
                r_opt = vt.T @ u.T

            delta_th = math.atan2(r_opt[1, 0], r_opt[0, 0])
            delta_trans = mu_t - r_opt @ mu_s

            # Update pose
            cur_x += float(delta_trans[0])
            cur_y += float(delta_trans[1])
            cur_th = wrap_angle(cur_th + delta_th)

            # Convergence check
            if abs(delta_trans[0]) < self.tolerance and abs(delta_trans[1]) < self.tolerance and abs(delta_th) < self.tolerance:
                converged = True
                break

        # Estimate Hessian / Covariance at convergence
        # H = sum J_i^T J_i where J_i = [1, 0, -sin(th)*x - cos(th)*y; 0, 1, cos(th)*x - sin(th)*y]
        cos_th = math.cos(cur_th)
        sin_th = math.sin(cur_th)
        j_x = -sin_th * source_scan[:, 0] - cos_th * source_scan[:, 1]
        j_y = cos_th * source_scan[:, 0] - sin_th * source_scan[:, 1]

        h_xx = float(len(source_scan))
        h_yy = float(len(source_scan))
        h_tt = float(np.sum(j_x**2 + j_y**2))
        h_xt = float(np.sum(j_x))
        h_yt = float(np.sum(j_y))

        hessian = np.array([
            [h_xx, 0.0, h_xt],
            [0.0, h_yy, h_yt],
            [h_xt, h_yt, h_tt],
        ], dtype=np.float64)

        # Scale by fitness residual variance
        sigma_res = max(0.02, fitness)
        cov = np.linalg.pinv(hessian) * (sigma_res**2)

        exec_ms = (time.perf_counter() - t0) * 1000.0

        return ScanMatchResult(
            estimated_pose=np.array([cur_x, cur_y, cur_th], dtype=np.float64),
            covariance=cov,
            fitness_score=fitness,
            inlier_ratio=inlier_ratio,
            iterations=final_iter,
            converged=converged,
            execution_time_ms=exec_ms,
        )


class NormalDistributionsTransform2D:
    """
    2D Normal Distributions Transform (NDT) scan registration.
    Partitions map into spatial cells, models points as local Gaussian distributions,
    and optimizes scan likelihood via Newton's method with analytical Hessians.
    """

    def __init__(
        self,
        cell_size: float = 1.0,
        max_iterations: int = 25,
        step_tolerance: float = 1e-4,
    ):
        self.cell_size = max(0.1, float(cell_size))
        self.max_iter = max_iterations
        self.step_tol = step_tolerance
        self.grid_cells: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}

    def build_ndt_grid(self, target_points: np.ndarray) -> None:
        """
        Partitions target reference points into cells and estimates
        local mean mu and covariance Sigma for each populated cell.
        """
        self.grid_cells.clear()
        cell_pts: dict[tuple[int, int], list[np.ndarray]] = {}

        for pt in target_points:
            key = (int(math.floor(pt[0] / self.cell_size)), int(math.floor(pt[1] / self.cell_size)))
            cell_pts.setdefault(key, []).append(pt[:2])

        # Compute Gaussian parameters per cell
        for key, pts in cell_pts.items():
            if len(pts) >= 3:
                pts_arr = np.array(pts, dtype=np.float64)
                mu = np.mean(pts_arr, axis=0)
                cov = np.cov(pts_arr, rowvar=False) if len(pts) > 3 else 0.1 * np.eye(2, dtype=np.float64)
                # Regularize covariance to guarantee invertibility and smooth score
                cov_reg = cov + 0.25 * np.eye(2, dtype=np.float64)
                inv_cov = np.linalg.inv(cov_reg)
                self.grid_cells[key] = (mu, inv_cov)

    def align(
        self,
        source_scan: np.ndarray,
        initial_pose: np.ndarray,
    ) -> ScanMatchResult:
        """
        Registers source scan against pre-built NDT grid using Gauss-Newton optimization.
        """
        t0 = time.perf_counter()

        if len(self.grid_cells) == 0 or len(source_scan) < 4:
            return ScanMatchResult(
                estimated_pose=initial_pose.copy(),
                covariance=np.diag([10.0, 10.0, 1.0]),
                fitness_score=999.0,
                inlier_ratio=0.0,
                iterations=0,
                converged=False,
                execution_time_ms=0.0,
            )

        pose = initial_pose.astype(np.float64).copy()
        converged = False
        final_iter = 0
        score = 0.0
        n_matched = 0

        # Gauss-Newton loop
        for it in range(self.max_iter):
            final_iter = it + 1
            cos_th = math.cos(pose[2])
            sin_th = math.sin(pose[2])

            grad = np.zeros(3, dtype=np.float64)
            hessian = np.zeros((3, 3), dtype=np.float64)
            score = 0.0
            n_matched = 0

            for pt in source_scan:
                # Transformed point in map frame
                p_prime = np.array([
                    cos_th * pt[0] - sin_th * pt[1] + pose[0],
                    sin_th * pt[0] + cos_th * pt[1] + pose[1],
                ], dtype=np.float64)

                cell_key = (
                    int(math.floor(p_prime[0] / self.cell_size)),
                    int(math.floor(p_prime[1] / self.cell_size)),
                )

                if cell_key not in self.grid_cells:
                    continue

                mu, inv_cov = self.grid_cells[cell_key]
                diff = p_prime - mu

                # Point likelihood score
                exponent = float(-0.5 * diff @ inv_cov @ diff)
                if exponent < -8.0:
                    continue

                weight = math.exp(exponent)
                score += weight
                n_matched += 1

                # Jacobian d(p') / d(pose)
                # J = [[1, 0, -sin(th)*x - cos(th)*y], [0, 1, cos(th)*x - sin(th)*y]]
                j_mat = np.array([
                    [1.0, 0.0, -sin_th * pt[0] - cos_th * pt[1]],
                    [0.0, 1.0, cos_th * pt[0] - sin_th * pt[1]],
                ], dtype=np.float64)

                # Gradient g = sum weight * J^T * inv_cov * diff
                e_vec = inv_cov @ diff
                grad += weight * (j_mat.T @ e_vec)

                # Gauss-Newton approximation to Hessian: H = sum weight * J^T * inv_cov * J
                hessian += weight * (j_mat.T @ inv_cov @ j_mat)

            if n_matched < 4:
                break

            # Regularize Hessian and solve step (descent step)
            hessian_reg = hessian + 0.1 * np.eye(3, dtype=np.float64)
            try:
                delta_pose = np.linalg.solve(hessian_reg, -grad)
            except np.linalg.LinAlgError:
                break

            pose[:2] += delta_pose[:2]
            pose[2] = wrap_angle(pose[2] + delta_pose[2])

            if np.max(np.abs(delta_pose)) < self.step_tol:
                converged = True
                break

        inlier_ratio = n_matched / len(source_scan) if len(source_scan) > 0 else 0.0
        fitness = 1.0 / (1.0 + score / max(1, n_matched))
        cov = np.linalg.pinv(hessian + 0.05 * np.eye(3)) * 0.1
        exec_ms = (time.perf_counter() - t0) * 1000.0

        return ScanMatchResult(
            estimated_pose=pose,
            covariance=cov,
            fitness_score=fitness,
            inlier_ratio=inlier_ratio,
            iterations=final_iter,
            converged=converged,
            execution_time_ms=exec_ms,
        )
