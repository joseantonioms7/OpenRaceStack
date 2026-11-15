"""
LiDAR Localizer: Simulates multi-beam planar LiDAR range measurements and performs
Local Map Alignment / Scan Matching against the Distance Field and Occupancy Grid.
Extracts Hessian curvature to evaluate degeneracy and estimate localization covariance.
"""

import math

import numpy as np

from .map_manager import MapManager
from .sensor_interface import LiDARPoseMeasurement
from .types import ExtrinsicsSE2, wrap_angle


class LiDARLocalizer:
    """
    Planar scan matcher simulating 2D LiDAR range beams and performing
    Nonlinear Gauss-Newton / Levenberg-Marquardt Point-to-Distance-Field scan matching.
    """

    def __init__(
        self,
        map_manager: MapManager,
        extrinsics: ExtrinsicsSE2 | None = None,
        num_beams: int = 64,
        max_range: float = 20.0,
        fov_rad: float = 2.0 * math.pi * (270.0 / 360.0)  # 270 deg FOV
    ):
        self.map_mgr = map_manager
        self.extrinsics = extrinsics if extrinsics is not None else ExtrinsicsSE2(dx=0.25, dy=0.0, dtheta=0.0)
        self.num_beams = num_beams
        self.max_range = max_range
        self.fov_rad = fov_rad
        self.beam_angles = np.linspace(-fov_rad / 2.0, fov_rad / 2.0, num_beams)

    def simulate_scan(self, true_pose: np.ndarray, noise_std: float = 0.03) -> np.ndarray:
        """
        Raycasts against the map's distance field / obstacle boundary.
        Returns point cloud in sensor frame: array of shape (K, 2).
        """
        p_sensor_global, th_sensor_global = self.extrinsics.vehicle_pose_to_sensor_global(true_pose[:2], true_pose[2])
        points_s = []

        for angle in self.beam_angles:
            global_ray_angle = th_sensor_global + angle
            c_ray = math.cos(global_ray_angle)
            s_ray = math.sin(global_ray_angle)

            # Ray marching along the distance field
            dist = 0.1
            hit = False
            for _ in range(35):
                rx = p_sensor_global[0] + dist * c_ray
                ry = p_sensor_global[1] + dist * s_ray
                d_obs = self.map_mgr.get_obstacle_distance(rx, ry)
                if d_obs <= 0.08:  # Hit surface
                    hit = True
                    break
                dist += max(d_obs, 0.05)
                if dist >= self.max_range:
                    break

            if hit and dist < self.max_range:
                noisy_dist = dist + np.random.normal(0.0, noise_std)
                # Point in sensor frame
                px_s = noisy_dist * math.cos(angle)
                py_s = noisy_dist * math.sin(angle)
                points_s.append([px_s, py_s])

        if len(points_s) < 8:
            return np.empty((0, 2), dtype=np.float64)
        return np.array(points_s, dtype=np.float64)

    def align_scan(
        self,
        scan_points_s: np.ndarray,
        prior_vehicle_pose: np.ndarray,
        timestamp: float
    ) -> LiDARPoseMeasurement:
        """
        Gauss-Newton registration minimizing residual distances to mapped obstacle walls.
        Outputs global estimated vehicle pose, covariance estimate from Hessian, and inlier fitness.
        """
        if len(scan_points_s) < 10 or self.map_mgr.distance_field is None:
            # Degenerate scan
            return LiDARPoseMeasurement(
                timestamp=timestamp,
                x=prior_vehicle_pose[0],
                y=prior_vehicle_pose[1],
                theta=prior_vehicle_pose[2],
                covariance=np.diag([10.0, 10.0, 1.0]),
                fitness_score=999.0,
                inliers_ratio=0.0,
                is_valid=False
            )

        pose = prior_vehicle_pose.copy()
        # Ensure heading is wrapped
        pose[2] = wrap_angle(pose[2])

        H_mat = np.zeros((3, 3), dtype=np.float64)
        best_pose = pose.copy()
        best_cost = float('inf')

        for _iter_idx in range(6):
            c_v = math.cos(pose[2])
            s_v = math.sin(pose[2])

            # Sensor position in global frame
            p_s_global = np.array([
                pose[0] + c_v * self.extrinsics.dx - s_v * self.extrinsics.dy,
                pose[1] + s_v * self.extrinsics.dx + c_v * self.extrinsics.dy
            ])
            th_s_global = wrap_angle(pose[2] + self.extrinsics.dtheta)
            c_s = math.cos(th_s_global)
            s_s = math.sin(th_s_global)

            # Transform sensor points to global metric frame
            pts_global_x = p_s_global[0] + c_s * scan_points_s[:, 0] - s_s * scan_points_s[:, 1]
            pts_global_y = p_s_global[1] + s_s * scan_points_s[:, 0] + c_s * scan_points_s[:, 1]

            residuals = []
            J_rows = []

            eps = 0.04
            for i in range(len(scan_points_s)):
                gx = pts_global_x[i]
                gy = pts_global_y[i]
                d = self.map_mgr.get_obstacle_distance(gx, gy)
                if d > 1.2:  # Outlier / dynamic point
                    continue
                residuals.append(d)

                # Numerical gradient of distance field
                dx_p = self.map_mgr.get_obstacle_distance(gx + eps, gy)
                dx_m = self.map_mgr.get_obstacle_distance(gx - eps, gy)
                dy_p = self.map_mgr.get_obstacle_distance(gx, gy + eps)
                dy_m = self.map_mgr.get_obstacle_distance(gx, gy - eps)

                grad_x = (dx_p - dx_m) / (2.0 * eps)
                grad_y = (dy_p - dy_m) / (2.0 * eps)

                # Jacobian w.r.t vehicle pose [x, y, theta]
                d_theta = (
                    grad_x * (-s_s * scan_points_s[i, 0] - c_s * scan_points_s[i, 1] - s_v * self.extrinsics.dx - c_v * self.extrinsics.dy) +
                    grad_y * ( c_s * scan_points_s[i, 0] - s_s * scan_points_s[i, 1] + c_v * self.extrinsics.dx - s_v * self.extrinsics.dy)
                )
                J_rows.append([grad_x, grad_y, d_theta])

            if len(residuals) < 8:
                break

            res_arr = np.array(residuals, dtype=np.float64)
            J_arr = np.array(J_rows, dtype=np.float64)

            cost = float(np.mean(res_arr**2))
            if cost < best_cost:
                best_cost = cost
                best_pose = pose.copy()

            # Normal equations: H delta = -g
            H_mat = J_arr.T @ J_arr + np.eye(3) * 0.2
            g = J_arr.T @ res_arr

            try:
                delta = np.linalg.solve(H_mat, -g)
                # Damp update step
                delta[0:2] = np.clip(delta[0:2], -0.2, 0.2)
                delta[2] = np.clip(delta[2], -0.1, 0.1)
                pose += delta
                pose[2] = wrap_angle(pose[2])
            except np.linalg.LinAlgError:
                break

        # Compute empirical covariance from inverse Hessian curvature
        try:
            cov = np.linalg.inv(H_mat) * (best_cost + 1e-4)
            # Inflation for safety
            cov = 0.5 * (cov + cov.T) + np.diag([0.04**2, 0.04**2, 0.01**2])
        except np.linalg.LinAlgError:
            cov = np.diag([0.25, 0.25, 0.04])

        fitness = best_cost
        inliers_ratio = float(len(residuals) / len(scan_points_s))
        is_valid = fitness < 0.25 and inliers_ratio > 0.40

        return LiDARPoseMeasurement(
            timestamp=timestamp,
            x=float(best_pose[0]),
            y=float(best_pose[1],),
            theta=float(wrap_angle(best_pose[2])),
            covariance=cov,
            fitness_score=fitness,
            inliers_ratio=inliers_ratio,
            is_valid=is_valid
        )
