"""
Visual Odometry and Camera Localization Layer:
Simulates keypoint feature tracking, landmark triangulation, visual relative pose estimation,
and landmark-based visual relocalization.
"""

import math

import numpy as np

from .map_manager import MapManager
from .sensor_interface import VisualOdometryMeasurement
from .types import ExtrinsicsSE2, wrap_angle


class CameraLocalizer:
    """
    Visual odometry & landmark tracking simulator and localizer.
    Tracks visual features on circuit curbs, barriers, and visual markers.
    """

    def __init__(
        self,
        map_manager: MapManager,
        extrinsics: ExtrinsicsSE2 | None = None,
        num_features: int = 40,
        fov_rad: float = math.radians(90.0),
        max_visual_range: float = 25.0
    ):
        self.map_mgr = map_manager
        self.extrinsics = extrinsics if extrinsics is not None else ExtrinsicsSE2(dx=0.35, dy=0.0, dtheta=0.0)
        self.num_features = num_features
        self.fov_rad = fov_rad
        self.max_visual_range = max_visual_range
        self.prev_pose: np.ndarray | None = None
        self.prev_time: float | None = None

    def estimate_visual_pose(
        self,
        true_pose: np.ndarray,
        prior_estimate: np.ndarray,
        timestamp: float,
        lighting_degradation: float = 0.0,  # 0.0 = clear daylight, 1.0 = pitch dark / blinded
        feature_noise_std: float = 0.04
    ) -> VisualOdometryMeasurement:
        """
        Calculates visual relative motion and absolute landmark corrections.
        When lighting_degradation is high (tunnels, blinding glare), feature tracking degrades gracefully.
        """
        # If camera is blinded or in darkness
        effective_features = int(self.num_features * (1.0 - np.clip(lighting_degradation, 0.0, 1.0)))

        if effective_features < 8:
            # Degraded / failed vision channel
            return VisualOdometryMeasurement(
                timestamp=timestamp,
                x=float(prior_estimate[0]),
                y=float(prior_estimate[1]),
                theta=float(prior_estimate[2]),
                covariance=np.diag([20.0, 20.0, 5.0]),
                tracked_features=effective_features,
                inlier_ratio=0.1,
                is_valid=False
            )

        # Feature noise inversely proportional to sqrt(tracked_features)
        noise_scale = feature_noise_std / math.sqrt(max(effective_features, 1) / 30.0)
        meas_x = float(true_pose[0] + np.random.normal(0.0, noise_scale))
        meas_y = float(true_pose[1] + np.random.normal(0.0, noise_scale))
        meas_th = float(wrap_angle(true_pose[2] + np.random.normal(0.0, noise_scale * 0.4)))

        # Dynamic visual covariance
        var_xy = float(noise_scale**2)
        var_th = float((noise_scale * 0.4)**2)
        cov = np.diag([var_xy, var_xy, var_th])

        inlier_ratio = float(np.clip(0.95 - lighting_degradation * 0.5 + np.random.normal(0.0, 0.03), 0.2, 0.99))
        is_valid = effective_features >= 12 and inlier_ratio >= 0.50

        return VisualOdometryMeasurement(
            timestamp=timestamp,
            x=meas_x,
            y=meas_y,
            theta=meas_th,
            covariance=cov,
            tracked_features=effective_features,
            inlier_ratio=inlier_ratio,
            is_valid=is_valid
        )
