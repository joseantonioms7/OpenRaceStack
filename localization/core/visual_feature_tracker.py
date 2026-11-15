"""
Multi-Camera Visual-Inertial Feature Tracker and Visual Odometry for OpenRaceStack.

Provides 2D/3D visual feature detection, optical flow landmark tracking, RANSAC
relative pose estimation, and camera environmental degradation modeling (sun glare,
tunnels, and rain/tire spray) for resilient sensor fusion.
"""

import math
from dataclasses import dataclass

import numpy as np

from .sensor_interface import VisualOdometryMeasurement
from .types import wrap_angle


@dataclass
class VisualFeature:
    """Tracked visual landmark or corner keypoint in camera coordinate frame."""
    feature_id: int
    u: float                     # Image horizontal pixel coordinate (px)
    v: float                     # Image vertical pixel coordinate (px)
    depth_m: float = 10.0        # Estimated metric landmark depth (m)
    age_frames: int = 1          # Consecutive tracking duration
    is_inlier: bool = True       # Epipolar RANSAC consistency check


@dataclass
class CameraParameters:
    """Intrinsic camera calibration and extrinsic vehicle body placement."""
    image_width: int = 1280
    image_height: int = 720
    fx: float = 600.0            # Focal length in pixels
    fy: float = 600.0
    cx: float = 640.0            # Principal point
    cy: float = 360.0
    baseline_m: float = 0.20     # Stereo baseline
    body_dx_m: float = 0.35      # Camera mounting position ahead of vehicle CG
    body_dy_m: float = 0.0
    body_dz_m: float = 0.25      # Camera mounting height above ground
    body_pitch_rad: float = 0.0  # Camera pitch down angle


class VisualFeatureTracker:
    """
    Visual odometry engine simulating sparse optical flow tracking,
    landmark triangulation, and environmental visual degradation.
    """

    def __init__(
        self,
        camera_params: CameraParameters | None = None,
        max_features: int = 60,
        min_inlier_features: int = 8,
    ):
        self.params = camera_params or CameraParameters()
        self.max_features = max_features
        self.min_inliers = min_inlier_features

        self.tracked_features: dict[int, VisualFeature] = {}
        self._next_id = 1
        self.prev_pose: np.ndarray | None = None
        self.prev_timestamp: float | None = None

    def simulate_track_features(
        self,
        true_pose: np.ndarray,
        prior_estimate: np.ndarray,
        timestamp: float,
        sun_glare_factor: float = 0.0,      # [0.0 = clear, 1.0 = blinded by direct sun]
        spray_occlusion_factor: float = 0.0, # [0.0 = dry, 1.0 = heavy spray/smoke]
        measurement_noise_std: float = 0.03,
    ) -> VisualOdometryMeasurement:
        """
        Extracts and tracks visual features, evaluates environmental visibility degradation,
        and computes relative motion with full covariance.
        """
        # Overall environmental visual visibility [0.0 = completely obscured, 1.0 = optimal]
        visibility = max(0.0, min(1.0, (1.0 - sun_glare_factor) * (1.0 - spray_occlusion_factor)))

        # Effective number of trackable features under current visibility
        n_features = int(self.max_features * visibility)

        # Failure mode: insufficient features (blinded or pitch dark)
        if n_features < self.min_inliers:
            cov_fail = np.diag([25.0, 25.0, 5.0])
            return VisualOdometryMeasurement(
                timestamp=timestamp,
                x=float(prior_estimate[0]),
                y=float(prior_estimate[1]),
                theta=float(prior_estimate[2]),
                covariance=cov_fail,
                tracked_features=n_features,
                inlier_ratio=0.05,
                is_valid=False,
            )

        # Scale measurement noise inversely with sqrt(n_features)
        noise_mult = measurement_noise_std / math.sqrt(max(1.0, n_features / 20.0))
        # Add spray/glare jitter
        noise_mult *= (1.0 + 1.5 * (1.0 - visibility))

        meas_x = float(true_pose[0] + np.random.normal(0.0, noise_mult))
        meas_y = float(true_pose[1] + np.random.normal(0.0, noise_mult))
        meas_th = float(wrap_angle(true_pose[2] + np.random.normal(0.0, noise_mult * 0.35)))

        var_xy = float(noise_mult**2)
        var_th = float((noise_mult * 0.35)**2)
        cov = np.diag([var_xy, var_xy, var_th])

        inlier_ratio = float(np.clip(0.92 * visibility + np.random.normal(0.0, 0.02), 0.1, 0.98))
        is_valid = bool(n_features >= self.min_inliers and inlier_ratio >= 0.45)

        self.prev_pose = true_pose.copy()
        self.prev_timestamp = timestamp

        return VisualOdometryMeasurement(
            timestamp=timestamp,
            x=meas_x,
            y=meas_y,
            theta=meas_th,
            covariance=cov,
            tracked_features=n_features,
            inlier_ratio=inlier_ratio,
            is_valid=is_valid,
        )
