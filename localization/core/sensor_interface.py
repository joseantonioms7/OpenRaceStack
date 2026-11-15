"""
Sensor abstraction layer: defines raw sensor measurement structures, health descriptors,
and hardware timestamps.
"""

from dataclasses import dataclass

import numpy as np

from .types import FixQuality


@dataclass
class GNSSMeasurement:
    """Raw or RTK GNSS observation."""
    timestamp: float
    x: float
    y: float
    z: float = 0.0
    cov_xx: float = 2.25
    cov_yy: float = 2.25
    cov_xy: float = 0.0
    fix_quality: FixQuality = FixQuality.SINGLE
    satellites_used: int = 12
    pdop: float = 1.5
    carrier_to_noise: float = 42.0  # dB-Hz
    course_over_ground: float | None = None
    is_valid: bool = True

    @property
    def covariance_matrix(self) -> np.ndarray:
        return np.array([
            [self.cov_xx, self.cov_xy],
            [self.cov_xy, self.cov_yy]
        ], dtype=np.float64)


@dataclass
class LiDARPoseMeasurement:
    """Relative or global 2D scan-matching pose estimate (e.g. from NDT / ICP)."""
    timestamp: float
    x: float
    y: float
    theta: float
    covariance: np.ndarray  # 3x3 matrix
    fitness_score: float  # Mean squared residual or inlier ratio
    inliers_ratio: float = 0.95
    is_valid: bool = True


@dataclass
class VisualOdometryMeasurement:
    """Visual Odometry relative or absolute pose update."""
    timestamp: float
    x: float
    y: float
    theta: float
    covariance: np.ndarray  # 3x3 matrix
    tracked_features: int = 85
    inlier_ratio: float = 0.88
    is_valid: bool = True

