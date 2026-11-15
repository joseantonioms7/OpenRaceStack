"""
Common geometric types, frames, and SE(2) Lie group utilities for autonomous vehicle localization.
"""

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    """Normalize angle(s) in radians to the principal branch [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class SensorType(str, Enum):
    GNSS = "gnss"
    IMU = "imu"
    WHEEL_ODOM = "wheel_odom"
    LIDAR = "lidar"
    CAMERA = "camera"


class FixQuality(str, Enum):
    INVALID = "invalid"
    SINGLE = "single_sps"
    DGPS = "dgps"
    RTK_FLOAT = "rtk_float"
    RTK_FIXED = "rtk_fixed"
    DEAD_RECKONING = "dead_reckoning"


class LocalizationStatus(str, Enum):
    OPTIMAL = "optimal"
    DEGRADED = "degraded"
    CRITICAL = "critical"
    OUTAGE = "outage"


@dataclass
class ExtrinsicsSE2:
    """Rigid 2D planar transform from sensor frame (s) to vehicle reference frame (v): T_v_s."""
    dx: float = 0.0  # Forward offset in vehicle frame (m)
    dy: float = 0.0  # Leftward offset in vehicle frame (m)
    dtheta: float = 0.0  # Yaw boresight rotation in vehicle frame (rad)

    def vehicle_pose_to_sensor_global(self, p_v: np.ndarray, theta_v: float) -> tuple[np.ndarray, float]:
        """Calculates global sensor position and heading given vehicle global pose."""
        c = math.cos(theta_v)
        s = math.sin(theta_v)
        p_sensor_global = np.array([
            p_v[0] + c * self.dx - s * self.dy,
            p_v[1] + s * self.dx + c * self.dy
        ], dtype=np.float64)
        theta_sensor_global = float(wrap_angle(theta_v + self.dtheta))
        return p_sensor_global, theta_sensor_global
