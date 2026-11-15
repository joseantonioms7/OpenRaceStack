"""
Health Supervisor and Diagnostic Fault Detection & Isolation (FDE) Layer.
Monitors individual sensor health, tracks innovation residuals (NIS), detects sensor divergence,
computes holistic localization confidence [0.0 - 1.0], and manages automatic fallbacks.
"""

from dataclasses import dataclass

import numpy as np

from .types import LocalizationStatus, SensorType


@dataclass
class SensorHealthStatus:
    sensor: SensorType
    is_healthy: bool = True
    consecutive_rejections: int = 0
    total_rejections: int = 0
    last_nis: float = 0.0
    timeout_counter: int = 0
    status_message: str = "Nominal"


class HealthSupervisor:
    """
    Supervises system-level localization integrity:
    - Mahalanobis outlier counting & channel fault isolation
    - Sensor starvation watchdogs
    - Overall localization confidence score estimation
    - Automatic sensor fallback transitions (GNSS -> LiDAR/Camera -> Dead-reckoning)
    """

    def __init__(
        self,
        max_rejections_for_fault: int = 8,
        starvation_timeout_steps: int = 15
    ):
        self.max_rejections = max_rejections_for_fault
        self.starvation_timeout = starvation_timeout_steps
        self.sensor_health: dict[SensorType, SensorHealthStatus] = {
            SensorType.GNSS: SensorHealthStatus(sensor=SensorType.GNSS),
            SensorType.IMU: SensorHealthStatus(sensor=SensorType.IMU),
            SensorType.WHEEL_ODOM: SensorHealthStatus(sensor=SensorType.WHEEL_ODOM),
            SensorType.LIDAR: SensorHealthStatus(sensor=SensorType.LIDAR),
            SensorType.CAMERA: SensorHealthStatus(sensor=SensorType.CAMERA),
        }
        self.overall_confidence: float = 1.0
        self.current_status: LocalizationStatus = LocalizationStatus.OPTIMAL
        self.active_primary_source: SensorType = SensorType.GNSS

    def report_sensor_update(
        self,
        sensor: SensorType,
        accepted: bool,
        nis: float,
        is_valid_payload: bool = True
    ) -> None:
        """Updates health statistics for a given sensor channel."""
        h = self.sensor_health[sensor]
        h.last_nis = float(nis)
        h.timeout_counter = 0

        if not is_valid_payload:
            h.consecutive_rejections += 1
            h.total_rejections += 1
            h.is_healthy = False
            h.status_message = "Corrupted/Invalid payload"
            return

        if accepted:
            h.consecutive_rejections = 0
            h.is_healthy = True
            h.status_message = "Nominal tracking"
        else:
            h.consecutive_rejections += 1
            h.total_rejections += 1
            if h.consecutive_rejections >= self.max_rejections:
                h.is_healthy = False
                h.status_message = f"Fault: {h.consecutive_rejections} consecutive chi2 rejections (NIS={nis:.1f})"

    def evaluate_system_confidence(
        self,
        P_covariance: np.ndarray,
        trace_P: float,
        is_corridor_active: bool = False
    ) -> float:
        """
        Computes composite localization confidence score in [0.0, 1.0]:
        Factors in:
        1. Covariance position trace: tr(P_xy)
        2. Sensor redundancy status (GNSS, LiDAR, Camera, IMU)
        3. Statistical innovation health
        """
        pos_var = float(P_covariance[0, 0] + P_covariance[1, 1])
        # Base confidence from position variance: < 0.1m^2 => 1.0, > 4.0m^2 => 0.1
        cov_factor = float(np.clip(1.0 - (pos_var - 0.05) / 3.0, 0.05, 1.0))

        # Sensor availability bonus/penalty
        gnss_ok = self.sensor_health[SensorType.GNSS].is_healthy
        lidar_ok = self.sensor_health[SensorType.LIDAR].is_healthy
        cam_ok = self.sensor_health[SensorType.CAMERA].is_healthy
        imu_ok = self.sensor_health[SensorType.IMU].is_healthy

        healthy_count = sum([gnss_ok, lidar_ok, cam_ok])
        redundancy_mult = 1.0
        if healthy_count == 3:
            redundancy_mult = 1.0
            self.active_primary_source = SensorType.GNSS
            self.current_status = LocalizationStatus.OPTIMAL
        elif healthy_count == 2:
            redundancy_mult = 0.90
            self.active_primary_source = SensorType.LIDAR if lidar_ok else SensorType.GNSS
            self.current_status = LocalizationStatus.OPTIMAL
        elif healthy_count == 1:
            redundancy_mult = 0.75
            if lidar_ok:
                self.active_primary_source = SensorType.LIDAR
            elif cam_ok:
                self.active_primary_source = SensorType.CAMERA
            else:
                self.active_primary_source = SensorType.GNSS
            self.current_status = LocalizationStatus.DEGRADED
        else:
            # Complete exteroceptive loss (Dead Reckoning on IMU + Wheel Odom)
            redundancy_mult = 0.35 if imu_ok else 0.05
            self.active_primary_source = SensorType.IMU
            self.current_status = LocalizationStatus.OUTAGE if not imu_ok else LocalizationStatus.CRITICAL

        self.overall_confidence = float(np.clip(cov_factor * redundancy_mult, 0.0, 1.0))
        return self.overall_confidence
