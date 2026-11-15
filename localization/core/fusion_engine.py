"""
Multi-Sensor Fusion Orchestrator and Unified State Estimator:
Integrates GNSS, IMU, Wheel Odometry, LiDAR Scan Matching, Visual Odometry,
and Map Prior Corridor Constraints into an autonomous driving localization platform.
"""

import math
from typing import Any

import numpy as np

from .camera_localizer import CameraLocalizer
from .ekf_engine import LocalizationEKF
from .health_supervisor import HealthSupervisor
from .lidar_localizer import LiDARLocalizer
from .map_manager import MapManager
from .sensor_interface import GNSSMeasurement
from .types import ExtrinsicsSE2, SensorType


class MultiSensorFusionEngine:
    """
    Principal Autonomous Vehicle Localization & Sensor Fusion Engine.
    Coordinates high-rate prediction, multi-sensor correction, map alignment,
    fault tolerance, and telemetry generation.
    """

    def __init__(
        self,
        map_manager: MapManager,
        dt_base: float = 0.05,
        gnss_extrinsics: ExtrinsicsSE2 | None = None,
        lidar_extrinsics: ExtrinsicsSE2 | None = None,
        camera_extrinsics: ExtrinsicsSE2 | None = None,
    ):
        self.map_mgr = map_manager
        self.dt_base = dt_base

        # Sensor extrinsics relative to vehicle rear-axle center
        self.gnss_extrinsics = gnss_extrinsics if gnss_extrinsics is not None else ExtrinsicsSE2(dx=-0.1, dy=0.0, dtheta=0.0)
        self.lidar_extrinsics = lidar_extrinsics if lidar_extrinsics is not None else ExtrinsicsSE2(dx=0.25, dy=0.0, dtheta=0.0)
        self.camera_extrinsics = camera_extrinsics if camera_extrinsics is not None else ExtrinsicsSE2(dx=0.35, dy=0.0, dtheta=0.0)

        # Subsystems
        self.lidar_localizer = LiDARLocalizer(self.map_mgr, extrinsics=self.lidar_extrinsics)
        self.camera_localizer = CameraLocalizer(self.map_mgr, extrinsics=self.camera_extrinsics)
        self.health_supervisor = HealthSupervisor()

        # Core EKF Estimator
        # [px, py, theta, v, omega]
        self.ekf = LocalizationEKF(
            dt=dt_base,
            Q_diag=[0.008, 0.008, 0.002, 0.06, 2.0],
            R_diag=[2.25, 2.25],
            sigma_a=0.6,
            sigma_alpha=4.0,
            use_dynamic_Q=True,
            gate_threshold=16.266
        )

        self.last_update_timestamp: float = 0.0
        self.step_count: int = 0

    def initialize_pose(self, x0: float, y0: float, theta0: float, v0: float = 0.0, omega0: float = 0.0, timestamp: float = 0.0):
        """Initializes vehicle pose and covariance at starting line or relocalization fix."""
        init_state = np.array([[x0], [y0], [theta0], [v0], [omega0]], dtype=np.float64)
        P0 = np.diag([0.15, 0.15, 0.015, 0.06, 0.06])
        self.ekf.initialize_state(init_state, P0=P0, timestamp=timestamp)
        self.last_update_timestamp = timestamp

    def predict_forward(self, dt: float) -> None:
        """Propagates state forward using CTRV motion model."""
        self.ekf.predict(dt)
        self.last_update_timestamp += dt
        self.step_count += 1

    def fuse_gnss(self, gnss: GNSSMeasurement) -> bool:
        """
        Fuses GNSS measurement with antenna lever-arm correction and dynamic receiver covariance.
        p_gnss = p_v + R(theta) * l_gnss => p_v_obs = p_gnss - R(theta) * l_gnss
        """
        if not gnss.is_valid:
            self.health_supervisor.report_sensor_update(SensorType.GNSS, accepted=False, nis=0.0, is_valid_payload=False)
            return False

        # Lever-arm translation from GNSS antenna to vehicle reference point
        theta_v = float(self.ekf.x[2, 0])
        c = math.cos(theta_v)
        s = math.sin(theta_v)
        dx_v = gnss.x - (c * self.gnss_extrinsics.dx - s * self.gnss_extrinsics.dy)
        dy_v = gnss.y - (s * self.gnss_extrinsics.dx + c * self.gnss_extrinsics.dy)

        # Dynamic covariance adjustment based on fix quality and satellite geometry (PDOP)
        scale_dop = max(gnss.pdop / 1.5, 1.0)
        cov_mat = gnss.covariance_matrix * (scale_dop**2)

        if gnss.course_over_ground is not None and abs(float(self.ekf.x[3, 0])) > 1.2:
            # Fuse full 3D pose [x, y, course_over_ground]
            z_pose = np.array([[dx_v], [dy_v], [gnss.course_over_ground]], dtype=np.float64)
            R_pose = np.zeros((3, 3), dtype=np.float64)
            R_pose[0:2, 0:2] = cov_mat
            R_pose[2, 2] = 0.05**2
            accepted = self.ekf.update_pose(z_pose, R_pose=R_pose, gate_threshold=16.266)
        else:
            # Fuse 2D position
            z_pos = np.array([[dx_v], [dy_v]], dtype=np.float64)
            accepted = self.ekf.update_gps(z_pos, R_gps=cov_mat, gate_threshold=13.8155)

        nis = self.ekf.last_nis
        self.health_supervisor.report_sensor_update(SensorType.GNSS, accepted=accepted, nis=nis, is_valid_payload=True)
        return accepted

    def fuse_imu_and_odometry(self, v_wheel: float, omega_imu: float, var_v: float = 0.08**2, var_omega: float = 0.02**2) -> bool:
        """Fuses high-rate wheel odometry and IMU angular rate."""
        z_twist = np.array([[v_wheel], [omega_imu]], dtype=np.float64)
        R_twist = np.diag([var_v, var_omega])
        accepted = self.ekf.update_imu_odom(z_twist, R_prop=R_twist, gate_threshold=25.0)

        nis = self.ekf.last_nis
        self.health_supervisor.report_sensor_update(SensorType.IMU, accepted=accepted, nis=nis, is_valid_payload=True)
        self.health_supervisor.report_sensor_update(SensorType.WHEEL_ODOM, accepted=accepted, nis=nis, is_valid_payload=True)
        return accepted

    def fuse_lidar_scan(self, lidar_scan_s: np.ndarray, timestamp: float) -> bool:
        """Registers LiDAR scan against map distance field and fuses resultant SE(2) pose."""
        prior_pose = np.array([self.ekf.x[0, 0], self.ekf.x[1, 0], self.ekf.x[2, 0]], dtype=np.float64)
        meas = self.lidar_localizer.align_scan(lidar_scan_s, prior_pose, timestamp)

        if not meas.is_valid:
            self.health_supervisor.report_sensor_update(SensorType.LIDAR, accepted=False, nis=meas.fitness_score, is_valid_payload=False)
            return False

        z_pose = np.array([[meas.x], [meas.y], [meas.theta]], dtype=np.float64)
        accepted = self.ekf.update_pose(z_pose, R_pose=meas.covariance, gate_threshold=16.266)
        nis = self.ekf.last_nis
        self.health_supervisor.report_sensor_update(SensorType.LIDAR, accepted=accepted, nis=nis, is_valid_payload=True)
        return accepted

    def fuse_camera_vision(self, true_pose_for_sim: np.ndarray, timestamp: float, lighting_degradation: float = 0.0) -> bool:
        """Simulates visual feature tracking and fuses visual relative/absolute pose."""
        prior_pose = np.array([self.ekf.x[0, 0], self.ekf.x[1, 0], self.ekf.x[2, 0]], dtype=np.float64)
        v_meas = self.camera_localizer.estimate_visual_pose(true_pose_for_sim, prior_pose, timestamp, lighting_degradation=lighting_degradation)

        if not v_meas.is_valid:
            self.health_supervisor.report_sensor_update(SensorType.CAMERA, accepted=False, nis=0.0, is_valid_payload=False)
            return False

        z_pose = np.array([[v_meas.x], [v_meas.y], [v_meas.theta]], dtype=np.float64)
        accepted = self.ekf.update_pose(z_pose, R_pose=v_meas.covariance, gate_threshold=16.266)
        nis = self.ekf.last_nis
        self.health_supervisor.report_sensor_update(SensorType.CAMERA, accepted=accepted, nis=nis, is_valid_payload=True)
        return accepted

    def apply_corridor_boundary_constraint(self) -> bool:
        """Applies physical corridor constraint using the MapManager's KD-tree."""
        x_v = float(self.ekf.x[0, 0])
        y_v = float(self.ekf.x[1, 0])
        cross_track, w_l, w_r, cl_pt = self.map_mgr.query_corridor_bounds(x_v, y_v)
        th_ref = float(self.ekf.x[2, 0])
        return self.ekf.constrain_to_corridor(cl_pt, th_ref, w_r, w_l)

    def get_telemetry_snapshot(self) -> dict[str, Any]:
        """Exposes standard autonomous vehicle localization telemetry (suitable for REST/FastAPI/WebSockets)."""
        tr_p = float(np.trace(self.ekf.P))
        confidence = self.health_supervisor.evaluate_system_confidence(self.ekf.P, tr_p)

        var_xy = float(self.ekf.P[0, 0] + self.ekf.P[1, 1])
        std_xy = math.sqrt(max(var_xy * 0.5, 1e-9))
        v_mps = float(self.ekf.x[3, 0])
        omega_radps = float(self.ekf.x[4, 0])
        ay_mps2 = v_mps * omega_radps

        return {
            "timestamp": self.last_update_timestamp,
            "step": self.step_count,
            "pose": {
                "x": float(self.ekf.x[0, 0]),
                "y": float(self.ekf.x[1, 0]),
                "theta_rad": float(self.ekf.x[2, 0]),
                "theta_deg": float(np.degrees(self.ekf.x[2, 0])),
            },
            "velocity": {
                "linear_mps": v_mps,
                "angular_radps": omega_radps,
            },
            "dynamics": {
                "ay_mps2": ay_mps2,
                "ay_g": float(ay_mps2 / 9.81),
            },
            "covariance": {
                "var_x": float(self.ekf.P[0, 0]),
                "var_y": float(self.ekf.P[1, 1]),
                "var_theta": float(self.ekf.P[2, 2]),
                "var_v": float(self.ekf.P[3, 3]),
                "var_omega": float(self.ekf.P[4, 4]),
                "trace": tr_p,
            },
            "safety_integrity": {
                "hpl_m": float(6.0 * std_xy),
                "cep50_est_m": float(1.1774 * std_xy),
                "cep95_est_m": float(2.4477 * std_xy),
            },
            "confidence": confidence,
            "status": self.health_supervisor.current_status.value,
            "active_source": self.health_supervisor.active_primary_source.value,
            "sensor_health": {
                s.value: {
                    "healthy": h.is_healthy,
                    "consecutive_rejections": h.consecutive_rejections,
                    "last_nis": h.last_nis,
                    "status": h.status_message
                }
                for s, h in self.health_supervisor.sensor_health.items()
            }
        }
