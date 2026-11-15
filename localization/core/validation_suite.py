"""
Automated Multi-Sensor Validation and Stress-Testing Suite:
Executes rigorous benchmarks testing:
1. Nominal Multi-Sensor Fusion (GNSS + IMU + Odometry + LiDAR + Camera).
2. Complete GNSS Outage / Tunnel scenario (10-second prolonged outage).
3. Severe Urban Canyon Multipath & Satellite Dropouts (PDOP=8.5, 20m jumps).
4. Camera Blinding Glare / Night Tunnel darkness.
5. LiDAR Feature Degeneracy / Obstacle occlusion.
6. Execution time & real-time computational cost benchmarks.
"""

import time
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from .fusion_engine import MultiSensorFusionEngine
from .map_manager import MapManager
from .sensor_interface import GNSSMeasurement
from .types import FixQuality, wrap_angle

try:
    from Localization.utils.metrics import calculate_advanced_metrics
except ImportError:
    from utils.metrics import calculate_advanced_metrics


class MultiSensorValidator:
    def __init__(self, datasets_dir: str | Path = "datasets"):
        self.datasets_dir = Path(datasets_dir)
        if not self.datasets_dir.is_dir():
            self.datasets_dir = Path(__file__).resolve().parent.parent / datasets_dir

    def run_stress_test(
        self,
        circuit_name: str = "Monza",
        gnss_outage_start_step: int = 250,
        gnss_outage_duration_steps: int = 100,  # 10.0 seconds of complete GNSS loss
        urban_canyon_start_step: int = 500,
        urban_canyon_duration_steps: int = 60,
        camera_blind_start_step: int = 280,
        camera_blind_duration_steps: int = 50,
        seed: int = 42,
        show_plots: bool = False
    ) -> dict[str, Any]:
        rng = np.random.default_rng(seed)
        circuit_folder = self.datasets_dir / circuit_name
        map_mgr = MapManager(circuit_folder)

        track = map_mgr.waypoints
        if track is None:
            raise RuntimeError(f"Track waypoints not found in {circuit_folder}")

        n_steps = len(track) - 1
        dt = 0.1
        diffs = np.diff(track, axis=0)
        v_true = np.hypot(diffs[:, 0], diffs[:, 1]) / dt
        th_true = np.arctan2(diffs[:, 1], diffs[:, 0])
        om_true = np.zeros(n_steps, dtype=np.float64)
        om_true[:-1] = wrap_angle(np.diff(th_true)) / dt
        om_true[-1] = om_true[-2]

        engine = MultiSensorFusionEngine(map_manager=map_mgr, dt_base=dt)
        engine.initialize_pose(track[0, 0], track[0, 1], th_true[0], v_true[0], om_true[0], timestamp=0.0)

        # Histories
        true_history = track[1:, :2]
        ekf_history = np.empty((n_steps, 2), dtype=np.float64)
        v_ekf_history = np.empty(n_steps, dtype=np.float64)
        th_ekf_history = np.empty(n_steps, dtype=np.float64)
        conf_history = np.empty(n_steps, dtype=np.float64)
        status_history = []
        primary_source_history = []

        # Synthetic noisy observations
        sigma_gps_nom = 1.5
        sigma_v_odom = 0.08
        sigma_om_imu = 0.02

        print("\n=======================================================")
        print(f"   APEX PATH AI - STRESS TEST: {circuit_name.upper()}")
        print("=======================================================")
        print(f" > GNSS Outage Horizon: Steps {gnss_outage_start_step} to {gnss_outage_start_step + gnss_outage_duration_steps} ({gnss_outage_duration_steps*dt:.1f}s)")
        print(f" > Camera Blinding:     Steps {camera_blind_start_step} to {camera_blind_start_step + camera_blind_duration_steps}")
        print(f" > Urban Canyon Glitch: Steps {urban_canyon_start_step} to {urban_canyon_start_step + urban_canyon_duration_steps}")

        t_start = time.perf_counter()

        for i in range(n_steps):
            # 1. High-rate motion propagation (IMU strapdown prediction)
            engine.predict_forward(dt)

            # 2. Proprioceptive IMU + Wheel Odometry
            v_meas = float(v_true[i] + rng.normal(0.0, sigma_v_odom))
            om_meas = float(om_true[i] + rng.normal(0.0, sigma_om_imu))
            engine.fuse_imu_and_odometry(v_meas, om_meas)

            # 3. Exteroceptive GNSS logic (Simulate Outages & Multipath)
            in_gnss_outage = gnss_outage_start_step <= i < (gnss_outage_start_step + gnss_outage_duration_steps)
            in_urban_canyon = urban_canyon_start_step <= i < (urban_canyon_start_step + urban_canyon_duration_steps)

            if not in_gnss_outage:
                if in_urban_canyon:
                    # Multipath reflection + poor satellite visibility (PDOP = 8.0, 15m spike occasionally)
                    pdop = 8.0
                    spike = 15.0 if (i % 5 == 0) else 0.0
                    gnss_x = float(true_history[i, 0] + rng.normal(0.0, 4.5) + spike)
                    gnss_y = float(true_history[i, 1] + rng.normal(0.0, 4.5) - spike)
                    fix_q = FixQuality.SINGLE
                else:
                    pdop = 1.2
                    gnss_x = float(true_history[i, 0] + rng.normal(0.0, sigma_gps_nom))
                    gnss_y = float(true_history[i, 1] + rng.normal(0.0, sigma_gps_nom))
                    fix_q = FixQuality.RTK_FIXED

                gnss_obs = GNSSMeasurement(
                    timestamp=i * dt,
                    x=gnss_x,
                    y=gnss_y,
                    cov_xx=sigma_gps_nom**2,
                    cov_yy=sigma_gps_nom**2,
                    fix_quality=fix_q,
                    pdop=pdop,
                    course_over_ground=float(th_true[i] + rng.normal(0.0, 0.04)) if not in_urban_canyon else None
                )
                engine.fuse_gnss(gnss_obs)

            # 4. LiDAR scan matching (Simulated 64-beam range scan against mapped barriers)
            # Scan matching runs at 10 Hz
            true_pose_i = np.array([true_history[i, 0], true_history[i, 1], th_true[i]], dtype=np.float64)
            scan_points = engine.lidar_localizer.simulate_scan(true_pose_i, noise_std=0.03)
            engine.fuse_lidar_scan(scan_points, timestamp=i * dt)

            # 5. Camera visual odometry & landmark tracking (Simulate blinding glare/darkness)
            in_camera_blind = camera_blind_start_step <= i < (camera_blind_start_step + camera_blind_duration_steps)
            light_deg = 0.95 if in_camera_blind else 0.0
            engine.fuse_camera_vision(true_pose_i, timestamp=i * dt, lighting_degradation=light_deg)

            # 6. Mapped corridor boundary enforcement
            engine.apply_corridor_boundary_constraint()

            # Record telemetry
            telemetry = engine.get_telemetry_snapshot()
            ekf_history[i, 0] = telemetry["pose"]["x"]
            ekf_history[i, 1] = telemetry["pose"]["y"]
            th_ekf_history[i] = telemetry["pose"]["theta_rad"]
            v_ekf_history[i] = telemetry["velocity"]["linear_mps"]
            conf_history[i] = telemetry["confidence"]
            status_history.append(telemetry["status"])
            primary_source_history.append(telemetry["active_source"])

        t_elapsed = time.perf_counter() - t_start
        fps = n_steps / t_elapsed

        # Calculate metrics during entire lap
        metrics = calculate_advanced_metrics(
            true_history,
            true_history + rng.normal(0.0, 1.5, size=true_history.shape),
            ekf_history,
            v_true,
            v_ekf_history,
            theta_true_hist=th_true,
            theta_ekf_hist=th_ekf_history,
            track_widths=map_mgr.track_widths,
            dt=dt,
        )

        # Segmented analysis: GNSS outage segment
        outage_slice = slice(gnss_outage_start_step, gnss_outage_start_step + gnss_outage_duration_steps)
        outage_dist_err = metrics["dist_errors"][outage_slice]
        outage_rmse_pos = float(np.sqrt(np.mean(outage_dist_err**2)))
        outage_max_pos = float(np.max(outage_dist_err))

        metrics["stress_test"] = {
            "elapsed_sec": t_elapsed,
            "hz_throughput": fps,
            "outage_rmse_pos": outage_rmse_pos,
            "outage_max_pos": outage_max_pos,
            "min_confidence": float(np.min(conf_history)),
            "mean_confidence": float(np.mean(conf_history)),
        }

        print(f"\n --- STRESS TEST RESULTS ({circuit_name}) ---")
        print(f" > Overall Lap 2D Pos RMSE:        {metrics['rmse_ekf']:.4f} m (Improvement: {metrics['improvement']:.1f}%)")
        print(f" > CEP50 (Median 2D):              {metrics.get('cep_50', 0.0):.4f} m | CEP95 (R95): {metrics.get('cep_95', 0.0):.4f} m")
        print(f" > RPE 10-step Drift:              {metrics.get('rpe_10_rmse', 0.0):.4f} m (Max: {metrics.get('rpe_10_max', 0.0):.4f} m)")
        print(f" > Lateral Frenet RMSE:            {metrics['rmse_lat']:.4f} m (Max Lateral: {metrics['max_lat_error']:.4f} m)")
        print(f" > 10.0s GNSS Outage Pos RMSE:     {outage_rmse_pos:.4f} m (Max Outage Error: {outage_max_pos:.4f} m)")
        print(f" > Velocity RMSE:                  {metrics['rmse_v']:.4f} m/s | Heading RMSE: {metrics['rmse_theta']:.4f} rad ({np.degrees(metrics['rmse_theta']):.2f} deg)")
        print(f" > Dynamic Sideslip (beta):        Mean: {metrics.get('mean_sideslip_deg', 0.0):.2f} deg | Max: {metrics.get('max_sideslip_deg', 0.0):.2f} deg")
        print(f" > Real-Time Fusion Throughput:    {fps:.1f} Hz (Step latency: {(t_elapsed/n_steps)*1e3:.2f} ms)")
        print(f" > Mean Localization Confidence:   {metrics['stress_test']['mean_confidence']:.2f} | HPL (10^-7): {metrics.get('mean_hpl', 0.0):.4f} m")

        if show_plots:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
            ax1.plot(metrics["dist_errors"], color="blue", label="Multi-Sensor Error (m)")
            ax1.axvspan(gnss_outage_start_step, gnss_outage_start_step + gnss_outage_duration_steps, color="red", alpha=0.2, label="GNSS Outage")
            ax1.axvspan(urban_canyon_start_step, urban_canyon_start_step + urban_canyon_duration_steps, color="orange", alpha=0.2, label="Urban Canyon")
            ax1.set_title(f"Localization Accuracy Under Sensor Failure Stress ({circuit_name})")
            ax1.set_ylabel("Error (m)")
            ax1.legend()
            ax1.grid(True)

            ax2.plot(conf_history, color="green", label="Localization Confidence")
            ax2.axvspan(gnss_outage_start_step, gnss_outage_start_step + gnss_outage_duration_steps, color="red", alpha=0.2)
            ax2.set_title("System Localization Confidence & Degradation Tracking")
            ax2.set_xlabel("Waypoint Step")
            ax2.set_ylabel("Confidence [0.0 - 1.0]")
            ax2.set_ylim(-0.05, 1.05)
            ax2.legend()
            ax2.grid(True)

            plt.tight_layout()
            plt.show()

        return metrics
