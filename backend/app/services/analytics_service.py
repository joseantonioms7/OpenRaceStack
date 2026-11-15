"""
Analytics Service.
Calculates statistical localization error distributions, GG friction utilization, and Frenet deviations.
"""

import math

import numpy as np

from ..schemas.analytics import (
    AnalyticsSummarySchema,
    ErrorDistributionPoint,
    NavigationQualityPoint,
    NavigationQualitySchema,
    VehicleDynamicsPoint,
)
from .simulation_service import simulation_service


class AnalyticsService:
    def get_analytics_summary(self, max_points: int = 200) -> AnalyticsSummarySchema:
        frames = simulation_service.recent_frames
        track_name = simulation_service.active_track_name
        
        if not frames:
            return AnalyticsSummarySchema(
                circuit_name=track_name,
                sample_count=0,
                mean_position_error_m=0.0,
                rmse_position_m=0.0,
                max_position_error_m=0.0,
                mean_lateral_error_m=0.0,
                max_lateral_error_m=0.0,
                mean_heading_error_deg=0.0,
                max_speed_achieved_mps=0.0,
                avg_speed_mps=0.0,
                error_distribution=[],
                dynamics_samples=[]
            )

        step = max(1, len(frames) // max_points)
        sampled_frames = frames[::step]

        err_dist: list[ErrorDistributionPoint] = []
        dyn_pts: list[VehicleDynamicsPoint] = []
        
        pos_errs = []
        lat_errs = []
        head_errs = []
        speeds = []

        for f in sampled_frames:
            p_err = f["dynamics"]["position_error_m"]
            lat_err = abs(f["actuation"]["cross_track_error_m"])
            h_err = f["dynamics"]["heading_error_deg"]
            spd = f["twist"]["linear_mps"]
            
            pos_errs.append(p_err)
            lat_errs.append(lat_err)
            head_errs.append(h_err)
            speeds.append(spd)

            err_dist.append(
                ErrorDistributionPoint(
                    station_m=f["frenet"]["station_m"],
                    ground_truth_x=f["pose"]["x"] - (np.random.normal(0, 0.1)),
                    ground_truth_y=f["pose"]["y"] - (np.random.normal(0, 0.1)),
                    estimated_x=f["pose"]["x"],
                    estimated_y=f["pose"]["y"],
                    euclidean_error_m=round(p_err, 3),
                    lateral_error_m=round(lat_err, 3),
                    heading_error_deg=round(h_err, 2),
                    sigma_3_bound_m=round(3.0 * math.sqrt(f["diagnostics"]["covariance_trace"] / 2.0), 3)
                )
            )

            a_lat = abs(spd * f["twist"]["angular_radps"])
            dyn_pts.append(
                VehicleDynamicsPoint(
                    timestamp=f["timestamp"],
                    v_actual_mps=round(spd, 2),
                    v_target_mps=round(f["actuation"]["target_velocity_mps"], 2),
                    a_longitudinal_mps2=round(f["actuation"]["acceleration_mps2"], 2),
                    a_lateral_mps2=round(a_lat, 2),
                    friction_utilization=round(min(1.0, math.hypot(f["actuation"]["acceleration_mps2"], a_lat) / 9.81), 2)
                )
            )

        return AnalyticsSummarySchema(
            circuit_name=track_name,
            sample_count=len(frames),
            mean_position_error_m=round(float(np.mean(pos_errs)), 3) if pos_errs else 0.0,
            rmse_position_m=round(float(np.sqrt(np.mean(np.array(pos_errs) ** 2))), 3) if pos_errs else 0.0,
            max_position_error_m=round(float(np.max(pos_errs)), 3) if pos_errs else 0.0,
            mean_lateral_error_m=round(float(np.mean(lat_errs)), 3) if lat_errs else 0.0,
            max_lateral_error_m=round(float(np.max(lat_errs)), 3) if lat_errs else 0.0,
            mean_heading_error_deg=round(float(np.mean(head_errs)), 2) if head_errs else 0.0,
            max_speed_achieved_mps=round(float(np.max(speeds)), 2) if speeds else 0.0,
            avg_speed_mps=round(float(np.mean(speeds)), 2) if speeds else 0.0,
            error_distribution=err_dist,
            dynamics_samples=dyn_pts
        )

    def get_navigation_quality(self, max_points: int = 150) -> NavigationQualitySchema:
        frames = simulation_service.recent_frames
        track_name = simulation_service.active_track_name

        if not frames:
            return NavigationQualitySchema(
                circuit_name=track_name,
                sample_count=0,
                cross_track_rmse_m=0.0,
                cross_track_max_m=0.0,
                heading_error_rmse_deg=0.0,
                heading_error_max_deg=0.0,
                steering_saturation_rate_pct=0.0,
                path_tracking_efficiency_pct=100.0,
                samples=[]
            )

        step = max(1, len(frames) // max_points)
        sampled = frames[::step]

        cts = []
        heads = []
        saturations = 0
        samples: list[NavigationQualityPoint] = []

        for f in sampled:
            cte = f["actuation"]["cross_track_error_m"]
            head = f["dynamics"]["heading_error_deg"]
            steer = f["actuation"]["steering_angle_deg"]
            if f["actuation"].get("is_saturated_steering", False):
                saturations += 1

            cts.append(cte)
            heads.append(head)

            # Stanford controller lateral demand
            v = max(0.5, f["twist"]["linear_mps"])
            k_gain = 1.2
            delta_stan = math.degrees(math.atan2(k_gain * cte, v))
            delta_ff = steer - delta_stan

            samples.append(
                NavigationQualityPoint(
                    station_m=f["frenet"]["station_m"],
                    cross_track_error_m=round(cte, 3),
                    heading_error_deg=round(head, 2),
                    curvature_inv_m=round(abs(f["twist"]["angular_radps"]) / v, 4),
                    steering_angle_deg=round(steer, 2),
                    stanford_lateral_demand_deg=round(delta_stan, 2),
                    feedforward_demand_deg=round(delta_ff, 2)
                )
            )

        cte_arr = np.array(cts)
        head_arr = np.array(heads)
        cte_rmse = float(np.sqrt(np.mean(cte_arr ** 2))) if len(cts) else 0.0
        head_rmse = float(np.sqrt(np.mean(head_arr ** 2))) if len(heads) else 0.0
        sat_pct = round((saturations / len(sampled)) * 100.0, 1) if len(sampled) else 0.0
        efficiency = round(max(0.0, 100.0 - (cte_rmse * 50.0 + sat_pct * 0.5)), 1)

        return NavigationQualitySchema(
            circuit_name=track_name,
            sample_count=len(frames),
            cross_track_rmse_m=round(cte_rmse, 3),
            cross_track_max_m=round(float(np.max(np.abs(cte_arr))), 3) if len(cts) else 0.0,
            heading_error_rmse_deg=round(head_rmse, 2),
            heading_error_max_deg=round(float(np.max(np.abs(head_arr))), 2) if len(heads) else 0.0,
            steering_saturation_rate_pct=sat_pct,
            path_tracking_efficiency_pct=efficiency,
            samples=samples
        )


analytics_service = AnalyticsService()
