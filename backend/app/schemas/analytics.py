"""
Localization & Vehicle Analytics Schemas.
"""

from pydantic import BaseModel


class ErrorDistributionPoint(BaseModel):
    station_m: float
    ground_truth_x: float
    ground_truth_y: float
    estimated_x: float
    estimated_y: float
    euclidean_error_m: float
    lateral_error_m: float
    heading_error_deg: float
    sigma_3_bound_m: float


class VehicleDynamicsPoint(BaseModel):
    timestamp: float
    v_actual_mps: float
    v_target_mps: float
    a_longitudinal_mps2: float
    a_lateral_mps2: float
    friction_utilization: float  # |a| / (mu * g)


class AnalyticsSummarySchema(BaseModel):
    circuit_name: str
    sample_count: int
    mean_position_error_m: float
    rmse_position_m: float
    max_position_error_m: float
    mean_lateral_error_m: float
    max_lateral_error_m: float
    mean_heading_error_deg: float
    max_speed_achieved_mps: float
    avg_speed_mps: float
    error_distribution: list[ErrorDistributionPoint]
    dynamics_samples: list[VehicleDynamicsPoint]


class NavigationQualityPoint(BaseModel):
    station_m: float
    cross_track_error_m: float
    heading_error_deg: float
    curvature_inv_m: float
    steering_angle_deg: float
    stanford_lateral_demand_deg: float
    feedforward_demand_deg: float


class NavigationQualitySchema(BaseModel):
    circuit_name: str
    sample_count: int
    cross_track_rmse_m: float
    cross_track_max_m: float
    heading_error_rmse_deg: float
    heading_error_max_deg: float
    steering_saturation_rate_pct: float
    path_tracking_efficiency_pct: float
    samples: list[NavigationQualityPoint]
