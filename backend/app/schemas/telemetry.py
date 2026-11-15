"""
Telemetry & Real-Time Stream Schemas.
"""

from typing import Any

from pydantic import BaseModel, Field


class PoseSchema(BaseModel):
    x: float
    y: float
    z: float = 0.0
    theta_rad: float
    theta_deg: float


class TwistSchema(BaseModel):
    linear_mps: float
    angular_radps: float


class ActuationSchema(BaseModel):
    steering_angle_rad: float
    steering_angle_deg: float
    acceleration_mps2: float
    target_velocity_mps: float
    cross_track_error_m: float
    heading_error_rad: float
    is_saturated_steering: bool = False
    is_saturated_accel: bool = False
    solve_time_ms: float = 0.0


class SensorDiagnosticsSchema(BaseModel):
    active_primary_source: str
    overall_confidence: float
    covariance_trace: float
    safety_integrity: str
    sensor_health: dict[str, Any]


class TelemetryFrameSchema(BaseModel):
    timestamp: float
    circuit: str
    session_id: str | None = None
    vehicle_id: str
    pose: PoseSchema
    twist: TwistSchema
    dynamics: dict[str, float]
    actuation: ActuationSchema
    diagnostics: SensorDiagnosticsSchema
    frenet: dict[str, float]
    estop_active: bool = False
    lookahead_waypoints: list[list[float]] = Field(default_factory=list)
