"""
Sensor Diagnostics & Fault Injection Schemas.
"""

from pydantic import BaseModel, Field


class SensorHealthItem(BaseModel):
    sensor_name: str
    is_healthy: bool
    status_message: str
    message_rate_hz: float
    total_updates: int
    consecutive_rejections: int
    total_rejections: int
    last_nis: float
    timeout_counter: int


class SensorMatrixResponse(BaseModel):
    overall_system_healthy: bool
    primary_source: str
    confidence_score: float
    sensors: list[SensorHealthItem]
    rejection_rates: dict[str, float]


class FaultInjectionRequest(BaseModel):
    sensor_name: str = Field(..., json_schema_extra={"example": "gnss"}, description="gnss, lidar, camera, imu")
    fault_type: str = Field(..., json_schema_extra={"example": "dropout"}, description="dropout, noise_bias, drift, delay")
    duration_sec: float = Field(default=5.0, ge=0.5, le=60.0)
    magnitude: float = Field(default=2.5, description="Noise standard deviation or bias magnitude")
