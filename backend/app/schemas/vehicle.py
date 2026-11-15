"""
Vehicle Pydantic Schemas.
"""

from datetime import datetime

from pydantic import BaseModel, ConfigDict, Field


class VehicleExtrinsics(BaseModel):
    dx: float = 0.0
    dy: float = 0.0
    dtheta: float = 0.0


class VehicleBase(BaseModel):
    name: str = Field(..., json_schema_extra={"example": "Apex-F1TENTH-01"})
    vehicle_type: str = Field(default="F1TENTH_SCALE", json_schema_extra={"example": "F1TENTH_SCALE"})
    wheelbase_m: float = Field(default=0.33, ge=0.1, le=5.0)
    track_width_m: float = Field(default=0.25, ge=0.1, le=3.0)
    mass_kg: float = Field(default=3.5, ge=0.5, le=5000.0)
    max_steering_rad: float = Field(default=0.4189, ge=0.1, le=1.5)
    max_accel_mps2: float = Field(default=9.81, ge=0.5, le=30.0)
    max_speed_mps: float = Field(default=20.0, ge=1.0, le=100.0)
    sensor_extrinsics: dict[str, dict[str, float]] = Field(
        default_factory=lambda: {
            "gnss": {"dx": -0.1, "dy": 0.0, "dtheta": 0.0},
            "lidar": {"dx": 0.25, "dy": 0.0, "dtheta": 0.0},
            "camera": {"dx": 0.35, "dy": 0.0, "dtheta": 0.0},
            "imu": {"dx": 0.0, "dy": 0.0, "dtheta": 0.0},
        }
    )
    firmware_version: str = "2.0.0"
    is_active: str = "ACTIVE"


class VehicleCreate(VehicleBase):
    id: str = Field(..., json_schema_extra={"example": "apex-f1tenth-01"})


class VehicleUpdate(BaseModel):
    name: str | None = None
    vehicle_type: str | None = None
    wheelbase_m: float | None = None
    mass_kg: float | None = None
    max_steering_rad: float | None = None
    max_accel_mps2: float | None = None
    max_speed_mps: float | None = None
    sensor_extrinsics: dict[str, dict[str, float]] | None = None
    firmware_version: str | None = None
    is_active: str | None = None


class VehicleResponse(VehicleBase):
    id: str
    created_at: datetime
    updated_at: datetime

    model_config = ConfigDict(from_attributes=True)
