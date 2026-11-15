"""
Test Session Pydantic Schemas.
"""

from datetime import datetime
from typing import Any

from pydantic import BaseModel, ConfigDict, Field


class SessionCreate(BaseModel):
    name: str = Field(..., json_schema_extra={"example": "Monza High-Speed Proving Ground Run"})
    track_name: str = Field(default="Monza", json_schema_extra={"example": "Monza"})
    vehicle_id: str = Field(default="apex-f1tenth-01", json_schema_extra={"example": "apex-f1tenth-01"})
    profile: str = Field(default="NOMINAL_RACELINE", json_schema_extra={"example": "NOMINAL_RACELINE"})
    target_laps: int = Field(default=3, ge=1, le=50)


class SessionTransition(BaseModel):
    action: str = Field(..., json_schema_extra={"example": "start"}, description="One of: arm, start, pause, stop, abort")


class LapRecordSchema(BaseModel):
    lap_number: int
    lap_time_s: float
    sector_1_s: float | None = None
    sector_2_s: float | None = None
    sector_3_s: float | None = None
    max_velocity_mps: float
    avg_velocity_mps: float
    lateral_rmse_m: float
    max_lateral_m: float

    model_config = ConfigDict(from_attributes=True)


class SessionResponse(BaseModel):
    id: str
    name: str
    track_name: str
    vehicle_id: str
    profile: str
    state: str
    target_laps: int
    completed_laps: int
    best_lap_time_s: float | None = None
    position_rmse_m: float | None = None
    max_position_error_m: float | None = None
    verdict: str
    start_time: datetime
    end_time: datetime | None = None

    model_config = ConfigDict(from_attributes=True)


class SessionDetail(SessionResponse):
    laps: list[LapRecordSchema] = Field(default_factory=list)
    scorecard: dict[str, Any] | None = None
