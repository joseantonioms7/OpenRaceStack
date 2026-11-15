"""
Replay System Schemas.
"""

from pydantic import BaseModel, Field


class IncidentBookmark(BaseModel):
    incident_id: str
    timestamp: float
    incident_type: str
    severity: str
    description: str
    position: list[float]


class ReplayTimelineSchema(BaseModel):
    circuit_name: str
    total_frames: int
    duration_seconds: float
    current_time: float
    is_playing: bool
    playback_speed: float
    incidents: list[IncidentBookmark] = Field(default_factory=list)


class ReplaySeekRequest(BaseModel):
    target_time: float = Field(..., ge=0.0)


class ReplayControlRequest(BaseModel):
    action: str = Field(..., json_schema_extra={"example": "play"}, description="play, pause, step_forward, step_backward, reset")
    playback_speed: float | None = Field(default=1.0, ge=0.1, le=10.0)
