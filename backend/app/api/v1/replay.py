"""
Replay System Router.
"""

from fastapi import APIRouter, HTTPException

from ...schemas.replay import ReplayControlRequest, ReplaySeekRequest, ReplayTimelineSchema
from ...services.replay_service import replay_service

router = APIRouter(prefix="/replay", tags=["Replay System"])


@router.get("/timeline", response_model=ReplayTimelineSchema)
def get_replay_timeline():
    """Returns available telemetry duration, frame counts, and incident markers."""
    return replay_service.get_timeline()


@router.post("/seek")
def seek_timeline(req: ReplaySeekRequest):
    """Retrieves full vehicle telemetry state at a designated timestamp."""
    frame = replay_service.seek(req.target_time)
    if not frame:
        raise HTTPException(status_code=404, detail="No telemetry frame found for specified timestamp.")
    return frame


@router.post("/control", response_model=ReplayTimelineSchema)
def control_playback(req: ReplayControlRequest):
    """Controls replay playback state (play, pause, speed)."""
    return replay_service.control(req)
