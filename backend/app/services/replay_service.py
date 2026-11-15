"""
Replay Service.
Interactive timeline scrubber, variable playback controller, and incident bookmarking.
"""

from typing import Any

from ..schemas.replay import (
    IncidentBookmark,
    ReplayControlRequest,
    ReplayTimelineSchema,
)
from .simulation_service import simulation_service


class ReplayService:
    def __init__(self):
        self.cursor_time = 0.0
        self.is_playing = False
        self.playback_speed = 1.0

    def get_timeline(self) -> ReplayTimelineSchema:
        frames = simulation_service.recent_frames
        track_name = simulation_service.active_track_name

        total_frames = len(frames)
        duration = frames[-1]["timestamp"] - frames[0]["timestamp"] if total_frames > 1 else 0.0

        # Scan for safety incidents to bookmark
        incidents: list[IncidentBookmark] = []
        for idx, f in enumerate(frames):
            lat_err = abs(f["actuation"]["cross_track_error_m"])
            if lat_err > 1.2:
                incidents.append(
                    IncidentBookmark(
                        incident_id=f"inc-boundary-{idx}",
                        timestamp=f["timestamp"],
                        incident_type="BOUNDARY_EXCEEDANCE",
                        severity="WARNING" if lat_err < 1.8 else "CRITICAL",
                        description=f"Lateral deviation of {lat_err:.2f}m exceeds proving ground corridor.",
                        position=[f["pose"]["x"], f["pose"]["y"]],
                    )
                )
            if f.get("estop_active"):
                incidents.append(
                    IncidentBookmark(
                        incident_id=f"inc-estop-{idx}",
                        timestamp=f["timestamp"],
                        incident_type="ESTOP_ENGAGED",
                        severity="CRITICAL",
                        description="Emergency Stop button triggered by operator.",
                        position=[f["pose"]["x"], f["pose"]["y"]],
                    )
                )

        # De-duplicate adjacent incidents within 2.0s
        deduped: list[IncidentBookmark] = []
        last_t = -999.0
        for inc in incidents:
            if inc.timestamp - last_t > 2.0:
                deduped.append(inc)
                last_t = inc.timestamp

        return ReplayTimelineSchema(
            circuit_name=track_name,
            total_frames=total_frames,
            duration_seconds=round(duration, 2),
            current_time=round(self.cursor_time, 2),
            is_playing=self.is_playing,
            playback_speed=self.playback_speed,
            incidents=deduped
        )

    def seek(self, target_time: float) -> dict[str, Any] | None:
        frames = simulation_service.recent_frames
        if not frames:
            return None

        self.cursor_time = target_time
        # Find closest frame
        closest = min(frames, key=lambda f: abs(f["timestamp"] - target_time))
        return closest

    def control(self, req: ReplayControlRequest) -> ReplayTimelineSchema:
        if req.action == "play":
            self.is_playing = True
        elif req.action == "pause":
            self.is_playing = False
        elif req.action == "reset":
            self.cursor_time = 0.0
            self.is_playing = False

        if req.playback_speed:
            self.playback_speed = req.playback_speed

        return self.get_timeline()


replay_service = ReplayService()
