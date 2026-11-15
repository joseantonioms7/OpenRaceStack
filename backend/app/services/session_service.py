"""
Session Service.
Campaign lifecycle, safety scorecard evaluation, and sector lap timing.
"""

from dataclasses import asdict
from datetime import datetime

import numpy as np

from ..schemas.session import LapRecordSchema, SessionCreate, SessionDetail, SessionResponse
from .simulation_service import simulation_service


class SessionService:
    def __init__(self):
        self._sessions: dict[str, SessionDetail] = {}

    def create_session(self, req: SessionCreate) -> SessionResponse:
        orch = simulation_service.orchestrator
        # Map profile string
        from Localization.core.session_orchestrator import TestProfile
        profile_map = {
            "NOMINAL_RACELINE": TestProfile.NOMINAL_RACELINE,
            "GNSS_DENIED_OUTAGE": TestProfile.GNSS_DENIED_OUTAGE,
            "URBAN_CANYON_MULTIPATH": TestProfile.URBAN_CANYON_MULTIPATH,
            "SENSOR_FAILOVER_STRESS": TestProfile.SENSOR_FAILOVER_STRESS,
            "HIGH_SPEED_LIMIT_HANDLING": TestProfile.HIGH_SPEED_LIMIT_HANDLING,
        }
        test_prof = profile_map.get(req.profile, TestProfile.NOMINAL_RACELINE)

        sess_obj = orch.create_session(
            name=req.name,
            track_name=req.track_name,
            profile=test_prof,
            target_laps=req.target_laps
        )

        resp = SessionDetail(
            id=sess_obj.session_id,
            name=sess_obj.name,
            track_name=sess_obj.track_name,
            vehicle_id=req.vehicle_id,
            profile=sess_obj.test_profile.value,
            state=sess_obj.state.value,
            target_laps=sess_obj.target_laps,
            completed_laps=0,
            verdict="PENDING",
            start_time=datetime.fromtimestamp(sess_obj.start_time),
            laps=[],
            scorecard=None
        )
        self._sessions[resp.id] = resp
        return resp

    def transition_session(self, action: str) -> SessionResponse | None:
        orch = simulation_service.orchestrator
        act = action.lower().strip()
        curr = orch.active_session
        if not curr:
            return None

        if act == "arm":
            orch.arm()
        elif act == "start":
            orch.start()
            simulation_service.resume()
        elif act == "pause":
            orch.pause()
            simulation_service.pause()
        elif act == "stop":
            orch.stop()
        elif act == "abort":
            orch.abort("Operator Abort")

        detail = self._sync_active_session()
        return detail

    def get_active_session(self) -> SessionDetail | None:
        return self._sync_active_session()

    def list_sessions(self) -> list[SessionResponse]:
        self._sync_active_session()
        return list(self._sessions.values())

    def get_session(self, session_id: str) -> SessionDetail | None:
        self._sync_active_session()
        return self._sessions.get(session_id)

    def _sync_active_session(self) -> SessionDetail | None:
        orch = simulation_service.orchestrator
        curr = orch.active_session
        if not curr:
            return None

        laps = [
            LapRecordSchema(
                lap_number=lap.lap_number,
                lap_time_s=lap.lap_time_s,
                sector_1_s=getattr(lap, "s1_s", None),
                sector_2_s=getattr(lap, "s2_s", None),
                sector_3_s=getattr(lap, "s3_s", None),
                max_velocity_mps=getattr(lap, "max_speed_kmh", 0.0) / 3.6,
                avg_velocity_mps=getattr(lap, "avg_speed_kmh", 0.0) / 3.6,
                lateral_rmse_m=getattr(lap, "lat_rmse_m", 0.0),
                max_lateral_m=getattr(lap, "max_lat_error_m", 0.0),
            )
            for lap in curr.laps
        ]

        scorecard = None
        verdict = "PENDING"
        if curr.scorecard:
            scorecard = asdict(curr.scorecard)
            verdict = "PASS" if getattr(curr.scorecard, "overall_passed", False) else "FAIL"

        best_t = min([lap.lap_time_s for lap in curr.laps]) if curr.laps else None
        rmse = float(np.mean([getattr(lap, "lat_rmse_m", 0.0) for lap in curr.laps])) if curr.laps else None

        detail = SessionDetail(
            id=curr.session_id,
            name=curr.name,
            track_name=curr.track_name,
            vehicle_id=simulation_service.active_vehicle_id,
            profile=curr.test_profile.value,
            state=curr.state.value,
            target_laps=curr.target_laps,
            completed_laps=len(curr.laps),
            best_lap_time_s=best_t,
            position_rmse_m=rmse,
            max_position_error_m=max([getattr(lap, "max_lat_error_m", 0.0) for lap in curr.laps]) if curr.laps else None,
            verdict=verdict,
            start_time=datetime.fromtimestamp(curr.start_time),
            end_time=datetime.fromtimestamp(curr.end_time) if curr.end_time else None,
            laps=laps,
            scorecard=scorecard
        )
        self._sessions[curr.session_id] = detail
        return detail


session_service = SessionService()
