"""
Test Session Entity ORM Model.
Tracks proving ground execution campaigns, lifecycle transitions, and safety verdicts.
"""

from datetime import datetime

from sqlalchemy import JSON, Column, DateTime, Float, Integer, String

from ..core.database import Base


class SessionModel(Base):
    __tablename__ = "test_sessions"

    id = Column(String(64), primary_key=True, index=True)
    name = Column(String(128), nullable=False)
    track_name = Column(String(64), nullable=False, index=True)
    vehicle_id = Column(String(64), default="apex-f1tenth-01")
    profile = Column(String(64), default="NOMINAL_RACELINE")
    state = Column(String(32), default="IDLE", index=True)  # IDLE, ARMED, RUNNING, PAUSED, COMPLETED, ABORTED
    target_laps = Column(Integer, default=3)
    completed_laps = Column(Integer, default=0)
    
    # Statistical validation outcomes
    best_lap_time_s = Column(Float, nullable=True)
    position_rmse_m = Column(Float, nullable=True)
    max_position_error_m = Column(Float, nullable=True)
    verdict = Column(String(32), default="PENDING")  # PASS, CONDITIONAL, FAIL, PENDING
    
    # Full KPI evaluation snapshot
    scorecard_json = Column(JSON, nullable=True)
    
    start_time = Column(DateTime, default=datetime.utcnow)
    end_time = Column(DateTime, nullable=True)
