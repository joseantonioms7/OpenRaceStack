"""
Telemetry Records & Incident Snapshot ORM Models.
Persists completed sector/lap records and critical safety incidents.
"""

from datetime import datetime

from sqlalchemy import JSON, Column, DateTime, Float, ForeignKey, Integer, String

from ..core.database import Base


class LapRecordModel(Base):
    __tablename__ = "lap_records"

    id = Column(Integer, primary_key=True, autoincrement=True)
    session_id = Column(String(64), ForeignKey("test_sessions.id"), index=True)
    lap_number = Column(Integer, nullable=False)
    lap_time_s = Column(Float, nullable=False)
    sector_1_s = Column(Float, nullable=True)
    sector_2_s = Column(Float, nullable=True)
    sector_3_s = Column(Float, nullable=True)
    max_velocity_mps = Column(Float, default=0.0)
    avg_velocity_mps = Column(Float, default=0.0)
    lateral_rmse_m = Column(Float, default=0.0)
    max_lateral_m = Column(Float, default=0.0)
    created_at = Column(DateTime, default=datetime.utcnow)


class IncidentSnapshotModel(Base):
    __tablename__ = "incident_snapshots"

    id = Column(String(64), primary_key=True, index=True)
    session_id = Column(String(64), ForeignKey("test_sessions.id"), index=True)
    timestamp = Column(Float, nullable=False)
    incident_type = Column(String(64), nullable=False)  # BOUNDARY_VIOLATION, ESTOP_ENGAGED, SENSOR_STARVATION
    severity = Column(String(32), default="CRITICAL")
    position_x = Column(Float, nullable=False)
    position_y = Column(Float, nullable=False)
    velocity_mps = Column(Float, default=0.0)
    lateral_error_m = Column(Float, default=0.0)
    details = Column(JSON, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
