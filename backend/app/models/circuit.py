"""
Circuit Entity ORM Model.
Represents proving ground race circuits with geometry metadata and map configurations.
"""

from datetime import datetime

from sqlalchemy import Boolean, Column, DateTime, Float, Integer, String

from ..core.database import Base


class CircuitModel(Base):
    __tablename__ = "circuits"

    id = Column(String(64), primary_key=True, index=True)
    name = Column(String(128), unique=True, nullable=False, index=True)
    country = Column(String(64), default="International")
    length_m = Column(Float, default=0.0)
    width_m = Column(Float, default=12.0)
    waypoints_count = Column(Integer, default=0)
    has_raceline = Column(Boolean, default=True)
    has_raster_map = Column(Boolean, default=True)
    resolution_m = Column(Float, default=0.05)
    origin_x = Column(Float, default=0.0)
    origin_y = Column(Float, default=0.0)
    created_at = Column(DateTime, default=datetime.utcnow)
