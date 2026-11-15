"""
Vehicle Entity ORM Model.
Stores autonomous vehicle fleet assets, kinematic constants, and sensor extrinsics.
"""

from datetime import datetime

from sqlalchemy import JSON, Column, DateTime, Float, String

from ..core.database import Base


class VehicleModel(Base):
    __tablename__ = "vehicles"

    id = Column(String(64), primary_key=True, index=True)
    name = Column(String(128), nullable=False)
    vehicle_type = Column(String(64), default="F1TENTH_SCALE")  # F1TENTH_SCALE, INDY_CAR, ROBOTAXI
    wheelbase_m = Column(Float, default=0.33)
    track_width_m = Column(Float, default=0.25)
    mass_kg = Column(Float, default=3.5)
    max_steering_rad = Column(Float, default=0.4189)  # 24 degrees
    max_accel_mps2 = Column(Float, default=9.81)
    max_speed_mps = Column(Float, default=20.0)
    
    # Sensor extrinsic parameters relative to rear axle
    sensor_extrinsics = Column(
        JSON,
        default=lambda: {
            "gnss": {"dx": -0.1, "dy": 0.0, "dtheta": 0.0},
            "lidar": {"dx": 0.25, "dy": 0.0, "dtheta": 0.0},
            "camera": {"dx": 0.35, "dy": 0.0, "dtheta": 0.0},
            "imu": {"dx": 0.0, "dy": 0.0, "dtheta": 0.0},
        }
    )
    
    # Active autonomy status
    firmware_version = Column(String(32), default="2.0.0")
    is_active = Column(String(16), default="ACTIVE")
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
