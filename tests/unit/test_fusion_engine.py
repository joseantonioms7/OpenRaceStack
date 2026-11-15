"""
Unit Tests for MultiSensorFusionEngine.
Verifies multi-sensor fusion, innovation gating, and telemetry snapshots.
"""

import numpy as np
import pytest

from backend.app.core.config import settings
from Localization.core.fusion_engine import MultiSensorFusionEngine
from Localization.core.map_manager import MapManager
from Localization.core.sensor_interface import GNSSMeasurement


@pytest.fixture
def fusion_engine():
    monza_dir = settings.DATASETS_DIR / "Monza"
    map_mgr = MapManager(monza_dir)
    engine = MultiSensorFusionEngine(map_mgr, dt_base=0.05)
    # Initialize EKF pose
    x0 = np.array([[0.0], [0.0], [0.0], [10.0], [0.0]], dtype=np.float64)
    engine.ekf.initialize_state(x0, timestamp=0.0)
    return engine


def test_fusion_engine_initialization(fusion_engine):
    assert fusion_engine is not None
    assert fusion_engine.ekf is not None
    assert fusion_engine.health_supervisor is not None


def test_fuse_gnss_nominal(fusion_engine):
    meas = GNSSMeasurement(
        timestamp=0.05,
        x=0.05,
        y=0.02,
        cov_xx=2.25,
        cov_yy=2.25
    )
    accepted = fusion_engine.fuse_gnss(meas)
    assert accepted is True
    assert fusion_engine.health_supervisor.current_status.value in ["optimal", "degraded", "critical"]


def test_fuse_imu_and_odometry(fusion_engine):
    accepted = fusion_engine.fuse_imu_and_odometry(v_wheel=10.2, omega_imu=0.01)
    assert accepted is True


def test_corridor_boundary_constraint(fusion_engine):
    res = fusion_engine.apply_corridor_boundary_constraint()
    assert isinstance(res, bool)


def test_telemetry_snapshot(fusion_engine):
    snap = fusion_engine.get_telemetry_snapshot()
    assert "timestamp" in snap
    assert "pose" in snap
    assert "velocity" in snap
    assert "covariance" in snap
    assert "active_source" in snap
    assert snap["confidence"] >= 0.0
