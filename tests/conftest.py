"""
ApexPathAI Test Suite Configuration & Shared Fixtures.
"""

import numpy as np
import pytest

from Localization.core.ekf_engine import LocalizationEKF
from Localization.core.eskf_3d import ErrorStateKalmanFilter3D
from Localization.core.mpc_controller import ModelPredictiveController
from Localization.core.session_orchestrator import (
    SessionOrchestrator,
    SessionThresholds,
    TestProfile,
)


@pytest.fixture
def circular_track():
    """Generates a synthetic circular racetrack of radius 100m with 200 waypoints."""
    radius = 100.0
    num_points = 200
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    return np.column_stack((x, y))


@pytest.fixture
def initialized_ekf():
    """Returns a 5-state CTRV LocalizationEKF initialized at (0, 0) with 15 m/s speed."""
    dt = 0.05
    Q_diag = [0.008, 0.008, 0.002, 0.06, 2.0]
    R_diag = [2.25, 2.25]
    ekf = LocalizationEKF(dt=dt, Q_diag=Q_diag, R_diag=R_diag)
    x0 = np.array([[0.0], [0.0], [0.0], [15.0], [0.0]], dtype=np.float64)
    P0 = np.diag([0.1, 0.1, 0.01, 0.05, 0.05])
    ekf.initialize_state(x0, P0=P0, timestamp=0.0)
    return ekf


@pytest.fixture
def initialized_eskf():
    """Returns a 15-DOF ErrorStateKalmanFilter3D initialized at (0, 0, 0)."""
    eskf = ErrorStateKalmanFilter3D(dt_default=0.05)
    eskf.initialize_state(p0=[0.0, 0.0, 0.0], v0=[15.0, 0.0, 0.0], yaw0=0.0)
    return eskf


@pytest.fixture
def mpc_controller():
    """Returns a ModelPredictiveController with standard F1TENTH / autonomous scale limits."""
    return ModelPredictiveController(wheelbase_m=0.33, horizon_steps=15, dt=0.05)


@pytest.fixture
def active_session_orchestrator():
    """Returns a SessionOrchestrator armed and running a nominal raceline session."""
    orch = SessionOrchestrator()
    orch.create_session(
        name="Unit Test Validation Session",
        track_name="Monza",
        profile=TestProfile.NOMINAL_RACELINE,
        target_laps=2,
        thresholds=SessionThresholds(),
    )
    orch.arm()
    orch.start()
    return orch
