"""
Integration Tests for Closed-Loop Session Orchestrator and 7-Rule KPI Scorecard.
"""


from Localization.core.session_orchestrator import (
    SessionOrchestrator,
    SessionState,
    TestProfile,
)


def test_session_lifecycle():
    """Verify session state transitions: IDLE -> ARMED -> RUNNING -> COMPLETED."""
    orch = SessionOrchestrator()
    session = orch.create_session(
        name="Lifecycle Test",
        track_name="Monza",
        profile=TestProfile.NOMINAL_RACELINE,
        target_laps=1,
    )
    assert session.state == SessionState.IDLE

    # Arm
    assert orch.arm() is True
    assert orch.active_session.state == SessionState.ARMED

    # Start
    assert orch.start() is True
    assert orch.active_session.state == SessionState.RUNNING

    # Pause & Resume
    assert orch.pause() is True
    assert orch.active_session.state == SessionState.PAUSED
    assert orch.start() is True
    assert orch.active_session.state == SessionState.RUNNING

    # Record 1 lap (target is 1)
    finished = orch.record_lap(
        lap_num=1,
        lap_time=82.5,
        s1=28.0,
        s2=25.0,
        s3=29.5,
        max_v=300.0,
        avg_v=240.0,
        lat_rmse=0.18,
        max_lat=0.45,
    )
    assert finished is True
    assert orch.active_session.state == SessionState.COMPLETED
    assert orch.active_session.scorecard is not None


def test_scorecard_seven_rules(active_session_orchestrator):
    """Verify that all 7 rules in the scorecard are evaluated."""
    # Feed 10 nominal steps
    for i in range(10):
        active_session_orchestrator.step_evaluator(
            station_s=float(i * 10),
            pos_err_2d=0.20,
            lat_err=0.15,
            hpl=1.2,
            nees=3.5,
            solve_time_ms=0.5,
            margin_left=2.0,
            margin_right=2.0,
            speed_kmh=120.0,
            dt=0.05,
        )

    scorecard = active_session_orchestrator.finalize_session()
    assert len(scorecard.rule_results) == 7
    assert scorecard.rule_results["Rule 1 - Position RMSE"]["passed"] is True
    assert scorecard.rule_results["Rule 2 - Lateral Deviation"]["passed"] is True
    assert scorecard.rule_results["Rule 3 - Geofence Containment"]["passed"] is True
    assert scorecard.rule_results["Rule 4 - Safety Integrity"]["passed"] is True
    assert scorecard.rule_results["Rule 5 - Kalman Consistency"]["passed"] is True
    assert scorecard.rule_results["Rule 6 - Real-Time Determinism"]["passed"] is True
    assert scorecard.rule_results["Rule 7 - Zero Disengagements"]["passed"] is True
    assert scorecard.overall_passed is True
