"""
Integration Tests for the Complete ApexPathAI Backend Platform.
Validates all 10 Core Pillars through the FastAPI REST & Monitoring Engine.
"""

import pytest
from starlette.testclient import TestClient

from backend.app.main import app


@pytest.fixture(scope="module")
def backend_client():
    with TestClient(app) as client:
        yield client


# Pillar 1 & Health: Real-Time Monitoring & Health
def test_system_health(backend_client):
    res = backend_client.get("/health")
    assert res.status_code == 200
    data = res.json()
    assert data["status"] == "healthy"
    assert "active_circuit" in data
    assert "version" in data


def test_monitoring_snapshot_and_estop(backend_client):
    res_snap = backend_client.get("/api/v1/monitoring/snapshot")
    assert res_snap.status_code == 200
    snap = res_snap.json()
    assert "pose" in snap
    assert "twist" in snap
    assert "dynamics" in snap
    assert "frenet" in snap

    # Toggle E-Stop
    res_estop = backend_client.post("/api/v1/monitoring/estop", json={"active": True})
    assert res_estop.status_code == 200
    assert res_estop.json()["estop_active"] is True

    # Release E-Stop
    res_release = backend_client.post("/api/v1/monitoring/estop", json={"active": False})
    assert res_release.status_code == 200
    assert res_release.json()["estop_active"] is False


# Pillar 2: Vehicle Management
def test_vehicle_management_crud(backend_client):
    # List vehicles (should contain 3 default seeds)
    res_list = backend_client.get("/api/v1/vehicles")
    assert res_list.status_code == 200
    vehicles = res_list.json()
    assert len(vehicles) >= 3
    ids = [v["id"] for v in vehicles]
    assert "apex-f1tenth-01" in ids

    # Create new vehicle
    payload = {
        "id": "apex-kart-test",
        "name": "Apex Test Electric Go-Kart",
        "vehicle_type": "KART_AUTONOMOUS",
        "wheelbase_m": 1.05,
        "track_width_m": 0.85,
        "mass_kg": 160.0,
        "max_steering_rad": 0.50,
        "max_accel_mps2": 8.0,
        "max_speed_mps": 25.0
    }
    res_create = backend_client.post("/api/v1/vehicles", json=payload)
    assert res_create.status_code == 201
    assert res_create.json()["id"] == "apex-kart-test"

    # Select active vehicle
    res_sel = backend_client.post("/api/v1/vehicles/apex-kart-test/select")
    assert res_sel.status_code == 200
    assert res_sel.json()["status"] == "success"

    # Delete vehicle
    res_del = backend_client.delete("/api/v1/vehicles/apex-kart-test")
    assert res_del.status_code == 200


# Pillar 3: Circuit Management
def test_circuits_catalog(backend_client):
    res = backend_client.get("/api/v1/circuits")
    assert res.status_code == 200
    circuits = res.json()
    assert len(circuits) == 23
    names = [c["name"] for c in circuits]
    assert "Monza" in names
    assert "Silverstone" in names
    assert "Spa" in names

    # Inspect single circuit geometry
    res_detail = backend_client.get("/api/v1/circuits/Monza")
    assert res_detail.status_code == 200
    detail = res_detail.json()
    assert detail["name"] == "Monza"
    assert len(detail["centerline_preview"]) > 0

    # Select circuit
    res_sel = backend_client.post("/api/v1/circuits/select", json={"circuit_name": "Monza"})
    assert res_sel.status_code == 200
    assert res_sel.json()["active_circuit"] == "Monza"


# Pillar 4: Test Session Management
def test_session_lifecycle(backend_client):
    create_req = {
        "name": "Integration Test Run",
        "track_name": "Monza",
        "vehicle_id": "apex-f1tenth-01",
        "profile": "NOMINAL_RACELINE",
        "target_laps": 2
    }
    res_create = backend_client.post("/api/v1/sessions", json=create_req)
    assert res_create.status_code == 201
    sess = res_create.json()
    sess_id = sess["id"]
    assert sess["state"] == "IDLE"

    # Arm session
    res_arm = backend_client.post(f"/api/v1/sessions/{sess_id}/transition", json={"action": "arm"})
    assert res_arm.status_code == 200
    assert res_arm.json()["state"] == "ARMED"

    # Start session
    res_start = backend_client.post(f"/api/v1/sessions/{sess_id}/transition", json={"action": "start"})
    assert res_start.status_code == 200
    assert res_start.json()["state"] == "RUNNING"

    # Pause session
    res_pause = backend_client.post(f"/api/v1/sessions/{sess_id}/transition", json={"action": "pause"})
    assert res_pause.status_code == 200
    assert res_pause.json()["state"] == "PAUSED"

    # Stop session
    res_stop = backend_client.post(f"/api/v1/sessions/{sess_id}/transition", json={"action": "stop"})
    assert res_stop.status_code == 200
    assert res_stop.json()["state"] in ["COMPLETED", "RUNNING", "IDLE"]


# Pillar 5 & 6: Analytics (Localization & Vehicle Dynamics)
def test_analytics_summary(backend_client):
    res = backend_client.get("/api/v1/analytics/summary?max_points=50")
    assert res.status_code == 200
    data = res.json()
    assert "circuit_name" in data
    assert "rmse_position_m" in data
    assert "max_speed_achieved_mps" in data
    assert isinstance(data["error_distribution"], list)
    assert isinstance(data["dynamics_samples"], list)


# Pillar 7: Sensor Diagnostics & Fault Injection
def test_sensor_diagnostics_and_fault(backend_client):
    res_matrix = backend_client.get("/api/v1/sensors/matrix")
    assert res_matrix.status_code == 200
    data = res_matrix.json()
    assert "overall_system_healthy" in data
    assert "sensors" in data
    assert len(data["sensors"]) >= 4

    # Inject fault
    fault_payload = {
        "sensor_name": "gnss",
        "fault_type": "dropout",
        "duration_sec": 2.0,
        "magnitude": 2.5
    }
    res_inj = backend_client.post("/api/v1/sensors/inject-fault", json=fault_payload)
    assert res_inj.status_code == 200
    assert res_inj.json()["status"] == "success"


# Pillar 8: Replay System
def test_replay_timeline_and_seek(backend_client):
    res_tl = backend_client.get("/api/v1/replay/timeline")
    assert res_tl.status_code == 200
    tl = res_tl.json()
    assert "total_frames" in tl
    assert "duration_seconds" in tl
    assert isinstance(tl["incidents"], list)

    # Seek
    res_seek = backend_client.post("/api/v1/replay/seek", json={"target_time": 0.5})
    # If frames exist, 200; if no frames, 404
    assert res_seek.status_code in [200, 404]


# Pillar 9: KPI Analysis Scorecard
def test_kpi_scorecard(backend_client):
    res = backend_client.get("/api/v1/kpis/scorecard")
    assert res.status_code == 200
    scorecard = res.json()
    assert scorecard["overall_verdict"] in ["PASS", "CONDITIONAL", "FAIL"]
    assert len(scorecard["rules"]) == 7
    assert scorecard["compliance_score_pct"] >= 0.0


# Pillar 10: Validation Reporting
def test_report_generation_and_export(backend_client):
    req = {
        "session_id": "test-session-suite",
        "title": "Automated Integration Test Validation Dossier",
        "author": "CI Automated Verifier"
    }
    res = backend_client.post("/api/v1/reports", json=req)
    assert res.status_code == 201
    rpt = res.json()
    rpt_id = rpt["id"]
    assert rpt["verdict"] in ["PASS", "CONDITIONAL", "FAIL"]
    assert "content_markdown" in rpt

    # Download Markdown
    res_dl = backend_client.get(f"/api/v1/reports/{rpt_id}/download?format=markdown")
    assert res_dl.status_code == 200
    assert "text/markdown" in res_dl.headers["content-type"]

    # Download HTML
    res_html = backend_client.get(f"/api/v1/reports/{rpt_id}/download?format=html")
    assert res_html.status_code == 200
    assert "text/html" in res_html.headers["content-type"]


def test_navigation_quality_analytics(backend_client):
    res = backend_client.get("/api/v1/analytics/navigation")
    assert res.status_code == 200
    data = res.json()
    assert "cross_track_rmse_m" in data
    assert "heading_error_rmse_deg" in data
    assert "path_tracking_efficiency_pct" in data
    assert isinstance(data["samples"], list)


def test_safety_events_and_reset(backend_client):
    res = backend_client.get("/api/v1/monitoring/safety-events")
    assert res.status_code == 200
    data = res.json()
    assert "events" in data
    assert "total_events" in data
    assert isinstance(data["events"], list)

    res_reset = backend_client.post("/api/v1/monitoring/reset-estop")
    assert res_reset.status_code == 200
    assert res_reset.json()["estop_active"] is False


def test_fleet_comparison_benchmark(backend_client):
    res = backend_client.get("/api/v1/vehicles/fleet-comparison")
    assert res.status_code == 200
    data = res.json()
    assert "fleet_comparison" in data
    assert len(data["fleet_comparison"]) >= 3
    assert data["fleet_comparison"][0]["vehicle_id"] == "apex-f1tenth-01"


def test_circuit_evaluation(backend_client):
    res = backend_client.get("/api/v1/circuits/Monza/evaluate")
    assert res.status_code == 200
    data = res.json()
    assert data["circuit_name"].lower() == "monza"
    assert "complexity_rating" in data
    assert "sectors" in data
    assert len(data["sectors"]) == 3
