"""
Integration Tests for FastAPI Telemetry Endpoints.
"""

import pytest
from starlette.testclient import TestClient

from Localization.telemetry_server import app


@pytest.fixture(scope="module")
def client():
    """Provides a TestClient instance for the FastAPI telemetry server."""
    with TestClient(app) as test_client:
        yield test_client


def test_health_endpoint(client):
    """Verify GET /health returns service status and version 2.0.0."""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert data["version"] == "2.0.0"
    assert "active_track" in data


def test_list_tracks_endpoint(client):
    """Verify GET /api/v1/tracks returns discovered race circuits."""
    response = client.get("/api/v1/tracks")
    assert response.status_code == 200
    data = response.json()
    assert data["total_circuits"] >= 20
    assert "Monza" in data["circuits"]


def test_list_scenarios_endpoint(client):
    """Verify GET /api/v1/scenarios returns proving ground scenarios."""
    response = client.get("/api/v1/scenarios")
    assert response.status_code == 200
    data = response.json()
    assert len(data["scenarios"]) >= 3
    scenario_ids = [s["scenario_id"] for s in data["scenarios"]]
    assert "monza_tunnel_blackout" in scenario_ids


def test_estop_toggle_endpoint(client):
    """Verify POST /api/v1/safety/estop actuates and releases emergency stop."""
    # Engage E-Stop
    res_engage = client.post("/api/v1/safety/estop", json={"active": True})
    assert res_engage.status_code == 200
    assert res_engage.json()["estop_active"] is True

    # Release E-Stop
    res_release = client.post("/api/v1/safety/estop", json={"active": False})
    assert res_release.status_code == 200
    assert res_release.json()["estop_active"] is False
