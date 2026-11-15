"""
Integration Tests for Circuit HD Vector Map API Endpoint.
"""

from fastapi.testclient import TestClient

from backend.app.main import app

client = TestClient(app)


def test_get_circuit_vector_map_endpoint():
    """Verify GET /api/v1/circuits/{track_name}/vector-map returns valid GeoJSON."""
    # Test with Monza
    response = client.get("/api/v1/circuits/Monza/vector-map")
    assert response.status_code == 200

    data = response.json()
    assert data["type"] == "FeatureCollection"
    assert len(data["features"]) >= 3

    layers = [f["properties"]["layer"] for f in data["features"]]
    assert "centerline" in layers
    assert "left_boundary" in layers
    assert "right_boundary" in layers


def test_get_circuit_vector_map_not_found():
    """Verify GET /api/v1/circuits/{track_name}/vector-map returns 404 for unknown track."""
    response = client.get("/api/v1/circuits/NonExistentTrack_99/vector-map")
    assert response.status_code == 404
