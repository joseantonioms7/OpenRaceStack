"""
Unit Tests for High-Definition (HD) Vector Map Geometry, Sectors, and GeoJSON.
"""

import math

import numpy as np
import pytest

from Localization.core.hd_vector_map import HDVectorMap


@pytest.fixture
def circular_vector_map():
    """Synthetic circular track HD Vector Map of radius 100m, width 8m (4m right, 4m left)."""
    angles = np.linspace(0.0, 2.0 * math.pi, 180, endpoint=False)
    x = 100.0 * np.cos(angles)
    y = 100.0 * np.sin(angles)
    centerline = np.column_stack((x, y))
    widths = np.full((180, 2), 4.0)
    return HDVectorMap(track_name="CircularProvingGround", centerline=centerline, widths=widths, is_closed=True)


def test_hd_vector_map_boundaries(circular_vector_map):
    """Verify extrusion of left and right track boundaries from centerline and widths."""
    v_map = circular_vector_map

    # On a 100m counter-clockwise circle:
    # Left boundary (inside of left turn) is at radius 100 - 4 = 96m
    # Right boundary (outside of left turn) is at radius 100 + 4 = 104m
    radii_right = np.hypot(v_map.right_boundary[:, 0], v_map.right_boundary[:, 1])
    radii_left = np.hypot(v_map.left_boundary[:, 0], v_map.left_boundary[:, 1])

    assert np.allclose(radii_left, 96.0, atol=0.2)
    assert np.allclose(radii_right, 104.0, atol=0.2)
    assert math.isclose(v_map.total_length, 2.0 * math.pi * 100.0, rel_tol=0.02)


def test_hd_vector_map_point_queries(circular_vector_map):
    """Verify point projection, corridor bounds, containment, and signed boundary clearance."""
    v_map = circular_vector_map

    # Point on centerline at (100, 0)
    s_val, e_y, _ = v_map.project_point(100.0, 0.0)
    assert math.isclose(e_y, 0.0, abs_tol=0.1)

    # Point is inside track
    assert v_map.is_inside_track(100.0, 0.0)
    # Signed distance to boundary is ~4.0m
    dist_clearance = v_map.distance_to_boundary(100.0, 0.0)
    assert math.isclose(dist_clearance, 4.0, abs_tol=0.2)

    # Point way outside at (150, 0) (50m off track)
    assert not v_map.is_inside_track(150.0, 0.0)
    assert v_map.distance_to_boundary(150.0, 0.0) < -40.0


def test_hd_vector_map_sectors(circular_vector_map):
    """Verify circuit partition into 3 timing sectors."""
    v_map = circular_vector_map
    assert len(v_map.sectors) == 3

    # At start of lap (s = 10) -> Sector 1
    assert v_map.get_sector(10.0) == 1

    # At mid-lap (s = total_length * 0.5) -> Sector 2
    assert v_map.get_sector(v_map.total_length * 0.5) == 2

    # At end of lap (s = total_length * 0.85) -> Sector 3
    assert v_map.get_sector(v_map.total_length * 0.85) == 3


def test_hd_vector_map_geojson_export(circular_vector_map):
    """Verify GeoJSON FeatureCollection serialization for Mapbox rendering."""
    v_map = circular_vector_map
    geojson = v_map.to_geojson()

    assert geojson["type"] == "FeatureCollection"
    layers = [f["properties"]["layer"] for f in geojson["features"]]
    assert "centerline" in layers
    assert "left_boundary" in layers
    assert "right_boundary" in layers
    assert "sector_gate" in layers

    # String serialization check
    raw_str = v_map.to_geojson_str()
    assert "CircularProvingGround" in raw_str
