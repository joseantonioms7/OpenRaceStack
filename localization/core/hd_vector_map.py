"""
High-Definition (HD) Vector Map Engine and Track Boundary Geometry for OpenRaceStack.

Provides sub-centimeter vector track geometry: centerline, barrier boundaries, kerbs,
sector splits, speed traps, and GeoJSON export for Mapbox GL vector tiles.
"""

import json
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class TrackSector:
    """Designated timing sector on race circuit (S1, S2, S3)."""
    sector_id: int               # Sector number (1, 2, or 3)
    start_s: float               # Sector entry arc-length (m)
    end_s: float                 # Sector exit arc-length (m)
    speed_trap_s: float | None = None  # Speed trap marker location (m)


@dataclass
class TrackKerb:
    """Physical race track curb / rumble strip geometry."""
    kerb_type: str               # "apex_kerb", "exit_kerb", "rumble_strip"
    side: str                    # "left" or "right"
    start_s: float
    end_s: float
    polygon: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float64))


class HDVectorMap:
    """
    Sub-centimeter vector track map with spatial KD-Tree queries,
    safety corridor boundaries, sector management, and GeoJSON export.
    """

    def __init__(
        self,
        track_name: str,
        centerline: np.ndarray,
        widths: np.ndarray,      # (N, 2) [w_right, w_left]
        is_closed: bool = True,
    ):
        self.track_name = track_name
        self.centerline = np.ascontiguousarray(centerline, dtype=np.float64)
        self.widths = np.ascontiguousarray(widths, dtype=np.float64)
        self.is_closed = is_closed
        self.n_pts = len(centerline)

        # 1. Compute arc-length s and normals
        diffs = np.zeros_like(centerline)
        for i in range(self.n_pts):
            next_idx = (i + 1) % self.n_pts if is_closed else min(i + 1, self.n_pts - 1)
            diffs[i] = centerline[next_idx] - centerline[i]

        seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
        seg_lengths = np.where(seg_lengths < 1e-6, 1e-6, seg_lengths)

        self.s = np.zeros(self.n_pts, dtype=np.float64)
        for i in range(1, self.n_pts):
            self.s[i] = self.s[i - 1] + seg_lengths[i - 1]

        self.total_length = float(self.s[-1] + seg_lengths[-1]) if is_closed else float(self.s[-1])

        # Tangents and unit normals
        tangents = diffs / seg_lengths[:, None]
        self.headings = np.arctan2(tangents[:, 1], tangents[:, 0])
        self.normals = np.column_stack((-np.sin(self.headings), np.cos(self.headings)))

        # 2. Extrude outer (left) and inner (right) track boundaries
        # Left boundary = centerline + w_left * normal
        # Right boundary = centerline - w_right * normal
        self.left_boundary = self.centerline + self.widths[:, 1:2] * self.normals
        self.right_boundary = self.centerline - self.widths[:, 0:1] * self.normals

        # KD-Tree for O(log N) projections
        self.centerline_tree = cKDTree(self.centerline)
        self.boundary_tree = cKDTree(np.vstack((self.left_boundary, self.right_boundary)))

        # 3. Partition circuit into 3 standard sectors
        s33 = self.total_length / 3.0
        s66 = 2.0 * s33
        self.sectors = [
            TrackSector(sector_id=1, start_s=0.0, end_s=s33, speed_trap_s=s33 * 0.8),
            TrackSector(sector_id=2, start_s=s33, end_s=s66, speed_trap_s=s66 * 0.9),
            TrackSector(sector_id=3, start_s=s66, end_s=self.total_length, speed_trap_s=self.total_length * 0.95),
        ]

        # Kerbs catalog
        self.kerbs: list[TrackKerb] = []
        self._generate_apex_kerbs()

    def _generate_apex_kerbs(self) -> None:
        """Identifies curvature peaks and adds apex kerb polygons."""
        # Simple apex detection from heading changes
        d_headings = np.abs(np.diff(self.headings))
        peaks = np.where(d_headings > np.percentile(d_headings, 85))[0]

        for p_idx in peaks[:10]: # Up to 10 prominent apexes
            s_val = self.s[p_idx]
            side = "left" if self.headings[p_idx] > 0 else "right"
            kerb = TrackKerb(
                kerb_type="apex_kerb",
                side=side,
                start_s=max(0.0, s_val - 10.0),
                end_s=min(self.total_length, s_val + 10.0),
            )
            self.kerbs.append(kerb)

    def get_corridor_bounds(self, s_query: float) -> tuple[float, float]:
        """
        Returns interpolated (w_right, w_left) track limits at arc-length s.
        """
        s_norm = s_query % self.total_length if self.total_length > 1e-3 else s_query
        idx = int(np.searchsorted(self.s, s_norm, side="right") - 1)
        idx = max(0, min(self.n_pts - 2, idx))

        s0 = self.s[idx]
        s1 = self.s[idx + 1] if idx + 1 < self.n_pts else self.total_length
        ratio = (s_norm - s0) / max(1e-4, s1 - s0)
        ratio = float(np.clip(ratio, 0.0, 1.0))

        w_r = float((1.0 - ratio) * self.widths[idx, 0] + ratio * self.widths[idx + 1, 0])
        w_l = float((1.0 - ratio) * self.widths[idx, 1] + ratio * self.widths[idx + 1, 1])
        return w_r, w_l

    def project_point(self, x: float, y: float) -> tuple[float, float, float]:
        """
        Projects point (x, y) onto centerline.
        Returns: (s, signed_cross_track_error, tangent_heading).
        """
        pt = np.array([x, y], dtype=np.float64)
        dist, idx = self.centerline_tree.query(pt)

        p_near = self.centerline[idx]
        tangent_h = self.headings[idx]
        norm = self.normals[idx]

        d_pos = pt - p_near
        e_y = float(np.dot(d_pos, norm))
        s_val = float(self.s[idx])

        return s_val, e_y, float(tangent_h)

    def distance_to_boundary(self, x: float, y: float) -> float:
        """
        Computes signed clearance to closest boundary.
        Positive = vehicle is inside track corridor.
        Negative = vehicle has violated boundary (off-track).
        """
        s_val, e_y, _ = self.project_point(x, y)
        w_r, w_l = self.get_corridor_bounds(s_val)

        # e_y is positive to left, negative to right
        dist_to_left = w_l - e_y
        dist_to_right = w_r + e_y
        return float(min(dist_to_left, dist_to_right))

    def is_inside_track(self, x: float, y: float, safety_margin: float = 0.0) -> bool:
        """Checks whether point is strictly inside track corridor."""
        return self.distance_to_boundary(x, y) >= safety_margin

    def get_sector(self, s_query: float) -> int:
        """Returns active timing sector number (1, 2, or 3) for given arc-length."""
        s_norm = s_query % self.total_length if self.total_length > 1e-3 else s_query
        for sec in self.sectors:
            if sec.start_s <= s_norm < sec.end_s:
                return sec.sector_id
        return 3

    def to_geojson(self) -> dict[str, Any]:
        """
        Exports HD vector geometry as GeoJSON FeatureCollection for Mapbox GL.
        """
        features = [
            {
                "type": "Feature",
                "properties": {"layer": "centerline", "track_name": self.track_name},
                "geometry": {
                    "type": "LineString",
                    "coordinates": self.centerline[:, :2].tolist(),
                },
            },
            {
                "type": "Feature",
                "properties": {"layer": "left_boundary", "barrier_type": "armco"},
                "geometry": {
                    "type": "LineString",
                    "coordinates": self.left_boundary[:, :2].tolist(),
                },
            },
            {
                "type": "Feature",
                "properties": {"layer": "right_boundary", "barrier_type": "armco"},
                "geometry": {
                    "type": "LineString",
                    "coordinates": self.right_boundary[:, :2].tolist(),
                },
            },
        ]

        # Sector split lines
        for sec in self.sectors:
            s_val = sec.start_s
            idx = int(np.searchsorted(self.s, s_val, side="right") - 1)
            idx = max(0, min(self.n_pts - 1, idx))
            p_l = self.left_boundary[idx].tolist()
            p_r = self.right_boundary[idx].tolist()
            features.append({
                "type": "Feature",
                "properties": {"layer": "sector_gate", "sector": sec.sector_id},
                "geometry": {"type": "LineString", "coordinates": [p_l, p_r]},
            })

        return {"type": "FeatureCollection", "features": features}

    def to_geojson_str(self) -> str:
        """Serializes GeoJSON FeatureCollection to string."""
        return json.dumps(self.to_geojson(), indent=2)
