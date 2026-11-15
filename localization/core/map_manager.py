"""
Map Management Layer: Loads, indexes, and queries 2D occupancy grid maps (*_map.yaml, *_map.png),
curvilinear racelines (*_raceline.csv), and track boundary corridors (*_centerline.csv).
Provides spatial KD-tree spatial queries and distance field lookups for scan matching and corridor constraints.
"""

import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree


class MapManager:
    """
    Unified map manager managing:
    1. Metric occupancy grid map raster & Euclidean Distance Transform (EDT).
    2. Spatial KD-Tree over circuit waypoints / boundaries for fast O(log N) projection.
    3. Optimal raceline speed and curvature profiles.
    """

    def __init__(self, track_folder_path: str | Path):
        self.folder_path = Path(track_folder_path)
        if not self.folder_path.is_dir():
            raise FileNotFoundError(f"Track directory not found: {self.folder_path}")

        self.track_name = self.folder_path.name
        self.occupancy_grid: np.ndarray | None = None
        self.distance_field: np.ndarray | None = None
        self.resolution: float = 0.05
        self.origin: np.ndarray = np.array([0.0, 0.0, 0.0])  # [x0, y0, theta0]
        self.grid_width: int = 0
        self.grid_height: int = 0

        # Geometric boundary data
        self.waypoints: np.ndarray | None = None  # (N, 2)
        self.track_widths: np.ndarray | None = None  # (N, 2) [w_right, w_left]
        self.raceline_data: dict[str, np.ndarray] | None = None
        self.kdtree: cKDTree | None = None

        self._load_map_yaml_and_png()
        self._load_centerline_and_raceline()

    def _load_map_yaml_and_png(self) -> None:
        yaml_files = list(self.folder_path.glob("*_map.yaml"))
        if not yaml_files:
            return

        yaml_path = yaml_files[0]
        meta: dict[str, str] = {}
        with open(yaml_path, encoding='utf-8') as f:
            for line in f:
                if ':' in line:
                    key, val = line.split(':', 1)
                    meta[key.strip()] = val.strip()

        image_filename = meta.get("image", f"{self.track_name}_map.png")
        image_path = self.folder_path / image_filename
        self.resolution = float(meta.get("resolution", 0.05))

        origin_str = meta.get("origin", "[-50.0, -50.0, 0.0]").strip("[]")
        self.origin = np.array([float(x.strip()) for x in origin_str.split(",")], dtype=np.float64)

        if image_path.is_file():
            # Load raster image (PNG)
            img = Image.open(image_path).convert("L")
            img_arr = np.array(img, dtype=np.uint8)
            self.grid_height, self.grid_width = img_arr.shape
            # Occupancy probability: 0 = free, 255 = occupied
            occupied_thresh = float(meta.get("occupied_thresh", 0.45)) * 255

            # Binary obstacle mask: occupied pixels are 1, free are 0
            # Note: in ROS map_server standard, white (255) is free, black (0) is occupied
            negate = int(meta.get("negate", 0))
            if negate == 0:
                is_obstacle = (img_arr < occupied_thresh).astype(np.uint8)
            else:
                is_obstacle = (img_arr > (255 - occupied_thresh)).astype(np.uint8)

            self.occupancy_grid = is_obstacle
            # Euclidean Distance Transform (EDT) gives distance to nearest obstacle in meters
            dist_pixels = distance_transform_edt(1 - is_obstacle)
            self.distance_field = dist_pixels * self.resolution

    def _load_centerline_and_raceline(self) -> None:
        import pandas as pd

        cl_files = list(self.folder_path.glob("*_centerline.csv"))
        if cl_files:
            df_cl = pd.read_csv(cl_files[0], skipinitialspace=True)
            df_cl.columns = df_cl.columns.str.replace('#', '', regex=False).str.strip()
            x = df_cl['x_m'].to_numpy(dtype=np.float64)
            y = df_cl['y_m'].to_numpy(dtype=np.float64)
            self.waypoints = np.ascontiguousarray(np.column_stack((x, y)), dtype=np.float64)
            if 'w_tr_right_m' in df_cl.columns and 'w_tr_left_m' in df_cl.columns:
                w_r = df_cl['w_tr_right_m'].to_numpy(dtype=np.float64)
                w_l = df_cl['w_tr_left_m'].to_numpy(dtype=np.float64)
                self.track_widths = np.ascontiguousarray(np.column_stack((w_r, w_l)), dtype=np.float64)
            self.kdtree = cKDTree(self.waypoints)

        rl_files = list(self.folder_path.glob("*_raceline.csv"))
        if rl_files:
            # Detect header index
            with open(rl_files[0], encoding='utf-8') as f:
                head = [f.readline() for _ in range(5)]
            hdr_idx = next((i for i, line in enumerate(head) if ';' in line and ('s_m' in line or 'x_m' in line)), 0)
            df_rl = pd.read_csv(rl_files[0], sep=';', header=hdr_idx, skipinitialspace=True)
            df_rl.columns = df_rl.columns.str.replace('#', '', regex=False).str.strip()
            if {'s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps'}.issubset(set(df_rl.columns)):
                self.raceline_data = {
                    's_m': df_rl['s_m'].to_numpy(dtype=np.float64),
                    'x_m': df_rl['x_m'].to_numpy(dtype=np.float64),
                    'y_m': df_rl['y_m'].to_numpy(dtype=np.float64),
                    'psi_rad': df_rl['psi_rad'].to_numpy(dtype=np.float64),
                    'kappa_radpm': df_rl['kappa_radpm'].to_numpy(dtype=np.float64),
                    'vx_mps': df_rl['vx_mps'].to_numpy(dtype=np.float64),
                }

    def world_to_map(self, x: float, y: float) -> tuple[int, int]:
        """Converts global metric coordinates [x, y] to occupancy grid pixel coordinates [col, row]."""
        c = int(math.floor((x - self.origin[0]) / self.resolution))
        # Note image coordinate row 0 is top
        r = self.grid_height - 1 - int(math.floor((y - self.origin[1]) / self.resolution))
        return c, r

    def get_obstacle_distance(self, x: float, y: float) -> float:
        """Returns metric distance to the nearest mapped barrier or wall."""
        if self.distance_field is None:
            return 10.0
        c, r = self.world_to_map(x, y)
        if 0 <= r < self.grid_height and 0 <= c < self.grid_width:
            return float(self.distance_field[r, c])
        return 0.0

    def query_nearest_waypoint(self, x: float, y: float) -> tuple[int, float, np.ndarray]:
        """Fast O(log N) spatial query returning (nearest_index, distance_to_centerline, [x_cl, y_cl])."""
        if self.kdtree is None or self.waypoints is None:
            return 0, 0.0, np.array([x, y])
        dist, idx = self.kdtree.query([x, y])
        return int(idx), float(dist), self.waypoints[idx]

    def query_corridor_bounds(self, x: float, y: float) -> tuple[float, float, float, np.ndarray]:
        """
        Returns (cross_track_offset, max_allowable_left, max_allowable_right, centerline_point).
        Cross-track offset > 0 means vehicle is to the left of centerline.
        """
        idx, _, cl_pt = self.query_nearest_waypoint(x, y)
        if self.waypoints is None or len(self.waypoints) < 2:
            return 0.0, 1.1, 1.1, cl_pt

        next_idx = (idx + 1) % len(self.waypoints)
        dx = self.waypoints[next_idx, 0] - cl_pt[0]
        dy = self.waypoints[next_idx, 1] - cl_pt[1]
        psi = math.atan2(dy, dx)

        # Vector from centerline point to vehicle
        vx = x - cl_pt[0]
        vy = y - cl_pt[1]
        # Normal pointing to the left (-sin, cos)
        nx = -math.sin(psi)
        ny = math.cos(psi)
        cross_track = vx * nx + vy * ny

        w_r = 1.1
        w_l = 1.1
        if self.track_widths is not None:
            w_r = float(self.track_widths[idx, 0])
            w_l = float(self.track_widths[idx, 1])

        return cross_track, w_l, w_r, cl_pt
