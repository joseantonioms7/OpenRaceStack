"""
ApexPathAI - Live Proving Ground Multi-Sensor Stress & Replay Engine.
Provides deterministic edge-case scenario execution (GNSS outages, multipath chaos,
sensor blinding, dynamic slip saturation), circular telemetry recording, and frame-accurate replay.
"""

import uuid
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np


class DisturbanceType(str, Enum):
    GNSS_OUTAGE = "GNSS_OUTAGE"
    URBAN_MULTIPATH = "URBAN_MULTIPATH"
    CAMERA_BLINDING = "CAMERA_BLINDING"
    IMU_BIAS_DRIFT = "IMU_BIAS_DRIFT"
    LIDAR_DEGRADATION = "LIDAR_DEGRADATION"
    FRICTION_DROP = "FRICTION_DROP"


@dataclass
class ScheduledDisturbance:
    """Defines an edge-case disturbance injected at specific track stations or timestamps."""
    disturbance_type: DisturbanceType
    start_station_m: float
    end_station_m: float
    severity_factor: float = 1.0  # e.g., multipath std dev in meters, friction coefficient
    description: str = ""


@dataclass
class StressScenarioDefinition:
    """Pre-configured closed-circuit proving ground stress scenario."""
    scenario_id: str
    name: str
    track_name: str
    description: str
    target_laps: int
    target_speed_mps: float
    disturbances: list[ScheduledDisturbance] = field(default_factory=list)


@dataclass
class TelemetryFrame:
    """Synchronous multi-sensor snapshot for frame-by-frame replay and forensic analysis."""
    frame_index: int
    timestamp: float
    station_s: float
    pose_true: list[float]       # [x, y, yaw]
    pose_est: list[float]        # [x, y, yaw]
    velocity_mps: float
    lateral_error_m: float
    heading_error_deg: float
    gnss_fixed: bool
    gnss_noise_m: float
    camera_blinded: bool
    lidar_inliers_pct: float
    imu_drift_rate: float
    hpl_m: float
    min_margin_m: float
    active_anomalies: list[str]


class StressReplayEngine:
    """
    Executes proving ground stress scenarios, manages scheduled disturbance injection,
    and maintains circular recording buffers for frame-accurate playback and incident extraction.
    """

    def __init__(self, datasets_dir: str | Path = "datasets"):
        self.datasets_dir = Path(datasets_dir)
        if not self.datasets_dir.is_dir():
            self.datasets_dir = Path(__file__).resolve().parent.parent / "datasets"

        self.scenarios: dict[str, StressScenarioDefinition] = self._init_standard_scenarios()
        self.active_scenario: StressScenarioDefinition | None = None
        
        # In-memory recorded buffer for active replay
        self.recorded_frames: list[TelemetryFrame] = []
        self.replay_index: int = 0
        self.is_replaying: bool = False
        self.playback_speed: float = 1.0
        
        # Incident snapshots catalog (incident_id -> 5-second window)
        self.incident_snapshots: dict[str, list[TelemetryFrame]] = {}

    def _init_standard_scenarios(self) -> dict[str, StressScenarioDefinition]:
        """Initializes standard autonomous vehicle proving ground stress scenarios."""
        return {
            "monza_tunnel_blackout": StressScenarioDefinition(
                scenario_id="monza_tunnel_blackout",
                name="Monza High-Speed Tunnel GNSS Blackout",
                track_name="Monza",
                description="10-second complete GNSS signal loss at 80+ km/h evaluating inertial/LiDAR dead-reckoning drift.",
                target_laps=2,
                target_speed_mps=24.5,
                disturbances=[
                    ScheduledDisturbance(
                        disturbance_type=DisturbanceType.GNSS_OUTAGE,
                        start_station_m=120.0,
                        end_station_m=260.0,
                        description="Tunnel Ingress: Complete GNSS Carrier Loss"
                    )
                ]
            ),
            "silverstone_multipath_chaos": StressScenarioDefinition(
                scenario_id="silverstone_multipath_chaos",
                name="Silverstone Multipath & Optical Glare",
                track_name="Silverstone",
                description="Simultaneous urban-canyon multipath jumps (3.5m noise) and forward camera blinding at Copse corner.",
                target_laps=2,
                target_speed_mps=22.0,
                disturbances=[
                    ScheduledDisturbance(
                        disturbance_type=DisturbanceType.URBAN_MULTIPATH,
                        start_station_m=180.0,
                        end_station_m=320.0,
                        severity_factor=3.5,
                        description="Grandstand Multipath Shadow: 3.5m pseudorange noise"
                    ),
                    ScheduledDisturbance(
                        disturbance_type=DisturbanceType.CAMERA_BLINDING,
                        start_station_m=220.0,
                        end_station_m=300.0,
                        description="Direct Low-Sun Glare: Optical feature tracking dropout"
                    )
                ]
            ),
            "spa_raidillon_dynamic_stress": StressScenarioDefinition(
                scenario_id="spa_raidillon_dynamic_stress",
                name="Spa Raidillon Handling & Friction Saturation",
                track_name="Spa",
                description="High lateral acceleration combined with low-friction surface transition testing Kamm limit recovery.",
                target_laps=2,
                target_speed_mps=25.0,
                disturbances=[
                    ScheduledDisturbance(
                        disturbance_type=DisturbanceType.FRICTION_DROP,
                        start_station_m=140.0,
                        end_station_m=280.0,
                        severity_factor=0.65,
                        description="Wet Kerb Ingress: Surface friction coefficient drops to mu=0.65"
                    )
                ]
            ),
            "catalunya_sensor_failover": StressScenarioDefinition(
                scenario_id="catalunya_sensor_failover",
                name="Catalunya Cascading Multi-Sensor Failover",
                track_name="Catalunya",
                description="Sequential sensor degradation (GNSS loss -> Camera blinding -> LiDAR recovery) testing fault isolation.",
                target_laps=2,
                target_speed_mps=20.0,
                disturbances=[
                    ScheduledDisturbance(
                        disturbance_type=DisturbanceType.GNSS_OUTAGE,
                        start_station_m=100.0,
                        end_station_m=220.0,
                        description="Primary GNSS Link Drop"
                    ),
                    ScheduledDisturbance(
                        disturbance_type=DisturbanceType.CAMERA_BLINDING,
                        start_station_m=160.0,
                        end_station_m=250.0,
                        description="Vision Odometry Feature Saturation"
                    ),
                    ScheduledDisturbance(
                        disturbance_type=DisturbanceType.LIDAR_DEGRADATION,
                        start_station_m=200.0,
                        end_station_m=280.0,
                        severity_factor=0.5,
                        description="LiDAR Dust/Spray Obstruction"
                    )
                ]
            )
        }

    def list_scenarios(self) -> list[dict[str, Any]]:
        """Returns catalog of all available proving ground stress scenarios."""
        return [
            {
                "scenario_id": s.scenario_id,
                "name": s.name,
                "track_name": s.track_name,
                "description": s.description,
                "target_laps": s.target_laps,
                "disturbances_count": len(s.disturbances),
                "disturbances": [
                    {
                        "type": d.disturbance_type.value,
                        "start_m": d.start_station_m,
                        "end_m": d.end_station_m,
                        "desc": d.description
                    }
                    for d in s.disturbances
                ]
            }
            for s in self.scenarios.values()
        ]

    def load_scenario(self, scenario_id: str) -> StressScenarioDefinition:
        """Loads and activates a scenario definition."""
        if scenario_id not in self.scenarios:
            raise KeyError(f"Unknown scenario ID '{scenario_id}'")
        self.active_scenario = self.scenarios[scenario_id]
        self.recorded_frames.clear()
        self.replay_index = 0
        self.is_replaying = False
        return self.active_scenario

    def evaluate_active_disturbances(self, station_s: float) -> dict[str, Any]:
        """Evaluates active disturbances for current vehicle track station."""
        flags = {
            "gnss_outage": False,
            "multipath_noise": 0.0,
            "camera_blinded": False,
            "imu_drift_rate": 0.0,
            "lidar_inliers_pct": 94.2,
            "friction_mu": 1.0,
            "active_labels": []
        }

        if not self.active_scenario:
            return flags

        for dist in self.active_scenario.disturbances:
            if dist.start_station_m <= station_s <= dist.end_station_m:
                flags["active_labels"].append(dist.description)
                
                if dist.disturbance_type == DisturbanceType.GNSS_OUTAGE:
                    flags["gnss_outage"] = True
                elif dist.disturbance_type == DisturbanceType.URBAN_MULTIPATH:
                    flags["multipath_noise"] = dist.severity_factor
                elif dist.disturbance_type == DisturbanceType.CAMERA_BLINDING:
                    flags["camera_blinded"] = True
                elif dist.disturbance_type == DisturbanceType.IMU_BIAS_DRIFT:
                    flags["imu_drift_rate"] = dist.severity_factor
                elif dist.disturbance_type == DisturbanceType.LIDAR_DEGRADATION:
                    flags["lidar_inliers_pct"] = 94.2 * dist.severity_factor
                elif dist.disturbance_type == DisturbanceType.FRICTION_DROP:
                    flags["friction_mu"] = dist.severity_factor

        return flags

    def record_frame(self, frame: TelemetryFrame):
        """Appends frame to recording buffer and tracks safety incident snapshots."""
        self.recorded_frames.append(frame)
        if len(self.recorded_frames) > 5000:
            self.recorded_frames.pop(0)

        # Check if frame has critical anomaly (TTB < 0.8s or boundary breach)
        if frame.active_anomalies:
            incident_id = str(uuid.uuid4())[:8]
            # Capture +/- 50 frames (approx 2.5s before and after)
            start_idx = max(0, len(self.recorded_frames) - 50)
            self.incident_snapshots[incident_id] = list(self.recorded_frames[start_idx:])

    def seek_frame(self, frame_idx: int) -> TelemetryFrame | None:
        """Seeks replay to specific frame index."""
        if not self.recorded_frames:
            return None
        self.replay_index = int(np.clip(frame_idx, 0, len(self.recorded_frames) - 1))
        return self.recorded_frames[self.replay_index]

    def get_timeline_markers(self) -> list[dict[str, Any]]:
        """Returns timeline event markers for interactive scrubber visualization."""
        markers = []
        if not self.recorded_frames:
            return markers

        for i, f in enumerate(self.recorded_frames):
            if f.active_anomalies:
                markers.append({
                    "frame_index": i,
                    "timestamp": f.timestamp,
                    "station_s": f.station_s,
                    "type": "ANOMALY",
                    "label": f.active_anomalies[0]
                })
            elif not f.gnss_fixed and (i == 0 or self.recorded_frames[i - 1].gnss_fixed):
                # Mark GNSS loss
                markers.append({
                        "frame_index": i,
                        "timestamp": f.timestamp,
                        "station_s": f.station_s,
                        "type": "OUTAGE",
                        "label": "GNSS Outage Start"
                    })
        return markers

    def get_incident_snapshot(self, incident_id: str) -> list[dict[str, Any]] | None:
        """Returns serialized frames for safety incident review."""
        if incident_id not in self.incident_snapshots:
            return None
        return [f.__dict__ for f in self.incident_snapshots[incident_id]]
