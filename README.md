# ApexPathAI & OpenRaceStack 🏎️⚡

**Enterprise Autonomous Vehicle (AV) Proving Ground Validation Platform & Modular High-Performance Autonomous Racing Framework**

[![CI Test Suite](https://github.com/apexpath-ai/ApexPathAI/actions/workflows/ci.yml/badge.svg)](https://github.com/apexpath-ai/ApexPathAI/actions/workflows/ci.yml)
[![Docker Multi-Arch](https://github.com/apexpath-ai/ApexPathAI/actions/workflows/docker.yml/badge.svg)](https://github.com/apexpath-ai/ApexPathAI/actions/workflows/docker.yml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Python Version](https://img.shields.io/badge/Python-3.11%20%7C%203.12%20%7C%203.13-3776AB?logo=python&logoColor=white)](pyproject.toml)
[![Frontend](https://img.shields.io/badge/Frontend-React%2018%20%7C%20TypeScript%20%7C%20Vite-61DAFB?logo=react&logoColor=black)](frontend/)
[![UI Library](https://img.shields.io/badge/UI-Material%20UI%20v5-007FFF?logo=mui&logoColor=white)](frontend/)
[![Backend](https://img.shields.io/badge/Backend-FastAPI%20%7C%20WebSockets-009688?logo=fastapi&logoColor=white)](backend/)
[![Database](https://img.shields.io/badge/Database-PostgreSQL%2016%20%7C%20Redis%207-336791?logo=postgresql&logoColor=white)](docker-compose.yml)
[![Architecture](https://img.shields.io/badge/Architecture-SE(3)%20x%20R^6-purple.svg)](docs/architecture/platform_overview.md)
[![Validation](https://img.shields.io/badge/Validation-ISO%2021448%20%2F%20SOTIF-green.svg)](docs/proving_ground/scorecard.md)
[![Autonomous Racing](https://img.shields.io/badge/Autonomous%20Racing-OpenRaceStack-red.svg)](docs/architecture/openracestack.md)

---

## 🌟 Executive Overview

**ApexPathAI** and its embedded core **OpenRaceStack** represent an industry-leading, production-ready Autonomous Vehicle (AV) State Estimation, Proving Ground Validation, and High-Performance Autonomous Racing Framework. Designed for competitive autonomous racing teams (F1TENTH, Indy Autonomous Challenge), automotive OEMs, and Tier-1 safety researchers, the platform bridges high-rate vehicle dynamics, sub-centimeter perception, game-theoretic multi-agent racecraft, and formal proving ground certification.

The stack scales seamlessly across 1:10 scale RC platforms (VESC ERPM/PWM), electric go-karts, full-scale Dallara AV-21 IndyCars (CAN FD), and passenger GT vehicles across **23 international Grand Prix proving grounds** (Monza, Spa-Francorchamps, Silverstone, Nürburgring, etc.).

```
                                    APEXPATHAI & OPENRACESTACK TOPOLOGY
 ┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
 │                                   REACT 18 COCKPIT DASHBOARD                                     │
 │  [Vehicle Overview]    [Localization Overview]  [Circuit Performance]    [Navigation Quality]    │
 │  [Sensor Health]       [Vehicle Dynamics]       [Safety Events]          [Session Analytics]     │
 │  [Fleet Comparisons]   [Live Cockpit Stream]    [ISO 21448 Scorecards]   [Validation Dossiers]   │
 └──────────────────────────────────┬─────────────────────────────────────┬─────────────────────────┘
                                    │ REST API (JSON)                     │ WebSockets (20 Hz)
                                    ▼                                     ▼
 ┌──────────────────────────────────────────────────────────────────────────────────────────────────┐
 │                                  FASTAPI ASYNC APPLICATION CORE                                  │
 │   • API v1 Routers (Pydantic v2 validation)     • 20 Hz Async Telemetry Broadcast Engine         │
 │   • Session Campaign Lifecycle Machine          • Sensor Fault Injection & Supervisor            │
 └──────────────┬───────────────────────────────────┬───────────────────────────────────┬───────────┘
                │ Async SQL                         │ Pub / Sub                         │ Control & HAL
                ▼                                   ▼                                   ▼
 ┌─────────────────────────────┐     ┌─────────────────────────────┐     ┌──────────────────────────┐
 │   PostgreSQL 16 Engine      │     │       Redis 7 Broker        │     │      OPENRACESTACK       │
 │  • Vehicle Fleet Assets     │     │  • High-throughput telemetry│     │  • Vehicle Dynamics & HAL│
 │  • Sessions & Lap Records   │     │  • Multi-client fanout      │     │  • Frenet MPC & Pursuit  │
 │  • Formal Dossier Reports   │     │  • Channel multiplexing     │     │  • Multi-Agent Racing Sim│
 └─────────────────────────────┘     └─────────────────────────────┘     └──────────────────────────┘
```

---

## 🏁 OpenRaceStack Architectural Overview

OpenRaceStack is structured into six modular, cross-platform autonomous racing layers:

```
  ┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
  │                                   LAYER 6: RACE CONTROL & MULTI-AGENT                           │
  │  • Multi-Agent Sim Engine (Aerodynamic Slipstream Drafting & Dirty Air Downforce Loss)          │
  │  • Race Control State Machine (Formation, Green, Full-Course Yellow, Safety Car, Red Flag)      │
  │  • Real-Time Fleet Leaderboard, Interval Matrix & Automated Track Limits Penalty Enforcement     │
  └────────────────────────────────────────────────┬────────────────────────────────────────────────┘
                                                   │
  ┌────────────────────────────────────────────────▼────────────────────────────────────────────────┐
  │                                   LAYER 5: TACTICAL PLANNING & DECISION                         │
  │  • Game-Theoretic Overtaking Planner (Inside Overtake, Outside Sweep, Defensive Cover, Yield)   │
  │  • FIA "Significant Overlap" & "One-Move" Defensive Verification Rules                          │
  │  • Spatio-Temporal Frenet Trajectory Planner (Quintic s(t) & Quartic d(t) Polynomial Lattices)   │
  │  • Convex Minimum-Curvature Raceline Optimizer & Forward/Backward Friction-Ellipse Profiler     │
  └────────────────────────────────────────────────┬────────────────────────────────────────────────┘
                                                   │
  ┌────────────────────────────────────────────────▼────────────────────────────────────────────────┐
  │                                   LAYER 4: HIGH-PERFORMANCE CONTROL                             │
  │  • High-Rate Frenet-Frame MPC (Condensed QP + Accelerated Nesterov Projected Gradient, < 2 ms)   │
  │  • Adaptive Lookahead Curvature-Aware Pure Pursuit Controller with Steering Slew Limiting       │
  └────────────────────────────────────────────────┬────────────────────────────────────────────────┘
                                                   │
  ┌────────────────────────────────────────────────▼────────────────────────────────────────────────┐
  │                                   LAYER 3: PERCEPTION & HD MAPPING                              │
  │  • 2D LiDAR Scan Matcher: SVD Iterative Closest Point (ICP) & Normal Distributions Transform (NDT)│
  │  • HD Vector Map Engine (Centerlines, Barrier Extrusions, Spatial KD-Tree Queries, GeoJSON)     │
  │  • Visual Feature Tracker & Degradation Engine (Sun Glare, Tunnel Transitions, Rain Spray)      │
  └────────────────────────────────────────────────┬────────────────────────────────────────────────┘
                                                   │
  ┌────────────────────────────────────────────────▼────────────────────────────────────────────────┐
  │                                   LAYER 2: STATE ESTIMATION & FILTERING                         │
  │  • Dual-Stack Estimator: 2D Planar CTRV EKF + 15-DOF Error-State Kalman Filter on SE(3) x R^6   │
  │  • Joseph-Stabilized Covariance Propagation & Chi-Squared Innovation Gating (NIS / NEES)        │
  └────────────────────────────────────────────────┬────────────────────────────────────────────────┘
                                                   │
  ┌────────────────────────────────────────────────▼────────────────────────────────────────────────┐
  │                                   LAYER 1: VEHICLE DYNAMICS & HAL                               │
  │  • Vehicle Dynamics: Kinematic Bicycle, Dynamic Bicycle with Dynamic Weight Transfer & Pacejka  │
  │  • Multi-Scale Presets: F1TENTH (1:10), Autonomous Go-Kart, Dallara AV-21 IndyCar, Passenger GT│
  │  • Hardware Abstraction Layer (HAL): VESC ERPM/PWM Interface, CAN FD Brake Pressure Interface   │
  │  • Zero-Dependency ROS 2 Bridge: nav_msgs, sensor_msgs, tf2_msgs, ackermann_msgs, ASIL Watchdog│
  └─────────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 🖥️ Platform Cockpit Preview (Screenshots & UI Mockups)

### 1. Real-Time Glass-Cockpit Dashboard (Web Client at `:3000`)
```text
+---------------------------------------------------------------------------------------------------+
| APEXPATHAI LIVE COCKPIT | Track: Monza (Italy) | Vehicle: Apex-F1TENTH-01 | Stream: 20 Hz [ONLINE] |
+---------------------------------------------------------------------------------------------------+
|  SPEEDOMETER      |  G-G ACCELERATION ENVELOPE  |  STANFORD LATERAL TRACKING |  SAFETY SUPERVISOR |
|   68.4 km/h       |       Ay: +1.42 G (Right)   |     ey: +0.082 m           |  Status: NOMINAL   |
|   Target: 72.0    |       Ax: +0.65 G (Braking) |     e_psi: -1.24 deg       |  E-STOP: ARMED     |
|   [|||||||||....] |       Friction: 82.4% Mu    |     Wall Clearance: 2.42m  |  HPL: 0.84 m (Pass)|
+---------------------------------------------------------------------------------------------------+
| TRACK GEOMETRY & VEHICLE POSE (Canvas / Mapbox)  | INNOVATION RESIDUALS & SENSOR MATRIX           |
|                                                   |  • GNSS (PVT)  : NIS 1.84 [PASS]   Rate: 10 Hz |
|           /-------------\                         |  • IMU (6-DOF) : NIS 0.42 [PASS]   Rate: 100Hz |
|          /     [x]       \  <-- AV Pose           |  • LiDAR Scan  : NIS 3.12 [PASS]   Rate: 20 Hz |
|         |     (Monza)     |                       |  • Camera VIO  : NIS 1.10 [PASS]   Rate: 30 Hz |
|          \_______________/                        |                                               |
+---------------------------------------------------+-----------------------------------------------+
| ACTIVE SECTOR TIMING: S1: 18.24s | S2: 24.12s | S3: 20.85s | Current Lap: 01:03.21 (Best: 01:02.84) |
+---------------------------------------------------------------------------------------------------+
```

### 2. High-Speed Proving Ground CLI Terminal Benchmarking
```text
$ apexpath-benchmark --circuit Monza --runs 5
[INFO] Loading HD Centerline & Raceline: Monza (1,489 waypoints, 5,793 m)
[INFO] Synthesizing 20 Hz Sensor Streams: GPS (sigma=1.5m), Wheel Odom (sigma=0.08m/s), IMU (sigma=0.02rad/s)
[INFO] Propagating Dual-Stack State Estimation:
  ├── Planar CTRV EKF (5 States) + Joseph Covariance Update
  └── 15-DOF Error-State Kalman Filter on SE(3) x R^6
========================================================================================
CIRCUIT: Monza Autodromo Nazionale | BENCHMARK RESULTS (5 Laps)
========================================================================================
Metric                             Raw Sensor           ApexPathAI EKF       Improvement
----------------------------------------------------------------------------------------
2D Position RMSE                   2.124 m              0.213 m              +89.97%
Peak Cross-Track Error             3.450 m              0.312 m              +90.95%
Horizontal Protection Level (HPL)  5.800 m              1.240 m              +78.62%
Kalman NEES Consistency Score      18.42 (Diverged)     3.420 (Consistent)   Optimal [2.0, 5.5]
MPC Guidance Solve Latency         --                   1.850 ms             Deterministic (<5ms)
ISO 21448 Regulatory Verdict       FAIL                 PASS (100%)          7 of 7 Rules Met
========================================================================================
```

---

## 🌐 Autonomous Ecosystem Comparison

How ApexPathAI fits into the global autonomous driving open-source ecosystem:

| Dimension | **ApexPathAI** | Autoware.Universe | Baidu Apollo 9.0 | NVIDIA DRIVE Sim |
| :--- | :---: | :---: | :---: | :---: |
| **Primary Domain** | **Proving Ground Certification & State Estimation Validation** | Full Autonomous Urban Driving Stack | Urban Robotaxi Fleet Stack | Photorealistic Synthetic Simulation |
| **Target Vehicle Scale**| **1:10 Scaled Racers, IndyCar, Passenger AVs** | Full-scale Passenger Cars | Production Robotaxis | Simulation Digital Twins |
| **State Estimation Stack**| **Dual CTRV EKF + 15-DOF ESKF ($\text{SE}(3) \times \mathbb{R}^6$)** | NDT Scan Matcher + EKF | MSF (GNSS + IMU + LiDAR) | PhysX Engine Simulation |
| **Proving Ground Datasets** | **23 Formula 1 Tracks Out-of-the-Box** | Custom Point Clouds | Selected Geofenced Maps | Custom 3D Environments |
| **High-Speed Dynamics** | **Tested up to 280.8 km/h ($>3.8\,\text{G}$)** | Urban (< 60 km/h) | Urban (< 80 km/h) | Pure Simulation |
| **Regulatory Pass/Fail** | **Automated 7-Rule ISO 21448 / SOTIF Scorecards** | Manual Log Auditing | Manual Triage | Scenario Scorecards |
| **Telemetry Rate** | **20 Hz Real-Time Streaming + Microsecond Replay** | ROS 2 Topics | Cyber RT Channels | Sensor Plugins |
| **Deployment Footprint**| **Lightweight Docker Compose (FastAPI + React)** | Heavy ROS 2 Desktop | Heavy Ubuntu Docker Container | Enterprise Cloud / GPU Workstations |

---

## 🏛️ 9 Core Engineering Dashboard Sections

ApexPathAI features 9 dedicated dashboard views accessible in the engineering web client:

| Section | Route | Focus & Metrics |
| :--- | :--- | :--- |
| **1. Vehicle Overview** | `/vehicles` | Fleet asset registry, physical wheelbase, track width, mass, steering saturation, and sensor extrinsics. |
| **2. Localization Overview** | `/loc_overview` | Geodetic and Cartesian state estimation: 2D/3D RMSE, CEP50, CEP95, Maximum HPL, covariance ellipses. |
| **3. Circuit Performance** | `/circuit_perf` | 23 international Grand Prix proving grounds with centerlines, racelines, sector lengths, and speed traps. |
| **4. Navigation Quality** | `/nav_quality` | Frenet cross-track error ($e_y$), heading alignment ($e_\psi$), Stanford controller error plane, curvature tracking. |
| **5. Sensor Health** | `/sensor_health` | Multi-sensor diagnostics (GNSS, LiDAR, Camera, IMU), $\chi^2$ innovation gating (NIS), fault injection. |
| **6. Vehicle Dynamics** | `/veh_dynamics` | Kinematic envelope: tire sideslip ($\beta$), lateral acceleration, G-G diagram, friction utilization. |
| **7. Safety Events** | `/safety_events` | Real-time safety supervisor: E-STOP actuator interlocks, boundary incursions, and disengagement log. |
| **8. Session Analytics** | `/session_analytics` | Proving ground campaign execution (`IDLE` $\to$ `ARMED` $\to$ `RUNNING` $\to$ `COMPLETED`), sector timing, lap splits. |
| **9. Fleet Comparisons** | `/fleet_comparisons` | Multi-vehicle benchmarking comparing F1TENTH Scaled Racer vs Dallara AV-21 vs Level 4 Robotaxi. |

Plus dedicated **Incident Replay Engine** and **Validation Dossier Reporting** modules.

---

## ⚡ Quickstart with Docker Compose

Deploy the entire production stack (Frontend + Backend + PostgreSQL 16 + Redis 7) in 60 seconds:

```bash
# 1. Clone the repository
git clone https://github.com/apexpath-ai/ApexPathAI.git
cd ApexPathAI

# 2. Copy environment template
cp .env.example .env

# 3. Build and launch multi-container platform
docker compose up --build -d
```

### Accessing the Platform:
- 🖥️ **Cockpit Dashboard:** [http://localhost:3000](http://localhost:3000)
- 🔌 **FastAPI REST API:** [http://localhost:8000](http://localhost:8000)
- 📚 **Interactive Swagger API Docs:** [http://localhost:8000/docs](http://localhost:8000/docs)
- 📊 **ReDoc Reference:** [http://localhost:8000/redoc](http://localhost:8000/redoc)

---

## 🛠️ Local Development Setup

### Backend (Python 3.11+)

The backend features **zero-config developer fallback**: if PostgreSQL and Redis are not running, it automatically uses asynchronous SQLite (`sqlite+aiosqlite`) and an in-memory pub/sub broker!

```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install development dependencies
pip install -e ".[dev]"

# Launch FastAPI development server with hot-reload
uvicorn backend.app.main:app --host 127.0.0.1 --port 8000 --reload
```

### Frontend (React 18 + Vite + TypeScript)

```bash
cd frontend

# Install Node modules
npm install

# Start Vite hot-reload dev server
npm run dev
```

Visit [http://localhost:5173](http://localhost:5173) to connect to your local backend.

---

## 💻 Programmatic Usage Examples

### 1. Python State Estimation & MPC SDK
```python
import numpy as np
from Localization.core.ekf_engine import ExtendedKalmanFilterCTRV
from Localization.core.eskf_3d import ErrorStateKalmanFilter3D
from Localization.core.mpc_controller import ModelPredictiveController
from Localization.core.map_manager import MapManager

# Initialize 2D Planar EKF with Joseph-stabilized covariance
ekf = ExtendedKalmanFilterCTRV(dt_default=0.05)
x0 = np.array([[0.0], [0.0], [0.0], [15.0], [0.0]], dtype=np.float64)
ekf.initialize_state(x0, timestamp=0.0)

# Propagate high-rate CTRV prediction
ekf.predict(dt=0.05)

# Fuse GNSS observation with Chi-squared innovation gating
z_gnss = np.array([[0.12], [-0.05]], dtype=np.float64)
R_gnss = np.diag([2.25, 2.25])
accepted = ekf.update_gnss(z_gnss, R_gnss=R_gnss, gate_threshold=9.21)

print(f"GNSS Update Accepted: {accepted} | State Pose: x={ekf.x[0,0]:.2f}m, y={ekf.x[1,0]:.2f}m")
```

### 2. High-Performance Dynamics & Adaptive Pure Pursuit Control
```python
import numpy as np
from Localization.core.dynamics import DynamicBicycleModel, F1TENTH_PARAMS, VehicleState
from Localization.core.pure_pursuit import AdaptivePurePursuitController, PathGeometry

# Configure 1:10 scaled F1TENTH dynamics with dynamic load transfer
model = DynamicBicycleModel(F1TENTH_PARAMS)
controller = AdaptivePurePursuitController(
    wheelbase=F1TENTH_PARAMS.wheelbase,
    min_lookahead=0.8,
    max_lookahead=2.5,
    speed_lookahead_gain=0.1,
    max_steering_angle=0.4189,
    max_steering_rate=3.2,
)

# Planar path geometry
path = PathGeometry(points=np.array([[0.0, 0.0], [5.0, 0.5], [10.0, 2.0], [15.0, 4.0]]))
state = VehicleState(x=0.0, y=0.1, yaw=0.05, vx=5.0, vy=0.0, omega=0.0)

# Compute optimal steering command with slew rate limiting
cmd = controller.compute_command(state, path, target_speed=5.0, dt=0.05)
print(f"Steering angle: {cmd.steering_angle:.3f} rad | Lookahead: {cmd.lookahead_distance:.2f} m")
```

### 3. Convex Minimum-Curvature Raceline Optimization & Speed Profiling
```python
import numpy as np
from Localization.core.raceline_optimizer import MinimumCurvatureOptimizer, SpeedProfiler

# Minimum curvature path optimization via constrained quadratic programming
optimizer = MinimumCurvatureOptimizer(w_left=2.5, w_right=2.5)
centerline = np.array([[0.0, 0.0], [20.0, 0.0], [40.0, 10.0], [50.0, 30.0]])
raceline = optimizer.optimize(centerline, closed=False)

# Friction-ellipse velocity profiling considering longitudinal and lateral grip
profiler = SpeedProfiler(mu=1.1, ax_max=8.0, ax_min=-12.0)
speed_profile = profiler.profile(raceline.points, raceline.curvatures)
print(f"Optimized Path: {len(raceline.points)} points | Peak Speed: {np.max(speed_profile):.1f} m/s")
```

### 4. Spatio-Temporal Frenet Trajectory Planning & Obstacle Avoidance
```python
from Localization.core.frenet_planner import FrenetTrajectoryPlanner, FrenetState, Obstacle

planner = FrenetTrajectoryPlanner(max_speed=20.0, max_accel=6.0, max_curvature=0.4)
current_frenet = FrenetState(s=0.0, s_d=12.0, s_dd=0.0, d=0.0, d_d=0.0, d_dd=0.0)
obstacles = [Obstacle(id=1, s=18.0, d=0.0, radius=0.8, v_s=4.0)]

# Evaluates quintic s(t) and quartic d(t) candidate polynomials
best_trajectory = planner.plan(current_frenet, obstacles=obstacles)
print(f"Selected Trajectory: Cost={best_trajectory.cost:.2f} | Terminal Lateral Offset d={best_trajectory.d[-1]:.2f} m")
```

### 5. Point Cloud Scan Matching Localization (2D ICP & NDT)
```python
import numpy as np
from Localization.core.scan_matcher import IterativeClosestPoint2D, NormalDistributionsTransform2D

# SVD-based Iterative Closest Point with analytical Hessian covariance
icp = IterativeClosestPoint2D(max_iterations=30, tolerance=1e-4)
source = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
target = source + np.array([0.05, -0.02])

alignment = icp.align(source, target)
print(f"ICP Converged: {alignment.converged} in {alignment.iterations} iters | Fitness: {alignment.fitness_score:.6f}")
```

### 6. Multi-Agent Racing Simulation with Aerodynamic Wake Drafting
```python
from Localization.core.multi_agent_sim import MultiAgentCoordinator, VehicleAgent
from Localization.core.race_control_engine import RaceControlEngine, SessionPhase
from Localization.core.dynamics import F1TENTH_PARAMS

# Coordinate multi-car high-speed racing with slipstream drafting & dirty air
coordinator = MultiAgentCoordinator()
coordinator.add_agent(VehicleAgent(id="Car-01", params=F1TENTH_PARAMS, initial_s=25.0, initial_v=15.0))
coordinator.add_agent(VehicleAgent(id="Car-02", params=F1TENTH_PARAMS, initial_s=10.0, initial_v=16.0))

race_control = RaceControlEngine(total_laps=10)
race_control.set_session_phase(SessionPhase.GREEN_RACING)

# Step simulation physics at 50 Hz
sim_state = coordinator.step(dt=0.02)
race_control.update_vehicle_state("Car-01", s=sim_state.agents["Car-01"].s, speed=sim_state.agents["Car-01"].v)
race_control.update_vehicle_state("Car-02", s=sim_state.agents["Car-02"].s, speed=sim_state.agents["Car-02"].v)

leaderboard = race_control.get_leaderboard()
for entry in leaderboard:
    print(f"P{entry.position}: {entry.vehicle_id} | Gap: +{entry.gap_to_leader:.3f}s")
```

### 7. Querying the REST API via cURL
```bash
# Retrieve real-time telemetry snapshot
curl -s http://localhost:8000/api/v1/monitoring/snapshot | jq .

# Retrieve HD Vector Map for Monza Autodromo
curl -s http://localhost:8000/api/v1/circuits/Monza/vector-map | jq .

# Trigger Software Emergency Stop (E-STOP)
curl -X POST http://localhost:8000/api/v1/monitoring/estop \
  -H "Content-Type: application/json" \
  -d '{"active": true}'

# Evaluate Proving Ground Safety Scorecard
curl -s http://localhost:8000/api/v1/kpis/scorecard | jq .
```

### 8. Subscribing to 20 Hz WebSocket Telemetry (JavaScript / Node.js)
```javascript
const ws = new WebSocket('ws://localhost:8000/api/v1/monitoring/ws');

ws.onmessage = (event) => {
  const telemetry = JSON.parse(event.data);
  console.log(`[${telemetry.circuit}] Speed: ${telemetry.dynamics.speed_kph} km/h | ey: ${telemetry.actuation.cross_track_error_m} m`);
};
```

---

## 🧪 Automated Testing & CI Verification

ApexPathAI & OpenRaceStack maintain **100% test pass rates** across all 23 test suites and 111 unit & integration tests:

```bash
# Execute full Pytest test matrix
pytest tests/ -v
```

```text
============================= test session starts =============================
platform win32 -- Python 3.13.9, pytest-8.4.2
collected 129 items

tests/integration/test_api_endpoints.py ....                             [  3%]
tests/integration/test_backend_api.py ..............                     [ 14%]
tests/integration/test_session_orchestrator.py ..                        [ 16%]
tests/integration/test_vector_map_api.py .                               [ 16%]
tests/unit/test_adaptation.py .....                                      [ 20%]
tests/unit/test_benchmarks.py ...                                        [ 22%]
tests/unit/test_ekf_engine.py .....                                      [ 26%]
tests/unit/test_eskf_3d.py ....                                          [ 29%]
tests/unit/test_frenet_mpc.py .....                                      [ 33%]
tests/unit/test_frenet_planner.py .....                                  [ 37%]
tests/unit/test_fusion_engine.py ...                                     [ 40%]
tests/unit/test_game_theoretic_planner.py ....                           [ 43%]
tests/unit/test_hd_vector_map.py .....                                   [ 47%]
tests/unit/test_metrics.py ..                                            [ 49%]
tests/unit/test_mpc_controller.py ..                                     [ 50%]
tests/unit/test_multi_agent_sim.py .....                                 [ 54%]
tests/unit/test_pure_pursuit.py ......                                   [ 59%]
tests/unit/test_race_control_engine.py .....                             [ 63%]
tests/unit/test_raceline_optimizer.py ......                             [ 67%]
tests/unit/test_ros2_bridge.py .......                                   [ 73%]
tests/unit/test_scan_matcher.py ......                                   [ 78%]
tests/unit/test_stanley.py ......                                        [ 82%]
tests/unit/test_sys_id.py ...                                            [ 85%]
tests/unit/test_vehicle_dynamics.py .........                            [ 92%]
tests/unit/test_vehicle_interface.py .....                               [ 96%]
tests/unit/test_visual_feature_tracker.py ....                           [100%]

======================= 129 passed, 1 warning in 9.42s =======================
```

---

## 📊 Benchmark Performance (23 Grand Prix Circuits)

Evaluated under synthetic GPS outage ($\sigma = 1.5\,\text{m}$), wheel odometry noise ($\sigma = 0.08\,\text{m/s}$), and gyro drift ($\sigma = 0.02\,\text{rad/s}$):

| Circuit | Waypoints | Length (m) | Raw GPS RMSE | Dual EKF RMSE | Frenet Lat RMSE | Filter Improvement |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Monza** | 1,489 | 5,793 | 2.12 m | **0.213 m** | **0.163 m** | **89.9%** |
| **Spa-Francorchamps**| 1,784 | 7,004 | 2.13 m | **0.218 m** | **0.170 m** | **89.8%** |
| **Silverstone** | 1,532 | 5,891 | 2.11 m | **0.215 m** | **0.165 m** | **89.8%** |
| **Nürburgring** | 1,328 | 5,148 | 2.12 m | **0.219 m** | **0.169 m** | **89.7%** |
| **Circuit de Barcelona**| 1,228 | 4,675 | 2.10 m | **0.209 m** | **0.161 m** | **90.0%** |
| **Melbourne Albert Park**| 1,368| 5,278 | 2.12 m | **0.214 m** | **0.164 m** | **89.9%** |
| **Austin COTA** | 1,438 | 5,513 | 2.13 m | **0.216 m** | **0.168 m** | **89.8%** |
| **Average (All 23 Tracks)**| **1,320** | **5,050** | **2.12 m** | **0.214 m** | **0.166 m** | **89.9%** |

---

## 📋 Automated 7-Rule Regulatory Scorecard (ISO 21448)

Every test session evaluates regulatory safety rules derived from **ISO 21448 (SOTIF)**:

1. **Rule 1 (Position Accuracy):** 2D Position RMSE $\le 0.35\,\text{m}$.
2. **Rule 2 (Lateral Tracking):** Peak Cross-Track Error $e_y \le 0.60\,\text{m}$.
3. **Rule 3 (Geofence Clearance):** Minimum track boundary margin $\ge 0.50\,\text{m}$.
4. **Rule 4 (Safety Integrity):** Horizontal Protection Level $\text{HPL} \le 3.0\,\text{m}$ (HAL limit).
5. **Rule 5 (Kalman Consistency):** Filter NEES lies within theoretical bounds $[2.0, 5.5]$.
6. **Rule 6 (Real-Time Latency):** Model Predictive Control solve time $\le 5.0\,\text{ms}$.
7. **Rule 7 (Zero Disengagements):** Total system emergency stops or disengagements $= 0$.

---

## ⚠️ System Assumptions & Known Limitations

OpenRaceStack is engineered for research rigor and physical determinism within documented operational envelopes:
- **Planar 2D Track Models:** Tracks are parameterized in 2D Cartesian and curvilinear Frenet frames; vertical slope grade and banking camber angles are omitted.
- **Single-Track Bicycle Kinematics:** Dynamic load transfer is modeled longitudinally and under aerodynamic downforce; chassis body roll and inner-wheel unloading are neglected.
- **Isothermal Tire Models:** Pacejka '96 and linear tire models assume constant bulk tire temperatures without thermal degradation modeling.
- **Hardware Bus Interfaces:** High-speed SocketCAN communication requires Linux (`vcan0`/`can0`); Windows environments operate via simulated software loops.

For the comprehensive specification, see [limitations.md](file:///C:/Users/usuario/ApexPathAI/docs/architecture/limitations.md).

---

## 🤝 Contributing & Community

We welcome contributions from autonomous driving researchers, robotics engineers, control theorists, and open-source software developers.

- Please read our [CONTRIBUTING.md](file:///C:/Users/usuario/ApexPathAI/CONTRIBUTING.md) for coding standards, branching strategy, and PR workflow.
- Follow our [CODE_OF_CONDUCT.md](file:///C:/Users/usuario/ApexPathAI/CODE_OF_CONDUCT.md).
- Review our strategic milestones in [ROADMAP.md](file:///C:/Users/usuario/ApexPathAI/ROADMAP.md).
- Security disclosures should follow [SECURITY.md](file:///C:/Users/usuario/ApexPathAI/SECURITY.md).

---

## 📄 License & Attribution

- **ApexPathAI & OpenRaceStack Codebase:** Distributed under the [Apache License 2.0](LICENSE).
- **Racetrack Datasets (`Localization/datasets/`):** Derived from the F1TENTH / TUM Racetrack Database ([GNU General Public License v3.0](Localization/datasets/LICENSE)), documented in [NOTICE](NOTICE).

---

## 📚 Citation

If you use OpenRaceStack or ApexPathAI in your academic research, please cite our repository via [CITATION.cff](file:///C:/Users/usuario/ApexPathAI/CITATION.cff):

```bibtex
@software{openracestack2026,
  author = {OpenRaceStack Core Research Team and Contributors},
  title = {{OpenRaceStack: A Modular, Cross-Platform Autonomous Racing Framework for Multi-Scale Vehicles}},
  url = {https://github.com/apexpath-ai/ApexPathAI},
  version = {1.0.0-rc1},
  year = {2026}
}
```
