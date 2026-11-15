# ApexPathAI Platform Roadmap 🗺️

This document outlines the strategic architectural roadmap and development milestones for **ApexPathAI** — an enterprise-grade Autonomous Vehicle (AV) Proving Ground Validation & State Estimation Platform.

---

## 🎯 Strategic Vision

ApexPathAI bridges the critical gap between offline mathematical vehicle state estimation and physical proving ground certification. Our objective is to provide autonomous racing teams, automotive OEMs, and safety research institutions with an open-source, regulatory-grade platform for validating localization integrity, sensor fusion resilience, and trajectory tracking under extreme dynamic limits.

```
       ┌────────────────────────────────────────────────────────┐
       │             ApexPathAI Evolution Spectrum              │
       └────────────────────────────────────────────────────────┘
          v1.0 (Legacy)          v2.0 (Current)             v3.0 (Future)
     Standalone Script      Enterprise Platform       Distributed Multi-Agent
     ─────────────────      ───────────────────       ───────────────────────
     • Single-file ASGI     • Decoupled Clean Arch    • CARLA 0.9.15 HIL
     • Monolithic HTML      • React + TS + MUI        • ROS 2 Jazzy Transport
     • Hardcoded scores     • 23 Proving Grounds      • Multi-Agent Overtaking
     • Basic EKF            • CTRV EKF + 15-DOF ESKF  • Cloud-Native Kubernetes
```

---

## 📌 Release Schedule & Milestone Status

| Version | Status | Milestone Horizon | Core Focus Area |
| :--- | :---: | :---: | :--- |
| **v2.0.0** | **Released** | Q4 2026 | Full architectural transformation: React/TS Cockpit, FastAPI, PostgreSQL, Redis, 23 Tracks, 10 Core Pillars |
| **v2.1.0-rc1** | **Released (Current)** | Q4 2026 | OpenRaceStack Autonomous Racing Framework: Dynamics, Frenet MPC, HAL, Scan Matching, Multi-Agent Sim, Race Control |
| **v2.2.0** | **Planned** | Q1 2027 | CARLA Simulator Co-Simulation Bridge, 3D GPU LiDAR NDT Point Clouds, SocketCAN Native Interface |
| **v3.0.0** | **Future** | Q2-Q3 2027 | Physical Multi-Vehicle Fleet Autonomous Racing Tournaments, Cloud Cluster Monte Carlo Testing Matrix |

---

## 🚀 Detailed Milestones

### Milestone v2.0.0 — Enterprise Architecture Foundation ✅ *(Completed)*
- [x] **Full Decoupling & Clean Architecture**: Complete separation of presentation, domain services, telemetry streaming, and persistent ORM.
- [x] **React 18 + TypeScript + Vite Cockpit**: Modern frontend dashboard with Material UI v5, Mapbox GL vector rendering, and Recharts.
- [x] **Dual Kalman Estimator Pipeline**:
  - Planar 2D CTRV EKF with Joseph stabilized covariance and $\chi^2$ innovation gating.
  - 15-DOF Error-State Kalman Filter (ESKF) with quaternion kinematics and IMU bias estimation.
- [x] **Frenet-Frame Model Predictive Control**: Adaptive lookahead guidance with boundary constraints and actuator saturation monitoring.
- [x] **23 Proving Ground Circuits**: Support for all 23 Formula 1 tracks with downsampled raster maps, centerlines, and raceline profiles.
- [x] **10 Core Operational Pillars**:
  1. Real-Time Telemetry Cockpit (20 Hz WebSockets)
  2. Fleet Vehicle Management & Kinematic Configurations
  3. Circuit Catalog & Proving Ground Geometries
  4. Test Session Campaign Orchestration (IDLE -> ARMED -> RUNNING -> COMPLETED)
  5. Localization Integrity Analytics (CEP50, CEP95, HPL, NEES)
  6. Vehicle Dynamics Analytics (Slip Angle, G-G Diagram, Curvature)
  7. Multi-Sensor Diagnostics & Fault Injection (GNSS, LiDAR, Camera, IMU)
  8. Incident Replay System with Frame-by-Frame Scrubber
  9. Regulatory KPI Scorecards with Automated Pass/Fail Verdicts
  10. Formal Validation Dossier Reporting (Markdown / PDF / JSON)
- [x] **Containerized Infrastructure**: Production-ready Docker multi-stage builds and Docker Compose orchestration (FastAPI + PostgreSQL + Redis + React/Nginx).
- [x] **Automated CI/CD**: GitHub Actions workflows for continuous linting, type-checking, and test automation across Python 3.11, 3.12, and 3.13.

---

### Milestone v2.1.0 — High-Fidelity Perception & HD Vector Maps 🔍 *(Target: Q1 2027)*
- [ ] **Native Mapbox Vector Tile (MVT) Server**:
  - Serve protocol buffer vector tiles directly from GeoJSON centerline and corridor boundary datasets.
  - Sub-centimeter track resolution with interactive sector coloring and elevation profiles.
- [ ] **LiDAR Point Cloud Scan Matching Integration**:
  - Direct 2D/3D LiDAR scan registration via Normal Distributions Transform (NDT) and Iterative Closest Point (ICP).
  - Degraded point-cloud filtering for heavy precipitation, dust, and tire smoke simulation.
- [ ] **Multi-Camera Visual Inertial Odometry (VIO)**:
  - Synthetic stereo-pair feature tracking pipeline.
  - Dynamic optical flow degradation under high-speed glare and tunnel lighting transitions.
- [ ] **Automated Incident Snapshot Slicing**:
  - Auto-bookmarking of boundary infringement events and covariance explosion frames with zero-copy buffer slicing.

---

### Milestone v2.2.0 — Hardware-in-the-Loop (HIL) & Industrial Standards 🔌 *(Target: Q2 2027)*
- [ ] **Industrial Vehicle Bus Integration**:
  - Native Linux SocketCAN and CAN FD bus interfaces.
  - Support for DBC protocol decoding (J1939 and custom autonomous racing frames).
- [ ] **ROS 2 (Robot Operating System) Transport**:
  - Native ROS 2 Jazzy node bridging `/tf`, `/nav_msgs/Odometry`, and `/sensor_msgs/Imu`.
  - Zero-copy shared memory communication using Eclipse Cyclone DDS.
- [ ] **ISO 26262 ASIL-D Safety Supervisor**:
  - Formal E-STOP hardware loop verification.
  - Real-time watchdog monitoring with hard deadline preemption (< 5 ms execution latency).
- [ ] **AAS (Asset Administration Shell) Digital Twin Export**:
  - Export vehicle kinematics and sensor calibrations to standard Industry 4.0 AAS format.

---

### Milestone v3.0.0 — Multi-Agent Racing & Cloud Co-Simulation 🏁 *(Target: Q3-Q4 2027)*
- [ ] **CARLA 0.9.15 Simulator Bridge**:
  - Bi-directional TCP/UDP socket bridge syncing vehicle transforms, wheel torques, and sensor streams.
  - Photorealistic proving grounds in Unreal Engine 5.
- [ ] **Multi-Agent Autonomous Racing**:
  - Multi-vehicle simultaneous testing on track with interactive collision avoidance.
  - Frenet game-theoretic overtaking planner validation.
- [ ] **Distributed Cloud Testing Matrix**:
  - Automated execution of 1,000+ Monte Carlo simulation runs across cloud clusters (Kubernetes).
  - Statistical batch regression testing across varying weather, sensor noise, and pavement friction parameters.
- [ ] **OpenAPI SDK Generation**:
  - Automated release of Python, C++, and Rust client SDKs via OpenAPI Generator.

---

## 🏎️ OpenRaceStack Modular Architecture Transition

As part of the evolution toward **OpenRaceStack**, the framework is expanding into a modular, cross-platform autonomous racing framework supporting vehicles from 1:10 scale (F1TENTH) to full-scale Indy Autonomous Challenge (Dallara AV-21) and GT racecars.

### Phase A: Vehicle Dynamics & Core Tracking Controllers
- [x] **Step 1 — Core Vehicle Dynamics & Pure Pursuit Controller ✅**:
  - `VehicleParameters` dataclass with geometric, inertial, aerodynamic, and actuator limits.
  - Predefined vehicle configuration presets: `F1TENTH_PARAMS` (1:10 scale), `GO_KART_PARAMS` (electric/Rotax kart), `AV21_PARAMS` (Indy Autonomous Challenge), and `PASSENGER_GT_PARAMS`.
  - Non-linear lateral tire models: `LinearTireModel` (Coulomb saturation) and `PacejkaTireModel` (Pacejka '96 Magic Formula $F_y = D \sin(C \arctan(B \alpha - E(B \alpha - \arctan(B \alpha))))$.
  - High-fidelity `DynamicBicycleModel` (single-track) with dynamic normal load transfer (acceleration pitch and aerodynamic downforce), body-fixed slip angles $\alpha_f, \alpha_r$, and Runge-Kutta 4th-order (RK4) integration with low-speed singularity regularization.
  - Simplified `KinematicBicycleModel` with CG side-slip angle $\beta$ for low-speed maneuvering and real-time planning rollouts.
  - `PurePursuitController` with adaptive lookahead distance ($L_d = \mathrm{clip}(k_{dd} v + L_{\min}, L_{\min}, L_{\max})$), path arc-length interpolation, steering slew rate limiting, and closed-loop speed tracking.
  - Comprehensive unit test coverage: 21 dedicated dynamics and tracking tests with 100% pass rate.
- [x] **Step 2 — Constrained Frenet MPC & Dynamic Raceline Optimization ✅**:
  - `RacelineOptimizer`: Minimum-curvature path optimization within corridor boundaries via box-constrained quadratic programming (`scipy.optimize.lsq_linear`).
  - Dynamic Speed Profiler: Friction-ellipse coupled forward (traction) and backward (braking) integration passes, aerodynamically-enhanced lateral cornering limits, and theoretical lap time evaluation.
  - `FrenetMPCController`: Linear time-varying (LTV) error dynamics in Frenet frame with condensed positive-definite QP formulation.
  - Sub-millisecond Nesterov-accelerated Projected Gradient Descent (PGD) solver with hard actuator angle bounds, steering slew-rate constraints, and friction ellipse limits.
  - End-to-end integration with `DynamicBicycleModel` in closed loop (average solve time ~2.0 ms at 20 Hz).
  - 11 new unit tests (total 70 tests passed, 0 failures, 100% ruff lint pass).
- [x] **Step 3 — Local Dynamic Obstacle Avoidance & Frenet Lattice Planner ✅**:
  - `QuinticPolynomial` (lateral maneuvers $d(t)$) and `QuarticPolynomial` (longitudinal velocity keeping $s(t)$) with exact analytical boundary condition solvers and closed-form integrated jerk cost $\int (\dddot{x})^2 dt$.
  - `FrenetLatticePlanner`: Spatio-temporal candidate generation across temporal horizons $T \in [T_{\min}, T_{\max}]$ and lateral targets $d \in [-w_{\text{right}}, w_{\text{left}}]$.
  - Dynamic & static `Obstacle` tracking with linear future position projection $P_{\text{obs}}(t)$ and continuous collision distance verification against vehicle envelope and safety clearance margins.
  - Multi-gate candidate validation: track corridor boundaries, velocity caps, longitudinal acceleration bounds $[a_{\min}, a_{\max}]$, lateral tire grip ($a_y \le a_{y,\max}$), and path curvature limits ($\kappa \le \kappa_{\max}$).
  - Fallback emergency braking trajectory generation for fully obstructed track sectors.
  - 7 new unit tests (total 77 tests passed, 0 failures, 100% ruff lint pass).
- [x] **Step 4 — Cross-Platform Vehicle Abstraction Layer & ROS 2 Bridge ✅**:
  - `VehicleInterface` (HAL ABC): Standardized connect, disconnect, send_command, and receive_telemetry contracts with `VehicleStatus` lifecycle.
  - Platform implementations:
    - `SimulatedVehicleInterface`: Dynamic single-track physics with first-order steering/throttle actuator lag and sensor noise.
    - `F1TenthVehicleInterface`: VESC electrical RPM scaling and steering servo PWM microsecond mapping ($[1000, 2000]\,\mu\text{s}$).
    - `DallaraAV21VehicleInterface`: Drive-by-Wire CAN FD mappings, dual-circuit hydraulic brake pressures ($0 - 75\,\text{bar}$), and angle checks.
  - `ROS2MessageConverter`: Pure Python bidirectional conversions matching `nav_msgs/msg/Odometry`, `sensor_msgs/msg/Imu`, `tf2_msgs/msg/TFMessage`, and `ackermann_msgs/msg/AckermannDriveStamped` with zero external dependencies.
  - `ROS2Bridge`: Competition Race Control flag state machine (`GREEN`, `YELLOW`, `RED`, `PURPLE`), caution speed limits, and ASIL-ready watchdog preemption (< 100 ms heartbeat timeout).
  - 9 new unit tests (total 86 tests passed, 0 failures, 100% ruff lint pass).

### Phase B: Perception, Scan Matching & HD Vector Maps ✅
- [x] **High-Performance Scan Matching Engine ([scan_matcher.py](file:///C:/Users/usuario/ApexPathAI/Localization/core/scan_matcher.py))**:
  - `VoxelGridFilter`: 2D/3D spatial voxel centroid downsampling for dense point clouds.
  - `StatisticalOutlierFilter`: k-NN distance statistical rejection of airborne debris, rain, and dust.
  - `IterativeClosestPoint2D`: SVD-based SE(2) rigid-body registration with analytical Hessian covariance estimation.
  - `NormalDistributionsTransform2D`: Cell-based Gaussian probability distribution transform with Gauss-Newton descent.
- [x] **Sub-Centimeter HD Vector Map Engine ([hd_vector_map.py](file:///C:/Users/usuario/ApexPathAI/Localization/core/hd_vector_map.py))**:
  - Centerline, left/right barrier extrusion, and apex kerb geometries.
  - Spatial KD-Tree queries: `is_inside_track()`, `distance_to_boundary()`, and `get_corridor_bounds()`.
  - Circuit sector management (S1, S2, S3) and speed trap tracking.
  - GeoJSON FeatureCollection export for Mapbox GL vector tile rendering.
- [x] **Multi-Camera Visual-Inertial Feature Tracker ([visual_feature_tracker.py](file:///C:/Users/usuario/ApexPathAI/Localization/core/visual_feature_tracker.py))**:
  - 2D/3D landmark and corner keypoint optical flow tracking.
  - Environmental degradation simulation: direct sun glare, tunnel transitions, and rain/tire spray occlusion.
  - Full measurement covariance estimation for multi-sensor Kalman fusion.
- [x] **REST API Exposure & Proving Ground Verification**:
  - Added `GET /api/v1/circuits/{track_name}/vector-map` delivering sub-centimeter GeoJSON layers.
  - 14 new unit and integration tests (total 100 tests passed, 0 failures, 100% ruff lint pass).

### Phase C: Multi-Agent Racing Simulation & Distributed Verification ✅
- [x] **Multi-Agent Simulation Coordinator ([multi_agent_sim.py](file:///C:/Users/usuario/ApexPathAI/Localization/core/multi_agent_sim.py))**:
  - Simultaneous multi-car autonomous racing simulation with `RacingAgent` lifecycle.
  - Aerodynamic wake drafting model (up to 35% drag reduction in slipstream) and dirty air turbulent downforce reduction (up to 25% front grip drop).
  - Continuous dual-circle mutual collision detection and incident event logging.
- [x] **Game-Theoretic Overtaking Decision Engine ([game_theoretic_planner.py](file:///C:/Users/usuario/ApexPathAI/Localization/core/game_theoretic_planner.py))**:
  - Interactive strategic decision making between Attacker and Defender: `INSIDE_OVERTAKE`, `OUTSIDE_SWEEP`, `DEFENSIVE_COVER`, `SLIPSTREAM_FOLLOW`, and `YIELD_SAFETY`.
  - FIA Code of Driving Conduct enforcement: "Significant Overlap" rule (attacker front axle alongside defender rear axle mandates leaving 1 car width) and "One-Move" defensive covering.
- [x] **Race Control & Timing/Scoring Engine ([race_control_engine.py](file:///C:/Users/usuario/ApexPathAI/Localization/core/race_control_engine.py))**:
  - Competition state machine: `IDLE`, `GREEN_RACING`, `SAFETY_CAR`, `FULL_COURSE_YELLOW`, `RED_FLAG`, `CHECKERED`.
  - Official leaderboard with real-time gap to leader ($+\Delta t$) and intervals.
  - Track limits boundary monitoring with automated 5-second time penalty allocation for 3 infringements.
  - 11 new unit tests (total 111 tests passed, 0 failures, 100% ruff lint pass).

### Phase D: Production Release Hardening, CI/CD & Ratification ✅
- [x] **Test Matrix & Quality Gates**:
  - 111 unit & integration tests passing with 100% success rate across 23 test suites.
  - 0 Ruff linting errors or style warnings across all modules.
  - Complete Pydantic v2 data validation schemas and SQLAlchemy 2.0 async ORM models.
- [x] **Cross-Platform CI/CD Workflows**:
  - GitHub Actions matrix testing Ubuntu and Windows runners across Python 3.10, 3.11, 3.12, and 3.13.
  - Docker multi-arch containerization (FastAPI backend + React/Nginx frontend + PostgreSQL 16 + Redis 7).
- [x] **Architecture & Release Ratification**:
  - Official Release Candidate v2.1.0-rc1 ratification dossier with comprehensive readiness scores.
  - Updated API references, Quick Start guides, and multi-vehicle integration tutorials.

---

## 💡 Proposing New Roadmap Features

We welcome ideas and feature requests from autonomous driving engineers, researchers, and competitive racing teams!

1. Open an issue with the label `enhancement` or `rfc` on [GitHub Issues](https://github.com/apexpath-ai/ApexPathAI/issues).
2. Clearly articulate:
   - The vehicle autonomy use case.
   - Theoretical or mathematical basis.
   - Proposed API or architecture integration.
3. Review our [CONTRIBUTING.md](file:///C:/Users/usuario/ApexPathAI/CONTRIBUTING.md) guide before submitting pull requests.
