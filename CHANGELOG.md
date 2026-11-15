# Changelog

All notable changes to the ApexPathAI autonomous vehicle validation platform will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [2.1.0-rc1] - 2026-11-15

### OpenRaceStack Modular Autonomous Racing Framework Release Candidate 1
Introduces **OpenRaceStack**, a modular, cross-platform autonomous racing framework powering both the proving ground validation platform and high-performance competitive autonomous racing across 1:10 scale (F1TENTH), electric go-karts, Dallara AV-21 IndyCars, and passenger GT racecars.

### Added
- **Core Vehicle Dynamics & Tracking Control (Phase A):**
  - High-fidelity `DynamicBicycleModel` with Runge-Kutta 4th-order (RK4) integration, longitudinal weight transfer, aerodynamic downforce pitch, and low-speed singularity regularization (`dynamics.py`).
  - Simplified `KinematicBicycleModel` with CG slip angle ($\beta$) for high-rate trajectory rollouts.
  - Non-linear lateral tire models: `LinearTireModel` with Coulomb saturation and `PacejkaTireModel` implementing Pacejka '96 Magic Formula.
  - Multi-scale vehicle configurations: `VehicleParameters` dataclass with presets for `F1TENTH_PARAMS`, `GO_KART_PARAMS`, `AV21_PARAMS`, and `PASSENGER_GT_PARAMS`.
  - `AdaptivePurePursuitController` with adaptive lookahead distance ($L_d = \mathrm{clip}(k_{dd} v + L_{\min}, L_{\min}, L_{\max})$), path arc-length interpolation, and steering slew rate limiting (`pure_pursuit.py`).
  - `RacelineOptimizer`: Minimum-curvature path optimization within corridor boundaries via box-constrained quadratic programming (`scipy.optimize.lsq_linear`) (`raceline_optimizer.py`).
  - Dynamic `SpeedProfiler`: Friction-ellipse coupled forward (traction) and backward (braking) integration passes with lateral cornering limits.
  - High-frequency `FrenetMPCController`: Linear time-varying (LTV) error dynamics in Frenet frame with condensed positive-definite QP formulation and sub-2ms Nesterov-accelerated Projected Gradient Descent (PGD) solver (`frenet_mpc.py`).
  - `FrenetLatticePlanner`: Spatio-temporal quintic $s(t)$ and quartic $d(t)$ polynomial trajectory planner with dynamic obstacle collision checking and emergency braking fallback (`frenet_planner.py`).
  - Cross-platform Hardware Abstraction Layer (`VehicleInterface` HAL): Standardized contracts with `SimulatedVehicleInterface`, `F1TenthVehicleInterface` (VESC ERPM/PWM microsecond mapping), and `DallaraAV21VehicleInterface` (CAN FD / hydraulic brake pressure / gear shifting) (`vehicle_interface.py`).
  - Zero-dependency `ROS2MessageConverter` and `ROS2Bridge`: Pure Python ROS 2 serialization matching `nav_msgs/msg/Odometry`, `sensor_msgs/msg/Imu`, `tf2_msgs/msg/TFMessage`, and `ackermann_msgs/msg/AckermannDriveStamped` with competition flag supervisor and ASIL-ready watchdog preemption (`ros2_bridge.py`).

- **Perception, Scan Matching & HD Vector Maps (Phase B):**
  - High-performance 2D LiDAR Scan Matcher (`scan_matcher.py`):
    - `VoxelGridFilter`: 2D/3D spatial voxel centroid downsampling.
    - `StatisticalOutlierFilter`: k-NN statistical rejection of rain, airborne dust, and tire smoke.
    - `IterativeClosestPoint2D`: SVD-based rigid-body registration with analytical Hessian covariance estimation.
    - `NormalDistributionsTransform2D`: Cell-based Gaussian probability distribution transform with regularized Gauss-Newton descent.
  - Sub-centimeter HD Vector Map Engine (`hd_vector_map.py`):
    - Centerline, inner/outer boundaries, kerbs, split sectors, and speed trap geometries.
    - Spatial KD-Tree queries: `is_inside_track()`, `distance_to_boundary()`, and `get_corridor_bounds()`.
    - GeoJSON FeatureCollection export for Mapbox GL vector tile rendering.
    - FastAPI endpoint: `GET /api/v1/circuits/{track_name}/vector-map` (`circuits.py`).
  - Multi-Camera Visual Feature Tracker (`visual_feature_tracker.py`):
    - Optical flow tracking of 2D/3D landmark and corner keypoints.
    - Environmental degradation simulation: direct sun glare, tunnel lighting transitions, and tire water spray occlusion.

- **Multi-Agent Racing Simulation & Race Control (Phase C):**
  - Multi-Agent Racing Coordinator (`multi_agent_sim.py`):
    - Simultaneous multi-vehicle physics simulation with `RacingAgent` lifecycle.
    - Aerodynamic wake drafting model (up to 35% drag reduction in slipstream) and dirty air turbulent downforce loss (up to 25% front grip drop).
    - Continuous dual-circle mutual collision detection and incident logging.
  - Game-Theoretic Overtaking Decision Engine (`game_theoretic_planner.py`):
    - Interactive tactical decision making between Attacker and Defender: `INSIDE_OVERTAKE`, `OUTSIDE_SWEEP`, `DEFENSIVE_COVER`, `SLIPSTREAM_FOLLOW`, and `YIELD_SAFETY`.
    - FIA Code of Driving Conduct enforcement: "Significant Overlap" rule (attacker front axle alongside defender rear axle mandates leaving 1 car width) and "One-Move" defensive covering.
  - Race Control & Timing/Scoring Engine (`race_control_engine.py`):
    - Competition state machine: `IDLE`, `FORMATION`, `GREEN_RACING`, `SAFETY_CAR`, `FULL_COURSE_YELLOW`, `RED_FLAG`, `CHEQUERED`.
    - Official leaderboard with real-time gap to leader ($+\Delta t$) and intervals.
    - Track limits boundary monitoring with automated 5-second time penalty allocation for 3 infringements.

- **System Identification & Online Adaptation (Stage 4 / OpenRaceStack v1.0 Foundation):**
  - `RecursiveLeastSquares` (RLS) with exponential forgetting factor for real-time parameter tracking (`sys_id.py`).
  - `OnlineVehicleSystemIdentifier`: Continuous estimation of front/rear tire cornering stiffness ($C_{\alpha,f}, C_{\alpha,r}$) and road adhesion friction ($\mu$).
  - `BatchSystemIdentifier`: Multivariate regularized least-squares batch parameter fitting with $R^2$ goodness-of-fit.
  - `ModelPredictionValidator`: $k$-step ahead dynamics model rollout validation computing RMSE for $v_x, v_y, \omega_z$ and 2D position (`adaptation.py`).
  - `OnlineAdaptationManager`: Real-time observer updating MPC constraints, friction ellipse ceilings, and speed profiles under surface grip degradation.
  - `StanleyController`: Non-linear front-axle trajectory tracking controller with cross-track and heading error compensation (`stanley.py`).
  - Cross-platform benchmarking suite: `scripts/benchmark_vehicles.py` executing comparative proving ground simulations across F1TENTH, Go-Kart, and Dallara AV-21 models.
  - Standalone runnable examples directory (`examples/01` through `examples/07`).
  - Standardized academic attribution (`CITATION.cff`) and comprehensive architectural boundary documentation (`docs/architecture/limitations.md`).

- **Quality Assurance & Verification (Phase D & v1.0 Validation):**
  - Expanded 129-test suite with 100% pass rate across 26 unit and integration suites (`pytest tests/`).
  - Zero Ruff linting warnings or style violations across all modules (`ruff check`).
  - Release candidate verification dossier (`RELEASE_CANDIDATE_v2.1.0-rc1.md`).

---

## [2.0.0] - 2026-11-14

### Major Platform Transformation
Transformed the repository into a production-grade, enterprise-ready **Autonomous Vehicle (AV) Proving Ground Validation & State Estimation Platform** with complete clean architecture, automated testing, and containerized deployment.

### Added
- **React 18 + TypeScript + Vite Cockpit Dashboard (`frontend/`):**
  - Enterprise Material UI (MUI v5) dark aeronautic cockpit theme.
  - Interactive Mapbox GL and HTML5 Canvas track layout visualizer.
  - Recharts telemetry graphs for speed, G-G diagram, Stanford tracking error, and innovation residuals.
  - 10 dedicated modular views covering all operational pillars: Real-Time Monitoring, Vehicle Catalog, Circuit Management, Session Manager, Localization Analytics, Vehicle Analytics, Sensor Diagnostics, Replay Scrubber, KPI Scorecard, and Dossier Generator.
  - Multi-stage Nginx production container with SPA routing.
- **Enterprise FastAPI Backend Application (`backend/`):**
  - Clean modular architecture: `api/v1`, `core`, `models`, `schemas`, and `services`.
  - Pydantic v2 data validation schemas across all endpoints.
  - SQLAlchemy 2.0 async ORM models for vehicles, circuits, test sessions, and validation reports.
  - Zero-config local development mode with automatic fallback to SQLite (`sqlite+aiosqlite`) and an in-memory pub/sub broker.
  - Production-ready support for PostgreSQL 16 and Redis 7 pub/sub streaming.
- **High-Frequency 20 Hz WebSocket Telemetry Engine:**
  - Real-time broadcast loop streaming 20 Hz vehicle poses, twist, dynamics, actuation, sensor health, and Frenet margins.
  - Instantaneous software Emergency Stop (E-STOP) endpoint and actuator interlock.
- **Comprehensive Proving Ground Testing Architecture:**
  - Dual Extended Kalman Filter stack: 2D Planar CTRV EKF + 15-DOF Error-State Kalman Filter on $\text{SE}(3) \times \mathbb{R}^6$.
  - Frenet-frame Model Predictive Control guidance with lookahead preview rollout.
  - 23 international Grand Prix racetrack datasets indexed via spatial `cKDTree` and Euclidean distance fields.
  - Automated 7-rule KPI scorecard validating position RMSE, cross-track error, geofence margin, HPL, NEES, and solve latency.
  - On-demand formal validation dossier generation with Markdown and JSON exports.
- **Infrastructure & DevOps:**
  - `docker-compose.yml` orchestrating FastAPI backend, React/Nginx frontend, PostgreSQL, and Redis.
  - Multi-stage Dockerfiles for backend and frontend.
  - GitHub Actions CI matrix testing Python 3.11, 3.12, and 3.13 alongside frontend linting and typechecking.
  - Docker publishing workflow for multi-platform container images.
- **Documentation & Open Source Standards:**
  - Comprehensive `README.md` with architectural topology, interactive 10-pillar matrix, and benchmark results.
  - Strategic `ROADMAP.md` mapping v2.0 -> v2.1 -> v3.0 horizons.
  - `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, and `LICENSE` (Apache-2.0).

### Fixed & Deprecated
- Removed legacy monolithic single-file web server string literals in favor of decoupled React SPA.
- Eliminated all circular imports between estimation, utility, and simulation modules.
- Modernized datetime calls to timezone-aware UTC representations (`datetime.now(timezone.utc)`).
- Upgraded all Pydantic configurations to `model_config = ConfigDict(...)` and `json_schema_extra`.
- Ensured Joseph-stabilized covariance propagation in Kalman filters to strictly preserve positive semi-definiteness ($P = P^T \ge 0$).
