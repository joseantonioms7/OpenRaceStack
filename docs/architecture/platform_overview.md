# Enterprise Platform Architecture

ApexPathAI is structured as an enterprise-grade Autonomous Vehicle (AV) Proving Ground Validation & State Estimation Platform. The platform decouples presentation, high-speed telemetry streaming, state estimation algorithms, and persistent storage.

---

## High-Level Topology

```mermaid
flowchart TD
    subgraph Presentation ["Presentation Layer (React 18 + TypeScript)"]
        UI["React 18 Single Page App<br/>(Vite + Material UI v5)"]
        MapBox["Mapbox GL Vector Engine<br/>& Track Canvas"]
        Charts["Recharts Telemetry Panels<br/>(G-G, Stanford, Residuals)"]
    end

    subgraph Gateway ["Communication Layer"]
        REST["REST API v1 (JSON over HTTP/2)"]
        WS["20 Hz Real-Time Telemetry Stream<br/>(RFC 6455 WebSockets)"]
    end

    subgraph Backend ["FastAPI Backend Application Core"]
        Routers["FastAPI API Routers<br/>(Monitoring, Vehicles, Sessions, Replay)"]
        Broadcaster["20 Hz Async Telemetry Broadcaster"]
        Orchestrator["Session Campaign Lifecycle Machine<br/>(IDLE -> ARMED -> RUNNING -> COMPLETED)"]
    end

    subgraph DataLayer ["Data & Caching Infrastructure"]
        Postgres[("PostgreSQL 16 Database<br/>(Vehicles, Laps, Reports)")]
        RedisBroker[("Redis 7 Pub/Sub Engine<br/>(High-throughput fanout)")]
        LocalFallback[("Zero-Config Fallback<br/>(Async SQLite + In-Memory Broker)")]
    end

    subgraph AutonomyCore ["Kinematics & Estimation Core"]
        CTRV["Planar CTRV EKF (5 States)<br/>Joseph Covariance & Chi-sq Gate"]
        ESKF["15-DOF Error-State Kalman Filter<br/>Strapdown Mechanization on SE(3)"]
        FrenetMPC["Frenet-Frame Model Predictive Control<br/>Curvature Feedforward & Slew Gating"]
        MapEngine["MapManager & Spatial cKDTree<br/>(23 Formula 1 Circuits)"]
    end

    UI --> REST
    UI --> WS
    REST --> Routers
    WS --> Broadcaster
    Routers --> Orchestrator
    Broadcaster --> RedisBroker
    Routers --> Postgres
    Routers --> LocalFallback
    Orchestrator --> AutonomyCore
    Broadcaster --> AutonomyCore
```

---

## 1. Frontend Architecture (`frontend/`)

- **Modern Stack:** React 18, TypeScript, Vite, Material UI (MUI v5), Mapbox GL, Recharts.
- **Dark Cockpit Aesthetic:** Specially crafted dark aeronautic color scheme (`#0B0F19` void background, `#00E5FF` electric cyan, `#00E676` telemetry green, `#FF3D71` emergency stop).
- **10 Modular Screen Views:**
  1. `LiveCockpit`: Real-time 20 Hz instrument cluster with G-force display, speed gauge, and Stanford error plane.
  2. `VehicleCatalog`: Fleet management and physical parameter tuning.
  3. `CircuitList`: Interactive 23-track gallery with downsampled centerline previews.
  4. `SessionManager`: Test campaign execution, sector split times, and state transitions.
  5. `LocalizationAnalytics`: CEP50, CEP95, HPL, and NEES consistency charts.
  6. `VehicleAnalytics`: Tire sideslip angle ($\beta$), curvature, and acceleration envelope.
  7. `SensorDiagnostics`: Live innovation residuals (NIS) and fault injection buttons.
  8. `ReplayScrubber`: Incident scrubber with interactive seek bar and playback rate controls.
  9. `KPIScorecardTable`: Automated pass/fail matrix against ISO 21448 thresholds.
  10. `ReportGenerator`: Formal validation dossier compiler with Markdown and JSON exports.

---

## 2. Backend Application Core (`backend/app/`)

- **Application Factory:** Configured with FastAPI lifespan managers for clean startup and asynchronous shutdown.
- **Pydantic v2 Contracts:** Strict data validation models ensuring deterministic API payloads across all endpoints.
- **Dual Database Strategy:**
  - **Production:** Async SQLAlchemy with asyncpg connecting to PostgreSQL 16.
  - **Zero-Config Developer Onboarding:** Automatic fallback to SQLite (`sqlite+aiosqlite`) when running standalone without external Docker containers.
- **Pub/Sub Telemetry Broker:**
  - Redis 7 pub/sub for high-throughput distributed fanout.
  - In-memory event broker fallback for single-instance development.

---

## 3. Autonomous Kinematics & Estimation Core (`Localization/core/`)

- **Dual-Stack Fusion:**
  - **Planar CTRV EKF:** 5-state vector $[p_x, p_y, \theta, v, \omega]^T$ tracking high-speed planar motion on $S^1$ manifold.
  - **15-DOF ESKF:** 3D strapdown inertial navigation on $\text{SE}(3) \times \mathbb{R}^6$ with unit quaternion error propagation.
- **Frenet Model Predictive Guidance:**
  - Coordinates tracking against precomputed optimal racelines.
  - Cross-track error ($e_y$) and heading alignment ($e_\psi$) calculations.
