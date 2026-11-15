# Release Notes — ApexPathAI v2.0.0

**Release Date:** November 14, 2026  
**Tag:** `v2.0.0`  
**License:** Apache License 2.0 (Software) / GNU GPL v3.0 (Datasets)  
**Documentation:** [https://yourorg.github.io/ApexPathAI/](https://yourorg.github.io/ApexPathAI/)

---

## Executive Summary

The ApexPathAI maintainers are proud to announce the official open-source release of **ApexPathAI v2.0.0**.

ApexPathAI is an autonomous vehicle localization, state estimation, and proving ground validation platform designed for closed-circuit testing and safety verification. This release transitions the platform into a production-ready open-source ecosystem featuring dual-stack Kalman filtering, real-time telemetry streaming at 20 Hz, an 8-screen glass-cockpit dashboard, and automated regulatory pass/fail verification across 23 international Grand Prix circuits.

---

## Key Highlights & New Features

### 1. 15-DOF Error-State Kalman Filter (ESKF) on $\text{SE}(3) \times \mathbb{R}^6$
- **High-Rate Strapdown Mechanization:** Propagates 3D position, velocity, and orientation using high-frequency 6-DOF IMU accelerometer and gyroscope observations.
- **Lie Group Orientation:** Represents vehicle attitude on $\text{SO}(3)$ using unit quaternions, avoiding gimbal lock and singularity artifacts.
- **Online Bias Estimation:** Dynamically estimates continuous accelerometer and gyroscope sensor bias walks.
- **Manifold Error State Reset:** Injects error states via quaternion exponential retracts and resets tangent-space errors cleanly.

### 2. Planar CTRV Extended Kalman Filter with Joseph Updates
- **Exact Circular-Arc Kinematics:** Analytical Jacobian propagation accounting for non-zero yaw rate curvature with Taylor fallback for $|\omega| \to 0$.
- **Continuous-to-Discrete Process Noise:** Projects physical body accelerations into Cartesian space along the circular midpoint chord.
- **Guaranteed Covariance Positive Definiteness:** Symmetric Joseph-stabilized measurement update preventing numerical covariance collapse.
- **Mahalanobis Outlier Gating:** Chi-squared $\chi^2$ innovation gating isolating multipath spikes and GPS jumps.

### 3. Model Predictive Path Guidance & Kinematics
- **Frenet-Frame Tracking:** Orthogonal projection calculating lateral cross-track errors ($e_y$) and heading deviations ($e_\psi$).
- **Optimal Curvature Feedforward:** Synthesizes steering commands matching instantaneous raceline curvature ($\delta_{\text{ff}} = \arctan(L \kappa)$).
- **Preview Trajectory Rollout:** Generates forward prediction horizon vectors for collision avoidance and cockpit display.

### 4. Closed-Loop Test Session Orchestrator & 7-Rule KPI Scorecard
- **Proving Ground State Machine:** Lifecycle management (IDLE, ARMED, RUNNING, PAUSED, COMPLETED, ABORTED) for autonomous proving ground campaigns.
- **Automated ISO 21448 / SOTIF Scorecard:** Evaluates 7 regulatory pass/fail rules:
  1. *Rule 1 (Position Accuracy):* 2D Position RMSE $\le 0.35\,\text{m}$.
  2. *Rule 2 (Lateral Tracking):* Max Cross-Track Error $\le 0.60\,\text{m}$.
  3. *Rule 3 (Geofence Clearance):* Minimum Track Wall Margin $\ge 0.50\,\text{m}$.
  4. *Rule 4 (Safety Integrity):* Horizontal Protection Level $\text{HPL} \le 3.00\,\text{m}$.
  5. *Rule 5 (Kalman Consistency):* NEES within theoretical confidence bounds $[2.0, 5.5]$.
  6. *Rule 6 (Real-Time Determinism):* Controller solve latency $\le 5.0\,\text{ms}$.
  7. *Rule 7 (Zero Disengagements):* Total disengagements $= 0$.

### 5. Proving Ground Multi-Sensor Stress & Replay Engine
- **Deterministic Fault Injection:** Pre-configured test scenarios simulating tunnel GNSS blackouts, high-variance urban canyon multipaths, and camera blinding glare.
- **Circular Telemetry Recording:** 5,000-frame circular buffer recording synchronized true/estimated states, residuals, and protection levels.
- **Forensic Incident Extraction:** Automatically captures $+/- 50$ frames around safety threshold violations for offline analysis.

### 6. Modern 8-Screen Engineering Glass Cockpit
- Real-time 20 Hz WebSocket streaming engine providing deep introspection into:
  1. Real-Time Vehicle Status
  2. Localization Quality & Safety Integrity
  3. Sensor Health & Innovation Residuals (NIS)
  4. Route Tracking & Curvilinear Frenet Errors
  5. Autonomy Quality & Chassis Dynamics
  6. Safety Events & Emergency Stop Controls
  7. Session Analytics & Sector Split Timing
  8. Historical Analysis & Frame-by-Frame Scrubbing

### 7. Complete Open Source Packaging & Verification
- **Modern Packaging:** Standard `pyproject.toml` (PEP 517/518/621), `requirements.txt`, `requirements-dev.txt`, and `.editorconfig`.
- **Licensing:** Apache License 2.0 with clear attribution for the third-party GPL-3.0 racetrack datasets in `NOTICE`.
- **Automated Testing:** 19 unit and integration tests with 100% pass rate.
- **CI/CD:** Multi-OS matrix testing across Ubuntu, Windows, and macOS on Python 3.10–3.13.
- **Containerization:** Multi-stage production `Dockerfile` and `docker-compose.yml`.

---

## 23-Track Benchmark Summary

| Metric | Raw GPS | EKF Estimator | Overall Improvement |
| :--- | :---: | :---: | :---: |
| **2D Position RMSE (Average across 23 Tracks)** | 2.12 m | **0.214 m** | **89.9%** |
| **Lateral Frenet Cross-Track RMSE** | 1.50 m | **0.166 m** | **88.9%** |
| **CEP50 (Median Position Error)** | 1.82 m | **0.175 m** | **90.4%** |
| **CEP95 (95th Percentile Error)** | 3.65 m | **0.373 m** | **89.8%** |

---

## Installation & Quickstart

```bash
# Install directly from source in editable mode
pip install -e .

# Launch Live Engineering Cockpit on Monza
apexpath-cockpit --track Monza --port 8000

# Execute batch benchmarks across all 23 tracks
apexpath-benchmark
```
