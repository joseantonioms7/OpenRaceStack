# Proving Ground Scenarios & Stress Replay

ApexPathAI features a deterministic disturbance injection engine capable of stressing vehicle perception and state estimation stacks with edge-case scenarios.

---

## Pre-Configured Proving Ground Scenarios

### 1. Monza High-Speed Tunnel GNSS Blackout (`monza_tunnel_blackout`)
- **Circuit:** Monza (5,793 m)
- **Target Velocity:** 24.5 m/s (88 km/h)
- **Disturbance:** Complete loss of GNSS signal ($10.0\,\text{s}$ duration) in Sector 2 between stations 150m and 250m.
- **Evaluation Goal:** Validates inertial dead-reckoning drift and fallback to LiDAR/visual odometry under high-speed cornering.

### 2. Spa Extreme Weather & Surface Friction Drop (`spa_friction_drop`)
- **Circuit:** Spa-Francorchamps (7,004 m)
- **Target Velocity:** 22.0 m/s
- **Disturbance:** Sudden friction coefficient reduction ($\mu = 0.45$) and camera feature degradation (blinding spray).
- **Evaluation Goal:** Validates sideslip angle ($\beta$) monitoring and path containment under low-grip conditions.

### 3. Melbourne Urban Canyon Multipath Reflection (`melbourne_multipath`)
- **Circuit:** Melbourne Albert Park (5,278 m)
- **Target Velocity:** 20.0 m/s
- **Disturbance:** High-variance satellite multipath reflection noise ($\sigma_{\text{GPS}} = 4.5\,\text{m}$) and intermittent $15\,\text{m}$ position jumps.
- **Evaluation Goal:** Stresses the Mahalanobis chi-squared innovation gate and verifies consecutive rejection recovery.

---

## Circular Recording & Forensic Replay

During test execution, `StressReplayEngine` maintains a circular buffer of `TelemetryFrame` structures recording:
- True ground-truth vs estimated pose $[x, y, \theta]$
- Instantaneous lateral error ($e_y$) and heading error ($e_\psi$)
- Sensor availability flags (GNSS fix, camera feature tracking, LiDAR inlier ratio)
- Protection levels (HPL) and track margin clearance

When an anomaly or threshold violation occurs, the engine automatically extracts an incident snapshot (capturing 50 frames before and after the event) for forensic analysis in the engineering dashboard.
