# 8-Screen Engineering Glass Cockpit

The ApexPathAI glass-cockpit dashboard provides real-time telemetry streaming at 20 Hz over WebSockets, organized into 8 synchronized engineering sections.

---

## Cockpit Screens Breakdown

### 1. Real-Time Vehicle Status
- Longitudinal speed gauge (km/h and m/s).
- Acceleration vector ($a_x, a_y$) and total dynamic G-force.
- Vehicle sideslip angle ($\beta$) and steering demand ($\delta$).
- Brake and throttle actuation activity.

### 2. Localization Quality & Safety Integrity
- Estimated vs reference vehicle coordinates $[x, y, \theta]$.
- Covariance trace $\text{tr}(P)$ and uncertainty ellipse.
- Horizontal Protection Level (HPL) with $10^{-7}$ integrity risk bound.
- Circular Error Probable (CEP50 and CEP95) error radii.

### 3. Sensor Health & Innovation Residuals
- Normalized Innovation Squared (NIS) charts across active sensor channels.
- GNSS receiver fix quality (RTK Fixed, RTK Float, Dead Reckoning).
- IMU gyro/accelerometer health and bias drift tracking.
- LiDAR point-cloud inlier ratios and visual feature counts.

### 4. Route Tracking & Curvilinear Frenet Errors
- Cross-track error ($e_y$) and heading alignment error ($e_\psi$).
- Dynamic clearance to left and right track boundaries.
- Time-to-Boundary (TTB) collision avoidance margins.

### 5. Autonomy Quality & Chassis Dynamics
- Stanford phase-plane error metric ($e_{\text{stanford}} = e_\psi + \arctan(k \cdot e_y / v)$).
- Tire friction circle utilization ($\mu$-meter).
- Dynamic jerk ($j_x, j_y$) in $\text{m/s}^3$.

### 6. Safety Events & Emergency Actuation
- Station-stamped safety event feed (INFO, WARN, CRITICAL).
- Operator Emergency Stop (E-STOP) actuator and release controls.
- Active disturbance indicators (GNSS outage, multipath, camera blinding).

### 7. Session Analytics & 7-Rule KPI Scorecard
- Current lap time, sector splits (S1, S2, S3), and delta-to-best.
- Automated pass/fail status for all 7 regulatory rules.
- Session lifecycle status (ARMED, RUNNING, COMPLETED, ABORTED).

### 8. Historical Analysis & Replay Scrubber
- Interactive timeline scrubber for seeking to specific simulation frames.
- Forensic incident viewer reviewing +/- 50 frames around safety violations.
- Comparative multi-lap velocity and lateral deviation overlays.
