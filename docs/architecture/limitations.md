# System Assumptions, Limitations & Known Issues ⚠️

This document transparently specifies the theoretical assumptions, kinematic approximations, and operational boundaries of the **OpenRaceStack** and **ApexPathAI** frameworks.

---

## 1. Mathematical & Physical Approximations

### A. Planar 2D Track Envelope
- **Assumption:** Racetrack centerlines and vehicle states are modeled in the two-dimensional Cartesian plane $(\mathbb{R}^2)$ and Frenet curvilinear frame $(s, d)$.
- **Limitation:** Elevation grade changes $(\theta_{\text{grade}})$ and track banking camber $(\theta_{\text{bank}})$ are not currently integrated into the single-track dynamics equations.
- **Operational Boundary:** Validated for flat circuits or proving grounds with $< 3\%$ banking angle.

### B. Single-Track Bicycle Dynamics & Lateral Load Transfer
- **Assumption:** Left and right tires on each axle are lumped into single equivalent front and rear contact points.
- **Limitation:** While dynamic longitudinal load transfer (braking/acceleration pitch) and aerodynamic downforce are fully computed, lateral load transfer (chassis body roll and inner-versus-outer wheel unloading) is neglected.
- **Operational Boundary:** Highly accurate for low-to-medium roll angles ($a_y \le 2.5\,\text{G}$). Vehicles exhibiting high roll compliance may experience unmodeled inner tire lift-off.

### C. Isothermal Tire Models
- **Assumption:** `LinearTireModel` and `PacejkaTireModel` assume a constant bulk tire friction coefficient $(\mu)$ and nominal cornering stiffness $(C_\alpha)$.
- **Limitation:** Dynamic tire temperature buildup, thermal carcass expansion, and rubber graining degradation are not simulated.
- **Operational Boundary:** Transient tire heating over multi-lap endurance stints is not reflected in tire force generation.

---

## 2. Sensor & Perception Boundaries

### A. Degenerate Geometry in LiDAR Scan Matching
- **Limitation:** In long, featureless straight tunnel sectors with completely parallel smooth walls, 2D ICP and NDT scan matching experience longitudinal aperture ambiguity along the direction of motion.
- **Mitigation:** The dual Extended Kalman Filter (`ekf_engine.py` and `eskf_3d.py`) fuses wheel odometry and high-rate IMU dead-reckoning to constrain drift during geometric LiDAR degeneracy.

### B. Optical Feature Occlusion Under Direct Glare
- **Limitation:** The visual feature tracker (`visual_feature_tracker.py`) simulates realistic optical feature loss under sun glare and tunnel dark-to-light transitions. Prolonged total optical loss requires fallback to dead-reckoning.

---

## 3. Platform & Hardware Integration

### A. Operating System CAN Interface
- **Limitation:** High-speed native SocketCAN and CAN FD hardware bus communication require a Linux kernel (`can0`, `vcan0`).
- **Operational Boundary:** On Windows workstations, `DallaraAV21VehicleInterface` operates via simulated software loopback. Physical vehicle hardware deployment requires Linux Ubuntu 22.04 LTS or 24.04 LTS.

### B. Watchdog Heartbeat Preemption
- **Requirement:** Low-level vehicle interfaces enforce a hard $100\,\text{ms}$ heartbeat timeout. If the autonomy planner fails to dispatch a `VehicleControlCommand` within $100\,\text{ms}$, the system immediately enters `EMERGENCY_STOP` status and commands maximum braking deceleration.
