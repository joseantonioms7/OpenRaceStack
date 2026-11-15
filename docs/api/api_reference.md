# REST & WebSocket API Reference 📡

ApexPathAI exposes a comprehensive RESTful API and a high-frequency WebSocket streaming endpoint under `/api/v1`.

Interactive OpenAPI documentation is available live at:
- **Swagger UI:** `http://localhost:8000/docs`
- **ReDoc:** `http://localhost:8000/redoc`

---

## 🛰️ 1. Real-Time Telemetry & Monitoring

### `GET /api/v1/monitoring/snapshot`
Returns the latest instantaneous vehicle pose, twist, actuation, sensor health, and Frenet tracking status.

### `POST /api/v1/monitoring/estop`
Triggers an immediate software Emergency Stop (E-STOP), bringing actuator demands to zero and logging an emergency event.

### `POST /api/v1/monitoring/reset-estop`
Clears active emergency stops and returns the platform to ready status.

### `WS /api/v1/monitoring/ws`
Full-duplex WebSocket streaming at 20 Hz (50 ms epoch). Pushes JSON telemetry frames:
```json
{
  "timestamp": 12.45,
  "circuit": "Monza",
  "vehicle_id": "apex-f1tenth-01",
  "pose": { "x": 124.5, "y": 88.2, "theta_rad": 1.54, "theta_deg": 88.2 },
  "twist": { "linear_mps": 18.4, "angular_radps": 0.12 },
  "dynamics": { "speed_kph": 66.2, "accel_mps2": 1.2, "lat_accel_g": 0.42 },
  "actuation": { "steering_angle_deg": -2.4, "target_velocity_mps": 20.0, "cross_track_error_m": 0.08 },
  "diagnostics": { "active_primary_source": "FUSION_EKF", "overall_confidence": 0.96 }
}
```

---

## 🏎️ 2. Vehicle Fleet Management

- `GET /api/v1/vehicles`: List all registered fleet vehicles.
- `GET /api/v1/vehicles/{vehicle_id}`: Retrieve vehicle parameters and sensor extrinsics.
- `POST /api/v1/vehicles`: Register a new vehicle model (wheelbase, max speed, steering limit).
- `PUT /api/v1/vehicles/{vehicle_id}`: Update kinematic configurations.
- `DELETE /api/v1/vehicles/{vehicle_id}`: Remove vehicle from fleet catalog.
- `POST /api/v1/vehicles/{vehicle_id}/select`: Set active vehicle model.

---

## 🏁 3. Circuit Proving Grounds

- `GET /api/v1/circuits`: List all 23 available Grand Prix circuits with lengths and waypoint counts.
- `GET /api/v1/circuits/{circuit_name}`: Retrieve detailed centerline and raceline coordinates preview.
- `POST /api/v1/circuits/select`: Switch proving ground simulation to the specified circuit.

---

## ⏱️ 4. Test Session Management

- `GET /api/v1/sessions`: List historical test sessions.
- `GET /api/v1/sessions/active`: Retrieve current running session with live sector splits.
- `GET /api/v1/sessions/{session_id}`: Detailed session metrics and lap records.
- `POST /api/v1/sessions`: Create and configure a new test campaign.
- `POST /api/v1/sessions/{session_id}/transition`: Command lifecycle state change (`arm`, `start`, `pause`, `stop`, `abort`).

---

## 📐 5. Localization & Vehicle Analytics

- `GET /api/v1/analytics/summary`: Instantaneous statistical summary (RMSE, max position error, sample count).
- `GET /api/v1/analytics/localization`: Deep localization metrics (CEP50, CEP95, Maximum HPL, NEES).
- `GET /api/v1/analytics/dynamics`: Dynamic envelope analysis (sideslip angle $\beta$, lateral acceleration, curvature $\kappa$).

---

## 🛡️ 6. Sensor Diagnostics & Fault Injection

- `GET /api/v1/sensors/matrix`: Real-time health status and innovation residuals (NIS) for GNSS, LiDAR, Camera, and IMU.
- `POST /api/v1/sensors/fault`: Inject controlled sensor disturbances (GNSS dropout, multipath noise, IMU drift).
- `POST /api/v1/sensors/fault/clear`: Reset all active sensor faults.

---

## ⏪ 7. Incident Replay System

- `GET /api/v1/replay/timeline`: Retrieve scrubbable replay buffer with auto-bookmarked incidents.
- `POST /api/v1/replay/seek`: Seek playback position to specific timestamp.
- `POST /api/v1/replay/control`: Control playback state (`play`, `pause`, `step_forward`, `step_backward`, `reset`).

---

## 🏆 8. Regulatory KPI Analysis

- `GET /api/v1/kpis/rules`: List all 7 regulatory safety rules and tolerance thresholds.
- `GET /api/v1/kpis/evaluate`: Evaluate real-time performance against ISO 21448 rules and return pass/fail scorecard.

---

## 📑 9. Validation Dossier Reporting

- `GET /api/v1/reports`: List compiled proving ground audit dossiers.
- `POST /api/v1/reports/generate`: Compile formal validation dossier with executive summary and sign-offs.
- `GET /api/v1/reports/{report_id}`: Retrieve report content (supports Markdown, JSON, and PDF formats).
