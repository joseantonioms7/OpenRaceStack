/**
 * ApexPathAI TypeScript Domain Definitions.
 * Comprehensive models covering all 10 platform pillars.
 */

export interface Pose {
  x: number;
  y: number;
  z: number;
  theta_rad: number;
  theta_deg: number;
}

export interface Twist {
  linear_mps: number;
  angular_radps: number;
}

export interface VehicleDynamics {
  speed_kph: number;
  accel_mps2: number;
  lat_accel_g: number;
  heading_error_deg: number;
  position_error_m: number;
}

export interface ActuationCommand {
  steering_angle_rad: number;
  steering_angle_deg: number;
  acceleration_mps2: number;
  target_velocity_mps: number;
  cross_track_error_m: number;
  heading_error_rad: number;
  is_saturated_steering: boolean;
  is_saturated_accel: boolean;
  solve_time_ms: number;
}

export interface SensorHealth {
  healthy: boolean;
  status: string;
  total_updates: number;
  consecutive_rejections: number;
  total_rejections: number;
  last_nis: number;
  timeout_counter: number;
}

export interface Diagnostics {
  active_primary_source: string;
  overall_confidence: number;
  covariance_trace: number;
  safety_integrity: string;
  sensor_health: Record<string, SensorHealth>;
}

export interface FrenetCoordinates {
  station_m: number;
  cross_track_error_m: number;
  margin_left_m: number;
  margin_right_m: number;
  lap: number;
}

export interface TelemetryFrame {
  timestamp: number;
  circuit: string;
  session_id: string | null;
  vehicle_id: string;
  pose: Pose;
  twist: Twist;
  dynamics: VehicleDynamics;
  actuation: ActuationCommand;
  diagnostics: Diagnostics;
  frenet: FrenetCoordinates;
  estop_active: boolean;
  lookahead_waypoints: [number, number][];
}

export interface Vehicle {
  id: string;
  name: string;
  vehicle_type: string;
  wheelbase_m: number;
  track_width_m: number;
  mass_kg: number;
  max_steering_rad: number;
  max_accel_mps2: number;
  max_speed_mps: number;
  sensor_extrinsics: Record<string, { dx: number; dy: number; dtheta: number }>;
  firmware_version: string;
  is_active: string;
  created_at: string;
  updated_at: string;
}

export interface CircuitSummary {
  id: string;
  name: string;
  country: string;
  length_m: number;
  width_m: number;
  waypoints_count: number;
  has_raceline: boolean;
  has_raster_map: boolean;
  is_active: boolean;
}

export interface CircuitDetail extends CircuitSummary {
  resolution_m: number;
  origin_x: number;
  origin_y: number;
  centerline_preview: [number, number][];
  raceline_preview: [number, number][];
}

export interface LapRecord {
  lap_number: number;
  lap_time_s: number;
  sector_1_s?: number;
  sector_2_s?: number;
  sector_3_s?: number;
  max_velocity_mps: number;
  avg_velocity_mps: number;
  lateral_rmse_m: number;
  max_lateral_m: number;
}

export interface TestSession {
  id: string;
  name: string;
  track_name: string;
  vehicle_id: string;
  profile: string;
  state: 'IDLE' | 'ARMED' | 'RUNNING' | 'PAUSED' | 'COMPLETED' | 'ABORTED';
  target_laps: number;
  completed_laps: number;
  best_lap_time_s?: number;
  position_rmse_m?: number;
  max_position_error_m?: number;
  verdict: 'PASS' | 'CONDITIONAL' | 'FAIL' | 'PENDING';
  start_time: string;
  end_time?: string;
  laps: LapRecord[];
  scorecard?: Record<string, any>;
}

export interface KPIRule {
  rule_id: string;
  rule_name: string;
  threshold: string;
  measured_value: string;
  passed: boolean;
  status: 'PASS' | 'CONDITIONAL' | 'FAIL';
  description: string;
}

export interface KPIScorecard {
  session_id: string;
  track_name: string;
  profile: string;
  overall_verdict: 'PASS' | 'CONDITIONAL' | 'FAIL';
  compliance_score_pct: number;
  rules: KPIRule[];
  summary_metrics: {
    rmse_position_m: number;
    max_position_error_m: number;
    max_speed_kph: number;
    total_samples: number;
  };
}

export interface IncidentBookmark {
  incident_id: string;
  timestamp: number;
  incident_type: string;
  severity: 'INFO' | 'WARNING' | 'CRITICAL';
  description: string;
  position: [number, number];
}

export interface ReplayTimeline {
  circuit_name: string;
  total_frames: number;
  duration_seconds: number;
  current_time: number;
  is_playing: boolean;
  playback_speed: number;
  incidents: IncidentBookmark[];
}

export interface ValidationReport {
  id: string;
  session_id: string;
  title: string;
  author: string;
  verdict: string;
  summary: string;
  content_markdown: string;
  data_snapshot: Record<string, any>;
  created_at: string;
}
