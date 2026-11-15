import {
  CircuitSummary,
  CircuitDetail,
  Vehicle,
  TestSession,
  KPIScorecard,
  ReplayTimeline,
  ValidationReport,
  TelemetryFrame
} from '../types';

const API_BASE = '/api/v1';

async function fetchJson<T>(url: string, options?: RequestInit): Promise<T> {
  const res = await fetch(`${API_BASE}${url}`, {
    headers: {
      'Content-Type': 'application/json',
      ...options?.headers,
    },
    ...options,
  });
  if (!res.ok) {
    const errorText = await res.text();
    throw new Error(`API Error ${res.status}: ${errorText || res.statusText}`);
  }
  return res.json();
}

export const api = {
  // Circuits
  getCircuits: () => fetchJson<CircuitSummary[]>('/circuits'),
  getCircuitDetail: (name: string) => fetchJson<CircuitDetail>(`/circuits/${encodeURIComponent(name)}`),
  selectCircuit: (circuit_name: string) => fetchJson<{ status: string; active_circuit: string }>('/circuits/select', {
    method: 'POST',
    body: JSON.stringify({ circuit_name }),
  }),

  // Vehicles
  getVehicles: () => fetchJson<Vehicle[]>('/vehicles'),
  getVehicle: (id: string) => fetchJson<Vehicle>(`/vehicles/${id}`),
  createVehicle: (data: Partial<Vehicle>) => fetchJson<Vehicle>('/vehicles', {
    method: 'POST',
    body: JSON.stringify(data),
  }),
  updateVehicle: (id: string, data: Partial<Vehicle>) => fetchJson<Vehicle>(`/vehicles/${id}`, {
    method: 'PUT',
    body: JSON.stringify(data),
  }),
  deleteVehicle: (id: string) => fetchJson<{ status: string }>(`/vehicles/${id}`, {
    method: 'DELETE',
  }),
  selectVehicle: (id: string) => fetchJson<{ status: string }>('/vehicles/' + id + '/select', {
    method: 'POST',
  }),

  // Sessions
  getSessions: () => fetchJson<TestSession[]>('/sessions'),
  getActiveSession: () => fetchJson<TestSession | null>('/sessions/active'),
  getSession: (id: string) => fetchJson<TestSession>(`/sessions/${id}`),
  createSession: (data: { name: string; track_name: string; vehicle_id: string; profile: string; target_laps: number }) =>
    fetchJson<TestSession>('/sessions', {
      method: 'POST',
      body: JSON.stringify(data),
    }),
  transitionSession: (sessionId: string, action: string) =>
    fetchJson<TestSession>(`/sessions/${sessionId}/transition`, {
      method: 'POST',
      body: JSON.stringify({ action }),
    }),

  // Monitoring & Telemetry
  getSnapshot: () => fetchJson<TelemetryFrame>('/monitoring/snapshot'),
  setEStop: (active: boolean) => fetchJson<{ status: string; estop_active: boolean }>('/monitoring/estop', {
    method: 'POST',
    body: JSON.stringify({ active }),
  }),
  resetEStop: () => fetchJson<{ status: string; estop_active: boolean }>('/monitoring/reset-estop', {
    method: 'POST',
  }),
  getSafetyEvents: () => fetchJson<{ total_events: number; estop_active: boolean; events: any[] }>('/monitoring/safety-events'),

  // Analytics & Navigation Quality
  getAnalyticsSummary: (max_points = 150) => fetchJson<any>(`/analytics/summary?max_points=${max_points}`),
  getNavigationQuality: (max_points = 150) => fetchJson<any>(`/analytics/navigation?max_points=${max_points}`),

  // Circuit Evaluation
  evaluateCircuit: (trackName: string) => fetchJson<any>(`/circuits/${encodeURIComponent(trackName)}/evaluate`),

  // Fleet Comparison
  getFleetComparison: () => fetchJson<any>('/vehicles/fleet-comparison'),

  // Sensors & Fault Injection
  getSensorMatrix: () => fetchJson<any>('/sensors/matrix'),
  injectFault: (data: { sensor_name: string; fault_type: string; duration_sec: number; magnitude: number }) =>
    fetchJson<{ status: string; message: string }>('/sensors/inject-fault', {
      method: 'POST',
      body: JSON.stringify(data),
    }),

  // Replay
  getReplayTimeline: () => fetchJson<ReplayTimeline>('/replay/timeline'),
  seekReplay: (target_time: number) => fetchJson<TelemetryFrame>('/replay/seek', {
    method: 'POST',
    body: JSON.stringify({ target_time }),
  }),
  controlReplay: (action: string, playback_speed = 1.0) => fetchJson<ReplayTimeline>('/replay/control', {
    method: 'POST',
    body: JSON.stringify({ action, playback_speed }),
  }),

  // KPIs & Scorecard
  getKPIScorecard: (session_id?: string) =>
    fetchJson<KPIScorecard>(`/kpis/scorecard${session_id ? `?session_id=${session_id}` : ''}`),

  // Reports
  getReports: () => fetchJson<ValidationReport[]>('/reports'),
  getReport: (id: string) => fetchJson<ValidationReport>(`/reports/${id}`),
  generateReport: (data: { session_id: string; title?: string; author?: string; format?: string }) =>
    fetchJson<ValidationReport>('/reports', {
      method: 'POST',
      body: JSON.stringify(data),
    }),
};
