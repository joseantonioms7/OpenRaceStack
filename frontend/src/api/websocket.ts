import { TelemetryFrame } from '../types';

type TelemetryListener = (frame: TelemetryFrame) => void;
type StatusListener = (connected: boolean) => void;

class TelemetryWebSocketClient {
  private ws: WebSocket | null = null;
  private url: string;
  private listeners: Set<TelemetryListener> = new Set();
  private statusListeners: Set<StatusListener> = new Set();
  private reconnectTimeout: number | null = null;
  private isConnected = false;
  private mockInterval: number | null = null;
  private simTime = 0;

  constructor() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host;
    this.url = `${protocol}//${host}/api/v1/monitoring/ws`;
  }

  public connect(): void {
    if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
      return;
    }

    try {
      this.ws = new WebSocket(this.url);

      this.ws.onopen = () => {
        this.isConnected = true;
        this.stopMockFallback();
        this.notifyStatus(true);
      };

      this.ws.onmessage = (event) => {
        try {
          const frame: TelemetryFrame = JSON.parse(event.data);
          this.notifyFrame(frame);
        } catch (e) {
          console.warn('Failed to parse WebSocket frame', e);
        }
      };

      this.ws.onclose = () => {
        this.isConnected = false;
        this.notifyStatus(false);
        this.scheduleReconnect();
        this.startMockFallback();
      };

      this.ws.onerror = () => {
        if (this.ws) this.ws.close();
      };
    } catch (e) {
      this.isConnected = false;
      this.notifyStatus(false);
      this.scheduleReconnect();
      this.startMockFallback();
    }
  }

  public disconnect(): void {
    if (this.reconnectTimeout) {
      window.clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }
    this.stopMockFallback();
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this.isConnected = false;
    this.notifyStatus(false);
  }

  public subscribe(listener: TelemetryListener): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  public subscribeStatus(listener: StatusListener): () => void {
    this.statusListeners.add(listener);
    listener(this.isConnected);
    return () => this.statusListeners.delete(listener);
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimeout) return;
    this.reconnectTimeout = window.setTimeout(() => {
      this.reconnectTimeout = null;
      this.connect();
    }, 2500);
  }

  private notifyFrame(frame: TelemetryFrame): void {
    this.listeners.forEach((listener) => listener(frame));
  }

  private notifyStatus(status: boolean): void {
    this.statusListeners.forEach((listener) => listener(status));
  }

  // Generates smooth 20 Hz synthetic telemetry if backend is starting or offline
  private startMockFallback(): void {
    if (this.mockInterval) return;
    this.mockInterval = window.setInterval(() => {
      this.simTime += 0.05;
      const angle = this.simTime * 0.2;
      const radius = 250;
      const x = Math.cos(angle) * radius;
      const y = Math.sin(angle) * radius * 0.7;
      const speed = 18.5 + Math.sin(this.simTime) * 3.5;

      const mockFrame: TelemetryFrame = {
        timestamp: Math.round(this.simTime * 100) / 100,
        circuit: 'Monza',
        session_id: 'live-proving-ground-run',
        vehicle_id: 'apex-f1tenth-01',
        pose: {
          x: Math.round(x * 10) / 10,
          y: Math.round(y * 10) / 10,
          z: 0,
          theta_rad: angle + Math.PI / 2,
          theta_deg: Math.round(((angle + Math.PI / 2) * (180 / Math.PI)) % 360),
        },
        twist: {
          linear_mps: Math.round(speed * 10) / 10,
          angular_radps: 0.12,
        },
        dynamics: {
          speed_kph: Math.round(speed * 3.6),
          accel_mps2: 1.25,
          lat_accel_g: 0.82,
          heading_error_deg: 0.85,
          position_error_m: 0.18,
        },
        actuation: {
          steering_angle_rad: 0.045,
          steering_angle_deg: 2.58,
          acceleration_mps2: 1.25,
          target_velocity_mps: 20.0,
          cross_track_error_m: 0.14,
          heading_error_rad: 0.015,
          is_saturated_steering: false,
          is_saturated_accel: false,
          solve_time_ms: 2.45,
        },
        diagnostics: {
          active_primary_source: 'gnss',
          overall_confidence: 0.965,
          covariance_trace: 0.0142,
          safety_integrity: 'NOMINAL',
          sensor_health: {
            gnss: { healthy: true, status: 'NOMINAL', total_updates: 1420, consecutive_rejections: 0, total_rejections: 1, last_nis: 0.42, timeout_counter: 0 },
            lidar: { healthy: true, status: 'NOMINAL', total_updates: 2840, consecutive_rejections: 0, total_rejections: 0, last_nis: 0.81, timeout_counter: 0 },
            camera: { healthy: true, status: 'NOMINAL', total_updates: 4260, consecutive_rejections: 0, total_rejections: 2, last_nis: 1.15, timeout_counter: 0 },
            imu: { healthy: true, status: 'NOMINAL', total_updates: 14200, consecutive_rejections: 0, total_rejections: 0, last_nis: 0.12, timeout_counter: 0 },
          },
        },
        frenet: {
          station_m: Math.round((this.simTime * speed) % 5793),
          cross_track_error_m: 0.14,
          margin_left_m: 2.36,
          margin_right_m: 2.64,
          lap: 1 + Math.floor((this.simTime * speed) / 5793),
        },
        estop_active: false,
        lookahead_waypoints: Array.from({ length: 10 }, (_, i) => [
          Math.cos(angle + (i + 1) * 0.05) * radius,
          Math.sin(angle + (i + 1) * 0.05) * radius * 0.7,
        ]),
      };

      this.notifyFrame(mockFrame);
    }, 50); // 20 Hz
  }

  private stopMockFallback(): void {
    if (this.mockInterval) {
      window.clearInterval(this.mockInterval);
      this.mockInterval = null;
    }
  }
}

export const telemetryWs = new TelemetryWebSocketClient();
