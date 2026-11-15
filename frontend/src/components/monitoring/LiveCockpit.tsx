import React, { useState, useEffect } from 'react';
import { Box, Grid, Paper, Typography, LinearProgress } from '@mui/material';
import { Gauge, Activity, ShieldCheck, Compass, Navigation } from 'lucide-react';
import { TelemetryFrame, CircuitDetail } from '../../types';
import { MetricCard } from '../common/MetricCard';
import { TrackVisualizer } from './TrackVisualizer';
import { api } from '../../api/client';
import {
  ResponsiveContainer,
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
} from 'recharts';

interface LiveCockpitProps {
  currentFrame: TelemetryFrame | null;
}

export const LiveCockpit: React.FC<LiveCockpitProps> = ({ currentFrame }) => {
  const [circuitDetail, setCircuitDetail] = useState<CircuitDetail | null>(null);
  const [historyTrail, setHistoryTrail] = useState<[number, number][]>([]);
  const [chartData, setChartData] = useState<any[]>([]);

  // Load circuit metadata
  useEffect(() => {
    const trackName = currentFrame?.circuit || 'Monza';
    api.getCircuitDetail(trackName)
      .then(setCircuitDetail)
      .catch((e) => console.warn('Could not load circuit detail', e));
  }, [currentFrame?.circuit]);

  // Append history trail and rolling chart
  useEffect(() => {
    if (!currentFrame) return;

    setHistoryTrail((prev) => {
      const next = [...prev, [currentFrame.pose.x, currentFrame.pose.y] as [number, number]];
      return next.length > 250 ? next.slice(-250) : next;
    });

    setChartData((prev) => {
      const item = {
        time: currentFrame.timestamp.toFixed(1),
        speed: currentFrame.dynamics.speed_kph,
        target_speed: Math.round(currentFrame.actuation.target_velocity_mps * 3.6),
        cte: Math.abs(currentFrame.actuation.cross_track_error_m),
      };
      const next = [...prev, item];
      return next.length > 30 ? next.slice(-30) : next;
    });
  }, [currentFrame]);

  const speedKph = currentFrame?.dynamics.speed_kph || 0;
  const cte = currentFrame?.actuation.cross_track_error_m || 0;
  const posError = currentFrame?.dynamics.position_error_m || 0;
  const confidence = Math.round((currentFrame?.diagnostics.overall_confidence || 0.95) * 100);

  return (
    <Box sx={{ p: 3, display: 'flex', flexDirection: 'column', gap: 3 }}>
      {/* 1. Header Metrics Row */}
      <Grid container spacing={2}>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="VEHICLE SPEED"
            value={speedKph.toFixed(1)}
            unit="KM/H"
            subtitle={`Target: ${(currentFrame?.actuation.target_velocity_mps ? currentFrame.actuation.target_velocity_mps * 3.6 : 0).toFixed(1)} km/h`}
            status="nominal"
            icon={<Gauge size={22} />}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="CROSS-TRACK ERROR"
            value={Math.abs(cte).toFixed(3)}
            unit="M"
            subtitle="Frenet lateral deviation"
            status={Math.abs(cte) < 0.35 ? 'nominal' : 'warning'}
            icon={<Activity size={22} />}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="EUCLIDEAN POSE ERROR"
            value={posError.toFixed(3)}
            unit="M"
            subtitle="Dual-stack EKF/ESKF delta"
            status={posError < 0.5 ? 'nominal' : 'critical'}
            icon={<Compass size={22} />}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="ESTIMATION CONFIDENCE"
            value={confidence}
            unit="%"
            subtitle={`Primary: ${currentFrame?.diagnostics.active_primary_source.toUpperCase() || 'GNSS'}`}
            status={confidence > 80 ? 'nominal' : 'warning'}
            icon={<ShieldCheck size={22} />}
          />
        </Grid>
      </Grid>

      {/* 2. Main Visualizer and Actuation Cockpit */}
      <Grid container spacing={3}>
        {/* Track Canvas & Pose Visualizer */}
        <Grid item xs={12} lg={8}>
          <Paper sx={{ p: 2, bgcolor: '#111827', borderRadius: 2 }}>
            <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 1.5 }}>
              <Typography variant="subtitle2" sx={{ fontWeight: 700, letterSpacing: 0.5, color: '#f3f4f6' }}>
                PROVING GROUND SPATIAL TRAJECTORY & LOOKAHEAD
              </Typography>
              <Typography variant="caption" sx={{ color: '#06b6d4', fontWeight: 600 }}>
                20 Hz HIGH-PRECISION INTEGRATION
              </Typography>
            </Box>
            <TrackVisualizer
              currentFrame={currentFrame}
              circuitDetail={circuitDetail}
              historyTrail={historyTrail}
            />
          </Paper>
        </Grid>

        {/* Dynamics, Actuation & Frenet Corridor Panel */}
        <Grid item xs={12} lg={4}>
          <Paper sx={{ p: 2.5, bgcolor: '#111827', borderRadius: 2, height: '100%', display: 'flex', flexDirection: 'column', gap: 2.5 }}>
            <Typography variant="subtitle2" sx={{ fontWeight: 700, letterSpacing: 0.5, color: '#f3f4f6' }}>
              ACTUATION & CORRIDOR DYNAMICS
            </Typography>

            {/* Steering Dial Gauge */}
            <Box sx={{ bgcolor: '#0b0f19', p: 2, borderRadius: 1.5, border: '1px solid #1f2937' }}>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                <Typography variant="caption" sx={{ color: '#9ca3af' }}>MPC STEERING ANGLE (δ)</Typography>
                <Typography variant="caption" sx={{ color: '#38bdf8', fontWeight: 700, fontFamily: 'monospace' }}>
                  {currentFrame?.actuation.steering_angle_deg.toFixed(2)}°
                </Typography>
              </Box>
              <LinearProgress
                variant="determinate"
                value={Math.min(100, Math.max(0, ((currentFrame?.actuation.steering_angle_deg || 0) + 24) * (100 / 48)))}
                sx={{ height: 8, borderRadius: 4, bgcolor: '#1f2937', '& .MuiLinearProgress-bar': { bgcolor: '#06b6d4' } }}
              />
              <Box sx={{ display: 'flex', justifyContent: 'space-between', mt: 0.5 }}>
                <Typography variant="caption" sx={{ color: '#4b5563' }}>-24° L</Typography>
                <Typography variant="caption" sx={{ color: '#4b5563' }}>0° Center</Typography>
                <Typography variant="caption" sx={{ color: '#4b5563' }}>+24° R</Typography>
              </Box>
            </Box>

            {/* Acceleration & Friction Utilization */}
            <Box sx={{ bgcolor: '#0b0f19', p: 2, borderRadius: 1.5, border: '1px solid #1f2937' }}>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                <Typography variant="caption" sx={{ color: '#9ca3af' }}>DRIVE ACCELERATION (a)</Typography>
                <Typography variant="caption" sx={{ color: '#10b981', fontWeight: 700, fontFamily: 'monospace' }}>
                  {currentFrame?.actuation.acceleration_mps2.toFixed(2)} m/s²
                </Typography>
              </Box>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                <Typography variant="caption" sx={{ color: '#9ca3af' }}>LATERAL ACCELERATION</Typography>
                <Typography variant="caption" sx={{ color: '#f59e0b', fontWeight: 700, fontFamily: 'monospace' }}>
                  {currentFrame?.dynamics.lat_accel_g.toFixed(2)} g
                </Typography>
              </Box>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                <Typography variant="caption" sx={{ color: '#9ca3af' }}>QP SOLVE TIME</Typography>
                <Typography variant="caption" sx={{ color: '#a855f7', fontWeight: 700, fontFamily: 'monospace' }}>
                  {currentFrame?.actuation.solve_time_ms.toFixed(2)} ms
                </Typography>
              </Box>
            </Box>

            {/* Frenet Corridor Boundaries */}
            <Box sx={{ bgcolor: '#0b0f19', p: 2, borderRadius: 1.5, border: '1px solid #1f2937' }}>
              <Typography variant="caption" sx={{ color: '#9ca3af', display: 'block', mb: 1 }}>
                FRENET CORRIDOR CLEARANCE
              </Typography>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
                <Typography variant="body2" sx={{ color: '#38bdf8', fontWeight: 600 }}>
                  Left Margin: {currentFrame?.frenet.margin_left_m.toFixed(2)} m
                </Typography>
                <Typography variant="body2" sx={{ color: '#38bdf8', fontWeight: 600 }}>
                  Right Margin: {currentFrame?.frenet.margin_right_m.toFixed(2)} m
                </Typography>
              </Box>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', mt: 1 }}>
                <Typography variant="caption" sx={{ color: '#6b7280' }}>
                  Track Station (s): {currentFrame?.frenet.station_m.toFixed(1)} m
                </Typography>
                <Typography variant="caption" sx={{ color: '#6b7280' }}>
                  Lap: {currentFrame?.frenet.lap}
                </Typography>
              </Box>
            </Box>
          </Paper>
        </Grid>
      </Grid>

      {/* 3. Rolling Telemetry Strip Charts */}
      <Paper sx={{ p: 2.5, bgcolor: '#111827', borderRadius: 2 }}>
        <Typography variant="subtitle2" sx={{ fontWeight: 700, letterSpacing: 0.5, color: '#f3f4f6', mb: 2 }}>
          ROLLING TELEMETRY (SPEED & LATERAL CONVERGENCE)
        </Typography>
        <Box sx={{ width: '100%', height: 220 }}>
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={chartData}>
              <CartesianGrid strokeDasharray="3 3" stroke="#1f2937" />
              <XAxis dataKey="time" stroke="#6b7280" tick={{ fontSize: 11 }} />
              <YAxis stroke="#6b7280" tick={{ fontSize: 11 }} />
              <Tooltip
                contentStyle={{ backgroundColor: '#0f172a', borderColor: '#334155', borderRadius: 6, color: '#f8fafc' }}
              />
              <Line type="monotone" dataKey="speed" name="Speed (km/h)" stroke="#06b6d4" strokeWidth={2} dot={false} />
              <Line type="monotone" dataKey="target_speed" name="Target (km/h)" stroke="#3b82f6" strokeWidth={1.5} strokeDasharray="4 4" dot={false} />
              <Line type="monotone" dataKey="cte" name="Cross-Track Error (m)" stroke="#f43f5e" strokeWidth={1.5} dot={false} />
            </LineChart>
          </ResponsiveContainer>
        </Box>
      </Paper>
    </Box>
  );
};
