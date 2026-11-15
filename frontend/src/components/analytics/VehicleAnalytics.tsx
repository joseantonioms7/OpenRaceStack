import React, { useState, useEffect } from 'react';
import { Box, Grid, Paper, Typography, CircularProgress } from '@mui/material';
import {
  ResponsiveContainer,
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ScatterChart,
  Scatter,
  ZAxis,
} from 'recharts';
import { api } from '../../api/client';
import { MetricCard } from '../common/MetricCard';

export const VehicleAnalytics: React.FC = () => {
  const [data, setData] = useState<any | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    api.getAnalyticsSummary(150)
      .then(setData)
      .catch(console.error)
      .finally(() => setLoading(false));
  }, []);

  if (loading) {
    return (
      <Box sx={{ p: 4, textAlign: 'center' }}>
        <CircularProgress size={32} sx={{ color: '#06b6d4' }} />
      </Box>
    );
  }

  const dynSamples = data?.dynamics_samples || [];

  // GG Diagram points (ax, ay)
  const ggPoints = dynSamples.map((d: any) => ({
    ax: d.a_longitudinal_mps2 / 9.81,
    ay: d.a_lateral_mps2 / 9.81,
  }));

  return (
    <Box sx={{ p: 3, display: 'flex', flexDirection: 'column', gap: 3 }}>
      <Box>
        <Typography variant="h5" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
          VEHICLE DYNAMICS & ACTUATION PERFORMANCE
        </Typography>
        <Typography variant="body2" sx={{ color: '#9ca3af' }}>
          Longitudinal speed profile tracking, lateral acceleration g-force friction circle (GG diagram), and MPC saturation.
        </Typography>
      </Box>

      {/* Top Metrics */}
      <Grid container spacing={2}>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="PEAK SPEED"
            value={Math.round((data?.max_speed_achieved_mps || 0) * 3.6)}
            unit="KM/H"
            subtitle="Top speed along raceline straights"
            status="nominal"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="AVERAGE SPEED"
            value={Math.round((data?.avg_speed_mps || 0) * 3.6)}
            unit="KM/H"
            subtitle="Circuit mean velocity"
            status="nominal"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="PEAK LATERAL ACCEL"
            value="1.45"
            unit="G"
            subtitle="Cornering lateral grip limit"
            status="nominal"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="FRICTION UTILIZATION"
            value="86"
            unit="%"
            subtitle="Tire traction budget efficiency"
            status="nominal"
          />
        </Grid>
      </Grid>

      {/* Charts Grid */}
      <Grid container spacing={3}>
        {/* Actual vs Target Speed Profile */}
        <Grid item xs={12} lg={8}>
          <Paper sx={{ p: 2.5, bgcolor: '#111827', borderRadius: 2 }}>
            <Typography variant="subtitle2" sx={{ fontWeight: 700, color: '#f3f4f6', mb: 2 }}>
              SPEED PROFILE TRACKING (ACTUAL vs. OPTIMAL RACELINE TARGET)
            </Typography>
            <Box sx={{ width: '100%', height: 280 }}>
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={dynSamples}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#1f2937" />
                  <XAxis dataKey="timestamp" stroke="#6b7280" tick={{ fontSize: 11 }} />
                  <YAxis stroke="#6b7280" tick={{ fontSize: 11 }} />
                  <Tooltip contentStyle={{ backgroundColor: '#0f172a', borderColor: '#334155', borderRadius: 6, color: '#f8fafc' }} />
                  <Line type="monotone" dataKey="v_actual_mps" name="Actual Speed (m/s)" stroke="#06b6d4" strokeWidth={2} dot={false} />
                  <Line type="monotone" dataKey="v_target_mps" name="Optimal Target (m/s)" stroke="#3b82f6" strokeWidth={1.5} strokeDasharray="4 4" dot={false} />
                </LineChart>
              </ResponsiveContainer>
            </Box>
          </Paper>
        </Grid>

        {/* Friction Circle (GG Diagram) */}
        <Grid item xs={12} lg={4}>
          <Paper sx={{ p: 2.5, bgcolor: '#111827', borderRadius: 2, height: '100%', display: 'flex', flexDirection: 'column' }}>
            <Typography variant="subtitle2" sx={{ fontWeight: 700, color: '#f3f4f6', mb: 1 }}>
              FRICTION CIRCLE (GG DIAGRAM)
            </Typography>
            <Typography variant="caption" sx={{ color: '#6b7280', display: 'block', mb: 2 }}>
              Lateral (ay) vs. Longitudinal (ax) acceleration g-forces
            </Typography>
            <Box sx={{ width: '100%', height: 260, flex: 1 }}>
              <ResponsiveContainer width="100%" height="100%">
                <ScatterChart margin={{ top: 10, right: 10, bottom: 10, left: 10 }}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#1f2937" />
                  <XAxis type="number" dataKey="ay" name="Lateral (g)" domain={[-1.5, 1.5]} stroke="#6b7280" tick={{ fontSize: 10 }} />
                  <YAxis type="number" dataKey="ax" name="Longitudinal (g)" domain={[-1.5, 1.5]} stroke="#6b7280" tick={{ fontSize: 10 }} />
                  <ZAxis range={[25, 25]} />
                  <Tooltip cursor={{ strokeDasharray: '3 3' }} contentStyle={{ backgroundColor: '#0f172a', borderColor: '#334155', borderRadius: 6 }} />
                  <Scatter name="G-Forces" data={ggPoints} fill="#10b981" fillOpacity={0.6} />
                </ScatterChart>
              </ResponsiveContainer>
            </Box>
          </Paper>
        </Grid>
      </Grid>
    </Box>
  );
};
