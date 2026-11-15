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
  AreaChart,
  Area,
  BarChart,
  Bar,
} from 'recharts';
import { api } from '../../api/client';
import { MetricCard } from '../common/MetricCard';

export const LocalizationAnalytics: React.FC = () => {
  const [data, setData] = useState<any | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    api.getAnalyticsSummary(200)
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

  const errDist = data?.error_distribution || [];

  return (
    <Box sx={{ p: 3, display: 'flex', flexDirection: 'column', gap: 3 }}>
      <Box>
        <Typography variant="h5" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
          LOCALIZATION ACCURACY & CONVERGENCE ANALYTICS
        </Typography>
        <Typography variant="body2" sx={{ color: '#9ca3af' }}>
          Continuous evaluation of Euclidean position errors, 3-sigma covariance containment, and dual-stack Kalman filtering.
        </Typography>
      </Box>

      {/* KPI Stats */}
      <Grid container spacing={2}>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="POSITION RMSE"
            value={data?.rmse_position_m?.toFixed(3) || '0.000'}
            unit="M"
            subtitle="ISO 26262 Threshold: <= 0.50m"
            status={data?.rmse_position_m < 0.5 ? 'nominal' : 'critical'}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="PEAK DEVIATION"
            value={data?.max_position_error_m?.toFixed(3) || '0.000'}
            unit="M"
            subtitle="Maximum transient divergence"
            status={data?.max_position_error_m < 1.2 ? 'nominal' : 'warning'}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="MEAN LATERAL ERROR"
            value={data?.mean_lateral_error_m?.toFixed(3) || '0.000'}
            unit="M"
            subtitle="Cross-track raceline accuracy"
            status="nominal"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="HEADING ERROR"
            value={data?.mean_heading_error_deg?.toFixed(2) || '0.00'}
            unit="DEG"
            subtitle="Attitude tracking alignment"
            status="nominal"
          />
        </Grid>
      </Grid>

      {/* Error vs 3-Sigma Bound Chart */}
      <Paper sx={{ p: 2.5, bgcolor: '#111827', borderRadius: 2 }}>
        <Typography variant="subtitle2" sx={{ fontWeight: 700, color: '#f3f4f6', mb: 2 }}>
          EUCLIDEAN POSITION ERROR vs. 3σ COVARIANCE UNCERTAINTY BOUND (ALONG TRACK STATION s)
        </Typography>
        <Box sx={{ width: '100%', height: 280 }}>
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={errDist}>
              <CartesianGrid strokeDasharray="3 3" stroke="#1f2937" />
              <XAxis dataKey="station_m" stroke="#6b7280" tick={{ fontSize: 11 }} label={{ value: 'Track Station s (m)', position: 'insideBottom', offset: -5, fill: '#6b7280' }} />
              <YAxis stroke="#6b7280" tick={{ fontSize: 11 }} label={{ value: 'Error (m)', angle: -90, position: 'insideLeft', fill: '#6b7280' }} />
              <Tooltip contentStyle={{ backgroundColor: '#0f172a', borderColor: '#334155', borderRadius: 6, color: '#f8fafc' }} />
              <Area type="monotone" dataKey="sigma_3_bound_m" name="3σ Covariance Envelope" stroke="#3b82f6" fill="#3b82f6" fillOpacity={0.15} />
              <Line type="monotone" dataKey="euclidean_error_m" name="Euclidean Error (m)" stroke="#06b6d4" strokeWidth={2} dot={false} />
              <Line type="monotone" dataKey="lateral_error_m" name="Lateral Error (m)" stroke="#f43f5e" strokeWidth={1.5} dot={false} />
            </AreaChart>
          </ResponsiveContainer>
        </Box>
      </Paper>

      {/* Heading Alignment Chart */}
      <Paper sx={{ p: 2.5, bgcolor: '#111827', borderRadius: 2 }}>
        <Typography variant="subtitle2" sx={{ fontWeight: 700, color: '#f3f4f6', mb: 2 }}>
          HEADING & ATTITUDE ERROR DISTRIBUTION (DEGREES)
        </Typography>
        <Box sx={{ width: '100%', height: 220 }}>
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={errDist}>
              <CartesianGrid strokeDasharray="3 3" stroke="#1f2937" />
              <XAxis dataKey="station_m" stroke="#6b7280" tick={{ fontSize: 11 }} />
              <YAxis stroke="#6b7280" tick={{ fontSize: 11 }} />
              <Tooltip contentStyle={{ backgroundColor: '#0f172a', borderColor: '#334155', borderRadius: 6, color: '#f8fafc' }} />
              <Line type="monotone" dataKey="heading_error_deg" name="Yaw Error (deg)" stroke="#f59e0b" strokeWidth={1.8} dot={false} />
            </LineChart>
          </ResponsiveContainer>
        </Box>
      </Paper>
    </Box>
  );
};
