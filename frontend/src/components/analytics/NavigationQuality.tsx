import React, { useState, useEffect } from 'react';
import {
  Box,
  Typography,
  Grid,
  Card,
  CardContent,
  Chip,
  LinearProgress,
  Button,
} from '@mui/material';
import {
  ResponsiveContainer,
  LineChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  CartesianGrid,
  Legend,
  AreaChart,
  Area,
} from 'recharts';
import { Compass, RefreshCw, AlertTriangle, ShieldCheck, Activity } from 'lucide-react';
import { MetricCard } from '../common/MetricCard';
import { api } from '../../api/client';

export const NavigationQuality: React.FC = () => {
  const [data, setData] = useState<any | null>(null);
  const [loading, setLoading] = useState(false);

  const fetchData = async () => {
    setLoading(true);
    try {
      const res = await api.getNavigationQuality(100);
      setData(res);
    } catch (err) {
      console.error('Failed to load navigation quality analytics', err);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchData();
    const interval = setInterval(fetchData, 4000);
    return () => clearInterval(interval);
  }, []);

  const samples = data?.samples || [];

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Box>
          <Typography variant="h5" sx={{ fontWeight: 700, color: '#f8fafc', display: 'flex', alignItems: 'center', gap: 1 }}>
            <Compass size={24} color="#06b6d4" />
            Navigation Quality & Path-Following Analytics
          </Typography>
          <Typography variant="body2" sx={{ color: '#94a3b8', mt: 0.5 }}>
            Frenet-frame curvilinear deviation ($e_y, e_\psi$), Stanford cross-track control error plane, and curvature feedforward.
          </Typography>
        </Box>
        <Button
          variant="outlined"
          size="small"
          startIcon={<RefreshCw size={14} />}
          onClick={fetchData}
          sx={{ borderColor: '#334155', color: '#94a3b8' }}
        >
          Refresh Metrics
        </Button>
      </Box>

      {loading && <LinearProgress sx={{ mb: 2, bgcolor: '#1e293b', '& .MuiLinearProgress-bar': { bgcolor: '#06b6d4' } }} />}

      {/* KPI Cards */}
      <Grid container spacing={2} sx={{ mb: 3 }}>
        <Grid item xs={12} sm={6} md={2.4}>
          <MetricCard
            title="Cross-Track RMSE"
            value={data ? `${data.cross_track_rmse_m.toFixed(3)} m` : '--'}
            subtitle="Rule 2 Threshold: <= 0.35 m"
            color={data && data.cross_track_rmse_m <= 0.35 ? '#10b981' : '#f59e0b'}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={2.4}>
          <MetricCard
            title="Peak Lateral Error"
            value={data ? `${data.cross_track_max_m.toFixed(3)} m` : '--'}
            subtitle="Worst-case apex departure"
            color={data && data.cross_track_max_m <= 0.60 ? '#10b981' : '#ef4444'}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={2.4}>
          <MetricCard
            title="Heading RMSE"
            value={data ? `${data.heading_error_rmse_deg.toFixed(2)}°` : '--'}
            subtitle="Raceline yaw alignment"
            color="#06b6d4"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={2.4}>
          <MetricCard
            title="Steering Saturation"
            value={data ? `${data.steering_saturation_rate_pct.toFixed(1)}%` : '--'}
            subtitle="Actuator limit occurrences"
            color={data && data.steering_saturation_rate_pct < 5.0 ? '#10b981' : '#f59e0b'}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={2.4}>
          <MetricCard
            title="Tracking Efficiency"
            value={data ? `${data.path_tracking_efficiency_pct.toFixed(1)}%` : '--'}
            subtitle="Curvilinear path score"
            color="#8b5cf6"
          />
        </Grid>
      </Grid>

      {/* Graphs */}
      <Grid container spacing={3}>
        <Grid item xs={12} lg={8}>
          <Card sx={{ bgcolor: '#0f172a', border: '1px solid #1e293b' }}>
            <CardContent>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
                <Typography variant="subtitle1" sx={{ fontWeight: 600, color: '#f8fafc' }}>
                  Frenet Lateral Error vs Stanford Controller Demand
                </Typography>
                <Chip size="small" label={`Proving Ground: ${data?.circuit_name || 'Monza'}`} sx={{ bgcolor: '#1e293b', color: '#94a3b8' }} />
              </Box>
              <Box sx={{ height: 320 }}>
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart data={samples}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
                    <XAxis dataKey="station_m" stroke="#64748b" tickFormatter={(v) => `${Math.round(v)}m`} />
                    <YAxis stroke="#64748b" />
                    <Tooltip contentStyle={{ backgroundColor: '#0f172a', borderColor: '#334155' }} />
                    <Legend />
                    <Line type="monotone" dataKey="cross_track_error_m" name="Cross-Track Error (m)" stroke="#06b6d4" dot={false} strokeWidth={2} />
                    <Line type="monotone" dataKey="stanford_lateral_demand_deg" name="Stanford Angle Demand (°)" stroke="#f59e0b" dot={false} strokeWidth={1.5} />
                    <Line type="monotone" dataKey="feedforward_demand_deg" name="Curvature Feedforward (°)" stroke="#8b5cf6" dot={false} strokeWidth={1.5} />
                  </LineChart>
                </ResponsiveContainer>
              </Box>
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12} lg={4}>
          <Card sx={{ bgcolor: '#0f172a', border: '1px solid #1e293b', height: '100%' }}>
            <CardContent>
              <Typography variant="subtitle1" sx={{ fontWeight: 600, color: '#f8fafc', mb: 2 }}>
                Heading Alignment & Yaw Deviation
              </Typography>
              <Box sx={{ height: 320 }}>
                <ResponsiveContainer width="100%" height="100%">
                  <AreaChart data={samples}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
                    <XAxis dataKey="station_m" stroke="#64748b" tickFormatter={(v) => `${Math.round(v)}m`} />
                    <YAxis stroke="#64748b" />
                    <Tooltip contentStyle={{ backgroundColor: '#0f172a', borderColor: '#334155' }} />
                    <Legend />
                    <Area type="monotone" dataKey="heading_error_deg" name="Heading Deviation (°)" stroke="#10b981" fill="rgba(16, 185, 129, 0.2)" />
                  </AreaChart>
                </ResponsiveContainer>
              </Box>
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12}>
          <Card sx={{ bgcolor: '#0f172a', border: '1px solid #1e293b' }}>
            <CardContent>
              <Typography variant="subtitle1" sx={{ fontWeight: 600, color: '#f8fafc', mb: 1 }}>
                Automated Guidance Validation Standards
              </Typography>
              <Typography variant="body2" sx={{ color: '#94a3b8', mb: 2 }}>
                In compliance with ISO 21448 (SOTIF) and dynamic track boundaries, cross-track error is evaluated with dynamic preview horizon steps ($N=15, \Delta t=0.05\,\text{s}$). Steering commands are subject to strict slew rate gating to prevent high-speed centrifugal instability.
              </Typography>
              <Grid container spacing={2}>
                <Grid item xs={12} md={4}>
                  <Box sx={{ p: 2, bgcolor: '#1e293b', borderRadius: 2 }}>
                    <Typography variant="caption" sx={{ color: '#64748b', textTransform: 'uppercase' }}>Guidance Controller</Typography>
                    <Typography variant="h6" sx={{ color: '#06b6d4' }}>Frenet MPC + Stanley Slew</Typography>
                    <Typography variant="body2" sx={{ color: '#94a3b8', fontSize: '0.8rem', mt: 0.5 }}>Wheelbase: 0.33m / Horizon: 15 steps (750 ms preview)</Typography>
                  </Box>
                </Grid>
                <Grid item xs={12} md={4}>
                  <Box sx={{ p: 2, bgcolor: '#1e293b', borderRadius: 2 }}>
                    <Typography variant="caption" sx={{ color: '#64748b', textTransform: 'uppercase' }}>Cross-Track Gate</Typography>
                    <Typography variant="h6" sx={{ color: '#10b981' }}>|e_y| &lt; 0.35 m (Nominal)</Typography>
                    <Typography variant="body2" sx={{ color: '#94a3b8', fontSize: '0.8rem', mt: 0.5 }}>Enforces 0.50m minimum clearance to proving ground walls</Typography>
                  </Box>
                </Grid>
                <Grid item xs={12} md={4}>
                  <Box sx={{ p: 2, bgcolor: '#1e293b', borderRadius: 2 }}>
                    <Typography variant="caption" sx={{ color: '#64748b', textTransform: 'uppercase' }}>Curvature Feedforward</Typography>
                    <Typography variant="h6" sx={{ color: '#8b5cf6' }}>Active ($\kappa(s)$ Raceline)</Typography>
                    <Typography variant="body2" sx={{ color: '#94a3b8', fontSize: '0.8rem', mt: 0.5 }}>Calculates preview centrifugal bias prior to corner turn-in</Typography>
                  </Box>
                </Grid>
              </Grid>
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Box>
  );
};
