import React, { useState, useEffect } from 'react';
import {
  Box,
  Typography,
  Grid,
  Card,
  CardContent,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Chip,
  Button,
  LinearProgress,
} from '@mui/material';
import {
  ResponsiveContainer,
  BarChart,
  Bar,
  XAxis,
  YAxis,
  Tooltip,
  CartesianGrid,
  Legend,
} from 'recharts';
import { Users, RefreshCw, Award, Zap, ShieldCheck } from 'lucide-react';
import { MetricCard } from '../common/MetricCard';
import { api } from '../../api/client';

export const FleetComparisons: React.FC = () => {
  const [data, setData] = useState<any | null>(null);
  const [loading, setLoading] = useState(false);

  const fetchFleet = async () => {
    setLoading(true);
    try {
      const res = await api.getFleetComparison();
      setData(res);
    } catch (err) {
      console.error('Failed to load fleet comparison data', err);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchFleet();
  }, []);

  const fleet = data?.fleet_comparison || [];

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Box>
          <Typography variant="h5" sx={{ fontWeight: 700, color: '#f8fafc', display: 'flex', alignItems: 'center', gap: 1 }}>
            <Users size={24} color="#06b6d4" />
            Fleet Evaluation & Platform Benchmarks
          </Typography>
          <Typography variant="body2" sx={{ color: '#94a3b8', mt: 0.5 }}>
            Cross-platform verification across autonomous racing archetypes (1:10 scale vs Indy autonomous vs Level 4 Robotaxi).
          </Typography>
        </Box>
        <Button
          variant="outlined"
          size="small"
          startIcon={<RefreshCw size={14} />}
          onClick={fetchFleet}
          sx={{ borderColor: '#334155', color: '#94a3b8' }}
        >
          Refresh Comparison
        </Button>
      </Box>

      {loading && <LinearProgress sx={{ mb: 2, bgcolor: '#1e293b', '& .MuiLinearProgress-bar': { bgcolor: '#06b6d4' } }} />}

      {/* KPI Cards */}
      <Grid container spacing={2} sx={{ mb: 3 }}>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="Fleet Archetypes"
            value="3 Evaluated"
            subtitle="F1TENTH / Indy / Robotaxi"
            color="#06b6d4"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="Benchmark Circuit"
            value="Monza Grand Prix"
            subtitle="5,793m High-Speed Proving Ground"
            color="#10b981"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="Max Proving Ground Speed"
            value="280.8 km/h"
            subtitle="Dallara AV-21 Super-Speedway"
            color="#f59e0b"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="Safety Compliance"
            value="99.5% Avg"
            subtitle="ISO 21448 7-Rule Pass Rate"
            color="#8b5cf6"
          />
        </Grid>
      </Grid>

      {/* Comparative Graphs */}
      <Grid container spacing={3} sx={{ mb: 3 }}>
        <Grid item xs={12} md={6}>
          <Card sx={{ bgcolor: '#0f172a', border: '1px solid #1e293b' }}>
            <CardContent>
              <Typography variant="subtitle1" sx={{ fontWeight: 600, color: '#f8fafc', mb: 2 }}>
                Speed & Lateral G Capabilities
              </Typography>
              <Box sx={{ height: 280 }}>
                <ResponsiveContainer width="100%" height="100%">
                  <BarChart data={fleet}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
                    <XAxis dataKey="name" stroke="#64748b" tickFormatter={(n) => n.split(' ')[0]} />
                    <YAxis yAxisId="left" orientation="left" stroke="#06b6d4" />
                    <YAxis yAxisId="right" orientation="right" stroke="#f59e0b" />
                    <Tooltip contentStyle={{ backgroundColor: '#0f172a', borderColor: '#334155' }} />
                    <Legend />
                    <Bar yAxisId="left" dataKey="top_speed_kph" name="Top Speed (km/h)" fill="#06b6d4" radius={[4, 4, 0, 0]} />
                    <Bar yAxisId="right" dataKey="max_lateral_g" name="Max Lateral (G)" fill="#f59e0b" radius={[4, 4, 0, 0]} />
                  </BarChart>
                </ResponsiveContainer>
              </Box>
            </CardContent>
          </Card>
        </Grid>

        <Grid item xs={12} md={6}>
          <Card sx={{ bgcolor: '#0f172a', border: '1px solid #1e293b' }}>
            <CardContent>
              <Typography variant="subtitle1" sx={{ fontWeight: 600, color: '#f8fafc', mb: 2 }}>
                State Estimation & Guidance Accuracy (RMSE)
              </Typography>
              <Box sx={{ height: 280 }}>
                <ResponsiveContainer width="100%" height="100%">
                  <BarChart data={fleet}>
                    <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
                    <XAxis dataKey="name" stroke="#64748b" tickFormatter={(n) => n.split(' ')[0]} />
                    <YAxis stroke="#64748b" />
                    <Tooltip contentStyle={{ backgroundColor: '#0f172a', borderColor: '#334155' }} />
                    <Legend />
                    <Bar dataKey="pos_rmse_m" name="2D Position RMSE (m)" fill="#10b981" radius={[4, 4, 0, 0]} />
                    <Bar dataKey="lat_rmse_m" name="Frenet Lateral RMSE (m)" fill="#8b5cf6" radius={[4, 4, 0, 0]} />
                  </BarChart>
                </ResponsiveContainer>
              </Box>
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* Comparison Table */}
      <Card sx={{ bgcolor: '#0f172a', border: '1px solid #1e293b' }}>
        <CardContent>
          <Typography variant="subtitle1" sx={{ fontWeight: 600, color: '#f8fafc', mb: 2 }}>
            Fleet Benchmark Specification Matrix
          </Typography>
          <TableContainer>
            <Table size="small">
              <TableHead>
                <TableRow sx={{ bgcolor: '#1e293b' }}>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Vehicle Platform</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Scale / Mass</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Top Speed</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Monza Lap Time</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Pos RMSE</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Lat RMSE</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>NEES Consistency</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>MPC Solve Time</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Compliance</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {fleet.map((v: any) => (
                  <TableRow key={v.vehicle_id} sx={{ '&:hover': { bgcolor: 'rgba(255,255,255,0.02)' } }}>
                    <TableCell sx={{ color: '#f8fafc', fontWeight: 600 }}>
                      {v.name}
                      <Typography variant="caption" sx={{ display: 'block', color: '#64748b' }}>
                        ID: {v.vehicle_id}
                      </Typography>
                    </TableCell>
                    <TableCell sx={{ color: '#94a3b8' }}>
                      {v.scale} ({v.mass_kg} kg)
                    </TableCell>
                    <TableCell sx={{ color: '#f59e0b', fontWeight: 600 }}>{v.top_speed_kph} km/h</TableCell>
                    <TableCell sx={{ color: '#06b6d4', fontWeight: 600, fontFamily: 'monospace' }}>
                      {v.monza_lap_time_s.toFixed(1)} s
                    </TableCell>
                    <TableCell sx={{ color: '#10b981', fontFamily: 'monospace' }}>{v.pos_rmse_m.toFixed(3)} m</TableCell>
                    <TableCell sx={{ color: '#8b5cf6', fontFamily: 'monospace' }}>{v.lat_rmse_m.toFixed(3)} m</TableCell>
                    <TableCell sx={{ color: '#94a3b8' }}>{v.nees_consistency.toFixed(2)}</TableCell>
                    <TableCell sx={{ color: '#94a3b8' }}>{v.mpc_solve_ms.toFixed(2)} ms</TableCell>
                    <TableCell>
                      <Chip
                        size="small"
                        label={`${v.safety_pass_rate_pct}% PASS`}
                        sx={{ bgcolor: 'rgba(16, 185, 129, 0.2)', color: '#10b981', fontWeight: 700 }}
                      />
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </TableContainer>
        </CardContent>
      </Card>
    </Box>
  );
};
