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
  Alert,
  LinearProgress,
} from '@mui/material';
import { ShieldAlert, ShieldCheck, Power, RefreshCw, AlertTriangle, CheckCircle } from 'lucide-react';
import { MetricCard } from '../common/MetricCard';
import { api } from '../../api/client';

export const SafetyEvents: React.FC = () => {
  const [events, setEvents] = useState<any[]>([]);
  const [estopActive, setEstopActive] = useState<boolean>(false);
  const [loading, setLoading] = useState<boolean>(false);

  const fetchSafetyStatus = async () => {
    setLoading(true);
    try {
      const res = await api.getSafetyEvents();
      setEvents(res.events || []);
      setEstopActive(res.estop_active);
    } catch (err) {
      console.error('Failed to load safety events', err);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    fetchSafetyStatus();
    const interval = setInterval(fetchSafetyStatus, 3000);
    return () => clearInterval(interval);
  }, []);

  const handleToggleEstop = async () => {
    try {
      if (estopActive) {
        await api.resetEStop();
      } else {
        await api.setEStop(true);
      }
      fetchSafetyStatus();
    } catch (err) {
      console.error('Failed to toggle E-Stop', err);
    }
  };

  return (
    <Box sx={{ p: 3 }}>
      {/* Header */}
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Box>
          <Typography variant="h5" sx={{ fontWeight: 700, color: '#f8fafc', display: 'flex', alignItems: 'center', gap: 1 }}>
            <ShieldAlert size={24} color={estopActive ? '#ef4444' : '#10b981'} />
            Safety Events & Disengagement Log
          </Typography>
          <Typography variant="body2" sx={{ color: '#94a3b8', mt: 0.5 }}>
            Real-time supervisor logging emergency stops, geofence track boundary incursions, and Kalman divergence alerts.
          </Typography>
        </Box>
        <Box sx={{ display: 'flex', gap: 2 }}>
          <Button
            variant="outlined"
            size="small"
            startIcon={<RefreshCw size={14} />}
            onClick={fetchSafetyStatus}
            sx={{ borderColor: '#334155', color: '#94a3b8' }}
          >
            Refresh
          </Button>
          <Button
            variant="contained"
            color={estopActive ? 'success' : 'error'}
            startIcon={<Power size={16} />}
            onClick={handleToggleEstop}
            sx={{ fontWeight: 700, px: 3 }}
          >
            {estopActive ? 'RESET E-STOP (RESUME)' : 'TRIGGER E-STOP'}
          </Button>
        </Box>
      </Box>

      {/* Active E-STOP Banner */}
      {estopActive && (
        <Alert
          severity="error"
          variant="filled"
          sx={{ mb: 3, fontWeight: 700, fontSize: '1rem', bgcolor: '#b91c1c' }}
        >
          CRITICAL SAFETY INTERLOCK ENGAGED: Software Emergency Stop (E-STOP) is currently active. All steering and propulsion commands are clamped to 0.
        </Alert>
      )}

      {loading && <LinearProgress sx={{ mb: 2, bgcolor: '#1e293b', '& .MuiLinearProgress-bar': { bgcolor: '#06b6d4' } }} />}

      {/* Metric Cards */}
      <Grid container spacing={2} sx={{ mb: 3 }}>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="Actuator State"
            value={estopActive ? 'E-STOP CLAMPED' : 'NOMINAL (READY)'}
            subtitle="Autonomous drive interlock"
            color={estopActive ? '#ef4444' : '#10b981'}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="Total Events Logged"
            value={events.length.toString()}
            subtitle="Autonomous proving ground run"
            color="#06b6d4"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="Disengagements"
            value={estopActive ? '1 Active' : '0'}
            subtitle="Rule 7 Threshold: 0 required"
            color={estopActive ? '#ef4444' : '#10b981'}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <MetricCard
            title="Geofence Margin Gate"
            value=">= 0.50 m"
            subtitle="Corridor KD-Tree constraint"
            color="#8b5cf6"
          />
        </Grid>
      </Grid>

      {/* Safety Events Table */}
      <Card sx={{ bgcolor: '#0f172a', border: '1px solid #1e293b', mb: 3 }}>
        <CardContent>
          <Typography variant="subtitle1" sx={{ fontWeight: 600, color: '#f8fafc', mb: 2 }}>
            Supervisor Audit Log
          </Typography>
          <TableContainer>
            <Table size="small">
              <TableHead>
                <TableRow sx={{ bgcolor: '#1e293b' }}>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>ID</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Severity</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Event Type</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Station (m)</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Description</TableCell>
                  <TableCell sx={{ color: '#94a3b8', fontWeight: 600 }}>Autonomous Mitigation</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {events.length === 0 ? (
                  <TableRow>
                    <TableCell colSpan={6} sx={{ color: '#64748b', textAlign: 'center', py: 3 }}>
                      No safety events recorded. Proving ground telemetry running nominally.
                    </TableCell>
                  </TableRow>
                ) : (
                  events.map((evt) => (
                    <TableRow key={evt.id} sx={{ '&:hover': { bgcolor: 'rgba(255,255,255,0.02)' } }}>
                      <TableCell sx={{ color: '#64748b', fontFamily: 'monospace' }}>{evt.id}</TableCell>
                      <TableCell>
                        <Chip
                          size="small"
                          label={evt.severity}
                          sx={{
                            fontWeight: 700,
                            bgcolor:
                              evt.severity === 'CRITICAL'
                                ? 'rgba(239, 68, 68, 0.2)'
                                : evt.severity === 'WARNING'
                                ? 'rgba(245, 158, 11, 0.2)'
                                : 'rgba(16, 185, 129, 0.2)',
                            color:
                              evt.severity === 'CRITICAL'
                                ? '#ef4444'
                                : evt.severity === 'WARNING'
                                ? '#f59e0b'
                                : '#10b981',
                          }}
                        />
                      </TableCell>
                      <TableCell sx={{ color: '#f8fafc', fontWeight: 600, fontSize: '0.85rem' }}>{evt.event_type}</TableCell>
                      <TableCell sx={{ color: '#06b6d4', fontFamily: 'monospace' }}>{evt.station_m} m</TableCell>
                      <TableCell sx={{ color: '#94a3b8' }}>{evt.description}</TableCell>
                      <TableCell sx={{ color: '#cbd5e1', fontSize: '0.85rem' }}>{evt.mitigation}</TableCell>
                    </TableRow>
                  ))
                )}
              </TableBody>
            </Table>
          </TableContainer>
        </CardContent>
      </Card>

      {/* Safety Policy Card */}
      <Card sx={{ bgcolor: '#0f172a', border: '1px solid #1e293b' }}>
        <CardContent>
          <Typography variant="subtitle1" sx={{ fontWeight: 600, color: '#f8fafc', mb: 1, display: 'flex', alignItems: 'center', gap: 1 }}>
            <CheckCircle size={18} color="#10b981" />
            ISO 21448 (SOTIF) & ASIL-D Physical Proving Ground Compliance Notice
          </Typography>
          <Typography variant="body2" sx={{ color: '#94a3b8', lineHeight: 1.6 }}>
            Software emergency stop commands operate via WebSocket and REST APIs to disengage autonomous actuation. In physical test track testing, vehicle hardware must incorporate an independent, hard-wired remote wireless E-STOP receiver cutting 48V/12V traction battery power directly to the motor controller independently of the computing platform.
          </Typography>
        </CardContent>
      </Card>
    </Box>
  );
};
