import React, { useState, useEffect } from 'react';
import {
  Box,
  Grid,
  Card,
  CardContent,
  Typography,
  Chip,
  Paper,
  Button,
  TextField,
  MenuItem,
  Slider,
  Alert,
} from '@mui/material';
import { Cpu, Zap, Radio, AlertTriangle, CheckCircle2, ShieldAlert } from 'lucide-react';
import { api } from '../../api/client';

export const SensorDiagnostics: React.FC = () => {
  const [matrix, setMatrix] = useState<any | null>(null);
  const [injectForm, setInjectForm] = useState({
    sensor_name: 'gnss',
    fault_type: 'dropout',
    duration_sec: 5.0,
    magnitude: 3.0,
  });
  const [alertMsg, setAlertMsg] = useState<string | null>(null);

  const loadMatrix = () => {
    api.getSensorMatrix().then(setMatrix).catch(console.error);
  };

  useEffect(() => {
    loadMatrix();
    const interval = setInterval(loadMatrix, 2000);
    return () => clearInterval(interval);
  }, []);

  const handleInject = async () => {
    try {
      const res = await api.injectFault(injectForm);
      setAlertMsg(res.message);
      loadMatrix();
      setTimeout(() => setAlertMsg(null), 5000);
    } catch (e: any) {
      setAlertMsg(`Fault Injection Error: ${e.message}`);
    }
  };

  const sensors = matrix?.sensors || [];

  return (
    <Box sx={{ p: 3, display: 'flex', flexDirection: 'column', gap: 3 }}>
      <Box>
        <Typography variant="h5" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
          SENSOR HEALTH SUPERVISOR & FAULT INJECTION
        </Typography>
        <Typography variant="body2" sx={{ color: '#9ca3af' }}>
          Statistical Chi-squared NIS outlier gating, starvation timeouts, and automated failover stress simulation.
        </Typography>
      </Box>

      {alertMsg && (
        <Alert severity="warning" sx={{ bgcolor: 'rgba(245, 158, 11, 0.15)', color: '#fbbf24', border: '1px solid #f59e0b' }}>
          {alertMsg}
        </Alert>
      )}

      {/* Sensor Health Grid */}
      <Grid container spacing={2.5}>
        {sensors.map((s: any) => {
          const isOk = s.is_healthy;
          return (
            <Grid item xs={12} sm={6} md={3} key={s.sensor_name}>
              <Card
                sx={{
                  bgcolor: '#111827',
                  border: 1,
                  borderColor: isOk ? '#1f2937' : '#ef4444',
                  borderRadius: 2,
                  boxShadow: isOk ? 'none' : '0 0 15px rgba(239, 68, 68, 0.3)',
                }}
              >
                <CardContent sx={{ p: 2.5 }}>
                  <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 1.5 }}>
                    <Typography variant="subtitle1" sx={{ fontWeight: 800, color: '#f9fafb', textTransform: 'uppercase' }}>
                      {s.sensor_name}
                    </Typography>
                    <Chip
                      icon={isOk ? <CheckCircle2 size={12} /> : <AlertTriangle size={12} />}
                      label={s.status_message}
                      size="small"
                      sx={{
                        bgcolor: isOk ? 'rgba(16, 185, 129, 0.15)' : 'rgba(244, 63, 94, 0.2)',
                        color: isOk ? '#10b981' : '#f43f5e',
                        fontWeight: 700,
                        fontSize: '0.68rem',
                      }}
                    />
                  </Box>

                  <Box sx={{ bgcolor: '#0b0f19', p: 1.5, borderRadius: 1.5, border: '1px solid #1f2937', display: 'flex', flexDirection: 'column', gap: 1 }}>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                      <Typography variant="caption" sx={{ color: '#6b7280' }}>UPDATE RATE</Typography>
                      <Typography variant="caption" sx={{ color: '#06b6d4', fontWeight: 700, fontFamily: 'monospace' }}>
                        {s.message_rate_hz} Hz
                      </Typography>
                    </Box>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                      <Typography variant="caption" sx={{ color: '#6b7280' }}>TOTAL UPDATES</Typography>
                      <Typography variant="caption" sx={{ color: '#f3f4f6', fontFamily: 'monospace' }}>
                        {s.total_updates}
                      </Typography>
                    </Box>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                      <Typography variant="caption" sx={{ color: '#6b7280' }}>CHI-SQUARED REJECTIONS</Typography>
                      <Typography variant="caption" sx={{ color: s.total_rejections > 5 ? '#f59e0b' : '#10b981', fontWeight: 700, fontFamily: 'monospace' }}>
                        {s.total_rejections}
                      </Typography>
                    </Box>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                      <Typography variant="caption" sx={{ color: '#6b7280' }}>LAST NIS VALUE</Typography>
                      <Typography variant="caption" sx={{ color: '#94a3b8', fontFamily: 'monospace' }}>
                        {s.last_nis.toFixed(2)}
                      </Typography>
                    </Box>
                  </Box>
                </CardContent>
              </Card>
            </Grid>
          );
        })}
      </Grid>

      {/* Proving Ground Fault Injection Panel */}
      <Paper sx={{ p: 3, bgcolor: '#111827', borderRadius: 2, border: '1px solid #1f2937' }}>
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1.5, mb: 2 }}>
          <Zap size={22} color="#f59e0b" />
          <Typography variant="h6" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
            PROVING GROUND STRESS & FAULT INJECTION CONTROLLER
          </Typography>
        </Box>
        <Typography variant="body2" sx={{ color: '#9ca3af', mb: 3 }}>
          Dynamically inject hardware failure modes into the real-time simulation loop to verify fail-operational Kalman filter convergence.
        </Typography>

        <Grid container spacing={3}>
          <Grid item xs={12} sm={6} md={3}>
            <TextField
              select
              label="Target Sensor"
              fullWidth
              value={injectForm.sensor_name}
              onChange={(e) => setInjectForm({ ...injectForm, sensor_name: e.target.value })}
            >
              <MenuItem value="gnss">GNSS Receiver (10 Hz)</MenuItem>
              <MenuItem value="lidar">LiDAR Scan Matcher (20 Hz)</MenuItem>
              <MenuItem value="camera">Visual Odometry Camera (30 Hz)</MenuItem>
              <MenuItem value="imu">6-DOF IMU Accelerometer (100 Hz)</MenuItem>
            </TextField>
          </Grid>

          <Grid item xs={12} sm={6} md={3}>
            <TextField
              select
              label="Disturbance Scenario"
              fullWidth
              value={injectForm.fault_type}
              onChange={(e) => setInjectForm({ ...injectForm, fault_type: e.target.value })}
            >
              <MenuItem value="dropout">Complete Signal Dropout / Blackout</MenuItem>
              <MenuItem value="noise_bias">Multipath Noise Clutter</MenuItem>
              <MenuItem value="drift">Sensor Gyro Drift Walk</MenuItem>
            </TextField>
          </Grid>

          <Grid item xs={12} sm={6} md={3}>
            <Typography variant="caption" sx={{ color: '#9ca3af', display: 'block', mb: 1 }}>
              DURATION: {injectForm.duration_sec} SECONDS
            </Typography>
            <Slider
              value={injectForm.duration_sec}
              min={1}
              max={30}
              step={1}
              onChange={(_, v) => setInjectForm({ ...injectForm, duration_sec: v as number })}
              sx={{ color: '#f59e0b' }}
            />
          </Grid>

          <Grid item xs={12} sm={6} md={3} sx={{ display: 'flex', alignItems: 'center' }}>
            <Button
              fullWidth
              variant="contained"
              startIcon={<Zap size={18} />}
              onClick={handleInject}
              sx={{ bgcolor: '#f59e0b', color: '#0f172a', fontWeight: 800, height: 52 }}
            >
              Trigger Failure Mode
            </Button>
          </Grid>
        </Grid>
      </Paper>
    </Box>
  );
};
