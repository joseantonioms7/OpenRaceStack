import React, { useState, useEffect } from 'react';
import {
  Box,
  Grid,
  Paper,
  Typography,
  Button,
  Chip,
  Table,
  TableHead,
  TableRow,
  TableCell,
  TableBody,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  TextField,
  MenuItem,
} from '@mui/material';
import {
  Play,
  Pause,
  Square,
  AlertOctagon,
  ShieldCheck,
  Plus,
  Clock,
  Flag,
} from 'lucide-react';
import { TestSession } from '../../types';
import { api } from '../../api/client';

export const SessionManager: React.FC = () => {
  const [activeSession, setActiveSession] = useState<TestSession | null>(null);
  const [openModal, setOpenModal] = useState(false);
  const [formData, setFormData] = useState({
    name: 'Monza High-Speed Proving Ground Campaign',
    track_name: 'Monza',
    vehicle_id: 'apex-f1tenth-01',
    profile: 'NOMINAL_RACELINE',
    target_laps: 3,
  });

  const loadActive = () => {
    api.getActiveSession().then(setActiveSession).catch(console.error);
  };

  useEffect(() => {
    loadActive();
    const interval = setInterval(loadActive, 2000);
    return () => clearInterval(interval);
  }, []);

  const handleTransition = async (action: string) => {
    if (!activeSession) return;
    await api.transitionSession(activeSession.id, action);
    loadActive();
  };

  const handleCreate = async () => {
    await api.createSession(formData);
    setOpenModal(false);
    loadActive();
  };

  const getStateColor = (st: string) => {
    switch (st) {
      case 'RUNNING': return '#10b981';
      case 'ARMED': return '#06b6d4';
      case 'PAUSED': return '#f59e0b';
      case 'ABORTED': return '#f43f5e';
      default: return '#9ca3af';
    }
  };

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Box>
          <Typography variant="h5" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
            TEST SESSION & CAMPAIGN ORCHESTRATOR
          </Typography>
          <Typography variant="body2" sx={{ color: '#9ca3af' }}>
            ISO 26262 proving ground campaign lifecycle, sector lap timing, and safety scorecards.
          </Typography>
        </Box>
        <Button
          variant="contained"
          startIcon={<Plus size={18} />}
          onClick={() => setOpenModal(true)}
          sx={{ bgcolor: '#06b6d4', color: '#0f172a', fontWeight: 700 }}
        >
          Initialize New Campaign
        </Button>
      </Box>

      {/* Active Session Card */}
      {activeSession ? (
        <Paper sx={{ p: 3, bgcolor: '#111827', borderRadius: 2, mb: 3, border: '1px solid #1f2937' }}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', mb: 2 }}>
            <Box>
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 1.5, mb: 0.5 }}>
                <Typography variant="h6" sx={{ fontWeight: 800, color: '#f9fafb' }}>
                  {activeSession.name}
                </Typography>
                <Chip
                  label={activeSession.state}
                  size="small"
                  sx={{
                    bgcolor: `${getStateColor(activeSession.state)}22`,
                    color: getStateColor(activeSession.state),
                    fontWeight: 800,
                    border: 1,
                    borderColor: getStateColor(activeSession.state),
                  }}
                />
              </Box>
              <Typography variant="caption" sx={{ color: '#6b7280', fontFamily: 'monospace' }}>
                SESSION ID: {activeSession.id} | PROFILE: {activeSession.profile}
              </Typography>
            </Box>

            {/* Lifecycle Action Buttons */}
            <Box sx={{ display: 'flex', gap: 1 }}>
              {activeSession.state === 'IDLE' && (
                <Button variant="outlined" onClick={() => handleTransition('arm')} sx={{ borderColor: '#06b6d4', color: '#06b6d4' }}>
                  Arm Session
                </Button>
              )}
              {activeSession.state === 'ARMED' && (
                <Button variant="contained" startIcon={<Play size={16} />} onClick={() => handleTransition('start')} sx={{ bgcolor: '#10b981', color: '#0f172a', fontWeight: 700 }}>
                  Start Campaign
                </Button>
              )}
              {activeSession.state === 'RUNNING' && (
                <>
                  <Button variant="outlined" startIcon={<Pause size={16} />} onClick={() => handleTransition('pause')} sx={{ borderColor: '#f59e0b', color: '#f59e0b' }}>
                    Pause
                  </Button>
                  <Button variant="outlined" startIcon={<Square size={16} />} onClick={() => handleTransition('stop')} sx={{ borderColor: '#3b82f6', color: '#3b82f6' }}>
                    Finalize
                  </Button>
                  <Button variant="outlined" startIcon={<AlertOctagon size={16} />} onClick={() => handleTransition('abort')} sx={{ borderColor: '#f43f5e', color: '#f43f5e' }}>
                    Abort
                  </Button>
                </>
              )}
              {activeSession.state === 'PAUSED' && (
                <Button variant="contained" startIcon={<Play size={16} />} onClick={() => handleTransition('start')} sx={{ bgcolor: '#10b981', color: '#0f172a', fontWeight: 700 }}>
                  Resume
                </Button>
              )}
            </Box>
          </Box>

          <Grid container spacing={2} sx={{ my: 1 }}>
            <Grid item xs={3}>
              <Box sx={{ bgcolor: '#0b0f19', p: 1.5, borderRadius: 1.5, border: '1px solid #1f2937' }}>
                <Typography variant="caption" sx={{ color: '#6b7280' }}>TARGET LAPS</Typography>
                <Typography variant="h5" sx={{ fontWeight: 800, fontFamily: 'monospace' }}>
                  {activeSession.completed_laps} / {activeSession.target_laps}
                </Typography>
              </Box>
            </Grid>
            <Grid item xs={3}>
              <Box sx={{ bgcolor: '#0b0f19', p: 1.5, borderRadius: 1.5, border: '1px solid #1f2937' }}>
                <Typography variant="caption" sx={{ color: '#6b7280' }}>BEST LAP TIME</Typography>
                <Typography variant="h5" sx={{ fontWeight: 800, fontFamily: 'monospace', color: '#06b6d4' }}>
                  {activeSession.best_lap_time_s ? `${activeSession.best_lap_time_s.toFixed(2)}s` : '--'}
                </Typography>
              </Box>
            </Grid>
            <Grid item xs={3}>
              <Box sx={{ bgcolor: '#0b0f19', p: 1.5, borderRadius: 1.5, border: '1px solid #1f2937' }}>
                <Typography variant="caption" sx={{ color: '#6b7280' }}>POSITION RMSE</Typography>
                <Typography variant="h5" sx={{ fontWeight: 800, fontFamily: 'monospace', color: '#10b981' }}>
                  {activeSession.position_rmse_m ? `${activeSession.position_rmse_m.toFixed(3)}m` : '--'}
                </Typography>
              </Box>
            </Grid>
            <Grid item xs={3}>
              <Box sx={{ bgcolor: '#0b0f19', p: 1.5, borderRadius: 1.5, border: '1px solid #1f2937' }}>
                <Typography variant="caption" sx={{ color: '#6b7280' }}>SAFETY VERDICT</Typography>
                <Typography variant="h5" sx={{ fontWeight: 800, color: activeSession.verdict === 'PASS' ? '#10b981' : '#f59e0b' }}>
                  {activeSession.verdict}
                </Typography>
              </Box>
            </Grid>
          </Grid>

          {/* Sector & Lap Timing Table */}
          <Typography variant="subtitle2" sx={{ fontWeight: 700, mt: 3, mb: 1.5, color: '#f3f4f6' }}>
            RECORDED SECTORS & LAP LEADERBOARD
          </Typography>
          <Table size="small" sx={{ bgcolor: '#0b0f19', borderRadius: 1.5, border: '1px solid #1f2937' }}>
            <TableHead>
              <TableRow>
                <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>LAP #</TableCell>
                <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>TOTAL TIME</TableCell>
                <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>SECTOR 1</TableCell>
                <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>SECTOR 2</TableCell>
                <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>SECTOR 3</TableCell>
                <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>MAX SPEED</TableCell>
                <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>LAT RMSE</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {activeSession.laps.length === 0 ? (
                <TableRow>
                  <TableCell colSpan={7} sx={{ textAlign: 'center', color: '#475569', py: 3 }}>
                    Vehicle currently running Lap 1... Data streaming into telemetry buffer.
                  </TableCell>
                </TableRow>
              ) : (
                activeSession.laps.map((lap) => (
                  <TableRow key={lap.lap_number}>
                    <TableCell sx={{ color: '#38bdf8', fontWeight: 700, fontFamily: 'monospace' }}>Lap {lap.lap_number}</TableCell>
                    <TableCell sx={{ color: '#f9fafb', fontWeight: 700, fontFamily: 'monospace' }}>{lap.lap_time_s.toFixed(2)}s</TableCell>
                    <TableCell sx={{ color: '#94a3b8', fontFamily: 'monospace' }}>{lap.sector_1_s ? `${lap.sector_1_s.toFixed(2)}s` : '--'}</TableCell>
                    <TableCell sx={{ color: '#94a3b8', fontFamily: 'monospace' }}>{lap.sector_2_s ? `${lap.sector_2_s.toFixed(2)}s` : '--'}</TableCell>
                    <TableCell sx={{ color: '#94a3b8', fontFamily: 'monospace' }}>{lap.sector_3_s ? `${lap.sector_3_s.toFixed(2)}s` : '--'}</TableCell>
                    <TableCell sx={{ color: '#10b981', fontFamily: 'monospace' }}>{lap.max_velocity_mps.toFixed(1)} km/h</TableCell>
                    <TableCell sx={{ color: '#f59e0b', fontFamily: 'monospace' }}>{lap.lateral_rmse_m.toFixed(3)} m</TableCell>
                  </TableRow>
                ))
              )}
            </TableBody>
          </Table>
        </Paper>
      ) : (
        <Paper sx={{ p: 4, textAlign: 'center', bgcolor: '#111827', borderRadius: 2 }}>
          <Typography variant="body1" sx={{ color: '#94a3b8', mb: 2 }}>
            No active test campaign currently armed.
          </Typography>
          <Button variant="contained" onClick={() => setOpenModal(true)} sx={{ bgcolor: '#06b6d4', color: '#0f172a', fontWeight: 700 }}>
            Create First Proving Ground Campaign
          </Button>
        </Paper>
      )}

      {/* New Campaign Modal */}
      <Dialog open={openModal} onClose={() => setOpenModal(false)} maxWidth="sm" fullWidth>
        <DialogTitle sx={{ bgcolor: '#111827', color: '#f3f4f6', fontWeight: 700 }}>
          Initialize Proving Ground Campaign
        </DialogTitle>
        <DialogContent sx={{ bgcolor: '#111827', display: 'flex', flexDirection: 'column', gap: 2, pt: 2 }}>
          <TextField
            label="Campaign Name"
            fullWidth
            value={formData.name}
            onChange={(e) => setFormData({ ...formData, name: e.target.value })}
          />
          <TextField
            select
            label="Proving Ground Circuit"
            fullWidth
            value={formData.track_name}
            onChange={(e) => setFormData({ ...formData, track_name: e.target.value })}
          >
            {['Monza', 'Silverstone', 'Spa', 'Catalunya', 'Hockenheim', 'Spielberg', 'Zandvoort'].map((t) => (
              <MenuItem key={t} value={t}>{t}</MenuItem>
            ))}
          </TextField>
          <TextField
            select
            label="Test Profile / Safety Gate"
            fullWidth
            value={formData.profile}
            onChange={(e) => setFormData({ ...formData, profile: e.target.value })}
          >
            <MenuItem value="NOMINAL_RACELINE">Nominal High-Speed Raceline</MenuItem>
            <MenuItem value="GNSS_DENIED_OUTAGE">GNSS-Denied Outage Stress</MenuItem>
            <MenuItem value="URBAN_CANYON_MULTIPATH">Urban Canyon Multipath Clutter</MenuItem>
            <MenuItem value="SENSOR_FAILOVER_STRESS">Sensor Failover Stress</MenuItem>
            <MenuItem value="HIGH_SPEED_LIMIT_HANDLING">High-Speed Limit Handling</MenuItem>
          </TextField>
          <TextField
            label="Target Lap Count"
            type="number"
            fullWidth
            value={formData.target_laps}
            onChange={(e) => setFormData({ ...formData, target_laps: parseInt(e.target.value) || 3 })}
          />
        </DialogContent>
        <DialogActions sx={{ bgcolor: '#111827', p: 2 }}>
          <Button onClick={() => setOpenModal(false)} sx={{ color: '#9ca3af' }}>Cancel</Button>
          <Button onClick={handleCreate} variant="contained" sx={{ bgcolor: '#06b6d4', color: '#0f172a', fontWeight: 700 }}>
            Create Campaign
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};
