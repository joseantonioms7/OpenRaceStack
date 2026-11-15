import React, { useState, useEffect } from 'react';
import {
  Box,
  Grid,
  Card,
  CardContent,
  Typography,
  Button,
  Chip,
  Dialog,
  DialogTitle,
  DialogContent,
  CircularProgress,
} from '@mui/material';
import { Map, Flag, Check, Eye } from 'lucide-react';
import { CircuitSummary, CircuitDetail } from '../../types';
import { api } from '../../api/client';

interface CircuitListProps {
  onCircuitSelected?: (trackName: string) => void;
}

export const CircuitList: React.FC<CircuitListProps> = ({ onCircuitSelected }) => {
  const [circuits, setCircuits] = useState<CircuitSummary[]>([]);
  const [selectedCircuit, setSelectedCircuit] = useState<CircuitDetail | null>(null);
  const [loading, setLoading] = useState(false);
  const [activeTrack, setActiveTrack] = useState<string>('Monza');

  const loadCircuits = () => {
    setLoading(true);
    api.getCircuits()
      .then((data) => {
        setCircuits(data);
        const active = data.find((c) => c.is_active);
        if (active) setActiveTrack(active.name);
      })
      .finally(() => setLoading(false));
  };

  useEffect(() => {
    loadCircuits();
  }, []);

  const handleSelect = async (name: string) => {
    await api.selectCircuit(name);
    setActiveTrack(name);
    loadCircuits();
    if (onCircuitSelected) onCircuitSelected(name);
  };

  const handleInspect = async (name: string) => {
    const detail = await api.getCircuitDetail(name);
    setSelectedCircuit(detail);
  };

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ mb: 3 }}>
        <Typography variant="h5" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
          PROVING GROUND CIRCUIT CATALOG (23 TRACKS)
        </Typography>
        <Typography variant="body2" sx={{ color: '#9ca3af' }}>
          Centimeter-grade centerline & raceline profiles across international Grand Prix test facilities.
        </Typography>
      </Box>

      {loading && <CircularProgress size={32} sx={{ color: '#06b6d4', my: 3 }} />}

      <Grid container spacing={2.5}>
        {circuits.map((c) => {
          const isActive = c.name.toLowerCase() === activeTrack.toLowerCase();
          return (
            <Grid item xs={12} sm={6} md={4} lg={3} key={c.id}>
              <Card
                sx={{
                  bgcolor: '#111827',
                  border: 1,
                  borderColor: isActive ? '#06b6d4' : '#1f2937',
                  borderRadius: 2,
                  boxShadow: isActive ? '0 0 15px rgba(6, 182, 212, 0.2)' : 'none',
                  transition: 'all 0.2s',
                  '&:hover': { borderColor: '#38bdf8' },
                }}
              >
                <CardContent sx={{ p: 2 }}>
                  <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 1 }}>
                    <Typography variant="subtitle1" sx={{ fontWeight: 700, color: '#f9fafb' }}>
                      {c.name}
                    </Typography>
                    {isActive ? (
                      <Chip label="ACTIVE" size="small" sx={{ bgcolor: 'rgba(16, 185, 129, 0.2)', color: '#10b981', fontWeight: 800, fontSize: '0.65rem' }} />
                    ) : (
                      <Chip label={c.country} size="small" sx={{ bgcolor: '#1e293b', color: '#94a3b8', fontSize: '0.65rem' }} />
                    )}
                  </Box>

                  <Box sx={{ bgcolor: '#0b0f19', p: 1.2, borderRadius: 1.5, my: 1.5, border: '1px solid #1f2937' }}>
                    <Typography variant="caption" sx={{ color: '#6b7280', display: 'block' }}>
                      LENGTH: <strong style={{ color: '#e5e7eb', fontFamily: 'monospace' }}>{c.length_m} m</strong>
                    </Typography>
                    <Typography variant="caption" sx={{ color: '#6b7280', display: 'block' }}>
                      WAYPOINTS: <strong style={{ color: '#e5e7eb', fontFamily: 'monospace' }}>{c.waypoints_count}</strong>
                    </Typography>
                  </Box>

                  <Box sx={{ display: 'flex', gap: 1, mt: 1.5 }}>
                    <Button
                      fullWidth
                      variant={isActive ? 'contained' : 'outlined'}
                      size="small"
                      startIcon={isActive ? <Check size={14} /> : undefined}
                      onClick={() => handleSelect(c.name)}
                      sx={{
                        bgcolor: isActive ? '#06b6d4' : 'transparent',
                        color: isActive ? '#0f172a' : '#38bdf8',
                        borderColor: '#334155',
                        fontWeight: 700,
                      }}
                    >
                      {isActive ? 'Loaded' : 'Select'}
                    </Button>
                    <Button
                      variant="text"
                      size="small"
                      onClick={() => handleInspect(c.name)}
                      sx={{ color: '#9ca3af', minWidth: 36 }}
                    >
                      <Eye size={16} />
                    </Button>
                  </Box>
                </CardContent>
              </Card>
            </Grid>
          );
        })}
      </Grid>

      {/* Circuit Geometry Inspection Modal */}
      {selectedCircuit && (
        <Dialog open={!!selectedCircuit} onClose={() => setSelectedCircuit(null)} maxWidth="md" fullWidth>
          <DialogTitle sx={{ bgcolor: '#111827', color: '#f3f4f6', fontWeight: 700 }}>
            Circuit Geometry Inspection: {selectedCircuit.name}
          </DialogTitle>
          <DialogContent sx={{ bgcolor: '#111827', color: '#9ca3af' }}>
            <Grid container spacing={2} sx={{ my: 1 }}>
              <Grid item xs={4}>
                <Typography variant="caption">COUNTRY</Typography>
                <Typography variant="h6" sx={{ color: '#f9fafb' }}>{selectedCircuit.country}</Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="caption">TRACK LENGTH</Typography>
                <Typography variant="h6" sx={{ color: '#06b6d4', fontFamily: 'monospace' }}>{selectedCircuit.length_m} m</Typography>
              </Grid>
              <Grid item xs={4}>
                <Typography variant="caption">METRIC RESOLUTION</Typography>
                <Typography variant="h6" sx={{ color: '#10b981', fontFamily: 'monospace' }}>{selectedCircuit.resolution_m} m/pixel</Typography>
              </Grid>
            </Grid>
            <Box sx={{ bgcolor: '#0b0f19', p: 2, borderRadius: 2, border: '1px solid #1f2937', mt: 2 }}>
              <Typography variant="caption" sx={{ color: '#6b7280', display: 'block', mb: 1 }}>
                CENTERLINE & RACELINE WAYPOINTS PREVIEW ({selectedCircuit.centerline_preview.length} points)
              </Typography>
              <Typography variant="body2" sx={{ fontFamily: 'monospace', fontSize: '0.8rem', color: '#94a3b8' }}>
                First coordinate: [{selectedCircuit.centerline_preview[0]?.[0]}, {selectedCircuit.centerline_preview[0]?.[1]}]
                <br />
                Has High-Speed Raceline: {selectedCircuit.has_raceline ? 'YES' : 'NO'} | Has Occupancy Raster: {selectedCircuit.has_raster_map ? 'YES' : 'NO'}
              </Typography>
            </Box>
          </DialogContent>
        </Dialog>
      )}
    </Box>
  );
};
