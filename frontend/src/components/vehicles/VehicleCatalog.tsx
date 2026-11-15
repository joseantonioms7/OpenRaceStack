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
  DialogActions,
  TextField,
  IconButton,
} from '@mui/material';
import { Car, CheckCircle2, Plus, Trash2, Cpu, Wrench } from 'lucide-react';
import { Vehicle } from '../../types';
import { api } from '../../api/client';

interface VehicleCatalogProps {
  onVehicleSelected?: (vehicleId: string) => void;
}

export const VehicleCatalog: React.FC<VehicleCatalogProps> = ({ onVehicleSelected }) => {
  const [vehicles, setVehicles] = useState<Vehicle[]>([]);
  const [openModal, setOpenModal] = useState(false);
  const [newVehicle, setNewVehicle] = useState({
    id: '',
    name: '',
    vehicle_type: 'F1TENTH_SCALE',
    wheelbase_m: 0.33,
    mass_kg: 3.5,
    max_steering_rad: 0.4189,
    max_speed_mps: 20.0,
  });

  const loadVehicles = () => {
    api.getVehicles().then(setVehicles).catch(console.error);
  };

  useEffect(() => {
    loadVehicles();
  }, []);

  const handleSelect = async (id: string) => {
    await api.selectVehicle(id);
    loadVehicles();
    if (onVehicleSelected) onVehicleSelected(id);
  };

  const handleCreate = async () => {
    if (!newVehicle.id || !newVehicle.name) return;
    await api.createVehicle(newVehicle);
    setOpenModal(false);
    loadVehicles();
  };

  const handleDelete = async (id: string) => {
    await api.deleteVehicle(id);
    loadVehicles();
  };

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Box>
          <Typography variant="h5" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
            AUTONOMOUS VEHICLE FLEET MANAGEMENT
          </Typography>
          <Typography variant="body2" sx={{ color: '#9ca3af' }}>
            Configure vehicle kinematic parameters, sensor boresight extrinsics, and autonomous test platforms.
          </Typography>
        </Box>
        <Button
          variant="contained"
          startIcon={<Plus size={18} />}
          onClick={() => setOpenModal(true)}
          sx={{ bgcolor: '#06b6d4', color: '#0f172a', fontWeight: 700 }}
        >
          Add Vehicle Platform
        </Button>
      </Box>

      <Grid container spacing={3}>
        {vehicles.map((v) => (
          <Grid item xs={12} md={6} lg={4} key={v.id}>
            <Card
              sx={{
                bgcolor: '#111827',
                border: '1px solid #1f2937',
                borderRadius: 2,
                transition: 'all 0.2s',
                '&:hover': { borderColor: '#06b6d4' },
              }}
            >
              <CardContent sx={{ p: 2.5 }}>
                <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', mb: 1.5 }}>
                  <Box>
                    <Typography variant="h6" sx={{ fontWeight: 700, color: '#f9fafb' }}>
                      {v.name}
                    </Typography>
                    <Typography variant="caption" sx={{ color: '#6b7280', fontFamily: 'monospace' }}>
                      ID: {v.id}
                    </Typography>
                  </Box>
                  <Chip
                    label={v.vehicle_type}
                    size="small"
                    sx={{ bgcolor: '#1e293b', color: '#06b6d4', fontWeight: 700, fontSize: '0.7rem' }}
                  />
                </Box>

                {/* Specs Grid */}
                <Box sx={{ bgcolor: '#0b0f19', p: 1.5, borderRadius: 1.5, my: 2, border: '1px solid #1f2937' }}>
                  <Grid container spacing={1}>
                    <Grid item xs={6}>
                      <Typography variant="caption" sx={{ color: '#6b7280', display: 'block' }}>WHEELBASE</Typography>
                      <Typography variant="body2" sx={{ fontWeight: 700, fontFamily: 'monospace' }}>{v.wheelbase_m} m</Typography>
                    </Grid>
                    <Grid item xs={6}>
                      <Typography variant="caption" sx={{ color: '#6b7280', display: 'block' }}>MASS</Typography>
                      <Typography variant="body2" sx={{ fontWeight: 700, fontFamily: 'monospace' }}>{v.mass_kg} kg</Typography>
                    </Grid>
                    <Grid item xs={6}>
                      <Typography variant="caption" sx={{ color: '#6b7280', display: 'block' }}>MAX STEERING</Typography>
                      <Typography variant="body2" sx={{ fontWeight: 700, fontFamily: 'monospace' }}>
                        {(v.max_steering_rad * (180 / Math.PI)).toFixed(1)}°
                      </Typography>
                    </Grid>
                    <Grid item xs={6}>
                      <Typography variant="caption" sx={{ color: '#6b7280', display: 'block' }}>TOP SPEED</Typography>
                      <Typography variant="body2" sx={{ fontWeight: 700, fontFamily: 'monospace' }}>
                        {(v.max_speed_mps * 3.6).toFixed(0)} km/h
                      </Typography>
                    </Grid>
                  </Grid>
                </Box>

                {/* Action buttons */}
                <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mt: 2 }}>
                  <Button
                    variant="outlined"
                    size="small"
                    startIcon={<CheckCircle2 size={16} />}
                    onClick={() => handleSelect(v.id)}
                    sx={{ borderColor: '#334155', color: '#38bdf8' }}
                  >
                    Set Active
                  </Button>
                  <IconButton size="small" onClick={() => handleDelete(v.id)} sx={{ color: '#64748b', '&:hover': { color: '#ef4444' } }}>
                    <Trash2 size={16} />
                  </IconButton>
                </Box>
              </CardContent>
            </Card>
          </Grid>
        ))}
      </Grid>

      {/* Add Vehicle Modal */}
      <Dialog open={openModal} onClose={() => setOpenModal(false)} maxWidth="sm" fullWidth>
        <DialogTitle sx={{ bgcolor: '#111827', color: '#f3f4f6', fontWeight: 700 }}>
          Register New Autonomous Platform
        </DialogTitle>
        <DialogContent sx={{ bgcolor: '#111827', display: 'flex', flexDirection: 'column', gap: 2, pt: 2 }}>
          <TextField
            label="Vehicle ID (Slug)"
            fullWidth
            value={newVehicle.id}
            onChange={(e) => setNewVehicle({ ...newVehicle, id: e.target.value })}
            placeholder="apex-indy-02"
          />
          <TextField
            label="Platform Name"
            fullWidth
            value={newVehicle.name}
            onChange={(e) => setNewVehicle({ ...newVehicle, name: e.target.value })}
            placeholder="Apex Autonomous IndyCar v2"
          />
          <TextField
            label="Wheelbase (m)"
            type="number"
            fullWidth
            value={newVehicle.wheelbase_m}
            onChange={(e) => setNewVehicle({ ...newVehicle, wheelbase_m: parseFloat(e.target.value) || 0.33 })}
          />
          <TextField
            label="Vehicle Mass (kg)"
            type="number"
            fullWidth
            value={newVehicle.mass_kg}
            onChange={(e) => setNewVehicle({ ...newVehicle, mass_kg: parseFloat(e.target.value) || 3.5 })}
          />
          <TextField
            label="Max Speed (m/s)"
            type="number"
            fullWidth
            value={newVehicle.max_speed_mps}
            onChange={(e) => setNewVehicle({ ...newVehicle, max_speed_mps: parseFloat(e.target.value) || 20.0 })}
          />
        </DialogContent>
        <DialogActions sx={{ bgcolor: '#111827', p: 2 }}>
          <Button onClick={() => setOpenModal(false)} sx={{ color: '#9ca3af' }}>Cancel</Button>
          <Button onClick={handleCreate} variant="contained" sx={{ bgcolor: '#06b6d4', color: '#0f172a', fontWeight: 700 }}>
            Create Vehicle
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};
