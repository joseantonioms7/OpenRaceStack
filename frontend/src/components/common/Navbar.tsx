import React, { useState, useEffect } from 'react';
import {
  AppBar,
  Toolbar,
  Typography,
  Box,
  Button,
  Chip,
  IconButton,
  Tooltip,
} from '@mui/material';
import {
  ShieldAlert,
  Activity,
  Wifi,
  WifiOff,
  Car,
  MapPin,
  RefreshCw,
} from 'lucide-react';
import { TelemetryFrame } from '../../types';
import { telemetryWs } from '../../api/websocket';
import { api } from '../../api/client';

interface NavbarProps {
  currentFrame: TelemetryFrame | null;
  onRefresh?: () => void;
}

export const Navbar: React.FC<NavbarProps> = ({ currentFrame, onRefresh }) => {
  const [wsConnected, setWsConnected] = useState(false);
  const [estopLoading, setEstopLoading] = useState(false);

  useEffect(() => {
    return telemetryWs.subscribeStatus((status) => setWsConnected(status));
  }, []);

  const handleToggleEStop = async () => {
    if (!currentFrame) return;
    setEstopLoading(true);
    try {
      await api.setEStop(!currentFrame.estop_active);
    } catch (e) {
      console.error('Failed to toggle E-Stop', e);
    } finally {
      setEstopLoading(false);
    }
  };

  const isEstop = currentFrame?.estop_active;

  return (
    <AppBar position="sticky" elevation={0}>
      <Toolbar sx={{ justifyContent: 'space-between', minHeight: 64 }}>
        {/* Brand & Logo */}
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1.5 }}>
          <Box
            sx={{
              width: 34,
              height: 34,
              borderRadius: 1.5,
              background: 'linear-gradient(135deg, #06b6d4 0%, #3b82f6 100%)',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              boxShadow: '0 0 15px rgba(6, 182, 212, 0.4)',
            }}
          >
            <Activity size={20} color="#ffffff" />
          </Box>
          <Box>
            <Typography variant="h6" sx={{ fontWeight: 800, lineHeight: 1.1, letterSpacing: -0.5 }}>
              APEX<span style={{ color: '#06b6d4' }}>PATH</span> <span style={{ fontSize: '0.75rem', fontWeight: 600, color: '#9ca3af' }}>AI</span>
            </Typography>
            <Typography variant="caption" sx={{ color: '#6b7280', fontSize: '0.68rem', display: 'block' }}>
              AUTONOMOUS VEHICLE VALIDATION PLATFORM
            </Typography>
          </Box>
        </Box>

        {/* Live Status Indicators */}
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1.5 }}>
          {/* Circuit Badge */}
          <Chip
            icon={<MapPin size={14} color="#06b6d4" />}
            label={`Track: ${currentFrame?.circuit || 'Monza'}`}
            size="small"
            sx={{ bgcolor: '#1e293b', borderColor: '#334155', border: 1, fontWeight: 600 }}
          />

          {/* Vehicle Badge */}
          <Chip
            icon={<Car size={14} color="#3b82f6" />}
            label={`Vehicle: ${currentFrame?.vehicle_id || 'apex-f1tenth-01'}`}
            size="small"
            sx={{ bgcolor: '#1e293b', borderColor: '#334155', border: 1, fontWeight: 600 }}
          />

          {/* Telemetry Stream Health */}
          <Tooltip title={wsConnected ? 'Connected to 20 Hz Real-Time Telemetry Stream' : 'Operating in High-Fidelity Simulation Stream'}>
            <Chip
              icon={wsConnected ? <Wifi size={14} color="#10b981" /> : <WifiOff size={14} color="#f59e0b" />}
              label={wsConnected ? '20 Hz LIVE' : 'SIMULATED'}
              size="small"
              sx={{
                bgcolor: wsConnected ? 'rgba(16, 185, 129, 0.1)' : 'rgba(245, 158, 11, 0.1)',
                color: wsConnected ? '#10b981' : '#f59e0b',
                fontWeight: 700,
                border: 1,
                borderColor: wsConnected ? '#10b981' : '#f59e0b',
              }}
            />
          </Tooltip>

          {/* Emergency Stop Button */}
          <Button
            variant="contained"
            color={isEstop ? 'error' : 'inherit'}
            disabled={estopLoading}
            onClick={handleToggleEStop}
            startIcon={<ShieldAlert size={18} />}
            sx={{
              bgcolor: isEstop ? '#ef4444' : '#b91c1c',
              color: '#ffffff',
              fontWeight: 800,
              letterSpacing: 0.5,
              animation: isEstop ? 'pulse 1.5s infinite' : 'none',
              '&:hover': { bgcolor: '#dc2626' },
            }}
          >
            {isEstop ? 'RELEASE E-STOP' : 'E-STOP'}
          </Button>

          {onRefresh && (
            <IconButton onClick={onRefresh} size="small" sx={{ color: '#9ca3af' }}>
              <RefreshCw size={18} />
            </IconButton>
          )}
        </Box>
      </Toolbar>
    </AppBar>
  );
};
