import React from 'react';
import {
  Box,
  List,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Typography,
  Divider,
} from '@mui/material';
import {
  Gauge,
  Car,
  Map,
  ClipboardList,
  Crosshair,
  TrendingUp,
  Cpu,
  History,
  CheckCircle2,
  FileText,
  Compass,
  ShieldAlert,
  Users,
} from 'lucide-react';

export type ActiveModule =
  | 'monitoring'
  | 'vehicles'
  | 'loc_overview'
  | 'circuit_perf'
  | 'nav_quality'
  | 'sensor_health'
  | 'veh_dynamics'
  | 'safety_events'
  | 'session_analytics'
  | 'fleet_comparisons'
  | 'replay'
  | 'kpis'
  | 'reports';

interface SidebarProps {
  activeModule: ActiveModule;
  onSelectModule: (module: ActiveModule) => void;
}

export const Sidebar: React.FC<SidebarProps> = ({ activeModule, onSelectModule }) => {
  const dashboardSections = [
    { id: 'vehicles' as ActiveModule, label: 'Vehicle Overview', icon: <Car size={18} /> },
    { id: 'loc_overview' as ActiveModule, label: 'Localization Overview', icon: <Crosshair size={18} /> },
    { id: 'circuit_perf' as ActiveModule, label: 'Circuit Performance', icon: <Map size={18} /> },
    { id: 'nav_quality' as ActiveModule, label: 'Navigation Quality', icon: <Compass size={18} /> },
    { id: 'sensor_health' as ActiveModule, label: 'Sensor Health', icon: <Cpu size={18} /> },
    { id: 'veh_dynamics' as ActiveModule, label: 'Vehicle Dynamics', icon: <TrendingUp size={18} /> },
    { id: 'safety_events' as ActiveModule, label: 'Safety Events', icon: <ShieldAlert size={18} /> },
    { id: 'session_analytics' as ActiveModule, label: 'Session Analytics', icon: <ClipboardList size={18} /> },
    { id: 'fleet_comparisons' as ActiveModule, label: 'Fleet Comparisons', icon: <Users size={18} /> },
  ];

  const toolsAndEngines = [
    { id: 'monitoring' as ActiveModule, label: 'Live Cockpit Stream', icon: <Gauge size={18} /> },
    { id: 'kpis' as ActiveModule, label: 'ISO 21448 Scorecards', icon: <CheckCircle2 size={18} /> },
    { id: 'replay' as ActiveModule, label: 'Incident Replay Engine', icon: <History size={18} /> },
    { id: 'reports' as ActiveModule, label: 'Validation Dossiers', icon: <FileText size={18} /> },
  ];

  const renderItem = (item: { id: ActiveModule; label: string; icon: React.ReactNode }) => {
    const isActive = activeModule === item.id;
    return (
      <ListItemButton
        key={item.id}
        onClick={() => onSelectModule(item.id)}
        sx={{
          borderRadius: 1.5,
          mb: 0.5,
          bgcolor: isActive ? 'rgba(6, 182, 212, 0.12)' : 'transparent',
          color: isActive ? '#06b6d4' : '#94a3b8',
          border: 1,
          borderColor: isActive ? 'rgba(6, 182, 212, 0.3)' : 'transparent',
          '&:hover': {
            bgcolor: isActive ? 'rgba(6, 182, 212, 0.16)' : 'rgba(255, 255, 255, 0.04)',
            color: isActive ? '#06b6d4' : '#f8fafc',
          },
        }}
      >
        <ListItemIcon sx={{ minWidth: 32, color: 'inherit' }}>{item.icon}</ListItemIcon>
        <ListItemText
          primary={item.label}
          primaryTypographyProps={{
            fontSize: '0.84rem',
            fontWeight: isActive ? 700 : 500,
          }}
        />
      </ListItemButton>
    );
  };

  return (
    <Box
      sx={{
        width: 260,
        flexShrink: 0,
        bgcolor: '#0f172a',
        borderRight: '1px solid #1f2937',
        height: 'calc(100vh - 64px)',
        display: 'flex',
        flexDirection: 'column',
      }}
    >
      <Box sx={{ p: 2, pb: 1 }}>
        <Typography variant="overline" sx={{ color: '#64748b', fontWeight: 700, letterSpacing: 1.2 }}>
          ENGINEERING DASHBOARD
        </Typography>
      </Box>

      <List sx={{ px: 1, py: 0.5, overflowY: 'auto' }}>
        {dashboardSections.map(renderItem)}
      </List>

      <Divider sx={{ my: 1, borderColor: '#1e293b' }} />

      <Box sx={{ px: 2, py: 0.5 }}>
        <Typography variant="overline" sx={{ color: '#64748b', fontWeight: 700, letterSpacing: 1.2 }}>
          PLATFORM ENGINES
        </Typography>
      </Box>

      <List sx={{ px: 1, py: 0.5, overflowY: 'auto' }}>
        {toolsAndEngines.map(renderItem)}
      </List>

      <Box sx={{ mt: 'auto', p: 2, borderTop: '1px solid #1f2937', bgcolor: '#0b1329' }}>
        <Typography variant="caption" sx={{ color: '#64748b', display: 'block' }}>
          ApexPathAI Platform v2.0.0-rc1
        </Typography>
        <Typography variant="caption" sx={{ color: '#10b981', display: 'flex', alignItems: 'center', gap: 0.5, mt: 0.5 }}>
          <Box sx={{ width: 6, height: 6, borderRadius: '50%', bgcolor: '#10b981' }} />
          Online • 20 Hz CTRV+ESKF
        </Typography>
      </Box>
    </Box>
  );
};
