import React, { useState, useEffect } from 'react';
import { Box, CssBaseline } from '@mui/material';
import { Navbar } from './components/common/Navbar';
import { Sidebar, ActiveModule } from './components/common/Sidebar';
import { LiveCockpit } from './components/monitoring/LiveCockpit';
import { VehicleCatalog } from './components/vehicles/VehicleCatalog';
import { CircuitList } from './components/circuits/CircuitList';
import { SessionManager } from './components/sessions/SessionManager';
import { LocalizationAnalytics } from './components/analytics/LocalizationAnalytics';
import { VehicleAnalytics } from './components/analytics/VehicleAnalytics';
import { SensorDiagnostics } from './components/sensors/SensorDiagnostics';
import { NavigationQuality } from './components/analytics/NavigationQuality';
import { SafetyEvents } from './components/monitoring/SafetyEvents';
import { FleetComparisons } from './components/vehicles/FleetComparisons';
import { ReplayScrubber } from './components/replay/ReplayScrubber';
import { KPIScorecardTable } from './components/kpis/KPIScorecardTable';
import { ReportGenerator } from './components/reports/ReportGenerator';

import { TelemetryFrame } from './types';
import { telemetryWs } from './api/websocket';

export const App: React.FC = () => {
  const [activeModule, setActiveModule] = useState<ActiveModule>('monitoring');
  const [currentFrame, setCurrentFrame] = useState<TelemetryFrame | null>(null);

  useEffect(() => {
    telemetryWs.connect();
    const unsubscribe = telemetryWs.subscribe((frame) => {
      setCurrentFrame(frame);
    });
    return () => {
      unsubscribe();
      telemetryWs.disconnect();
    };
  }, []);

  const renderModule = () => {
    switch (activeModule) {
      // 9 Core Engineering Dashboard Sections
      case 'vehicles':
        return <VehicleCatalog />;
      case 'loc_overview':
        return <LocalizationAnalytics />;
      case 'circuit_perf':
        return <CircuitList />;
      case 'nav_quality':
        return <NavigationQuality />;
      case 'sensor_health':
        return <SensorDiagnostics />;
      case 'veh_dynamics':
        return <VehicleAnalytics />;
      case 'safety_events':
        return <SafetyEvents />;
      case 'session_analytics':
        return <SessionManager />;
      case 'fleet_comparisons':
        return <FleetComparisons />;

      // Proving Ground Verification Engines
      case 'monitoring':
        return <LiveCockpit currentFrame={currentFrame} />;
      case 'kpis':
        return <KPIScorecardTable />;
      case 'replay':
        return <ReplayScrubber />;
      case 'reports':
        return <ReportGenerator />;
      default:
        return <LiveCockpit currentFrame={currentFrame} />;
    }
  };

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', minHeight: '100vh', bgcolor: '#0b0f19' }}>
      <CssBaseline />
      <Navbar currentFrame={currentFrame} />
      <Box sx={{ display: 'flex', flex: 1, overflow: 'hidden' }}>
        <Sidebar activeModule={activeModule} onSelectModule={setActiveModule} />
        <Box
          component="main"
          sx={{
            flex: 1,
            overflowY: 'auto',
            height: 'calc(100vh - 64px)',
            bgcolor: '#0b0f19',
          }}
        >
          {renderModule()}
        </Box>
      </Box>
    </Box>
  );
};
