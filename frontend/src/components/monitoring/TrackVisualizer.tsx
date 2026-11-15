import React, { useRef, useEffect } from 'react';
import { Box, Typography } from '@mui/material';
import { TelemetryFrame, CircuitDetail } from '../../types';

interface TrackVisualizerProps {
  currentFrame: TelemetryFrame | null;
  circuitDetail: CircuitDetail | null;
  historyTrail: [number, number][];
}

export const TrackVisualizer: React.FC<TrackVisualizerProps> = ({
  currentFrame,
  circuitDetail,
  historyTrail,
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;

    // Clear background
    ctx.fillStyle = '#0b0f19';
    ctx.fillRect(0, 0, width, height);

    // Grid lines
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    const gridSize = 40;
    for (let x = 0; x < width; x += gridSize) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
    }
    for (let y = 0; y < height; y += gridSize) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }

    const clPoints = circuitDetail?.centerline_preview || [];
    const rlPoints = circuitDetail?.raceline_preview || [];

    // Calculate bounding box
    let minX = -100, maxX = 100, minY = -100, maxY = 100;
    if (clPoints.length > 0) {
      minX = Math.min(...clPoints.map((p) => p[0]));
      maxX = Math.max(...clPoints.map((p) => p[0]));
      minY = Math.min(...clPoints.map((p) => p[1]));
      maxY = Math.max(...clPoints.map((p) => p[1]));
    } else if (historyTrail.length > 0) {
      minX = Math.min(...historyTrail.map((p) => p[0]));
      maxX = Math.max(...historyTrail.map((p) => p[0]));
      minY = Math.min(...historyTrail.map((p) => p[1]));
      maxY = Math.max(...historyTrail.map((p) => p[1]));
    }

    const pad = 40;
    const spanX = Math.max(1, maxX - minX);
    const spanY = Math.max(1, maxY - minY);
    const scale = Math.min((width - pad * 2) / spanX, (height - pad * 2) / spanY);

    const toCanvas = (wx: number, wy: number): [number, number] => {
      const cx = pad + (wx - minX) * scale;
      const cy = height - (pad + (wy - minY) * scale); // Invert Y
      return [cx, cy];
    };

    // 1. Draw Centerline
    if (clPoints.length > 1) {
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 6;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      const [startCx, startCy] = toCanvas(clPoints[0][0], clPoints[0][1]);
      ctx.moveTo(startCx, startCy);
      for (let i = 1; i < clPoints.length; i++) {
        const [cpx, cpy] = toCanvas(clPoints[i][0], clPoints[i][1]);
        ctx.lineTo(cpx, cpy);
      }
      ctx.closePath();
      ctx.stroke();
    }

    // 2. Draw Raceline
    if (rlPoints.length > 1) {
      ctx.strokeStyle = '#06b6d4';
      ctx.lineWidth = 2.5;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      const [rStartCx, rStartCy] = toCanvas(rlPoints[0][0], rlPoints[0][1]);
      ctx.moveTo(rStartCx, rStartCy);
      for (let i = 1; i < rlPoints.length; i++) {
        const [cpx, cpy] = toCanvas(rlPoints[i][0], rlPoints[i][1]);
        ctx.lineTo(cpx, cpy);
      }
      ctx.closePath();
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 3. Draw Vehicle History Trail
    if (historyTrail.length > 1) {
      ctx.strokeStyle = '#3b82f6';
      ctx.lineWidth = 2;
      ctx.beginPath();
      const [hStartCx, hStartCy] = toCanvas(historyTrail[0][0], historyTrail[0][1]);
      ctx.moveTo(hStartCx, hStartCy);
      for (let i = 1; i < historyTrail.length; i++) {
        const [cpx, cpy] = toCanvas(historyTrail[i][0], historyTrail[i][1]);
        ctx.lineTo(cpx, cpy);
      }
      ctx.stroke();
    }

    // 4. Draw MPC Lookahead Waypoints
    if (currentFrame?.lookahead_waypoints && currentFrame.lookahead_waypoints.length > 0) {
      ctx.fillStyle = '#f59e0b';
      currentFrame.lookahead_waypoints.forEach((pt) => {
        const [lx, ly] = toCanvas(pt[0], pt[1]);
        ctx.beginPath();
        ctx.arc(lx, ly, 3, 0, 2 * Math.PI);
        ctx.fill();
      });
    }

    // 5. Draw Vehicle Pose
    if (currentFrame?.pose) {
      const [vx, vy] = toCanvas(currentFrame.pose.x, currentFrame.pose.y);
      const theta = currentFrame.pose.theta_rad;

      ctx.save();
      ctx.translate(vx, vy);
      ctx.rotate(-theta); // Screen rotation inverted

      // Vehicle Body Box
      ctx.fillStyle = currentFrame.estop_active ? '#ef4444' : '#10b981';
      ctx.shadowColor = currentFrame.estop_active ? '#ef4444' : '#10b981';
      ctx.shadowBlur = 12;
      ctx.fillRect(-12, -7, 24, 14);

      // Heading arrow indicator
      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.moveTo(12, 0);
      ctx.lineTo(6, -5);
      ctx.lineTo(6, 5);
      ctx.closePath();
      ctx.fill();

      ctx.restore();
    }
  }, [currentFrame, circuitDetail, historyTrail]);

  return (
    <Box sx={{ position: 'relative', width: '100%', height: '100%', minHeight: 440, bgcolor: '#0b0f19', borderRadius: 2, overflow: 'hidden', border: '1px solid #1f2937' }}>
      <canvas ref={canvasRef} width={800} height={500} style={{ width: '100%', height: '100%', display: 'block' }} />
      <Box sx={{ position: 'absolute', bottom: 12, left: 16, bgcolor: 'rgba(15, 23, 42, 0.85)', backdropFilter: 'blur(6px)', p: 1.5, borderRadius: 1.5, border: '1px solid #334155' }}>
        <Typography variant="caption" sx={{ color: '#06b6d4', fontWeight: 700, display: 'block' }}>
          TRACK: {circuitDetail?.name || 'MONZA'}
        </Typography>
        <Typography variant="caption" sx={{ color: '#94a3b8', display: 'block' }}>
          X: {currentFrame?.pose.x.toFixed(2)} m | Y: {currentFrame?.pose.y.toFixed(2)} m | Heading: {currentFrame?.pose.theta_deg.toFixed(1)}°
        </Typography>
      </Box>
    </Box>
  );
};
