import React, { useState, useEffect } from 'react';
import {
  Box,
  Paper,
  Typography,
  Slider,
  IconButton,
  Button,
  Grid,
  Chip,
  Card,
  CardContent,
} from '@mui/material';
import {
  Play,
  Pause,
  RotateCcw,
  SkipForward,
  SkipBack,
  AlertTriangle,
  Clock,
  Car,
} from 'lucide-react';
import { ReplayTimeline, TelemetryFrame } from '../../types';
import { api } from '../../api/client';

export const ReplayScrubber: React.FC = () => {
  const [timeline, setTimeline] = useState<ReplayTimeline | null>(null);
  const [currentFrame, setCurrentFrame] = useState<TelemetryFrame | null>(null);
  const [currentTime, setCurrentTime] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [speed, setSpeed] = useState(1.0);

  const loadTimeline = () => {
    api.getReplayTimeline().then(setTimeline).catch(console.error);
  };

  useEffect(() => {
    loadTimeline();
  }, []);

  const handleSeek = async (target: number) => {
    setCurrentTime(target);
    try {
      const frame = await api.seekReplay(target);
      setCurrentFrame(frame);
    } catch (e) {
      console.warn('Seek error', e);
    }
  };

  const handleTogglePlay = async () => {
    const next = !isPlaying;
    setIsPlaying(next);
    await api.controlReplay(next ? 'play' : 'pause', speed);
  };

  const maxDuration = timeline?.duration_seconds || 100;
  const incidents = timeline?.incidents || [];

  return (
    <Box sx={{ p: 3, display: 'flex', flexDirection: 'column', gap: 3 }}>
      <Box>
        <Typography variant="h5" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
          PROVING GROUND TELEMETRY REPLAY SYSTEM
        </Typography>
        <Typography variant="body2" sx={{ color: '#9ca3af' }}>
          Frame-accurate historical playback, interactive timeline scrubbing, and critical safety incident inspection.
        </Typography>
      </Box>

      {/* Main Scrubber Control Bar */}
      <Paper sx={{ p: 3, bgcolor: '#111827', borderRadius: 2, border: '1px solid #1f2937' }}>
        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1.5 }}>
            <Clock size={18} color="#06b6d4" />
            <Typography variant="h6" sx={{ fontFamily: 'monospace', fontWeight: 800, color: '#f9fafb' }}>
              t = {currentTime.toFixed(2)} s / {maxDuration.toFixed(2)} s
            </Typography>
          </Box>
          <Box sx={{ display: 'flex', gap: 1 }}>
            {[0.5, 1.0, 2.0, 5.0].map((s) => (
              <Chip
                key={s}
                label={`${s}x`}
                size="small"
                onClick={() => setSpeed(s)}
                sx={{
                  bgcolor: speed === s ? '#06b6d4' : '#1e293b',
                  color: speed === s ? '#0f172a' : '#9ca3af',
                  fontWeight: 700,
                  cursor: 'pointer',
                }}
              />
            ))}
          </Box>
        </Box>

        {/* Timeline Slider with Marks */}
        <Box sx={{ px: 2, my: 1 }}>
          <Slider
            value={currentTime}
            min={0}
            max={Math.max(10, maxDuration)}
            step={0.05}
            onChange={(_, val) => handleSeek(val as number)}
            sx={{
              color: '#06b6d4',
              height: 8,
              '& .MuiSlider-thumb': {
                width: 18,
                height: 18,
                bgcolor: '#38bdf8',
                boxShadow: '0 0 10px rgba(56, 189, 248, 0.5)',
              },
            }}
          />
        </Box>

        {/* Playback Controls */}
        <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', gap: 2, mt: 1 }}>
          <IconButton onClick={() => handleSeek(Math.max(0, currentTime - 1.0))} sx={{ color: '#9ca3af' }}>
            <SkipBack size={20} />
          </IconButton>
          <Button
            variant="contained"
            onClick={handleTogglePlay}
            sx={{
              bgcolor: isPlaying ? '#f59e0b' : '#06b6d4',
              color: '#0f172a',
              fontWeight: 800,
              width: 120,
            }}
          >
            {isPlaying ? 'PAUSE' : 'PLAY'}
          </Button>
          <IconButton onClick={() => handleSeek(Math.min(maxDuration, currentTime + 1.0))} sx={{ color: '#9ca3af' }}>
            <SkipForward size={20} />
          </IconButton>
          <IconButton onClick={() => handleSeek(0)} sx={{ color: '#9ca3af' }}>
            <RotateCcw size={18} />
          </IconButton>
        </Box>
      </Paper>

      {/* Incident Bookmarks List */}
      <Paper sx={{ p: 2.5, bgcolor: '#111827', borderRadius: 2 }}>
        <Typography variant="subtitle2" sx={{ fontWeight: 700, color: '#f3f4f6', mb: 2 }}>
          DETECTED SAFETY INCIDENTS & BOOKMARKS ({incidents.length})
        </Typography>
        {incidents.length === 0 ? (
          <Typography variant="body2" sx={{ color: '#6b7280', py: 2, textAlign: 'center' }}>
            Zero safety boundary excursions or E-stop triggers detected in this recording buffer.
          </Typography>
        ) : (
          <Grid container spacing={2}>
            {incidents.map((inc) => (
              <Grid item xs={12} sm={6} md={4} key={inc.incident_id}>
                <Card sx={{ bgcolor: '#0b0f19', border: '1px solid #334155', borderRadius: 1.5 }}>
                  <CardContent sx={{ p: 2 }}>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                      <Chip label={inc.incident_type} size="small" sx={{ bgcolor: 'rgba(239, 68, 68, 0.2)', color: '#f87171', fontWeight: 700, fontSize: '0.65rem' }} />
                      <Typography variant="caption" sx={{ color: '#06b6d4', fontFamily: 'monospace', fontWeight: 700 }}>
                        t = {inc.timestamp.toFixed(2)}s
                      </Typography>
                    </Box>
                    <Typography variant="body2" sx={{ color: '#e2e8f0', fontSize: '0.82rem', mb: 1.5 }}>
                      {inc.description}
                    </Typography>
                    <Button size="small" variant="outlined" onClick={() => handleSeek(inc.timestamp)} sx={{ color: '#38bdf8', borderColor: '#334155' }}>
                      Jump to Frame
                    </Button>
                  </CardContent>
                </Card>
              </Grid>
            ))}
          </Grid>
        )}
      </Paper>
    </Box>
  );
};
