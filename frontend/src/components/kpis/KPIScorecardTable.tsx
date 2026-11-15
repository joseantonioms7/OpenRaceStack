import React, { useState, useEffect } from 'react';
import {
  Box,
  Paper,
  Typography,
  Table,
  TableHead,
  TableRow,
  TableCell,
  TableBody,
  Chip,
  Grid,
  CircularProgress,
} from '@mui/material';
import { CheckCircle2, AlertTriangle, XCircle, Award } from 'lucide-react';
import { KPIScorecard } from '../../types';
import { api } from '../../api/client';

export const KPIScorecardTable: React.FC = () => {
  const [scorecard, setScorecard] = useState<KPIScorecard | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    api.getKPIScorecard()
      .then(setScorecard)
      .catch(console.error)
      .finally(() => setLoading(false));
  }, []);

  if (loading) {
    return (
      <Box sx={{ p: 4, textAlign: 'center' }}>
        <CircularProgress size={32} sx={{ color: '#06b6d4' }} />
      </Box>
    );
  }

  const isPass = scorecard?.overall_verdict === 'PASS';

  return (
    <Box sx={{ p: 3, display: 'flex', flexDirection: 'column', gap: 3 }}>
      <Box>
        <Typography variant="h5" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
          PROVING GROUND SAFETY & PERFORMANCE KPI SCORECARD
        </Typography>
        <Typography variant="body2" sx={{ color: '#9ca3af' }}>
          Automated regulatory verification against 7 foundational ISO 26262 ASIL-D & Euro NCAP safety standards.
        </Typography>
      </Box>

      {/* Overall Verdict Banner */}
      <Paper
        sx={{
          p: 3,
          bgcolor: isPass ? 'rgba(16, 185, 129, 0.12)' : 'rgba(244, 63, 94, 0.12)',
          border: 1,
          borderColor: isPass ? '#10b981' : '#f43f5e',
          borderRadius: 2,
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
        }}
      >
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
          <Box
            sx={{
              width: 50,
              height: 50,
              borderRadius: '50%',
              bgcolor: isPass ? '#10b981' : '#f43f5e',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              color: '#0f172a',
            }}
          >
            <Award size={28} />
          </Box>
          <Box>
            <Typography variant="h5" sx={{ fontWeight: 800, color: '#f9fafb' }}>
              OVERALL VERDICT: {scorecard?.overall_verdict}
            </Typography>
            <Typography variant="body2" sx={{ color: '#9ca3af' }}>
              Proving Ground Circuit: {scorecard?.track_name} | Profile: {scorecard?.profile}
            </Typography>
          </Box>
        </Box>

        <Box sx={{ textAlign: 'right' }}>
          <Typography variant="h4" sx={{ fontWeight: 800, fontFamily: 'monospace', color: isPass ? '#10b981' : '#f43f5e' }}>
            {scorecard?.compliance_score_pct}%
          </Typography>
          <Typography variant="caption" sx={{ color: '#9ca3af' }}>
            REGULATORY COMPLIANCE SCORE
          </Typography>
        </Box>
      </Paper>

      {/* Rules Evaluation Table */}
      <Paper sx={{ bgcolor: '#111827', borderRadius: 2, overflow: 'hidden', border: '1px solid #1f2937' }}>
        <Table>
          <TableHead sx={{ bgcolor: '#0b0f19' }}>
            <TableRow>
              <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>RULE ID</TableCell>
              <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>SAFETY RULE NAME</TableCell>
              <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>STANDARD THRESHOLD</TableCell>
              <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>MEASURED VALUE</TableCell>
              <TableCell sx={{ color: '#6b7280', fontWeight: 700 }}>VERDICT</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {scorecard?.rules.map((rule) => (
              <TableRow key={rule.rule_id} sx={{ '&:hover': { bgcolor: 'rgba(255,255,255,0.02)' } }}>
                <TableCell sx={{ fontFamily: 'monospace', color: '#06b6d4', fontWeight: 700 }}>
                  {rule.rule_id}
                </TableCell>
                <TableCell>
                  <Typography variant="body2" sx={{ fontWeight: 600, color: '#f3f4f6' }}>
                    {rule.rule_name}
                  </Typography>
                  <Typography variant="caption" sx={{ color: '#6b7280' }}>
                    {rule.description}
                  </Typography>
                </TableCell>
                <TableCell sx={{ fontFamily: 'monospace', color: '#94a3b8' }}>
                  {rule.threshold}
                </TableCell>
                <TableCell sx={{ fontFamily: 'monospace', color: '#f9fafb', fontWeight: 700 }}>
                  {rule.measured_value}
                </TableCell>
                <TableCell>
                  <Chip
                    icon={rule.passed ? <CheckCircle2 size={14} /> : <XCircle size={14} />}
                    label={rule.status}
                    size="small"
                    sx={{
                      bgcolor: rule.passed ? 'rgba(16, 185, 129, 0.15)' : 'rgba(244, 63, 94, 0.2)',
                      color: rule.passed ? '#10b981' : '#f43f5e',
                      fontWeight: 800,
                    }}
                  />
                </TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      </Paper>
    </Box>
  );
};
