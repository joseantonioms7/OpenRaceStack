import React, { useState, useEffect } from 'react';
import {
  Box,
  Paper,
  Typography,
  Button,
  Grid,
  Card,
  CardContent,
  Chip,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  TextField,
  CircularProgress,
} from '@mui/material';
import { FileText, Download, Plus, CheckCircle2 } from 'lucide-react';
import { ValidationReport } from '../../types';
import { api } from '../../api/client';

export const ReportGenerator: React.FC = () => {
  const [reports, setReports] = useState<ValidationReport[]>([]);
  const [selectedReport, setSelectedReport] = useState<ValidationReport | null>(null);
  const [openModal, setOpenModal] = useState(false);
  const [loading, setLoading] = useState(false);
  const [formData, setFormData] = useState({
    session_id: 'monza-proving-ground-run',
    title: 'ApexPathAI Formal Proving Ground Dossier',
    author: 'Principal Autonomous Safety Architect',
  });

  const loadReports = () => {
    setLoading(true);
    api.getReports()
      .then(setReports)
      .catch(console.error)
      .finally(() => setLoading(false));
  };

  useEffect(() => {
    loadReports();
  }, []);

  const handleGenerate = async () => {
    const rpt = await api.generateReport(formData);
    setOpenModal(false);
    setSelectedReport(rpt);
    loadReports();
  };

  const downloadFile = (content: string, filename: string, type: string) => {
    const blob = new Blob([content], { type });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    a.click();
    URL.revokeObjectURL(url);
  };

  return (
    <Box sx={{ p: 3, display: 'flex', flexDirection: 'column', gap: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <Box>
          <Typography variant="h5" sx={{ fontWeight: 800, color: '#f3f4f6' }}>
            VALIDATION DOSSIERS & REGULATORY REPORTING
          </Typography>
          <Typography variant="body2" sx={{ color: '#9ca3af' }}>
            Automated compilation of formal proving ground sign-offs, KPI compliance matrices, and safety documentation.
          </Typography>
        </Box>
        <Button
          variant="contained"
          startIcon={<Plus size={18} />}
          onClick={() => setOpenModal(true)}
          sx={{ bgcolor: '#06b6d4', color: '#0f172a', fontWeight: 700 }}
        >
          Compile New Dossier
        </Button>
      </Box>

      {/* Reports List */}
      <Grid container spacing={2.5}>
        {reports.map((r) => (
          <Grid item xs={12} md={6} key={r.id}>
            <Card sx={{ bgcolor: '#111827', border: '1px solid #1f2937', borderRadius: 2 }}>
              <CardContent sx={{ p: 2.5 }}>
                <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', mb: 1 }}>
                  <Typography variant="h6" sx={{ fontWeight: 700, color: '#f9fafb' }}>
                    {r.title}
                  </Typography>
                  <Chip
                    label={r.verdict}
                    size="small"
                    sx={{ bgcolor: 'rgba(16, 185, 129, 0.2)', color: '#10b981', fontWeight: 800 }}
                  />
                </Box>
                <Typography variant="caption" sx={{ color: '#6b7280', display: 'block', mb: 1 }}>
                  ID: {r.id} | Session: {r.session_id} | Author: {r.author}
                </Typography>
                <Typography variant="body2" sx={{ color: '#94a3b8', mb: 2, fontSize: '0.85rem' }}>
                  {r.summary}
                </Typography>

                <Box sx={{ display: 'flex', gap: 1.5 }}>
                  <Button
                    variant="outlined"
                    size="small"
                    onClick={() => setSelectedReport(r)}
                    sx={{ borderColor: '#334155', color: '#38bdf8' }}
                  >
                    View Dossier
                  </Button>
                  <Button
                    variant="text"
                    size="small"
                    startIcon={<Download size={16} />}
                    onClick={() => downloadFile(r.content_markdown, `${r.id}.md`, 'text/markdown')}
                    sx={{ color: '#9ca3af' }}
                  >
                    Download Markdown
                  </Button>
                </Box>
              </CardContent>
            </Card>
          </Grid>
        ))}
      </Grid>

      {/* Dossier Preview Modal */}
      {selectedReport && (
        <Dialog open={!!selectedReport} onClose={() => setSelectedReport(null)} maxWidth="md" fullWidth>
          <DialogTitle sx={{ bgcolor: '#111827', color: '#f3f4f6', fontWeight: 700, display: 'flex', justifyContent: 'space-between' }}>
            <span>{selectedReport.title}</span>
            <Chip label={selectedReport.verdict} size="small" sx={{ bgcolor: 'rgba(16, 185, 129, 0.2)', color: '#10b981' }} />
          </DialogTitle>
          <DialogContent sx={{ bgcolor: '#111827' }}>
            <Box sx={{ bgcolor: '#0b0f19', p: 3, borderRadius: 2, border: '1px solid #1f2937', color: '#e2e8f0', fontFamily: 'monospace', fontSize: '0.82rem', whiteSpace: 'pre-wrap' }}>
              {selectedReport.content_markdown}
            </Box>
          </DialogContent>
          <DialogActions sx={{ bgcolor: '#111827', p: 2 }}>
            <Button onClick={() => setSelectedReport(null)} sx={{ color: '#9ca3af' }}>Close</Button>
            <Button
              variant="contained"
              startIcon={<Download size={16} />}
              onClick={() => downloadFile(selectedReport.content_markdown, `${selectedReport.id}.md`, 'text/markdown')}
              sx={{ bgcolor: '#06b6d4', color: '#0f172a', fontWeight: 700 }}
            >
              Export Report (.md)
            </Button>
          </DialogActions>
        </Dialog>
      )}

      {/* Generate Report Dialog */}
      <Dialog open={openModal} onClose={() => setOpenModal(false)} maxWidth="sm" fullWidth>
        <DialogTitle sx={{ bgcolor: '#111827', color: '#f3f4f6', fontWeight: 700 }}>
          Compile Formal Proving Ground Dossier
        </DialogTitle>
        <DialogContent sx={{ bgcolor: '#111827', display: 'flex', flexDirection: 'column', gap: 2, pt: 2 }}>
          <TextField
            label="Report Title"
            fullWidth
            value={formData.title}
            onChange={(e) => setFormData({ ...formData, title: e.target.value })}
          />
          <TextField
            label="Author / Certifying Engineer"
            fullWidth
            value={formData.author}
            onChange={(e) => setFormData({ ...formData, author: e.target.value })}
          />
          <TextField
            label="Target Session ID"
            fullWidth
            value={formData.session_id}
            onChange={(e) => setFormData({ ...formData, session_id: e.target.value })}
          />
        </DialogContent>
        <DialogActions sx={{ bgcolor: '#111827', p: 2 }}>
          <Button onClick={() => setOpenModal(false)} sx={{ color: '#9ca3af' }}>Cancel</Button>
          <Button onClick={handleGenerate} variant="contained" sx={{ bgcolor: '#06b6d4', color: '#0f172a', fontWeight: 700 }}>
            Generate & Sign Off
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};
