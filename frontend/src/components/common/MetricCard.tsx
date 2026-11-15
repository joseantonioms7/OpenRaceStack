import React from 'react';
import { Card, CardContent, Typography, Box } from '@mui/material';

interface MetricCardProps {
  title: string;
  value: string | number;
  unit?: string;
  subtitle?: string;
  status?: 'nominal' | 'warning' | 'critical' | 'info';
  icon?: React.ReactNode;
}

export const MetricCard: React.FC<MetricCardProps> = ({
  title,
  value,
  unit,
  subtitle,
  status = 'nominal',
  icon,
}) => {
  const getStatusColor = () => {
    switch (status) {
      case 'nominal':
        return '#10b981';
      case 'warning':
        return '#f59e0b';
      case 'critical':
        return '#f43f5e';
      default:
        return '#06b6d4';
    }
  };

  const color = getStatusColor();

  return (
    <Card
      sx={{
        bgcolor: '#111827',
        border: '1px solid #1f2937',
        borderRadius: 2,
        height: '100%',
        position: 'relative',
        overflow: 'hidden',
        '&::before': {
          content: '""',
          position: 'absolute',
          top: 0,
          left: 0,
          right: 0,
          height: 3,
          bgcolor: color,
        },
      }}
    >
      <CardContent sx={{ p: 2, '&:last-child': { pb: 2 } }}>
        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', mb: 1 }}>
          <Typography variant="caption" sx={{ color: '#9ca3af', fontWeight: 600, letterSpacing: 0.5, textTransform: 'uppercase' }}>
            {title}
          </Typography>
          {icon && <Box sx={{ color }}>{icon}</Box>}
        </Box>

        <Box sx={{ display: 'flex', alignItems: 'baseline', gap: 0.8, my: 0.5 }}>
          <Typography
            variant="h4"
            sx={{
              fontWeight: 800,
              fontFamily: '"JetBrains Mono", monospace',
              color: '#f9fafb',
              lineHeight: 1,
            }}
          >
            {value}
          </Typography>
          {unit && (
            <Typography variant="body2" sx={{ color: '#6b7280', fontWeight: 600 }}>
              {unit}
            </Typography>
          )}
        </Box>

        {subtitle && (
          <Typography variant="caption" sx={{ color: '#6b7280', display: 'block', mt: 0.5 }}>
            {subtitle}
          </Typography>
        )}
      </CardContent>
    </Card>
  );
};
