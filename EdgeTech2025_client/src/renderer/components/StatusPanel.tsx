import React from 'react';
import {
  Card,
  CardContent,
  Typography,
  Box,
  Chip,
  Avatar
} from '@mui/material';
import {
  Person,
  SmartToy,
  CheckCircle,
  Schedule,
  Work,
  Error,
  PanTool,
  OpenInFull
} from '@mui/icons-material';

export interface StatusPanelProps {
  title: string;
  status: string;
  type: 'worker' | 'robot';
  robotGrip?: 'open' | 'closed';
  lastUpdated?: Date | null;
  height?: number;
}

// 作業者状態のマッピング
const workerStatusConfig = {
  'Absent': {
    label: '不在',
    color: 'error' as const,
    icon: <Error />,
    backgroundColor: '#d32f2f',
    illustration: '👤❌'
  },
  'Waiting': {
    label: '待機中',
    color: 'warning' as const,
    icon: <Schedule />,
    backgroundColor: '#ed6c02',
    illustration: '👤⏳'
  },
  'Working': {
    label: '作業中',
    color: 'success' as const,
    icon: <Work />,
    backgroundColor: '#2e7d32',
    illustration: '👤🔧'
  },
  'Work Completed': {
    label: '作業完了',
    color: 'info' as const,
    icon: <CheckCircle />,
    backgroundColor: '#0288d1',
    illustration: '👤✅'
  }
};

// ロボット状態のマッピング
const robotStatusConfig = {
  'idle': {
    label: 'アイドル',
    color: 'default' as const,
    icon: <SmartToy />,
    backgroundColor: '#757575',
    illustration: '🤖💤'
  },
  'moving': {
    label: '移動中',
    color: 'info' as const,
    icon: <SmartToy />,
    backgroundColor: '#0288d1',
    illustration: '🤖➡️'
  },
  'working': {
    label: '作業中',
    color: 'success' as const,
    icon: <Work />,
    backgroundColor: '#2e7d32',
    illustration: '🤖🔧'
  },
  'error': {
    label: 'エラー',
    color: 'error' as const,
    icon: <Error />,
    backgroundColor: '#d32f2f',
    illustration: '🤖❌'
  },
  'ready': {
    label: '準備完了',
    color: 'success' as const,
    icon: <CheckCircle />,
    backgroundColor: '#2e7d32',
    illustration: '🤖✅'
  }
};

const StatusPanel: React.FC<StatusPanelProps> = ({
  title,
  status,
  type,
  robotGrip,
  lastUpdated,
  height = 320
}) => {
  // 状態設定の取得
  const getStatusConfig = () => {
    if (type === 'worker') {
      return workerStatusConfig[status as keyof typeof workerStatusConfig] || {
        label: status,
        color: 'default' as const,
        icon: <Person />,
        backgroundColor: '#757575',
        illustration: '👤❓'
      };
    } else {
      return robotStatusConfig[status as keyof typeof robotStatusConfig] || {
        label: status,
        color: 'default' as const,
        icon: <SmartToy />,
        backgroundColor: '#757575',
        illustration: '🤖❓'
      };
    }
  };

  const statusConfig = getStatusConfig();

  // ロボットのグリップ状態表示
  const getGripDisplay = () => {
    if (type !== 'robot' || !robotGrip) return null;
    
    return (
      <Chip
        icon={robotGrip === 'open' ? <OpenInFull /> : <PanTool />}
        label={robotGrip === 'open' ? 'グリップ開' : 'グリップ閉'}
        color={robotGrip === 'open' ? 'default' : 'primary'}
        size="small"
        sx={{ mt: 1 }}
      />
    );
  };

  return (
    <Card sx={{ height: height, display: 'flex', flexDirection: 'column' }}>
      <CardContent sx={{ flexGrow: 1, display: 'flex', flexDirection: 'column' }}>
        <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 2 }}>
          <Typography variant="h6" component="div">
            {title}
          </Typography>
          <Avatar sx={{ bgcolor: statusConfig.backgroundColor }}>
            {type === 'worker' ? <Person /> : <SmartToy />}
          </Avatar>
        </Box>

        {/* 状態表示エリア */}
        <Box
          sx={{
            flexGrow: 1,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            backgroundColor: '#2a2a2a',
            borderRadius: 1,
            p: 2,
            position: 'relative',
            minHeight: 150
          }}
        >
          {/* イラスト表示 */}
          <Typography
            variant="h1"
            sx={{
              fontSize: '4rem',
              mb: 2,
              userSelect: 'none'
            }}
          >
            {statusConfig.illustration}
          </Typography>

          {/* 状態チップ */}
          <Chip
            icon={statusConfig.icon}
            label={statusConfig.label}
            color={statusConfig.color}
            sx={{
              fontSize: '1rem',
              height: 'auto',
              py: 1,
              px: 2,
              '& .MuiChip-label': {
                fontWeight: 'bold'
              }
            }}
          />

          {/* ロボットのグリップ状態 */}
          {getGripDisplay()}
        </Box>

        {/* 最終更新時刻 */}
        {lastUpdated && (
          <Typography variant="caption" color="text.secondary" sx={{ mt: 1, display: 'block' }}>
            最終更新: {lastUpdated.toLocaleString('ja-JP')}
          </Typography>
        )}

        {/* 詳細情報 */}
        <Box sx={{ mt: 2 }}>
          <Typography variant="body2" color="text.secondary">
            現在の状態: <strong>{statusConfig.label}</strong>
          </Typography>
          {type === 'robot' && robotGrip && (
            <Typography variant="body2" color="text.secondary">
              グリップ: <strong>{robotGrip === 'open' ? '開' : '閉'}</strong>
            </Typography>
          )}
        </Box>
      </CardContent>
    </Card>
  );
};

export default StatusPanel;