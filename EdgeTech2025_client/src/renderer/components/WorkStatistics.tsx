import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Grid,
  Chip,
  LinearProgress,
  Divider,
  useTheme,
  Skeleton,
  Tooltip,
  IconButton
} from '@mui/material';
import {
  Build,
  ViewInAr,
  Assignment,
  TrendingUp,
  TrendingDown,
  TrendingFlat,
  Refresh,
  Timer,
  Assessment
} from '@mui/icons-material';
import { WorkStatistics as WorkStatsType } from '../../shared/types';

interface WorkStatisticsProps {
  statistics: WorkStatsType;
  height?: number;
  title?: string;
  showRefreshButton?: boolean;
  onRefresh?: () => void;
  autoRefresh?: boolean;
  refreshInterval?: number;
}

interface TaskStatItem {
  icon: React.ReactNode;
  label: string;
  count: number;
  color: string;
  target?: number;
}

const WorkStatistics: React.FC<WorkStatisticsProps> = ({
  statistics,
  height = 300,
  title = '作業統計',
  showRefreshButton = true,
  onRefresh,
  autoRefresh = true,
  refreshInterval = 30000 // 30秒
}) => {
  const theme = useTheme();
  const [isLoading, setIsLoading] = useState(false);
  const [lastUpdated, setLastUpdated] = useState<Date>(new Date());

  // 自動更新の設定
  useEffect(() => {
    if (autoRefresh && onRefresh) {
      const interval = setInterval(() => {
        onRefresh();
        setLastUpdated(new Date());
      }, refreshInterval);

      return () => clearInterval(interval);
    }
  }, [autoRefresh, onRefresh, refreshInterval]);

  // 手動更新ハンドラ
  const handleRefresh = async () => {
    if (onRefresh) {
      setIsLoading(true);
      try {
        await onRefresh();
        setLastUpdated(new Date());
      } finally {
        setTimeout(() => setIsLoading(false), 500);
      }
    }
  };

  // 作業タスクの統計データ
  const taskStats: TaskStatItem[] = [
    {
      icon: <Build sx={{ fontSize: 20 }} />,
      label: 'ネジ締め',
      count: statistics.screwCount,
      color: '#ff6384',
      target: 10 // 1日の目標値
    },
    {
      icon: <ViewInAr sx={{ fontSize: 20 }} />,
      label: '積み木',
      count: statistics.blocksCount,
      color: '#36a2eb',
      target: 8
    },
    {
      icon: <Assignment sx={{ fontSize: 20 }} />,
      label: 'アンケート',
      count: statistics.surveyCount,
      color: '#ffce56',
      target: 15
    }
  ];

  // パフォーマンス評価の表示設定
  const getPerformanceDisplay = () => {
    switch (statistics.recentPerformance) {
      case 'fast':
        return {
          icon: <TrendingUp sx={{ color: '#4caf50' }} />,
          label: '高速',
          color: '#4caf50',
          description: '標準時間より20%以上速い'
        };
      case 'slow':
        return {
          icon: <TrendingDown sx={{ color: '#f44336' }} />,
          label: '低速',
          color: '#f44336',
          description: '標準時間より20%以上遅い'
        };
      case 'normal':
        return {
          icon: <TrendingFlat sx={{ color: '#ff9800' }} />,
          label: '標準',
          color: '#ff9800',
          description: '標準時間の範囲内'
        };
      default:
        return {
          icon: <Assessment sx={{ color: theme.palette.text.secondary }} />,
          label: '評価中',
          color: theme.palette.text.secondary,
          description: 'データ不足のため評価できません'
        };
    }
  };

  const performanceDisplay = getPerformanceDisplay();

  // 作業時間の表示フォーマット
  const formatWorkTime = (seconds: number): string => {
    if (seconds < 60) {
      return `${seconds}秒`;
    } else if (seconds < 3600) {
      const minutes = Math.floor(seconds / 60);
      const remainingSeconds = seconds % 60;
      return remainingSeconds > 0 ? `${minutes}分${remainingSeconds}秒` : `${minutes}分`;
    } else {
      const hours = Math.floor(seconds / 3600);
      const minutes = Math.floor((seconds % 3600) / 60);
      return minutes > 0 ? `${hours}時間${minutes}分` : `${hours}時間`;
    }
  };

  // 進捗率の計算
  const calculateProgress = (current: number, target: number): number => {
    return Math.min((current / target) * 100, 100);
  };

  return (
    <Card sx={{ height: height, display: 'flex', flexDirection: 'column' }}>
      <CardContent sx={{ flexGrow: 1, display: 'flex', flexDirection: 'column' }}>
        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
          <Typography variant="h6" component="h2">
            {title}
          </Typography>
          {showRefreshButton && (
            <Tooltip title="統計データを更新">
              <IconButton 
                onClick={handleRefresh} 
                disabled={isLoading}
                size="small"
              >
                <Refresh sx={{ 
                  animation: isLoading ? 'spin 1s linear infinite' : 'none',
                  '@keyframes spin': {
                    '0%': { transform: 'rotate(0deg)' },
                    '100%': { transform: 'rotate(360deg)' }
                  }
                }} />
              </IconButton>
            </Tooltip>
          )}
        </Box>

        <Box sx={{ height: height - 80, overflow: 'auto' }}>
          {isLoading ? (
            <Box sx={{ space: 2 }}>
              <Skeleton variant="rectangular" height={60} sx={{ mb: 2 }} />
              <Skeleton variant="rectangular" height={60} sx={{ mb: 2 }} />
              <Skeleton variant="rectangular" height={60} sx={{ mb: 2 }} />
            </Box>
          ) : (
            <>
              {/* 作業実行回数 */}
              <Typography variant="subtitle2" gutterBottom sx={{ fontWeight: 'bold' }}>
                本日の作業実行回数
              </Typography>
              
              <Grid container spacing={1} sx={{ mb: 3 }}>
                {taskStats.map((task, index) => (
                  <Grid item xs={12} key={index}>
                    <Box
                      sx={{
                        display: 'flex',
                        alignItems: 'center',
                        p: 1.5,
                        backgroundColor: theme.palette.background.default,
                        borderRadius: 1,
                        border: `1px solid ${theme.palette.divider}`
                      }}
                    >
                      <Box sx={{ color: task.color, mr: 1 }}>
                        {task.icon}
                      </Box>
                      <Box sx={{ flexGrow: 1, mr: 2 }}>
                        <Typography variant="body2" sx={{ fontWeight: 'medium' }}>
                          {task.label}
                        </Typography>
                        <Box sx={{ display: 'flex', alignItems: 'center', mt: 0.5 }}>
                          <Typography variant="h6" sx={{ mr: 1, color: task.color }}>
                            {task.count}
                          </Typography>
                          <Typography variant="caption" color="text.secondary">
                            / {task.target} 回
                          </Typography>
                        </Box>
                        <LinearProgress
                          variant="determinate"
                          value={calculateProgress(task.count, task.target || 1)}
                          sx={{
                            mt: 0.5,
                            height: 4,
                            borderRadius: 2,
                            backgroundColor: theme.palette.grey[300],
                            '& .MuiLinearProgress-bar': {
                              backgroundColor: task.color
                            }
                          }}
                        />
                      </Box>
                      <Typography variant="caption" color="text.secondary">
                        {Math.round(calculateProgress(task.count, task.target || 1))}%
                      </Typography>
                    </Box>
                  </Grid>
                ))}
              </Grid>

              <Divider sx={{ my: 2 }} />

              {/* 作業時間統計 */}
              <Typography variant="subtitle2" gutterBottom sx={{ fontWeight: 'bold' }}>
                作業時間統計
              </Typography>
              
              <Grid container spacing={2} sx={{ mb: 3 }}>
                <Grid item xs={6}>
                  <Box sx={{ textAlign: 'center', p: 1 }}>
                    <Timer sx={{ color: theme.palette.primary.main, mb: 0.5 }} />
                    <Typography variant="caption" display="block" color="text.secondary">
                      総作業時間
                    </Typography>
                    <Typography variant="body1" sx={{ fontWeight: 'bold' }}>
                      {formatWorkTime(statistics.totalWorkTime)}
                    </Typography>
                  </Box>
                </Grid>
                <Grid item xs={6}>
                  <Box sx={{ textAlign: 'center', p: 1 }}>
                    <Assessment sx={{ color: theme.palette.secondary.main, mb: 0.5 }} />
                    <Typography variant="caption" display="block" color="text.secondary">
                      平均作業時間
                    </Typography>
                    <Typography variant="body1" sx={{ fontWeight: 'bold' }}>
                      {formatWorkTime(statistics.averageWorkTime)}
                    </Typography>
                  </Box>
                </Grid>
              </Grid>

              <Divider sx={{ my: 2 }} />

              {/* 直近作業評価 */}
              <Typography variant="subtitle2" gutterBottom sx={{ fontWeight: 'bold' }}>
                直近作業評価
              </Typography>
              
              <Box
                sx={{
                  display: 'flex',
                  alignItems: 'center',
                  p: 2,
                  backgroundColor: theme.palette.background.default,
                  borderRadius: 1,
                  border: `1px solid ${theme.palette.divider}`
                }}
              >
                <Box sx={{ mr: 2 }}>
                  {performanceDisplay.icon}
                </Box>
                <Box sx={{ flexGrow: 1 }}>
                  <Typography variant="body1" sx={{ fontWeight: 'bold', color: performanceDisplay.color }}>
                    {performanceDisplay.label}
                  </Typography>
                  <Typography variant="caption" color="text.secondary">
                    {performanceDisplay.description}
                  </Typography>
                </Box>
                <Chip
                  label={performanceDisplay.label}
                  size="small"
                  sx={{
                    backgroundColor: `${performanceDisplay.color}20`,
                    color: performanceDisplay.color,
                    fontWeight: 'bold'
                  }}
                />
              </Box>

              {/* 最終更新時刻 */}
              <Box sx={{ mt: 2, textAlign: 'center' }}>
                <Typography variant="caption" color="text.secondary">
                  最終更新: {lastUpdated.toLocaleTimeString('ja-JP')}
                </Typography>
              </Box>
            </>
          )}
        </Box>
      </CardContent>
    </Card>
  );
};

export default WorkStatistics;