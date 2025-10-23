import React, { useState, useEffect, useMemo } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  ToggleButton,
  ToggleButtonGroup,
  useTheme,
  Skeleton
} from '@mui/material';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  Title,
  Tooltip,
  Legend,
  TimeScale,
  ChartOptions
} from 'chart.js';
import { Line } from 'react-chartjs-2';
import 'chartjs-adapter-date-fns';
import { ja } from 'date-fns/locale';
import { WorkHistoryEntry } from '../../shared/types';

// Chart.jsの登録
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  Title,
  Tooltip,
  Legend,
  TimeScale
);

interface TimeChartProps {
  data: WorkHistoryEntry[];
  height?: number;
  title?: string;
  showControls?: boolean;
}

type TimeRange = '1h' | '4h' | '24h';

const TimeChart: React.FC<TimeChartProps> = ({
  data,
  height = 300,
  title = '作業履歴タイムチャート',
  showControls = true
}) => {
  const theme = useTheme();
  const [timeRange, setTimeRange] = useState<TimeRange>('4h');
  const [isLoading, setIsLoading] = useState(false);

  // 時間範囲に基づいてデータをフィルタリング
  const filteredData = useMemo(() => {
    const now = new Date();
    const cutoffTime = new Date();
    
    switch (timeRange) {
      case '1h':
        cutoffTime.setHours(now.getHours() - 1);
        break;
      case '4h':
        cutoffTime.setHours(now.getHours() - 4);
        break;
      case '24h':
        cutoffTime.setDate(now.getDate() - 1);
        break;
    }

    return data.filter(entry => entry.timestamp >= cutoffTime);
  }, [data, timeRange]);

  // 作業タスク別の色分け設定
  const taskColors = {
    'Screw_tightening': {
      backgroundColor: 'rgba(255, 99, 132, 0.2)',
      borderColor: 'rgba(255, 99, 132, 1)',
      label: 'ネジ締め'
    },
    'Building_blocks': {
      backgroundColor: 'rgba(54, 162, 235, 0.2)',
      borderColor: 'rgba(54, 162, 235, 1)',
      label: '積み木'
    },
    'Survey_responses': {
      backgroundColor: 'rgba(255, 206, 86, 0.2)',
      borderColor: 'rgba(255, 206, 86, 1)',
      label: 'アンケート'
    },
    'Waiting': {
      backgroundColor: 'rgba(153, 102, 255, 0.2)',
      borderColor: 'rgba(153, 102, 255, 1)',
      label: '待機'
    }
  };

  // チャートデータの準備
  const chartData = useMemo(() => {
    const datasets = Object.keys(taskColors).map(task => {
      const taskData = filteredData
        .filter(entry => entry.task === task)
        .map(entry => ({
          x: entry.timestamp,
          y: entry.duration / 60 // 分単位に変換
        }));

      return {
        label: taskColors[task as keyof typeof taskColors].label,
        data: taskData,
        backgroundColor: taskColors[task as keyof typeof taskColors].backgroundColor,
        borderColor: taskColors[task as keyof typeof taskColors].borderColor,
        borderWidth: 2,
        pointRadius: 4,
        pointHoverRadius: 6,
        tension: 0.1
      };
    });

    return { datasets };
  }, [filteredData]);

  // チャートオプション
  const chartOptions: ChartOptions<'line'> = useMemo(() => ({
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'top' as const,
        labels: {
          color: theme.palette.text.primary,
          font: {
            size: 12
          }
        }
      },
      title: {
        display: false
      },
      tooltip: {
        mode: 'index',
        intersect: false,
        backgroundColor: theme.palette.background.paper,
        titleColor: theme.palette.text.primary,
        bodyColor: theme.palette.text.secondary,
        borderColor: theme.palette.divider,
        borderWidth: 1,
        callbacks: {
          label: (context) => {
            const label = context.dataset.label || '';
            const value = context.parsed.y ?? 0;
            return `${label}: ${value.toFixed(1)}分`;
          },
          labelColor: (context) => ({
            borderColor: context.dataset.borderColor as string,
            backgroundColor: context.dataset.backgroundColor as string
          })
        }
      }
    },
    scales: {
      x: {
        type: 'time',
        time: {
          displayFormats: {
            hour: 'HH:mm',
            minute: 'HH:mm'
          },
          tooltipFormat: 'yyyy/MM/dd HH:mm'
        },
        adapters: {
          date: {
            locale: ja
          }
        },
        grid: {
          color: theme.palette.divider,
          drawBorder: false
        },
        ticks: {
          color: theme.palette.text.secondary,
          maxTicksLimit: 8
        },
        title: {
          display: true,
          text: '時刻',
          color: theme.palette.text.primary
        }
      },
      y: {
        beginAtZero: true,
        grid: {
          color: theme.palette.divider,
          drawBorder: false
        },
        ticks: {
          color: theme.palette.text.secondary,
          callback: (value) => `${value}分`
        },
        title: {
          display: true,
          text: '作業時間 (分)',
          color: theme.palette.text.primary
        }
      }
    },
    interaction: {
      mode: 'nearest',
      axis: 'x',
      intersect: false
    },
    elements: {
      point: {
        hoverBackgroundColor: theme.palette.primary.main
      }
    }
  }), [theme, timeRange]);

  // 時間範囲変更ハンドラ
  const handleTimeRangeChange = (
    event: React.MouseEvent<HTMLElement>,
    newTimeRange: TimeRange | null
  ) => {
    if (newTimeRange !== null) {
      setIsLoading(true);
      setTimeRange(newTimeRange);
      
      // 模擬的なローディング時間
      setTimeout(() => {
        setIsLoading(false);
      }, 300);
    }
  };

  // データが空の場合の表示
  if (filteredData.length === 0) {
    return (
      <Card sx={{ height: height, display: 'flex', flexDirection: 'column' }}>
        <CardContent sx={{ flexGrow: 1, display: 'flex', flexDirection: 'column' }}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
            <Typography variant="h6" component="h2">
              {title}
            </Typography>
            {showControls && (
              <ToggleButtonGroup
                value={timeRange}
                exclusive
                onChange={handleTimeRangeChange}
                size="small"
              >
                <ToggleButton value="1h">1時間</ToggleButton>
                <ToggleButton value="4h">4時間</ToggleButton>
                <ToggleButton value="24h">24時間</ToggleButton>
              </ToggleButtonGroup>
            )}
          </Box>
          <Box
            sx={{
              flexGrow: 1,
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              backgroundColor: theme.palette.background.default,
              borderRadius: 1,
              border: `1px solid ${theme.palette.divider}`,
              minHeight: 200
            }}
          >
            <Typography variant="body2" color="text.secondary">
              選択した時間範囲にデータがありません
            </Typography>
          </Box>
        </CardContent>
      </Card>
    );
  }

  return (
    <Card sx={{ height: height, display: 'flex', flexDirection: 'column' }}>
      <CardContent sx={{ flexGrow: 1, display: 'flex', flexDirection: 'column' }}>
        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
          <Typography variant="h6" component="h2">
            {title}
          </Typography>
          {showControls && (
            <ToggleButtonGroup
              value={timeRange}
              exclusive
              onChange={handleTimeRangeChange}
              size="small"
            >
              <ToggleButton value="1h">1時間</ToggleButton>
              <ToggleButton value="4h">4時間</ToggleButton>
              <ToggleButton value="24h">24時間</ToggleButton>
            </ToggleButtonGroup>
          )}
        </Box>
        
        <Box sx={{ flexGrow: 1, position: 'relative', minHeight: 200 }}>
          {isLoading ? (
            <Skeleton 
              variant="rectangular" 
              width="100%" 
              height="100%" 
              sx={{ borderRadius: 1 }}
            />
          ) : (
            <Line data={chartData} options={chartOptions} />
          )}
        </Box>
        
        {/* データ統計情報 */}
        <Box sx={{ mt: 2, display: 'flex', gap: 2, flexWrap: 'wrap' }}>
          <Typography variant="caption" color="text.secondary">
            表示期間: {timeRange === '1h' ? '1時間' : timeRange === '4h' ? '4時間' : '24時間'}
          </Typography>
          <Typography variant="caption" color="text.secondary">
            データ点数: {filteredData.length}件
          </Typography>
          {filteredData.length > 0 && (
            <Typography variant="caption" color="text.secondary">
              平均作業時間: {(filteredData.reduce((sum, entry) => sum + entry.duration, 0) / filteredData.length / 60).toFixed(1)}分
            </Typography>
          )}
        </Box>
      </CardContent>
    </Card>
  );
};

export default TimeChart;