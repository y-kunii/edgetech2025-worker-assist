import React, { useEffect, useRef, useState } from 'react';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  TimeScale,
  ChartOptions,
  ScatterDataPoint,
  LineController
} from 'chart.js';
import 'chartjs-adapter-date-fns';
import { WorkHistory, WorkerStatus } from '../../types';

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  TimeScale,
  LineController
);

interface TimelineChartProps {
  workHistory: WorkHistory[];
  maxDataPoints?: number;
  timeRangeMinutes?: number;
  deviceCapabilities?: Record<string, boolean>;
}

const TimelineChart: React.FC<TimelineChartProps> = ({ 
  workHistory, 
  maxDataPoints = 100,
  timeRangeMinutes = 30,
  deviceCapabilities
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const chartRef = useRef<ChartJS<'line'> | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // 作業種別ごとの色定義（シンプル版）
  const statusColors: Record<WorkerStatus, { background: string; border: string; label: string }> = {
    waiting: { 
      background: '#6C757D', 
      border: '#6C757D', 
      label: '待機' 
    },
    screw_tightening: { 
      background: '#007BFF', 
      border: '#007BFF', 
      label: 'ネジ締め' 
    },
    bolt_tightening: { 
      background: '#28A745', 
      border: '#28A745', 
      label: 'ボルト締め' 
    },
    tool_handover: { 
      background: '#FFC107', 
      border: '#FFC107', 
      label: '工具受け渡し' 
    },
    absent: { 
      background: '#DC3545', 
      border: '#DC3545', 
      label: '不在' 
    }
  };

  // データを時系列チャート用に変換
  const prepareChartData = () => {
    if (!workHistory.length) {
      return { datasets: [] };
    }

    // 現在時刻から指定時間範囲内のデータのみを取得
    const now = new Date();
    const timeRangeMs = timeRangeMinutes * 60 * 1000;
    const cutoffTime = new Date(now.getTime() - timeRangeMs);
    
    // 最新のデータを取得し、時間範囲でフィルタリング
    const recentHistory = workHistory
      .filter(entry => entry.timestamp >= cutoffTime)
      .slice(-maxDataPoints)
      .sort((a, b) => a.timestamp.getTime() - b.timestamp.getTime());

    // 作業種別ごとにデータセットを作成
    const datasets = Object.entries(statusColors).map(([status, config]) => {
      const statusData = recentHistory
        .filter(entry => entry.workerStatus === status)
        .map(entry => ({
          x: entry.timestamp.getTime(),
          y: getStatusYValue(entry.workerStatus),
          duration: entry.duration,
          efficiency: entry.efficiency,
          id: entry.id
        }));

      return {
        label: config.label,
        data: statusData,
        backgroundColor: config.background,
        borderColor: config.border,
        borderWidth: 2,
        pointRadius: 4,
        pointHoverRadius: 6,
        tension: 0,
        fill: false,
        showLine: true
      };
    });

    return { datasets };
  };

  // 作業種別をY軸の値に変換
  const getStatusYValue = (status: WorkerStatus): number => {
    const statusOrder: Record<WorkerStatus, number> = {
      absent: 0,
      waiting: 1,
      tool_handover: 2,
      screw_tightening: 3,
      bolt_tightening: 4
    };
    return statusOrder[status] || 0;
  };

  // Y軸の値を作業種別ラベルに変換
  const getStatusLabel = (value: number): string => {
    const labels = ['不在', '待機', '工具受け渡し', 'ネジ締め', 'ボルト締め'];
    return labels[value] || '不明';
  };

  useEffect(() => {
    if (!canvasRef.current) return;

    const ctx = canvasRef.current.getContext('2d');
    if (!ctx) return;

    // 既存のチャートを破棄
    if (chartRef.current) {
      chartRef.current.destroy();
    }

    const data = prepareChartData();

    const options: ChartOptions<'line'> = {
      responsive: true,
      maintainAspectRatio: false,
      interaction: {
        mode: 'nearest',
        axis: 'x',
        intersect: false
      },
      plugins: {
        title: {
          display: false
        },
        legend: {
          display: true,
          position: 'top',
          labels: {
            usePointStyle: false,
            padding: 12,
            font: {
              size: 14
            },
            color: '#343A40'
          }
        },
        tooltip: {
          backgroundColor: 'rgba(52, 58, 64, 0.9)',
          titleColor: '#ffffff',
          bodyColor: '#ffffff',
          borderColor: '#DEE2E6',
          borderWidth: 1,
          callbacks: {
            title: (context) => {
              const point = context[0];
              const timestamp = new Date(point.parsed.x);
              return timestamp.toLocaleTimeString('ja-JP');
            },
            label: (context) => {
              const point = context.raw as any;
              const status = getStatusLabel(point.y);
              const duration = point.duration ? `${Math.round(point.duration / 1000)}秒` : '不明';
              const efficiency = point.efficiency ? `${point.efficiency}%` : '不明';
              
              return [
                `作業: ${status}`,
                `継続時間: ${duration}`,
                `効率: ${efficiency}`
              ];
            }
          }
        }
      },
      scales: {
        x: {
          type: 'time',
          time: {
            displayFormats: {
              minute: 'HH:mm',
              hour: 'HH:mm'
            },
            tooltipFormat: 'yyyy/MM/dd HH:mm:ss'
          },
          title: {
            display: true,
            text: '時刻',
            font: {
              size: 14
            },
            color: '#343A40'
          },
          grid: {
            color: '#DEE2E6',
            lineWidth: 1
          },
          ticks: {
            color: '#6C757D',
            font: {
              size: 12
            }
          }
        },
        y: {
          min: -0.5,
          max: 4.5,
          ticks: {
            stepSize: 1,
            callback: function(value) {
              return getStatusLabel(Number(value));
            },
            font: {
              size: 12
            },
            color: '#6C757D'
          },
          title: {
            display: true,
            text: '作業種別',
            font: {
              size: 14
            },
            color: '#343A40'
          },
          grid: {
            color: '#DEE2E6',
            lineWidth: 1
          }
        }
      },
      animation: {
        duration: 200,
        easing: 'linear'
      },
      elements: {
        point: {
          hoverBorderWidth: 2
        },
        line: {
          borderJoinStyle: 'miter'
        }
      }
    };

    chartRef.current = new ChartJS(ctx, {
      type: 'line',
      data,
      options
    });

    setIsLoading(false);

    return () => {
      if (chartRef.current) {
        chartRef.current.destroy();
      }
    };
  }, [workHistory, maxDataPoints, timeRangeMinutes, deviceCapabilities]);

  // リアルタイムデータ更新
  useEffect(() => {
    if (!chartRef.current || isLoading) return;

    const data = prepareChartData();
    chartRef.current.data = data;
    // Use animation only on high-performance devices for real-time updates
    const updateMode = deviceCapabilities?.highPerformance ? 'active' : 'none';
    chartRef.current.update(updateMode);
  }, [workHistory]);

  if (isLoading) {
    return (
      <div className="timeline-chart-loading">
        <div className="loading-spinner"></div>
        <div>タイムチャートを読み込み中...</div>
      </div>
    );
  }

  return (
    <div className="timeline-chart-container">
      <canvas 
        ref={canvasRef}
        style={{ 
          width: '100%', 
          height: '100%',
          minHeight: '200px'
        }}
      />
      
      {workHistory.length === 0 && (
        <div className="timeline-chart-empty">
          <div className="empty-text">作業履歴がありません</div>
          <div className="empty-subtext">作業が開始されるとここにタイムチャートが表示されます</div>
        </div>
      )}
      
      <div className="timeline-chart-info">
        <div className="info-item">
          <span className="info-label">表示範囲:</span>
          <span className="info-value">{timeRangeMinutes}分</span>
        </div>
        <div className="info-item">
          <span className="info-label">データ点数:</span>
          <span className="info-value">{workHistory.length}</span>
        </div>
        <div className="info-item">
          <span className="info-label">最終更新:</span>
          <span className="info-value">
            {workHistory.length > 0 
              ? workHistory[workHistory.length - 1].timestamp.toLocaleTimeString('ja-JP')
              : '未更新'
            }
          </span>
        </div>
      </div>
    </div>
  );
};

export default TimelineChart;