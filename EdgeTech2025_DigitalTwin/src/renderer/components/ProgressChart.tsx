import React, { useEffect, useRef } from 'react';
import {
  Chart as ChartJS,
  ArcElement,
  Tooltip,
  Legend,
  ChartOptions,
  Plugin,
  DoughnutController
} from 'chart.js';

ChartJS.register(ArcElement, Tooltip, Legend, DoughnutController);

interface ProgressChartProps {
  type: 'screw' | 'bolt';
  current: number;
  target: number;
  isThresholdReached: boolean;
  animationManager?: any;
  deviceCapabilities?: Record<string, boolean>;
}

const ProgressChart: React.FC<ProgressChartProps> = ({ 
  type, 
  current, 
  target, 
  isThresholdReached,
  animationManager,
  deviceCapabilities
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const chartRef = useRef<ChartJS<'doughnut'> | null>(null);
  const animationRef = useRef<number | null>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  
  const percentage = target > 0 ? Math.min((current / target) * 100, 100) : 0;
  const remaining = Math.max(target - current, 0);
  const title = type === 'screw' ? 'ネジ締め進捗' : 'ボルト締め進捗';

  // シンプルなプラグイン（エフェクトなし）
  const simplePlugin: Plugin<'doughnut'> = {
    id: 'simple',
    afterDraw: () => {
      // No special effects for clean design
    }
  };

  useEffect(() => {
    if (!canvasRef.current) return;

    const ctx = canvasRef.current.getContext('2d');
    if (!ctx) return;

    // 既存のチャートを破棄
    if (chartRef.current) {
      chartRef.current.destroy();
    }

    const completedValue = percentage;
    const remainingValue = 100 - percentage;

    const data = {
      datasets: [{
        data: [completedValue, remainingValue],
        backgroundColor: [
          isThresholdReached ? '#28A745' : '#007BFF',
          '#F8F9FA'
        ],
        borderColor: [
          isThresholdReached ? '#28A745' : '#007BFF',
          '#DEE2E6'
        ],
        borderWidth: 1,
        cutout: '70%',
        circumference: 360,
        rotation: -90
      }]
    };

    const options: ChartOptions<'doughnut'> = {
      responsive: true,
      maintainAspectRatio: false,
      plugins: {
        legend: {
          display: false
        },
        tooltip: {
          enabled: false
        }
      },
      animation: {
        animateRotate: false,
        animateScale: false,
        duration: 200
      },
      elements: {
        arc: {
          borderRadius: 0
        }
      }
    };

    chartRef.current = new ChartJS(ctx, {
      type: 'doughnut',
      data,
      options,
      plugins: [simplePlugin]
    });

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
      if (chartRef.current) {
        chartRef.current.destroy();
      }
    };
  }, [current, target, isThresholdReached, percentage, animationManager, deviceCapabilities]);

  return (
    <div 
      ref={containerRef}
      className={`content-area ${type === 'screw' ? 'screw-progress-area' : 'bolt-progress-area'} ${isThresholdReached ? 'threshold-reached' : ''}`}
    >
      <div className="area-header">
        {title}
      </div>
      <div className="area-content">
        <div className="progress-chart-container">
          <div className="chart-wrapper">
            <canvas 
              ref={canvasRef}
              width={deviceCapabilities?.highPerformance ? 140 : 120}
              height={deviceCapabilities?.highPerformance ? 140 : 120}
              style={{ 
                maxWidth: deviceCapabilities?.highPerformance ? '140px' : '120px', 
                maxHeight: deviceCapabilities?.highPerformance ? '140px' : '120px' 
              }}
            />
            <div className="chart-center-text">
              <div className="progress-value">
                {current}/{target}
              </div>
              <div className="progress-percentage">
                {Math.round(percentage)}%
              </div>
            </div>
          </div>
        </div>
        
        <div className="progress-info">
          <div className="progress-status">
            {current}/{target} 完了
          </div>
          {!isThresholdReached && (
            <div className="progress-remaining">
              残り: {remaining}回
            </div>
          )}
          {isThresholdReached && (
            <div className="progress-achieved">
              目標達成
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default ProgressChart;