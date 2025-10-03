import React, { useEffect, useState, useMemo } from 'react';
import { EfficiencyMetrics } from '../../types';
import './EfficiencyIndicator.css';

interface EfficiencyIndicatorProps {
  metrics: EfficiencyMetrics;
  onAlert?: (alertType: 'low_efficiency' | 'efficiency_drop', message: string) => void;
  lowEfficiencyThreshold?: number;
  criticalEfficiencyThreshold?: number;
}

interface TrendIconProps {
  trend: 'up' | 'down' | 'stable';
  size?: 'small' | 'medium' | 'large';
}

const TrendIcon: React.FC<TrendIconProps> = ({ trend, size = 'medium' }) => {
  const getIcon = () => {
    switch (trend) {
      case 'up': return '↗️';
      case 'down': return '↘️';
      case 'stable': return '➡️';
      default: return '➡️';
    }
  };

  const getColor = () => {
    switch (trend) {
      case 'up': return '#4CAF50';
      case 'down': return '#F44336';
      case 'stable': return '#FF9800';
      default: return '#999';
    }
  };

  return (
    <span 
      className={`trend-icon trend-${trend} trend-${size}`}
      style={{ color: getColor() }}
      title={`効率トレンド: ${trend === 'up' ? '向上' : trend === 'down' ? '低下' : '安定'}`}
    >
      {getIcon()}
    </span>
  );
};

export const EfficiencyIndicator: React.FC<EfficiencyIndicatorProps> = ({
  metrics,
  onAlert,
  lowEfficiencyThreshold = 70,
  criticalEfficiencyThreshold = 50
}) => {
  const [previousEfficiency, setPreviousEfficiency] = useState<number>(metrics.current);
  const [alertShown, setAlertShown] = useState<Set<string>>(new Set());

  // 効率レベルの判定
  const efficiencyLevel = useMemo(() => {
    if (metrics.current >= metrics.target) return 'excellent';
    if (metrics.current >= lowEfficiencyThreshold) return 'good';
    if (metrics.current >= criticalEfficiencyThreshold) return 'warning';
    return 'critical';
  }, [metrics.current, metrics.target, lowEfficiencyThreshold, criticalEfficiencyThreshold]);

  // 効率の変化量を計算
  const efficiencyChange = useMemo(() => {
    return metrics.current - previousEfficiency;
  }, [metrics.current, previousEfficiency]);

  // アラート処理
  useEffect(() => {
    if (!onAlert) return;

    const currentTime = Date.now();
    const alertKey = `${efficiencyLevel}_${Math.floor(currentTime / 60000)}`; // 1分間隔でアラート

    // 低効率アラート
    if (metrics.current < lowEfficiencyThreshold && !alertShown.has(alertKey)) {
      const severity = metrics.current < criticalEfficiencyThreshold ? '重大' : '警告';
      onAlert(
        'low_efficiency',
        `${severity}: 効率が${metrics.current.toFixed(1)}%に低下しています（目標: ${metrics.target}%）`
      );
      setAlertShown(prev => new Set(prev).add(alertKey));
    }

    // 効率急降下アラート
    const dropThreshold = 10; // 10%以上の急降下
    if (efficiencyChange < -dropThreshold && !alertShown.has(`drop_${alertKey}`)) {
      onAlert(
        'efficiency_drop',
        `効率が急激に低下しました（${efficiencyChange.toFixed(1)}%の変化）`
      );
      setAlertShown(prev => new Set(prev).add(`drop_${alertKey}`));
    }

    // 前回の効率を更新
    setPreviousEfficiency(metrics.current);
  }, [metrics.current, metrics.target, lowEfficiencyThreshold, criticalEfficiencyThreshold, efficiencyChange, onAlert, alertShown]);

  // 効率バーの色を取得
  const getEfficiencyColor = () => {
    switch (efficiencyLevel) {
      case 'excellent': return '#4CAF50';
      case 'good': return '#8BC34A';
      case 'warning': return '#FF9800';
      case 'critical': return '#F44336';
      default: return '#999';
    }
  };

  // 効率バーの幅を計算
  const getEfficiencyWidth = () => {
    return Math.min(Math.max(metrics.current, 0), 100);
  };

  // 目標ラインの位置を計算
  const getTargetPosition = () => {
    return Math.min(Math.max(metrics.target, 0), 100);
  };

  // 最終更新時刻のフォーマット
  const formatLastUpdated = (date: Date) => {
    const now = new Date();
    const diff = now.getTime() - date.getTime();
    const seconds = Math.floor(diff / 1000);
    
    if (seconds < 60) return `${seconds}秒前`;
    
    const minutes = Math.floor(seconds / 60);
    if (minutes < 60) return `${minutes}分前`;
    
    const hours = Math.floor(minutes / 60);
    return `${hours}時間前`;
  };

  return (
    <div className={`efficiency-indicator efficiency-${efficiencyLevel}`}>
      <div className="efficiency-header">
        <h3 className="efficiency-title">
          効率指標
          <TrendIcon trend={metrics.trend} size="medium" />
        </h3>
        <div className="efficiency-value">
          <span className="current-efficiency">{metrics.current.toFixed(1)}%</span>
          <span className="target-efficiency">/ {metrics.target}%</span>
        </div>
      </div>

      <div className="efficiency-bar-container">
        <div className="efficiency-bar-background">
          {/* 効率バー */}
          <div 
            className="efficiency-bar"
            style={{
              width: `${getEfficiencyWidth()}%`,
              backgroundColor: getEfficiencyColor()
            }}
          />
          
          {/* 目標ライン */}
          <div 
            className="target-line"
            style={{ left: `${getTargetPosition()}%` }}
            title={`目標効率: ${metrics.target}%`}
          />
        </div>
        
        {/* スケール */}
        <div className="efficiency-scale">
          <span>0%</span>
          <span>25%</span>
          <span>50%</span>
          <span>75%</span>
          <span>100%</span>
        </div>
      </div>

      <div className="efficiency-details">
        <div className="efficiency-status">
          <span className={`status-indicator status-${efficiencyLevel}`}>
            {efficiencyLevel === 'excellent' && '優秀'}
            {efficiencyLevel === 'good' && '良好'}
            {efficiencyLevel === 'warning' && '注意'}
            {efficiencyLevel === 'critical' && '危険'}
          </span>
          
          {efficiencyChange !== 0 && (
            <span className={`efficiency-change ${efficiencyChange > 0 ? 'positive' : 'negative'}`}>
              {efficiencyChange > 0 ? '+' : ''}{efficiencyChange.toFixed(1)}%
            </span>
          )}
        </div>
        
        <div className="last-updated">
          最終更新: {formatLastUpdated(metrics.lastUpdated)}
        </div>
      </div>

      {/* アラート表示エリア */}
      {efficiencyLevel === 'critical' && (
        <div className="efficiency-alert critical-alert">
          <span className="alert-icon">⚠️</span>
          効率が危険レベルです。作業プロセスを確認してください。
        </div>
      )}
      
      {efficiencyLevel === 'warning' && (
        <div className="efficiency-alert warning-alert">
          <span className="alert-icon">⚡</span>
          効率が低下しています。改善が必要です。
        </div>
      )}
    </div>
  );
};

export default EfficiencyIndicator;