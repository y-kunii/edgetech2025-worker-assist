import React from 'react';
import { WorkStatistics, EfficiencyMetrics, WorkHistory } from '../../types';

interface StatsDashboardProps {
  statistics: WorkStatistics;
  efficiencyMetrics: EfficiencyMetrics;
  workHistory: WorkHistory[];
}

const StatsDashboard: React.FC<StatsDashboardProps> = ({
  statistics,
  efficiencyMetrics,
  workHistory
}) => {
  // 累積作業時間を時間:分形式に変換
  const formatWorkTime = (minutes: number): string => {
    if (minutes < 1) {
      return '0分';
    }
    
    const hours = Math.floor(minutes / 60);
    const mins = Math.round(minutes % 60);
    
    if (hours > 0) {
      return `${hours}時間${mins > 0 ? `${mins}分` : ''}`;
    }
    return `${mins}分`;
  };

  // 効率のトレンド表示（シンプル版）
  const getTrendIcon = (trend: 'up' | 'down' | 'stable'): string => {
    switch (trend) {
      case 'up': return '↑';
      case 'down': return '↓';
      case 'stable': return '→';
      default: return '→';
    }
  };

  // 効率の色を取得
  const getEfficiencyColor = (efficiency: number, target: number): string => {
    if (efficiency >= target) return 'var(--color-success)';
    if (efficiency >= target * 0.8) return 'var(--color-warning)';
    return 'var(--color-error)';
  };

  // 最近の作業効率を計算（直近10件の平均）
  const getRecentEfficiency = (): number => {
    if (workHistory.length === 0) return 0;
    
    const recentEntries = workHistory.slice(-10);
    const sum = recentEntries.reduce((total, entry) => total + entry.efficiency, 0);
    return Math.round(sum / recentEntries.length);
  };

  // 作業種別ごとの統計を計算
  const getWorkTypeStats = () => {
    const stats = {
      screw_tightening: { count: 0, totalTime: 0 },
      bolt_tightening: { count: 0, totalTime: 0 },
      tool_handover: { count: 0, totalTime: 0 },
      waiting: { count: 0, totalTime: 0 },
      absent: { count: 0, totalTime: 0 }
    };

    workHistory.forEach(entry => {
      if (stats[entry.workerStatus]) {
        stats[entry.workerStatus].count++;
        stats[entry.workerStatus].totalTime += entry.duration;
      }
    });

    return stats;
  };

  const workTypeStats = getWorkTypeStats();
  const recentEfficiency = getRecentEfficiency();

  return (
    <div className="stats-dashboard">
      {/* メイン統計情報 */}
      <div className="main-stats">
        <div className="stat-card primary">
          <div className="stat-content">
            <div className="stat-value" style={{ fontSize: 'calc(max(24px, 24px * var(--font-scale)))' }}>{formatWorkTime(statistics.totalWorkTime)}</div>
            <div className="stat-label" style={{ fontSize: 'calc(max(16px, 16px * var(--font-scale)))' }}>総作業時間</div>
          </div>
        </div>

        <div className="stat-card success">
          <div className="stat-content">
            <div className="stat-value" style={{ fontSize: 'calc(max(28px, 28px * var(--font-scale)))' }}>{statistics.completedTasks}件</div>
            <div className="stat-label" style={{ fontSize: 'calc(max(16px, 16px * var(--font-scale)))' }}>完了タスク</div>
          </div>
        </div>

        <div className="stat-card efficiency">
          <div className="stat-content">
            <div 
              className="stat-value"
              style={{ 
                color: getEfficiencyColor(statistics.averageEfficiency, efficiencyMetrics.target),
                fontSize: 'calc(max(28px, 28px * var(--font-scale)))'
              }}
            >
              {statistics.averageEfficiency.toFixed(1)}%
            </div>
            <div className="stat-label" style={{ fontSize: 'calc(max(16px, 16px * var(--font-scale)))' }}>平均効率</div>
          </div>
        </div>

        <div className="stat-card error">
          <div className="stat-content">
            <div className="stat-value" style={{ fontSize: 'calc(max(28px, 28px * var(--font-scale)))' }}>{statistics.errorCount}件</div>
            <div className="stat-label" style={{ fontSize: 'calc(max(16px, 16px * var(--font-scale)))' }}>エラー発生</div>
          </div>
        </div>
      </div>

      {/* 効率指標 */}
      <div className="efficiency-section" style={{ padding: '18px' }}>
        <div className="section-header">
          <h4 style={{ fontSize: 'calc(max(18px, 18px * var(--font-scale)))' }}>効率指標</h4>
          <div className="trend-indicator" style={{ fontSize: 'calc(max(14px, 14px * var(--font-scale)))' }}>
            <span className="trend-icon" style={{ fontSize: 'calc(max(18px, 18px * var(--font-scale)))' }}>{getTrendIcon(efficiencyMetrics.trend)}</span>
            <span className="trend-text">
              {efficiencyMetrics.trend === 'up' && '向上中'}
              {efficiencyMetrics.trend === 'down' && '低下中'}
              {efficiencyMetrics.trend === 'stable' && '安定'}
            </span>
          </div>
        </div>

        <div className="efficiency-bars">
          <div className="efficiency-bar">
            <div className="bar-label">
              <span>現在の効率</span>
              <span className="bar-value">{efficiencyMetrics.current.toFixed(1)}%</span>
            </div>
            <div className="bar-track">
              <div 
                className="bar-fill current"
                style={{ 
                  width: `${Math.min(efficiencyMetrics.current, 100)}%`,
                  backgroundColor: getEfficiencyColor(efficiencyMetrics.current, efficiencyMetrics.target)
                }}
              />
            </div>
          </div>

          <div className="efficiency-bar">
            <div className="bar-label">
              <span>目標効率</span>
              <span className="bar-value">{efficiencyMetrics.target.toFixed(1)}%</span>
            </div>
            <div className="bar-track">
              <div 
                className="bar-fill target"
                style={{ width: `${Math.min(efficiencyMetrics.target, 100)}%` }}
              />
            </div>
          </div>

          <div className="efficiency-bar">
            <div className="bar-label">
              <span>直近効率</span>
              <span className="bar-value">{recentEfficiency.toFixed(1)}%</span>
            </div>
            <div className="bar-track">
              <div 
                className="bar-fill recent"
                style={{ 
                  width: `${Math.min(recentEfficiency, 100)}%`,
                  backgroundColor: getEfficiencyColor(recentEfficiency, efficiencyMetrics.target)
                }}
              />
            </div>
          </div>
        </div>
      </div>

      {/* 作業種別統計 */}
      <div className="work-type-section" style={{ padding: '18px' }}>
        <div className="section-header">
          <h4 style={{ fontSize: 'calc(max(18px, 18px * var(--font-scale)))' }}>作業種別統計</h4>
        </div>

        <div className="work-type-stats">
          <div className="work-type-item" style={{ padding: '14px' }}>
            <div className="work-type-info">
              <div className="work-type-name" style={{ fontSize: 'calc(max(16px, 16px * var(--font-scale)))' }}>ネジ締め</div>
              <div className="work-type-details" style={{ fontSize: 'calc(max(14px, 14px * var(--font-scale)))' }}>
                <span>{workTypeStats.screw_tightening.count}回</span>
                <span>・</span>
                <span>{formatWorkTime(workTypeStats.screw_tightening.totalTime / (1000 * 60))}</span>
              </div>
            </div>
          </div>

          <div className="work-type-item" style={{ padding: '14px' }}>
            <div className="work-type-info">
              <div className="work-type-name" style={{ fontSize: 'calc(max(16px, 16px * var(--font-scale)))' }}>ボルト締め</div>
              <div className="work-type-details" style={{ fontSize: 'calc(max(14px, 14px * var(--font-scale)))' }}>
                <span>{workTypeStats.bolt_tightening.count}回</span>
                <span>・</span>
                <span>{formatWorkTime(workTypeStats.bolt_tightening.totalTime / (1000 * 60))}</span>
              </div>
            </div>
          </div>

          <div className="work-type-item" style={{ padding: '14px' }}>
            <div className="work-type-info">
              <div className="work-type-name" style={{ fontSize: 'calc(max(16px, 16px * var(--font-scale)))' }}>工具受け渡し</div>
              <div className="work-type-details" style={{ fontSize: 'calc(max(14px, 14px * var(--font-scale)))' }}>
                <span>{workTypeStats.tool_handover.count}回</span>
                <span>・</span>
                <span>{formatWorkTime(workTypeStats.tool_handover.totalTime / (1000 * 60))}</span>
              </div>
            </div>
          </div>

          <div className="work-type-item" style={{ padding: '14px' }}>
            <div className="work-type-info">
              <div className="work-type-name" style={{ fontSize: 'calc(max(16px, 16px * var(--font-scale)))' }}>待機</div>
              <div className="work-type-details" style={{ fontSize: 'calc(max(14px, 14px * var(--font-scale)))' }}>
                <span>{workTypeStats.waiting.count}回</span>
                <span>・</span>
                <span>{formatWorkTime(workTypeStats.waiting.totalTime / (1000 * 60))}</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* 最終更新時刻 */}
      <div className="stats-footer">
        <div className="last-updated">
          最終更新: {efficiencyMetrics.lastUpdated.toLocaleString('ja-JP')}
        </div>
      </div>
    </div>
  );
};

export default StatsDashboard;