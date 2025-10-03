import React from 'react';
import TimelineChart from './TimelineChart';
import StatsDashboard from './StatsDashboard';
import { WorkHistory, WorkStatistics, EfficiencyMetrics } from '../../types';

interface TimelineAndStatsProps {
  workHistory: WorkHistory[];
  statistics: WorkStatistics;
  efficiencyMetrics: EfficiencyMetrics;
  timeRangeMinutes?: number;
  maxDataPoints?: number;
  deviceCapabilities?: Record<string, boolean>;
}

const TimelineAndStats: React.FC<TimelineAndStatsProps> = ({
  workHistory,
  statistics,
  efficiencyMetrics,
  timeRangeMinutes = 30,
  maxDataPoints = 100,
  deviceCapabilities
}) => {
  return (
    <div className="content-area timeline-stats-area">
      <div className="area-header" style={{ marginBottom: '12px', paddingBottom: '8px' }}>
        <span style={{ fontSize: 'calc(max(20px, 20px * var(--font-scale)))', fontWeight: '600' }}>📊 作業履歴タイムチャート & 統計情報</span>
      </div>
      <div className="area-content timeline-stats-content" style={{ minHeight: '240px' }}>
        {/* タイムチャート部分 */}
        <div className="timeline-section">
          <TimelineChart 
            workHistory={workHistory}
            timeRangeMinutes={timeRangeMinutes}
            maxDataPoints={maxDataPoints}
            deviceCapabilities={deviceCapabilities}
          />
        </div>
        
        {/* 統計情報部分 */}
        <div className="stats-section">
          <StatsDashboard 
            statistics={statistics}
            efficiencyMetrics={efficiencyMetrics}
            workHistory={workHistory}
          />
        </div>
      </div>
    </div>
  );
};

export default TimelineAndStats;