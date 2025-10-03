import React from 'react';
import { render, screen } from '@testing-library/react';
import TimelineAndStats from '../renderer/components/TimelineAndStats';
import { WorkHistory, WorkStatistics, EfficiencyMetrics } from '../types';

// TimelineChartコンポーネントのモック
jest.mock('../renderer/components/TimelineChart', () => {
  return function MockTimelineChart({ workHistory, timeRangeMinutes }: any) {
    return (
      <div data-testid="timeline-chart">
        <div>表示範囲: {timeRangeMinutes}分</div>
        <div>データ点数: {workHistory.length}</div>
        {workHistory.length === 0 && <div>作業履歴がありません</div>}
      </div>
    );
  };
});

// StatsDashboardコンポーネントのモック
jest.mock('../renderer/components/StatsDashboard', () => {
  return function MockStatsDashboard({ statistics }: any) {
    return (
      <div data-testid="stats-dashboard">
        <div>総作業時間</div>
        <div>完了タスク</div>
        <div>平均効率</div>
        <div>エラー発生</div>
        <div>{statistics.completedTasks}件</div>
        <div>{Math.round(statistics.averageEfficiency)}%</div>
      </div>
    );
  };
});

describe('TimelineAndStats', () => {
  const mockWorkHistory: WorkHistory[] = [
    {
      id: '1',
      timestamp: new Date('2024-01-01T10:00:00Z'),
      workerStatus: 'screw_tightening',
      duration: 30000,
      efficiency: 90
    },
    {
      id: '2',
      timestamp: new Date('2024-01-01T10:01:00Z'),
      workerStatus: 'bolt_tightening',
      duration: 25000,
      efficiency: 85
    }
  ];

  const mockStatistics: WorkStatistics = {
    totalWorkTime: 65.5,
    completedTasks: 8,
    averageEfficiency: 87.5,
    errorCount: 1
  };

  const mockEfficiencyMetrics: EfficiencyMetrics = {
    current: 92,
    target: 85,
    trend: 'up',
    lastUpdated: new Date('2024-01-01T10:30:00Z')
  };

  it('コンポーネントが正しくレンダリングされる', () => {
    render(
      <TimelineAndStats 
        workHistory={mockWorkHistory}
        statistics={mockStatistics}
        efficiencyMetrics={mockEfficiencyMetrics}
      />
    );

    // ヘッダーが表示される
    expect(screen.getByText('📊 作業履歴タイムチャート & 統計情報')).toBeInTheDocument();
  });

  it('タイムチャートセクションが表示される', () => {
    render(
      <TimelineAndStats 
        workHistory={mockWorkHistory}
        statistics={mockStatistics}
        efficiencyMetrics={mockEfficiencyMetrics}
      />
    );

    // タイムチャートの情報が表示される
    expect(screen.getByText(/表示範囲:/)).toBeInTheDocument();
    expect(screen.getByText(/30/)).toBeInTheDocument();
    expect(screen.getByText(/データ点数:/)).toBeInTheDocument();
  });

  it('統計ダッシュボードセクションが表示される', () => {
    render(
      <TimelineAndStats 
        workHistory={mockWorkHistory}
        statistics={mockStatistics}
        efficiencyMetrics={mockEfficiencyMetrics}
      />
    );

    // 統計情報が表示される
    expect(screen.getByText('総作業時間')).toBeInTheDocument();
    expect(screen.getByText('完了タスク')).toBeInTheDocument();
    expect(screen.getByText('平均効率')).toBeInTheDocument();
    expect(screen.getByText('エラー発生')).toBeInTheDocument();
  });

  it('カスタム設定が正しく適用される', () => {
    render(
      <TimelineAndStats 
        workHistory={mockWorkHistory}
        statistics={mockStatistics}
        efficiencyMetrics={mockEfficiencyMetrics}
        timeRangeMinutes={60}
        maxDataPoints={50}
      />
    );

    // カスタム時間範囲が適用される
    expect(screen.getByText(/60/)).toBeInTheDocument();
  });

  it('空のデータでも正しく動作する', () => {
    const emptyStatistics: WorkStatistics = {
      totalWorkTime: 0,
      completedTasks: 0,
      averageEfficiency: 0,
      errorCount: 0
    };

    render(
      <TimelineAndStats 
        workHistory={[]}
        statistics={emptyStatistics}
        efficiencyMetrics={mockEfficiencyMetrics}
      />
    );

    // 空の状態メッセージが表示される
    expect(screen.getByText('作業履歴がありません')).toBeInTheDocument();
    
    // 統計情報は0で表示される
    expect(screen.getByText('0件')).toBeInTheDocument();
  });

  it('レスポンシブレイアウトクラスが適用される', () => {
    const { container } = render(
      <TimelineAndStats 
        workHistory={mockWorkHistory}
        statistics={mockStatistics}
        efficiencyMetrics={mockEfficiencyMetrics}
      />
    );

    // 適切なCSSクラスが適用されている
    expect(container.querySelector('.timeline-stats-area')).toBeInTheDocument();
    expect(container.querySelector('.timeline-stats-content')).toBeInTheDocument();
    expect(container.querySelector('.timeline-section')).toBeInTheDocument();
    expect(container.querySelector('.stats-section')).toBeInTheDocument();
  });

  it('プロパティが子コンポーネントに正しく渡される', () => {
    render(
      <TimelineAndStats 
        workHistory={mockWorkHistory}
        statistics={mockStatistics}
        efficiencyMetrics={mockEfficiencyMetrics}
        timeRangeMinutes={45}
        maxDataPoints={75}
      />
    );

    // TimelineChartに渡されるプロパティの確認
    expect(screen.getByText(/45/)).toBeInTheDocument();
    
    // StatsDashboardに渡されるプロパティの確認
    expect(screen.getByText('8件')).toBeInTheDocument(); // completedTasks
    expect(screen.getByText('88%')).toBeInTheDocument(); // averageEfficiency (四捨五入)
  });

  it('デフォルト値が正しく適用される', () => {
    render(
      <TimelineAndStats 
        workHistory={mockWorkHistory}
        statistics={mockStatistics}
        efficiencyMetrics={mockEfficiencyMetrics}
      />
    );

    // デフォルトの時間範囲（30分）が適用される
    expect(screen.getByText(/30/)).toBeInTheDocument();
  });
});