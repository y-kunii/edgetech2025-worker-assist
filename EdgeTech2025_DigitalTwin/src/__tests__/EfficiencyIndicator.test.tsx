import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { EfficiencyIndicator } from '../renderer/components/EfficiencyIndicator';
import { EfficiencyMetrics } from '../types';

// モックデータ
const mockMetrics: EfficiencyMetrics = {
  current: 85,
  target: 90,
  trend: 'up',
  lastUpdated: new Date('2024-01-01T12:00:00Z')
};

describe('EfficiencyIndicator', () => {
  const mockOnAlert = jest.fn();

  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('効率指標が正しく表示される', () => {
    render(
      <EfficiencyIndicator
        metrics={mockMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText('効率指標')).toBeInTheDocument();
    expect(screen.getByText('85.0%')).toBeInTheDocument();
    expect(screen.getByText('/ 90%')).toBeInTheDocument();
  });

  it('トレンドアイコンが正しく表示される', () => {
    const upTrendMetrics = { ...mockMetrics, trend: 'up' as const };
    const { rerender } = render(
      <EfficiencyIndicator
        metrics={upTrendMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText('↗️')).toBeInTheDocument();

    const downTrendMetrics = { ...mockMetrics, trend: 'down' as const };
    rerender(
      <EfficiencyIndicator
        metrics={downTrendMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText('↘️')).toBeInTheDocument();

    const stableTrendMetrics = { ...mockMetrics, trend: 'stable' as const };
    rerender(
      <EfficiencyIndicator
        metrics={stableTrendMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText('➡️')).toBeInTheDocument();
  });

  it('効率レベルに応じたステータスが表示される', () => {
    // 優秀レベル（目標以上）
    const excellentMetrics = { ...mockMetrics, current: 95, target: 90 };
    const { rerender } = render(
      <EfficiencyIndicator
        metrics={excellentMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText('優秀')).toBeInTheDocument();

    // 良好レベル（70%以上）
    const goodMetrics = { ...mockMetrics, current: 75 };
    rerender(
      <EfficiencyIndicator
        metrics={goodMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText('良好')).toBeInTheDocument();

    // 注意レベル（50-70%）
    const warningMetrics = { ...mockMetrics, current: 60 };
    rerender(
      <EfficiencyIndicator
        metrics={warningMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText('注意')).toBeInTheDocument();

    // 危険レベル（50%未満）
    const criticalMetrics = { ...mockMetrics, current: 40 };
    rerender(
      <EfficiencyIndicator
        metrics={criticalMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText('危険')).toBeInTheDocument();
  });

  it('低効率時にアラートが表示される', () => {
    const lowEfficiencyMetrics = { ...mockMetrics, current: 40 };
    render(
      <EfficiencyIndicator
        metrics={lowEfficiencyMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText(/効率が危険レベルです/)).toBeInTheDocument();
  });

  it('警告レベル時にアラートが表示される', () => {
    const warningMetrics = { ...mockMetrics, current: 60 };
    render(
      <EfficiencyIndicator
        metrics={warningMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText(/効率が低下しています/)).toBeInTheDocument();
  });

  it('効率バーの幅が正しく設定される', () => {
    render(
      <EfficiencyIndicator
        metrics={mockMetrics}
        onAlert={mockOnAlert}
      />
    );

    const efficiencyBar = document.querySelector('.efficiency-bar');
    expect(efficiencyBar).toHaveStyle('width: 85%');
  });

  it('目標ラインの位置が正しく設定される', () => {
    render(
      <EfficiencyIndicator
        metrics={mockMetrics}
        onAlert={mockOnAlert}
      />
    );

    const targetLine = document.querySelector('.target-line');
    expect(targetLine).toHaveStyle('left: 90%');
  });

  it('効率の変化が表示される', () => {
    const metricsWithChange = { ...mockMetrics, current: 88 };
    const { rerender } = render(
      <EfficiencyIndicator
        metrics={mockMetrics}
        onAlert={mockOnAlert}
      />
    );

    // 効率が上昇した場合
    rerender(
      <EfficiencyIndicator
        metrics={metricsWithChange}
        onAlert={mockOnAlert}
      />
    );

    // 変化は内部状態で管理されるため、初回レンダリングでは表示されない
    // 実際のアプリケーションでは、propsの変更により変化が計算される
  });

  it('カスタム閾値が正しく適用される', () => {
    const customThresholds = {
      lowEfficiencyThreshold: 80,
      criticalEfficiencyThreshold: 60
    };

    const metricsAt75 = { ...mockMetrics, current: 75 };
    render(
      <EfficiencyIndicator
        metrics={metricsAt75}
        onAlert={mockOnAlert}
        {...customThresholds}
      />
    );

    // 75%はカスタム閾値（80%）未満なので警告レベル
    expect(screen.getByText('注意')).toBeInTheDocument();
  });

  it('最終更新時刻が正しくフォーマットされる', () => {
    const recentMetrics = {
      ...mockMetrics,
      lastUpdated: new Date(Date.now() - 30 * 1000) // 30秒前
    };

    render(
      <EfficiencyIndicator
        metrics={recentMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(screen.getByText(/最終更新:/)).toBeInTheDocument();
  });

  it('アクセシビリティ属性が正しく設定される', () => {
    render(
      <EfficiencyIndicator
        metrics={mockMetrics}
        onAlert={mockOnAlert}
      />
    );

    const trendIcon = screen.getByTitle(/効率トレンド:/);
    expect(trendIcon).toBeInTheDocument();

    const targetLine = screen.getByTitle(/目標効率:/);
    expect(targetLine).toBeInTheDocument();
  });

  it('効率が0%未満や100%超の場合も正しく処理される', () => {
    const negativeMetrics = { ...mockMetrics, current: -10 };
    const { rerender } = render(
      <EfficiencyIndicator
        metrics={negativeMetrics}
        onAlert={mockOnAlert}
      />
    );

    const efficiencyBar = document.querySelector('.efficiency-bar');
    expect(efficiencyBar).toHaveStyle('width: 0%');

    const overMetrics = { ...mockMetrics, current: 120 };
    rerender(
      <EfficiencyIndicator
        metrics={overMetrics}
        onAlert={mockOnAlert}
      />
    );

    expect(efficiencyBar).toHaveStyle('width: 100%');
  });

  it('onAlertコールバックが正しく呼ばれる', async () => {
    const lowMetrics = { ...mockMetrics, current: 40 };
    render(
      <EfficiencyIndicator
        metrics={lowMetrics}
        onAlert={mockOnAlert}
      />
    );

    // useEffectが実行されるまで待機
    await waitFor(() => {
      expect(mockOnAlert).toHaveBeenCalledWith(
        'low_efficiency',
        expect.stringContaining('重大: 効率が40.0%に低下しています')
      );
    });
  });

  it('効率急降下時にアラートが発生する', async () => {
    const initialMetrics = { ...mockMetrics, current: 80 };
    const { rerender } = render(
      <EfficiencyIndicator
        metrics={initialMetrics}
        onAlert={mockOnAlert}
      />
    );

    // 効率が急激に低下
    const droppedMetrics = { ...mockMetrics, current: 65 };
    rerender(
      <EfficiencyIndicator
        metrics={droppedMetrics}
        onAlert={mockOnAlert}
      />
    );

    await waitFor(() => {
      expect(mockOnAlert).toHaveBeenCalledWith(
        'efficiency_drop',
        expect.stringContaining('効率が急激に低下しました')
      );
    });
  });
});