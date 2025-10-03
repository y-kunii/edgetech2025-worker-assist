// Chart.jsのモック
jest.mock('chart.js', () => {
  const mockChart = {
    destroy: jest.fn(),
    data: {},
    update: jest.fn()
  };
  
  const mockRegister = jest.fn();
  
  return {
    Chart: Object.assign(jest.fn().mockImplementation(() => mockChart), {
      register: mockRegister
    }),
    CategoryScale: jest.fn(),
    LinearScale: jest.fn(),
    PointElement: jest.fn(),
    LineElement: jest.fn(),
    Title: jest.fn(),
    Tooltip: jest.fn(),
    Legend: jest.fn(),
    TimeScale: jest.fn(),
    register: mockRegister
  };
});

// chartjs-adapter-date-fnsのモック
jest.mock('chartjs-adapter-date-fns', () => ({}));

import React from 'react';
import { render, screen } from '@testing-library/react';
import TimelineChart from '../renderer/components/TimelineChart';
import { WorkHistory } from '../types';

describe('TimelineChart', () => {
  const mockWorkHistory: WorkHistory[] = [
    {
      id: '1',
      timestamp: new Date('2024-01-01T10:00:00Z'),
      workerStatus: 'waiting',
      duration: 5000,
      efficiency: 85
    },
    {
      id: '2',
      timestamp: new Date('2024-01-01T10:01:00Z'),
      workerStatus: 'screw_tightening',
      duration: 10000,
      efficiency: 92
    },
    {
      id: '3',
      timestamp: new Date('2024-01-01T10:02:00Z'),
      workerStatus: 'bolt_tightening',
      duration: 8000,
      efficiency: 88
    }
  ];

  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('コンポーネントがレンダリングされる', () => {
    render(<TimelineChart workHistory={[]} />);
    
    // コンポーネントがレンダリングされることを確認
    expect(screen.getByText('タイムチャートを読み込み中...')).toBeInTheDocument();
  });

  it('作業履歴データでレンダリングされる', () => {
    render(<TimelineChart workHistory={mockWorkHistory} />);
    
    // コンポーネントがレンダリングされることを確認
    expect(screen.getByText('タイムチャートを読み込み中...')).toBeInTheDocument();
  });

  it('カスタム設定でレンダリングされる', () => {
    render(
      <TimelineChart 
        workHistory={mockWorkHistory}
        timeRangeMinutes={60}
        maxDataPoints={50}
      />
    );
    
    // コンポーネントがレンダリングされることを確認
    expect(screen.getByText('タイムチャートを読み込み中...')).toBeInTheDocument();
  });

  it('プロパティが正しく渡される', () => {
    const { container } = render(
      <TimelineChart 
        workHistory={mockWorkHistory}
        timeRangeMinutes={45}
        maxDataPoints={75}
      />
    );
    
    // コンポーネントがレンダリングされることを確認
    expect(container.querySelector('.timeline-chart-container, .timeline-chart-loading')).toBeInTheDocument();
  });

  it('空の履歴でもエラーが発生しない', () => {
    expect(() => {
      render(<TimelineChart workHistory={[]} />);
    }).not.toThrow();
  });

  it('大量のデータでもエラーが発生しない', () => {
    const manyDataPoints: WorkHistory[] = Array.from({ length: 150 }, (_, i) => ({
      id: `${i}`,
      timestamp: new Date(Date.now() - i * 1000),
      workerStatus: 'screw_tightening',
      duration: 5000,
      efficiency: 85
    }));

    expect(() => {
      render(
        <TimelineChart 
          workHistory={manyDataPoints}
          maxDataPoints={100}
        />
      );
    }).not.toThrow();
  });
});