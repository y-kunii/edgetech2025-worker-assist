import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import ProgressChart from '../renderer/components/ProgressChart';

// Mock Chart.js
const mockChart = {
  destroy: jest.fn(),
  update: jest.fn()
};

jest.mock('chart.js', () => {
  const mockChartJS = jest.fn().mockImplementation(() => mockChart);
  mockChartJS.register = jest.fn();
  
  return {
    Chart: mockChartJS,
    ArcElement: {},
    Tooltip: {},
    Legend: {},
    ChartOptions: {},
    Plugin: {}
  };
});

describe('ProgressChart', () => {
  beforeEach(() => {
    // Mock canvas context
    HTMLCanvasElement.prototype.getContext = jest.fn(() => ({
      clearRect: jest.fn(),
      fillRect: jest.fn(),
      beginPath: jest.fn(),
      arc: jest.fn(),
      stroke: jest.fn(),
      save: jest.fn(),
      restore: jest.fn()
    })) as any;
  });

  it('renders screw progress chart with correct title', () => {
    render(
      <ProgressChart 
        type="screw" 
        current={3} 
        target={5} 
        isThresholdReached={false} 
      />
    );
    
    expect(screen.getByText('🔩 ネジ締め進捗')).toBeInTheDocument();
    expect(screen.getByText('3/5 完了')).toBeInTheDocument();
    expect(screen.getByText('🎯 目標まで2回')).toBeInTheDocument();
  });

  it('renders bolt progress chart with correct title', () => {
    render(
      <ProgressChart 
        type="bolt" 
        current={1} 
        target={3} 
        isThresholdReached={false} 
      />
    );
    
    expect(screen.getByText('🔧 ボルト締め進捗')).toBeInTheDocument();
    expect(screen.getByText('1/3 完了')).toBeInTheDocument();
    expect(screen.getByText('🎯 目標まで2回')).toBeInTheDocument();
  });

  it('shows achievement message when threshold is reached', () => {
    render(
      <ProgressChart 
        type="screw" 
        current={5} 
        target={5} 
        isThresholdReached={true} 
      />
    );
    
    expect(screen.getByText('🎉 目標達成！')).toBeInTheDocument();
    expect(screen.queryByText(/目標まで/)).not.toBeInTheDocument();
  });

  it('calculates percentage correctly', () => {
    render(
      <ProgressChart 
        type="screw" 
        current={3} 
        target={5} 
        isThresholdReached={false} 
      />
    );
    
    expect(screen.getByText('60%')).toBeInTheDocument();
  });

  it('handles zero target gracefully', () => {
    render(
      <ProgressChart 
        type="screw" 
        current={0} 
        target={0} 
        isThresholdReached={false} 
      />
    );
    
    expect(screen.getByText('0%')).toBeInTheDocument();
    expect(screen.getByText('0/0 完了')).toBeInTheDocument();
  });

  it('applies threshold-reached class when threshold is reached', () => {
    const { container } = render(
      <ProgressChart 
        type="screw" 
        current={5} 
        target={5} 
        isThresholdReached={true} 
      />
    );
    
    const progressArea = container.querySelector('.screw-progress-area');
    expect(progressArea).toHaveClass('threshold-reached');
  });

  it('does not exceed 100% even when current > target', () => {
    render(
      <ProgressChart 
        type="screw" 
        current={7} 
        target={5} 
        isThresholdReached={true} 
      />
    );
    
    expect(screen.getByText('100%')).toBeInTheDocument();
  });
});