import React from 'react';
import { render, screen, act } from '@testing-library/react';
import WorkStepDisplay from '../renderer/components/WorkStepDisplay';
import { WorkStepManager } from '../utils/workStepManager';
import { WorkerStatus } from '../types';

// Mock the WorkStepManager
jest.mock('../utils/workStepManager');

const MockedWorkStepManager = WorkStepManager as jest.MockedClass<typeof WorkStepManager>;

describe('WorkStepDisplay', () => {
  let mockStepManager: jest.Mocked<WorkStepManager>;

  beforeEach(() => {
    mockStepManager = {
      updateWorkStep: jest.fn(),
      getCurrentTiming: jest.fn(),
      calculateCurrentEfficiency: jest.fn(),
      updateElapsedTime: jest.fn(),
      getStepStatistics: jest.fn(),
      reset: jest.fn()
    } as any;

    // Mock static methods
    MockedWorkStepManager.formatTime = jest.fn((seconds: number) => {
      const mins = Math.floor(seconds / 60);
      const secs = seconds % 60;
      return `${mins}:${secs.toString().padStart(2, '0')}`;
    });

    MockedWorkStepManager.getStepDisplayName = jest.fn((step: WorkerStatus) => {
      switch (step) {
        case 'waiting': return '待機中';
        case 'screw_tightening': return 'ネジ締め作業';
        case 'bolt_tightening': return 'ボルト締め作業';
        case 'tool_handover': return '工具受け渡し';
        case 'absent': return '不在';
        default: return '不明な状態';
      }
    });

    MockedWorkStepManager.getStepIcon = jest.fn((step: WorkerStatus) => {
      switch (step) {
        case 'waiting': return '⏳';
        case 'screw_tightening': return '🔧';
        case 'bolt_tightening': return '🔩';
        case 'tool_handover': return '🤝';
        case 'absent': return '👤';
        default: return '📋';
      }
    });

    MockedWorkStepManager.getEfficiencyColor = jest.fn((efficiency: number) => {
      if (efficiency >= 90) return 'var(--color-success)';
      if (efficiency >= 75) return 'var(--color-primary-blue)';
      if (efficiency >= 50) return 'var(--color-warning)';
      return 'var(--color-error)';
    });

    MockedWorkStepManager.getTrendIcon = jest.fn((trend) => {
      switch (trend) {
        case 'improving': return '📈';
        case 'declining': return '📉';
        case 'stable': return '📊';
      }
    });

    // Set up default mock returns
    mockStepManager.getCurrentTiming.mockReturnValue({
      currentStep: 'waiting',
      stepStartTime: new Date(),
      elapsedTime: 30,
      totalStepTime: 120,
      stepCount: 3
    });

    mockStepManager.calculateCurrentEfficiency.mockReturnValue({
      currentEfficiency: 85,
      averageStepTime: 40,
      expectedStepTime: 45,
      trend: 'stable'
    });
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  it('should render current step information', () => {
    render(
      <WorkStepDisplay 
        currentStep="screw_tightening" 
        stepManager={mockStepManager}
      />
    );

    expect(screen.getByText('📋 現在の作業')).toBeInTheDocument();
    expect(MockedWorkStepManager.getStepDisplayName).toHaveBeenCalledWith('waiting');
    expect(MockedWorkStepManager.getStepIcon).toHaveBeenCalledWith('waiting');
  });

  it('should display elapsed time correctly', () => {
    render(
      <WorkStepDisplay 
        currentStep="screw_tightening" 
        stepManager={mockStepManager}
      />
    );

    expect(screen.getByText('⏱️ 経過時間')).toBeInTheDocument();
    expect(MockedWorkStepManager.formatTime).toHaveBeenCalledWith(30);
  });

  it('should display efficiency with correct color and trend', () => {
    render(
      <WorkStepDisplay 
        currentStep="screw_tightening" 
        stepManager={mockStepManager}
      />
    );

    expect(screen.getByText('📈 作業効率')).toBeInTheDocument();
    expect(screen.getByText('85%')).toBeInTheDocument();
    expect(MockedWorkStepManager.getEfficiencyColor).toHaveBeenCalledWith(85);
    expect(MockedWorkStepManager.getTrendIcon).toHaveBeenCalledWith('stable');
  });

  it('should display step statistics', () => {
    render(
      <WorkStepDisplay 
        currentStep="screw_tightening" 
        stepManager={mockStepManager}
      />
    );

    expect(screen.getByText('実行回数')).toBeInTheDocument();
    expect(screen.getByText('3回')).toBeInTheDocument();
    expect(screen.getByText('平均時間')).toBeInTheDocument();
  });

  it('should show progress bar for expected time', () => {
    render(
      <WorkStepDisplay 
        currentStep="screw_tightening" 
        stepManager={mockStepManager}
      />
    );

    // Should show time comparison text
    const progressElements = screen.getByText(/予定まで|予定より/);
    expect(progressElements).toBeInTheDocument();
  });

  it('should handle absent status correctly', () => {
    mockStepManager.getCurrentTiming.mockReturnValue({
      currentStep: 'absent',
      stepStartTime: new Date(),
      elapsedTime: 0,
      totalStepTime: 0,
      stepCount: 0
    });

    mockStepManager.calculateCurrentEfficiency.mockReturnValue({
      currentEfficiency: 100,
      averageStepTime: 0,
      expectedStepTime: 0,
      trend: 'stable'
    });

    render(
      <WorkStepDisplay 
        currentStep="absent" 
        stepManager={mockStepManager}
      />
    );

    expect(screen.getAllByText('不在')).toHaveLength(2); // Both title and status
  });

  it('should work without step manager (fallback mode)', () => {
    render(
      <WorkStepDisplay 
        currentStep="waiting"
      />
    );

    expect(screen.getByText('📋 現在の作業')).toBeInTheDocument();
    expect(MockedWorkStepManager.getStepDisplayName).toHaveBeenCalledWith('waiting');
  });

  it('should update when step changes', () => {
    const { rerender } = render(
      <WorkStepDisplay 
        currentStep="waiting" 
        stepManager={mockStepManager}
      />
    );

    // Change step
    rerender(
      <WorkStepDisplay 
        currentStep="screw_tightening" 
        stepManager={mockStepManager}
      />
    );

    expect(mockStepManager.updateWorkStep).toHaveBeenCalledWith('screw_tightening');
  });

  it('should handle overtime correctly', () => {
    // Mock overtime scenario
    mockStepManager.getCurrentTiming.mockReturnValue({
      currentStep: 'screw_tightening',
      stepStartTime: new Date(),
      elapsedTime: 60, // 60 seconds elapsed
      totalStepTime: 120,
      stepCount: 2
    });

    mockStepManager.calculateCurrentEfficiency.mockReturnValue({
      currentEfficiency: 75,
      averageStepTime: 50,
      expectedStepTime: 45, // Expected 45, actual 60 = overtime
      trend: 'declining'
    });

    render(
      <WorkStepDisplay 
        currentStep="screw_tightening" 
        stepManager={mockStepManager}
      />
    );

    // Should show overtime message
    expect(screen.getByText(/予定より.*超過/)).toBeInTheDocument();
  });

  it('should update timing every second', async () => {
    jest.useFakeTimers();

    render(
      <WorkStepDisplay 
        currentStep="screw_tightening" 
        stepManager={mockStepManager}
      />
    );

    // Fast-forward time
    act(() => {
      jest.advanceTimersByTime(1000);
    });

    expect(mockStepManager.updateWorkStep).toHaveBeenCalled();
    expect(mockStepManager.getCurrentTiming).toHaveBeenCalled();
    expect(mockStepManager.calculateCurrentEfficiency).toHaveBeenCalled();

    jest.useRealTimers();
  });
});