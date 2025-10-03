import { 
  calculateProgress, 
  detectProgressChange, 
  generateProgressMessage, 
  getProgressColor,
  getProgressAnimationConfig,
  initializeCountingState,
  updateCountingState,
  validateCountIncrement,
  getRecommendedAction
} from '../utils/progressManager';
import { WorkData, ThresholdSettings } from '../types';

describe('progressManager', () => {
  const mockWorkData: WorkData = {
    timestamp: new Date(),
    workerStatus: 'screw_tightening',
    robotStatus: { state: 'waiting', grip: 'open' },
    screwCount: 3,
    boltCount: 1
  };

  const mockThresholdSettings: ThresholdSettings = {
    screwThreshold: 5,
    boltThreshold: 3
  };

  describe('calculateProgress', () => {
    it('calculates progress correctly with valid data', () => {
      const progress = calculateProgress(mockWorkData, mockThresholdSettings);

      expect(progress.screwProgress.current).toBe(3);
      expect(progress.screwProgress.target).toBe(5);
      expect(progress.screwProgress.percentage).toBe(60);
      expect(progress.screwProgress.remaining).toBe(2);
      expect(progress.screwProgress.isThresholdReached).toBe(false);
      expect(progress.screwProgress.isOverTarget).toBe(false);

      expect(progress.boltProgress.current).toBe(1);
      expect(progress.boltProgress.target).toBe(3);
      expect(progress.boltProgress.percentage).toBeCloseTo(33.33, 1);
      expect(progress.boltProgress.remaining).toBe(2);
      expect(progress.boltProgress.isThresholdReached).toBe(false);
      expect(progress.boltProgress.isOverTarget).toBe(false);
    });

    it('handles null work data', () => {
      const progress = calculateProgress(null, mockThresholdSettings);

      expect(progress.screwProgress.current).toBe(0);
      expect(progress.screwProgress.percentage).toBe(0);
      expect(progress.boltProgress.current).toBe(0);
      expect(progress.boltProgress.percentage).toBe(0);
    });

    it('handles threshold reached', () => {
      const workDataAtThreshold: WorkData = {
        ...mockWorkData,
        screwCount: 5,
        boltCount: 3
      };

      const progress = calculateProgress(workDataAtThreshold, mockThresholdSettings);

      expect(progress.screwProgress.isThresholdReached).toBe(true);
      expect(progress.screwProgress.percentage).toBe(100);
      expect(progress.screwProgress.remaining).toBe(0);
      expect(progress.screwProgress.isOverTarget).toBe(false);

      expect(progress.boltProgress.isThresholdReached).toBe(true);
      expect(progress.boltProgress.percentage).toBe(100);
      expect(progress.boltProgress.remaining).toBe(0);
      expect(progress.boltProgress.isOverTarget).toBe(false);
    });

    it('does not exceed 100% when count exceeds threshold', () => {
      const workDataOverThreshold: WorkData = {
        ...mockWorkData,
        screwCount: 7,
        boltCount: 5
      };

      const progress = calculateProgress(workDataOverThreshold, mockThresholdSettings);

      expect(progress.screwProgress.percentage).toBe(100);
      expect(progress.boltProgress.percentage).toBe(100);
    });

    it('handles zero threshold gracefully', () => {
      const zeroThresholdSettings: ThresholdSettings = {
        screwThreshold: 0,
        boltThreshold: 0
      };

      const progress = calculateProgress(mockWorkData, zeroThresholdSettings);

      expect(progress.screwProgress.percentage).toBe(0);
      expect(progress.boltProgress.percentage).toBe(0);
    });
  });

  describe('detectProgressChange', () => {
    it('detects changes correctly', () => {
      const previousProgress = calculateProgress(mockWorkData, mockThresholdSettings);
      const newWorkData: WorkData = { ...mockWorkData, screwCount: 4, boltCount: 2 };
      const currentProgress = calculateProgress(newWorkData, mockThresholdSettings);

      const changes = detectProgressChange(previousProgress, currentProgress);

      expect(changes.screwChanged).toBe(true);
      expect(changes.boltChanged).toBe(true);
      expect(changes.screwThresholdReached).toBe(false);
      expect(changes.boltThresholdReached).toBe(false);
    });

    it('detects threshold reached', () => {
      const previousProgress = calculateProgress(mockWorkData, mockThresholdSettings);
      const newWorkData: WorkData = { ...mockWorkData, screwCount: 5, boltCount: 3 };
      const currentProgress = calculateProgress(newWorkData, mockThresholdSettings);

      const changes = detectProgressChange(previousProgress, currentProgress);

      expect(changes.screwThresholdReached).toBe(true);
      expect(changes.boltThresholdReached).toBe(true);
    });

    it('handles null previous progress', () => {
      const currentProgress = calculateProgress(mockWorkData, mockThresholdSettings);
      const changes = detectProgressChange(null, currentProgress);

      expect(changes.screwChanged).toBe(true);
      expect(changes.boltChanged).toBe(true);
    });
  });

  describe('generateProgressMessage', () => {
    it('generates correct messages for in-progress work', () => {
      const progress = calculateProgress(mockWorkData, mockThresholdSettings);
      const messages = generateProgressMessage(progress);

      expect(messages.screwMessage).toBe('🎯 目標まで2回');
      expect(messages.boltMessage).toBe('🎯 目標まで2回');
    });

    it('generates achievement messages when threshold reached', () => {
      const workDataAtThreshold: WorkData = {
        ...mockWorkData,
        screwCount: 5,
        boltCount: 3
      };
      const progress = calculateProgress(workDataAtThreshold, mockThresholdSettings);
      const messages = generateProgressMessage(progress);

      expect(messages.screwMessage).toBe('🎉 目標達成！');
      expect(messages.boltMessage).toBe('🎉 目標達成！');
    });

    it('generates over-target messages when count exceeds threshold', () => {
      const workDataOverThreshold: WorkData = {
        ...mockWorkData,
        screwCount: 7,
        boltCount: 5
      };
      const progress = calculateProgress(workDataOverThreshold, mockThresholdSettings);
      const messages = generateProgressMessage(progress);

      expect(messages.screwMessage).toBe('🎉 目標超過！ (+2)');
      expect(messages.boltMessage).toBe('🎉 目標超過！ (+2)');
    });

    it('handles zero threshold settings', () => {
      const zeroThresholdSettings: ThresholdSettings = {
        screwThreshold: 0,
        boltThreshold: 0
      };
      const progress = calculateProgress(mockWorkData, zeroThresholdSettings);
      const messages = generateProgressMessage(progress);

      expect(messages.screwMessage).toBe('目標未設定');
      expect(messages.boltMessage).toBe('目標未設定');
    });
  });

  describe('getProgressColor', () => {
    it('returns success color when threshold reached', () => {
      const progress = { current: 5, target: 5, isThresholdReached: true };
      expect(getProgressColor(progress)).toBe('#4CAF50');
    });

    it('returns primary color for high progress', () => {
      const progress = { current: 4, target: 5, isThresholdReached: false };
      expect(getProgressColor(progress)).toBe('#2196F3');
    });

    it('returns warning color for medium progress', () => {
      const progress = { current: 3, target: 5, isThresholdReached: false };
      expect(getProgressColor(progress)).toBe('#FF9800');
    });

    it('returns error color for low progress', () => {
      const progress = { current: 1, target: 5, isThresholdReached: false };
      expect(getProgressColor(progress)).toBe('#F44336');
    });
  });

  describe('getProgressAnimationConfig', () => {
    it('returns celebration config when threshold reached', () => {
      const config = getProgressAnimationConfig(true);
      expect(config.duration).toBe(1500);
      expect(config.easing).toBe('easeOutBounce');
      expect(config.delay).toBe(200);
    });

    it('returns normal config when threshold not reached', () => {
      const config = getProgressAnimationConfig(false);
      expect(config.duration).toBe(1000);
      expect(config.easing).toBe('easeOutQuart');
      expect(config.delay).toBe(0);
    });
  });

  describe('counting state management', () => {
    it('initializes counting state correctly', () => {
      const state = initializeCountingState();
      expect(state.previousScrewCount).toBe(0);
      expect(state.previousBoltCount).toBe(0);
      expect(state.screwIncrements).toBe(0);
      expect(state.boltIncrements).toBe(0);
      expect(state.lastWorkerStatus).toBe(null);
    });

    it('updates counting state when counts increase', () => {
      const initialState = initializeCountingState();
      const workDataWithIncrease: WorkData = {
        ...mockWorkData,
        screwCount: 5,
        boltCount: 2
      };

      const updatedState = updateCountingState(initialState, workDataWithIncrease);

      expect(updatedState.screwIncrements).toBe(5);
      expect(updatedState.boltIncrements).toBe(2);
      expect(updatedState.previousScrewCount).toBe(5);
      expect(updatedState.previousBoltCount).toBe(2);
      expect(updatedState.lastWorkerStatus).toBe('screw_tightening');
    });

    it('handles incremental updates correctly', () => {
      let state = initializeCountingState();
      
      // First update
      const firstWorkData: WorkData = { ...mockWorkData, screwCount: 2, boltCount: 1 };
      state = updateCountingState(state, firstWorkData);
      
      // Second update
      const secondWorkData: WorkData = { ...mockWorkData, screwCount: 4, boltCount: 3 };
      state = updateCountingState(state, secondWorkData);

      expect(state.screwIncrements).toBe(4); // 2 + 2
      expect(state.boltIncrements).toBe(3); // 1 + 2
    });
  });

  describe('validateCountIncrement', () => {
    it('validates normal count increment', () => {
      const result = validateCountIncrement(3, 4, 'screw_tightening', 'screw_tightening');
      expect(result.isValid).toBe(true);
      expect(result.increment).toBe(1);
      expect(result.message).toBeUndefined();
    });

    it('detects negative count increment', () => {
      const result = validateCountIncrement(5, 3, 'screw_tightening', 'screw_tightening');
      expect(result.isValid).toBe(false);
      expect(result.increment).toBe(0);
      expect(result.message).toBe('カウントが減少しました。データエラーの可能性があります。');
    });

    it('detects abnormally large increment', () => {
      const result = validateCountIncrement(1, 15, 'screw_tightening', 'screw_tightening');
      expect(result.isValid).toBe(false);
      expect(result.increment).toBe(14);
      expect(result.message).toBe('異常に大きなカウント増加が検出されました。');
    });

    it('detects count increment with wrong worker status', () => {
      const result = validateCountIncrement(3, 4, 'waiting', 'screw_tightening');
      expect(result.isValid).toBe(false);
      expect(result.increment).toBe(1);
      expect(result.message).toBe('作業状態がscrew_tighteningでないのにカウントが増加しました。');
    });
  });

  describe('getRecommendedAction', () => {
    it('returns correct action for screw threshold reached', () => {
      const action = getRecommendedAction('screw', true, false);
      expect(action).toBe('ネジ締め作業が完了しました。ロボットに工具を渡してください。');
    });

    it('returns correct action for bolt threshold reached', () => {
      const action = getRecommendedAction('bolt', true, false);
      expect(action).toBe('ボルト締め作業が完了しました。次のタスクに進んでください。');
    });

    it('returns correct action for over-target screw', () => {
      const action = getRecommendedAction('screw', true, true);
      expect(action).toBe('ネジ締め作業が目標を超過しました。次の工程に進んでください。');
    });

    it('returns null when threshold not reached', () => {
      const action = getRecommendedAction('screw', false, false);
      expect(action).toBe(null);
    });
  });
});