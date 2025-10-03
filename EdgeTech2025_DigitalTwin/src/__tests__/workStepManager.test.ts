import { WorkStepManager } from '../utils/workStepManager';
import { WorkerStatus } from '../types';

// Mock Date for consistent testing
const mockDate = new Date('2024-01-01T10:00:00Z');
let currentMockTime = mockDate.getTime();

beforeAll(() => {
  jest.useFakeTimers();
  jest.setSystemTime(mockDate);
});

afterAll(() => {
  jest.useRealTimers();
});

describe('WorkStepManager', () => {
  let manager: WorkStepManager;

  beforeEach(() => {
    manager = new WorkStepManager();
    jest.clearAllMocks();
  });

  describe('initialization', () => {
    it('should initialize with waiting step', () => {
      const timing = manager.getCurrentTiming();
      expect(timing.currentStep).toBe('waiting');
      expect(timing.elapsedTime).toBe(0);
      expect(timing.stepCount).toBe(0);
    });

    it('should initialize with custom step', () => {
      const customManager = new WorkStepManager('screw_tightening');
      const timing = customManager.getCurrentTiming();
      expect(timing.currentStep).toBe('screw_tightening');
    });
  });

  describe('updateWorkStep', () => {
    it('should update to new step and reset timing', () => {
      // Simulate time passing
      jest.advanceTimersByTime(5000); // 5 seconds later

      manager.updateWorkStep('screw_tightening');
      const timing = manager.getCurrentTiming();
      
      expect(timing.currentStep).toBe('screw_tightening');
      expect(timing.elapsedTime).toBe(0); // Should reset for new step
    });

    it('should not reset timing if step is the same', () => {
      // Simulate time passing
      jest.advanceTimersByTime(3000); // 3 seconds later

      manager.updateElapsedTime();
      const initialElapsed = manager.getCurrentTiming().elapsedTime;
      
      manager.updateWorkStep('waiting'); // Same step
      const timing = manager.getCurrentTiming();
      
      expect(timing.currentStep).toBe('waiting');
      expect(timing.elapsedTime).toBe(initialElapsed); // Should not reset
    });
  });

  describe('updateElapsedTime', () => {
    it('should calculate elapsed time correctly', () => {
      // Simulate 10 seconds passing
      jest.advanceTimersByTime(10000);

      manager.updateElapsedTime();
      const timing = manager.getCurrentTiming();
      
      expect(timing.elapsedTime).toBe(10);
    });
  });

  describe('calculateCurrentEfficiency', () => {
    it('should calculate efficiency based on expected time', () => {
      // Set up screw tightening step (expected: 45 seconds)
      manager.updateWorkStep('screw_tightening');
      
      // Simulate 30 seconds elapsed (should be efficient)
      jest.advanceTimersByTime(30000);
      
      // Update elapsed time to reflect the time passage
      manager.updateElapsedTime();

      const efficiency = manager.calculateCurrentEfficiency();
      
      expect(efficiency.currentEfficiency).toBeGreaterThan(90);
      expect(efficiency.expectedStepTime).toBe(45);
    });

    it('should return lower efficiency for overtime', () => {
      // Set up screw tightening step (expected: 45 seconds)
      manager.updateWorkStep('screw_tightening');
      
      // Simulate 90 seconds elapsed (double expected time)
      jest.advanceTimersByTime(90000);
      
      // Update elapsed time to reflect the time passage
      manager.updateElapsedTime();

      const efficiency = manager.calculateCurrentEfficiency();
      
      expect(efficiency.currentEfficiency).toBeLessThan(60);
    });

    it('should handle absent status correctly', () => {
      manager.updateWorkStep('absent');
      const efficiency = manager.calculateCurrentEfficiency();
      
      expect(efficiency.currentEfficiency).toBe(100);
      expect(efficiency.expectedStepTime).toBe(0);
    });
  });

  describe('getStepStatistics', () => {
    it('should track step counts and times', () => {
      // Simulate a sequence of steps
      manager.updateWorkStep('screw_tightening');
      
      // Simulate 30 seconds
      jest.advanceTimersByTime(30000);
      
      manager.updateWorkStep('bolt_tightening');
      
      // Simulate another 15 seconds
      jest.advanceTimersByTime(15000);
      
      manager.updateWorkStep('waiting');
      
      const stats = manager.getStepStatistics();
      
      expect(stats.stepCounts.screw_tightening).toBe(1);
      expect(stats.stepCounts.bolt_tightening).toBe(1);
      expect(stats.stepCounts.waiting).toBe(1); // Initial waiting step
      expect(stats.totalSteps).toBe(3); // waiting -> screw -> bolt
      expect(stats.stepTotalTimes.screw_tightening).toBe(30);
      expect(stats.stepTotalTimes.bolt_tightening).toBe(15);
    });
  });

  describe('static utility methods', () => {
    it('should format time correctly', () => {
      expect(WorkStepManager.formatTime(0)).toBe('0:00');
      expect(WorkStepManager.formatTime(65)).toBe('1:05');
      expect(WorkStepManager.formatTime(3661)).toBe('61:01');
    });

    it('should return correct step display names', () => {
      expect(WorkStepManager.getStepDisplayName('waiting')).toBe('待機中');
      expect(WorkStepManager.getStepDisplayName('screw_tightening')).toBe('ネジ締め作業');
      expect(WorkStepManager.getStepDisplayName('bolt_tightening')).toBe('ボルト締め作業');
      expect(WorkStepManager.getStepDisplayName('tool_handover')).toBe('工具受け渡し');
      expect(WorkStepManager.getStepDisplayName('absent')).toBe('不在');
    });

    it('should return correct step icons', () => {
      expect(WorkStepManager.getStepIcon('waiting')).toBe('⏳');
      expect(WorkStepManager.getStepIcon('screw_tightening')).toBe('🔧');
      expect(WorkStepManager.getStepIcon('bolt_tightening')).toBe('🔩');
      expect(WorkStepManager.getStepIcon('tool_handover')).toBe('🤝');
      expect(WorkStepManager.getStepIcon('absent')).toBe('👤');
    });

    it('should return correct efficiency colors', () => {
      expect(WorkStepManager.getEfficiencyColor(95)).toBe('var(--color-success)');
      expect(WorkStepManager.getEfficiencyColor(80)).toBe('var(--color-primary-blue)');
      expect(WorkStepManager.getEfficiencyColor(60)).toBe('var(--color-warning)');
      expect(WorkStepManager.getEfficiencyColor(40)).toBe('var(--color-error)');
    });

    it('should return correct trend icons', () => {
      expect(WorkStepManager.getTrendIcon('improving')).toBe('📈');
      expect(WorkStepManager.getTrendIcon('declining')).toBe('📉');
      expect(WorkStepManager.getTrendIcon('stable')).toBe('📊');
    });
  });

  describe('reset', () => {
    it('should reset all data to initial state', () => {
      // Add some history
      manager.updateWorkStep('screw_tightening');
      manager.updateWorkStep('waiting');
      
      manager.reset();
      
      const timing = manager.getCurrentTiming();
      const stats = manager.getStepStatistics();
      
      expect(timing.currentStep).toBe('waiting');
      expect(timing.elapsedTime).toBe(0);
      expect(stats.totalSteps).toBe(0);
    });
  });
});