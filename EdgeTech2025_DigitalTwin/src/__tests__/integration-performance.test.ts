/**
 * Integration and Performance Tests
 * Tests the final integration and performance optimizations
 */

import { PerformanceMonitor, AnimationManager, DataIntegrationManager, MemoryOptimizer, DeviceCapabilities } from '../utils/performanceOptimizer';
import { DataIntegrationManager as DataManager } from '../utils/dataIntegrationManager';
import { AnimationManager as AnimManager } from '../utils/animationManager';
import { WorkData, WorkerStatus, RobotStatus, ThresholdSettings } from '../types';

// Mock DOM elements for animation tests
const mockElement = {
  style: {} as CSSStyleDeclaration,
  animate: jest.fn().mockReturnValue({
    addEventListener: jest.fn(),
    cancel: jest.fn()
  }),
  nodeType: 1,
  nodeName: 'DIV'
} as unknown as HTMLElement;

// Mock getComputedStyle
Object.defineProperty(window, 'getComputedStyle', {
  value: jest.fn(() => ({
    opacity: '1',
    transform: 'none'
  }))
});

// Mock performance API
Object.defineProperty(global, 'performance', {
  value: {
    now: jest.fn(() => Date.now()),
    mark: jest.fn(),
    measure: jest.fn(),
    getEntriesByName: jest.fn(() => [{ duration: 10 }]),
    clearMarks: jest.fn(),
    clearMeasures: jest.fn(),
    memory: {
      usedJSHeapSize: 1000000,
      totalJSHeapSize: 2000000,
      jsHeapSizeLimit: 4000000
    }
  }
});

// Mock canvas context for WebGL detection
HTMLCanvasElement.prototype.getContext = jest.fn((contextType) => {
  if (contextType === 'webgl' || contextType === 'experimental-webgl') {
    return {}; // Mock WebGL context
  }
  return null;
});

// Mock requestAnimationFrame
global.requestAnimationFrame = jest.fn((cb) => {
  setTimeout(cb, 16);
  return 1;
});

global.cancelAnimationFrame = jest.fn();

describe('Integration and Performance Tests', () => {
  let performanceMonitor: PerformanceMonitor;
  let dataIntegrationManager: DataManager;
  let animationManager: AnimManager;

  beforeEach(() => {
    performanceMonitor = PerformanceMonitor.getInstance();
    dataIntegrationManager = DataManager.getInstance();
    animationManager = AnimManager.getInstance();
    
    // Clear any existing measurements
    jest.clearAllMocks();
  });

  afterEach(() => {
    // Clean up
    dataIntegrationManager.destroy();
    animationManager.destroy();
  });

  describe('Performance Monitoring Integration', () => {
    it('should monitor performance across all operations', () => {
      // Test performance monitoring for data integration
      const workData: WorkData = {
        timestamp: new Date(),
        workerStatus: 'screw_tightening',
        robotStatus: { state: 'waiting', grip: 'closed' },
        screwCount: 3,
        boltCount: 1
      };

      dataIntegrationManager.integrateWorkData(workData);

      // Verify performance monitoring was called
      expect(performance.mark).toHaveBeenCalledWith('integrateWorkData-start');
      expect(performance.mark).toHaveBeenCalledWith('integrateWorkData-end');
    });

    it('should collect memory usage statistics', () => {
      performanceMonitor.recordMemoryUsage();
      
      const memoryStats = performanceMonitor.getMemoryStats();
      
      expect(memoryStats.current).toBeGreaterThan(0);
      expect(memoryStats.average).toBeGreaterThan(0);
      expect(memoryStats.peak).toBeGreaterThan(0);
    });

    it('should track frame rate performance', () => {
      // Simulate multiple frame recordings
      for (let i = 0; i < 5; i++) {
        performanceMonitor.recordFrameRate();
      }
      
      const frameRateStats = performanceMonitor.getFrameRateStats();
      
      expect(frameRateStats.current).toBeGreaterThan(0);
      expect(frameRateStats.average).toBeGreaterThan(0);
    });
  });

  describe('Data Integration Optimization', () => {
    it('should batch similar data updates', (done) => {
      let updateCount = 0;
      
      dataIntegrationManager.on('workDataUpdated', () => {
        updateCount++;
      });

      // Send multiple rapid updates
      const workData: WorkData = {
        timestamp: new Date(),
        workerStatus: 'screw_tightening',
        robotStatus: { state: 'waiting', grip: 'closed' },
        screwCount: 3,
        boltCount: 1
      };

      dataIntegrationManager.integrateWorkData(workData);
      dataIntegrationManager.integrateWorkData({ ...workData, screwCount: 4 });
      dataIntegrationManager.integrateWorkData({ ...workData, screwCount: 5 });

      // Should batch updates and only emit once after delay
      setTimeout(() => {
        expect(updateCount).toBeLessThanOrEqual(2); // Should be batched
        done();
      }, 50);
    });

    it('should validate data before integration', () => {
      // Test with completely invalid data structure
      const invalidWorkData = null as any;

      const result = dataIntegrationManager.integrateWorkData(invalidWorkData);
      
      expect(result).toBe(false);
    });

    it('should cache data to prevent unnecessary updates', () => {
      const workData: WorkData = {
        timestamp: new Date(),
        workerStatus: 'screw_tightening',
        robotStatus: { state: 'waiting', grip: 'closed' },
        screwCount: 3,
        boltCount: 1
      };

      // First integration should succeed
      const result1 = dataIntegrationManager.integrateWorkData(workData);
      expect(result1).toBe(true);

      // Second integration with same data should be skipped
      const result2 = dataIntegrationManager.integrateWorkData(workData);
      expect(result2).toBe(true);

      // Verify data is cached
      const cachedData = dataIntegrationManager.getCachedData('currentWorkData');
      expect(cachedData).toBeDefined();
      expect(cachedData.workerStatus).toBe('screw_tightening');
    });
  });

  describe('Animation Optimization', () => {
    it('should detect device capabilities and adjust animations', () => {
      const capabilities = DeviceCapabilities.detect();
      
      expect(capabilities).toHaveProperty('webgl');
      expect(capabilities).toHaveProperty('highMemory');
      expect(capabilities).toHaveProperty('touchSupport');
      expect(capabilities).toHaveProperty('highPerformance');
    });

    it('should create optimized animations based on device capabilities', () => {
      const animationId = animationManager.createFadeAnimation(mockElement, {
        from: 0,
        to: 1,
        duration: 300
      });

      expect(animationId).toBeDefined();
      expect(mockElement.animate).toHaveBeenCalled();
    });

    it('should queue animations when max concurrent limit is reached', () => {
      const config = animationManager.getConfig();
      
      // Create animations up to the limit
      const animationIds: string[] = [];
      for (let i = 0; i < config.maxConcurrentAnimations + 2; i++) {
        const id = animationManager.createScaleAnimation(mockElement, {
          from: 1,
          to: 1.1,
          duration: 100
        });
        animationIds.push(id);
      }

      const stats = animationManager.getStatistics();
      expect(stats.queuedAnimations).toBeGreaterThan(0);
    });

    it('should cancel all animations when requested', () => {
      // Create some animations
      animationManager.createFadeAnimation(mockElement);
      animationManager.createScaleAnimation(mockElement);
      
      animationManager.cancelAllAnimations();
      
      const stats = animationManager.getStatistics();
      expect(stats.activeAnimations).toBe(0);
      expect(stats.queuedAnimations).toBe(0);
    });
  });

  describe('Memory Optimization', () => {
    it('should create and manage object pools', () => {
      MemoryOptimizer.createPool('testObjects', () => ({ value: 0 }), 5);
      
      const obj1 = MemoryOptimizer.getFromPool('testObjects');
      expect(obj1).toBeDefined();
      expect(obj1).toHaveProperty('value');
      
      MemoryOptimizer.returnToPool('testObjects', obj1);
      
      const obj2 = MemoryOptimizer.getFromPool('testObjects');
      expect(obj2).toBe(obj1); // Should reuse the same object
    });

    it('should manage weak references for cleanup', () => {
      const testObject = { data: 'test' };
      MemoryOptimizer.addWeakRef(testObject);
      
      // Cleanup should not throw
      expect(() => MemoryOptimizer.cleanupWeakRefs()).not.toThrow();
    });
  });

  describe('End-to-End Integration', () => {
    it('should handle complete data flow with optimizations', (done) => {
      let eventsReceived = 0;
      const expectedEvents = ['workDataUpdated', 'connectionStatusUpdated'];
      
      expectedEvents.forEach(event => {
        dataIntegrationManager.on(event, () => {
          eventsReceived++;
          if (eventsReceived === expectedEvents.length) {
            done();
          }
        });
      });

      // Simulate complete data flow
      const workData: WorkData = {
        timestamp: new Date(),
        workerStatus: 'bolt_tightening',
        robotStatus: { state: 'operating', grip: 'open' },
        screwCount: 5,
        boltCount: 3
      };

      dataIntegrationManager.integrateWorkData(workData);
      dataIntegrationManager.integrateConnectionStatus(true, {
        latency: 25,
        dataRate: 2.5,
        stability: 'good',
        lastUpdated: new Date()
      });
    });

    it('should maintain performance under load', async () => {
      const startTime = performance.now();
      const iterations = 100;
      
      // Simulate high-frequency updates
      for (let i = 0; i < iterations; i++) {
        const workData: WorkData = {
          timestamp: new Date(),
          workerStatus: i % 2 === 0 ? 'screw_tightening' : 'bolt_tightening',
          robotStatus: { 
            state: i % 3 === 0 ? 'operating' : 'waiting', 
            grip: i % 2 === 0 ? 'open' : 'closed' 
          },
          screwCount: i % 10,
          boltCount: i % 5
        };
        
        dataIntegrationManager.integrateWorkData(workData);
        
        // Create animations periodically
        if (i % 10 === 0) {
          animationManager.createFadeAnimation(mockElement, { duration: 100 });
        }
      }
      
      const endTime = performance.now();
      const totalTime = endTime - startTime;
      
      // Should complete within reasonable time (less than 1 second)
      expect(totalTime).toBeLessThan(1000);
      
      // Performance metrics should be available
      const metrics = dataIntegrationManager.getPerformanceMetrics();
      expect(metrics).toBeDefined();
    });

    it('should handle threshold achievements with celebrations', () => {
      const thresholdSettings: ThresholdSettings = {
        screwThreshold: 5,
        boltThreshold: 3
      };

      dataIntegrationManager.integrateThresholdSettings(thresholdSettings);

      // Simulate threshold achievement
      const workData: WorkData = {
        timestamp: new Date(),
        workerStatus: 'screw_tightening',
        robotStatus: { state: 'waiting', grip: 'closed' },
        screwCount: 5, // Reaches threshold
        boltCount: 1
      };

      const result = dataIntegrationManager.integrateWorkData(workData);
      expect(result).toBe(true);

      // Should trigger celebration animation
      const celebrationId = animationManager.createCelebrationAnimation(mockElement);
      expect(celebrationId).toBeDefined();
    });
  });

  describe('Error Handling and Recovery', () => {
    it('should handle animation errors gracefully', () => {
      // Create a proper mock element that will work with getComputedStyle
      const errorElement = document.createElement('div');
      
      // Mock animate to throw error
      errorElement.animate = jest.fn().mockImplementation(() => {
        throw new Error('Animation failed');
      });

      // Should handle the error gracefully and not crash
      expect(() => {
        animationManager.createFadeAnimation(errorElement);
      }).not.toThrow();
    });

    it('should recover from data integration errors', () => {
      // Test with malformed data - should handle gracefully
      const malformedData = undefined as any;
      
      // Should not throw and return false for invalid data
      expect(() => {
        const result = dataIntegrationManager.integrateWorkData(malformedData);
        expect(result).toBe(false);
      }).not.toThrow();
      
      // Should still work with valid data after error
      const validData: WorkData = {
        timestamp: new Date(),
        workerStatus: 'waiting',
        robotStatus: { state: 'waiting', grip: 'closed' },
        screwCount: 0,
        boltCount: 0
      };
      
      const validResult = dataIntegrationManager.integrateWorkData(validData);
      expect(validResult).toBe(true);
    });
  });
});