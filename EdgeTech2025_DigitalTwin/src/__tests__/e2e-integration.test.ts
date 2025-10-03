import { WebSocketManager, ConnectionState } from '../main/websocket/WebSocketManager';
import { DataProcessor } from '../main/processors/DataProcessor';
import { RobotCommandManager } from '../main/services/RobotCommandManager';
import { WorkDataStore } from '../stores/WorkDataStore';
import { MockDataGenerator } from '../mock-server/mockDataGenerator';

// Mock socket.io-client for testing
jest.mock('socket.io-client', () => ({
  io: jest.fn(() => ({
    connect: jest.fn(),
    disconnect: jest.fn(),
    on: jest.fn(),
    emit: jest.fn(),
    connected: true,
  })),
}));

describe('E2E Integration Tests', () => {
  let webSocketManager: WebSocketManager;
  let dataProcessor: DataProcessor;
  let robotCommandManager: RobotCommandManager;
  let workDataStore: WorkDataStore;
  let mockDataGenerator: MockDataGenerator;

  beforeEach(() => {
    // Initialize components
    workDataStore = new WorkDataStore();
    dataProcessor = new DataProcessor();
    
    const wsConfig = {
      url: 'http://localhost:3001',
      reconnectionAttempts: 3,
      reconnectionDelay: 1000,
      timeout: 5000,
      heartbeatInterval: 30000
    };
    
    webSocketManager = new WebSocketManager(wsConfig);
    
    const robotConfig = {
      maxRetries: 3,
      retryDelay: 1000,
      commandTimeout: 5000,
      enableAutoRetry: true
    };
    
    // Create a mock WebSocket service for robot command manager
    const mockWebSocketService = {
      isConnected: jest.fn().mockReturnValue(true),
      sendRobotCommand: jest.fn().mockReturnValue(true),
      on: jest.fn(),
      off: jest.fn()
    } as any;
    
    robotCommandManager = new RobotCommandManager(mockWebSocketService, robotConfig);
    
    mockDataGenerator = new MockDataGenerator();
  });

  afterEach(() => {
    webSocketManager?.destroy();
    dataProcessor?.destroy();
    robotCommandManager?.destroy();
    workDataStore?.destroy();
    mockDataGenerator?.stop();
  });

  describe('Complete Workflow Integration', () => {
    it('should handle complete screw tightening workflow', async () => {
      // Set up event listeners
      const thresholdReachedEvents: any[] = [];
      const robotCommandEvents: any[] = [];
      
      workDataStore.on('screw_threshold_reached', (data) => {
        thresholdReachedEvents.push(data);
      });
      
      robotCommandManager.on('threshold_command_success', (data) => {
        robotCommandEvents.push(data);
      });

      // Configure low threshold for testing
      workDataStore.updateThresholdSettings({
        screwThreshold: 3,
        boltThreshold: 2
      });

      // Simulate sensor data progression
      const sensorDataSequence = [
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:00:00Z',
          data: {
            worker_status: 'waiting',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 0,
            bolt_count: 0
          }
        },
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:00:30Z',
          data: {
            worker_status: 'screw_tightening',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 1,
            bolt_count: 0
          }
        },
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:01:00Z',
          data: {
            worker_status: 'screw_tightening',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 2,
            bolt_count: 0
          }
        },
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:01:30Z',
          data: {
            worker_status: 'screw_tightening',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 3,
            bolt_count: 0
          }
        }
      ];

      // Process sensor data sequence
      for (const sensorData of sensorDataSequence) {
        const success = dataProcessor.processSensorData(sensorData);
        expect(success).toBe(true);
      }

      // Verify threshold was reached
      expect(thresholdReachedEvents).toHaveLength(1);
      expect(thresholdReachedEvents[0].count).toBe(3);
      expect(thresholdReachedEvents[0].threshold).toBe(3);

      // Simulate robot command execution
      await robotCommandManager.sendThresholdCommand('screw_threshold_reached', {
        screwCount: 3,
        timestamp: '2024-01-01T12:01:30Z'
      });

      // Verify robot command was executed
      expect(robotCommandEvents).toHaveLength(1);
      expect(robotCommandEvents[0].command).toBe('tool_handover');

      // Verify work data store state
      const finalState = workDataStore.getState();
      expect(finalState.currentWorkData?.screwCount).toBe(3);
      expect(finalState.workHistory).toHaveLength(4);
      expect(finalState.statistics.totalWorkTime).toBeGreaterThan(0);
    });

    it('should handle bolt tightening workflow', async () => {
      const thresholdReachedEvents: any[] = [];
      
      workDataStore.on('bolt_threshold_reached', (data) => {
        thresholdReachedEvents.push(data);
      });

      // Configure threshold
      workDataStore.updateThresholdSettings({
        screwThreshold: 5,
        boltThreshold: 2
      });

      // Simulate bolt tightening sequence
      const boltSequence = [
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:02:00Z',
          data: {
            worker_status: 'tool_handover',
            robot_status: { state: 'operating', grip: 'closed' },
            screw_count: 3,
            bolt_count: 0
          }
        },
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:02:30Z',
          data: {
            worker_status: 'bolt_tightening',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 3,
            bolt_count: 1
          }
        },
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:03:00Z',
          data: {
            worker_status: 'bolt_tightening',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 3,
            bolt_count: 2
          }
        }
      ];

      for (const sensorData of boltSequence) {
        dataProcessor.processSensorData(sensorData);
      }

      expect(thresholdReachedEvents).toHaveLength(1);
      expect(thresholdReachedEvents[0].count).toBe(2);
    });
  });

  describe('Error Handling Integration', () => {
    it('should handle invalid sensor data gracefully', () => {
      const invalidDataSequence = [
        { type: 'invalid_type', data: {} },
        { type: 'sensor_data', timestamp: 'invalid', data: {} },
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:00:00Z',
          data: {
            worker_status: 'invalid_status',
            robot_status: { state: 'invalid', grip: 'invalid' },
            screw_count: -1,
            bolt_count: 'not_a_number'
          }
        }
      ];

      let successCount = 0;
      for (const invalidData of invalidDataSequence) {
        const success = dataProcessor.processSensorData(invalidData);
        if (success) successCount++;
      }

      expect(successCount).toBe(0);

      // Verify notifications were added for errors
      const state = workDataStore.getState();
      expect(state.notifications.length).toBeGreaterThan(0);
      expect(state.notifications.some(n => n.type === 'error')).toBe(true);
    });

    it('should handle connection failures', () => {
      // Simulate connection status changes
      workDataStore.updateConnectionStatus(true);
      expect(workDataStore.getState().isConnected).toBe(true);

      workDataStore.updateConnectionStatus(false);
      expect(workDataStore.getState().isConnected).toBe(false);

      // Verify connection quality updates
      workDataStore.updateConnectionQuality({
        latency: 1000,
        dataRate: 0.1,
        stability: 'poor',
        lastUpdated: new Date()
      });

      const state = workDataStore.getState();
      expect(state.connectionQuality.stability).toBe('poor');
    });
  });

  describe('Performance and Stress Testing', () => {
    it('should handle high-frequency data updates', () => {
      const startTime = Date.now();
      const dataCount = 100;

      // Generate high-frequency sensor data
      for (let i = 0; i < dataCount; i++) {
        const sensorData = {
          type: 'sensor_data',
          timestamp: new Date(Date.now() + i * 100).toISOString(),
          data: {
            worker_status: i % 2 === 0 ? 'screw_tightening' : 'waiting',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: Math.floor(i / 10),
            bolt_count: Math.floor(i / 20)
          }
        };

        dataProcessor.processSensorData(sensorData);
      }

      const endTime = Date.now();
      const processingTime = endTime - startTime;

      // Verify all data was processed
      const state = workDataStore.getState();
      expect(state.workHistory).toHaveLength(dataCount);

      // Verify reasonable processing time (should be under 1 second for 100 items)
      expect(processingTime).toBeLessThan(1000);

      // Verify statistics were updated
      expect(state.statistics.totalWorkTime).toBeGreaterThan(0);
    });

    it('should handle memory management with large datasets', () => {
      // Add a large number of notifications
      for (let i = 0; i < 100; i++) {
        workDataStore.addNotification({
          type: 'info',
          title: `Notification ${i}`,
          message: `Message ${i}`
        });
      }

      const state = workDataStore.getState();
      
      // Verify notification limit is enforced
      expect(state.notifications.length).toBeLessThanOrEqual(50);

      // Verify work history management
      for (let i = 0; i < 200; i++) {
        const sensorData = {
          type: 'sensor_data',
          timestamp: new Date(Date.now() + i * 1000).toISOString(),
          data: {
            worker_status: 'screw_tightening',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: i,
            bolt_count: 0
          }
        };

        dataProcessor.processSensorData(sensorData);
      }

      const finalState = workDataStore.getState();
      
      // Verify history limit is enforced (assuming limit exists)
      expect(finalState.workHistory.length).toBeLessThanOrEqual(100);
    });
  });

  describe('Mock Data Generator Integration', () => {
    it('should generate realistic sensor data', () => {
      mockDataGenerator.start();

      // Generate a sequence of data
      const generatedData = [];
      for (let i = 0; i < 10; i++) {
        const data = mockDataGenerator.generateSensorData();
        generatedData.push(data);
      }

      mockDataGenerator.stop();

      // Verify data structure
      generatedData.forEach(data => {
        expect(data.type).toBe('sensor_data');
        expect(data.timestamp).toBeDefined();
        expect(data.data.worker_status).toBeDefined();
        expect(data.data.robot_status).toBeDefined();
        expect(typeof data.data.screw_count).toBe('number');
        expect(typeof data.data.bolt_count).toBe('number');
      });

      // Verify data can be processed
      generatedData.forEach(data => {
        const success = dataProcessor.processSensorData(data);
        expect(success).toBe(true);
      });
    });

    it('should simulate realistic work scenarios', () => {
      mockDataGenerator.setWorkerStatus('screw_tightening');
      
      const screwData = [];
      for (let i = 0; i < 5; i++) {
        screwData.push(mockDataGenerator.generateSensorData());
      }

      // Verify screw count progression
      const screwCounts = screwData.map(d => d.data.screw_count);
      expect(screwCounts[screwCounts.length - 1]).toBeGreaterThanOrEqual(screwCounts[0]);

      mockDataGenerator.setWorkerStatus('bolt_tightening');
      
      const boltData = [];
      for (let i = 0; i < 3; i++) {
        boltData.push(mockDataGenerator.generateSensorData());
      }

      // Verify bolt count progression
      const boltCounts = boltData.map(d => d.data.bolt_count);
      expect(boltCounts[boltCounts.length - 1]).toBeGreaterThanOrEqual(boltCounts[0]);
    });
  });

  describe('Statistics and Analytics Integration', () => {
    it('should calculate accurate work statistics', () => {
      const workSequence = [
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:00:00Z',
          data: {
            worker_status: 'waiting',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 0,
            bolt_count: 0
          }
        },
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:01:00Z',
          data: {
            worker_status: 'screw_tightening',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 3,
            bolt_count: 0
          }
        },
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:02:00Z',
          data: {
            worker_status: 'bolt_tightening',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 3,
            bolt_count: 2
          }
        },
        {
          type: 'sensor_data',
          timestamp: '2024-01-01T12:03:00Z',
          data: {
            worker_status: 'waiting',
            robot_status: { state: 'waiting', grip: 'open' },
            screw_count: 3,
            bolt_count: 2
          }
        }
      ];

      workSequence.forEach(data => {
        dataProcessor.processSensorData(data);
      });

      const state = workDataStore.getState();
      
      // Verify statistics calculation
      expect(state.statistics.totalWorkTime).toBeGreaterThan(0);
      expect(state.statistics.completedTasks).toBeGreaterThan(0);
      
      // Verify progress calculation
      const progress = workDataStore.getCurrentProgress();
      expect(progress.screwProgress.current).toBe(3);
      expect(progress.boltProgress.current).toBe(2);
    });
  });
});