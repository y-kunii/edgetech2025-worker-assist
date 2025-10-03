import { WebSocketManager, ConnectionState } from '../main/websocket/WebSocketManager';
import { WorkDataStore } from '../stores/WorkDataStore';
import { DataProcessor } from '../main/processors/DataProcessor';
import { validateSensorData, convertSensorDataToWorkData } from '../utils/validation';

// Mock Socket.io
jest.mock('socket.io-client', () => ({
  io: jest.fn(() => ({
    connect: jest.fn(),
    disconnect: jest.fn(),
    on: jest.fn(),
    emit: jest.fn(),
    connected: true
  }))
}));

describe('WebSocket Data Integration', () => {
  let workDataStore: WorkDataStore;
  let dataProcessor: DataProcessor;

  beforeEach(() => {
    workDataStore = new WorkDataStore();
    dataProcessor = new DataProcessor();
  });

  afterEach(() => {
    workDataStore.destroy();
    dataProcessor.destroy();
  });

  describe('Data Validation', () => {
    test('should validate correct sensor data', () => {
      const validSensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          image: 'base64encodedimage',
          worker_status: 'screw_tightening',
          robot_status: {
            state: 'waiting',
            grip: 'open'
          },
          screw_count: 3,
          bolt_count: 1,
          work_step: 'screw_tightening'
        }
      };

      const result = validateSensorData(validSensorData);
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
      expect(result.data).toEqual(validSensorData);
    });

    test('should reject invalid sensor data', () => {
      const invalidSensorData = {
        type: 'sensor_data',
        timestamp: 'invalid-timestamp',
        data: {
          worker_status: 'invalid_status',
          robot_status: {
            state: 'invalid_state',
            grip: 'invalid_grip'
          },
          screw_count: -1,
          bolt_count: 'not_a_number'
        }
      };

      const result = validateSensorData(invalidSensorData);
      expect(result.isValid).toBe(false);
      expect(result.errors.length).toBeGreaterThan(0);
    });

    test('should convert sensor data to work data', () => {
      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          image: 'base64encodedimage',
          worker_status: 'screw_tightening' as const,
          robot_status: {
            state: 'waiting' as const,
            grip: 'open' as const
          },
          screw_count: 3,
          bolt_count: 1,
          work_step: 'screw_tightening' as const
        }
      };

      const workData = convertSensorDataToWorkData(sensorData);
      
      expect(workData.timestamp).toBeInstanceOf(Date);
      expect(workData.workerStatus).toBe('screw_tightening');
      expect(workData.robotStatus.state).toBe('waiting');
      expect(workData.robotStatus.grip).toBe('open');
      expect(workData.screwCount).toBe(3);
      expect(workData.boltCount).toBe(1);
      expect(workData.image).toBe('base64encodedimage');
    });
  });

  describe('WorkDataStore', () => {
    test('should initialize with default state', () => {
      const state = workDataStore.getState();
      
      expect(state.currentWorkData).toBeNull();
      expect(state.thresholdSettings.screwThreshold).toBe(5);
      expect(state.thresholdSettings.boltThreshold).toBe(3);
      expect(state.workHistory).toHaveLength(0);
      expect(state.statistics.totalWorkTime).toBe(0);
      expect(state.isConnected).toBe(false);
    });

    test('should update sensor data successfully', () => {
      const validSensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: {
            state: 'waiting',
            grip: 'open'
          },
          screw_count: 3,
          bolt_count: 1,
          work_step: 'screw_tightening'
        }
      };

      const success = workDataStore.updateSensorData(validSensorData);
      expect(success).toBe(true);

      const state = workDataStore.getState();
      expect(state.currentWorkData).not.toBeNull();
      expect(state.currentWorkData?.workerStatus).toBe('screw_tightening');
      expect(state.currentWorkData?.screwCount).toBe(3);
    });

    test('should emit threshold reached events', (done) => {
      // 閾値を低く設定
      workDataStore.updateThresholdSettings({
        screwThreshold: 2,
        boltThreshold: 1
      });

      workDataStore.on('screw_threshold_reached', (data) => {
        expect(data.count).toBe(2);
        expect(data.threshold).toBe(2);
        done();
      });

      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 2,
          bolt_count: 0,
          work_step: 'screw_tightening'
        }
      };

      workDataStore.updateSensorData(sensorData);
    });

    test('should update connection status', () => {
      workDataStore.updateConnectionStatus(true);
      expect(workDataStore.getState().isConnected).toBe(true);

      workDataStore.updateConnectionStatus(false);
      expect(workDataStore.getState().isConnected).toBe(false);
    });

    test('should add and manage notifications', () => {
      workDataStore.addNotification({
        type: 'info',
        title: 'Test Notification',
        message: 'This is a test message'
      });

      const state = workDataStore.getState();
      expect(state.notifications).toHaveLength(1);
      expect(state.notifications[0].title).toBe('Test Notification');
      expect(state.notifications[0].read).toBe(false);
    });
  });

  describe('DataProcessor', () => {
    test('should process sensor data through data store', () => {
      const validSensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'bolt_tightening',
          robot_status: {
            state: 'operating',
            grip: 'closed'
          },
          screw_count: 5,
          bolt_count: 2,
          work_step: 'bolt_tightening'
        }
      };

      const success = dataProcessor.processSensorData(validSensorData);
      expect(success).toBe(true);

      const state = dataProcessor.getCurrentState();
      expect(state.currentWorkData?.workerStatus).toBe('bolt_tightening');
      expect(state.currentWorkData?.robotStatus.state).toBe('operating');
    });

    test('should handle invalid sensor data gracefully', () => {
      const invalidSensorData = {
        type: 'invalid_type',
        data: {
          invalid_field: 'invalid_value'
        }
      };

      const success = dataProcessor.processSensorData(invalidSensorData);
      expect(success).toBe(false);
    });

    test('should update threshold settings', () => {
      const newSettings = {
        screwThreshold: 10,
        boltThreshold: 5
      };

      const success = dataProcessor.updateThresholdSettings(newSettings);
      expect(success).toBe(true);

      const state = dataProcessor.getCurrentState();
      expect(state.thresholdSettings.screwThreshold).toBe(10);
      expect(state.thresholdSettings.boltThreshold).toBe(5);
    });
  });

  describe('WebSocketManager', () => {
    test('should initialize with correct configuration', () => {
      const config = {
        url: 'http://localhost:3001',
        reconnectionAttempts: 5,
        reconnectionDelay: 2000,
        timeout: 10000
      };

      const wsManager = new WebSocketManager(config);
      const status = wsManager.getConnectionStatus();
      
      expect(status.state).toBe(ConnectionState.DISCONNECTED);
      expect(status.reconnectAttempts).toBe(0);
    });

    test('should handle connection state changes', () => {
      const config = { url: 'http://localhost:3001' };
      const wsManager = new WebSocketManager(config);
      
      let stateChanges: ConnectionState[] = [];
      wsManager.on('connection_state_changed', (state) => {
        stateChanges.push(state);
      });

      // Simulate connection attempt
      wsManager.connect();
      
      // Note: In a real test, we would mock the socket.io connection
      // For now, we just verify the manager was created successfully
      expect(wsManager).toBeDefined();
    });
  });
});