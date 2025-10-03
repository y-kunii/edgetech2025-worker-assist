import { DataProcessor } from '../main/processors/DataProcessor';
import { WorkDataStore } from '../stores/WorkDataStore';
import { validateSensorData } from '../utils/validation';

// Mock dependencies
jest.mock('../stores/WorkDataStore');
jest.mock('../utils/validation');

const MockedWorkDataStore = WorkDataStore as jest.MockedClass<typeof WorkDataStore>;
const mockedValidateSensorData = validateSensorData as jest.MockedFunction<typeof validateSensorData>;

describe('DataProcessor', () => {
  let dataProcessor: DataProcessor;
  let mockWorkDataStore: jest.Mocked<WorkDataStore>;

  beforeEach(() => {
    mockWorkDataStore = new MockedWorkDataStore() as jest.Mocked<WorkDataStore>;
    mockWorkDataStore.updateSensorData = jest.fn();
    mockWorkDataStore.getState = jest.fn();
    mockWorkDataStore.updateThresholdSettings = jest.fn();
    mockWorkDataStore.updateConnectionStatus = jest.fn();
    mockWorkDataStore.addNotification = jest.fn();
    mockWorkDataStore.destroy = jest.fn();

    dataProcessor = new DataProcessor();
    // Replace the internal store with our mock
    (dataProcessor as any).workDataStore = mockWorkDataStore;
  });

  afterEach(() => {
    dataProcessor.destroy();
    jest.clearAllMocks();
  });

  describe('processSensorData', () => {
    it('should process valid sensor data successfully', () => {
      const validSensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1,
          work_step: 'screw_tightening'
        }
      };

      mockedValidateSensorData.mockReturnValue({
        isValid: true,
        errors: [],
        data: validSensorData
      });

      mockWorkDataStore.updateSensorData.mockReturnValue(true);

      const result = dataProcessor.processSensorData(validSensorData);

      expect(result).toBe(true);
      expect(mockedValidateSensorData).toHaveBeenCalledWith(validSensorData);
      expect(mockWorkDataStore.updateSensorData).toHaveBeenCalledWith(validSensorData);
    });

    it('should reject invalid sensor data', () => {
      const invalidSensorData = {
        type: 'invalid_type',
        data: { invalid_field: 'invalid_value' }
      };

      mockedValidateSensorData.mockReturnValue({
        isValid: false,
        errors: ['Invalid data type', 'Missing required fields'],
        data: null
      });

      const result = dataProcessor.processSensorData(invalidSensorData);

      expect(result).toBe(false);
      expect(mockedValidateSensorData).toHaveBeenCalledWith(invalidSensorData);
      expect(mockWorkDataStore.updateSensorData).not.toHaveBeenCalled();
      expect(mockWorkDataStore.addNotification).toHaveBeenCalledWith({
        type: 'error',
        title: 'データ検証エラー',
        message: 'Invalid data type, Missing required fields'
      });
    });

    it('should handle store update failure', () => {
      const validSensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      mockedValidateSensorData.mockReturnValue({
        isValid: true,
        errors: [],
        data: validSensorData
      });

      mockWorkDataStore.updateSensorData.mockReturnValue(false);

      const result = dataProcessor.processSensorData(validSensorData);

      expect(result).toBe(false);
      expect(mockWorkDataStore.addNotification).toHaveBeenCalledWith({
        type: 'error',
        title: 'データ更新エラー',
        message: 'センサーデータの更新に失敗しました'
      });
    });

    it('should handle processing errors gracefully', () => {
      const sensorData = { type: 'sensor_data' };

      mockedValidateSensorData.mockImplementation(() => {
        throw new Error('Validation error');
      });

      const result = dataProcessor.processSensorData(sensorData);

      expect(result).toBe(false);
      expect(mockWorkDataStore.addNotification).toHaveBeenCalledWith({
        type: 'error',
        title: 'データ処理エラー',
        message: 'センサーデータの処理中にエラーが発生しました: Validation error'
      });
    });
  });

  describe('updateThresholdSettings', () => {
    it('should update threshold settings successfully', () => {
      const newSettings = {
        screwThreshold: 10,
        boltThreshold: 5
      };

      mockWorkDataStore.updateThresholdSettings.mockReturnValue(true);

      const result = dataProcessor.updateThresholdSettings(newSettings);

      expect(result).toBe(true);
      expect(mockWorkDataStore.updateThresholdSettings).toHaveBeenCalledWith(newSettings);
    });

    it('should handle threshold update failure', () => {
      const newSettings = {
        screwThreshold: 10,
        boltThreshold: 5
      };

      mockWorkDataStore.updateThresholdSettings.mockReturnValue(false);

      const result = dataProcessor.updateThresholdSettings(newSettings);

      expect(result).toBe(false);
      expect(mockWorkDataStore.addNotification).toHaveBeenCalledWith({
        type: 'error',
        title: '設定更新エラー',
        message: '閾値設定の更新に失敗しました'
      });
    });

    it('should validate threshold values', () => {
      const invalidSettings = {
        screwThreshold: -1,
        boltThreshold: 0
      };

      const result = dataProcessor.updateThresholdSettings(invalidSettings);

      expect(result).toBe(false);
      expect(mockWorkDataStore.addNotification).toHaveBeenCalledWith({
        type: 'error',
        title: '設定検証エラー',
        message: '閾値は1以上の値を設定してください'
      });
    });
  });

  describe('updateConnectionStatus', () => {
    it('should update connection status', () => {
      dataProcessor.updateConnectionStatus(true);
      expect(mockWorkDataStore.updateConnectionStatus).toHaveBeenCalledWith(true);

      dataProcessor.updateConnectionStatus(false);
      expect(mockWorkDataStore.updateConnectionStatus).toHaveBeenCalledWith(false);
    });
  });

  describe('getCurrentState', () => {
    it('should return current state from store', () => {
      const mockState = {
        currentWorkData: null,
        thresholdSettings: { screwThreshold: 5, boltThreshold: 3 },
        workHistory: [],
        statistics: { totalWorkTime: 0, completedTasks: 0, averageEfficiency: 0 },
        isConnected: false,
        notifications: []
      };

      mockWorkDataStore.getState.mockReturnValue(mockState);

      const result = dataProcessor.getCurrentState();

      expect(result).toEqual(mockState);
      expect(mockWorkDataStore.getState).toHaveBeenCalled();
    });
  });

  describe('getProcessingStatistics', () => {
    it('should return processing statistics', () => {
      // Process some data to generate statistics
      const validSensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      mockedValidateSensorData.mockReturnValue({
        isValid: true,
        errors: [],
        data: validSensorData
      });

      mockWorkDataStore.updateSensorData.mockReturnValue(true);

      // Process multiple data points
      dataProcessor.processSensorData(validSensorData);
      dataProcessor.processSensorData(validSensorData);

      // Process invalid data
      mockedValidateSensorData.mockReturnValue({
        isValid: false,
        errors: ['Invalid data'],
        data: null
      });

      dataProcessor.processSensorData({ type: 'invalid' });

      const stats = dataProcessor.getProcessingStatistics();

      expect(stats.totalProcessed).toBe(3);
      expect(stats.successfullyProcessed).toBe(2);
      expect(stats.validationErrors).toBe(1);
      expect(stats.processingErrors).toBe(0);
      expect(stats.successRate).toBeCloseTo(66.67, 1);
    });
  });

  describe('resetStatistics', () => {
    it('should reset processing statistics', () => {
      // Process some data first
      const validSensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      mockedValidateSensorData.mockReturnValue({
        isValid: true,
        errors: [],
        data: validSensorData
      });

      mockWorkDataStore.updateSensorData.mockReturnValue(true);
      dataProcessor.processSensorData(validSensorData);

      // Reset statistics
      dataProcessor.resetStatistics();

      const stats = dataProcessor.getProcessingStatistics();

      expect(stats.totalProcessed).toBe(0);
      expect(stats.successfullyProcessed).toBe(0);
      expect(stats.validationErrors).toBe(0);
      expect(stats.processingErrors).toBe(0);
      expect(stats.successRate).toBe(0);
    });
  });

  describe('destroy', () => {
    it('should clean up resources', () => {
      dataProcessor.destroy();
      expect(mockWorkDataStore.destroy).toHaveBeenCalled();
    });
  });
});