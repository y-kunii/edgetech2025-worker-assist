import { DataProcessor, DataValidationError } from '../../src/main/data-processor';
import { StatusData } from '../../src/shared/types';

// Mock the database and statistics services
jest.mock('../../src/main/database-service');
jest.mock('../../src/main/statistics-service');

describe('DataProcessor', () => {
  beforeAll(() => {
    // Initialize the data processor
    DataProcessor.initialize();
  });

  describe('Data Validation', () => {
    const validStatusData = {
      worker_status: 'Working',
      space_status: 'Screw_tightening',
      robot_status: {
        state: 'moving',
        grip: 'open'
      },
      timestamp: '20241023120000',
      tool_delivery: 1,
      status: 'Working'
    };

    test('should validate correct status data', () => {
      const result = DataProcessor.validateStatusData(validStatusData);
      expect(result).toEqual(validStatusData);
    });

    test('should throw error for invalid worker_status', () => {
      const invalidData = { ...validStatusData, worker_status: 'InvalidStatus' };
      
      expect(() => {
        DataProcessor.validateStatusData(invalidData);
      }).toThrow(DataValidationError);
    });

    test('should throw error for invalid space_status', () => {
      const invalidData = { ...validStatusData, space_status: 'InvalidSpace' };
      
      expect(() => {
        DataProcessor.validateStatusData(invalidData);
      }).toThrow(DataValidationError);
    });

    test('should throw error for invalid robot_status', () => {
      const invalidData = { ...validStatusData, robot_status: { state: 'moving' } }; // Missing grip
      
      expect(() => {
        DataProcessor.validateStatusData(invalidData);
      }).toThrow(DataValidationError);
    });

    test('should throw error for invalid robot grip', () => {
      const invalidData = { 
        ...validStatusData, 
        robot_status: { state: 'moving', grip: 'invalid' }
      };
      
      expect(() => {
        DataProcessor.validateStatusData(invalidData);
      }).toThrow(DataValidationError);
    });

    test('should throw error for invalid timestamp format', () => {
      const invalidData = { ...validStatusData, timestamp: '2024-10-23 12:00:00' };
      
      expect(() => {
        DataProcessor.validateStatusData(invalidData);
      }).toThrow(DataValidationError);
    });

    test('should throw error for invalid timestamp length', () => {
      const invalidData = { ...validStatusData, timestamp: '202410231200' }; // Too short
      
      expect(() => {
        DataProcessor.validateStatusData(invalidData);
      }).toThrow(DataValidationError);
    });

    test('should throw error for invalid date values', () => {
      const invalidData = { ...validStatusData, timestamp: '20241301120000' }; // Invalid month
      
      expect(() => {
        DataProcessor.validateStatusData(invalidData);
      }).toThrow(DataValidationError);
    });

    test('should throw error for invalid tool_delivery', () => {
      const invalidData = { ...validStatusData, tool_delivery: 'not_a_number' };
      
      expect(() => {
        DataProcessor.validateStatusData(invalidData);
      }).toThrow(DataValidationError);
    });

    test('should throw error for non-object data', () => {
      expect(() => {
        DataProcessor.validateStatusData('not an object');
      }).toThrow(DataValidationError);
    });

    test('should throw error for null data', () => {
      expect(() => {
        DataProcessor.validateStatusData(null);
      }).toThrow(DataValidationError);
    });
  });

  describe('JSON Parsing and Validation', () => {
    const validJsonData = JSON.stringify({
      worker_status: 'Working',
      space_status: 'Screw_tightening',
      robot_status: {
        state: 'moving',
        grip: 'open'
      },
      timestamp: '20241023120000',
      tool_delivery: 1,
      status: 'Working'
    });

    test('should parse and validate correct JSON data', () => {
      const result = DataProcessor.validateAndParseStatusData(validJsonData);
      expect(result.worker_status).toBe('Working');
      expect(result.space_status).toBe('Screw_tightening');
    });

    test('should throw error for invalid JSON', () => {
      expect(() => {
        DataProcessor.validateAndParseStatusData('invalid json');
      }).toThrow(DataValidationError);
    });

    test('should throw error for JSON with invalid data', () => {
      const invalidJson = JSON.stringify({
        worker_status: 'InvalidStatus',
        space_status: 'Screw_tightening',
        robot_status: { state: 'moving', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 1,
        status: 'Working'
      });

      expect(() => {
        DataProcessor.validateAndParseStatusData(invalidJson);
      }).toThrow(DataValidationError);
    });
  });

  describe('Timestamp Utilities', () => {
    test('should parse timestamp correctly', () => {
      const timestamp = '20241023143000';
      const date = DataProcessor.parseTimestamp(timestamp);
      
      expect(date.getFullYear()).toBe(2024);
      expect(date.getMonth()).toBe(9); // October (0-based)
      expect(date.getDate()).toBe(23);
      expect(date.getHours()).toBe(14);
      expect(date.getMinutes()).toBe(30);
      expect(date.getSeconds()).toBe(0);
    });

    test('should format timestamp correctly', () => {
      const date = new Date(2024, 9, 23, 14, 30, 0); // October 23, 2024 14:30:00
      const timestamp = DataProcessor.formatTimestamp(date);
      
      expect(timestamp).toBe('20241023143000');
    });

    test('should handle single digit values in formatting', () => {
      const date = new Date(2024, 0, 5, 8, 5, 3); // January 5, 2024 08:05:03
      const timestamp = DataProcessor.formatTimestamp(date);
      
      expect(timestamp).toBe('20240105080503');
    });
  });

  describe('Work Task Determination', () => {
    test('should determine Screw_tightening task', () => {
      const statusData: StatusData = {
        worker_status: 'Working',
        space_status: 'Screw_tightening',
        robot_status: { state: 'moving', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 1,
        status: 'Working'
      };

      const task = DataProcessor.determineWorkTask(statusData);
      expect(task).toBe('Screw_tightening');
    });

    test('should determine Building_blocks task', () => {
      const statusData: StatusData = {
        worker_status: 'Working',
        space_status: 'Building_blocks',
        robot_status: { state: 'moving', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 1,
        status: 'Working'
      };

      const task = DataProcessor.determineWorkTask(statusData);
      expect(task).toBe('Building_blocks');
    });

    test('should determine Survey_responses task', () => {
      const statusData: StatusData = {
        worker_status: 'Working',
        space_status: 'Survey_responses',
        robot_status: { state: 'moving', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 1,
        status: 'Working'
      };

      const task = DataProcessor.determineWorkTask(statusData);
      expect(task).toBe('Survey_responses');
    });

    test('should determine Waiting task when space is Nothing and worker is Waiting', () => {
      const statusData: StatusData = {
        worker_status: 'Waiting',
        space_status: 'Nothing',
        robot_status: { state: 'idle', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 0,
        status: 'Waiting'
      };

      const task = DataProcessor.determineWorkTask(statusData);
      expect(task).toBe('Waiting');
    });

    test('should determine Nothing task when space is Nothing and worker is not Waiting', () => {
      const statusData: StatusData = {
        worker_status: 'Absent',
        space_status: 'Nothing',
        robot_status: { state: 'idle', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 0,
        status: 'Absent'
      };

      const task = DataProcessor.determineWorkTask(statusData);
      expect(task).toBe('Nothing');
    });
  });

  describe('Status Change Detection', () => {
    const baseStatus: StatusData = {
      worker_status: 'Waiting',
      space_status: 'Nothing',
      robot_status: { state: 'idle', grip: 'open' },
      timestamp: '20241023120000',
      tool_delivery: 0,
      status: 'Waiting'
    };

    test('should detect initial status (no previous status)', () => {
      const change = DataProcessor.detectStatusChange(null, baseStatus);
      
      expect(change.hasChanged).toBe(true);
      expect(change.changedFields).toContain('initial');
      expect(change.workTaskChanged).toBe(true);
      expect(change.currentTask).toBe('Waiting');
    });

    test('should detect no change when status is identical', () => {
      const change = DataProcessor.detectStatusChange(baseStatus, baseStatus);
      
      expect(change.hasChanged).toBe(false);
      expect(change.changedFields).toHaveLength(0);
      expect(change.workTaskChanged).toBe(false);
    });

    test('should detect worker status change', () => {
      const newStatus = { ...baseStatus, worker_status: 'Working' as const };
      const change = DataProcessor.detectStatusChange(baseStatus, newStatus);
      
      expect(change.hasChanged).toBe(true);
      expect(change.changedFields).toContain('worker_status');
    });

    test('should detect space status change', () => {
      const newStatus = { ...baseStatus, space_status: 'Screw_tightening' as const };
      const change = DataProcessor.detectStatusChange(baseStatus, newStatus);
      
      expect(change.hasChanged).toBe(true);
      expect(change.changedFields).toContain('space_status');
      expect(change.workTaskChanged).toBe(true);
      expect(change.previousTask).toBe('Waiting');
      expect(change.currentTask).toBe('Screw_tightening');
    });

    test('should detect robot status changes', () => {
      const newStatus = { 
        ...baseStatus, 
        robot_status: { state: 'moving', grip: 'closed' }
      };
      const change = DataProcessor.detectStatusChange(baseStatus, newStatus);
      
      expect(change.hasChanged).toBe(true);
      expect(change.changedFields).toContain('robot_status.state');
      expect(change.changedFields).toContain('robot_status.grip');
    });

    test('should detect tool delivery change', () => {
      const newStatus = { ...baseStatus, tool_delivery: 1 };
      const change = DataProcessor.detectStatusChange(baseStatus, newStatus);
      
      expect(change.hasChanged).toBe(true);
      expect(change.changedFields).toContain('tool_delivery');
    });

    test('should detect multiple field changes', () => {
      const newStatus = { 
        ...baseStatus, 
        worker_status: 'Working' as const,
        space_status: 'Screw_tightening' as const,
        tool_delivery: 1
      };
      const change = DataProcessor.detectStatusChange(baseStatus, newStatus);
      
      expect(change.hasChanged).toBe(true);
      expect(change.changedFields).toContain('worker_status');
      expect(change.changedFields).toContain('space_status');
      expect(change.changedFields).toContain('tool_delivery');
      expect(change.workTaskChanged).toBe(true);
    });
  });

  describe('Error Handling', () => {
    test('DataValidationError should include field information', () => {
      try {
        throw new DataValidationError('Test error', 'test_field');
      } catch (error) {
        expect(error).toBeInstanceOf(DataValidationError);
        expect((error as DataValidationError).field).toBe('test_field');
        expect((error as DataValidationError).message).toBe('Test error');
      }
    });

    test('DataValidationError should work without field information', () => {
      try {
        throw new DataValidationError('Test error');
      } catch (error) {
        expect(error).toBeInstanceOf(DataValidationError);
        expect((error as DataValidationError).field).toBeUndefined();
      }
    });
  });
});