import {
  validateSensorData,
  convertSensorDataToWorkData,
  validateThresholdSettings,
  validateWorkData,
  sanitizeImageData,
  validateTimestamp,
  validateWorkerStatus,
  validateRobotStatus
} from '../utils/validation';
import { WorkerStatus, RobotStatus } from '../types';

describe('validation utilities', () => {
  describe('validateSensorData', () => {
    it('should validate correct sensor data', () => {
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

    it('should reject data with invalid type', () => {
      const invalidData = {
        type: 'invalid_type',
        timestamp: '2024-01-01T12:00:00Z',
        data: {}
      };

      const result = validateSensorData(invalidData);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Invalid message type. Expected "sensor_data"');
    });

    it('should reject data with invalid timestamp', () => {
      const invalidData = {
        type: 'sensor_data',
        timestamp: 'invalid-timestamp',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      const result = validateSensorData(invalidData);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Invalid timestamp format');
    });

    it('should reject data with invalid worker status', () => {
      const invalidData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'invalid_status',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      const result = validateSensorData(invalidData);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Invalid worker status');
    });

    it('should reject data with invalid robot status', () => {
      const invalidData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'invalid_state', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      const result = validateSensorData(invalidData);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Invalid robot status');
    });

    it('should reject data with negative counts', () => {
      const invalidData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: -1,
          bolt_count: -2
        }
      };

      const result = validateSensorData(invalidData);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Screw count must be non-negative');
      expect(result.errors).toContain('Bolt count must be non-negative');
    });

    it('should reject data with non-integer counts', () => {
      const invalidData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3.5,
          bolt_count: 'not_a_number'
        }
      };

      const result = validateSensorData(invalidData);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Screw count must be an integer');
      expect(result.errors).toContain('Bolt count must be an integer');
    });

    it('should handle missing optional fields', () => {
      const dataWithoutOptionalFields = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
          // image and work_step are optional
        }
      };

      const result = validateSensorData(dataWithoutOptionalFields);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });
  });

  describe('convertSensorDataToWorkData', () => {
    it('should convert valid sensor data to work data', () => {
      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          image: 'base64encodedimage',
          worker_status: 'screw_tightening' as WorkerStatus,
          robot_status: {
            state: 'waiting' as const,
            grip: 'open' as const
          },
          screw_count: 3,
          bolt_count: 1,
          work_step: 'screw_tightening' as WorkerStatus
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

    it('should handle missing optional fields', () => {
      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'bolt_tightening' as WorkerStatus,
          robot_status: {
            state: 'operating' as const,
            grip: 'closed' as const
          },
          screw_count: 5,
          bolt_count: 2
        }
      };

      const workData = convertSensorDataToWorkData(sensorData);
      
      expect(workData.image).toBeUndefined();
      expect(workData.timestamp).toBeInstanceOf(Date);
      expect(workData.workerStatus).toBe('bolt_tightening');
    });
  });

  describe('validateThresholdSettings', () => {
    it('should validate correct threshold settings', () => {
      const validSettings = {
        screwThreshold: 5,
        boltThreshold: 3
      };

      const result = validateThresholdSettings(validSettings);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('should reject negative thresholds', () => {
      const invalidSettings = {
        screwThreshold: -1,
        boltThreshold: -2
      };

      const result = validateThresholdSettings(invalidSettings);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Screw threshold must be positive');
      expect(result.errors).toContain('Bolt threshold must be positive');
    });

    it('should reject zero thresholds', () => {
      const invalidSettings = {
        screwThreshold: 0,
        boltThreshold: 0
      };

      const result = validateThresholdSettings(invalidSettings);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Screw threshold must be positive');
      expect(result.errors).toContain('Bolt threshold must be positive');
    });

    it('should reject non-integer thresholds', () => {
      const invalidSettings = {
        screwThreshold: 3.5,
        boltThreshold: 'not_a_number' as any
      };

      const result = validateThresholdSettings(invalidSettings);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Screw threshold must be an integer');
      expect(result.errors).toContain('Bolt threshold must be an integer');
    });

    it('should reject excessively high thresholds', () => {
      const invalidSettings = {
        screwThreshold: 1001,
        boltThreshold: 1001
      };

      const result = validateThresholdSettings(invalidSettings);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Screw threshold must be 1000 or less');
      expect(result.errors).toContain('Bolt threshold must be 1000 or less');
    });
  });

  describe('validateWorkData', () => {
    it('should validate correct work data', () => {
      const validWorkData = {
        timestamp: new Date(),
        workerStatus: 'screw_tightening' as WorkerStatus,
        robotStatus: { state: 'waiting' as const, grip: 'open' as const },
        screwCount: 3,
        boltCount: 1,
        image: 'base64encodedimage'
      };

      const result = validateWorkData(validWorkData);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('should reject work data with invalid timestamp', () => {
      const invalidWorkData = {
        timestamp: 'not_a_date' as any,
        workerStatus: 'screw_tightening' as WorkerStatus,
        robotStatus: { state: 'waiting' as const, grip: 'open' as const },
        screwCount: 3,
        boltCount: 1
      };

      const result = validateWorkData(invalidWorkData);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Timestamp must be a Date object');
    });
  });

  describe('sanitizeData', () => {
    it('should sanitize string data', () => {
      const dirtyString = '<script>alert("xss")</script>Hello World';
      
      const result = sanitizeData(dirtyString);
      
      expect(result).toBe('Hello World');
    });

    it('should sanitize object data', () => {
      const dirtyObject = {
        name: '<script>alert("xss")</script>John',
        message: 'javascript:alert("xss")'
      };
      
      const result = sanitizeData(dirtyObject);
      
      expect(result.name).toBe('John');
      expect(result.message).toBe('alert("xss")');
    });

    it('should handle null and undefined', () => {
      expect(sanitizeData(null)).toBe(null);
      expect(sanitizeData(undefined)).toBe(undefined);
    });

    it('should sanitize arrays', () => {
      const dirtyArray = ['<script>test</script>', 'clean', 'javascript:alert()'];
      
      const result = sanitizeData(dirtyArray);
      
      expect(result).toEqual(['test', 'clean', 'alert()']);
    });
  });

  describe('validateWorkerStatus', () => {
    it('should validate all valid worker statuses', () => {
      const validStatuses: WorkerStatus[] = [
        'waiting',
        'screw_tightening',
        'tool_handover',
        'bolt_tightening',
        'absent'
      ];

      validStatuses.forEach(status => {
        const result = validateWorkerStatus(status);
        expect(result.isValid).toBe(true);
        expect(result.data).toBe(status);
      });
    });

    it('should reject invalid worker status', () => {
      const invalidStatus = 'invalid_status' as WorkerStatus;
      
      const result = validateWorkerStatus(invalidStatus);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toHaveLength(1);
    });

    it('should reject non-string input', () => {
      const result = validateWorkerStatus(123 as any);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toHaveLength(1);
    });
  });

  describe('validateRobotStatus', () => {
    it('should validate valid robot status', () => {
      const validStatus: RobotStatus = {
        state: 'waiting',
        grip: 'open'
      };
      
      const result = validateRobotStatus(validStatus);
      
      expect(result.isValid).toBe(true);
      expect(result.data).toEqual(validStatus);
    });

    it('should validate all valid robot states and grips', () => {
      const validStates = ['waiting', 'operating'];
      const validGrips = ['open', 'closed'];

      validStates.forEach(state => {
        validGrips.forEach(grip => {
          const status: RobotStatus = { state: state as any, grip: grip as any };
          const result = validateRobotStatus(status);
          expect(result.isValid).toBe(true);
        });
      });
    });

    it('should reject invalid robot state', () => {
      const invalidStatus: RobotStatus = {
        state: 'invalid_state' as any,
        grip: 'open'
      };
      
      const result = validateRobotStatus(invalidStatus);
      
      expect(result.isValid).toBe(false);
      expect(result.errors.length).toBeGreaterThan(0);
    });

    it('should reject invalid robot grip', () => {
      const invalidStatus: RobotStatus = {
        state: 'waiting',
        grip: 'invalid_grip' as any
      };
      
      const result = validateRobotStatus(invalidStatus);
      
      expect(result.isValid).toBe(false);
      expect(result.errors.length).toBeGreaterThan(0);
    });

    it('should reject missing fields', () => {
      const incompleteStatus = { state: 'waiting' } as any;
      
      const result = validateRobotStatus(incompleteStatus);
      
      expect(result.isValid).toBe(false);
      expect(result.errors.length).toBeGreaterThan(0);
    });

    it('should reject non-object input', () => {
      const result = validateRobotStatus('not_an_object' as any);
      
      expect(result.isValid).toBe(false);
      expect(result.errors.length).toBeGreaterThan(0);
    });
  });
});