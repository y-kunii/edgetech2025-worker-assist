import { 
  WorkerStatus, 
  RobotStatus, 
  WorkData, 
  SensorData, 
  ThresholdSettings,
  RobotCommand,
  RobotCommandResponse,
  WorkStatistics,
  WorkHistory,
  EfficiencyMetrics,
  Notification,
  ConnectionQuality
} from '../types';

/**
 * バリデーションエラーの型定義
 */
export interface ValidationError {
  field: string;
  message: string;
  value?: any;
}

/**
 * バリデーション結果の型定義
 */
export interface ValidationResult<T> {
  isValid: boolean;
  data?: T;
  errors: ValidationError[];
}

/**
 * WorkerStatusの検証
 */
export function validateWorkerStatus(status: any): ValidationResult<WorkerStatus> {
  const validStatuses: WorkerStatus[] = ['waiting', 'screw_tightening', 'tool_handover', 'bolt_tightening', 'absent'];
  
  if (typeof status !== 'string') {
    return {
      isValid: false,
      errors: [{ field: 'workerStatus', message: 'Worker status must be a string', value: status }]
    };
  }
  
  if (!validStatuses.includes(status as WorkerStatus)) {
    return {
      isValid: false,
      errors: [{ 
        field: 'workerStatus', 
        message: `Invalid worker status. Must be one of: ${validStatuses.join(', ')}`, 
        value: status 
      }]
    };
  }
  
  return {
    isValid: true,
    data: status as WorkerStatus,
    errors: []
  };
}

/**
 * RobotStatusの検証
 */
export function validateRobotStatus(status: any): ValidationResult<RobotStatus> {
  const errors: ValidationError[] = [];
  
  if (!status || typeof status !== 'object') {
    return {
      isValid: false,
      errors: [{ field: 'robotStatus', message: 'Robot status must be an object', value: status }]
    };
  }
  
  // state の検証
  const validStates = ['waiting', 'operating'];
  if (typeof status.state !== 'string' || !validStates.includes(status.state)) {
    errors.push({
      field: 'robotStatus.state',
      message: `Invalid robot state. Must be one of: ${validStates.join(', ')}`,
      value: status.state
    });
  }
  
  // grip の検証
  const validGrips = ['open', 'closed'];
  if (typeof status.grip !== 'string' || !validGrips.includes(status.grip)) {
    errors.push({
      field: 'robotStatus.grip',
      message: `Invalid robot grip. Must be one of: ${validGrips.join(', ')}`,
      value: status.grip
    });
  }
  
  if (errors.length > 0) {
    return { isValid: false, errors };
  }
  
  return {
    isValid: true,
    data: status as RobotStatus,
    errors: []
  };
}

/**
 * SensorDataの検証
 */
export function validateSensorData(data: any): ValidationResult<SensorData> {
  const errors: ValidationError[] = [];
  
  if (!data || typeof data !== 'object') {
    return {
      isValid: false,
      errors: [{ field: 'sensorData', message: 'Sensor data must be an object', value: data }]
    };
  }
  
  // type の検証
  if (typeof data.type !== 'string') {
    errors.push({
      field: 'type',
      message: 'Type must be a string',
      value: data.type
    });
  }
  
  // timestamp の検証
  if (typeof data.timestamp !== 'string') {
    errors.push({
      field: 'timestamp',
      message: 'Timestamp must be a string',
      value: data.timestamp
    });
  } else {
    const timestamp = new Date(data.timestamp);
    if (isNaN(timestamp.getTime())) {
      errors.push({
        field: 'timestamp',
        message: 'Invalid timestamp format',
        value: data.timestamp
      });
    }
  }
  
  // data オブジェクトの検証
  if (!data.data || typeof data.data !== 'object') {
    errors.push({
      field: 'data',
      message: 'Data field must be an object',
      value: data.data
    });
  } else {
    const sensorDataObj = data.data;
    
    // image の検証（オプショナル）
    if (sensorDataObj.image !== undefined && typeof sensorDataObj.image !== 'string') {
      errors.push({
        field: 'data.image',
        message: 'Image must be a string (base64 encoded)',
        value: sensorDataObj.image
      });
    }
    
    // worker_status の検証
    const workerStatusResult = validateWorkerStatus(sensorDataObj.worker_status);
    if (!workerStatusResult.isValid) {
      errors.push(...workerStatusResult.errors.map(err => ({
        ...err,
        field: `data.${err.field}`
      })));
    }
    
    // robot_status の検証
    const robotStatusResult = validateRobotStatus(sensorDataObj.robot_status);
    if (!robotStatusResult.isValid) {
      errors.push(...robotStatusResult.errors.map(err => ({
        ...err,
        field: `data.${err.field}`
      })));
    }
    
    // screw_count の検証
    if (typeof sensorDataObj.screw_count !== 'number' || sensorDataObj.screw_count < 0) {
      errors.push({
        field: 'data.screw_count',
        message: 'Screw count must be a non-negative number',
        value: sensorDataObj.screw_count
      });
    }
    
    // bolt_count の検証
    if (typeof sensorDataObj.bolt_count !== 'number' || sensorDataObj.bolt_count < 0) {
      errors.push({
        field: 'data.bolt_count',
        message: 'Bolt count must be a non-negative number',
        value: sensorDataObj.bolt_count
      });
    }
    
    // work_step の検証
    const workStepResult = validateWorkerStatus(sensorDataObj.work_step);
    if (!workStepResult.isValid) {
      errors.push(...workStepResult.errors.map(err => ({
        ...err,
        field: `data.work_step`
      })));
    }
  }
  
  if (errors.length > 0) {
    return { isValid: false, errors };
  }
  
  return {
    isValid: true,
    data: data as SensorData,
    errors: []
  };
}

/**
 * ThresholdSettingsの検証
 */
export function validateThresholdSettings(settings: any): ValidationResult<ThresholdSettings> {
  const errors: ValidationError[] = [];
  
  if (!settings || typeof settings !== 'object') {
    return {
      isValid: false,
      errors: [{ field: 'thresholdSettings', message: 'Threshold settings must be an object', value: settings }]
    };
  }
  
  // screwThreshold の検証
  if (typeof settings.screwThreshold !== 'number' || settings.screwThreshold <= 0) {
    errors.push({
      field: 'screwThreshold',
      message: 'Screw threshold must be a positive number',
      value: settings.screwThreshold
    });
  }
  
  // boltThreshold の検証
  if (typeof settings.boltThreshold !== 'number' || settings.boltThreshold <= 0) {
    errors.push({
      field: 'boltThreshold',
      message: 'Bolt threshold must be a positive number',
      value: settings.boltThreshold
    });
  }
  
  if (errors.length > 0) {
    return { isValid: false, errors };
  }
  
  return {
    isValid: true,
    data: settings as ThresholdSettings,
    errors: []
  };
}

/**
 * RobotCommandの検証
 */
export function validateRobotCommand(command: any): ValidationResult<RobotCommand> {
  const errors: ValidationError[] = [];
  
  if (!command || typeof command !== 'object') {
    return {
      isValid: false,
      errors: [{ field: 'robotCommand', message: 'Robot command must be an object', value: command }]
    };
  }
  
  // id の検証
  if (typeof command.id !== 'string' || command.id.trim() === '') {
    errors.push({
      field: 'id',
      message: 'Command ID must be a non-empty string',
      value: command.id
    });
  }
  
  // type の検証
  const validTypes = ['tool_handover', 'next_task'];
  if (typeof command.type !== 'string' || !validTypes.includes(command.type)) {
    errors.push({
      field: 'type',
      message: `Command type must be one of: ${validTypes.join(', ')}`,
      value: command.type
    });
  }
  
  // timestamp の検証
  if (typeof command.timestamp !== 'string') {
    errors.push({
      field: 'timestamp',
      message: 'Timestamp must be a string',
      value: command.timestamp
    });
  } else {
    const timestamp = new Date(command.timestamp);
    if (isNaN(timestamp.getTime())) {
      errors.push({
        field: 'timestamp',
        message: 'Invalid timestamp format',
        value: command.timestamp
      });
    }
  }
  
  // parameters の検証（オプショナル）
  if (command.parameters !== undefined && (typeof command.parameters !== 'object' || command.parameters === null)) {
    errors.push({
      field: 'parameters',
      message: 'Parameters must be an object if provided',
      value: command.parameters
    });
  }
  
  if (errors.length > 0) {
    return { isValid: false, errors };
  }
  
  return {
    isValid: true,
    data: command as RobotCommand,
    errors: []
  };
}

/**
 * 数値の範囲検証
 */
export function validateNumberRange(value: any, min: number, max: number, fieldName: string): ValidationError | null {
  if (typeof value !== 'number') {
    return {
      field: fieldName,
      message: `${fieldName} must be a number`,
      value
    };
  }
  
  if (value < min || value > max) {
    return {
      field: fieldName,
      message: `${fieldName} must be between ${min} and ${max}`,
      value
    };
  }
  
  return null;
}

/**
 * 日付の検証
 */
export function validateDate(value: any, fieldName: string): ValidationError | null {
  if (!(value instanceof Date)) {
    return {
      field: fieldName,
      message: `${fieldName} must be a Date object`,
      value
    };
  }
  
  if (isNaN(value.getTime())) {
    return {
      field: fieldName,
      message: `${fieldName} must be a valid date`,
      value
    };
  }
  
  return null;
}

/**
 * SensorDataをWorkDataに変換
 */
export function convertSensorDataToWorkData(sensorData: SensorData): WorkData {
  return {
    timestamp: new Date(sensorData.timestamp),
    workerStatus: sensorData.data.worker_status,
    robotStatus: sensorData.data.robot_status,
    screwCount: sensorData.data.screw_count,
    boltCount: sensorData.data.bolt_count,
    image: sensorData.data.image
  };
}

/**
 * データのサニタイズ（危険な値を除去）
 */
export function sanitizeData<T>(data: T): T {
  if (data === null || data === undefined) {
    return data;
  }
  
  if (typeof data === 'string') {
    // HTMLタグやスクリプトを除去
    return data.replace(/<[^>]*>/g, '').replace(/javascript:/gi, '') as T;
  }
  
  if (typeof data === 'object' && !Array.isArray(data)) {
    const sanitized = {} as T;
    for (const [key, value] of Object.entries(data)) {
      (sanitized as any)[key] = sanitizeData(value);
    }
    return sanitized;
  }
  
  if (Array.isArray(data)) {
    return data.map(item => sanitizeData(item)) as T;
  }
  
  return data;
}