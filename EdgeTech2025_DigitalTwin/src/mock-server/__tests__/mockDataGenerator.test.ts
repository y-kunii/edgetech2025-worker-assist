import { MockDataGenerator, WorkerStatus } from '../mockDataGenerator';

describe('MockDataGenerator', () => {
  let generator: MockDataGenerator;

  beforeEach(() => {
    generator = new MockDataGenerator();
  });

  test('should generate valid sensor data', () => {
    const data = generator.generateData();
    
    expect(data.type).toBe('sensor_data');
    expect(data.timestamp).toBeDefined();
    expect(data.data).toBeDefined();
    expect(data.data.worker_status).toBeDefined();
    expect(data.data.robot_status).toBeDefined();
    expect(data.data.screw_count).toBeGreaterThanOrEqual(0);
    expect(data.data.bolt_count).toBeGreaterThanOrEqual(0);
    expect(data.data.image).toBeDefined();
  });

  test('should have valid worker status values', () => {
    const data = generator.generateData();
    const validStatuses: WorkerStatus[] = [
      'waiting', 
      'screw_tightening', 
      'tool_handover', 
      'bolt_tightening', 
      'absent'
    ];
    
    expect(validStatuses).toContain(data.data.worker_status);
  });

  test('should have valid robot status', () => {
    const data = generator.generateData();
    
    expect(['waiting', 'operating']).toContain(data.data.robot_status.state);
    expect(['open', 'closed']).toContain(data.data.robot_status.grip);
  });

  test('should generate base64 image data', () => {
    const data = generator.generateData();
    
    expect(data.data.image).toMatch(/^data:image\/svg\+xml;base64,/);
  });

  test('should reset counts correctly', () => {
    // Generate some data to increment counts
    for (let i = 0; i < 5; i++) {
      generator.generateData();
    }
    
    generator.resetCounts();
    const data = generator.generateData();
    
    expect(data.data.screw_count).toBe(0);
    expect(data.data.bolt_count).toBe(0);
  });

  test('should set worker status manually', () => {
    generator.setWorkerStatus('screw_tightening');
    const data = generator.generateData();
    
    expect(data.data.worker_status).toBe('screw_tightening');
  });
});