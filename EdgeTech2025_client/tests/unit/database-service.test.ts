import { StatusData, CommandData } from '../../src/shared/types';
import Database from 'better-sqlite3';
import fs from 'fs';
import path from 'path';

// Mock electron app
jest.mock('electron', () => ({
  app: {
    getPath: jest.fn(() => '/tmp/test-db')
  }
}));

// Mock constants
jest.mock('../../src/shared/constants', () => ({
  APP_CONFIG: {
    DB_FILE_NAME: 'test.sqlite',
    DATA_RETENTION_DAYS: 30
  }
}));

describe('DatabaseService', () => {
  let dbService: DatabaseService;
  let testDbPath: string;

  beforeEach(() => {
    // Create a unique temporary database for each test
    testDbPath = path.join('/tmp', `test-db-${Date.now()}-${Math.random()}.sqlite`);
    
    // Mock the database path
    const mockApp = require('electron').app;
    mockApp.getPath.mockReturnValue(path.dirname(testDbPath));
    
    // Update the mock to use the new path
    const mockConstants = require('../../src/shared/constants');
    mockConstants.APP_CONFIG.DB_FILE_NAME = path.basename(testDbPath);
    
    // Import and create database service after mocking
    const { DatabaseService } = require('../../src/main/database-service');
    dbService = new DatabaseService();
  });

  afterEach(() => {
    // Clean up
    if (dbService) {
      dbService.close();
    }
    
    // Remove test database file
    if (fs.existsSync(testDbPath)) {
      fs.unlinkSync(testDbPath);
    }
  });

  describe('Database Initialization', () => {
    test('should initialize database with required tables', () => {
      const tables = dbService.db.prepare(`
        SELECT name FROM sqlite_master WHERE type='table'
      `).all();

      const tableNames = tables.map((t: any) => t.name);
      expect(tableNames).toContain('status_history');
      expect(tableNames).toContain('command_history');
      expect(tableNames).toContain('work_statistics');
    });

    test('should create indexes for performance', () => {
      const indexes = dbService.db.prepare(`
        SELECT name FROM sqlite_master WHERE type='index'
      `).all();

      const indexNames = indexes.map((i: any) => i.name);
      expect(indexNames).toContain('idx_status_timestamp');
      expect(indexNames).toContain('idx_command_timestamp');
    });
  });

  describe('Status Data Operations', () => {
    const testStatusData: StatusData = {
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

    test('should save status data successfully', () => {
      const result = dbService.saveStatusData(testStatusData);
      expect(result).toBe(true);

      // Verify data was saved
      const saved = dbService.getLatestStatus();
      expect(saved).toBeTruthy();
      expect(saved!.worker_status).toBe('Working');
      expect(saved!.space_status).toBe('Screw_tightening');
    });

    test('should retrieve status history', () => {
      // Save multiple records
      const timestamps = ['20241023120000', '20241023120100', '20241023120200'];
      timestamps.forEach(timestamp => {
        dbService.saveStatusData({ ...testStatusData, timestamp });
      });

      const history = dbService.getStatusHistory();
      expect(history).toHaveLength(3);
      expect(history[0].timestamp).toBe('20241023120200'); // Most recent first
    });

    test('should retrieve status history with time range', () => {
      // Save records with different timestamps
      const data1 = { ...testStatusData, timestamp: '20241023120000' };
      const data2 = { ...testStatusData, timestamp: '20241023130000' };
      const data3 = { ...testStatusData, timestamp: '20241023140000' };

      dbService.saveStatusData(data1);
      dbService.saveStatusData(data2);
      dbService.saveStatusData(data3);

      const history = dbService.getStatusHistory('20241023125000', '20241023135000');
      expect(history).toHaveLength(1);
      expect(history[0].timestamp).toBe('20241023130000');
    });

    test('should limit status history results', () => {
      // Save 5 records
      for (let i = 0; i < 5; i++) {
        const timestamp = `2024102312${i.toString().padStart(2, '0')}00`;
        dbService.saveStatusData({ ...testStatusData, timestamp });
      }

      const history = dbService.getStatusHistory(undefined, undefined, 3);
      expect(history).toHaveLength(3);
    });
  });

  describe('Command Data Operations', () => {
    const testCommandData: CommandData = {
      command: 'tool_handover',
      timestamp: '20241023120000'
    };

    test('should save command history successfully', () => {
      const result = dbService.saveCommandHistory(testCommandData, true);
      expect(result).toBe(true);

      // Verify data was saved
      const history = dbService.getCommandHistory(1);
      expect(history).toHaveLength(1);
      expect(history[0].command).toBe('tool_handover');
      expect(history[0].success).toBe(true);
    });

    test('should save failed command history', () => {
      const result = dbService.saveCommandHistory(testCommandData, false);
      expect(result).toBe(true);

      const history = dbService.getCommandHistory(1);
      expect(history[0].success).toBe(false);
    });

    test('should retrieve command history with limit', () => {
      // Save multiple commands
      const commands = ['tool_handover', 'tool_collection', 'wait'];
      commands.forEach((cmd, index) => {
        const timestamp = `2024102312${index.toString().padStart(2, '0')}00`;
        dbService.saveCommandHistory({ command: cmd as any, timestamp }, true);
      });

      const history = dbService.getCommandHistory(2);
      expect(history).toHaveLength(2);
      expect(history[0].command).toBe('wait'); // Most recent first
    });
  });

  describe('Data Validation', () => {
    test('should validate data integrity successfully', () => {
      const testData: StatusData = {
        worker_status: 'Working',
        space_status: 'Screw_tightening',
        robot_status: { state: 'moving', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 1,
        status: 'Working'
      };

      dbService.saveStatusData(testData);
      
      const validation = dbService.validateDataIntegrity();
      expect(validation.isValid).toBe(true);
      expect(validation.errors).toHaveLength(0);
    });

    test('should detect invalid timestamp format', () => {
      // Insert invalid data directly
      dbService.db.prepare(`
        INSERT INTO status_history (timestamp, worker_status, space_status, robot_state, robot_grip, tool_delivery, demo_status)
        VALUES (?, ?, ?, ?, ?, ?, ?)
      `).run('invalid', 'Working', 'Nothing', 'idle', 'open', 0, 'Working');

      const validation = dbService.validateDataIntegrity();
      expect(validation.isValid).toBe(false);
      expect(validation.errors.length).toBeGreaterThan(0);
    });
  });

  describe('Data Cleanup', () => {
    test('should clean up old data', () => {
      // Insert old data (simulate by setting created_at to past date)
      const oldDate = new Date();
      oldDate.setDate(oldDate.getDate() - 40); // 40 days ago

      dbService.db.prepare(`
        INSERT INTO status_history (timestamp, worker_status, space_status, robot_state, robot_grip, tool_delivery, demo_status, created_at)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
      `).run('20241001120000', 'Working', 'Nothing', 'idle', 'open', 0, 'Working', oldDate.toISOString());

      // Insert recent data
      const testData: StatusData = {
        worker_status: 'Working',
        space_status: 'Screw_tightening',
        robot_status: { state: 'moving', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 1,
        status: 'Working'
      };
      dbService.saveStatusData(testData);

      const cleanup = dbService.cleanupOldData();
      expect(cleanup.deletedStatusRecords).toBe(1);

      // Verify recent data is still there
      const remaining = dbService.getStatusHistory();
      expect(remaining).toHaveLength(1);
      expect(remaining[0].timestamp).toBe('20241023120000');
    });
  });

  describe('Database Statistics', () => {
    test('should return database statistics', () => {
      const testData: StatusData = {
        worker_status: 'Working',
        space_status: 'Screw_tightening',
        robot_status: { state: 'moving', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 1,
        status: 'Working'
      };

      const testCommand: CommandData = {
        command: 'tool_handover',
        timestamp: '20241023120000'
      };

      dbService.saveStatusData(testData);
      dbService.saveCommandHistory(testCommand, true);

      const stats = dbService.getDatabaseStats();
      expect(stats.statusRecords).toBe(1);
      expect(stats.commandRecords).toBe(1);
      expect(stats.statisticsRecords).toBe(0);
    });
  });

  describe('Error Handling', () => {
    test('should handle database errors gracefully', () => {
      // Close the database to simulate error
      dbService.close();

      const testData: StatusData = {
        worker_status: 'Working',
        space_status: 'Screw_tightening',
        robot_status: { state: 'moving', grip: 'open' },
        timestamp: '20241023120000',
        tool_delivery: 1,
        status: 'Working'
      };

      const result = dbService.saveStatusData(testData);
      expect(result).toBe(false);
    });
  });
});