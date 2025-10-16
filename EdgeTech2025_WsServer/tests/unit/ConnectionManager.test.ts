/**
 * ConnectionManager 単体テスト
 */

import { ConnectionManager } from '../../src/server/ConnectionManager';
import { Logger } from '../../src/utils/Logger';

// Socket.ioのSocketをモック
class MockSocket {
  id: string;
  disconnected: boolean = false;

  constructor(id: string) {
    this.id = id;
  }

  disconnect(_close?: boolean): void {
    this.disconnected = true;
  }
}

describe('ConnectionManager', () => {
  let connectionManager: ConnectionManager;
  let logger: Logger;

  beforeEach(() => {
    // Loggerのモック（実際のファイル出力を避けるため）
    logger = {
      debug: jest.fn(),
      info: jest.fn(),
      warn: jest.fn(),
      error: jest.fn(),
    } as any;

    connectionManager = new ConnectionManager(logger);
  });

  describe('registerClient', () => {
    test('should register client with correct type', () => {
      const socket = new MockSocket('socket-1') as any;
      connectionManager.registerClient('socket-1', 'electron', socket);

      expect(connectionManager.getTotalClients()).toBe(1);
      expect(connectionManager.isClientConnected('electron')).toBe(true);
    });

    test('should register multiple electron clients', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;

      connectionManager.registerClient('socket-1', 'electron', socket1);
      connectionManager.registerClient('socket-2', 'electron', socket2);

      expect(connectionManager.getTotalClients()).toBe(2);
      expect(connectionManager.getElectronClients()).toHaveLength(2);
    });

    test('should disconnect existing sensor client when new sensor connects', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;

      connectionManager.registerClient('socket-1', 'sensor', socket1);
      expect(connectionManager.getTotalClients()).toBe(1);

      connectionManager.registerClient('socket-2', 'sensor', socket2);
      
      expect(socket1.disconnected).toBe(true);
      expect(connectionManager.getTotalClients()).toBe(1);
      expect(connectionManager.getSensorClient()?.id).toBe('socket-2');
    });

    test('should disconnect existing robot client when new robot connects', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;

      connectionManager.registerClient('socket-1', 'robot', socket1);
      expect(connectionManager.getTotalClients()).toBe(1);

      connectionManager.registerClient('socket-2', 'robot', socket2);
      
      expect(socket1.disconnected).toBe(true);
      expect(connectionManager.getTotalClients()).toBe(1);
      expect(connectionManager.getRobotClient()?.id).toBe('socket-2');
    });

    test('should log warning when duplicate sensor/robot client connects', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;

      connectionManager.registerClient('socket-1', 'sensor', socket1);
      connectionManager.registerClient('socket-2', 'sensor', socket2);

      expect(logger.warn).toHaveBeenCalledWith(
        expect.stringContaining('Duplicate sensor client detected'),
        expect.any(Object)
      );
    });
  });

  describe('unregisterClient', () => {
    test('should unregister client', () => {
      const socket = new MockSocket('socket-1') as any;
      connectionManager.registerClient('socket-1', 'electron', socket);
      
      expect(connectionManager.getTotalClients()).toBe(1);
      
      connectionManager.unregisterClient('socket-1');
      
      expect(connectionManager.getTotalClients()).toBe(0);
      expect(connectionManager.isClientConnected('electron')).toBe(false);
    });

    test('should handle unregistering non-existent client', () => {
      connectionManager.unregisterClient('non-existent');
      
      expect(connectionManager.getTotalClients()).toBe(0);
    });
  });

  describe('getClientsByType', () => {
    test('should return clients of specified type', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;
      const socket3 = new MockSocket('socket-3') as any;

      connectionManager.registerClient('socket-1', 'electron', socket1);
      connectionManager.registerClient('socket-2', 'electron', socket2);
      connectionManager.registerClient('socket-3', 'sensor', socket3);

      const electronClients = connectionManager.getClientsByType('electron');
      
      expect(electronClients).toHaveLength(2);
      expect(electronClients[0].id).toBe('socket-1');
      expect(electronClients[1].id).toBe('socket-2');
    });

    test('should return empty array when no clients of type exist', () => {
      const clients = connectionManager.getClientsByType('robot');
      
      expect(clients).toHaveLength(0);
    });
  });

  describe('getSensorClient', () => {
    test('should return sensor client when registered', () => {
      const socket = new MockSocket('socket-1') as any;
      connectionManager.registerClient('socket-1', 'sensor', socket);

      const sensorClient = connectionManager.getSensorClient();
      
      expect(sensorClient).not.toBeNull();
      expect(sensorClient?.id).toBe('socket-1');
    });

    test('should return null when no sensor client registered', () => {
      const sensorClient = connectionManager.getSensorClient();
      
      expect(sensorClient).toBeNull();
    });
  });

  describe('getRobotClient', () => {
    test('should return robot client when registered', () => {
      const socket = new MockSocket('socket-1') as any;
      connectionManager.registerClient('socket-1', 'robot', socket);

      const robotClient = connectionManager.getRobotClient();
      
      expect(robotClient).not.toBeNull();
      expect(robotClient?.id).toBe('socket-1');
    });

    test('should return null when no robot client registered', () => {
      const robotClient = connectionManager.getRobotClient();
      
      expect(robotClient).toBeNull();
    });
  });

  describe('getElectronClients', () => {
    test('should return all electron clients', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;

      connectionManager.registerClient('socket-1', 'electron', socket1);
      connectionManager.registerClient('socket-2', 'electron', socket2);

      const electronClients = connectionManager.getElectronClients();
      
      expect(electronClients).toHaveLength(2);
    });

    test('should return empty array when no electron clients', () => {
      const electronClients = connectionManager.getElectronClients();
      
      expect(electronClients).toHaveLength(0);
    });
  });

  describe('getConnectionStats', () => {
    test('should return correct connection statistics', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;
      const socket3 = new MockSocket('socket-3') as any;
      const socket4 = new MockSocket('socket-4') as any;

      connectionManager.registerClient('socket-1', 'electron', socket1);
      connectionManager.registerClient('socket-2', 'electron', socket2);
      connectionManager.registerClient('socket-3', 'sensor', socket3);
      connectionManager.registerClient('socket-4', 'robot', socket4);

      const stats = connectionManager.getConnectionStats();
      
      expect(stats.total).toBe(4);
      expect(stats.byType.electron).toBe(2);
      expect(stats.byType.sensor).toBe(1);
      expect(stats.byType.robot).toBe(1);
      expect(stats.byType.unknown).toBe(0);
    });

    test('should return zero stats when no clients connected', () => {
      const stats = connectionManager.getConnectionStats();
      
      expect(stats.total).toBe(0);
      expect(stats.byType.electron).toBe(0);
      expect(stats.byType.sensor).toBe(0);
      expect(stats.byType.robot).toBe(0);
      expect(stats.byType.unknown).toBe(0);
    });

    test('should count unknown type clients', () => {
      const socket = new MockSocket('socket-1') as any;
      connectionManager.registerClient('socket-1', 'unknown', socket);

      const stats = connectionManager.getConnectionStats();
      
      expect(stats.total).toBe(1);
      expect(stats.byType.unknown).toBe(1);
    });
  });

  describe('isClientConnected', () => {
    test('should return true when client type is connected', () => {
      const socket = new MockSocket('socket-1') as any;
      connectionManager.registerClient('socket-1', 'sensor', socket);

      expect(connectionManager.isClientConnected('sensor')).toBe(true);
    });

    test('should return false when client type is not connected', () => {
      expect(connectionManager.isClientConnected('robot')).toBe(false);
    });

    test('should return true for electron when multiple electron clients connected', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;

      connectionManager.registerClient('socket-1', 'electron', socket1);
      connectionManager.registerClient('socket-2', 'electron', socket2);

      expect(connectionManager.isClientConnected('electron')).toBe(true);
    });
  });

  describe('integration scenarios', () => {
    test('should handle complete client lifecycle', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;
      const socket3 = new MockSocket('socket-3') as any;

      // Register clients
      connectionManager.registerClient('socket-1', 'electron', socket1);
      connectionManager.registerClient('socket-2', 'sensor', socket2);
      connectionManager.registerClient('socket-3', 'robot', socket3);

      expect(connectionManager.getTotalClients()).toBe(3);

      // Check connections
      expect(connectionManager.isClientConnected('electron')).toBe(true);
      expect(connectionManager.isClientConnected('sensor')).toBe(true);
      expect(connectionManager.isClientConnected('robot')).toBe(true);

      // Unregister sensor
      connectionManager.unregisterClient('socket-2');
      expect(connectionManager.getTotalClients()).toBe(2);
      expect(connectionManager.isClientConnected('sensor')).toBe(false);

      // Stats should reflect changes
      const stats = connectionManager.getConnectionStats();
      expect(stats.total).toBe(2);
      expect(stats.byType.electron).toBe(1);
      expect(stats.byType.sensor).toBe(0);
      expect(stats.byType.robot).toBe(1);
    });

    test('should handle sensor reconnection scenario', () => {
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;

      // First sensor connects
      connectionManager.registerClient('socket-1', 'sensor', socket1);
      expect(connectionManager.getSensorClient()?.id).toBe('socket-1');

      // Second sensor connects (should disconnect first)
      connectionManager.registerClient('socket-2', 'sensor', socket2);
      expect(socket1.disconnected).toBe(true);
      expect(connectionManager.getSensorClient()?.id).toBe('socket-2');
      expect(connectionManager.getTotalClients()).toBe(1);
    });
  });
});
