import { EventEmitter } from 'events';

// Mock WebSocket class
class MockWebSocket extends EventEmitter {
  public readyState: number = 1; // OPEN
  public static OPEN = 1;
  public static CLOSED = 3;

  constructor(public url: string) {
    super();
    // Simulate connection after a short delay
    setTimeout(() => {
      this.emit('open');
    }, 10);
  }

  send(data: string, callback?: (error?: Error) => void) {
    // Simulate successful send
    setTimeout(() => {
      if (callback) callback();
    }, 5);
  }

  close(code?: number, reason?: string) {
    this.readyState = MockWebSocket.CLOSED;
    this.emit('close', code || 1000, Buffer.from(reason || ''));
  }
}

// Mock the ws module
jest.mock('ws', () => MockWebSocket);

import { WebSocketClient, WebSocketClientConfig } from '../../src/main/websocket-client';
import { StatusData, CommandData } from '../../src/shared/types';

describe('WebSocketClient', () => {
  let client: WebSocketClient;
  let config: WebSocketClientConfig;

  beforeEach(() => {
    config = {
      url: 'ws://localhost:9090',
      reconnectInterval: 1000,
      maxReconnectAttempts: 3,
      heartbeatInterval: 5000,
    };
    client = new WebSocketClient(config);
  });

  afterEach(() => {
    client.disconnect();
  });

  describe('Connection Management', () => {
    test('should initialize with disconnected state', () => {
      const state = client.getConnectionState();
      expect(state.isConnected).toBe(false);
      expect(state.reconnectAttempts).toBe(0);
      expect(state.lastHeartbeat).toBeNull();
    });

    test('should connect successfully', (done) => {
      client.on('connected', () => {
        const state = client.getConnectionState();
        expect(state.isConnected).toBe(true);
        expect(state.reconnectAttempts).toBe(0);
        done();
      });

      client.connect();
    });

    test('should emit connectionStateChanged event', (done) => {
      client.on('connectionStateChanged', (state) => {
        if (state.isConnected) {
          expect(state.isConnected).toBe(true);
          done();
        }
      });

      client.connect();
    });

    test('should disconnect properly', (done) => {
      client.on('connected', () => {
        client.disconnect();
      });

      client.on('disconnected', () => {
        const state = client.getConnectionState();
        expect(state.isConnected).toBe(false);
        done();
      });

      client.connect();
    });
  });

  describe('Data Processing', () => {
    const validStatusData: StatusData = {
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

    test('should process valid status data', (done) => {
      client.on('statusData', (data: StatusData) => {
        expect(data.worker_status).toBe('Working');
        expect(data.space_status).toBe('Screw_tightening');
        expect(data.robot_status.state).toBe('moving');
        expect(data.robot_status.grip).toBe('open');
        done();
      });

      client.on('connected', () => {
        // Simulate receiving data
        const ws = (client as any).ws;
        ws.emit('message', JSON.stringify(validStatusData));
      });

      client.connect();
    });

    test('should detect status changes', (done) => {
      let changeCount = 0;

      client.on('statusDataChanged', (event) => {
        changeCount++;
        expect(event.statusData).toBeDefined();
        expect(event.changeInfo.hasChanged).toBe(true);
        
        if (changeCount === 2) {
          expect(event.changeInfo.changedFields).toContain('worker_status');
          done();
        }
      });

      client.on('connected', () => {
        const ws = (client as any).ws;
        
        // Send first status
        ws.emit('message', JSON.stringify(validStatusData));
        
        // Send changed status after a delay
        setTimeout(() => {
          const changedData = { ...validStatusData, worker_status: 'Waiting' };
          ws.emit('message', JSON.stringify(changedData));
        }, 50);
      });

      client.connect();
    });

    test('should handle invalid JSON data', (done) => {
      client.on('dataValidationError', (error) => {
        expect(error.error).toContain('Invalid JSON format');
        done();
      });

      client.on('connected', () => {
        const ws = (client as any).ws;
        ws.emit('message', 'invalid json data');
      });

      client.connect();
    });
  });

  describe('Command Sending', () => {
    const validCommand: CommandData = {
      command: 'tool_handover',
      timestamp: '20241023120000'
    };

    test('should send command successfully', async () => {
      await new Promise<void>((resolve) => {
        client.on('connected', resolve);
        client.connect();
      });

      const result = await client.sendCommand(validCommand);
      expect(result.success).toBe(true);
      expect(result.command).toEqual(validCommand);
    });

    test('should fail to send command when disconnected', async () => {
      try {
        await client.sendCommand(validCommand);
        fail('Should have thrown an error');
      } catch (error) {
        expect(error).toBeInstanceOf(Error);
        expect((error as Error).message).toContain('WebSocket is not connected');
      }
    });

    test('should create and send command', async () => {
      await new Promise<void>((resolve) => {
        client.on('connected', resolve);
        client.connect();
      });

      const result = await client.createAndSendCommand('tool_collection');
      expect(result.success).toBe(true);
      expect(result.command.command).toBe('tool_collection');
    });
  });

  describe('Configuration', () => {
    test('should update configuration', () => {
      const newConfig = { url: 'ws://localhost:8080' };
      client.updateConfig(newConfig);
      
      const updatedConfig = (client as any).config;
      expect(updatedConfig.url).toBe('ws://localhost:8080');
      expect(updatedConfig.reconnectInterval).toBe(config.reconnectInterval); // Should keep original values
    });
  });

  describe('Error Handling', () => {
    test('should handle WebSocket errors', (done) => {
      client.on('error', (error) => {
        expect(error).toBeInstanceOf(Error);
        done();
      });

      client.on('connected', () => {
        const ws = (client as any).ws;
        ws.emit('error', new Error('Test error'));
      });

      client.connect();
    });
  });
});