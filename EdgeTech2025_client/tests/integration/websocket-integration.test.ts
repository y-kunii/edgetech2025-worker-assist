import { WebSocketClient, WebSocketClientConfig } from '../../src/main/websocket-client';
import { DataProcessor } from '../../src/main/data-processor';
import { StatusData, CommandData } from '../../src/shared/types';
import WebSocket from 'ws';
import { EventEmitter } from 'events';

// Mock WebSocket Server for integration testing
class MockWebSocketServer extends EventEmitter {
  private server: WebSocket.Server;
  private clients: Set<WebSocket> = new Set();

  constructor(port: number) {
    super();
    this.server = new WebSocket.Server({ port });
    this.setupServerEvents();
  }

  private setupServerEvents() {
    this.server.on('connection', (ws: WebSocket) => {
      this.clients.add(ws);
      console.log('Client connected to mock server');

      ws.on('message', (data: WebSocket.Data) => {
        const message = data.toString();
        console.log('Server received:', message);
        
        try {
          const command = JSON.parse(message) as CommandData;
          this.emit('commandReceived', command);
        } catch (error) {
          console.error('Invalid command received:', error);
        }
      });

      ws.on('close', () => {
        this.clients.delete(ws);
        console.log('Client disconnected from mock server');
      });

      // Send initial status data after connection
      setTimeout(() => {
        this.sendStatusData({
          worker_status: 'Waiting',
          space_status: 'Nothing',
          robot_status: { state: 'idle', grip: 'open' },
          timestamp: '20241023120000',
          tool_delivery: 0,
          status: 'Waiting'
        });
      }, 100);
    });
  }

  sendStatusData(statusData: StatusData) {
    const message = JSON.stringify(statusData);
    this.clients.forEach(client => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  }

  close() {
    this.clients.forEach(client => client.close());
    this.server.close();
  }
}

describe('WebSocket Integration Tests', () => {
  let mockServer: MockWebSocketServer;
  let client: WebSocketClient;
  const serverPort = 9091;

  beforeAll(() => {
    // Initialize DataProcessor for integration tests
    DataProcessor.initialize();
  });

  beforeEach(async () => {
    // Start mock server
    mockServer = new MockWebSocketServer(serverPort);
    
    // Wait for server to be ready
    await new Promise(resolve => setTimeout(resolve, 100));

    // Create client
    const config: WebSocketClientConfig = {
      url: `ws://localhost:${serverPort}`,
      reconnectInterval: 500,
      maxReconnectAttempts: 3,
      heartbeatInterval: 1000,
    };
    client = new WebSocketClient(config);
  });

  afterEach(() => {
    if (client) {
      client.disconnect();
    }
    if (mockServer) {
      mockServer.close();
    }
  });

  describe('Connection and Data Flow', () => {
    test('should establish connection and receive initial data', (done) => {
      let connected = false;
      let dataReceived = false;

      client.on('connected', () => {
        connected = true;
        console.log('Client connected');
        if (connected && dataReceived) done();
      });

      client.on('statusData', (data: StatusData) => {
        dataReceived = true;
        console.log('Data received:', data);
        expect(data.worker_status).toBe('Waiting');
        expect(data.space_status).toBe('Nothing');
        if (connected && dataReceived) done();
      });

      client.connect();
    }, 10000);

    test('should handle real-time status updates', (done) => {
      let updateCount = 0;
      const expectedUpdates = [
        { worker_status: 'Waiting', space_status: 'Nothing' },
        { worker_status: 'Working', space_status: 'Screw_tightening' },
        { worker_status: 'Work Completed', space_status: 'Screw_tightening' }
      ];

      client.on('statusData', (data: StatusData) => {
        const expected = expectedUpdates[updateCount];
        if (expected) {
          expect(data.worker_status).toBe(expected.worker_status);
          expect(data.space_status).toBe(expected.space_status);
          updateCount++;

          if (updateCount === expectedUpdates.length) {
            done();
          }
        }
      });

      client.on('connected', () => {
        // Send additional status updates
        setTimeout(() => {
          mockServer.sendStatusData({
            worker_status: 'Working',
            space_status: 'Screw_tightening',
            robot_status: { state: 'moving', grip: 'closed' },
            timestamp: '20241023120100',
            tool_delivery: 1,
            status: 'Working'
          });
        }, 200);

        setTimeout(() => {
          mockServer.sendStatusData({
            worker_status: 'Work Completed',
            space_status: 'Screw_tightening',
            robot_status: { state: 'idle', grip: 'open' },
            timestamp: '20241023120200',
            tool_delivery: 1,
            status: 'Work Completed'
          });
        }, 400);
      });

      client.connect();
    }, 10000);

    test('should detect and emit status changes', (done) => {
      let changeCount = 0;

      client.on('statusDataChanged', (event) => {
        changeCount++;
        expect(event.changeInfo.hasChanged).toBe(true);
        
        if (changeCount === 1) {
          // First change should be initial data
          expect(event.changeInfo.changedFields).toContain('initial');
        } else if (changeCount === 2) {
          // Second change should be work task change
          expect(event.changeInfo.workTaskChanged).toBe(true);
          expect(event.changeInfo.currentTask).toBe('Screw_tightening');
          done();
        }
      });

      client.on('connected', () => {
        setTimeout(() => {
          mockServer.sendStatusData({
            worker_status: 'Working',
            space_status: 'Screw_tightening',
            robot_status: { state: 'moving', grip: 'closed' },
            timestamp: '20241023120100',
            tool_delivery: 1,
            status: 'Working'
          });
        }, 200);
      });

      client.connect();
    }, 10000);
  });

  describe('Command Sending Integration', () => {
    test('should send commands to server successfully', (done) => {
      const testCommand: CommandData = {
        command: 'tool_handover',
        timestamp: '20241023120000'
      };

      mockServer.on('commandReceived', (command: CommandData) => {
        expect(command.command).toBe('tool_handover');
        expect(command.timestamp).toBe('20241023120000');
        done();
      });

      client.on('connected', async () => {
        try {
          const result = await client.sendCommand(testCommand);
          expect(result.success).toBe(true);
        } catch (error) {
          done(error);
        }
      });

      client.connect();
    }, 10000);

    test('should handle multiple command types', (done) => {
      const commands: CommandData[] = [
        { command: 'tool_handover', timestamp: '20241023120000' },
        { command: 'tool_collection', timestamp: '20241023120100' },
        { command: 'wait', timestamp: '20241023120200' }
      ];

      let receivedCommands: CommandData[] = [];

      mockServer.on('commandReceived', (command: CommandData) => {
        receivedCommands.push(command);
        
        if (receivedCommands.length === commands.length) {
          expect(receivedCommands).toHaveLength(3);
          expect(receivedCommands[0].command).toBe('tool_handover');
          expect(receivedCommands[1].command).toBe('tool_collection');
          expect(receivedCommands[2].command).toBe('wait');
          done();
        }
      });

      client.on('connected', async () => {
        try {
          for (const command of commands) {
            await client.sendCommand(command);
            await new Promise(resolve => setTimeout(resolve, 50)); // Small delay between commands
          }
        } catch (error) {
          done(error);
        }
      });

      client.connect();
    }, 10000);
  });

  describe('Error Handling and Recovery', () => {
    test('should handle server disconnection and reconnect', (done) => {
      let disconnected = false;
      let reconnected = false;

      client.on('connected', () => {
        if (!disconnected) {
          console.log('Initial connection established');
          // Close server to simulate disconnection
          setTimeout(() => {
            mockServer.close();
            disconnected = true;
          }, 200);
        } else {
          console.log('Reconnected successfully');
          reconnected = true;
          done();
        }
      });

      client.on('disconnected', () => {
        if (disconnected && !reconnected) {
          console.log('Disconnected, restarting server...');
          // Restart server after disconnection
          setTimeout(() => {
            mockServer = new MockWebSocketServer(serverPort);
          }, 300);
        }
      });

      client.connect();
    }, 15000);

    test('should handle invalid data gracefully', (done) => {
      client.on('dataValidationError', (error) => {
        expect(error.error).toContain('Invalid JSON format');
        done();
      });

      client.on('connected', () => {
        // Send invalid data through the mock server
        mockServer.clients.forEach(client => {
          if (client.readyState === WebSocket.OPEN) {
            client.send('invalid json data');
          }
        });
      });

      client.connect();
    }, 10000);

    test('should validate received status data', (done) => {
      client.on('dataValidationError', (error) => {
        expect(error.error).toContain('Invalid worker_status');
        expect(error.field).toBe('worker_status');
        done();
      });

      client.on('connected', () => {
        // Send invalid status data
        const invalidData = {
          worker_status: 'InvalidStatus',
          space_status: 'Nothing',
          robot_status: { state: 'idle', grip: 'open' },
          timestamp: '20241023120000',
          tool_delivery: 0,
          status: 'Waiting'
        };
        
        mockServer.clients.forEach(client => {
          if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify(invalidData));
          }
        });
      });

      client.connect();
    }, 10000);
  });

  describe('Performance and Reliability', () => {
    test('should handle rapid status updates', (done) => {
      let receivedCount = 0;
      const totalUpdates = 10;

      client.on('statusData', () => {
        receivedCount++;
        if (receivedCount === totalUpdates + 1) { // +1 for initial data
          done();
        }
      });

      client.on('connected', () => {
        // Send rapid updates
        for (let i = 0; i < totalUpdates; i++) {
          setTimeout(() => {
            const timestamp = `2024102312${i.toString().padStart(2, '0')}00`;
            mockServer.sendStatusData({
              worker_status: i % 2 === 0 ? 'Working' : 'Waiting',
              space_status: 'Screw_tightening',
              robot_status: { state: 'moving', grip: 'open' },
              timestamp,
              tool_delivery: i,
              status: i % 2 === 0 ? 'Working' : 'Waiting'
            });
          }, i * 10); // 10ms intervals
        }
      });

      client.connect();
    }, 10000);

    test('should maintain connection state accuracy', (done) => {
      let stateChanges = 0;

      client.on('connectionStateChanged', (state) => {
        stateChanges++;
        
        if (stateChanges === 1) {
          // First change should be connected
          expect(state.isConnected).toBe(true);
          expect(state.reconnectAttempts).toBe(0);
          
          // Disconnect to test state change
          setTimeout(() => {
            mockServer.close();
          }, 100);
        } else if (stateChanges === 2) {
          // Second change should be disconnected
          expect(state.isConnected).toBe(false);
          done();
        }
      });

      client.connect();
    }, 10000);
  });
});