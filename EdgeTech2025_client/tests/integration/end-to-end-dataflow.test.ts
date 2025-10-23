import { WebSocketClient, WebSocketClientConfig } from '../../src/main/websocket-client';
import { DataProcessor } from '../../src/main/data-processor';
import { DatabaseService } from '../../src/main/database-service';
import { FileWatcher, FileWatcherConfig } from '../../src/main/file-watcher';
import { StatusData, CommandData } from '../../src/shared/types';
import WebSocket from 'ws';
import fs from 'fs';
import path from 'path';

// Mock WebSocket Server for E2E testing
class E2EWebSocketServer {
  private server: WebSocket.Server;
  private clients: Set<WebSocket> = new Set();

  constructor(port: number) {
    this.server = new WebSocket.Server({ port });
    this.setupServerEvents();
  }

  private setupServerEvents() {
    this.server.on('connection', (ws: WebSocket) => {
      this.clients.add(ws);

      ws.on('message', (data: WebSocket.Data) => {
        console.log('E2E Server received command:', data.toString());
      });

      ws.on('close', () => {
        this.clients.delete(ws);
      });
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

describe('End-to-End Data Flow Tests', () => {
  let webSocketServer: E2EWebSocketServer;
  let webSocketClient: WebSocketClient;
  let fileWatcher: FileWatcher;
  let databaseService: DatabaseService;
  let testImageDir: string;
  let testDbPath: string;

  const serverPort = 9092;

  beforeAll(() => {
    // Initialize DataProcessor
    DataProcessor.initialize();
  });

  beforeEach(async () => {
    // Setup test directories
    testImageDir = path.join('/tmp', `e2e-images-${Date.now()}`);
    fs.mkdirSync(testImageDir, { recursive: true });

    testDbPath = path.join('/tmp', `e2e-test-${Date.now()}.sqlite`);

    // Mock electron app for database
    jest.doMock('electron', () => ({
      app: {
        getPath: jest.fn(() => path.dirname(testDbPath))
      }
    }));

    // Mock constants for database
    jest.doMock('../../src/shared/constants', () => ({
      APP_CONFIG: {
        DB_FILE_NAME: path.basename(testDbPath),
        DATA_RETENTION_DAYS: 30
      }
    }));

    // Start WebSocket server
    webSocketServer = new E2EWebSocketServer(serverPort);
    await new Promise(resolve => setTimeout(resolve, 100));

    // Initialize services
    databaseService = new DatabaseService();

    const wsConfig: WebSocketClientConfig = {
      url: `ws://localhost:${serverPort}`,
      reconnectInterval: 500,
      maxReconnectAttempts: 3,
      heartbeatInterval: 1000,
    };
    webSocketClient = new WebSocketClient(wsConfig);

    const fileConfig: FileWatcherConfig = {
      watchPath: testImageDir,
      supportedExtensions: ['.jpg', '.jpeg', '.png'],
      debounceDelay: 100,
      maxFileSize: 10 * 1024 * 1024,
    };
    fileWatcher = new FileWatcher(fileConfig);
  });

  afterEach(async () => {
    // Cleanup services
    if (webSocketClient) {
      webSocketClient.disconnect();
    }
    if (fileWatcher) {
      await fileWatcher.stop();
    }
    if (databaseService) {
      databaseService.close();
    }
    if (webSocketServer) {
      webSocketServer.close();
    }

    // Cleanup test files
    if (fs.existsSync(testImageDir)) {
      const files = fs.readdirSync(testImageDir);
      files.forEach(file => {
        fs.unlinkSync(path.join(testImageDir, file));
      });
      fs.rmdirSync(testImageDir);
    }

    if (fs.existsSync(testDbPath)) {
      fs.unlinkSync(testDbPath);
    }
  });

  describe('Complete Data Pipeline', () => {
    test('should handle complete workflow: WebSocket → Database → Statistics', (done) => {
      let statusDataReceived = false;
      let dataSavedToDb = false;

      // Monitor WebSocket data reception
      webSocketClient.on('statusData', (data: StatusData) => {
        statusDataReceived = true;
        expect(data.worker_status).toBe('Working');
        expect(data.space_status).toBe('Screw_tightening');

        // Check if data was saved to database
        setTimeout(() => {
          const latestStatus = databaseService.getLatestStatus();
          if (latestStatus) {
            dataSavedToDb = true;
            expect(latestStatus.worker_status).toBe('Working');
            expect(latestStatus.space_status).toBe('Screw_tightening');

            if (statusDataReceived && dataSavedToDb) {
              done();
            }
          }
        }, 200);
      });

      // Connect and send test data
      webSocketClient.on('connected', () => {
        setTimeout(() => {
          const testData: StatusData = {
            worker_status: 'Working',
            space_status: 'Screw_tightening',
            robot_status: { state: 'moving', grip: 'closed' },
            timestamp: '20241023120000',
            tool_delivery: 1,
            status: 'Working'
          };
          webSocketServer.sendStatusData(testData);
        }, 100);
      });

      webSocketClient.connect();
    }, 10000);

    test('should handle work task transitions and statistics updates', (done) => {
      let transitionCount = 0;
      const expectedTransitions = [
        { from: undefined, to: 'Waiting' },
        { from: 'Waiting', to: 'Screw_tightening' },
        { from: 'Screw_tightening', to: 'Building_blocks' }
      ];

      webSocketClient.on('workTaskChanged', (event) => {
        const expected = expectedTransitions[transitionCount];
        expect(event.previousTask).toBe(expected.from);
        expect(event.currentTask).toBe(expected.to);
        
        transitionCount++;
        
        if (transitionCount === expectedTransitions.length) {
          // Verify database contains all transitions
          setTimeout(() => {
            const history = databaseService.getStatusHistory();
            expect(history.length).toBeGreaterThanOrEqual(3);
            done();
          }, 200);
        }
      });

      webSocketClient.on('connected', () => {
        const statusSequence = [
          {
            worker_status: 'Waiting' as const,
            space_status: 'Nothing' as const,
            timestamp: '20241023120000'
          },
          {
            worker_status: 'Working' as const,
            space_status: 'Screw_tightening' as const,
            timestamp: '20241023120100'
          },
          {
            worker_status: 'Working' as const,
            space_status: 'Building_blocks' as const,
            timestamp: '20241023120200'
          }
        ];

        statusSequence.forEach((status, index) => {
          setTimeout(() => {
            const fullStatus: StatusData = {
              ...status,
              robot_status: { state: 'moving', grip: 'open' },
              tool_delivery: index,
              status: status.worker_status
            };
            webSocketServer.sendStatusData(fullStatus);
          }, index * 300);
        });
      });

      webSocketClient.connect();
    }, 15000);

    test('should handle command sending and history tracking', (done) => {
      let commandsSent = 0;
      const testCommands = ['tool_handover', 'tool_collection', 'wait'] as const;

      webSocketClient.on('commandSent', (result) => {
        commandsSent++;
        expect(result.success).toBe(true);
        
        if (commandsSent === testCommands.length) {
          // Verify command history in database
          setTimeout(() => {
            const commandHistory = databaseService.getCommandHistory();
            expect(commandHistory.length).toBe(testCommands.length);
            
            // Verify all commands are recorded
            const recordedCommands = commandHistory.map(h => h.command);
            testCommands.forEach(cmd => {
              expect(recordedCommands).toContain(cmd);
            });
            
            done();
          }, 200);
        }
      });

      webSocketClient.on('connected', async () => {
        // Send commands sequentially
        for (const command of testCommands) {
          try {
            await webSocketClient.createAndSendCommand(command);
            await new Promise(resolve => setTimeout(resolve, 100));
          } catch (error) {
            done(error);
          }
        }
      });

      webSocketClient.connect();
    }, 10000);
  });

  describe('File Watching Integration', () => {
    test('should detect image updates while processing WebSocket data', (done) => {
      let imageDetected = false;
      let statusReceived = false;

      fileWatcher.on('imageUpdated', (event) => {
        imageDetected = true;
        expect(event.fileName).toBe('live-feed.jpg');
        
        if (imageDetected && statusReceived) {
          done();
        }
      });

      webSocketClient.on('statusData', () => {
        statusReceived = true;
        
        if (imageDetected && statusReceived) {
          done();
        }
      });

      Promise.all([
        fileWatcher.start(),
        new Promise<void>(resolve => {
          webSocketClient.on('connected', resolve);
          webSocketClient.connect();
        })
      ]).then(() => {
        // Simulate concurrent image and status updates
        setTimeout(() => {
          // Create image file
          const imagePath = path.join(testImageDir, 'live-feed.jpg');
          fs.writeFileSync(imagePath, 'live image data');
          
          // Send status data
          const statusData: StatusData = {
            worker_status: 'Working',
            space_status: 'Screw_tightening',
            robot_status: { state: 'moving', grip: 'open' },
            timestamp: '20241023120000',
            tool_delivery: 1,
            status: 'Working'
          };
          webSocketServer.sendStatusData(statusData);
        }, 200);
      });
    }, 10000);

    test('should maintain image state during WebSocket reconnections', (done) => {
      let reconnectionCount = 0;
      let imagePathBeforeDisconnect: string | null = null;

      fileWatcher.on('imageUpdated', (event) => {
        if (reconnectionCount === 0) {
          // First image detected, trigger disconnection
          setTimeout(() => {
            imagePathBeforeDisconnect = fileWatcher.getLatestImagePath();
            webSocketServer.close();
          }, 100);
        }
      });

      webSocketClient.on('disconnected', () => {
        // Restart server after disconnection
        setTimeout(() => {
          webSocketServer = new E2EWebSocketServer(serverPort);
          
          // Check that image path is still available
          const imagePathAfterDisconnect = fileWatcher.getLatestImagePath();
          expect(imagePathAfterDisconnect).toBe(imagePathBeforeDisconnect);
          done();
        }, 300);
      });

      Promise.all([
        fileWatcher.start(),
        new Promise<void>(resolve => {
          webSocketClient.on('connected', resolve);
          webSocketClient.connect();
        })
      ]).then(() => {
        // Create image to trigger the test sequence
        setTimeout(() => {
          const imagePath = path.join(testImageDir, 'persistent-image.jpg');
          fs.writeFileSync(imagePath, 'persistent image data');
        }, 200);
      });
    }, 15000);
  });

  describe('Error Recovery and Data Integrity', () => {
    test('should maintain data integrity during service failures', (done) => {
      let statusDataCount = 0;
      const totalStatusUpdates = 5;

      webSocketClient.on('statusData', () => {
        statusDataCount++;
        
        // Simulate database error by closing it temporarily
        if (statusDataCount === 3) {
          databaseService.close();
          
          // Reinitialize database after a delay
          setTimeout(() => {
            databaseService = new DatabaseService();
          }, 200);
        }
        
        if (statusDataCount === totalStatusUpdates) {
          // Check data integrity after recovery
          setTimeout(() => {
            const history = databaseService.getStatusHistory();
            expect(history.length).toBeGreaterThan(0);
            
            // Verify data integrity
            const integrity = databaseService.validateDataIntegrity();
            expect(integrity.isValid).toBe(true);
            done();
          }, 300);
        }
      });

      webSocketClient.on('connected', () => {
        // Send multiple status updates
        for (let i = 0; i < totalStatusUpdates; i++) {
          setTimeout(() => {
            const statusData: StatusData = {
              worker_status: i % 2 === 0 ? 'Working' : 'Waiting',
              space_status: 'Screw_tightening',
              robot_status: { state: 'moving', grip: 'open' },
              timestamp: `2024102312${i.toString().padStart(2, '0')}00`,
              tool_delivery: i,
              status: i % 2 === 0 ? 'Working' : 'Waiting'
            };
            webSocketServer.sendStatusData(statusData);
          }, i * 200);
        }
      });

      webSocketClient.connect();
    }, 15000);

    test('should handle invalid data gracefully without affecting other operations', (done) => {
      let validDataCount = 0;
      let errorCount = 0;

      webSocketClient.on('statusData', () => {
        validDataCount++;
      });

      webSocketClient.on('dataValidationError', () => {
        errorCount++;
      });

      webSocketClient.on('connected', () => {
        const dataSequence = [
          // Valid data
          {
            worker_status: 'Working',
            space_status: 'Screw_tightening',
            robot_status: { state: 'moving', grip: 'open' },
            timestamp: '20241023120000',
            tool_delivery: 1,
            status: 'Working'
          },
          // Invalid data (will be sent as raw string)
          'invalid json data',
          // Another valid data
          {
            worker_status: 'Waiting',
            space_status: 'Nothing',
            robot_status: { state: 'idle', grip: 'open' },
            timestamp: '20241023120100',
            tool_delivery: 0,
            status: 'Waiting'
          }
        ];

        dataSequence.forEach((data, index) => {
          setTimeout(() => {
            if (typeof data === 'string') {
              // Send invalid data directly
              webSocketServer.clients.forEach(client => {
                if (client.readyState === WebSocket.OPEN) {
                  client.send(data);
                }
              });
            } else {
              // Send valid data
              webSocketServer.sendStatusData(data as StatusData);
            }
          }, index * 200);
        });

        // Check results after all data is sent
        setTimeout(() => {
          expect(validDataCount).toBe(2); // Two valid data items
          expect(errorCount).toBe(1); // One invalid data item
          
          // Verify database contains only valid data
          const history = databaseService.getStatusHistory();
          expect(history.length).toBe(2);
          done();
        }, 1000);
      });

      webSocketClient.connect();
    }, 10000);
  });

  describe('Performance Under Load', () => {
    test('should handle high-frequency data updates', (done) => {
      let receivedCount = 0;
      const totalUpdates = 50;
      const startTime = Date.now();

      webSocketClient.on('statusData', () => {
        receivedCount++;
        
        if (receivedCount === totalUpdates) {
          const endTime = Date.now();
          const duration = endTime - startTime;
          
          console.log(`Processed ${totalUpdates} updates in ${duration}ms`);
          
          // Verify all data was saved to database
          setTimeout(() => {
            const history = databaseService.getStatusHistory(undefined, undefined, totalUpdates + 10);
            expect(history.length).toBe(totalUpdates);
            done();
          }, 500);
        }
      });

      webSocketClient.on('connected', () => {
        // Send rapid updates
        for (let i = 0; i < totalUpdates; i++) {
          setTimeout(() => {
            const statusData: StatusData = {
              worker_status: i % 3 === 0 ? 'Working' : i % 3 === 1 ? 'Waiting' : 'Work Completed',
              space_status: i % 2 === 0 ? 'Screw_tightening' : 'Building_blocks',
              robot_status: { state: 'moving', grip: i % 2 === 0 ? 'open' : 'closed' },
              timestamp: `20241023${(120000 + i).toString()}`,
              tool_delivery: i % 5,
              status: i % 3 === 0 ? 'Working' : i % 3 === 1 ? 'Waiting' : 'Work Completed'
            };
            webSocketServer.sendStatusData(statusData);
          }, i * 10); // 10ms intervals
        }
      });

      webSocketClient.connect();
    }, 20000);
  });
});