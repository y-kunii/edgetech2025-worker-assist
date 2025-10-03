import { WebSocketService, WebSocketConfig } from '../main/services/WebSocketService';
import { io, Socket } from 'socket.io-client';

// Mock socket.io-client
jest.mock('socket.io-client');
const mockIo = io as jest.MockedFunction<typeof io>;

describe('WebSocketService', () => {
  let webSocketService: WebSocketService;
  let mockSocket: jest.Mocked<Socket>;
  let config: WebSocketConfig;

  beforeEach(() => {
    mockSocket = {
      connect: jest.fn(),
      disconnect: jest.fn(),
      on: jest.fn(),
      off: jest.fn(),
      emit: jest.fn(),
      connected: false,
      id: 'test-socket-id'
    } as any;

    mockIo.mockReturnValue(mockSocket);

    config = {
      url: 'http://localhost:3001',
      reconnectionAttempts: 5,
      reconnectionDelay: 1000,
      timeout: 5000
    };

    webSocketService = new WebSocketService(config);
  });

  afterEach(() => {
    webSocketService.destroy();
    jest.clearAllMocks();
  });

  describe('initialization', () => {
    it('should create socket with correct configuration', () => {
      expect(mockIo).toHaveBeenCalledWith(config.url, {
        reconnectionAttempts: config.reconnectionAttempts,
        reconnectionDelay: config.reconnectionDelay,
        timeout: config.timeout,
        autoConnect: false
      });
    });

    it('should set up event listeners', () => {
      expect(mockSocket.on).toHaveBeenCalledWith('connect', expect.any(Function));
      expect(mockSocket.on).toHaveBeenCalledWith('disconnect', expect.any(Function));
      expect(mockSocket.on).toHaveBeenCalledWith('sensor_data', expect.any(Function));
      expect(mockSocket.on).toHaveBeenCalledWith('robot-response', expect.any(Function));
      expect(mockSocket.on).toHaveBeenCalledWith('connect_error', expect.any(Function));
    });
  });

  describe('connect', () => {
    it('should connect to WebSocket server', () => {
      webSocketService.connect();
      expect(mockSocket.connect).toHaveBeenCalled();
    });

    it('should emit connection_attempt event', () => {
      const connectionAttemptSpy = jest.fn();
      webSocketService.on('connection_attempt', connectionAttemptSpy);

      webSocketService.connect();

      expect(connectionAttemptSpy).toHaveBeenCalled();
    });
  });

  describe('disconnect', () => {
    it('should disconnect from WebSocket server', () => {
      webSocketService.disconnect();
      expect(mockSocket.disconnect).toHaveBeenCalled();
    });
  });

  describe('isConnected', () => {
    it('should return socket connection status', () => {
      mockSocket.connected = true;
      expect(webSocketService.isConnected()).toBe(true);

      mockSocket.connected = false;
      expect(webSocketService.isConnected()).toBe(false);
    });
  });

  describe('sendRobotCommand', () => {
    it('should send robot command when connected', () => {
      mockSocket.connected = true;
      const commandData = { type: 'tool_handover', parameters: {} };

      const result = webSocketService.sendRobotCommand('robot_command', commandData);

      expect(result).toBe(true);
      expect(mockSocket.emit).toHaveBeenCalledWith('robot_command', commandData);
    });

    it('should return false when not connected', () => {
      mockSocket.connected = false;
      const commandData = { type: 'tool_handover', parameters: {} };

      const result = webSocketService.sendRobotCommand('robot_command', commandData);

      expect(result).toBe(false);
      expect(mockSocket.emit).not.toHaveBeenCalled();
    });
  });

  describe('event handling', () => {
    it('should handle connect event', () => {
      const connectSpy = jest.fn();
      webSocketService.on('connected', connectSpy);

      // Simulate connect event
      const connectHandler = mockSocket.on.mock.calls.find(call => call[0] === 'connect')?.[1];
      connectHandler?.();

      expect(connectSpy).toHaveBeenCalled();
    });

    it('should handle disconnect event', () => {
      const disconnectSpy = jest.fn();
      webSocketService.on('disconnected', disconnectSpy);

      // Simulate disconnect event
      const disconnectHandler = mockSocket.on.mock.calls.find(call => call[0] === 'disconnect')?.[1];
      disconnectHandler?.('transport close');

      expect(disconnectSpy).toHaveBeenCalledWith('transport close');
    });

    it('should handle sensor_data event', () => {
      const sensorDataSpy = jest.fn();
      webSocketService.on('sensor_data', sensorDataSpy);

      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      // Simulate sensor_data event
      const sensorDataHandler = mockSocket.on.mock.calls.find(call => call[0] === 'sensor_data')?.[1];
      sensorDataHandler?.(sensorData);

      expect(sensorDataSpy).toHaveBeenCalledWith(sensorData);
    });

    it('should handle robot-response event', () => {
      const robotResponseSpy = jest.fn();
      webSocketService.on('robot-response', robotResponseSpy);

      const robotResponse = {
        commandId: 'test-command-id',
        status: 'success',
        message: 'Command executed successfully',
        timestamp: '2024-01-01T12:00:00Z'
      };

      // Simulate robot-response event
      const robotResponseHandler = mockSocket.on.mock.calls.find(call => call[0] === 'robot-response')?.[1];
      robotResponseHandler?.(robotResponse);

      expect(robotResponseSpy).toHaveBeenCalledWith(robotResponse);
    });

    it('should handle connect_error event', () => {
      const errorSpy = jest.fn();
      webSocketService.on('connection_error', errorSpy);

      const error = new Error('Connection failed');

      // Simulate connect_error event
      const errorHandler = mockSocket.on.mock.calls.find(call => call[0] === 'connect_error')?.[1];
      errorHandler?.(error);

      expect(errorSpy).toHaveBeenCalledWith(error);
    });
  });

  describe('updateConfig', () => {
    it('should update configuration and reconnect', () => {
      const newConfig = {
        url: 'http://localhost:8080',
        reconnectionAttempts: 10
      };

      webSocketService.updateConfig(newConfig);

      expect(mockSocket.disconnect).toHaveBeenCalled();
      expect(mockIo).toHaveBeenCalledWith(newConfig.url, expect.objectContaining({
        reconnectionAttempts: newConfig.reconnectionAttempts
      }));
    });
  });

  describe('getConnectionInfo', () => {
    it('should return connection information', () => {
      mockSocket.connected = true;
      mockSocket.id = 'test-socket-id';

      const info = webSocketService.getConnectionInfo();

      expect(info.connected).toBe(true);
      expect(info.socketId).toBe('test-socket-id');
      expect(info.url).toBe(config.url);
    });
  });

  describe('destroy', () => {
    it('should clean up resources', () => {
      webSocketService.destroy();

      expect(mockSocket.off).toHaveBeenCalledWith('connect');
      expect(mockSocket.off).toHaveBeenCalledWith('disconnect');
      expect(mockSocket.off).toHaveBeenCalledWith('sensor_data');
      expect(mockSocket.off).toHaveBeenCalledWith('robot-response');
      expect(mockSocket.off).toHaveBeenCalledWith('connect_error');
      expect(mockSocket.disconnect).toHaveBeenCalled();
    });
  });
});