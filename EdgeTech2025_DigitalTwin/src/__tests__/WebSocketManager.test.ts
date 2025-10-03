import { WebSocketManager, ConnectionState, ConnectionConfig } from '../main/websocket/WebSocketManager';
import { io } from 'socket.io-client';

// Mock socket.io-client
jest.mock('socket.io-client');
const mockIo = io as jest.MockedFunction<typeof io>;

describe('WebSocketManager', () => {
  let webSocketManager: WebSocketManager;
  let mockSocket: any;
  let config: ConnectionConfig;

  beforeEach(() => {
    mockSocket = {
      connect: jest.fn(),
      disconnect: jest.fn(),
      on: jest.fn(),
      emit: jest.fn(),
      connected: false,
    };

    mockIo.mockReturnValue(mockSocket);

    config = {
      url: 'http://localhost:3001',
      reconnectionAttempts: 5,
      reconnectionDelay: 1000,
      timeout: 5000
    };

    webSocketManager = new WebSocketManager(config);
  });

  afterEach(() => {
    webSocketManager.disconnect();
    jest.clearAllMocks();
  });

  describe('initialization', () => {
    it('should initialize with disconnected state', () => {
      const status = webSocketManager.getConnectionStatus();
      expect(status.state).toBe(ConnectionState.DISCONNECTED);
      expect(status.reconnectAttempts).toBe(0);
    });
  });

  describe('connect', () => {
    it('should create socket and attempt connection', () => {
      webSocketManager.connect();
      
      expect(mockIo).toHaveBeenCalledWith(config.url, {
        timeout: config.timeout,
        autoConnect: false,
        transports: ['websocket']
      });
      expect(mockSocket.connect).toHaveBeenCalled();
    });

    it('should update connection state to connecting', () => {
      webSocketManager.connect();
      
      const status = webSocketManager.getConnectionStatus();
      expect(status.state).toBe(ConnectionState.CONNECTING);
    });

    it('should handle successful connection', () => {
      webSocketManager.connect();
      
      // Simulate connect event
      const connectHandler = mockSocket.on.mock.calls.find(call => call[0] === 'connect')?.[1];
      connectHandler?.();

      const status = webSocketManager.getConnectionStatus();
      expect(status.state).toBe(ConnectionState.CONNECTED);
      expect(status.lastConnected).toBeDefined();
      expect(status.reconnectAttempts).toBe(0);
    });
  });

  describe('disconnect', () => {
    it('should disconnect socket', () => {
      webSocketManager.connect();
      webSocketManager.disconnect();
      
      expect(mockSocket.disconnect).toHaveBeenCalled();
      
      const status = webSocketManager.getConnectionStatus();
      expect(status.state).toBe(ConnectionState.DISCONNECTED);
    });
  });

  describe('sendMessage', () => {
    it('should send message when connected', () => {
      mockSocket.connected = true;
      webSocketManager.connect();
      
      const result = webSocketManager.sendMessage('test_event', { data: 'test' });
      
      expect(result).toBe(true);
      expect(mockSocket.emit).toHaveBeenCalledWith('test_event', { data: 'test' });
    });

    it('should return false when not connected', () => {
      mockSocket.connected = false;
      webSocketManager.connect();
      
      const result = webSocketManager.sendMessage('test_event', { data: 'test' });
      
      expect(result).toBe(false);
    });
  });

  describe('isConnected', () => {
    it('should return socket connection status', () => {
      webSocketManager.connect();
      
      mockSocket.connected = true;
      expect(webSocketManager.isConnected()).toBe(true);

      mockSocket.connected = false;
      expect(webSocketManager.isConnected()).toBe(false);
    });
  });

  describe('event handling', () => {
    it('should emit sensor_data events', () => {
      const sensorDataSpy = jest.fn();
      webSocketManager.on('sensor_data', sensorDataSpy);
      webSocketManager.connect();

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

    it('should emit robot_response events', () => {
      const robotResponseSpy = jest.fn();
      webSocketManager.on('robot_response', robotResponseSpy);
      webSocketManager.connect();

      const robotResponse = {
        commandId: 'test-command-id',
        status: 'success',
        message: 'Command executed successfully',
        timestamp: '2024-01-01T12:00:00Z'
      };

      // Simulate robot_response event
      const robotResponseHandler = mockSocket.on.mock.calls.find(call => call[0] === 'robot_response')?.[1];
      robotResponseHandler?.(robotResponse);

      expect(robotResponseSpy).toHaveBeenCalledWith(robotResponse);
    });

    it('should handle disconnect events', () => {
      const disconnectedSpy = jest.fn();
      webSocketManager.on('disconnected', disconnectedSpy);
      webSocketManager.connect();

      // Simulate disconnect event
      const disconnectHandler = mockSocket.on.mock.calls.find(call => call[0] === 'disconnect')?.[1];
      disconnectHandler?.('transport close');

      expect(disconnectedSpy).toHaveBeenCalledWith('transport close');
      
      const status = webSocketManager.getConnectionStatus();
      expect(status.state).toBe(ConnectionState.DISCONNECTED);
    });

    it('should handle connection errors', () => {
      const errorSpy = jest.fn();
      webSocketManager.on('error', errorSpy);
      webSocketManager.connect();

      const error = new Error('Connection failed');

      // Simulate connect_error event
      const errorHandler = mockSocket.on.mock.calls.find(call => call[0] === 'connect_error')?.[1];
      errorHandler?.(error);

      expect(errorSpy).toHaveBeenCalledWith(error);
      
      const status = webSocketManager.getConnectionStatus();
      expect(status.state).toBe(ConnectionState.ERROR);
    });
  });

  describe('reconnection handling', () => {
    it('should attempt reconnection on connection error', () => {
      jest.useFakeTimers();
      
      webSocketManager.connect();
      
      // Simulate connection error
      const errorHandler = mockSocket.on.mock.calls.find(call => call[0] === 'connect_error')?.[1];
      errorHandler?.(new Error('Connection failed'));

      let status = webSocketManager.getConnectionStatus();
      expect(status.state).toBe(ConnectionState.RECONNECTING);
      expect(status.reconnectAttempts).toBe(0);

      // Advance timer to trigger reconnection
      jest.advanceTimersByTime(config.reconnectionDelay!);
      
      // Should attempt to connect again
      expect(mockSocket.connect).toHaveBeenCalledTimes(2); // Initial + 1 retry

      jest.useRealTimers();
    });

    it('should stop reconnecting after max attempts', () => {
      jest.useFakeTimers();
      
      const maxReconnectSpy = jest.fn();
      webSocketManager.on('max_reconnect_attempts_reached', maxReconnectSpy);
      
      webSocketManager.connect();
      
      const errorHandler = mockSocket.on.mock.calls.find(call => call[0] === 'connect_error')?.[1];
      const error = new Error('Connection failed');

      // Simulate multiple connection errors
      for (let i = 0; i < config.reconnectionAttempts! + 1; i++) {
        errorHandler?.(error);
        jest.advanceTimersByTime(config.reconnectionDelay!);
      }

      expect(maxReconnectSpy).toHaveBeenCalled();

      jest.useRealTimers();
    });
  });

  describe('latency monitoring', () => {
    it('should handle pong events for latency calculation', () => {
      const latencyUpdateSpy = jest.fn();
      webSocketManager.on('latency_update', latencyUpdateSpy);
      webSocketManager.connect();

      const startTime = Date.now() - 50; // 50ms ago

      // Simulate pong event
      const pongHandler = mockSocket.on.mock.calls.find(call => call[0] === 'pong')?.[1];
      pongHandler?.(startTime);

      expect(latencyUpdateSpy).toHaveBeenCalledWith(expect.any(Number));
      
      const status = webSocketManager.getConnectionStatus();
      expect(status.latency).toBeGreaterThan(0);
    });
  });
});