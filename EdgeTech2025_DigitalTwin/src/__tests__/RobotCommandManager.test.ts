import { RobotCommandManager, RobotCommandManagerConfig } from '../main/services/RobotCommandManager';
import { WebSocketService } from '../main/services/WebSocketService';
import { RobotCommandResponse } from '../types';

// WebSocketServiceのモック
const mockWebSocketService = {
  isConnected: jest.fn(),
  sendRobotCommand: jest.fn(),
  on: jest.fn(),
  off: jest.fn()
} as unknown as WebSocketService;

describe('RobotCommandManager', () => {
  let robotCommandManager: RobotCommandManager;
  let config: RobotCommandManagerConfig;

  beforeEach(() => {
    config = {
      maxRetries: 3,
      retryDelay: 1000,
      commandTimeout: 5000,
      enableAutoRetry: true
    };

    robotCommandManager = new RobotCommandManager(mockWebSocketService, config);
    
    // モックをリセット
    jest.clearAllMocks();
  });

  afterEach(() => {
    robotCommandManager.destroy();
  });

  describe('sendCommand', () => {
    it('should send tool_handover command successfully', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(true);

      // レスポンスをシミュレート
      const mockResponse: RobotCommandResponse = {
        commandId: 'test-command-id',
        status: 'success',
        message: 'Command executed successfully',
        timestamp: new Date().toISOString()
      };

      // sendCommandを呼び出し、レスポンスハンドラーをトリガー
      const commandPromise = robotCommandManager.sendCommand('tool_handover', {
        reason: 'screw_work_completed'
      });

      // レスポンスハンドラーを手動でトリガー
      setTimeout(() => {
        const onHandler = (mockWebSocketService.on as jest.Mock).mock.calls
          .find(call => call[0] === 'robot-response')?.[1];
        if (onHandler) {
          // コマンドIDを実際のものに合わせる
          const logs = robotCommandManager.getCommandLogs();
          if (logs.length > 0) {
            mockResponse.commandId = logs[0].id;
            onHandler(mockResponse);
          }
        }
      }, 100);

      const response = await commandPromise;

      expect(response).toEqual(mockResponse);
      expect(mockWebSocketService.sendRobotCommand).toHaveBeenCalledWith(
        'robot_command',
        expect.objectContaining({
          type: 'tool_handover',
          parameters: { reason: 'screw_work_completed' }
        })
      );
    });

    it('should send next_task command successfully', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(true);

      // レスポンスをシミュレート
      const mockResponse: RobotCommandResponse = {
        commandId: 'test-command-id',
        status: 'success',
        message: 'Command executed successfully',
        timestamp: new Date().toISOString()
      };

      // sendCommandを呼び出し、レスポンスハンドラーをトリガー
      const commandPromise = robotCommandManager.sendCommand('next_task', {
        reason: 'bolt_work_completed'
      });

      // レスポンスハンドラーを手動でトリガー
      setTimeout(() => {
        const onHandler = (mockWebSocketService.on as jest.Mock).mock.calls
          .find(call => call[0] === 'robot-response')?.[1];
        if (onHandler) {
          // コマンドIDを実際のものに合わせる
          const logs = robotCommandManager.getCommandLogs();
          if (logs.length > 0) {
            mockResponse.commandId = logs[0].id;
            onHandler(mockResponse);
          }
        }
      }, 100);

      const response = await commandPromise;

      expect(response).toEqual(mockResponse);
      expect(mockWebSocketService.sendRobotCommand).toHaveBeenCalledWith(
        'robot_command',
        expect.objectContaining({
          type: 'next_task',
          parameters: { reason: 'bolt_work_completed' }
        })
      );
    });

    it('should throw error when WebSocket is not connected', async () => {
      // WebSocket未接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(false);

      await expect(
        robotCommandManager.sendCommand('tool_handover')
      ).rejects.toThrow('WebSocket not connected');
    });

    it('should throw error when command sending fails', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(false);

      await expect(
        robotCommandManager.sendCommand('tool_handover')
      ).rejects.toThrow('Failed to send command via WebSocket');
    });

    it('should timeout when no response is received', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(true);

      // タイムアウトを短く設定
      const shortTimeoutManager = new RobotCommandManager(mockWebSocketService, {
        ...config,
        commandTimeout: 100
      });

      await expect(
        shortTimeoutManager.sendCommand('tool_handover')
      ).rejects.toThrow(/Command timeout/);

      shortTimeoutManager.destroy();
    });
  });

  describe('sendThresholdCommand', () => {
    it('should send tool_handover command for screw threshold', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(true);

      const testData = {
        screwCount: 5,
        timestamp: new Date().toISOString()
      };

      // レスポンスをシミュレート
      const mockResponse: RobotCommandResponse = {
        commandId: 'test-command-id',
        status: 'success',
        message: 'Command executed successfully',
        timestamp: new Date().toISOString()
      };

      // イベントリスナーを設定
      const successHandler = jest.fn();
      robotCommandManager.on('threshold_command_success', successHandler);

      // sendThresholdCommandを呼び出し
      const commandPromise = robotCommandManager.sendThresholdCommand(
        'screw_threshold_reached',
        testData
      );

      // レスポンスハンドラーを手動でトリガー
      setTimeout(() => {
        const onHandler = (mockWebSocketService.on as jest.Mock).mock.calls
          .find(call => call[0] === 'robot-response')?.[1];
        if (onHandler) {
          const logs = robotCommandManager.getCommandLogs();
          if (logs.length > 0) {
            mockResponse.commandId = logs[0].id;
            onHandler(mockResponse);
          }
        }
      }, 100);

      await commandPromise;

      expect(mockWebSocketService.sendRobotCommand).toHaveBeenCalledWith(
        'robot_command',
        expect.objectContaining({
          type: 'tool_handover',
          parameters: {
            reason: 'screw_work_completed',
            screwCount: 5,
            timestamp: testData.timestamp
          }
        })
      );

      // 成功イベントが発火されることを確認
      expect(successHandler).toHaveBeenCalledWith(
        expect.objectContaining({
          reason: 'screw_threshold_reached',
          command: 'tool_handover'
        })
      );
    });

    it('should send next_task command for bolt threshold', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(true);

      const testData = {
        boltCount: 3,
        timestamp: new Date().toISOString()
      };

      // レスポンスをシミュレート
      const mockResponse: RobotCommandResponse = {
        commandId: 'test-command-id',
        status: 'success',
        message: 'Command executed successfully',
        timestamp: new Date().toISOString()
      };

      // イベントリスナーを設定
      const successHandler = jest.fn();
      robotCommandManager.on('threshold_command_success', successHandler);

      // sendThresholdCommandを呼び出し
      const commandPromise = robotCommandManager.sendThresholdCommand(
        'bolt_threshold_reached',
        testData
      );

      // レスポンスハンドラーを手動でトリガー
      setTimeout(() => {
        const onHandler = (mockWebSocketService.on as jest.Mock).mock.calls
          .find(call => call[0] === 'robot-response')?.[1];
        if (onHandler) {
          const logs = robotCommandManager.getCommandLogs();
          if (logs.length > 0) {
            mockResponse.commandId = logs[0].id;
            onHandler(mockResponse);
          }
        }
      }, 100);

      await commandPromise;

      expect(mockWebSocketService.sendRobotCommand).toHaveBeenCalledWith(
        'robot_command',
        expect.objectContaining({
          type: 'next_task',
          parameters: {
            reason: 'bolt_work_completed',
            boltCount: 3,
            timestamp: testData.timestamp
          }
        })
      );

      // 成功イベントが発火されることを確認
      expect(successHandler).toHaveBeenCalledWith(
        expect.objectContaining({
          reason: 'bolt_threshold_reached',
          command: 'next_task'
        })
      );
    });

    it('should emit error event when threshold command fails', async () => {
      // WebSocket未接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(false);

      const testData = {
        screwCount: 5,
        timestamp: new Date().toISOString()
      };

      // イベントリスナーを設定
      const errorHandler = jest.fn();
      robotCommandManager.on('threshold_command_error', errorHandler);

      await robotCommandManager.sendThresholdCommand('screw_threshold_reached', testData);

      // エラーイベントが発火されることを確認
      expect(errorHandler).toHaveBeenCalledWith(
        expect.objectContaining({
          reason: 'screw_threshold_reached',
          error: expect.any(Error)
        })
      );
    });
  });

  describe('retryCommand', () => {
    it('should retry a failed command', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(true);

      // 最初のコマンドを失敗させる
      const commandPromise = robotCommandManager.sendCommand('tool_handover');

      // エラーレスポンスをシミュレート
      const onHandler = (mockWebSocketService.on as jest.Mock).mock.calls
        .find(call => call[0] === 'robot-response')?.[1];
      
      const logs = robotCommandManager.getCommandLogs();
      const errorResponse: RobotCommandResponse = {
        commandId: logs[0].id,
        status: 'error',
        message: 'Command failed',
        timestamp: new Date().toISOString()
      };
      
      // 即座にエラーレスポンスを送信
      setTimeout(() => onHandler(errorResponse), 10);

      try {
        await commandPromise;
      } catch (error) {
        // エラーは期待される
      }

      expect(logs).toHaveLength(1);
      expect(logs[0].status).toBe('error');

      // リトライを実行
      const retryPromise = robotCommandManager.retryCommand(logs[0].id);

      // 成功レスポンスをシミュレート
      const successResponse: RobotCommandResponse = {
        commandId: logs[0].id,
        status: 'success',
        message: 'Command executed successfully',
        timestamp: new Date().toISOString()
      };
      
      // 即座に成功レスポンスを送信
      setTimeout(() => onHandler(successResponse), 10);

      const response = await retryPromise;

      expect(response.status).toBe('success');
      expect(logs[0].retryCount).toBe(1);
    });

    it('should throw error when trying to retry non-existent command', async () => {
      await expect(
        robotCommandManager.retryCommand('non-existent-id')
      ).rejects.toThrow('Command not found');
    });
  });

  describe('cancelCommand', () => {
    it('should cancel a pending command', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(true);

      // コマンドを送信（レスポンスは送信しない）
      robotCommandManager.sendCommand('tool_handover');

      const logs = robotCommandManager.getCommandLogs();
      expect(logs).toHaveLength(1);
      expect(logs[0].status).toBe('pending');

      // コマンドをキャンセル
      const cancelled = robotCommandManager.cancelCommand(logs[0].id);

      expect(cancelled).toBe(true);
      expect(logs[0].status).toBe('error');
      expect(logs[0].error).toBe('Command cancelled by user');
    });

    it('should return false when trying to cancel non-existent command', () => {
      const cancelled = robotCommandManager.cancelCommand('non-existent-id');
      expect(cancelled).toBe(false);
    });
  });

  describe('getStatistics', () => {
    it('should return correct statistics', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(true);

      // 複数のコマンドを送信
      const command1Promise = robotCommandManager.sendCommand('tool_handover');
      const command2Promise = robotCommandManager.sendCommand('next_task');

      const onHandler = (mockWebSocketService.on as jest.Mock).mock.calls
        .find(call => call[0] === 'robot-response')?.[1];
      
      const logs = robotCommandManager.getCommandLogs();
      
      // 1つ目を成功、2つ目を失敗
      setTimeout(() => {
        onHandler({
          commandId: logs[0].id,
          status: 'success',
          message: 'Success',
          timestamp: new Date().toISOString()
        });
        
        onHandler({
          commandId: logs[1].id,
          status: 'error',
          message: 'Failed',
          timestamp: new Date().toISOString()
        });
      }, 10);

      await Promise.allSettled([command1Promise, command2Promise]);

      const statistics = robotCommandManager.getStatistics();

      expect(statistics.total).toBe(2);
      expect(statistics.successful).toBe(1);
      expect(statistics.failed).toBe(1);
      expect(statistics.pending).toBe(0);
      expect(statistics.successRate).toBe(50);
    });
  });

  describe('updateConfig', () => {
    it('should update configuration', () => {
      const newConfig = {
        maxRetries: 5,
        retryDelay: 2000
      };

      const configHandler = jest.fn();
      robotCommandManager.on('config_updated', configHandler);

      robotCommandManager.updateConfig(newConfig);

      expect(configHandler).toHaveBeenCalledWith(
        expect.objectContaining(newConfig)
      );
    });
  });

  describe('clearLogs', () => {
    it('should clear all command logs', async () => {
      // WebSocket接続をモック
      (mockWebSocketService.isConnected as jest.Mock).mockReturnValue(true);
      (mockWebSocketService.sendRobotCommand as jest.Mock).mockReturnValue(true);

      // コマンドを送信
      robotCommandManager.sendCommand('tool_handover');

      expect(robotCommandManager.getCommandLogs()).toHaveLength(1);

      const clearHandler = jest.fn();
      robotCommandManager.on('logs_cleared', clearHandler);

      robotCommandManager.clearLogs();

      expect(robotCommandManager.getCommandLogs()).toHaveLength(0);
      expect(clearHandler).toHaveBeenCalled();
    });
  });
});