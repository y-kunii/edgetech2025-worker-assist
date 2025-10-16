/**
 * MessageRouter 単体テスト
 */

import { MessageRouter, ErrorCodes } from '../../src/server/MessageRouter';
import { ConnectionManager } from '../../src/server/ConnectionManager';
import { Logger } from '../../src/utils/Logger';
import { SensorData, RobotCommand, RobotResponse, ErrorInfo } from '../../src/types';
import { Socket } from 'socket.io';

// モックSocket作成ヘルパー
const createMockSocket = (id: string = 'test-socket-id'): Socket => {
  const emittedEvents: Array<{ event: string; data: any }> = [];
  
  return {
    id,
    emit: jest.fn((event: string, data: any) => {
      emittedEvents.push({ event, data });
    }),
    disconnect: jest.fn(),
    _emittedEvents: emittedEvents,
  } as any;
};

describe('MessageRouter', () => {
  let messageRouter: MessageRouter;
  let connectionManager: ConnectionManager;
  let logger: Logger;

  beforeEach(() => {
    logger = new Logger('error', 'logs/test.log');
    connectionManager = new ConnectionManager(logger);
    messageRouter = new MessageRouter(connectionManager, logger);
  });

  describe('routeSensorData', () => {
    const sensorData: SensorData = {
      worker_status: 'waiting',
      robot_status: { state: 'waiting', grip: 'open' },
      screw_count: 5,
      bolt_count: 3,
      work_step: 'step1',
      timestamp: '2025-10-14T10:00:00Z',
    };

    test('センサーデータをElectronクライアントに転送する', () => {
      // センサークライアントを登録
      const sensorSocket = createMockSocket('sensor-socket');
      connectionManager.registerClient('sensor-socket', 'sensor', sensorSocket);

      // Electronクライアントを登録
      const electronSocket1 = createMockSocket('electron-1');
      const electronSocket2 = createMockSocket('electron-2');
      connectionManager.registerClient('electron-1', 'electron', electronSocket1);
      connectionManager.registerClient('electron-2', 'electron', electronSocket2);

      // センサーデータをルーティング
      messageRouter.routeSensorData(sensorData, sensorSocket);

      // 両方のElectronクライアントにデータが送信されたことを確認
      expect(electronSocket1.emit).toHaveBeenCalledWith('sensor_data', sensorData);
      expect(electronSocket2.emit).toHaveBeenCalledWith('sensor_data', sensorData);
    });

    test('センサープログラム未接続時にエラー応答を送信する', () => {
      // センサークライアントを登録しない
      const fromSocket = createMockSocket('from-socket');

      // センサーデータをルーティング
      messageRouter.routeSensorData(sensorData, fromSocket);

      // エラー応答が送信されたことを確認
      expect(fromSocket.emit).toHaveBeenCalledWith('error', expect.objectContaining({
        code: ErrorCodes.SENSOR_NOT_CONNECTED,
        message: 'Sensor program is not connected',
      }));
    });

    test('Electronクライアントが接続していない場合は警告ログのみ', () => {
      // センサークライアントのみ登録
      const sensorSocket = createMockSocket('sensor-socket');
      connectionManager.registerClient('sensor-socket', 'sensor', sensorSocket);

      // Electronクライアントは登録しない
      const loggerWarnSpy = jest.spyOn(logger, 'warn');

      // センサーデータをルーティング
      messageRouter.routeSensorData(sensorData, sensorSocket);

      // 警告ログが記録されたことを確認
      expect(loggerWarnSpy).toHaveBeenCalledWith(
        'No Electron clients connected to receive sensor data'
      );
    });

    test('データ転送失敗時にエラーログを記録する', () => {
      // センサークライアントを登録
      const sensorSocket = createMockSocket('sensor-socket');
      connectionManager.registerClient('sensor-socket', 'sensor', sensorSocket);

      // Electronクライアントを登録（emitがエラーをスローするようにモック）
      const electronSocket = createMockSocket('electron-1');
      (electronSocket.emit as jest.Mock).mockImplementation(() => {
        throw new Error('Emit failed');
      });
      connectionManager.registerClient('electron-1', 'electron', electronSocket);

      const loggerErrorSpy = jest.spyOn(logger, 'error');

      // センサーデータをルーティング
      messageRouter.routeSensorData(sensorData, sensorSocket);

      // エラーログが記録されたことを確認
      expect(loggerErrorSpy).toHaveBeenCalledWith(
        'Failed to send sensor data to Electron client',
        expect.any(Error),
        expect.objectContaining({ clientId: 'electron-1' })
      );
    });
  });

  describe('routeRobotCommand', () => {
    const robotCommand: RobotCommand = {
      command: 'tool_handover',
      timestamp: '2025-10-14T10:00:00Z',
    };

    test('ロボット指示をロボット制御プログラムに転送する', () => {
      // ロボットクライアントを登録
      const robotSocket = createMockSocket('robot-socket');
      connectionManager.registerClient('robot-socket', 'robot', robotSocket);

      // Electronクライアントから指示を送信
      const electronSocket = createMockSocket('electron-socket');

      // ロボット指示をルーティング
      messageRouter.routeRobotCommand(robotCommand, electronSocket);

      // ロボットクライアントに指示が送信されたことを確認
      expect(robotSocket.emit).toHaveBeenCalledWith('robot_command', robotCommand);
    });

    test('ロボット制御プログラム未接続時にエラー応答を送信する', () => {
      // ロボットクライアントを登録しない
      const electronSocket = createMockSocket('electron-socket');

      // ロボット指示をルーティング
      messageRouter.routeRobotCommand(robotCommand, electronSocket);

      // エラー応答が送信されたことを確認
      expect(electronSocket.emit).toHaveBeenCalledWith('error', expect.objectContaining({
        code: ErrorCodes.ROBOT_NOT_CONNECTED,
        message: 'Robot control program is not connected',
      }));
    });

    test('コマンド転送失敗時にエラー応答を送信する', () => {
      // ロボットクライアントを登録（emitがエラーをスローするようにモック）
      const robotSocket = createMockSocket('robot-socket');
      (robotSocket.emit as jest.Mock).mockImplementation(() => {
        throw new Error('Emit failed');
      });
      connectionManager.registerClient('robot-socket', 'robot', robotSocket);

      const electronSocket = createMockSocket('electron-socket');
      const loggerErrorSpy = jest.spyOn(logger, 'error');

      // ロボット指示をルーティング
      messageRouter.routeRobotCommand(robotCommand, electronSocket);

      // エラーログが記録されたことを確認
      expect(loggerErrorSpy).toHaveBeenCalledWith(
        'Failed to send robot command',
        expect.any(Error),
        expect.objectContaining({ robotClientId: 'robot-socket' })
      );

      // エラー応答が送信されたことを確認
      expect(electronSocket.emit).toHaveBeenCalledWith('error', expect.objectContaining({
        code: ErrorCodes.ROUTING_ERROR,
        message: 'Failed to route robot command',
      }));
    });
  });

  describe('routeRobotResponse', () => {
    const robotResponse: RobotResponse = {
      command: 'tool_handover',
      status: 'success',
      timestamp: '2025-10-14T10:00:00Z',
    };

    test('ロボット応答をElectronクライアントに転送する', () => {
      // Electronクライアントを登録
      const electronSocket1 = createMockSocket('electron-1');
      const electronSocket2 = createMockSocket('electron-2');
      connectionManager.registerClient('electron-1', 'electron', electronSocket1);
      connectionManager.registerClient('electron-2', 'electron', electronSocket2);

      // ロボットクライアントから応答を送信
      const robotSocket = createMockSocket('robot-socket');

      // ロボット応答をルーティング
      messageRouter.routeRobotResponse(robotResponse, robotSocket);

      // 両方のElectronクライアントに応答が送信されたことを確認
      expect(electronSocket1.emit).toHaveBeenCalledWith('robot_response', robotResponse);
      expect(electronSocket2.emit).toHaveBeenCalledWith('robot_response', robotResponse);
    });

    test('Electronクライアントが接続していない場合は警告ログのみ', () => {
      // Electronクライアントを登録しない
      const robotSocket = createMockSocket('robot-socket');
      const loggerWarnSpy = jest.spyOn(logger, 'warn');

      // ロボット応答をルーティング
      messageRouter.routeRobotResponse(robotResponse, robotSocket);

      // 警告ログが記録されたことを確認
      expect(loggerWarnSpy).toHaveBeenCalledWith(
        'No Electron clients connected to receive robot response'
      );
    });

    test('応答転送失敗時にエラーログを記録する', () => {
      // Electronクライアントを登録（emitがエラーをスローするようにモック）
      const electronSocket = createMockSocket('electron-1');
      (electronSocket.emit as jest.Mock).mockImplementation(() => {
        throw new Error('Emit failed');
      });
      connectionManager.registerClient('electron-1', 'electron', electronSocket);

      const robotSocket = createMockSocket('robot-socket');
      const loggerErrorSpy = jest.spyOn(logger, 'error');

      // ロボット応答をルーティング
      messageRouter.routeRobotResponse(robotResponse, robotSocket);

      // エラーログが記録されたことを確認
      expect(loggerErrorSpy).toHaveBeenCalledWith(
        'Failed to send robot response to Electron client',
        expect.any(Error),
        expect.objectContaining({ clientId: 'electron-1' })
      );
    });
  });

  describe('sendErrorResponse', () => {
    test('エラー応答をクライアントに送信する', () => {
      const error: ErrorInfo = {
        code: 'TEST_ERROR',
        message: 'Test error message',
        timestamp: '2025-10-14T10:00:00Z',
      };

      const socket = createMockSocket('test-socket');

      // エラー応答を送信
      messageRouter.sendErrorResponse(socket, error);

      // エラーイベントが送信されたことを確認
      expect(socket.emit).toHaveBeenCalledWith('error', error);
    });

    test('エラー応答送信失敗時にエラーログを記録する', () => {
      const error: ErrorInfo = {
        code: 'TEST_ERROR',
        message: 'Test error message',
        timestamp: '2025-10-14T10:00:00Z',
      };

      const socket = createMockSocket('test-socket');
      (socket.emit as jest.Mock).mockImplementation(() => {
        throw new Error('Emit failed');
      });

      const loggerErrorSpy = jest.spyOn(logger, 'error');

      // エラー応答を送信
      messageRouter.sendErrorResponse(socket, error);

      // エラーログが記録されたことを確認
      expect(loggerErrorSpy).toHaveBeenCalledWith(
        'Failed to send error response',
        expect.any(Error),
        expect.objectContaining({ socketId: 'test-socket', errorCode: 'TEST_ERROR' })
      );
    });
  });

  describe('ErrorCodes', () => {
    test('全てのエラーコードが定義されている', () => {
      expect(ErrorCodes.SENSOR_NOT_CONNECTED).toBe('SENSOR_NOT_CONNECTED');
      expect(ErrorCodes.ROBOT_NOT_CONNECTED).toBe('ROBOT_NOT_CONNECTED');
      expect(ErrorCodes.INVALID_DATA_FORMAT).toBe('INVALID_DATA_FORMAT');
      expect(ErrorCodes.ROUTING_ERROR).toBe('ROUTING_ERROR');
      expect(ErrorCodes.CONNECTION_TIMEOUT).toBe('CONNECTION_TIMEOUT');
      expect(ErrorCodes.DUPLICATE_CLIENT_TYPE).toBe('DUPLICATE_CLIENT_TYPE');
    });
  });
});
