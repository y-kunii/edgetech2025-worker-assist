/**
 * MessageRouter
 * メッセージのルーティングを担当
 */

import { Socket } from 'socket.io';
import { SensorData, RobotCommand, RobotResponse, ErrorInfo } from '../types';
import { ConnectionManager } from './ConnectionManager';
import { Logger } from '../utils/Logger';

export class MessageRouter {
  private connectionManager: ConnectionManager;
  private logger: Logger;

  constructor(connectionManager: ConnectionManager, logger: Logger) {
    this.connectionManager = connectionManager;
    this.logger = logger;
  }

  /**
   * センサーデータをElectronクライアントにルーティング
   * 最適化: データのコピーを避け、直接参照を渡す
   */
  routeSensorData(data: SensorData, fromSocket: Socket): void {
    this.logger.debug('Routing sensor data to Electron clients', {
      fromSocketId: fromSocket.id,
      dataKeys: Object.keys(data),
    });

    // センサープログラムが接続されているか確認
    if (!this.connectionManager.isClientConnected('sensor')) {
      this.logger.warn('Sensor program not connected when routing sensor data');
      const error: ErrorInfo = {
        code: 'SENSOR_NOT_CONNECTED',
        message: 'Sensor program is not connected',
        timestamp: new Date().toISOString(),
      };
      this.sendErrorResponse(fromSocket, error);
      return;
    }

    // Electronクライアントを取得
    const electronClients = this.connectionManager.getElectronClients();

    if (electronClients.length === 0) {
      this.logger.warn('No Electron clients connected to receive sensor data');
      return;
    }

    // 全Electronクライアントにデータを転送（データのコピーを避ける）
    let successCount = 0;
    let errorCount = 0;

    for (const client of electronClients) {
      try {
        // データを直接渡す（コピーしない）
        client.emit('sensor_data', data);
        successCount++;
        this.logger.debug('Sensor data sent to Electron client', {
          clientId: client.id,
        });
      } catch (error) {
        errorCount++;
        this.logger.error('Failed to send sensor data to Electron client', error as Error, {
          clientId: client.id,
        });
        
        // データ転送失敗時のエラー通知
        const errorInfo: ErrorInfo = {
          code: 'ROUTING_ERROR',
          message: 'Failed to route sensor data',
          timestamp: new Date().toISOString(),
        };
        this.sendErrorResponse(client, errorInfo);
      }
    }

    this.logger.info('Sensor data routed to Electron clients', {
      successCount,
      errorCount,
      totalClients: electronClients.length,
    });
  }

  /**
   * ロボット指示をロボット制御プログラムにルーティング
   * 最適化: データのコピーを避け、直接参照を渡す
   */
  routeRobotCommand(command: RobotCommand, fromSocket: Socket): void {
    this.logger.info('Routing robot command', {
      fromSocketId: fromSocket.id,
      command: command.command,
      timestamp: command.timestamp,
    });

    // ロボット制御プログラムが接続されているか確認
    const robotClient = this.connectionManager.getRobotClient();

    if (!robotClient) {
      this.logger.warn('Robot control program not connected when routing command', {
        command: command.command,
      });
      
      const error: ErrorInfo = {
        code: 'ROBOT_NOT_CONNECTED',
        message: 'Robot control program is not connected',
        timestamp: new Date().toISOString(),
      };
      this.sendErrorResponse(fromSocket, error);
      return;
    }

    // ロボット制御プログラムに指示を転送（データのコピーを避ける）
    try {
      robotClient.emit('robot_command', command);
      this.logger.info('Robot command sent to robot control program', {
        robotClientId: robotClient.id,
        command: command.command,
      });
    } catch (error) {
      this.logger.error('Failed to send robot command', error as Error, {
        robotClientId: robotClient.id,
        command: command.command,
      });
      
      const errorInfo: ErrorInfo = {
        code: 'ROUTING_ERROR',
        message: 'Failed to route robot command',
        timestamp: new Date().toISOString(),
      };
      this.sendErrorResponse(fromSocket, errorInfo);
    }
  }

  /**
   * ロボット応答をElectronクライアントにルーティング
   */
  routeRobotResponse(response: RobotResponse, fromSocket: Socket): void {
    this.logger.info('Routing robot response to Electron clients', {
      fromSocketId: fromSocket.id,
      command: response.command,
      status: response.status,
      timestamp: response.timestamp,
    });

    // Electronクライアントを取得
    const electronClients = this.connectionManager.getElectronClients();

    if (electronClients.length === 0) {
      this.logger.warn('No Electron clients connected to receive robot response');
      return;
    }

    // 全Electronクライアントに応答を転送
    let successCount = 0;
    let errorCount = 0;

    for (const client of electronClients) {
      try {
        client.emit('robot_response', response);
        successCount++;
        this.logger.debug('Robot response sent to Electron client', {
          clientId: client.id,
        });
      } catch (error) {
        errorCount++;
        this.logger.error('Failed to send robot response to Electron client', error as Error, {
          clientId: client.id,
        });
      }
    }

    this.logger.info('Robot response routed to Electron clients', {
      successCount,
      errorCount,
      totalClients: electronClients.length,
    });
  }

  /**
   * エラー応答を送信
   */
  sendErrorResponse(socket: Socket, error: ErrorInfo): void {
    try {
      socket.emit('error', error);
      this.logger.debug('Error response sent to client', {
        socketId: socket.id,
        errorCode: error.code,
        errorMessage: error.message,
      });
    } catch (err) {
      this.logger.error('Failed to send error response', err as Error, {
        socketId: socket.id,
        errorCode: error.code,
      });
    }
  }
}

/**
 * エラーコード定義
 */
export const ErrorCodes = {
  SENSOR_NOT_CONNECTED: 'SENSOR_NOT_CONNECTED',
  ROBOT_NOT_CONNECTED: 'ROBOT_NOT_CONNECTED',
  INVALID_DATA_FORMAT: 'INVALID_DATA_FORMAT',
  ROUTING_ERROR: 'ROUTING_ERROR',
  CONNECTION_TIMEOUT: 'CONNECTION_TIMEOUT',
  DUPLICATE_CLIENT_TYPE: 'DUPLICATE_CLIENT_TYPE',
} as const;

