/**
 * MainServer
 * WebSocketサーバーのメインクラス
 */

import express, { Express } from 'express';
import { createServer, Server as HttpServer } from 'http';
import { Server as SocketIOServer } from 'socket.io';
import cors from 'cors';
import { ServerConfig, HealthStatus, ClientType, SensorData, RobotCommand, RobotResponse } from '../types';
import { Logger } from '../utils/Logger';
import { MemoryMonitor } from '../utils/MemoryMonitor';
import { ConnectionManager } from './ConnectionManager';
import { MessageRouter } from './MessageRouter';
import { HeartbeatManager } from './HeartbeatManager';

export class MainServer {
  private config: ServerConfig;
  private logger: Logger;
  private memoryMonitor: MemoryMonitor;
  private app: Express;
  private httpServer: HttpServer | null;
  private io: SocketIOServer | null;
  private connectionManager: ConnectionManager;
  private messageRouter: MessageRouter;
  private heartbeatManager: HeartbeatManager;
  private isRunning: boolean;

  constructor(config: ServerConfig) {
    this.config = config;
    this.isRunning = false;
    this.httpServer = null;
    this.io = null;

    // Loggerの初期化
    this.logger = new Logger(config.log_level, config.log_file);
    this.logger.info('MainServer initializing', {
      port: config.port,
      cors_origin: config.cors_origin,
      heartbeat_interval: config.heartbeat_interval,
      connection_timeout: config.connection_timeout,
    });

    // メモリモニターの初期化
    this.memoryMonitor = new MemoryMonitor(this.logger);

    // Expressアプリの初期化
    this.app = express();
    this.app.use(cors({ origin: config.cors_origin }));
    this.app.use(express.json());

    // 各マネージャーのインスタンス作成
    this.connectionManager = new ConnectionManager(this.logger);
    this.messageRouter = new MessageRouter(this.connectionManager, this.logger);
    this.heartbeatManager = new HeartbeatManager(
      this.logger,
      config.heartbeat_interval,
      config.connection_timeout
    );

    this.logger.info('MainServer initialized successfully');
  }


  /**
   * WebSocket接続を処理
   */
  private handleConnection(socket: any): void {
    const socketId = socket.id;
    this.logger.info('New WebSocket connection', { socketId });

    // デフォルトでunknownタイプとして一時登録
    let clientType: ClientType = 'unknown';

    // register_clientイベントハンドラー
    socket.on('register_client', (data: { client_type?: string }) => {
      try {
        // クライアントタイプの識別
        const requestedType = data?.client_type;
        
        if (requestedType === 'electron' || requestedType === 'sensor' || requestedType === 'robot') {
          clientType = requestedType as ClientType;
        } else if (requestedType) {
          this.logger.warn('Unknown client type specified, using "unknown"', {
            socketId,
            requestedType,
          });
          clientType = 'unknown';
        } else {
          this.logger.warn('No client type specified, using "unknown"', { socketId });
          clientType = 'unknown';
        }

        // クライアントを登録
        this.connectionManager.registerClient(socketId, clientType, socket);

        // ハートビートを開始
        this.heartbeatManager.startHeartbeat(socket);

        // 登録確認メッセージを送信
        socket.emit('registered', {
          client_type: clientType,
          timestamp: new Date().toISOString(),
        });

        this.logger.info('Client registered successfully', {
          socketId,
          clientType,
        });
      } catch (error) {
        this.logger.error('Error during client registration', error as Error, {
          socketId,
        });
      }
    });

    // 接続直後にregister_clientを待つタイムアウトを設定（オプション）
    // クライアントが一定時間内に登録しない場合は警告
    setTimeout(() => {
      if (clientType === 'unknown') {
        this.logger.warn('Client has not registered within timeout period', {
          socketId,
        });
      }
    }, 5000);
  }


  /**
   * WebSocket切断を処理
   */
  private handleDisconnection(socket: any): void {
    const socketId = socket.id;
    
    // クライアント情報を取得（切断前に）
    const allClients = this.connectionManager.getAllClients();
    const clientInfo = allClients.get(socketId);
    const clientType = clientInfo?.clientType || 'unknown';

    this.logger.info('WebSocket disconnection', {
      socketId,
      clientType,
    });

    // ハートビートを停止
    this.heartbeatManager.stopHeartbeat(socketId);

    // クライアントの登録を解除
    this.connectionManager.unregisterClient(socketId);

    // センサーまたはロボットクライアントが切断した場合、Electronクライアントに通知
    if (clientType === 'sensor' || clientType === 'robot') {
      this.logger.warn(`External program disconnected: ${clientType}`, {
        socketId,
      });

      // 全Electronクライアントに切断通知を送信
      const electronClients = this.connectionManager.getElectronClients();
      for (const electronClient of electronClients) {
        try {
          electronClient.emit('external_disconnected', {
            client_type: clientType,
            timestamp: new Date().toISOString(),
          });
          this.logger.debug('Disconnection notification sent to Electron client', {
            electronClientId: electronClient.id,
            disconnectedType: clientType,
          });
        } catch (error) {
          this.logger.error('Failed to send disconnection notification', error as Error, {
            electronClientId: electronClient.id,
          });
        }
      }
    }

    this.logger.info('Client cleanup completed', {
      socketId,
      clientType,
      remainingClients: this.connectionManager.getTotalClients(),
    });
  }


  /**
   * イベントハンドラーをセットアップ
   */
  private setupEventHandlers(socket: any): void {
    const socketId = socket.id;

    // sensor_dataイベントハンドラー
    socket.on('sensor_data', (data: SensorData) => {
      try {
        this.logger.debug('Received sensor_data event', {
          socketId,
          dataKeys: Object.keys(data),
        });
        this.messageRouter.routeSensorData(data, socket);
      } catch (error) {
        this.logger.error('Error handling sensor_data event', error as Error, {
          socketId,
        });
      }
    });

    // robot_commandイベントハンドラー
    socket.on('robot_command', (command: RobotCommand) => {
      try {
        this.logger.debug('Received robot_command event', {
          socketId,
          command: command.command,
        });
        this.messageRouter.routeRobotCommand(command, socket);
      } catch (error) {
        this.logger.error('Error handling robot_command event', error as Error, {
          socketId,
        });
      }
    });

    // robot_responseイベントハンドラー
    socket.on('robot_response', (response: RobotResponse) => {
      try {
        this.logger.debug('Received robot_response event', {
          socketId,
          command: response.command,
          status: response.status,
        });
        this.messageRouter.routeRobotResponse(response, socket);
      } catch (error) {
        this.logger.error('Error handling robot_response event', error as Error, {
          socketId,
        });
      }
    });

    // pongイベントハンドラー
    socket.on('pong', (data: { timestamp?: number }) => {
      try {
        this.logger.debug('Received pong event', {
          socketId,
          timestamp: data?.timestamp,
        });
        this.heartbeatManager.handlePong(socketId);
      } catch (error) {
        this.logger.error('Error handling pong event', error as Error, {
          socketId,
        });
      }
    });

    // disconnectイベントハンドラー
    socket.on('disconnect', () => {
      this.handleDisconnection(socket);
    });
  }


  /**
   * HTTPヘルスチェックエンドポイントをセットアップ
   */
  private setupHealthCheckEndpoint(): void {
    this.app.get('/health', (_req, res) => {
      try {
        const healthStatus = this.getHealthStatus();
        const statusCode = healthStatus.status === 'ok' ? 200 : 503;
        
        res.status(statusCode).json(healthStatus);
        
        this.logger.debug('Health check requested', {
          status: healthStatus.status,
          connections: healthStatus.connections.total,
        });
      } catch (error) {
        this.logger.error('Error in health check endpoint', error as Error);
        res.status(503).json({
          status: 'error',
          timestamp: new Date().toISOString(),
          error: 'Failed to generate health status',
        });
      }
    });
  }

  /**
   * ヘルスステータスを取得
   */
  getHealthStatus(): HealthStatus {
    const stats = this.connectionManager.getConnectionStats();
    const sensorConnected = this.connectionManager.isClientConnected('sensor');
    const robotConnected = this.connectionManager.isClientConnected('robot');

    // サーバーが正常に動作しているかチェック
    const status: 'ok' | 'error' = this.isRunning ? 'ok' : 'error';

    return {
      status,
      timestamp: new Date().toISOString(),
      connections: {
        electron: stats.byType.electron,
        sensor: stats.byType.sensor,
        robot: stats.byType.robot,
        unknown: stats.byType.unknown,
        total: stats.total,
      },
      sensor_connected: sensorConnected,
      robot_connected: robotConnected,
    };
  }


  /**
   * サーバーを起動
   */
  async start(): Promise<void> {
    if (this.isRunning) {
      this.logger.warn('Server is already running');
      return;
    }

    try {
      this.logger.info('Starting server...');

      // HTTPサーバーを作成
      this.httpServer = createServer(this.app);

      // Socket.ioサーバーを初期化
      this.io = new SocketIOServer(this.httpServer, {
        cors: {
          origin: this.config.cors_origin,
          methods: ['GET', 'POST'],
        },
      });

      // ヘルスチェックエンドポイントをセットアップ
      this.setupHealthCheckEndpoint();

      // WebSocket接続イベントをセットアップ
      this.io.on('connection', (socket) => {
        this.handleConnection(socket);
        this.setupEventHandlers(socket);
      });

      // HTTPサーバーを起動
      await new Promise<void>((resolve, reject) => {
        if (!this.httpServer) {
          reject(new Error('HTTP server not initialized'));
          return;
        }

        this.httpServer.listen(this.config.port, () => {
          this.isRunning = true;
          
          // メモリ監視を開始
          this.memoryMonitor.start();
          
          this.logger.info('Server started successfully', {
            port: this.config.port,
            url: `http://localhost:${this.config.port}`,
            healthCheckUrl: `http://localhost:${this.config.port}/health`,
          });
          resolve();
        });

        this.httpServer.on('error', (error) => {
          this.logger.error('Failed to start server', error);
          reject(error);
        });
      });
    } catch (error) {
      this.logger.error('Error starting server', error as Error);
      this.isRunning = false;
      throw error;
    }
  }

  /**
   * サーバーを停止
   */
  async stop(): Promise<void> {
    if (!this.isRunning) {
      this.logger.warn('Server is not running');
      return;
    }

    try {
      this.logger.info('Stopping server...');

      // メモリ監視を停止
      this.memoryMonitor.stop();

      // 全てのハートビートを停止
      this.heartbeatManager.stopAll();

      // 全てのSocket.io接続をクローズ
      if (this.io) {
        const sockets = await this.io.fetchSockets();
        for (const socket of sockets) {
          socket.disconnect(true);
        }
        this.io.close();
        this.logger.info('All WebSocket connections closed');
      }

      // HTTPサーバーをクローズ
      if (this.httpServer) {
        await new Promise<void>((resolve, reject) => {
          this.httpServer!.close((error) => {
            if (error) {
              this.logger.error('Error closing HTTP server', error);
              reject(error);
            } else {
              this.logger.info('HTTP server closed');
              resolve();
            }
          });
        });
      }

      this.isRunning = false;
      this.logger.info('Server stopped successfully');
    } catch (error) {
      this.logger.error('Error stopping server', error as Error);
      throw error;
    }
  }


  /**
   * グローバルエラーハンドラーをセットアップ
   */
  setupGlobalErrorHandlers(): void {
    // 未処理のPromise拒否をキャッチ
    process.on('unhandledRejection', (reason: any, promise: Promise<any>) => {
      this.logger.error('Unhandled Promise Rejection', new Error(String(reason)), {
        reason: String(reason),
        promise: String(promise),
      });
      
      // 致命的なエラーの場合はシャットダウン
      if (this.isFatalError(reason)) {
        this.handleFatalError(reason);
      }
    });

    // 未処理の例外をキャッチ
    process.on('uncaughtException', (error: Error) => {
      this.logger.error('Uncaught Exception', error, {
        stack: error.stack,
      });
      
      // 致命的なエラーとして扱い、シャットダウン
      this.handleFatalError(error);
    });

    // Socket.ioエラーハンドラー
    if (this.io) {
      this.io.engine.on('connection_error', (error: any) => {
        this.logger.error('Socket.io connection error', error, {
          code: error.code,
          message: error.message,
        });
      });
    }

    this.logger.info('Global error handlers setup completed');
  }

  /**
   * エラーが致命的かどうかを判定
   */
  private isFatalError(error: any): boolean {
    // メモリ不足エラー
    if (error instanceof Error && error.message.includes('out of memory')) {
      return true;
    }

    // ポート使用中エラー
    if (error?.code === 'EADDRINUSE') {
      return true;
    }

    // その他の致命的なシステムエラー
    const fatalCodes = ['EACCES', 'ENOSPC', 'EMFILE'];
    if (error?.code && fatalCodes.includes(error.code)) {
      return true;
    }

    return false;
  }

  /**
   * 致命的エラーを処理
   */
  private async handleFatalError(error: any): Promise<void> {
    this.logger.error('Fatal error detected, initiating shutdown', error as Error);

    try {
      // 優雅なシャットダウンを試みる
      await this.stop();
    } catch (shutdownError) {
      this.logger.error('Error during fatal error shutdown', shutdownError as Error);
    } finally {
      // 終了コード1でプロセスを終了
      process.exit(1);
    }
  }

  /**
   * サーバーが実行中かどうかを取得
   */
  isServerRunning(): boolean {
    return this.isRunning;
  }
}
