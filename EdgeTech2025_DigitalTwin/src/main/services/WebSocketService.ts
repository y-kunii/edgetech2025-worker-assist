import { BrowserWindow } from 'electron';
import { EventEmitter } from 'events';
import { WebSocketManager, ConnectionConfig, ConnectionState } from '../websocket/WebSocketManager';
import { DataProcessor } from '../processors/DataProcessor';
import { RobotCommandManager } from './RobotCommandManager';

export class WebSocketService extends EventEmitter {
  private wsManager: WebSocketManager;
  private dataProcessor: DataProcessor;
  private robotCommandManager: RobotCommandManager;
  private mainWindow: BrowserWindow | null = null;

  constructor(config: ConnectionConfig) {
    super();
    this.wsManager = new WebSocketManager(config);
    this.dataProcessor = new DataProcessor();
    this.robotCommandManager = new RobotCommandManager(this);
    this.setupEventHandlers();
  }

  /**
   * メインウィンドウを設定
   */
  public setMainWindow(window: BrowserWindow): void {
    this.mainWindow = window;
    this.dataProcessor.setMainWindow(window);
    this.dataProcessor.setRobotCommandManager(this.robotCommandManager);
    this.setupRobotCommandHandlers();
  }

  /**
   * WebSocket接続を開始
   */
  public connect(): void {
    this.wsManager.connect();
  }

  /**
   * WebSocket接続を切断
   */
  public disconnect(): void {
    this.wsManager.disconnect();
  }

  /**
   * ロボットに指示を送信
   */
  public sendRobotCommand(command: string, data?: any): boolean {
    return this.wsManager.sendMessage('robot_command', {
      command,
      data,
      timestamp: new Date().toISOString()
    });
  }

  /**
   * 接続状態を取得
   */
  public getConnectionStatus() {
    return this.wsManager.getConnectionStatus();
  }

  /**
   * 接続中かどうかを確認
   */
  public isConnected(): boolean {
    return this.wsManager.isConnected();
  }

  /**
   * イベントハンドラーを設定
   */
  private setupEventHandlers(): void {
    // 接続状態の変更をレンダラープロセスに通知
    this.wsManager.on('connection_state_changed', (state: ConnectionState) => {
      this.sendToRenderer('websocket-connection-state', { state });
    });

    // 接続成功をレンダラープロセスに通知
    this.wsManager.on('connected', () => {
      console.log('WebSocket service: Connected to server');
      this.dataProcessor.updateConnectionStatus(true);
      this.sendToRenderer('websocket-connected');
    });

    // 切断をレンダラープロセスに通知
    this.wsManager.on('disconnected', (reason: string) => {
      console.log('WebSocket service: Disconnected from server:', reason);
      this.dataProcessor.updateConnectionStatus(false);
      this.sendToRenderer('websocket-disconnected', { reason });
    });

    // エラーをレンダラープロセスに通知
    this.wsManager.on('error', (error: Error) => {
      console.error('WebSocket service error:', error);
      this.sendToRenderer('websocket-error', { 
        message: error.message,
        stack: error.stack 
      });
    });

    // センサーデータを処理
    this.wsManager.on('sensor_data', (data: any) => {
      this.dataProcessor.processSensorData(data);
    });

    // ロボット応答をレンダラープロセスに転送
    this.wsManager.on('robot_response', (data: any) => {
      this.sendToRenderer('robot-response', data);
    });

    // レイテンシ更新をレンダラープロセスに通知
    this.wsManager.on('latency_update', (latency: number) => {
      this.dataProcessor.updateConnectionQuality(latency);
      this.sendToRenderer('websocket-latency', { latency });
    });

    // 最大再接続試行回数に達した場合の通知
    this.wsManager.on('max_reconnect_attempts_reached', () => {
      this.sendToRenderer('websocket-max-reconnect-reached');
    });
  }

  /**
   * レンダラープロセスにメッセージを送信
   */
  private sendToRenderer(channel: string, data?: any): void {
    if (this.mainWindow && !this.mainWindow.isDestroyed()) {
      this.mainWindow.webContents.send(channel, data);
    }
  }

  /**
   * データプロセッサを取得
   */
  public getDataProcessor(): DataProcessor {
    return this.dataProcessor;
  }

  /**
   * ロボットコマンドマネージャーを取得
   */
  public getRobotCommandManager(): RobotCommandManager {
    return this.robotCommandManager;
  }

  /**
   * ロボットコマンドハンドラーを設定
   */
  private setupRobotCommandHandlers(): void {
    // 閾値達成時のロボット指示送信
    this.robotCommandManager.on('command_sent', (commandLog) => {
      this.sendToRenderer('robot-command-sent', commandLog);
    });

    this.robotCommandManager.on('command_success', (commandLog) => {
      this.sendToRenderer('robot-command-success', commandLog);
    });

    this.robotCommandManager.on('command_error', (commandLog) => {
      this.sendToRenderer('robot-command-error', commandLog);
    });

    this.robotCommandManager.on('command_retry_scheduled', (commandLog) => {
      this.sendToRenderer('robot-command-retry-scheduled', commandLog);
    });

    this.robotCommandManager.on('command_cancelled', (commandLog) => {
      this.sendToRenderer('robot-command-cancelled', commandLog);
    });

    this.robotCommandManager.on('threshold_command_success', (data) => {
      this.sendToRenderer('threshold-command-success', data);
    });

    this.robotCommandManager.on('threshold_command_error', (data) => {
      this.sendToRenderer('threshold-command-error', data);
    });
  }

  /**
   * サービスを破棄
   */
  public destroy(): void {
    this.wsManager.disconnect();
    this.wsManager.removeAllListeners();
    this.dataProcessor.destroy();
    this.robotCommandManager.destroy();
    this.mainWindow = null;
  }
}