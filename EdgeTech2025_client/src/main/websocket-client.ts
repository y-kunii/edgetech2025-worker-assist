import WebSocket from 'ws';
import { EventEmitter } from 'events';
import { StatusData, CommandData } from '../shared/types';
import { DataProcessor, DataValidationError } from './data-processor';
import { CommandManager, CommandSendResult } from './command-manager';

export interface WebSocketClientConfig {
  url: string;
  reconnectInterval: number;
  maxReconnectAttempts: number;
  heartbeatInterval: number;
}

export interface ConnectionState {
  isConnected: boolean;
  lastHeartbeat: Date | null;
  reconnectAttempts: number;
  error: string | null;
}

export class WebSocketClient extends EventEmitter {
  private ws: WebSocket | null = null;
  private config: WebSocketClientConfig;
  private connectionState: ConnectionState;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private heartbeatTimer: NodeJS.Timeout | null = null;
  private isReconnecting: boolean = false;
  private lastStatusData: StatusData | null = null;
  private commandManager: CommandManager;

  constructor(config: WebSocketClientConfig) {
    super();
    this.config = config;
    this.connectionState = {
      isConnected: false,
      lastHeartbeat: null,
      reconnectAttempts: 0,
      error: null
    };
    
    this.commandManager = new CommandManager();
    this.setupCommandManagerEvents();
  }

  /**
   * WebSocket接続を開始
   */
  public connect(): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      console.log('WebSocket is already connected');
      return;
    }

    try {
      console.log(`Connecting to WebSocket server: ${this.config.url}`);
      this.ws = new WebSocket(this.config.url);
      
      this.setupEventHandlers();
    } catch (error) {
      console.error('Failed to create WebSocket connection:', error);
      this.handleConnectionError(error as Error);
    }
  }

  /**
   * WebSocket接続を切断
   */
  public disconnect(): void {
    console.log('Disconnecting WebSocket...');
    this.isReconnecting = false;
    
    this.clearTimers();
    
    // 保留中のコマンドをキャンセル
    this.commandManager.cancelAllPendingCommands();
    
    if (this.ws) {
      this.ws.close(1000, 'Client disconnect');
      this.ws = null;
    }
    
    this.updateConnectionState({
      isConnected: false,
      lastHeartbeat: null,
      reconnectAttempts: 0,
      error: null
    });
  }

  /**
   * コマンドデータを送信（履歴管理付き）
   */
  public async sendCommand(command: CommandData): Promise<CommandSendResult> {
    // コマンドの検証
    if (!CommandManager.validateCommand(command)) {
      throw new Error('Invalid command data');
    }

    // CommandManagerを使用して送信
    return this.commandManager.sendCommand(command, async (cmd) => {
      return this.sendRawCommand(cmd);
    });
  }

  /**
   * 新しいコマンドを作成して送信
   */
  public async createAndSendCommand(commandType: 'tool_handover' | 'tool_collection' | 'wait'): Promise<CommandSendResult> {
    const command = this.commandManager.createCommand(commandType);
    return this.sendCommand(command);
  }

  /**
   * 生のコマンドデータを送信（内部使用）
   */
  private sendRawCommand(command: CommandData): Promise<boolean> {
    return new Promise((resolve, reject) => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
        const error = new Error('WebSocket is not connected');
        console.error('Cannot send command:', error.message);
        reject(error);
        return;
      }

      try {
        const message = JSON.stringify(command);
        console.log('Sending raw command:', message);
        
        this.ws.send(message, (error) => {
          if (error) {
            console.error('Failed to send command:', error);
            reject(error);
          } else {
            console.log('Command sent successfully');
            resolve(true);
          }
        });
      } catch (error) {
        console.error('Failed to serialize command:', error);
        reject(error);
      }
    });
  }

  /**
   * 現在の接続状態を取得
   */
  public getConnectionState(): ConnectionState {
    return { ...this.connectionState };
  }

  /**
   * 最後に受信したStatusDataを取得
   */
  public getLastStatusData(): StatusData | null {
    return this.lastStatusData ? { ...this.lastStatusData } : null;
  }

  /**
   * コマンドマネージャーを取得
   */
  public getCommandManager(): CommandManager {
    return this.commandManager;
  }

  /**
   * 設定を更新
   */
  public updateConfig(newConfig: Partial<WebSocketClientConfig>): void {
    this.config = { ...this.config, ...newConfig };
    console.log('WebSocket config updated:', this.config);
  }

  /**
   * WebSocketイベントハンドラーを設定
   */
  private setupEventHandlers(): void {
    if (!this.ws) return;

    this.ws.on('open', () => {
      console.log('WebSocket connected successfully');
      this.updateConnectionState({
        isConnected: true,
        lastHeartbeat: new Date(),
        reconnectAttempts: 0,
        error: null
      });
      
      this.startHeartbeat();
      this.emit('connected');
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const message = data.toString();
        console.log('Received raw message:', message);
        
        // ハートビート更新
        this.updateConnectionState({
          ...this.connectionState,
          lastHeartbeat: new Date()
        });

        // データ検証とパース
        const statusData = DataProcessor.validateAndParseStatusData(message);
        console.log('Validated status data:', statusData);

        // 状態変化の検出
        const changeInfo = DataProcessor.detectStatusChange(this.lastStatusData, statusData);
        console.log('Status change detected:', changeInfo);

        // 前回のデータを更新
        this.lastStatusData = statusData;

        // イベントを発行
        this.emit('statusData', statusData);
        
        if (changeInfo.hasChanged) {
          this.emit('statusDataChanged', {
            statusData,
            changeInfo
          });
        }

        if (changeInfo.workTaskChanged) {
          this.emit('workTaskChanged', {
            previousTask: changeInfo.previousTask,
            currentTask: changeInfo.currentTask,
            statusData
          });
        }

      } catch (error) {
        console.error('Failed to process received message:', error);
        
        if (error instanceof DataValidationError) {
          this.emit('dataValidationError', {
            error: error.message,
            field: error.field,
            rawMessage: data.toString()
          });
        } else {
          this.emit('error', error);
        }
      }
    });

    this.ws.on('close', (code: number, reason: Buffer) => {
      const reasonString = reason.toString();
      console.log(`WebSocket closed: ${code} - ${reasonString}`);
      
      this.updateConnectionState({
        isConnected: false,
        lastHeartbeat: null,
        reconnectAttempts: this.connectionState.reconnectAttempts,
        error: `Connection closed: ${code} - ${reasonString}`
      });
      
      this.clearTimers();
      this.emit('disconnected', { code, reason: reasonString });
      
      // 自動再接続を試行（意図的な切断でない場合）
      if (code !== 1000 && !this.isReconnecting) {
        this.attemptReconnect();
      }
    });

    this.ws.on('error', (error: Error) => {
      console.error('WebSocket error:', error);
      this.handleConnectionError(error);
    });
  }

  /**
   * 接続エラーを処理
   */
  private handleConnectionError(error: Error): void {
    this.updateConnectionState({
      isConnected: false,
      lastHeartbeat: null,
      reconnectAttempts: this.connectionState.reconnectAttempts,
      error: error.message
    });
    
    this.emit('error', error);
    
    if (!this.isReconnecting) {
      this.attemptReconnect();
    }
  }

  /**
   * 自動再接続を試行
   */
  private attemptReconnect(): void {
    if (this.connectionState.reconnectAttempts >= this.config.maxReconnectAttempts) {
      console.log('Max reconnect attempts reached');
      this.emit('maxReconnectAttemptsReached');
      return;
    }

    this.isReconnecting = true;
    const attempts = this.connectionState.reconnectAttempts + 1;
    
    this.updateConnectionState({
      ...this.connectionState,
      reconnectAttempts: attempts
    });

    // 指数バックオフで再接続間隔を調整
    const delay = Math.min(
      this.config.reconnectInterval * Math.pow(2, attempts - 1),
      30000 // 最大30秒
    );

    console.log(`Attempting reconnect ${attempts}/${this.config.maxReconnectAttempts} in ${delay}ms`);
    
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, delay);
  }

  /**
   * ハートビート監視を開始
   */
  private startHeartbeat(): void {
    this.clearHeartbeatTimer();
    
    this.heartbeatTimer = setInterval(() => {
      const now = new Date();
      const lastHeartbeat = this.connectionState.lastHeartbeat;
      
      if (lastHeartbeat) {
        const timeSinceLastHeartbeat = now.getTime() - lastHeartbeat.getTime();
        
        // ハートビートタイムアウトをチェック
        if (timeSinceLastHeartbeat > this.config.heartbeatInterval * 2) {
          console.warn('Heartbeat timeout detected');
          this.ws?.close(1001, 'Heartbeat timeout');
        }
      }
    }, this.config.heartbeatInterval);
  }

  /**
   * CommandManagerのイベントハンドラーを設定
   */
  private setupCommandManagerEvents(): void {
    this.commandManager.on('commandSent', (data) => {
      this.emit('commandSent', data);
    });

    this.commandManager.on('commandFailed', (data) => {
      this.emit('commandFailed', data);
    });

    this.commandManager.on('commandError', (data) => {
      this.emit('commandError', data);
    });

    this.commandManager.on('commandTimeout', (data) => {
      this.emit('commandTimeout', data);
    });

    this.commandManager.on('historyUpdated', (entry) => {
      this.emit('commandHistoryUpdated', entry);
    });

    this.commandManager.on('allCommandsCancelled', () => {
      this.emit('allCommandsCancelled');
    });

    this.commandManager.on('historyCleared', () => {
      this.emit('commandHistoryCleared');
    });
  }

  /**
   * 接続状態を更新
   */
  private updateConnectionState(newState: Partial<ConnectionState>): void {
    this.connectionState = { ...this.connectionState, ...newState };
    this.emit('connectionStateChanged', this.connectionState);
  }

  /**
   * タイマーをクリア
   */
  private clearTimers(): void {
    this.clearReconnectTimer();
    this.clearHeartbeatTimer();
  }

  private clearReconnectTimer(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  private clearHeartbeatTimer(): void {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }
}