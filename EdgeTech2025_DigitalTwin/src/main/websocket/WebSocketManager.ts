import { io, Socket } from 'socket.io-client';
import { EventEmitter } from 'events';

export interface ConnectionConfig {
  url: string;
  reconnectionAttempts?: number;
  reconnectionDelay?: number;
  timeout?: number;
}

export enum ConnectionState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  RECONNECTING = 'reconnecting',
  ERROR = 'error'
}

export interface ConnectionStatus {
  state: ConnectionState;
  lastConnected?: Date;
  reconnectAttempts: number;
  latency?: number;
}

export class WebSocketManager extends EventEmitter {
  private socket: Socket | null = null;
  private config: ConnectionConfig;
  private connectionStatus: ConnectionStatus;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private pingInterval: NodeJS.Timeout | null = null;

  constructor(config: ConnectionConfig) {
    super();
    this.config = {
      reconnectionAttempts: 10,
      reconnectionDelay: 1000,
      timeout: 5000,
      ...config
    };
    
    this.connectionStatus = {
      state: ConnectionState.DISCONNECTED,
      reconnectAttempts: 0
    };
  }

  /**
   * WebSocket接続を開始
   */
  public connect(): void {
    if (this.socket && this.socket.connected) {
      console.log('WebSocket is already connected');
      return;
    }

    this.updateConnectionState(ConnectionState.CONNECTING);
    
    try {
      this.socket = io(this.config.url, {
        timeout: this.config.timeout,
        autoConnect: false,
        transports: ['websocket']
      });

      this.setupEventHandlers();
      this.socket.connect();
    } catch (error) {
      console.error('Failed to create WebSocket connection:', error);
      this.handleConnectionError(error as Error);
    }
  }

  /**
   * WebSocket接続を切断
   */
  public disconnect(): void {
    this.clearReconnectTimer();
    this.clearPingInterval();
    
    if (this.socket) {
      this.socket.disconnect();
      this.socket = null;
    }
    
    this.updateConnectionState(ConnectionState.DISCONNECTED);
  }

  /**
   * メッセージを送信
   */
  public sendMessage(event: string, data: any): boolean {
    if (!this.socket || !this.socket.connected) {
      console.warn('Cannot send message: WebSocket not connected');
      this.emit('error', new Error('WebSocket not connected'));
      return false;
    }

    try {
      this.socket.emit(event, data);
      return true;
    } catch (error) {
      console.error('Failed to send message:', error);
      this.emit('error', error);
      return false;
    }
  }

  /**
   * 接続状態を取得
   */
  public getConnectionStatus(): ConnectionStatus {
    return { ...this.connectionStatus };
  }

  /**
   * 接続中かどうかを確認
   */
  public isConnected(): boolean {
    return this.socket?.connected === true;
  }

  /**
   * イベントハンドラーを設定
   */
  private setupEventHandlers(): void {
    if (!this.socket) return;

    this.socket.on('connect', () => {
      console.log('WebSocket connected successfully');
      this.connectionStatus.lastConnected = new Date();
      this.connectionStatus.reconnectAttempts = 0;
      this.updateConnectionState(ConnectionState.CONNECTED);
      this.startPingMonitoring();
      this.emit('connected');
    });

    this.socket.on('disconnect', (reason) => {
      console.log('WebSocket disconnected:', reason);
      this.clearPingInterval();
      this.updateConnectionState(ConnectionState.DISCONNECTED);
      this.emit('disconnected', reason);
      
      // 自動再接続を試行（サーバー側からの切断でない場合）
      if (reason !== 'io server disconnect') {
        this.scheduleReconnect();
      }
    });

    this.socket.on('connect_error', (error) => {
      console.error('WebSocket connection error:', error);
      this.handleConnectionError(error);
    });

    this.socket.on('error', (error) => {
      console.error('WebSocket error:', error);
      this.emit('error', error);
    });

    // データ受信イベント
    this.socket.on('sensor_data', (data) => {
      this.emit('sensor_data', data);
    });

    this.socket.on('robot_response', (data) => {
      this.emit('robot_response', data);
    });

    // Ping/Pong for latency monitoring
    this.socket.on('pong', (startTime: number) => {
      const latency = Date.now() - startTime;
      this.connectionStatus.latency = latency;
      this.emit('latency_update', latency);
    });
  }

  /**
   * 接続エラーを処理
   */
  private handleConnectionError(error: Error): void {
    this.updateConnectionState(ConnectionState.ERROR);
    this.emit('error', error);
    this.scheduleReconnect();
  }

  /**
   * 再接続をスケジュール
   */
  private scheduleReconnect(): void {
    if (this.connectionStatus.reconnectAttempts >= (this.config.reconnectionAttempts || 10)) {
      console.error('Max reconnection attempts reached');
      this.emit('max_reconnect_attempts_reached');
      return;
    }

    this.clearReconnectTimer();
    this.updateConnectionState(ConnectionState.RECONNECTING);
    
    const delay = this.calculateReconnectDelay();
    console.log(`Scheduling reconnect in ${delay}ms (attempt ${this.connectionStatus.reconnectAttempts + 1})`);
    
    this.reconnectTimer = setTimeout(() => {
      this.connectionStatus.reconnectAttempts++;
      this.connect();
    }, delay);
  }

  /**
   * 再接続遅延時間を計算（指数バックオフ）
   */
  private calculateReconnectDelay(): number {
    const baseDelay = this.config.reconnectionDelay || 1000;
    const maxDelay = 30000; // 最大30秒
    const delay = Math.min(baseDelay * Math.pow(2, this.connectionStatus.reconnectAttempts), maxDelay);
    return delay + Math.random() * 1000; // ジッターを追加
  }

  /**
   * 接続状態を更新
   */
  private updateConnectionState(state: ConnectionState): void {
    if (this.connectionStatus.state !== state) {
      this.connectionStatus.state = state;
      this.emit('connection_state_changed', state);
    }
  }

  /**
   * Ping監視を開始
   */
  private startPingMonitoring(): void {
    this.clearPingInterval();
    this.pingInterval = setInterval(() => {
      if (this.socket && this.socket.connected) {
        this.socket.emit('ping', Date.now());
      }
    }, 5000); // 5秒間隔でping
  }

  /**
   * 再接続タイマーをクリア
   */
  private clearReconnectTimer(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  /**
   * Ping間隔をクリア
   */
  private clearPingInterval(): void {
    if (this.pingInterval) {
      clearInterval(this.pingInterval);
      this.pingInterval = null;
    }
  }
}