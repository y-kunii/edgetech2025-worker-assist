/**
 * HeartbeatManager
 * クライアントのハートビート管理とタイムアウト検出を担当
 */

import { Socket } from 'socket.io';
import { Logger } from '../utils/Logger';

interface HeartbeatInfo {
  intervalId: NodeJS.Timeout;
  lastPongTime: number;
  socket: Socket;
}

export class HeartbeatManager {
  private heartbeats: Map<string, HeartbeatInfo>;
  private logger: Logger;
  private heartbeatInterval: number;
  private connectionTimeout: number;
  private timeoutCheckInterval: NodeJS.Timeout | null;

  constructor(logger: Logger, heartbeatInterval: number = 30000, connectionTimeout: number = 60000) {
    this.heartbeats = new Map();
    this.logger = logger;
    this.heartbeatInterval = heartbeatInterval;
    this.connectionTimeout = connectionTimeout;
    this.timeoutCheckInterval = null;

    // タイムアウトチェックを定期的に実行
    this.startTimeoutChecker();
  }

  /**
   * クライアントのハートビートを開始
   */
  startHeartbeat(socket: Socket): void {
    const socketId = socket.id;

    // 既存のハートビートがあれば停止
    if (this.heartbeats.has(socketId)) {
      this.stopHeartbeat(socketId);
    }

    // 初期pong時刻を設定
    const now = Date.now();

    // 定期的にpingを送信
    const intervalId = setInterval(() => {
      if (socket.connected) {
        socket.emit('ping', { timestamp: Date.now() });
        this.logger.debug(`Heartbeat ping sent to client`, { socketId });
      }
    }, this.heartbeatInterval);

    // ハートビート情報を保存
    this.heartbeats.set(socketId, {
      intervalId,
      lastPongTime: now,
      socket,
    });

    this.logger.debug(`Heartbeat started for client`, {
      socketId,
      heartbeatInterval: this.heartbeatInterval,
    });
  }

  /**
   * クライアントのハートビートを停止
   */
  stopHeartbeat(socketId: string): void {
    const heartbeatInfo = this.heartbeats.get(socketId);
    if (heartbeatInfo) {
      clearInterval(heartbeatInfo.intervalId);
      this.heartbeats.delete(socketId);
      this.logger.debug(`Heartbeat stopped for client`, { socketId });
    }
  }

  /**
   * pong受信を処理
   */
  handlePong(socketId: string): void {
    const heartbeatInfo = this.heartbeats.get(socketId);
    if (heartbeatInfo) {
      heartbeatInfo.lastPongTime = Date.now();
      this.logger.debug(`Pong received from client`, { socketId });
    }
  }

  /**
   * タイムアウトをチェックし、タイムアウトしたクライアントを切断
   */
  checkTimeouts(): string[] {
    const now = Date.now();
    const timedOutClients: string[] = [];

    for (const [socketId, heartbeatInfo] of this.heartbeats.entries()) {
      const timeSinceLastPong = now - heartbeatInfo.lastPongTime;
      
      if (timeSinceLastPong > this.connectionTimeout) {
        timedOutClients.push(socketId);
        this.logger.warn(`Client connection timeout detected, disconnecting`, {
          socketId,
          timeSinceLastPong,
          connectionTimeout: this.connectionTimeout,
        });

        // タイムアウトしたクライアントを切断
        if (heartbeatInfo.socket.connected) {
          heartbeatInfo.socket.disconnect(true);
        }

        // ハートビートを停止
        this.stopHeartbeat(socketId);
      }
    }

    return timedOutClients;
  }

  /**
   * タイムアウトチェッカーを開始
   */
  private startTimeoutChecker(): void {
    // 10秒ごとにタイムアウトをチェック
    this.timeoutCheckInterval = setInterval(() => {
      this.checkTimeouts();
    }, 10000);
  }

  /**
   * タイムアウトチェッカーを停止
   */
  stopTimeoutChecker(): void {
    if (this.timeoutCheckInterval) {
      clearInterval(this.timeoutCheckInterval);
      this.timeoutCheckInterval = null;
    }
  }

  /**
   * 全てのハートビートを停止（シャットダウン時）
   */
  stopAll(): void {
    for (const socketId of this.heartbeats.keys()) {
      this.stopHeartbeat(socketId);
    }
    this.stopTimeoutChecker();
    this.logger.info('All heartbeats stopped');
  }

  /**
   * アクティブなハートビート数を取得
   */
  getActiveHeartbeatsCount(): number {
    return this.heartbeats.size;
  }

  /**
   * 全ハートビート情報を取得（テスト用）
   */
  getAllHeartbeats(): Map<string, HeartbeatInfo> {
    return this.heartbeats;
  }
}
