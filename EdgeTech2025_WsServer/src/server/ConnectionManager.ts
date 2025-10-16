/**
 * ConnectionManager
 * WebSocket接続の管理を担当
 */

import { Socket } from 'socket.io';
import { ClientType, ConnectionStats } from '../types';
import { Logger } from '../utils/Logger';

interface ClientInfo {
  socket: Socket;
  clientType: ClientType;
  connectedAt: Date;
}

export class ConnectionManager {
  private clients: Map<string, ClientInfo>;
  private logger: Logger;

  constructor(logger: Logger) {
    this.clients = new Map();
    this.logger = logger;
  }

  /**
   * クライアントを登録
   * センサー/ロボットクライアントの場合、既存接続があれば切断
   */
  registerClient(socketId: string, clientType: ClientType, socket: Socket): void {
    // センサーまたはロボットクライアントの場合、既存接続をチェック
    if (clientType === 'sensor' || clientType === 'robot') {
      const existingClient = this.getClientByType(clientType);
      if (existingClient) {
        this.logger.warn(`Duplicate ${clientType} client detected. Disconnecting existing connection.`, {
          existingSocketId: existingClient.id,
          newSocketId: socketId,
        });
        existingClient.disconnect(true);
        this.unregisterClient(existingClient.id);
      }
    }

    // クライアント情報を登録
    this.clients.set(socketId, {
      socket,
      clientType,
      connectedAt: new Date(),
    });

    this.logger.info(`Client registered: ${clientType}`, {
      socketId,
      totalClients: this.clients.size,
    });
  }

  /**
   * クライアントの登録を解除
   */
  unregisterClient(socketId: string): void {
    const clientInfo = this.clients.get(socketId);
    if (clientInfo) {
      this.logger.info(`Client unregistered: ${clientInfo.clientType}`, {
        socketId,
        totalClients: this.clients.size - 1,
      });
      this.clients.delete(socketId);
    }
  }

  /**
   * 指定されたタイプのクライアントを取得
   */
  getClientsByType(clientType: ClientType): Socket[] {
    const sockets: Socket[] = [];
    for (const clientInfo of this.clients.values()) {
      if (clientInfo.clientType === clientType) {
        sockets.push(clientInfo.socket);
      }
    }
    return sockets;
  }

  /**
   * センサークライアントを取得（単一接続）
   */
  getSensorClient(): Socket | null {
    return this.getClientByType('sensor');
  }

  /**
   * ロボットクライアントを取得（単一接続）
   */
  getRobotClient(): Socket | null {
    return this.getClientByType('robot');
  }

  /**
   * Electronクライアントを取得（複数接続可能）
   */
  getElectronClients(): Socket[] {
    return this.getClientsByType('electron');
  }

  /**
   * 接続統計を取得
   */
  getConnectionStats(): ConnectionStats {
    const stats: ConnectionStats = {
      total: this.clients.size,
      byType: {
        electron: 0,
        sensor: 0,
        robot: 0,
        unknown: 0,
      },
    };

    for (const clientInfo of this.clients.values()) {
      stats.byType[clientInfo.clientType]++;
    }

    return stats;
  }

  /**
   * 指定されたタイプのクライアントが接続されているかチェック
   */
  isClientConnected(clientType: ClientType): boolean {
    for (const clientInfo of this.clients.values()) {
      if (clientInfo.clientType === clientType) {
        return true;
      }
    }
    return false;
  }

  /**
   * 指定されたタイプの単一クライアントを取得（内部ヘルパー）
   */
  private getClientByType(clientType: ClientType): Socket | null {
    for (const clientInfo of this.clients.values()) {
      if (clientInfo.clientType === clientType) {
        return clientInfo.socket;
      }
    }
    return null;
  }

  /**
   * 全クライアント数を取得
   */
  getTotalClients(): number {
    return this.clients.size;
  }

  /**
   * 全クライアントを取得（テスト用）
   */
  getAllClients(): Map<string, ClientInfo> {
    return this.clients;
  }
}
