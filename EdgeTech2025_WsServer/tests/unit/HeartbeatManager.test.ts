/**
 * HeartbeatManager 単体テスト
 */

import { HeartbeatManager } from '../../src/server/HeartbeatManager';
import { Logger } from '../../src/utils/Logger';

// Socket.ioのSocketをモック
class MockSocket {
  id: string;
  connected: boolean = true;
  disconnected: boolean = false;
  emittedEvents: Array<{ event: string; data: any }> = [];

  constructor(id: string) {
    this.id = id;
  }

  emit(event: string, data: any): void {
    this.emittedEvents.push({ event, data });
  }

  disconnect(_close?: boolean): void {
    this.connected = false;
    this.disconnected = true;
  }
}

describe('HeartbeatManager', () => {
  let heartbeatManager: HeartbeatManager;
  let logger: Logger;
  const baseTime = new Date('2025-01-01T00:00:00.000Z').getTime();

  beforeEach(() => {
    // Loggerのモック
    logger = {
      debug: jest.fn(),
      info: jest.fn(),
      warn: jest.fn(),
      error: jest.fn(),
    } as any;

    // タイマーをモック化（Date.now()も含む）
    jest.useFakeTimers();
    jest.setSystemTime(baseTime);
  });

  afterEach(() => {
    // タイマーをクリーンアップ
    if (heartbeatManager) {
      heartbeatManager.stopAll();
    }
    jest.clearAllTimers();
    jest.useRealTimers();
  });

  describe('startHeartbeat', () => {
    test('should start heartbeat for client', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);

      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(1);
      expect(logger.debug).toHaveBeenCalledWith(
        expect.stringContaining('Heartbeat started'),
        expect.any(Object)
      );
    });

    test('should send ping at regular intervals', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);

      // 最初はpingが送信されていない
      expect(socket.emittedEvents.length).toBe(0);

      // 30秒進める
      jest.advanceTimersByTime(30000);

      // pingが送信される
      expect(socket.emittedEvents.length).toBe(1);
      expect(socket.emittedEvents[0].event).toBe('ping');
      expect(socket.emittedEvents[0].data).toHaveProperty('timestamp');

      // さらに30秒進める
      jest.advanceTimersByTime(30000);

      // 2回目のpingが送信される
      expect(socket.emittedEvents.length).toBe(2);
    });

    test('should not send ping if socket is disconnected', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);

      // ソケットを切断
      socket.connected = false;

      // 30秒進める
      jest.advanceTimersByTime(30000);

      // pingは送信されない
      expect(socket.emittedEvents.length).toBe(0);
    });

    test('should stop existing heartbeat when starting new one for same socket', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(1);

      heartbeatManager.startHeartbeat(socket);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(1);
    });
  });

  describe('stopHeartbeat', () => {
    test('should stop heartbeat for client', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(1);

      heartbeatManager.stopHeartbeat('socket-1');
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(0);
      expect(logger.debug).toHaveBeenCalledWith(
        expect.stringContaining('Heartbeat stopped'),
        expect.any(Object)
      );
    });

    test('should not send ping after heartbeat is stopped', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);
      heartbeatManager.stopHeartbeat('socket-1');

      // 30秒進める
      jest.advanceTimersByTime(30000);

      // pingは送信されない
      expect(socket.emittedEvents.length).toBe(0);
    });

    test('should handle stopping non-existent heartbeat', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);

      heartbeatManager.stopHeartbeat('non-existent');
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(0);
    });
  });

  describe('handlePong', () => {
    test('should update last pong time when pong received', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);

      const heartbeats = heartbeatManager.getAllHeartbeats();
      const initialPongTime = heartbeats.get('socket-1')?.lastPongTime;

      // 時間を進める
      jest.advanceTimersByTime(5000);

      // pongを受信
      heartbeatManager.handlePong('socket-1');

      const updatedPongTime = heartbeats.get('socket-1')?.lastPongTime;

      expect(updatedPongTime).toBeGreaterThan(initialPongTime!);
      expect(logger.debug).toHaveBeenCalledWith(
        expect.stringContaining('Pong received'),
        expect.any(Object)
      );
    });

    test('should handle pong for non-existent heartbeat', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);

      // エラーが発生しないことを確認
      expect(() => {
        heartbeatManager.handlePong('non-existent');
      }).not.toThrow();
    });
  });

  describe('checkTimeouts', () => {
    test('should detect timeout when no pong received', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);

      // タイムアウト時間を超えて進める
      jest.advanceTimersByTime(65000);

      const timedOutClients = heartbeatManager.checkTimeouts();

      expect(timedOutClients).toContain('socket-1');
      expect(logger.warn).toHaveBeenCalledWith(
        expect.stringContaining('timeout detected'),
        expect.any(Object)
      );
    });

    test('should disconnect timed out client', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);

      // タイムアウト時間を超えて進める
      jest.advanceTimersByTime(65000);

      heartbeatManager.checkTimeouts();

      expect(socket.disconnected).toBe(true);
    });

    test('should stop heartbeat for timed out client', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(1);

      // タイムアウト時間を超えて進める
      jest.advanceTimersByTime(65000);

      heartbeatManager.checkTimeouts();

      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(0);
    });

    test('should not timeout when pong received within timeout period', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);

      // 50秒進める（タイムアウト前）
      jest.advanceTimersByTime(50000);

      // pongを受信
      heartbeatManager.handlePong('socket-1');

      // さらに50秒進める（最後のpongから50秒、開始から100秒）
      jest.advanceTimersByTime(50000);

      const timedOutClients = heartbeatManager.checkTimeouts();

      // タイムアウトしない（最後のpongから60秒経っていない）
      expect(timedOutClients).toHaveLength(0);
      expect(socket.disconnected).toBe(false);
    });

    test('should handle multiple clients with different timeout states', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;

      // socket1を開始（時刻: 0）
      heartbeatManager.startHeartbeat(socket1);
      
      // 30秒進める（時刻: 30000）
      jest.setSystemTime(baseTime + 30000);
      jest.advanceTimersByTime(30000);
      
      // socket2を開始（時刻: 30000）
      heartbeatManager.startHeartbeat(socket2);

      // さらに40秒進める（時刻: 70000、socket1は70秒経過、socket2は40秒経過）
      // 自動タイムアウトチェックが実行される（10秒ごと）
      jest.setSystemTime(baseTime + 70000);
      jest.advanceTimersByTime(40000);

      // socket1のみタイムアウト（60秒超過）、自動チェックで切断される
      expect(socket1.disconnected).toBe(true);
      expect(socket2.disconnected).toBe(false);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(1);
    });
  });

  describe('automatic timeout checking', () => {
    test('should automatically check timeouts periodically', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);

      // タイムアウト時間を超えて進める（時刻とタイマーを同期）
      jest.setSystemTime(baseTime + 65000);
      jest.advanceTimersByTime(65000);

      // 自動チェックが実行される（10秒ごと）
      expect(socket.disconnected).toBe(true);
    });
  });

  describe('stopAll', () => {
    test('should stop all heartbeats', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;

      heartbeatManager.startHeartbeat(socket1);
      heartbeatManager.startHeartbeat(socket2);

      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(2);

      heartbeatManager.stopAll();

      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(0);
      expect(logger.info).toHaveBeenCalledWith('All heartbeats stopped');
    });

    test('should stop timeout checker', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      heartbeatManager.startHeartbeat(socket);
      heartbeatManager.stopAll();

      // タイムアウト時間を超えて進める
      jest.advanceTimersByTime(65000);

      // タイムアウトチェックが実行されない（ソケットは切断されない）
      expect(socket.disconnected).toBe(false);
    });
  });

  describe('getActiveHeartbeatsCount', () => {
    test('should return correct count of active heartbeats', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;
      const socket3 = new MockSocket('socket-3') as any;

      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(0);

      heartbeatManager.startHeartbeat(socket1);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(1);

      heartbeatManager.startHeartbeat(socket2);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(2);

      heartbeatManager.startHeartbeat(socket3);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(3);

      heartbeatManager.stopHeartbeat('socket-2');
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(2);
    });
  });

  describe('integration scenarios', () => {
    test('should handle complete heartbeat lifecycle', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      // ハートビート開始
      heartbeatManager.startHeartbeat(socket);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(1);

      // 30秒進めてpingを送信
      jest.advanceTimersByTime(30000);
      expect(socket.emittedEvents.length).toBe(1);
      expect(socket.emittedEvents[0].event).toBe('ping');

      // pongを受信
      heartbeatManager.handlePong('socket-1');

      // さらに30秒進める
      jest.advanceTimersByTime(30000);
      expect(socket.emittedEvents.length).toBe(2);

      // タイムアウトチェック（タイムアウトしない）
      const timedOutClients = heartbeatManager.checkTimeouts();
      expect(timedOutClients).toHaveLength(0);
      expect(socket.disconnected).toBe(false);

      // ハートビート停止
      heartbeatManager.stopHeartbeat('socket-1');
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(0);
    });

    test('should handle timeout scenario without pong', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket = new MockSocket('socket-1') as any;

      // ハートビート開始（時刻: 0）
      heartbeatManager.startHeartbeat(socket);

      // pingを複数回送信（pongなし）
      jest.advanceTimersByTime(30000);
      expect(socket.emittedEvents.length).toBe(1);

      jest.advanceTimersByTime(30000);
      expect(socket.emittedEvents.length).toBe(2);

      // タイムアウト時間を超える（時刻: 70000）
      // 自動タイムアウトチェックが実行される
      jest.setSystemTime(baseTime + 70000);
      jest.advanceTimersByTime(10000);

      // タイムアウト検出され、自動的に切断される
      expect(socket.disconnected).toBe(true);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(0);
    });

    test('should handle multiple clients with mixed pong responses', () => {
      heartbeatManager = new HeartbeatManager(logger, 30000, 60000);
      const socket1 = new MockSocket('socket-1') as any;
      const socket2 = new MockSocket('socket-2') as any;
      const socket3 = new MockSocket('socket-3') as any;

      // 全クライアントのハートビート開始（時刻: 0）
      heartbeatManager.startHeartbeat(socket1);
      heartbeatManager.startHeartbeat(socket2);
      heartbeatManager.startHeartbeat(socket3);

      // 30秒進める（時刻: 30000）
      jest.setSystemTime(baseTime + 30000);
      jest.advanceTimersByTime(30000);

      // socket1とsocket2のみpongを返す（lastPongTimeが30000に更新される）
      heartbeatManager.handlePong('socket-1');
      heartbeatManager.handlePong('socket-2');

      // さらに40秒進める（時刻: 70000）
      // socket1とsocket2は最後のpongから40秒（タイムアウトしない）
      // socket3は開始から70秒（タイムアウトする）
      // 自動タイムアウトチェックが実行される
      jest.setSystemTime(baseTime + 70000);
      jest.advanceTimersByTime(40000);

      // socket3のみタイムアウト、自動的に切断される
      expect(socket3.disconnected).toBe(true);
      expect(socket1.disconnected).toBe(false);
      expect(socket2.disconnected).toBe(false);
      expect(heartbeatManager.getActiveHeartbeatsCount()).toBe(2);
    });
  });
});
