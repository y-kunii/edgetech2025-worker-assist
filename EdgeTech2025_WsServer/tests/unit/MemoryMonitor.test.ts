/**
 * MemoryMonitor単体テスト
 */

import { MemoryMonitor, MemoryThresholds } from '../../src/utils/MemoryMonitor';
import { Logger } from '../../src/utils/Logger';

// Loggerをモック
jest.mock('../../src/utils/Logger');

describe('MemoryMonitor', () => {
  let logger: jest.Mocked<Logger>;
  let memoryMonitor: MemoryMonitor;

  beforeEach(() => {
    // Loggerのモックを作成
    logger = {
      debug: jest.fn(),
      info: jest.fn(),
      warn: jest.fn(),
      error: jest.fn(),
    } as any;

    jest.clearAllMocks();
  });

  afterEach(() => {
    if (memoryMonitor && memoryMonitor.isRunning()) {
      memoryMonitor.stop();
    }
  });

  describe('初期化', () => {
    test('デフォルト設定で初期化できる', () => {
      memoryMonitor = new MemoryMonitor(logger);
      expect(memoryMonitor).toBeDefined();
      expect(memoryMonitor.isRunning()).toBe(false);
    });

    test('カスタム閾値で初期化できる', () => {
      const customThresholds: Partial<MemoryThresholds> = {
        rssWarning: 100 * 1024 * 1024,  // 100MB
        rssCritical: 200 * 1024 * 1024, // 200MB
      };

      memoryMonitor = new MemoryMonitor(logger, customThresholds);
      expect(memoryMonitor).toBeDefined();
    });

    test('カスタムチェック間隔で初期化できる', () => {
      memoryMonitor = new MemoryMonitor(logger, undefined, 10000); // 10秒
      expect(memoryMonitor).toBeDefined();
    });
  });

  describe('監視の開始と停止', () => {
    test('監視を開始できる', () => {
      memoryMonitor = new MemoryMonitor(logger, undefined, 100000); // 長い間隔
      memoryMonitor.start();

      expect(memoryMonitor.isRunning()).toBe(true);
      expect(logger.info).toHaveBeenCalledWith(
        'Starting memory monitor',
        expect.any(Object)
      );
    });

    test('監視を停止できる', () => {
      memoryMonitor = new MemoryMonitor(logger, undefined, 100000);
      memoryMonitor.start();
      expect(memoryMonitor.isRunning()).toBe(true);

      memoryMonitor.stop();
      expect(memoryMonitor.isRunning()).toBe(false);
      expect(logger.info).toHaveBeenCalledWith('Memory monitor stopped');
    });

    test('既に実行中の場合は警告を出す', () => {
      memoryMonitor = new MemoryMonitor(logger, undefined, 100000);
      memoryMonitor.start();
      memoryMonitor.start(); // 2回目

      expect(logger.warn).toHaveBeenCalledWith('Memory monitor is already running');
    });

    test('停止していない状態で停止を呼んでもエラーにならない', () => {
      memoryMonitor = new MemoryMonitor(logger);
      expect(() => memoryMonitor.stop()).not.toThrow();
    });
  });

  describe('メモリ統計の取得', () => {
    test('現在のメモリ統計を取得できる', () => {
      memoryMonitor = new MemoryMonitor(logger);
      const stats = memoryMonitor.getMemoryStats();

      expect(stats).toBeDefined();
      expect(stats.rss).toBeGreaterThan(0);
      expect(stats.heapTotal).toBeGreaterThan(0);
      expect(stats.heapUsed).toBeGreaterThan(0);
      expect(stats.external).toBeGreaterThanOrEqual(0);
      expect(stats.arrayBuffers).toBeGreaterThanOrEqual(0);
    });

    test('メモリ統計の値が妥当な範囲内', () => {
      memoryMonitor = new MemoryMonitor(logger);
      const stats = memoryMonitor.getMemoryStats();

      // ヒープ使用量はヒープ総量以下
      expect(stats.heapUsed).toBeLessThanOrEqual(stats.heapTotal);
      
      // RSSはヒープ総量以上（通常）
      expect(stats.rss).toBeGreaterThanOrEqual(stats.heapTotal);
    });
  });

  describe('閾値の更新', () => {
    test('閾値を更新できる', () => {
      memoryMonitor = new MemoryMonitor(logger);
      
      const newThresholds: Partial<MemoryThresholds> = {
        rssWarning: 500 * 1024 * 1024,
      };

      memoryMonitor.updateThresholds(newThresholds);

      expect(logger.info).toHaveBeenCalledWith(
        'Memory thresholds updated',
        expect.any(Object)
      );
    });

    test('部分的な閾値更新が可能', () => {
      memoryMonitor = new MemoryMonitor(logger);
      
      // RSS警告閾値のみ更新
      memoryMonitor.updateThresholds({
        rssWarning: 450 * 1024 * 1024,
      });

      expect(logger.info).toHaveBeenCalled();
    });
  });

  describe('メモリチェック', () => {
    test('定期的にメモリチェックが実行される', (done) => {
      memoryMonitor = new MemoryMonitor(logger, undefined, 100); // 100ms間隔
      memoryMonitor.start();

      setTimeout(() => {
        // デバッグログが複数回呼ばれていることを確認
        expect(logger.debug).toHaveBeenCalledWith(
          'Memory usage check',
          expect.any(Object)
        );
        memoryMonitor.stop();
        done();
      }, 350); // 3回以上チェックされる時間
    }, 500);

    test('低メモリ閾値では警告が出ない', () => {
      // 非常に高い閾値を設定（警告が出ないようにする）
      const highThresholds: Partial<MemoryThresholds> = {
        rssWarning: 10 * 1024 * 1024 * 1024,    // 10GB
        rssCritical: 20 * 1024 * 1024 * 1024,   // 20GB
        heapWarning: 10 * 1024 * 1024 * 1024,   // 10GB
        heapCritical: 20 * 1024 * 1024 * 1024,  // 20GB
      };

      memoryMonitor = new MemoryMonitor(logger, highThresholds, 100000);
      memoryMonitor.start();

      // 警告やエラーログが呼ばれていないことを確認
      expect(logger.warn).not.toHaveBeenCalledWith(
        expect.stringContaining('WARNING: Memory usage'),
        expect.any(Object)
      );
      expect(logger.error).not.toHaveBeenCalledWith(
        expect.stringContaining('CRITICAL: Memory usage'),
        expect.any(Error),
        expect.any(Object)
      );
    });
  });

  describe('実行状態の確認', () => {
    test('開始前はfalseを返す', () => {
      memoryMonitor = new MemoryMonitor(logger);
      expect(memoryMonitor.isRunning()).toBe(false);
    });

    test('開始後はtrueを返す', () => {
      memoryMonitor = new MemoryMonitor(logger, undefined, 100000);
      memoryMonitor.start();
      expect(memoryMonitor.isRunning()).toBe(true);
    });

    test('停止後はfalseを返す', () => {
      memoryMonitor = new MemoryMonitor(logger, undefined, 100000);
      memoryMonitor.start();
      memoryMonitor.stop();
      expect(memoryMonitor.isRunning()).toBe(false);
    });
  });

  describe('メモリリーク検出', () => {
    test('メモリ統計が時間経過で大きく変化しない', () => {
      memoryMonitor = new MemoryMonitor(logger);
      
      const stats1 = memoryMonitor.getMemoryStats();
      
      // 少し待機
      const start = Date.now();
      while (Date.now() - start < 100) {
        // 待機
      }
      
      const stats2 = memoryMonitor.getMemoryStats();

      // メモリ使用量が極端に増加していないことを確認
      // （通常の変動範囲内）
      const rssDiff = Math.abs(stats2.rss - stats1.rss);
      expect(rssDiff).toBeLessThan(50 * 1024 * 1024); // 50MB以内の変動
    });
  });
});
