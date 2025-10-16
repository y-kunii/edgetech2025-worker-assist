/**
 * MemoryMonitor
 * メモリ使用量の監視と警告
 */

import { Logger } from './Logger';

export interface MemoryStats {
  rss: number;          // Resident Set Size (物理メモリ使用量)
  heapTotal: number;    // ヒープ総量
  heapUsed: number;     // ヒープ使用量
  external: number;     // C++オブジェクトのメモリ使用量
  arrayBuffers: number; // ArrayBufferのメモリ使用量
}

export interface MemoryThresholds {
  rssWarning: number;      // RSS警告閾値（バイト）
  rssCritical: number;     // RSS危険閾値（バイト）
  heapWarning: number;     // ヒープ警告閾値（バイト）
  heapCritical: number;    // ヒープ危険閾値（バイト）
}

export class MemoryMonitor {
  private logger: Logger;
  private thresholds: MemoryThresholds;
  private checkInterval: number;
  private intervalId: NodeJS.Timeout | null;
  private lastWarningTime: number;
  private warningCooldown: number; // 警告のクールダウン時間（ミリ秒）

  constructor(
    logger: Logger,
    thresholds?: Partial<MemoryThresholds>,
    checkInterval: number = 30000, // デフォルト30秒
    warningCooldown: number = 300000 // デフォルト5分
  ) {
    this.logger = logger;
    this.checkInterval = checkInterval;
    this.intervalId = null;
    this.lastWarningTime = 0;
    this.warningCooldown = warningCooldown;

    // デフォルト閾値（ラズパイ向けに512MB以下を目標）
    this.thresholds = {
      rssWarning: thresholds?.rssWarning || 400 * 1024 * 1024,    // 400MB
      rssCritical: thresholds?.rssCritical || 480 * 1024 * 1024,  // 480MB
      heapWarning: thresholds?.heapWarning || 300 * 1024 * 1024,  // 300MB
      heapCritical: thresholds?.heapCritical || 400 * 1024 * 1024, // 400MB
    };
  }

  /**
   * メモリ監視を開始
   */
  start(): void {
    if (this.intervalId) {
      this.logger.warn('Memory monitor is already running');
      return;
    }

    this.logger.info('Starting memory monitor', {
      checkInterval: this.checkInterval,
      thresholds: {
        rssWarning: this.formatBytes(this.thresholds.rssWarning),
        rssCritical: this.formatBytes(this.thresholds.rssCritical),
        heapWarning: this.formatBytes(this.thresholds.heapWarning),
        heapCritical: this.formatBytes(this.thresholds.heapCritical),
      },
    });

    // 初回チェック
    this.checkMemory();

    // 定期チェック
    this.intervalId = setInterval(() => {
      this.checkMemory();
    }, this.checkInterval);
  }

  /**
   * メモリ監視を停止
   */
  stop(): void {
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
      this.logger.info('Memory monitor stopped');
    }
  }

  /**
   * 現在のメモリ使用状況を取得
   */
  getMemoryStats(): MemoryStats {
    const usage = process.memoryUsage();
    return {
      rss: usage.rss,
      heapTotal: usage.heapTotal,
      heapUsed: usage.heapUsed,
      external: usage.external,
      arrayBuffers: usage.arrayBuffers,
    };
  }

  /**
   * メモリ使用量をチェックして警告
   */
  private checkMemory(): void {
    const stats = this.getMemoryStats();
    const now = Date.now();

    // クールダウン中は警告をスキップ
    const shouldWarn = now - this.lastWarningTime > this.warningCooldown;

    // RSS（物理メモリ）のチェック
    if (stats.rss >= this.thresholds.rssCritical) {
      if (shouldWarn) {
        this.logger.error('CRITICAL: Memory usage exceeded critical threshold', new Error('Memory critical'), {
          rss: this.formatBytes(stats.rss),
          threshold: this.formatBytes(this.thresholds.rssCritical),
          heapUsed: this.formatBytes(stats.heapUsed),
          heapTotal: this.formatBytes(stats.heapTotal),
        });
        this.lastWarningTime = now;
        
        // 危険レベルの場合はガベージコレクションを試行
        this.forceGarbageCollection();
      }
    } else if (stats.rss >= this.thresholds.rssWarning) {
      if (shouldWarn) {
        this.logger.warn('WARNING: Memory usage exceeded warning threshold', {
          rss: this.formatBytes(stats.rss),
          threshold: this.formatBytes(this.thresholds.rssWarning),
          heapUsed: this.formatBytes(stats.heapUsed),
          heapTotal: this.formatBytes(stats.heapTotal),
        });
        this.lastWarningTime = now;
      }
    }

    // ヒープメモリのチェック
    if (stats.heapUsed >= this.thresholds.heapCritical) {
      if (shouldWarn) {
        this.logger.error('CRITICAL: Heap usage exceeded critical threshold', new Error('Heap critical'), {
          heapUsed: this.formatBytes(stats.heapUsed),
          threshold: this.formatBytes(this.thresholds.heapCritical),
          heapTotal: this.formatBytes(stats.heapTotal),
        });
        this.lastWarningTime = now;
        
        // 危険レベルの場合はガベージコレクションを試行
        this.forceGarbageCollection();
      }
    } else if (stats.heapUsed >= this.thresholds.heapWarning) {
      if (shouldWarn) {
        this.logger.warn('WARNING: Heap usage exceeded warning threshold', {
          heapUsed: this.formatBytes(stats.heapUsed),
          threshold: this.formatBytes(this.thresholds.heapWarning),
          heapTotal: this.formatBytes(stats.heapTotal),
        });
        this.lastWarningTime = now;
      }
    }

    // デバッグログ（定期的なメモリ状況）
    this.logger.debug('Memory usage check', {
      rss: this.formatBytes(stats.rss),
      heapUsed: this.formatBytes(stats.heapUsed),
      heapTotal: this.formatBytes(stats.heapTotal),
      external: this.formatBytes(stats.external),
    });
  }

  /**
   * ガベージコレクションを強制実行
   * 注: --expose-gc フラグが必要
   */
  private forceGarbageCollection(): void {
    if (global.gc) {
      this.logger.info('Forcing garbage collection');
      try {
        global.gc();
        
        // GC後のメモリ状況をログ
        const statsAfterGC = this.getMemoryStats();
        this.logger.info('Garbage collection completed', {
          rss: this.formatBytes(statsAfterGC.rss),
          heapUsed: this.formatBytes(statsAfterGC.heapUsed),
          heapTotal: this.formatBytes(statsAfterGC.heapTotal),
        });
      } catch (error) {
        this.logger.error('Failed to force garbage collection', error as Error);
      }
    } else {
      this.logger.warn('Garbage collection not available (run with --expose-gc flag)');
    }
  }

  /**
   * バイト数を人間が読みやすい形式に変換
   */
  private formatBytes(bytes: number): string {
    if (bytes === 0) return '0 B';
    
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    
    return `${(bytes / Math.pow(k, i)).toFixed(2)} ${sizes[i]}`;
  }

  /**
   * 閾値を更新
   */
  updateThresholds(thresholds: Partial<MemoryThresholds>): void {
    this.thresholds = {
      ...this.thresholds,
      ...thresholds,
    };
    
    this.logger.info('Memory thresholds updated', {
      rssWarning: this.formatBytes(this.thresholds.rssWarning),
      rssCritical: this.formatBytes(this.thresholds.rssCritical),
      heapWarning: this.formatBytes(this.thresholds.heapWarning),
      heapCritical: this.formatBytes(this.thresholds.heapCritical),
    });
  }

  /**
   * 監視が実行中かどうかを取得
   */
  isRunning(): boolean {
    return this.intervalId !== null;
  }
}
