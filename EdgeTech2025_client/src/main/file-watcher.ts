import * as chokidar from 'chokidar';
import * as path from 'path';
import * as fs from 'fs';
import { EventEmitter } from 'events';
import { APP_CONFIG } from '../shared/constants';

export interface FileWatcherConfig {
  watchPath: string;
  supportedFormats?: readonly string[];
  debounceMs?: number;
}

export interface ImageUpdateEvent {
  filePath: string;
  fileName: string;
  timestamp: Date;
  size: number;
}

export class FileWatcher extends EventEmitter {
  private watcher: chokidar.FSWatcher | null = null;
  private config: FileWatcherConfig;
  private debounceTimer: NodeJS.Timeout | null = null;
  private lastImagePath: string | null = null;
  private isWatching: boolean = false;

  constructor(config: FileWatcherConfig) {
    super();
    this.config = {
      supportedFormats: APP_CONFIG.SUPPORTED_IMAGE_FORMATS,
      debounceMs: APP_CONFIG.IMAGE_WATCH_DEBOUNCE,
      ...config
    };
  }

  /**
   * ファイル監視を開始する
   */
  public async startWatching(): Promise<boolean> {
    try {
      if (this.isWatching) {
        console.log('File watcher is already running');
        return true;
      }

      // 監視対象ディレクトリの存在確認
      if (!fs.existsSync(this.config.watchPath)) {
        console.warn(`Watch directory does not exist: ${this.config.watchPath}`);
        // ディレクトリを作成
        fs.mkdirSync(this.config.watchPath, { recursive: true });
        console.log(`Created watch directory: ${this.config.watchPath}`);
      }

      // 初期画像の検索
      await this.findLatestImage();

      // chokidarでファイル監視を開始
      this.watcher = chokidar.watch(this.config.watchPath, {
        ignored: /^\./, // 隠しファイルを無視
        persistent: true,
        ignoreInitial: false,
        depth: 1 // サブディレクトリは1階層まで
      });

      this.setupWatcherEvents();
      this.isWatching = true;

      console.log(`File watcher started for: ${this.config.watchPath}`);
      this.emit('watcherStarted', { watchPath: this.config.watchPath });
      
      return true;
    } catch (error) {
      console.error('Failed to start file watcher:', error);
      this.emit('watcherError', error);
      return false;
    }
  }

  /**
   * ファイル監視を停止する
   */
  public async stopWatching(): Promise<void> {
    try {
      if (this.debounceTimer) {
        clearTimeout(this.debounceTimer);
        this.debounceTimer = null;
      }

      if (this.watcher) {
        await this.watcher.close();
        this.watcher = null;
      }

      this.isWatching = false;
      console.log('File watcher stopped');
      this.emit('watcherStopped');
    } catch (error) {
      console.error('Failed to stop file watcher:', error);
      this.emit('watcherError', error);
    }
  }

  /**
   * 現在の最新画像パスを取得する
   */
  public getLatestImagePath(): string | null {
    return this.lastImagePath;
  }

  /**
   * 監視状態を取得する
   */
  public isActive(): boolean {
    return this.isWatching;
  }

  /**
   * 監視設定を更新する
   */
  public async updateConfig(newConfig: Partial<FileWatcherConfig>): Promise<boolean> {
    const wasWatching = this.isWatching;
    
    if (wasWatching) {
      await this.stopWatching();
    }

    this.config = { ...this.config, ...newConfig };

    if (wasWatching) {
      return await this.startWatching();
    }

    return true;
  }

  /**
   * chokidarイベントハンドラーの設定
   */
  private setupWatcherEvents(): void {
    if (!this.watcher) return;

    this.watcher.on('add', (filePath: string) => {
      this.handleFileChange(filePath, 'added');
    });

    this.watcher.on('change', (filePath: string) => {
      this.handleFileChange(filePath, 'changed');
    });

    this.watcher.on('unlink', (filePath: string) => {
      console.log(`File removed: ${filePath}`);
      // 削除されたファイルが現在の最新画像の場合、新しい最新画像を検索
      if (filePath === this.lastImagePath) {
        this.findLatestImage();
      }
    });

    this.watcher.on('error', (error: Error) => {
      console.error('File watcher error:', error);
      this.emit('watcherError', error);
    });

    this.watcher.on('ready', () => {
      console.log('File watcher is ready');
      this.emit('watcherReady');
    });
  }

  /**
   * ファイル変更の処理（デバウンス付き）
   */
  private handleFileChange(filePath: string, changeType: 'added' | 'changed'): void {
    // サポートされている画像形式かチェック
    if (!this.isSupportedImageFile(filePath)) {
      return;
    }

    console.log(`Image file ${changeType}: ${filePath}`);

    // デバウンス処理
    if (this.debounceTimer) {
      clearTimeout(this.debounceTimer);
    }

    this.debounceTimer = setTimeout(() => {
      this.processImageUpdate(filePath);
    }, this.config.debounceMs);
  }

  /**
   * 画像更新の処理
   */
  private async processImageUpdate(filePath: string): Promise<void> {
    try {
      // ファイルの存在確認
      if (!fs.existsSync(filePath)) {
        console.warn(`Image file no longer exists: ${filePath}`);
        return;
      }

      // ファイル情報の取得
      const stats = fs.statSync(filePath);
      const fileName = path.basename(filePath);

      // 最新画像パスの更新
      this.lastImagePath = filePath;

      const updateEvent: ImageUpdateEvent = {
        filePath,
        fileName,
        timestamp: new Date(),
        size: stats.size
      };

      console.log(`Image updated: ${fileName} (${stats.size} bytes)`);
      this.emit('imageUpdated', updateEvent);

    } catch (error) {
      console.error(`Failed to process image update for ${filePath}:`, error);
      this.emit('imageError', { filePath, error });
    }
  }

  /**
   * サポートされている画像ファイルかチェック
   */
  private isSupportedImageFile(filePath: string): boolean {
    const ext = path.extname(filePath).toLowerCase();
    return this.config.supportedFormats!.includes(ext);
  }

  /**
   * 最新の画像ファイルを検索
   */
  private async findLatestImage(): Promise<void> {
    try {
      if (!fs.existsSync(this.config.watchPath)) {
        return;
      }

      const files = fs.readdirSync(this.config.watchPath);
      let latestFile: string | null = null;
      let latestTime = 0;

      for (const file of files) {
        const filePath = path.join(this.config.watchPath, file);
        
        if (!this.isSupportedImageFile(filePath)) {
          continue;
        }

        try {
          const stats = fs.statSync(filePath);
          if (stats.mtime.getTime() > latestTime) {
            latestTime = stats.mtime.getTime();
            latestFile = filePath;
          }
        } catch (error) {
          console.warn(`Failed to get stats for ${filePath}:`, error);
        }
      }

      if (latestFile) {
        this.lastImagePath = latestFile;
        console.log(`Found latest image: ${latestFile}`);
        
        // 初期画像が見つかった場合はイベントを発火
        const stats = fs.statSync(latestFile);
        const updateEvent: ImageUpdateEvent = {
          filePath: latestFile,
          fileName: path.basename(latestFile),
          timestamp: new Date(),
          size: stats.size
        };
        
        this.emit('imageUpdated', updateEvent);
      } else {
        console.log('No image files found in watch directory');
      }
    } catch (error) {
      console.error('Failed to find latest image:', error);
    }
  }
}