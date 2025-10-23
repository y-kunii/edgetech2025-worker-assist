import { app, BrowserWindow, ipcMain } from 'electron';
import * as path from 'path';
import { WebSocketClient, WebSocketClientConfig, ConnectionState } from './websocket-client';
import { StatusData, CommandData, AppSettings } from '../shared/types';
import { DataProcessor } from './data-processor';
import { closeDatabaseService } from './database-service';
import { FileWatcher, FileWatcherConfig, ImageUpdateEvent } from './file-watcher';
import { SettingsService } from './settings-service';
import { InitializationService } from './initialization-service';

class DigitalTwinDashboard {
  private mainWindow: BrowserWindow | null = null;
  private wsClient: WebSocketClient | null = null;
  private fileWatcher: FileWatcher | null = null;
  private settingsService: SettingsService;
  private initializationService: InitializationService;
  private isShuttingDown: boolean = false;

  constructor() {
    // サービスを初期化
    this.settingsService = SettingsService.getInstance();
    this.initializationService = InitializationService.getInstance();
    this.initializeApp();
  }

  private initializeApp(): void {
    // 単一インスタンスの確保
    const gotTheLock = app.requestSingleInstanceLock();

    if (!gotTheLock) {
      app.quit();
      return;
    }

    app.on('second-instance', () => {
      // 既にアプリが起動している場合、既存のウィンドウにフォーカス
      if (this.mainWindow) {
        if (this.mainWindow.isMinimized()) this.mainWindow.restore();
        this.mainWindow.focus();
      }
    });

    // アプリケーションの準備完了時
    app.whenReady().then(async () => {
      try {
        // アプリケーションの初期化処理
        const initResult = await this.initializationService.initialize();
        
        if (!initResult.success) {
          console.error('Application initialization failed:', initResult.errors);
          // 初期化に失敗した場合でも基本機能は動作させる
          if (initResult.errors.some(error => error.includes('Critical'))) {
            app.quit();
            return;
          }
        }

        if (initResult.warnings.length > 0) {
          console.warn('Initialization warnings:', initResult.warnings);
        }

        this.createMainWindow();
        
        // macOSでアプリがアクティブになった時の処理
        app.on('activate', () => {
          if (BrowserWindow.getAllWindows().length === 0 && !this.mainWindow) {
            this.createMainWindow();
          }
        });

        // WebSocketとファイル監視の初期化
        this.initializeWebSocket();
        this.initializeFileWatcher();
        
      } catch (error) {
        console.error('Critical error during app initialization:', error);
        app.quit();
      }
    });

    // 全てのウィンドウが閉じられた時の処理
    app.on('window-all-closed', () => {
      if (process.platform !== 'darwin') {
        this.performShutdown();
      }
    });

    // アプリケーション終了前の処理
    app.on('before-quit', (event) => {
      if (!this.isShuttingDown) {
        event.preventDefault();
        this.performShutdown();
      }
    });

    // 予期しない終了の処理
    process.on('SIGINT', () => {
      console.log('Received SIGINT, shutting down gracefully...');
      this.performShutdown();
    });

    process.on('SIGTERM', () => {
      console.log('Received SIGTERM, shutting down gracefully...');
      this.performShutdown();
    });

    this.setupIpcHandlers();
  }

  private createMainWindow(): void {
    // メインウィンドウの作成
    this.mainWindow = new BrowserWindow({
      width: 1400,
      height: 900,
      minWidth: 1200,
      minHeight: 800,
      webPreferences: {
        nodeIntegration: false,
        contextIsolation: true,
        preload: path.join(__dirname, 'preload.js')
      },
      title: 'Digital Twin Dashboard',
      icon: path.join(__dirname, '../../assets/icon.png'), // アイコンファイルは後で追加
      show: false // 準備完了まで非表示
    });

    // 開発環境とプロダクション環境での読み込み先の切り替え
    if (process.env.NODE_ENV === 'development') {
      this.mainWindow.loadURL('http://localhost:3000');
      this.mainWindow.webContents.openDevTools();
    } else {
      this.mainWindow.loadFile(path.join(__dirname, '../../renderer/index.html'));
    }

    // ウィンドウの準備完了時に表示
    this.mainWindow.once('ready-to-show', () => {
      this.mainWindow?.show();
      
      if (process.env.NODE_ENV === 'development') {
        this.mainWindow?.webContents.openDevTools();
      }
    });

    // ウィンドウが閉じられた時の処理
    this.mainWindow.on('closed', () => {
      this.mainWindow = null;
    });
  }

  private setupIpcHandlers(): void {
    // レンダラープロセスとの通信ハンドラーを設定
    ipcMain.handle('app:getVersion', () => {
      return app.getVersion();
    });

    ipcMain.handle('app:getName', () => {
      return app.getName();
    });

    // アプリケーション設定の取得・保存
    ipcMain.handle('settings:get', () => {
      return this.settingsService.getSettings();
    });

    ipcMain.handle('settings:save', async (_event, settings: Partial<AppSettings>) => {
      return await this.settingsService.saveSettings(settings);
    });

    ipcMain.handle('settings:reset', async () => {
      return await this.settingsService.resetSettings();
    });

    ipcMain.handle('settings:validate', (_event, settings: AppSettings) => {
      return this.settingsService.validateSettings(settings);
    });

    ipcMain.handle('settings:getSetting', (_event, key: keyof AppSettings) => {
      return this.settingsService.getSetting(key);
    });

    ipcMain.handle('settings:updateSetting', async (_event, key: keyof AppSettings, value: any) => {
      return await this.settingsService.updateSetting(key, value);
    });

    ipcMain.handle('settings:getDefaults', () => {
      return this.settingsService.getDefaultSettings();
    });

    ipcMain.handle('settings:fileExists', () => {
      return this.settingsService.settingsFileExists();
    });

    ipcMain.handle('settings:getPath', () => {
      return this.settingsService.getSettingsPath();
    });

    // 初期化関連のIPCハンドラー
    ipcMain.handle('app:isInitialized', () => {
      return this.initializationService.isAppInitialized();
    });

    ipcMain.handle('app:performHealthCheck', async () => {
      return await this.initializationService.performHealthCheck();
    });

    ipcMain.handle('app:reinitialize', async () => {
      return await this.initializationService.initialize();
    });

    ipcMain.handle('app:shutdown', async () => {
      await this.performShutdown();
      return true;
    });

    // WebSocket関連のIPCハンドラー
    ipcMain.handle('websocket:connect', async (_event, config: WebSocketClientConfig) => {
      try {
        if (this.wsClient) {
          this.wsClient.disconnect();
        }
        
        this.wsClient = new WebSocketClient(config);
        this.setupWebSocketEventHandlers();
        this.wsClient.connect();
        
        return { success: true };
      } catch (error) {
        console.error('Failed to connect WebSocket:', error);
        return { success: false, error: (error as Error).message };
      }
    });

    ipcMain.handle('websocket:disconnect', async () => {
      try {
        if (this.wsClient) {
          this.wsClient.disconnect();
          this.wsClient = null;
        }
        return { success: true };
      } catch (error) {
        console.error('Failed to disconnect WebSocket:', error);
        return { success: false, error: (error as Error).message };
      }
    });

    ipcMain.handle('websocket:sendCommand', async (_event, command: CommandData) => {
      try {
        if (!this.wsClient) {
          throw new Error('WebSocket client is not initialized');
        }
        
        const result = await this.wsClient.sendCommand(command);
        return result;
      } catch (error) {
        console.error('Failed to send command:', error);
        return { 
          success: false, 
          commandId: 'error',
          error: (error as Error).message,
          responseTime: 0
        };
      }
    });

    ipcMain.handle('websocket:createAndSendCommand', async (_event, commandType: 'tool_handover' | 'tool_collection' | 'wait') => {
      try {
        if (!this.wsClient) {
          throw new Error('WebSocket client is not initialized');
        }
        
        const result = await this.wsClient.createAndSendCommand(commandType);
        return result;
      } catch (error) {
        console.error('Failed to create and send command:', error);
        return { 
          success: false, 
          commandId: 'error',
          error: (error as Error).message,
          responseTime: 0
        };
      }
    });

    ipcMain.handle('websocket:getConnectionState', () => {
      if (!this.wsClient) {
        return {
          isConnected: false,
          lastHeartbeat: null,
          reconnectAttempts: 0,
          error: 'WebSocket client not initialized'
        };
      }
      
      return this.wsClient.getConnectionState();
    });

    ipcMain.handle('websocket:getLastStatusData', () => {
      if (!this.wsClient) {
        return null;
      }
      
      return this.wsClient.getLastStatusData();
    });

    // コマンド履歴関連のIPCハンドラー
    ipcMain.handle('command:getHistory', (_event, limit?: number) => {
      if (!this.wsClient) {
        return [];
      }
      
      return this.wsClient.getCommandManager().getCommandHistory(limit);
    });

    ipcMain.handle('command:getStatistics', (_event, timeRange?: { start: Date; end: Date }) => {
      if (!this.wsClient) {
        return {
          totalCommands: 0,
          successfulCommands: 0,
          failedCommands: 0,
          successRate: 0,
          commandCounts: {},
          averageResponseTime: 0,
          timeRange: null
        };
      }
      
      return this.wsClient.getCommandManager().getCommandStatistics(timeRange);
    });

    ipcMain.handle('command:getPendingCount', () => {
      if (!this.wsClient) {
        return 0;
      }
      
      return this.wsClient.getCommandManager().getPendingCommandsCount();
    });

    ipcMain.handle('command:clearHistory', () => {
      if (!this.wsClient) {
        return false;
      }
      
      this.wsClient.getCommandManager().clearHistory();
      return true;
    });

    // データベース関連のIPCハンドラー
    ipcMain.handle('database:getStats', () => {
      return DataProcessor.getDatabaseStats();
    });

    ipcMain.handle('database:validateIntegrity', () => {
      return DataProcessor.validateDatabaseIntegrity();
    });

    ipcMain.handle('database:cleanupOldData', () => {
      return DataProcessor.cleanupOldData();
    });

    // 統計関連のIPCハンドラー
    ipcMain.handle('statistics:getCurrent', () => {
      return DataProcessor.getCurrentStatistics();
    });

    ipcMain.handle('statistics:getHistory', (_event, startDate?: string, endDate?: string, limit?: number) => {
      const dbService = DataProcessor.getDatabaseService();
      if (!dbService) {
        return [];
      }
      return dbService.getStatusHistory(startDate, endDate, limit);
    });

    ipcMain.handle('statistics:getCommandHistory', (_event, limit?: number) => {
      const dbService = DataProcessor.getDatabaseService();
      if (!dbService) {
        return [];
      }
      return dbService.getCommandHistory(limit);
    });

    ipcMain.handle('statistics:getPeriodStatistics', (_event, startDate: string, endDate: string) => {
      const statisticsService = DataProcessor.getStatisticsService();
      if (!statisticsService) {
        return [];
      }
      return statisticsService.getPeriodStatistics(startDate, endDate);
    });

    ipcMain.handle('statistics:generateEfficiencyReport', (_event, days?: number) => {
      const statisticsService = DataProcessor.getStatisticsService();
      if (!statisticsService) {
        return {
          totalTasks: 0,
          averageTasksPerDay: 0,
          mostProductiveDay: null,
          improvementTrend: 'stable'
        };
      }
      return statisticsService.generateEfficiencyReport(days);
    });

    // ファイル監視関連のIPCハンドラー
    ipcMain.handle('filewatch:start', async (_event, watchPath: string) => {
      try {
        if (this.fileWatcher) {
          await this.fileWatcher.stopWatching();
        }

        const config: FileWatcherConfig = {
          watchPath: watchPath || './images'
        };

        this.fileWatcher = new FileWatcher(config);
        this.setupFileWatcherEventHandlers();
        
        const success = await this.fileWatcher.startWatching();
        return success;
      } catch (error) {
        console.error('Failed to start file watcher:', error);
        return false;
      }
    });

    ipcMain.handle('filewatch:stop', async () => {
      try {
        if (this.fileWatcher) {
          await this.fileWatcher.stopWatching();
          this.fileWatcher = null;
        }
        return true;
      } catch (error) {
        console.error('Failed to stop file watcher:', error);
        return false;
      }
    });

    ipcMain.handle('filewatch:getLatestImage', () => {
      if (!this.fileWatcher) {
        return null;
      }
      return this.fileWatcher.getLatestImagePath();
    });

    ipcMain.handle('filewatch:isActive', () => {
      if (!this.fileWatcher) {
        return false;
      }
      return this.fileWatcher.isActive();
    });

    ipcMain.handle('filewatch:updateConfig', async (_event, config: Partial<FileWatcherConfig>) => {
      try {
        if (!this.fileWatcher) {
          return false;
        }
        return await this.fileWatcher.updateConfig(config);
      } catch (error) {
        console.error('Failed to update file watcher config:', error);
        return false;
      }
    });
  }

  private initializeWebSocket(): void {
    // 設定からWebSocketクライアントを初期化
    const settings = this.settingsService.getSettings();
    const config: WebSocketClientConfig = {
      url: settings.wsServerUrl,
      reconnectInterval: 1000,
      maxReconnectAttempts: 10,
      heartbeatInterval: 30000
    };

    this.wsClient = new WebSocketClient(config);
    this.setupWebSocketEventHandlers();

    // 自動接続が有効な場合は接続を開始
    if (settings.autoConnect) {
      this.wsClient.connect();
    }
  }

  private setupWebSocketEventHandlers(): void {
    if (!this.wsClient) return;

    this.wsClient.on('connected', () => {
      console.log('WebSocket connected');
      this.sendToRenderer('websocket:connected');
    });

    this.wsClient.on('disconnected', (info) => {
      console.log('WebSocket disconnected:', info);
      this.sendToRenderer('websocket:disconnected', info);
    });

    this.wsClient.on('statusData', (data: StatusData) => {
      console.log('Received status data:', data);
      this.sendToRenderer('websocket:statusData', data);
    });

    this.wsClient.on('statusDataChanged', (changeData) => {
      console.log('Status data changed:', changeData);
      this.sendToRenderer('websocket:statusDataChanged', changeData);
    });

    this.wsClient.on('workTaskChanged', (taskChangeData) => {
      console.log('Work task changed:', taskChangeData);
      this.sendToRenderer('websocket:workTaskChanged', taskChangeData);
    });

    this.wsClient.on('dataValidationError', (errorData) => {
      console.error('Data validation error:', errorData);
      this.sendToRenderer('websocket:dataValidationError', errorData);
    });

    // コマンド関連のイベント
    this.wsClient.on('commandSent', (data) => {
      console.log('Command sent:', data);
      this.sendToRenderer('command:sent', data);
    });

    this.wsClient.on('commandFailed', (data) => {
      console.log('Command failed:', data);
      this.sendToRenderer('command:failed', data);
    });

    this.wsClient.on('commandError', (data) => {
      console.error('Command error:', data);
      this.sendToRenderer('command:error', data);
    });

    this.wsClient.on('commandTimeout', (data) => {
      console.warn('Command timeout:', data);
      this.sendToRenderer('command:timeout', data);
    });

    this.wsClient.on('commandHistoryUpdated', (entry) => {
      this.sendToRenderer('command:historyUpdated', entry);
    });

    this.wsClient.on('allCommandsCancelled', () => {
      this.sendToRenderer('command:allCancelled');
    });

    this.wsClient.on('commandHistoryCleared', () => {
      this.sendToRenderer('command:historyCleared');
    });

    this.wsClient.on('error', (error: Error) => {
      console.error('WebSocket error:', error);
      this.sendToRenderer('websocket:error', error.message);
    });

    this.wsClient.on('connectionStateChanged', (state: ConnectionState) => {
      console.log('Connection state changed:', state);
      this.sendToRenderer('websocket:connectionStateChanged', state);
    });

    this.wsClient.on('maxReconnectAttemptsReached', () => {
      console.log('Max reconnect attempts reached');
      this.sendToRenderer('websocket:maxReconnectAttemptsReached');
    });
  }

  private initializeFileWatcher(): void {
    // 設定からファイル監視を初期化
    const settings = this.settingsService.getSettings();
    const config: FileWatcherConfig = {
      watchPath: settings.imageWatchPath
    };

    this.fileWatcher = new FileWatcher(config);
    this.setupFileWatcherEventHandlers();
  }

  private setupFileWatcherEventHandlers(): void {
    if (!this.fileWatcher) return;

    this.fileWatcher.on('imageUpdated', (updateEvent: ImageUpdateEvent) => {
      console.log('Image updated:', updateEvent);
      this.sendToRenderer('filewatch:imageUpdate', updateEvent.filePath);
    });

    this.fileWatcher.on('watcherStarted', (data) => {
      console.log('File watcher started:', data);
      this.sendToRenderer('filewatch:started', data);
    });

    this.fileWatcher.on('watcherStopped', () => {
      console.log('File watcher stopped');
      this.sendToRenderer('filewatch:stopped');
    });

    this.fileWatcher.on('watcherError', (error) => {
      console.error('File watcher error:', error);
      this.sendToRenderer('filewatch:error', error.message);
    });

    this.fileWatcher.on('watcherReady', () => {
      console.log('File watcher ready');
      this.sendToRenderer('filewatch:ready');
    });

    this.fileWatcher.on('imageError', (errorData) => {
      console.error('Image processing error:', errorData);
      this.sendToRenderer('filewatch:imageError', errorData);
    });
  }

  private sendToRenderer(channel: string, data?: any): void {
    if (this.mainWindow && !this.mainWindow.isDestroyed()) {
      this.mainWindow.webContents.send(channel, data);
    }
  }

  /**
   * アプリケーションの安全な終了処理
   */
  private async performShutdown(): Promise<void> {
    if (this.isShuttingDown) {
      return;
    }

    this.isShuttingDown = true;
    console.log('Application shutdown initiated...');

    try {
      // 1. WebSocket接続の切断
      if (this.wsClient) {
        console.log('Disconnecting WebSocket...');
        this.wsClient.disconnect();
        this.wsClient = null;
      }
      
      // 2. ファイル監視の停止
      if (this.fileWatcher) {
        console.log('Stopping file watcher...');
        await this.fileWatcher.stopWatching();
        this.fileWatcher = null;
      }
      
      // 3. 初期化サービスによる終了処理
      console.log('Running shutdown procedures...');
      const shutdownResult = await this.initializationService.shutdown();
      
      if (!shutdownResult.success) {
        console.error('Shutdown warnings/errors:', shutdownResult.errors, shutdownResult.warnings);
      }
      
      console.log('Application shutdown completed successfully');
      
      // 4. アプリケーション終了
      app.quit();
      
    } catch (error) {
      console.error('Error during shutdown:', error);
      // エラーが発生してもアプリケーションを終了
      app.quit();
    }
  }

  /**
   * 従来のクリーンアップ処理（後方互換性のため保持）
   */
  private cleanup(): void {
    // performShutdown()に処理を移譲
    this.performShutdown();
  }
}

// アプリケーションのインスタンス化
new DigitalTwinDashboard();