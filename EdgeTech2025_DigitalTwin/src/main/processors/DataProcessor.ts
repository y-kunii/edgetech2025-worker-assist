import { WorkDataStore } from '../../stores/WorkDataStore';
import { BrowserWindow } from 'electron';
import { RobotCommandManager } from '../services/RobotCommandManager';

export class DataProcessor {
  private workDataStore: WorkDataStore;
  private mainWindow: BrowserWindow | null = null;
  private robotCommandManager: RobotCommandManager | null = null;

  constructor() {
    this.workDataStore = new WorkDataStore();
    this.setupEventHandlers();
  }

  /**
   * メインウィンドウを設定
   */
  public setMainWindow(window: BrowserWindow): void {
    this.mainWindow = window;
  }

  /**
   * ロボットコマンドマネージャーを設定
   */
  public setRobotCommandManager(robotCommandManager: RobotCommandManager): void {
    this.robotCommandManager = robotCommandManager;
  }

  /**
   * センサーデータを処理
   */
  public processSensorData(data: any): boolean {
    console.log('Processing sensor data:', data);
    
    const success = this.workDataStore.updateSensorData(data);
    
    if (success) {
      // 処理成功をレンダラープロセスに通知
      this.sendToRenderer('sensor-data-processed', {
        success: true,
        data: this.workDataStore.getState().currentWorkData
      });
    } else {
      // 処理失敗をレンダラープロセスに通知
      this.sendToRenderer('sensor-data-error', {
        success: false,
        error: 'Failed to process sensor data'
      });
    }
    
    return success;
  }

  /**
   * 閾値設定を更新
   */
  public updateThresholdSettings(settings: any): boolean {
    const success = this.workDataStore.updateThresholdSettings(settings);
    
    if (success) {
      this.sendToRenderer('threshold-settings-updated', settings);
    }
    
    return success;
  }

  /**
   * 接続状態を更新
   */
  public updateConnectionStatus(isConnected: boolean): void {
    this.workDataStore.updateConnectionStatus(isConnected);
  }

  /**
   * 接続品質を更新
   */
  public updateConnectionQuality(latency: number): void {
    const quality = this.calculateConnectionQuality(latency);
    this.workDataStore.updateConnectionQuality(quality);
  }

  /**
   * 現在の状態を取得
   */
  public getCurrentState() {
    return this.workDataStore.getState();
  }

  /**
   * 通知を追加
   */
  public addNotification(notification: any): void {
    this.workDataStore.addNotification(notification);
  }

  /**
   * 統計情報をリセット
   */
  public resetStatistics(): void {
    this.workDataStore.resetStatistics();
  }

  /**
   * イベントハンドラーを設定
   */
  private setupEventHandlers(): void {
    // 作業データ更新イベント
    this.workDataStore.on('work_data_updated', (workData) => {
      this.sendToRenderer('work-data-updated', workData);
    });

    // 作業状態変更イベント
    this.workDataStore.on('work_status_changed', (status) => {
      this.sendToRenderer('work-status-changed', { status });
    });

    // 閾値達成イベント
    this.workDataStore.on('screw_threshold_reached', (data) => {
      console.log('Screw threshold reached:', data);
      this.sendToRenderer('screw-threshold-reached', data);
      
      // ロボットコマンドマネージャーが設定されている場合、自動的に指示を送信
      if (this.robotCommandManager) {
        this.robotCommandManager.sendThresholdCommand('screw_threshold_reached', data);
      } else {
        // フォールバック: レンダラープロセスに手動送信をトリガー
        this.sendToRenderer('trigger-robot-command', {
          command: 'tool_handover',
          reason: 'screw_threshold_reached',
          data
        });
      }
    });

    this.workDataStore.on('bolt_threshold_reached', (data) => {
      console.log('Bolt threshold reached:', data);
      this.sendToRenderer('bolt-threshold-reached', data);
      
      // ロボットコマンドマネージャーが設定されている場合、自動的に指示を送信
      if (this.robotCommandManager) {
        this.robotCommandManager.sendThresholdCommand('bolt_threshold_reached', data);
      } else {
        // フォールバック: レンダラープロセスに手動送信をトリガー
        this.sendToRenderer('trigger-robot-command', {
          command: 'next_task',
          reason: 'bolt_threshold_reached',
          data
        });
      }
    });

    // 統計情報更新イベント
    this.workDataStore.on('statistics_updated', (statistics) => {
      this.sendToRenderer('statistics-updated', statistics);
    });

    // 効率指標更新イベント
    this.workDataStore.on('efficiency_metrics_updated', (metrics) => {
      this.sendToRenderer('efficiency-metrics-updated', metrics);
    });

    // 通知追加イベント
    this.workDataStore.on('notification_added', (notification) => {
      this.sendToRenderer('notification-added', notification);
    });

    // 作業履歴更新イベント
    this.workDataStore.on('work_history_updated', (historyEntry) => {
      this.sendToRenderer('work-history-updated', historyEntry);
    });

    // 接続状態変更イベント
    this.workDataStore.on('connection_status_changed', (isConnected) => {
      this.sendToRenderer('connection-status-changed', { isConnected });
    });

    // 接続品質更新イベント
    this.workDataStore.on('connection_quality_updated', (quality) => {
      this.sendToRenderer('connection-quality-updated', quality);
    });

    // バリデーションエラーイベント
    this.workDataStore.on('validation_error', (errors) => {
      console.error('Data validation errors:', errors);
      this.sendToRenderer('validation-error', { errors });
    });

    // エラーイベント
    this.workDataStore.on('error', (error) => {
      console.error('WorkDataStore error:', error);
      this.sendToRenderer('data-store-error', {
        message: error.message,
        stack: error.stack
      });
    });
  }

  /**
   * 接続品質を計算
   */
  private calculateConnectionQuality(latency: number) {
    let stability: 'excellent' | 'good' | 'fair' | 'poor';
    
    if (latency < 50) {
      stability = 'excellent';
    } else if (latency < 100) {
      stability = 'good';
    } else if (latency < 200) {
      stability = 'fair';
    } else {
      stability = 'poor';
    }
    
    return {
      latency,
      dataRate: 0, // WebSocketサービスから取得する必要がある
      stability,
      lastUpdated: new Date()
    };
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
   * プロセッサを破棄
   */
  public destroy(): void {
    this.workDataStore.destroy();
    this.mainWindow = null;
  }
}