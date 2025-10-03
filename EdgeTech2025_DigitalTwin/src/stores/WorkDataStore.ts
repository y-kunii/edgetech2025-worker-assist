import { EventEmitter } from 'events';
import { 
  WorkData, 
  WorkerStatus, 
  RobotStatus, 
  ThresholdSettings, 
  WorkStatistics, 
  WorkHistory, 
  EfficiencyMetrics, 
  Notification,
  ConnectionQuality
} from '../types';
import { validateSensorData, convertSensorDataToWorkData, sanitizeData } from '../utils/validation';

export interface WorkDataStoreState {
  currentWorkData: WorkData | null;
  thresholdSettings: ThresholdSettings;
  workHistory: WorkHistory[];
  statistics: WorkStatistics;
  efficiencyMetrics: EfficiencyMetrics;
  notifications: Notification[];
  connectionQuality: ConnectionQuality | null;
  isConnected: boolean;
}

export class WorkDataStore extends EventEmitter {
  private state: WorkDataStoreState;
  private readonly maxHistorySize = 1000;
  private readonly maxNotifications = 50;

  constructor() {
    super();
    
    // ローカルストレージから閾値設定を読み込み
    const savedThresholdSettings = this.loadThresholdSettings();
    
    this.state = {
      currentWorkData: null,
      thresholdSettings: savedThresholdSettings,
      workHistory: [],
      statistics: {
        totalWorkTime: 0,
        completedTasks: 0,
        averageEfficiency: 0,
        errorCount: 0
      },
      efficiencyMetrics: {
        current: 0,
        target: 85,
        trend: 'stable',
        lastUpdated: new Date()
      },
      notifications: [],
      connectionQuality: {
        latency: 25,
        dataRate: 2.5,
        stability: 'good',
        lastUpdated: new Date()
      },
      isConnected: true
    };

    // Start simulating connection quality updates
    this.startConnectionQualitySimulation();
  }

  /**
   * 現在の状態を取得
   */
  public getState(): Readonly<WorkDataStoreState> {
    return { ...this.state };
  }

  /**
   * センサーデータを更新
   */
  public updateSensorData(rawData: any): boolean {
    try {
      // データのサニタイズ
      const sanitizedData = sanitizeData(rawData);
      
      // データの検証
      const validationResult = validateSensorData(sanitizedData);
      if (!validationResult.isValid) {
        console.error('Invalid sensor data:', validationResult.errors);
        this.incrementErrorCount();
        this.emit('validation_error', validationResult.errors);
        return false;
      }

      // WorkDataに変換
      const workData = convertSensorDataToWorkData(validationResult.data!);
      
      // 前回のデータと比較して変更があるかチェック
      const hasChanged = this.hasWorkDataChanged(workData);
      
      // 作業履歴を更新
      if (hasChanged && this.state.currentWorkData) {
        this.addToWorkHistory(this.state.currentWorkData);
      }
      
      // 現在のデータを更新
      this.state.currentWorkData = workData;
      
      // 統計情報を更新
      this.updateStatistics();
      
      // 効率指標を更新
      this.updateEfficiencyMetrics();
      
      // 閾値チェック
      this.checkThresholds(workData);
      
      // 変更を通知
      this.emit('work_data_updated', workData);
      
      if (hasChanged) {
        this.emit('work_status_changed', workData.workerStatus);
      }
      
      return true;
    } catch (error) {
      console.error('Error updating sensor data:', error);
      this.incrementErrorCount();
      this.emit('error', error);
      return false;
    }
  }

  /**
   * 閾値設定を更新
   */
  public updateThresholdSettings(settings: ThresholdSettings): boolean {
    try {
      this.state.thresholdSettings = { ...settings };
      
      // ローカルストレージに保存
      this.saveThresholdSettings(settings);
      
      this.emit('threshold_settings_updated', settings);
      
      // 現在のデータで閾値チェックを再実行
      if (this.state.currentWorkData) {
        this.checkThresholds(this.state.currentWorkData);
      }
      
      return true;
    } catch (error) {
      console.error('Error updating threshold settings:', error);
      this.emit('error', error);
      return false;
    }
  }

  /**
   * 接続状態を更新
   */
  public updateConnectionStatus(isConnected: boolean): void {
    if (this.state.isConnected !== isConnected) {
      this.state.isConnected = isConnected;
      this.emit('connection_status_changed', isConnected);
      
      if (!isConnected) {
        this.addNotification({
          type: 'warning',
          title: '接続が切断されました',
          message: 'WebSocket接続が切断されました。再接続を試行中...'
        });
      } else {
        this.addNotification({
          type: 'success',
          title: '接続が復旧しました',
          message: 'WebSocket接続が正常に復旧しました。'
        });
      }
    }
  }

  /**
   * 接続品質を更新
   */
  public updateConnectionQuality(quality: ConnectionQuality): void {
    this.state.connectionQuality = { ...quality };
    this.emit('connection_quality_updated', quality);
  }

  /**
   * 通知を追加
   */
  public addNotification(notification: Omit<Notification, 'id' | 'timestamp' | 'read'>): void {
    const newNotification: Notification = {
      id: this.generateId(),
      timestamp: new Date(),
      read: false,
      ...notification
    };
    
    this.state.notifications.unshift(newNotification);
    
    // 最大数を超えた場合は古いものを削除
    if (this.state.notifications.length > this.maxNotifications) {
      this.state.notifications = this.state.notifications.slice(0, this.maxNotifications);
    }
    
    this.emit('notification_added', newNotification);
  }

  /**
   * 通知を既読にする
   */
  public markNotificationAsRead(notificationId: string): boolean {
    const notification = this.state.notifications.find(n => n.id === notificationId);
    if (notification && !notification.read) {
      notification.read = true;
      this.emit('notification_read', notification);
      return true;
    }
    return false;
  }

  /**
   * 全通知を既読にする
   */
  public markAllNotificationsAsRead(): void {
    let hasUnread = false;
    this.state.notifications.forEach(notification => {
      if (!notification.read) {
        notification.read = true;
        hasUnread = true;
      }
    });
    
    if (hasUnread) {
      this.emit('all_notifications_read');
    }
  }

  /**
   * 統計情報をリセット
   */
  public resetStatistics(): void {
    this.state.statistics = {
      totalWorkTime: 0,
      completedTasks: 0,
      averageEfficiency: 0,
      errorCount: 0
    };
    
    this.state.workHistory = [];
    this.emit('statistics_reset');
  }

  /**
   * WorkDataが変更されたかチェック
   */
  private hasWorkDataChanged(newData: WorkData): boolean {
    if (!this.state.currentWorkData) {
      return true;
    }
    
    const current = this.state.currentWorkData;
    return (
      current.workerStatus !== newData.workerStatus ||
      current.robotStatus.state !== newData.robotStatus.state ||
      current.robotStatus.grip !== newData.robotStatus.grip ||
      current.screwCount !== newData.screwCount ||
      current.boltCount !== newData.boltCount
    );
  }

  /**
   * 作業履歴に追加
   */
  private addToWorkHistory(workData: WorkData): void {
    const now = new Date();
    const lastHistory = this.state.workHistory[this.state.workHistory.length - 1];
    
    let duration = 0;
    if (lastHistory) {
      duration = now.getTime() - lastHistory.timestamp.getTime();
    }
    
    const historyEntry: WorkHistory = {
      id: this.generateId(),
      timestamp: workData.timestamp,
      workerStatus: workData.workerStatus,
      duration,
      efficiency: this.calculateCurrentEfficiency()
    };
    
    this.state.workHistory.push(historyEntry);
    
    // 最大履歴数を超えた場合は古いものを削除
    if (this.state.workHistory.length > this.maxHistorySize) {
      this.state.workHistory = this.state.workHistory.slice(-this.maxHistorySize);
    }
    
    this.emit('work_history_updated', historyEntry);
  }

  /**
   * 統計情報を更新
   */
  private updateStatistics(): void {
    const history = this.state.workHistory;
    
    // 総作業時間を計算（分単位）
    const totalWorkTime = history.reduce((total, entry) => total + entry.duration, 0) / (1000 * 60);
    
    // 完了タスク数を計算
    const completedTasks = history.filter(entry => 
      entry.workerStatus === 'screw_tightening' || entry.workerStatus === 'bolt_tightening'
    ).length;
    
    // 平均効率を計算
    const efficiencySum = history.reduce((sum, entry) => sum + entry.efficiency, 0);
    const averageEfficiency = history.length > 0 ? efficiencySum / history.length : 0;
    
    this.state.statistics = {
      totalWorkTime,
      completedTasks,
      averageEfficiency,
      errorCount: this.state.statistics.errorCount
    };
    
    this.emit('statistics_updated', this.state.statistics);
  }

  /**
   * 効率指標を更新
   */
  private updateEfficiencyMetrics(): void {
    const currentEfficiency = this.calculateCurrentEfficiency();
    const previousEfficiency = this.state.efficiencyMetrics.current;
    
    let trend: 'up' | 'down' | 'stable' = 'stable';
    const threshold = 2; // 2%以上の変化でトレンドを判定
    
    if (currentEfficiency > previousEfficiency + threshold) {
      trend = 'up';
    } else if (currentEfficiency < previousEfficiency - threshold) {
      trend = 'down';
    }
    
    this.state.efficiencyMetrics = {
      current: currentEfficiency,
      target: this.state.efficiencyMetrics.target,
      trend,
      lastUpdated: new Date()
    };
    
    this.emit('efficiency_metrics_updated', this.state.efficiencyMetrics);
  }

  /**
   * 現在の効率を計算
   */
  private calculateCurrentEfficiency(): number {
    // 簡単な効率計算ロジック（実際の実装では更に複雑になる可能性があります）
    if (!this.state.currentWorkData) {
      return 0;
    }
    
    const { screwCount, boltCount } = this.state.currentWorkData;
    const { screwThreshold, boltThreshold } = this.state.thresholdSettings;
    
    const screwProgress = Math.min(screwCount / screwThreshold, 1);
    const boltProgress = Math.min(boltCount / boltThreshold, 1);
    
    return Math.round((screwProgress + boltProgress) / 2 * 100);
  }

  /**
   * 閾値チェック
   */
  private checkThresholds(workData: WorkData): void {
    const { screwCount, boltCount } = workData;
    const { screwThreshold, boltThreshold } = this.state.thresholdSettings;
    
    // ネジ締め閾値チェック
    if (screwCount >= screwThreshold) {
      this.emit('screw_threshold_reached', { count: screwCount, threshold: screwThreshold });
      this.addNotification({
        type: 'success',
        title: 'ネジ締め完了',
        message: `ネジ締め作業が目標回数（${screwThreshold}回）に達しました。`
      });
    }
    
    // ボルト締め閾値チェック
    if (boltCount >= boltThreshold) {
      this.emit('bolt_threshold_reached', { count: boltCount, threshold: boltThreshold });
      this.addNotification({
        type: 'success',
        title: 'ボルト締め完了',
        message: `ボルト締め作業が目標回数（${boltThreshold}回）に達しました。`
      });
    }
  }

  /**
   * エラーカウントを増加
   */
  private incrementErrorCount(): void {
    this.state.statistics.errorCount++;
    this.emit('error_count_updated', this.state.statistics.errorCount);
  }

  /**
   * 閾値設定をローカルストレージに保存
   */
  private saveThresholdSettings(settings: ThresholdSettings): void {
    try {
      if (typeof window !== 'undefined' && window.localStorage) {
        localStorage.setItem('manufacturing-twin-threshold-settings', JSON.stringify(settings));
      }
    } catch (error) {
      console.warn('Failed to save threshold settings to localStorage:', error);
    }
  }

  /**
   * ローカルストレージから閾値設定を読み込み
   */
  private loadThresholdSettings(): ThresholdSettings {
    const defaultSettings: ThresholdSettings = {
      screwThreshold: 5,
      boltThreshold: 3
    };

    try {
      if (typeof window !== 'undefined' && window.localStorage) {
        const saved = localStorage.getItem('manufacturing-twin-threshold-settings');
        if (saved) {
          const parsed = JSON.parse(saved);
          // 設定値の妥当性をチェック
          if (
            typeof parsed.screwThreshold === 'number' &&
            typeof parsed.boltThreshold === 'number' &&
            parsed.screwThreshold >= 1 && parsed.screwThreshold <= 20 &&
            parsed.boltThreshold >= 1 && parsed.boltThreshold <= 20
          ) {
            return parsed;
          }
        }
      }
    } catch (error) {
      console.warn('Failed to load threshold settings from localStorage:', error);
    }

    return defaultSettings;
  }

  /**
   * ユニークIDを生成
   */
  private generateId(): string {
    return `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * 接続品質のシミュレーションを開始
   */
  private startConnectionQualitySimulation(): void {
    setInterval(() => {
      if (this.state.isConnected && this.state.connectionQuality) {
        // Simulate realistic connection quality variations
        const baseLatency = 25;
        const latencyVariation = Math.random() * 20 - 10; // ±10ms variation
        const newLatency = Math.max(5, baseLatency + latencyVariation);

        const baseDataRate = 2.5;
        const dataRateVariation = Math.random() * 1 - 0.5; // ±0.5 variation
        const newDataRate = Math.max(0.1, baseDataRate + dataRateVariation);

        // Determine stability based on latency and data rate
        let stability: 'excellent' | 'good' | 'fair' | 'poor';
        if (newLatency < 20 && newDataRate > 2.0) {
          stability = 'excellent';
        } else if (newLatency < 50 && newDataRate > 1.5) {
          stability = 'good';
        } else if (newLatency < 100 && newDataRate > 1.0) {
          stability = 'fair';
        } else {
          stability = 'poor';
        }

        const updatedQuality: ConnectionQuality = {
          latency: Math.round(newLatency),
          dataRate: Math.round(newDataRate * 10) / 10,
          stability,
          lastUpdated: new Date()
        };

        this.updateConnectionQuality(updatedQuality);
      }
    }, 2000); // Update every 2 seconds
  }

  /**
   * ストアを破棄
   */
  public destroy(): void {
    this.removeAllListeners();
  }
}