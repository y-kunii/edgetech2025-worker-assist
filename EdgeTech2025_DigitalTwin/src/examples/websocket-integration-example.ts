/**
 * WebSocket統合の使用例
 * 
 * このファイルは、WebSocket通信基盤とデータモデル・バリデーション機能の
 * 統合使用方法を示すサンプルコードです。
 */

import { WebSocketService } from '../main/services/WebSocketService';
import { DataProcessor } from '../main/processors/DataProcessor';
import { WorkDataStore } from '../stores/WorkDataStore';
import { validateSensorData, convertSensorDataToWorkData } from '../utils/validation';
import { SensorData, WorkData, ThresholdSettings } from '../types';

/**
 * WebSocket統合の使用例
 */
export class WebSocketIntegrationExample {
  private webSocketService: WebSocketService;
  private dataProcessor: DataProcessor;
  private workDataStore: WorkDataStore;

  constructor() {
    // WebSocketサービスの初期化
    this.webSocketService = new WebSocketService({
      url: 'http://localhost:3001',
      reconnectionAttempts: 10,
      reconnectionDelay: 1000,
      timeout: 5000
    });

    // データプロセッサの初期化
    this.dataProcessor = new DataProcessor();

    // ワークデータストアの初期化
    this.workDataStore = new WorkDataStore();

    this.setupEventHandlers();
  }

  /**
   * 統合システムを開始
   */
  public async start(): Promise<void> {
    console.log('Starting WebSocket integration example...');

    // WebSocket接続を開始
    this.webSocketService.connect();

    // 閾値設定を更新
    const thresholdSettings: ThresholdSettings = {
      screwThreshold: 5,
      boltThreshold: 3
    };
    this.dataProcessor.updateThresholdSettings(thresholdSettings);

    console.log('WebSocket integration example started successfully');
  }

  /**
   * 統合システムを停止
   */
  public stop(): void {
    console.log('Stopping WebSocket integration example...');
    
    this.webSocketService.disconnect();
    this.webSocketService.destroy();
    this.dataProcessor.destroy();
    this.workDataStore.destroy();

    console.log('WebSocket integration example stopped');
  }

  /**
   * サンプルセンサーデータを処理
   */
  public processSampleData(): void {
    const sampleSensorData: SensorData = {
      type: 'sensor_data',
      timestamp: new Date().toISOString(),
      data: {
        image: 'data:image/jpeg;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAYEBQYFBAYGBQYHBwYIChAKCgkJChQODwwQFxQYGBcUFhYaHSUfGhsjHBYWICwgIyYnKSopGR8tMC0oMCUoKSj/2wBDAQcHBwoIChMKChMoGhYaKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCgoKCj/wAARCAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAv/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/8QAFQEBAQAAAAAAAAAAAAAAAAAAAAX/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIRAxEAPwCdABmX/9k=',
        worker_status: 'screw_tightening',
        robot_status: {
          state: 'waiting',
          grip: 'open'
        },
        screw_count: 3,
        bolt_count: 1,
        work_step: 'screw_tightening'
      }
    };

    console.log('Processing sample sensor data...');
    
    // データの検証
    const validationResult = validateSensorData(sampleSensorData);
    if (!validationResult.isValid) {
      console.error('Validation failed:', validationResult.errors);
      return;
    }

    // WorkDataに変換
    const workData = convertSensorDataToWorkData(validationResult.data!);
    console.log('Converted work data:', workData);

    // データストアを通じて処理
    const success = this.workDataStore.updateSensorData(sampleSensorData);
    if (success) {
      console.log('Sample data processed successfully');
      console.log('Current state:', this.workDataStore.getState());
    } else {
      console.error('Failed to process sample data');
    }
  }

  /**
   * 閾値達成のシミュレーション
   */
  public simulateThresholdReached(): void {
    console.log('Simulating threshold reached scenario...');

    // 閾値を低く設定
    this.dataProcessor.updateThresholdSettings({
      screwThreshold: 2,
      boltThreshold: 1
    });

    // 閾値に達するデータを送信
    const thresholdData: SensorData = {
      type: 'sensor_data',
      timestamp: new Date().toISOString(),
      data: {
        image: '', // 画像データを追加（空文字列でも可）
        worker_status: 'screw_tightening',
        robot_status: {
          state: 'waiting',
          grip: 'open'
        },
        screw_count: 2, // 閾値に達する
        bolt_count: 1,  // 閾値に達する
        work_step: 'screw_tightening'
      }
    };

    this.workDataStore.updateSensorData(thresholdData);
  }

  /**
   * 接続品質のシミュレーション
   */
  public simulateConnectionQuality(): void {
    console.log('Simulating connection quality updates...');

    // 良好な接続品質
    this.dataProcessor.updateConnectionQuality(25); // 25ms latency

    setTimeout(() => {
      // 接続品質の悪化
      this.dataProcessor.updateConnectionQuality(150); // 150ms latency
    }, 2000);

    setTimeout(() => {
      // 接続品質の回復
      this.dataProcessor.updateConnectionQuality(30); // 30ms latency
    }, 4000);
  }

  /**
   * イベントハンドラーを設定
   */
  private setupEventHandlers(): void {
    // WebSocketイベント
    this.webSocketService.getDataProcessor().getCurrentState();

    // データストアイベント
    this.workDataStore.on('work_data_updated', (workData: WorkData) => {
      console.log('Work data updated:', workData);
    });

    this.workDataStore.on('screw_threshold_reached', (data) => {
      console.log('🔩 Screw threshold reached!', data);
      // ロボットに工具受け渡し指示を送信
      this.webSocketService.sendRobotCommand('tool_handover', {
        reason: 'screw_threshold_reached',
        screwCount: data.count
      });
    });

    this.workDataStore.on('bolt_threshold_reached', (data) => {
      console.log('🔧 Bolt threshold reached!', data);
      // ロボットに次のタスク指示を送信
      this.webSocketService.sendRobotCommand('next_task', {
        reason: 'bolt_threshold_reached',
        boltCount: data.count
      });
    });

    this.workDataStore.on('notification_added', (notification) => {
      console.log('📢 New notification:', notification);
    });

    this.workDataStore.on('connection_status_changed', (isConnected) => {
      console.log('🔗 Connection status changed:', isConnected ? 'Connected' : 'Disconnected');
    });

    this.workDataStore.on('validation_error', (errors) => {
      console.error('❌ Validation errors:', errors);
    });

    this.workDataStore.on('error', (error) => {
      console.error('💥 Data store error:', error);
    });
  }
}

/**
 * 使用例の実行
 */
export function runWebSocketIntegrationExample(): void {
  const example = new WebSocketIntegrationExample();

  // システムを開始
  example.start().then(() => {
    console.log('Integration example is running...');

    // サンプルデータを処理
    setTimeout(() => {
      example.processSampleData();
    }, 1000);

    // 閾値達成をシミュレート
    setTimeout(() => {
      example.simulateThresholdReached();
    }, 3000);

    // 接続品質をシミュレート
    setTimeout(() => {
      example.simulateConnectionQuality();
    }, 5000);

    // 10秒後にシステムを停止
    setTimeout(() => {
      example.stop();
    }, 10000);
  }).catch((error) => {
    console.error('Failed to start integration example:', error);
  });
}

// このファイルが直接実行された場合の処理
if (require.main === module) {
  runWebSocketIntegrationExample();
}