/**
 * 基本的なセンサーデータ送信アプリ
 * Raspberry Pi WebSocketサーバーと連携してセンサーデータを送信
 */

const io = require('socket.io-client');

class BasicSensorApp {
  constructor(serverUrl = 'ws://localhost:3001') {
    this.serverUrl = serverUrl;
    this.socket = null;
    this.isConnected = false;
    this.isRegistered = false;
    this.dataInterval = null;
    
    // センサーデータの状態
    this.sensorState = {
      screwCount: 0,
      boltCount: 0,
      lastWorkerStatus: 'waiting',
      lastRobotState: 'waiting',
      lastRobotGrip: 'closed'
    };
    
    console.log(`Initializing sensor app for server: ${this.serverUrl}`);
  }
  
  /**
   * WebSocketサーバーに接続
   */
  connect() {
    console.log('Connecting to WebSocket server...');
    
    this.socket = io(this.serverUrl, {
      reconnection: true,
      reconnectionAttempts: 10,
      reconnectionDelay: 1000,
      timeout: 5000
    });
    
    this.setupEventHandlers();
  }
  
  /**
   * イベントハンドラーの設定
   */
  setupEventHandlers() {
    // 接続成功
    this.socket.on('connect', () => {
      console.log('✅ Connected to WebSocket server');
      this.isConnected = true;
      
      // センサークライアントとして登録
      console.log('📝 Registering as sensor client...');
      this.socket.emit('register_client', { client_type: 'sensor' });
    });
    
    // 登録完了
    this.socket.on('registered', (data) => {
      console.log('✅ Registered successfully:', data);
      this.isRegistered = true;
      
      // データ送信開始
      this.startSendingData();
    });
    
    // 切断
    this.socket.on('disconnect', (reason) => {
      console.log('❌ Disconnected from server:', reason);
      this.isConnected = false;
      this.isRegistered = false;
      this.stopSendingData();
    });
    
    // 接続エラー
    this.socket.on('connect_error', (error) => {
      console.error('❌ Connection error:', error.message);
    });
    
    // サーバーエラー
    this.socket.on('error', (error) => {
      console.error('❌ Server error:', error);
    });
    
    // ハートビート
    this.socket.on('ping', (data) => {
      this.socket.emit('pong', data);
      console.log('💓 Heartbeat responded');
    });
    
    // 再接続試行
    this.socket.on('reconnect_attempt', (attemptNumber) => {
      console.log(`🔄 Reconnection attempt ${attemptNumber}`);
    });
    
    // 再接続成功
    this.socket.on('reconnect', (attemptNumber) => {
      console.log(`✅ Reconnected after ${attemptNumber} attempts`);
    });
  }
  
  /**
   * データ送信開始
   */
  startSendingData() {
    if (this.dataInterval) {
      return;
    }
    
    console.log('🚀 Starting sensor data transmission...');
    
    // 1秒間隔でデータ送信
    this.dataInterval = setInterval(() => {
      if (this.isConnected && this.isRegistered) {
        const sensorData = this.collectSensorData();
        this.socket.emit('sensor_data', sensorData);
        console.log('📤 Sensor data sent:', {
          worker_status: sensorData.worker_status,
          robot_state: sensorData.robot_status.state,
          screw_count: sensorData.screw_count,
          bolt_count: sensorData.bolt_count
        });
      }
    }, 1000);
  }
  
  /**
   * データ送信停止
   */
  stopSendingData() {
    if (this.dataInterval) {
      clearInterval(this.dataInterval);
      this.dataInterval = null;
      console.log('⏹️ Stopped sensor data transmission');
    }
  }
  
  /**
   * センサーデータの収集
   * 実際の実装では、ここで各種センサーからデータを取得
   */
  collectSensorData() {
    // 作業者状態の検出（モック実装）
    const workerStatus = this.detectWorkerStatus();
    
    // ロボット状態の取得（モック実装）
    const robotStatus = this.getRobotStatus();
    
    // カウント値の更新
    this.updateCounts(workerStatus);
    
    return {
      worker_status: workerStatus,
      robot_status: robotStatus,
      screw_count: this.sensorState.screwCount,
      bolt_count: this.sensorState.boltCount,
      work_step: workerStatus,
      timestamp: new Date().toISOString()
    };
  }
  
  /**
   * 作業者状態の検出（モック実装）
   * 実際の実装では、カメラ画像解析やセンサーデータから判定
   */
  detectWorkerStatus() {
    const states = ['waiting', 'screw_tightening', 'bolt_tightening', 'tool_handover', 'absent'];
    
    // 80%の確率で前回と同じ状態を維持（リアルな状態遷移をシミュレート）
    if (Math.random() > 0.2) {
      return this.sensorState.lastWorkerStatus;
    }
    
    // 20%の確率で状態変更
    const newStatus = states[Math.floor(Math.random() * states.length)];
    this.sensorState.lastWorkerStatus = newStatus;
    
    if (newStatus !== 'waiting') {
      console.log(`👷 Worker status changed to: ${newStatus}`);
    }
    
    return newStatus;
  }
  
  /**
   * ロボット状態の取得（モック実装）
   * 実際の実装では、ロボットAPIから状態を取得
   */
  getRobotStatus() {
    // 90%の確率で前回と同じ状態を維持
    if (Math.random() > 0.1) {
      return {
        state: this.sensorState.lastRobotState,
        grip: this.sensorState.lastRobotGrip
      };
    }
    
    // 10%の確率で状態変更
    const states = ['waiting', 'operating'];
    const grips = ['open', 'closed'];
    
    this.sensorState.lastRobotState = states[Math.floor(Math.random() * states.length)];
    this.sensorState.lastRobotGrip = grips[Math.floor(Math.random() * grips.length)];
    
    console.log(`🤖 Robot status changed: ${this.sensorState.lastRobotState}, grip: ${this.sensorState.lastRobotGrip}`);
    
    return {
      state: this.sensorState.lastRobotState,
      grip: this.sensorState.lastRobotGrip
    };
  }
  
  /**
   * カウント値の更新
   */
  updateCounts(workerStatus) {
    // ネジ締め作業中の場合、5%の確率でカウント増加
    if (workerStatus === 'screw_tightening' && Math.random() > 0.95) {
      this.sensorState.screwCount++;
      console.log(`🔩 Screw count increased: ${this.sensorState.screwCount}`);
    }
    
    // ボルト締め作業中の場合、3%の確率でカウント増加
    if (workerStatus === 'bolt_tightening' && Math.random() > 0.97) {
      this.sensorState.boltCount++;
      console.log(`🔧 Bolt count increased: ${this.sensorState.boltCount}`);
    }
  }
  
  /**
   * アプリケーションの停止
   */
  stop() {
    console.log('🛑 Stopping sensor app...');
    this.stopSendingData();
    
    if (this.socket) {
      this.socket.disconnect();
    }
  }
}

// 使用例
function main() {
  // 環境変数からサーバーURLを取得（デフォルト: localhost:3001）
  const serverUrl = process.env.WEBSOCKET_URL || 'ws://localhost:3001';
  
  console.log('🚀 Starting Basic Sensor App');
  console.log(`📡 Target server: ${serverUrl}`);
  
  const sensorApp = new BasicSensorApp(serverUrl);
  
  // サーバーに接続
  sensorApp.connect();
  
  // 優雅な終了処理
  process.on('SIGINT', () => {
    console.log('\n🛑 Received SIGINT, shutting down gracefully...');
    sensorApp.stop();
    process.exit(0);
  });
  
  process.on('SIGTERM', () => {
    console.log('\n🛑 Received SIGTERM, shutting down gracefully...');
    sensorApp.stop();
    process.exit(0);
  });
}

// スクリプトが直接実行された場合のみmain関数を呼び出し
if (require.main === module) {
  main();
}

module.exports = BasicSensorApp;