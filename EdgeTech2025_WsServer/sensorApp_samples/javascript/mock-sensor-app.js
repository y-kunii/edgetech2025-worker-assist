/**
 * モックセンサーデータ送信アプリ
 * テスト用のリアルなセンサーデータパターンを生成
 */

const io = require('socket.io-client');

class MockSensorApp {
  constructor(serverUrl = 'ws://localhost:3001') {
    this.serverUrl = serverUrl;
    this.socket = null;
    this.isConnected = false;
    this.isRegistered = false;
    this.dataInterval = null;
    
    // シナリオベースのデータ生成
    this.scenario = {
      currentStep: 0,
      steps: [
        { name: 'waiting', duration: 5, worker_status: 'waiting' },
        { name: 'screw_start', duration: 8, worker_status: 'screw_tightening' },
        { name: 'tool_handover', duration: 3, worker_status: 'tool_handover' },
        { name: 'bolt_start', duration: 6, worker_status: 'bolt_tightening' },
        { name: 'waiting_end', duration: 4, worker_status: 'waiting' }
      ],
      stepStartTime: Date.now(),
      screwCount: 0,
      boltCount: 0,
      cycleCount: 0
    };
    
    console.log(`Initializing mock sensor app for server: ${this.serverUrl}`);
  }
  
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
  
  setupEventHandlers() {
    this.socket.on('connect', () => {
      console.log('✅ Connected to WebSocket server');
      this.isConnected = true;
      this.socket.emit('register_client', { client_type: 'sensor' });
    });
    
    this.socket.on('registered', (data) => {
      console.log('✅ Registered as mock sensor client:', data);
      this.isRegistered = true;
      this.startSendingData();
    });
    
    this.socket.on('disconnect', (reason) => {
      console.log('❌ Disconnected from server:', reason);
      this.isConnected = false;
      this.isRegistered = false;
      this.stopSendingData();
    });
    
    this.socket.on('connect_error', (error) => {
      console.error('❌ Connection error:', error.message);
    });
    
    this.socket.on('error', (error) => {
      console.error('❌ Server error:', error);
    });
    
    this.socket.on('ping', (data) => {
      this.socket.emit('pong', data);
    });
  }
  
  startSendingData() {
    if (this.dataInterval) return;
    
    console.log('🚀 Starting mock sensor data transmission...');
    console.log('📋 Scenario-based data generation active');
    
    this.dataInterval = setInterval(() => {
      if (this.isConnected && this.isRegistered) {
        const sensorData = this.generateScenarioData();
        this.socket.emit('sensor_data', sensorData);
        
        const currentStep = this.scenario.steps[this.scenario.currentStep];
        console.log(`📤 Mock data sent [${currentStep.name}]: ${sensorData.worker_status} | Screws: ${sensorData.screw_count} | Bolts: ${sensorData.bolt_count}`);
      }
    }, 1000);
  }
  
  stopSendingData() {
    if (this.dataInterval) {
      clearInterval(this.dataInterval);
      this.dataInterval = null;
      console.log('⏹️ Stopped mock sensor data transmission');
    }
  }
  
  /**
   * シナリオベースのデータ生成
   * リアルな製造作業のパターンをシミュレート
   */
  generateScenarioData() {
    const now = Date.now();
    const currentStep = this.scenario.steps[this.scenario.currentStep];
    const stepElapsed = (now - this.scenario.stepStartTime) / 1000;
    
    // ステップの進行チェック
    if (stepElapsed >= currentStep.duration) {
      this.advanceScenario();
    }
    
    // 現在のステップに基づいてデータ生成
    const workerStatus = currentStep.worker_status;
    const robotStatus = this.generateRobotStatus(workerStatus);
    
    // 作業に応じてカウント更新
    this.updateCountsForScenario(workerStatus, stepElapsed);
    
    return {
      worker_status: workerStatus,
      robot_status: robotStatus,
      screw_count: this.scenario.screwCount,
      bolt_count: this.scenario.boltCount,
      work_step: workerStatus,
      image: this.generateMockImage(),
      timestamp: new Date().toISOString()
    };
  }
  
  /**
   * シナリオの進行
   */
  advanceScenario() {
    const currentStep = this.scenario.steps[this.scenario.currentStep];
    console.log(`✅ Completed step: ${currentStep.name} (${currentStep.duration}s)`);
    
    this.scenario.currentStep = (this.scenario.currentStep + 1) % this.scenario.steps.length;
    this.scenario.stepStartTime = Date.now();
    
    // 1サイクル完了時
    if (this.scenario.currentStep === 0) {
      this.scenario.cycleCount++;
      console.log(`🔄 Completed cycle ${this.scenario.cycleCount}`);
    }
    
    const nextStep = this.scenario.steps[this.scenario.currentStep];
    console.log(`➡️ Starting step: ${nextStep.name} (${nextStep.duration}s)`);
  }
  
  /**
   * ロボット状態の生成
   */
  generateRobotStatus(workerStatus) {
    let state = 'waiting';
    let grip = 'closed';
    
    switch (workerStatus) {
      case 'tool_handover':
        state = 'operating';
        grip = 'open';
        break;
      case 'screw_tightening':
      case 'bolt_tightening':
        // 作業中は時々ロボットも動作
        if (Math.random() > 0.7) {
          state = 'operating';
        }
        break;
    }
    
    return { state, grip };
  }
  
  /**
   * シナリオに基づくカウント更新
   */
  updateCountsForScenario(workerStatus, stepElapsed) {
    const currentStep = this.scenario.steps[this.scenario.currentStep];
    
    if (workerStatus === 'screw_tightening') {
      // ネジ締め作業中、2-3秒ごとに1本完了
      const expectedScrews = Math.floor(stepElapsed / 2.5);
      const targetScrews = this.scenario.cycleCount * 3 + expectedScrews;
      
      if (this.scenario.screwCount < targetScrews) {
        this.scenario.screwCount++;
        console.log(`🔩 Screw completed! Total: ${this.scenario.screwCount}`);
      }
    }
    
    if (workerStatus === 'bolt_tightening') {
      // ボルト締め作業中、3-4秒ごとに1本完了
      const expectedBolts = Math.floor(stepElapsed / 3.5);
      const targetBolts = this.scenario.cycleCount * 2 + expectedBolts;
      
      if (this.scenario.boltCount < targetBolts) {
        this.scenario.boltCount++;
        console.log(`🔧 Bolt completed! Total: ${this.scenario.boltCount}`);
      }
    }
  }
  
  /**
   * モック画像データの生成（オプション）
   */
  generateMockImage() {
    // 実際の実装では不要だが、テスト用に小さなBase64画像を生成
    // 1x1ピクセルの透明PNG
    const mockImageBase64 = 'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==';
    
    // 10%の確率で画像データを含める（帯域幅節約のため）
    if (Math.random() > 0.9) {
      return `data:image/png;base64,${mockImageBase64}`;
    }
    
    return undefined;
  }
  
  /**
   * 現在のシナリオ状態を表示
   */
  printScenarioStatus() {
    const currentStep = this.scenario.steps[this.scenario.currentStep];
    const stepElapsed = (Date.now() - this.scenario.stepStartTime) / 1000;
    const remaining = Math.max(0, currentStep.duration - stepElapsed);
    
    console.log(`📊 Scenario Status:`);
    console.log(`   Current Step: ${currentStep.name} (${remaining.toFixed(1)}s remaining)`);
    console.log(`   Cycle: ${this.scenario.cycleCount + 1}`);
    console.log(`   Screws: ${this.scenario.screwCount}, Bolts: ${this.scenario.boltCount}`);
  }
  
  stop() {
    console.log('🛑 Stopping mock sensor app...');
    this.stopSendingData();
    
    if (this.socket) {
      this.socket.disconnect();
    }
  }
}

function main() {
  const serverUrl = process.env.WEBSOCKET_URL || 'ws://localhost:3001';
  
  console.log('🎭 Starting Mock Sensor App');
  console.log(`📡 Target server: ${serverUrl}`);
  console.log('📋 Simulating realistic manufacturing workflow...');
  
  const mockSensorApp = new MockSensorApp(serverUrl);
  mockSensorApp.connect();
  
  // 10秒ごとにシナリオ状態を表示
  const statusInterval = setInterval(() => {
    if (mockSensorApp.isRegistered) {
      mockSensorApp.printScenarioStatus();
    }
  }, 10000);
  
  // 優雅な終了処理
  const shutdown = () => {
    clearInterval(statusInterval);
    mockSensorApp.stop();
    process.exit(0);
  };
  
  process.on('SIGINT', () => {
    console.log('\n🛑 Received SIGINT, shutting down gracefully...');
    shutdown();
  });
  
  process.on('SIGTERM', () => {
    console.log('\n🛑 Received SIGTERM, shutting down gracefully...');
    shutdown();
  });
}

if (require.main === module) {
  main();
}

module.exports = MockSensorApp;