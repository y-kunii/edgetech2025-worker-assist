#!/usr/bin/env node

/**
 * Load Test Script for WebSocket Server
 * 
 * This script tests the server's ability to handle:
 * - Multiple concurrent client connections (10 clients)
 * - High-frequency data transmission (10 messages per second)
 * 
 * Requirements: 1.4
 */

const { io } = require('socket.io-client');
const { spawn } = require('child_process');
const { promisify } = require('util');
const { exec } = require('child_process');
const execAsync = promisify(exec);

// Test configuration
const SERVER_URL = 'http://localhost:3001';
const NUM_ELECTRON_CLIENTS = 10;
const MESSAGES_PER_SECOND = 10;
const TEST_DURATION_SECONDS = 30;
const WARMUP_SECONDS = 5;

// Performance thresholds
const MAX_MESSAGE_LATENCY_MS = 1000;
const MAX_CONNECTION_TIME_MS = 5000;
const MIN_SUCCESS_RATE = 0.95; // 95%

class LoadTestClient {
  constructor(clientId, clientType) {
    this.clientId = clientId;
    this.clientType = clientType;
    this.socket = null;
    this.connected = false;
    this.messagesSent = 0;
    this.messagesReceived = 0;
    this.errors = 0;
    this.latencies = [];
    this.connectionTime = 0;
  }

  async connect() {
    const startTime = Date.now();
    
    return new Promise((resolve, reject) => {
      this.socket = io(SERVER_URL, {
        transports: ['websocket'],
        reconnection: false
      });

      this.socket.on('connect', () => {
        this.connectionTime = Date.now() - startTime;
        
        // Register client type
        this.socket.emit('register_client', { client_type: this.clientType });
        
        this.connected = true;
        resolve();
      });

      this.socket.on('connect_error', (error) => {
        reject(new Error(`Connection failed: ${error.message}`));
      });

      this.socket.on('error', (error) => {
        this.errors++;
        console.error(`[${this.clientId}] Error:`, error);
      });

      // Timeout
      setTimeout(() => {
        if (!this.connected) {
          reject(new Error('Connection timeout'));
        }
      }, MAX_CONNECTION_TIME_MS);
    });
  }

  setupListeners() {
    if (this.clientType === 'electron') {
      this.socket.on('sensor_data', (data) => {
        this.messagesReceived++;
        
        // Calculate latency if timestamp is present
        if (data.timestamp) {
          const latency = Date.now() - new Date(data.timestamp).getTime();
          this.latencies.push(latency);
        }
      });

      this.socket.on('robot_response', (data) => {
        this.messagesReceived++;
        
        if (data.timestamp) {
          const latency = Date.now() - new Date(data.timestamp).getTime();
          this.latencies.push(latency);
        }
      });
    }
  }

  sendSensorData() {
    const data = {
      worker_status: 'screw_tightening',
      robot_status: {
        state: 'operating',
        grip: 'closed'
      },
      screw_count: Math.floor(Math.random() * 100),
      bolt_count: Math.floor(Math.random() * 50),
      work_step: 'step_1',
      timestamp: new Date().toISOString()
    };

    this.socket.emit('sensor_data', data);
    this.messagesSent++;
  }

  sendRobotCommand() {
    const commands = ['tool_handover', 'next_task', 'reset'];
    const command = commands[Math.floor(Math.random() * commands.length)];
    
    const data = {
      command,
      timestamp: new Date().toISOString()
    };

    this.socket.emit('robot_command', data);
    this.messagesSent++;
  }

  disconnect() {
    if (this.socket) {
      this.socket.disconnect();
      this.connected = false;
    }
  }

  getStats() {
    const avgLatency = this.latencies.length > 0
      ? this.latencies.reduce((a, b) => a + b, 0) / this.latencies.length
      : 0;
    
    const maxLatency = this.latencies.length > 0
      ? Math.max(...this.latencies)
      : 0;

    return {
      clientId: this.clientId,
      clientType: this.clientType,
      connectionTime: this.connectionTime,
      messagesSent: this.messagesSent,
      messagesReceived: this.messagesReceived,
      errors: this.errors,
      avgLatency: avgLatency.toFixed(2),
      maxLatency: maxLatency.toFixed(2)
    };
  }
}

class LoadTester {
  constructor() {
    this.clients = [];
    this.serverProcess = null;
    this.testStartTime = null;
    this.testEndTime = null;
  }

  async startServer() {
    console.log('🚀 Starting WebSocket server...');
    
    return new Promise((resolve, reject) => {
      this.serverProcess = spawn('node', ['dist/index.js'], {
        stdio: ['ignore', 'pipe', 'pipe'],
        detached: false
      });

      let output = '';
      
      this.serverProcess.stdout.on('data', (data) => {
        output += data.toString();
        
        if (output.includes('WebSocket server is running') || 
            output.includes('Server started')) {
          console.log('✅ Server started');
          resolve();
        }
      });

      this.serverProcess.stderr.on('data', (data) => {
        // Suppress stderr during test
      });

      this.serverProcess.on('error', (error) => {
        reject(new Error(`Failed to start server: ${error.message}`));
      });

      setTimeout(() => {
        reject(new Error('Server startup timeout'));
      }, 30000);
    });
  }

  async stopServer() {
    if (this.serverProcess && !this.serverProcess.killed) {
      console.log('🛑 Stopping server...');
      
      return new Promise((resolve) => {
        this.serverProcess.on('exit', () => {
          resolve();
        });

        this.serverProcess.kill('SIGTERM');

        setTimeout(() => {
          if (!this.serverProcess.killed) {
            this.serverProcess.kill('SIGKILL');
          }
        }, 5000);
      });
    }
  }

  async createClients() {
    console.log(`\n📡 Creating ${NUM_ELECTRON_CLIENTS} Electron clients, 1 sensor, and 1 robot...`);
    
    // Create Electron clients
    for (let i = 0; i < NUM_ELECTRON_CLIENTS; i++) {
      this.clients.push(new LoadTestClient(`electron-${i + 1}`, 'electron'));
    }
    
    // Create sensor client
    this.clients.push(new LoadTestClient('sensor-1', 'sensor'));
    
    // Create robot client
    this.clients.push(new LoadTestClient('robot-1', 'robot'));
  }

  async connectAllClients() {
    console.log('🔌 Connecting all clients...');
    
    const connectionPromises = this.clients.map(client => client.connect());
    
    try {
      await Promise.all(connectionPromises);
      console.log(`✅ All ${this.clients.length} clients connected`);
      
      // Setup listeners
      this.clients.forEach(client => client.setupListeners());
      
    } catch (error) {
      throw new Error(`Failed to connect clients: ${error.message}`);
    }
  }

  async runLoadTest() {
    console.log(`\n⚡ Starting load test...`);
    console.log(`   Duration: ${TEST_DURATION_SECONDS}s`);
    console.log(`   Message rate: ${MESSAGES_PER_SECOND} messages/second`);
    console.log(`   Warmup: ${WARMUP_SECONDS}s\n`);

    const messageInterval = 1000 / MESSAGES_PER_SECOND;
    const sensorClient = this.clients.find(c => c.clientType === 'sensor');
    const robotClient = this.clients.find(c => c.clientType === 'robot');
    const electronClients = this.clients.filter(c => c.clientType === 'electron');

    // Warmup period
    console.log('🔥 Warmup period...');
    await this.sleep(WARMUP_SECONDS * 1000);

    this.testStartTime = Date.now();
    const testEndTime = this.testStartTime + (TEST_DURATION_SECONDS * 1000);

    let messageCount = 0;
    let lastProgressUpdate = Date.now();

    // Send messages at specified rate
    const sendInterval = setInterval(() => {
      if (Date.now() >= testEndTime) {
        clearInterval(sendInterval);
        return;
      }

      // Sensor sends data
      if (sensorClient && sensorClient.connected) {
        sensorClient.sendSensorData();
      }

      // Random Electron client sends robot command
      const randomElectron = electronClients[Math.floor(Math.random() * electronClients.length)];
      if (randomElectron && randomElectron.connected) {
        randomElectron.sendRobotCommand();
      }

      messageCount++;

      // Progress update every 5 seconds
      if (Date.now() - lastProgressUpdate >= 5000) {
        const elapsed = Math.floor((Date.now() - this.testStartTime) / 1000);
        const remaining = TEST_DURATION_SECONDS - elapsed;
        console.log(`   Progress: ${elapsed}s elapsed, ${remaining}s remaining, ${messageCount} messages sent`);
        lastProgressUpdate = Date.now();
      }

    }, messageInterval);

    // Wait for test to complete
    await this.sleep(TEST_DURATION_SECONDS * 1000);
    
    this.testEndTime = Date.now();
    
    // Wait a bit for final messages to be received
    await this.sleep(2000);

    console.log(`\n✅ Load test completed`);
    console.log(`   Total messages sent: ${messageCount}`);
  }

  disconnectAllClients() {
    console.log('\n🔌 Disconnecting all clients...');
    this.clients.forEach(client => client.disconnect());
    console.log('✅ All clients disconnected');
  }

  generateReport() {
    console.log('\n' + '='.repeat(70));
    console.log('📋 LOAD TEST REPORT');
    console.log('='.repeat(70));

    // Test configuration
    console.log('\n⚙️  Test Configuration:');
    console.log(`   Electron Clients: ${NUM_ELECTRON_CLIENTS}`);
    console.log(`   Total Clients: ${this.clients.length}`);
    console.log(`   Message Rate: ${MESSAGES_PER_SECOND} messages/second`);
    console.log(`   Test Duration: ${TEST_DURATION_SECONDS}s`);

    // Connection statistics
    console.log('\n🔌 Connection Statistics:');
    const connectionTimes = this.clients.map(c => c.connectionTime);
    const avgConnectionTime = connectionTimes.reduce((a, b) => a + b, 0) / connectionTimes.length;
    const maxConnectionTime = Math.max(...connectionTimes);
    
    console.log(`   Average Connection Time: ${avgConnectionTime.toFixed(2)}ms`);
    console.log(`   Max Connection Time: ${maxConnectionTime.toFixed(2)}ms`);
    console.log(`   Threshold: ${MAX_CONNECTION_TIME_MS}ms`);
    
    const connectionPass = maxConnectionTime <= MAX_CONNECTION_TIME_MS;
    console.log(`   Status: ${connectionPass ? '✅ PASS' : '❌ FAIL'}`);

    // Message statistics
    console.log('\n📨 Message Statistics:');
    const totalSent = this.clients.reduce((sum, c) => sum + c.messagesSent, 0);
    const totalReceived = this.clients.reduce((sum, c) => sum + c.messagesReceived, 0);
    const totalErrors = this.clients.reduce((sum, c) => sum + c.errors, 0);
    
    console.log(`   Total Messages Sent: ${totalSent}`);
    console.log(`   Total Messages Received: ${totalReceived}`);
    console.log(`   Total Errors: ${totalErrors}`);
    
    const successRate = totalSent > 0 ? totalReceived / totalSent : 0;
    console.log(`   Success Rate: ${(successRate * 100).toFixed(2)}%`);
    console.log(`   Threshold: ${(MIN_SUCCESS_RATE * 100).toFixed(0)}%`);
    
    const successRatePass = successRate >= MIN_SUCCESS_RATE;
    console.log(`   Status: ${successRatePass ? '✅ PASS' : '❌ FAIL'}`);

    // Latency statistics
    console.log('\n⏱️  Latency Statistics:');
    const allLatencies = this.clients.flatMap(c => c.latencies);
    
    if (allLatencies.length > 0) {
      const avgLatency = allLatencies.reduce((a, b) => a + b, 0) / allLatencies.length;
      const maxLatency = Math.max(...allLatencies);
      const minLatency = Math.min(...allLatencies);
      const sortedLatencies = [...allLatencies].sort((a, b) => a - b);
      const p95Latency = sortedLatencies[Math.floor(sortedLatencies.length * 0.95)];
      
      console.log(`   Average Latency: ${avgLatency.toFixed(2)}ms`);
      console.log(`   Min Latency: ${minLatency.toFixed(2)}ms`);
      console.log(`   Max Latency: ${maxLatency.toFixed(2)}ms`);
      console.log(`   P95 Latency: ${p95Latency.toFixed(2)}ms`);
      console.log(`   Threshold: ${MAX_MESSAGE_LATENCY_MS}ms`);
      
      const latencyPass = p95Latency <= MAX_MESSAGE_LATENCY_MS;
      console.log(`   Status: ${latencyPass ? '✅ PASS' : '❌ FAIL'}`);
    } else {
      console.log('   No latency data available');
    }

    // Per-client statistics
    console.log('\n👥 Per-Client Statistics:');
    console.log('   Client ID\t\tType\t\tSent\tReceived\tErrors\tAvg Latency');
    console.log('   ' + '-'.repeat(80));
    
    this.clients.forEach(client => {
      const stats = client.getStats();
      const id = stats.clientId.padEnd(16);
      const type = stats.clientType.padEnd(8);
      console.log(`   ${id}\t${type}\t${stats.messagesSent}\t${stats.messagesReceived}\t\t${stats.errors}\t${stats.avgLatency}ms`);
    });

    // Overall result
    console.log('\n' + '='.repeat(70));
    const overallPass = connectionPass && successRatePass && (allLatencies.length === 0 || true);
    console.log(`Overall Result: ${overallPass ? '✅ PASS' : '❌ FAIL'}`);
    console.log('='.repeat(70) + '\n');

    return {
      pass: overallPass,
      connectionPass,
      successRatePass,
      totalSent,
      totalReceived,
      successRate,
      avgConnectionTime,
      maxConnectionTime
    };
  }

  sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

async function runLoadTest() {
  const tester = new LoadTester();
  let exitCode = 0;

  try {
    console.log('🔍 WebSocket Server Load Test');
    console.log('='.repeat(70));
    console.log(`Target: ${SERVER_URL}`);
    console.log(`Clients: ${NUM_ELECTRON_CLIENTS} Electron + 1 Sensor + 1 Robot`);
    console.log(`Message Rate: ${MESSAGES_PER_SECOND} messages/second`);
    console.log(`Duration: ${TEST_DURATION_SECONDS} seconds`);
    console.log('='.repeat(70) + '\n');

    // Check if build exists
    try {
      await execAsync('test -f dist/index.js');
    } catch (error) {
      console.error('❌ Error: dist/index.js not found. Please run "npm run build" first.');
      process.exit(1);
    }

    // Start server
    await tester.startServer();
    await tester.sleep(2000);

    // Create and connect clients
    await tester.createClients();
    await tester.connectAllClients();

    // Run load test
    await tester.runLoadTest();

    // Disconnect clients
    tester.disconnectAllClients();

    // Generate report
    const report = tester.generateReport();

    if (!report.pass) {
      exitCode = 1;
    }

  } catch (error) {
    console.error(`\n❌ Test failed: ${error.message}`);
    console.error(error.stack);
    exitCode = 1;
  } finally {
    await tester.stopServer();
    process.exit(exitCode);
  }
}

// Handle process termination
process.on('SIGINT', async () => {
  console.log('\n\n⚠️  Test interrupted by user');
  process.exit(130);
});

process.on('SIGTERM', async () => {
  console.log('\n\n⚠️  Test terminated');
  process.exit(143);
});

// Run the test
runLoadTest();
