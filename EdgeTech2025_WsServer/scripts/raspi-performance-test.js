#!/usr/bin/env node

/**
 * Raspberry Pi Performance Test Script
 * 
 * This script verifies that the WebSocket server meets the performance requirements:
 * - Memory usage: ≤ 512MB
 * - CPU usage: ≤ 50%
 * - Startup time: measured and reported
 * 
 * Requirements: 8.1, 8.2
 */

const { spawn, exec } = require('child_process');
const { promisify } = require('util');
const execAsync = promisify(exec);

// Performance thresholds
const MEMORY_THRESHOLD_MB = 512;
const CPU_THRESHOLD_PERCENT = 50;
const MONITORING_DURATION_MS = 60000; // 1 minute
const SAMPLE_INTERVAL_MS = 1000; // 1 second

class PerformanceMonitor {
  constructor() {
    this.serverProcess = null;
    this.serverPid = null;
    this.startTime = null;
    this.samples = {
      memory: [],
      cpu: []
    };
  }

  /**
   * Start the WebSocket server and measure startup time
   */
  async startServer() {
    console.log('🚀 Starting WebSocket server...');
    this.startTime = Date.now();

    return new Promise((resolve, reject) => {
      this.serverProcess = spawn('node', ['dist/index.js'], {
        stdio: ['ignore', 'pipe', 'pipe'],
        detached: false
      });

      this.serverPid = this.serverProcess.pid;

      let output = '';
      
      this.serverProcess.stdout.on('data', (data) => {
        output += data.toString();
        process.stdout.write(data);
        
        // Check if server has started
        if (output.includes('WebSocket server is running') || 
            output.includes('Server started')) {
          const startupTime = Date.now() - this.startTime;
          console.log(`\n✅ Server started in ${startupTime}ms (PID: ${this.serverPid})`);
          resolve(startupTime);
        }
      });

      this.serverProcess.stderr.on('data', (data) => {
        process.stderr.write(data);
      });

      this.serverProcess.on('error', (error) => {
        reject(new Error(`Failed to start server: ${error.message}`));
      });

      this.serverProcess.on('exit', (code) => {
        if (code !== 0 && code !== null) {
          reject(new Error(`Server exited with code ${code}`));
        }
      });

      // Timeout if server doesn't start within 30 seconds
      setTimeout(() => {
        if (!this.serverProcess.killed) {
          reject(new Error('Server startup timeout (30s)'));
        }
      }, 30000);
    });
  }

  /**
   * Get memory usage for the server process in MB
   */
  async getMemoryUsage() {
    try {
      const { stdout } = await execAsync(`ps -p ${this.serverPid} -o rss=`);
      const memoryKB = parseInt(stdout.trim(), 10);
      return memoryKB / 1024; // Convert to MB
    } catch (error) {
      throw new Error(`Failed to get memory usage: ${error.message}`);
    }
  }

  /**
   * Get CPU usage for the server process as percentage
   */
  async getCPUUsage() {
    try {
      const { stdout } = await execAsync(`ps -p ${this.serverPid} -o %cpu=`);
      return parseFloat(stdout.trim());
    } catch (error) {
      throw new Error(`Failed to get CPU usage: ${error.message}`);
    }
  }

  /**
   * Monitor server performance for specified duration
   */
  async monitorPerformance(durationMs) {
    console.log(`\n📊 Monitoring performance for ${durationMs / 1000} seconds...`);
    console.log('Time(s)\tMemory(MB)\tCPU(%)');
    console.log('-------\t----------\t------');

    const startTime = Date.now();
    let sampleCount = 0;

    while (Date.now() - startTime < durationMs) {
      try {
        const memory = await this.getMemoryUsage();
        const cpu = await this.getCPUUsage();

        this.samples.memory.push(memory);
        this.samples.cpu.push(cpu);

        sampleCount++;
        const elapsed = Math.floor((Date.now() - startTime) / 1000);
        console.log(`${elapsed}\t${memory.toFixed(2)}\t\t${cpu.toFixed(2)}`);

        await this.sleep(SAMPLE_INTERVAL_MS);
      } catch (error) {
        console.error(`⚠️  Monitoring error: ${error.message}`);
        break;
      }
    }

    return sampleCount;
  }

  /**
   * Calculate statistics from samples
   */
  calculateStats(samples) {
    if (samples.length === 0) {
      return { min: 0, max: 0, avg: 0, median: 0 };
    }

    const sorted = [...samples].sort((a, b) => a - b);
    const sum = samples.reduce((acc, val) => acc + val, 0);

    return {
      min: sorted[0],
      max: sorted[sorted.length - 1],
      avg: sum / samples.length,
      median: sorted[Math.floor(sorted.length / 2)]
    };
  }

  /**
   * Generate performance report
   */
  generateReport(startupTime) {
    console.log('\n' + '='.repeat(60));
    console.log('📋 PERFORMANCE TEST REPORT');
    console.log('='.repeat(60));

    // Startup time
    console.log('\n🚀 Startup Time:');
    console.log(`   ${startupTime}ms`);

    // Memory statistics
    const memStats = this.calculateStats(this.samples.memory);
    console.log('\n💾 Memory Usage:');
    console.log(`   Minimum:  ${memStats.min.toFixed(2)} MB`);
    console.log(`   Maximum:  ${memStats.max.toFixed(2)} MB`);
    console.log(`   Average:  ${memStats.avg.toFixed(2)} MB`);
    console.log(`   Median:   ${memStats.median.toFixed(2)} MB`);
    console.log(`   Threshold: ${MEMORY_THRESHOLD_MB} MB`);
    
    const memoryPass = memStats.max <= MEMORY_THRESHOLD_MB;
    console.log(`   Status:   ${memoryPass ? '✅ PASS' : '❌ FAIL'}`);

    // CPU statistics
    const cpuStats = this.calculateStats(this.samples.cpu);
    console.log('\n⚡ CPU Usage:');
    console.log(`   Minimum:  ${cpuStats.min.toFixed(2)}%`);
    console.log(`   Maximum:  ${cpuStats.max.toFixed(2)}%`);
    console.log(`   Average:  ${cpuStats.avg.toFixed(2)}%`);
    console.log(`   Median:   ${cpuStats.median.toFixed(2)}%`);
    console.log(`   Threshold: ${CPU_THRESHOLD_PERCENT}%`);
    
    const cpuPass = cpuStats.avg <= CPU_THRESHOLD_PERCENT;
    console.log(`   Status:   ${cpuPass ? '✅ PASS' : '❌ FAIL'}`);

    // Overall result
    console.log('\n' + '='.repeat(60));
    const overallPass = memoryPass && cpuPass;
    console.log(`Overall Result: ${overallPass ? '✅ PASS' : '❌ FAIL'}`);
    console.log('='.repeat(60) + '\n');

    return {
      pass: overallPass,
      startupTime,
      memory: memStats,
      cpu: cpuStats
    };
  }

  /**
   * Stop the server
   */
  async stopServer() {
    if (this.serverProcess && !this.serverProcess.killed) {
      console.log('\n🛑 Stopping server...');
      
      return new Promise((resolve) => {
        this.serverProcess.on('exit', () => {
          console.log('✅ Server stopped');
          resolve();
        });

        // Try graceful shutdown first
        this.serverProcess.kill('SIGTERM');

        // Force kill after 5 seconds if still running
        setTimeout(() => {
          if (!this.serverProcess.killed) {
            console.log('⚠️  Forcing server shutdown...');
            this.serverProcess.kill('SIGKILL');
          }
        }, 5000);
      });
    }
  }

  /**
   * Sleep utility
   */
  sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

/**
 * Main test execution
 */
async function runPerformanceTest() {
  const monitor = new PerformanceMonitor();
  let exitCode = 0;

  try {
    console.log('🔍 Raspberry Pi Performance Test');
    console.log('='.repeat(60));
    console.log(`Memory Threshold: ${MEMORY_THRESHOLD_MB} MB`);
    console.log(`CPU Threshold: ${CPU_THRESHOLD_PERCENT}%`);
    console.log(`Monitoring Duration: ${MONITORING_DURATION_MS / 1000}s`);
    console.log('='.repeat(60) + '\n');

    // Check if build exists
    try {
      await execAsync('test -f dist/index.js');
    } catch (error) {
      console.error('❌ Error: dist/index.js not found. Please run "npm run build" first.');
      process.exit(1);
    }

    // Start server and measure startup time
    const startupTime = await monitor.startServer();

    // Wait a bit for server to stabilize
    await monitor.sleep(3000);

    // Monitor performance
    await monitor.monitorPerformance(MONITORING_DURATION_MS);

    // Generate report
    const report = monitor.generateReport(startupTime);

    if (!report.pass) {
      exitCode = 1;
    }

  } catch (error) {
    console.error(`\n❌ Test failed: ${error.message}`);
    exitCode = 1;
  } finally {
    await monitor.stopServer();
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
runPerformanceTest();
