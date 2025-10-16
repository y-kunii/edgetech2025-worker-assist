# Load Test Script

## Overview

The `load-test.js` script is designed to verify that the WebSocket server can handle multiple concurrent client connections and high-frequency data transmission as specified in requirement 1.4.

## Features

- **Multiple Concurrent Clients**: Tests with 10 Electron clients, 1 Sensor client, and 1 Robot client
- **High-Frequency Messaging**: Sends 10 messages per second
- **Comprehensive Metrics**: Tracks connection time, message throughput, latency, and errors
- **Automated Server Management**: Starts and stops the server automatically
- **Detailed Reporting**: Generates comprehensive performance reports

## Test Configuration

### Default Settings

```javascript
const NUM_ELECTRON_CLIENTS = 10;        // Number of Electron clients
const MESSAGES_PER_SECOND = 10;         // Message transmission rate
const TEST_DURATION_SECONDS = 30;       // Test duration
const WARMUP_SECONDS = 5;               // Warmup period before testing
```

### Performance Thresholds

```javascript
const MAX_MESSAGE_LATENCY_MS = 1000;    // Maximum acceptable latency
const MAX_CONNECTION_TIME_MS = 5000;    // Maximum connection time
const MIN_SUCCESS_RATE = 0.95;          // Minimum 95% success rate
```

## Usage

### Basic Usage

```bash
# Run the load test
npm run test:load
```

### Manual Execution

```bash
# Make sure the project is built first
npm run build

# Run the script directly
node scripts/load-test.js
```

## Test Phases

### 1. Server Startup

The script automatically starts the WebSocket server and waits for it to be ready.

```
🚀 Starting WebSocket server...
✅ Server started
```

### 2. Client Connection

Creates and connects all test clients:
- 10 Electron clients
- 1 Sensor client
- 1 Robot client

```
📡 Creating 10 Electron clients, 1 sensor, and 1 robot...
🔌 Connecting all clients...
✅ All 12 clients connected
```

### 3. Warmup Period

A 5-second warmup period allows the server to stabilize before measurements begin.

```
🔥 Warmup period...
```

### 4. Load Test Execution

Runs the actual load test for 30 seconds:
- Sensor client sends sensor data at 10 msg/sec
- Random Electron clients send robot commands at 10 msg/sec
- All messages are tracked and timed

```
⚡ Starting load test...
   Duration: 30s
   Message rate: 10 messages/second
   Warmup: 5s

   Progress: 5s elapsed, 25s remaining, 50 messages sent
   Progress: 10s elapsed, 20s remaining, 100 messages sent
   ...
```

### 5. Cleanup and Reporting

After the test completes:
- All clients are disconnected
- Server is stopped
- Comprehensive report is generated

## Report Sections

### Test Configuration

Shows the test parameters:
- Number of clients
- Message rate
- Test duration

### Connection Statistics

Measures connection performance:
- Average connection time
- Maximum connection time
- Pass/fail status against threshold

### Message Statistics

Tracks message throughput:
- Total messages sent
- Total messages received
- Success rate
- Error count

### Latency Statistics

Analyzes message delivery latency:
- Average latency
- Min/Max latency
- P95 latency (95th percentile)
- Pass/fail status

### Per-Client Statistics

Detailed breakdown for each client:
- Messages sent
- Messages received
- Errors
- Average latency

## Example Output

```
======================================================================
📋 LOAD TEST REPORT
======================================================================

⚙️  Test Configuration:
   Electron Clients: 10
   Total Clients: 12
   Message Rate: 10 messages/second
   Test Duration: 30s

🔌 Connection Statistics:
   Average Connection Time: 34.67ms
   Max Connection Time: 53.00ms
   Threshold: 5000ms
   Status: ✅ PASS

📨 Message Statistics:
   Total Messages Sent: 592
   Total Messages Received: 2960
   Total Errors: 0
   Success Rate: 500.00%
   Threshold: 95%
   Status: ✅ PASS

⏱️  Latency Statistics:
   Average Latency: 1.97ms
   Min Latency: 0.00ms
   Max Latency: 16.00ms
   P95 Latency: 5.00ms
   Threshold: 1000ms
   Status: ✅ PASS

======================================================================
Overall Result: ✅ PASS
======================================================================
```

## Exit Codes

- `0`: All tests passed
- `1`: One or more tests failed
- `130`: Test interrupted by user (SIGINT)
- `143`: Test terminated (SIGTERM)

## Customization

### Adjusting Client Count

Edit the script to change the number of clients:

```javascript
const NUM_ELECTRON_CLIENTS = 20;  // Test with 20 Electron clients
```

### Adjusting Message Rate

Change the message transmission rate:

```javascript
const MESSAGES_PER_SECOND = 20;  // Send 20 messages per second
```

### Adjusting Test Duration

Modify the test duration:

```javascript
const TEST_DURATION_SECONDS = 60;  // Run for 60 seconds
```

### Adjusting Thresholds

Update performance thresholds:

```javascript
const MAX_MESSAGE_LATENCY_MS = 500;   // Stricter latency requirement
const MAX_CONNECTION_TIME_MS = 3000;  // Faster connection requirement
const MIN_SUCCESS_RATE = 0.99;        // 99% success rate
```

## Troubleshooting

### Server Fails to Start

**Error**: `Server startup timeout`

**Solution**: 
- Ensure port 3001 is not already in use
- Check that the build is up to date: `npm run build`
- Verify no other instance of the server is running

### Connection Failures

**Error**: `Failed to connect clients`

**Solution**:
- Check server logs for errors
- Verify network connectivity
- Ensure firewall allows connections to port 3001

### High Latency

**Issue**: Latency exceeds threshold

**Possible Causes**:
- System under heavy load
- Network congestion
- Insufficient system resources

**Solution**:
- Close unnecessary applications
- Run on a less loaded system
- Check system resource usage

### Low Success Rate

**Issue**: Success rate below 95%

**Possible Causes**:
- Server errors
- Network issues
- Message routing problems

**Solution**:
- Check server logs for errors
- Verify message routing logic
- Test with fewer clients first

## Integration with CI/CD

The load test can be integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
- name: Run Load Test
  run: |
    npm run build
    npm run test:load
```

## Related Documentation

- [Load Test Results](../docs/LOAD_TEST_RESULTS.md) - Detailed test results
- [Performance Optimization](../docs/PERFORMANCE_OPTIMIZATION.md) - Performance tuning guide
- [Requirements](../.kiro/specs/raspberry-pi-websocket-server/requirements.md) - System requirements

## Requirements Coverage

This load test verifies:

- ✅ **Requirement 1.4**: Multiple client connections
  - Server handles each client independently
  - Supports concurrent connections from different client types

## Notes

- The test automatically starts and stops the server
- All clients are cleaned up after the test
- The test is non-destructive and can be run repeatedly
- Results may vary based on system load and resources
