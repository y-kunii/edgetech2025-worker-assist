# Load Test Quick Start Guide

## Quick Start

Run the load test in 2 simple steps:

```bash
# 1. Build the project (if not already built)
npm run build

# 2. Run the load test
npm run test:load
```

## What Gets Tested

- ✅ **10 Electron clients** connecting simultaneously
- ✅ **1 Sensor client** sending data at 10 msg/sec
- ✅ **1 Robot client** receiving commands
- ✅ **30 seconds** of continuous operation
- ✅ **Connection performance** (< 5 seconds)
- ✅ **Message latency** (< 1000ms)
- ✅ **Success rate** (> 95%)

## Expected Results

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
   Average Connection Time: ~35ms
   Max Connection Time: ~53ms
   Status: ✅ PASS

📨 Message Statistics:
   Total Messages Sent: ~592
   Total Messages Received: ~2960
   Success Rate: ~500%
   Status: ✅ PASS

⏱️  Latency Statistics:
   Average Latency: ~2ms
   P95 Latency: ~5ms
   Status: ✅ PASS

======================================================================
Overall Result: ✅ PASS
======================================================================
```

## Test Duration

- **Warmup**: 5 seconds
- **Test**: 30 seconds
- **Cleanup**: 2 seconds
- **Total**: ~40 seconds

## Troubleshooting

### Port Already in Use

If you see "port already in use" error:

```bash
# Find and kill the process using port 3001
lsof -ti:3001 | xargs kill -9

# Then run the test again
npm run test:load
```

### Build Not Found

If you see "dist/index.js not found":

```bash
# Build the project first
npm run build

# Then run the test
npm run test:load
```

### Test Fails

If the test fails:

1. Check server logs: `tail -f logs/server-*.log`
2. Verify all dependencies: `npm install`
3. Check system resources: `top` or `htop`
4. Review detailed results in `docs/LOAD_TEST_RESULTS.md`

## Customization

To customize the test, edit `scripts/load-test.js`:

```javascript
// Change number of clients
const NUM_ELECTRON_CLIENTS = 20;  // Default: 10

// Change message rate
const MESSAGES_PER_SECOND = 20;   // Default: 10

// Change test duration
const TEST_DURATION_SECONDS = 60; // Default: 30
```

## Documentation

For more details, see:

- **Test Results**: [docs/LOAD_TEST_RESULTS.md](docs/LOAD_TEST_RESULTS.md)
- **Script Documentation**: [scripts/README_LOAD_TEST.md](scripts/README_LOAD_TEST.md)
- **Task Summary**: [docs/TASK_14.3_SUMMARY.md](docs/TASK_14.3_SUMMARY.md)

## CI/CD Integration

Add to your CI/CD pipeline:

```yaml
# GitHub Actions example
- name: Run Load Test
  run: |
    npm run build
    npm run test:load
```

## Requirements Verified

✅ **Requirement 1.4**: Multiple client connections
- Server handles each client independently
- Supports concurrent connections from different client types

---

**Quick Tip**: The test automatically starts and stops the server, so you don't need to run it manually!
