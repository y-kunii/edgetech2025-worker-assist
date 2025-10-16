# Raspberry Pi Performance Test

このドキュメントでは、ラズパイ実機でのパフォーマンステストの実施方法を説明します。

## 概要

パフォーマンステストスクリプトは、以下の要件を検証します：

- **メモリ使用量**: 512MB以下
- **CPU使用率**: 50%以下（平均）
- **起動時間**: 測定と記録

**要件**: 8.1, 8.2

## 前提条件

1. プロジェクトがビルド済みであること
   ```bash
   npm run build
   ```

2. Node.jsがインストールされていること（v14以上推奨）

3. `ps`コマンドが利用可能であること（通常のLinux/macOS環境）

## テストの実行

### 基本的な実行方法

```bash
node scripts/raspi-performance-test.js
```

または

```bash
npm run test:performance
```

### ラズパイでの実行

1. ラズパイにSSH接続
   ```bash
   ssh pi@raspberrypi
   ```

2. プロジェクトディレクトリに移動
   ```bash
   cd raspberry-pi-websocket-server
   ```

3. 最新のコードをビルド
   ```bash
   npm run build
   ```

4. パフォーマンステストを実行
   ```bash
   node scripts/raspi-performance-test.js
   ```

## テストの流れ

1. **サーバー起動**: WebSocketサーバーを起動し、起動時間を測定
2. **安定化待機**: 3秒間待機してサーバーを安定化
3. **パフォーマンス監視**: 60秒間、1秒ごとにメモリとCPU使用率をサンプリング
4. **レポート生成**: 統計情報を計算し、合否判定を実施
5. **サーバー停止**: 優雅にサーバーをシャットダウン

## 出力例

```
🔍 Raspberry Pi Performance Test
============================================================
Memory Threshold: 512 MB
CPU Threshold: 50%
Monitoring Duration: 60s
============================================================

🚀 Starting WebSocket server...
[Server logs...]

✅ Server started in 1234ms (PID: 12345)

📊 Monitoring performance for 60 seconds...
Time(s)	Memory(MB)	CPU(%)
-------	----------	------
0	45.23		12.50
1	46.12		15.30
2	45.89		13.20
...

============================================================
📋 PERFORMANCE TEST REPORT
============================================================

🚀 Startup Time:
   1234ms

💾 Memory Usage:
   Minimum:  45.23 MB
   Maximum:  52.34 MB
   Average:  48.56 MB
   Median:   48.12 MB
   Threshold: 512 MB
   Status:   ✅ PASS

⚡ CPU Usage:
   Minimum:  8.50%
   Maximum:  25.30%
   Average:  15.67%
   Median:   14.80%
   Threshold: 50%
   Status:   ✅ PASS

============================================================
Overall Result: ✅ PASS
============================================================
```

## 合否判定基準

### メモリ使用量
- **PASS**: 最大メモリ使用量が512MB以下
- **FAIL**: 最大メモリ使用量が512MBを超える

### CPU使用率
- **PASS**: 平均CPU使用率が50%以下
- **FAIL**: 平均CPU使用率が50%を超える

### 総合判定
- **PASS**: メモリとCPUの両方がPASS
- **FAIL**: メモリまたはCPUのいずれかがFAIL

## トラブルシューティング

### エラー: dist/index.js not found

**原因**: プロジェクトがビルドされていない

**解決方法**:
```bash
npm run build
```

### エラー: Failed to start server

**原因**: ポート3001が既に使用されている、または設定ファイルに問題がある

**解決方法**:
1. 既存のサーバープロセスを停止
   ```bash
   pkill -f "node dist/index.js"
   ```

2. ポートの使用状況を確認
   ```bash
   lsof -i :3001
   ```

3. 設定ファイルを確認
   ```bash
   cat config/config.json
   ```

### エラー: Failed to get memory/CPU usage

**原因**: `ps`コマンドが利用できない、またはプロセスが既に終了している

**解決方法**:
1. `ps`コマンドが利用可能か確認
   ```bash
   which ps
   ```

2. サーバーが正常に起動しているか確認
   ```bash
   ps aux | grep node
   ```

### テストが途中で停止する

**原因**: サーバーがクラッシュした、またはメモリ不足

**解決方法**:
1. サーバーログを確認
   ```bash
   tail -f logs/server.log
   ```

2. システムのメモリ状況を確認
   ```bash
   free -h
   ```

3. 不要なプロセスを停止してメモリを確保

## 監視期間のカスタマイズ

スクリプト内の定数を変更することで、監視期間を調整できます：

```javascript
const MONITORING_DURATION_MS = 60000; // 60秒（デフォルト）
const SAMPLE_INTERVAL_MS = 1000;      // 1秒ごとにサンプリング
```

より長期間のテストを実施する場合：

```javascript
const MONITORING_DURATION_MS = 300000; // 5分
```

## 負荷テストとの組み合わせ

より現実的なテストを実施するには、負荷をかけながらパフォーマンステストを実行します：

1. パフォーマンステストを開始
   ```bash
   node scripts/raspi-performance-test.js
   ```

2. 別のターミナルで負荷テストを実行
   ```bash
   # 複数クライアントの接続
   # センサーデータの送信
   # ロボットコマンドの送信
   ```

## CI/CDへの統合

パフォーマンステストをCI/CDパイプラインに統合する場合：

```yaml
# .github/workflows/performance-test.yml
name: Performance Test

on:
  push:
    branches: [ main ]

jobs:
  performance:
    runs-on: self-hosted  # ラズパイをセルフホストランナーとして設定
    steps:
      - uses: actions/checkout@v2
      - name: Install dependencies
        run: npm ci
      - name: Build
        run: npm run build
      - name: Run performance test
        run: node scripts/raspi-performance-test.js
```

## 参考情報

- [要件定義書](../.kiro/specs/raspberry-pi-websocket-server/requirements.md) - 要件8.1, 8.2
- [設計書](../.kiro/specs/raspberry-pi-websocket-server/design.md) - パフォーマンス最適化セクション
- [パフォーマンス最適化ドキュメント](./PERFORMANCE_OPTIMIZATION.md)
