# パフォーマンス最適化ドキュメント

## 概要

このドキュメントは、Raspberry Pi WebSocketサーバーに実装されたパフォーマンス最適化について説明します。

## 実装された最適化

### 1. メモリ使用量の最適化

#### 1.1 オブジェクトプール

頻繁に生成・破棄されるオブジェクトを再利用することで、ガベージコレクションの負荷を軽減し、メモリ使用量を削減します。

**実装場所:** `src/utils/ObjectPool.ts`

**使用例:**
```typescript
// ErrorInfoオブジェクトプール
const error = errorInfoPool.acquire();
error.code = 'ERROR_CODE';
error.message = 'Error message';
error.timestamp = new Date().toISOString();

// 使用後に返却
errorInfoPool.release(error);
```

**メリット:**
- オブジェクト生成のオーバーヘッド削減
- ガベージコレクション頻度の低減
- メモリフラグメンテーションの軽減

#### 1.2 データコピーの削減

データルーティング時に不要なコピーを避け、直接参照を渡すことでメモリ使用量を削減します。

**実装場所:** `src/server/MessageRouter.ts`

**最適化内容:**
- センサーデータの転送時にデータをコピーせず、直接参照を渡す
- ロボットコマンドの転送時も同様に直接参照を使用
- 複数クライアントへの送信でも同一オブジェクトを共有

**注意事項:**
- Socket.ioは内部でシリアライズを行うため、データの変更は影響しない
- 送信後にデータを変更する場合は注意が必要

### 2. メモリ監視機能

#### 2.1 自動メモリ監視

定期的にメモリ使用量をチェックし、閾値を超えた場合に警告を出力します。

**実装場所:** `src/utils/MemoryMonitor.ts`

**監視項目:**
- RSS (Resident Set Size): 物理メモリ使用量
- Heap Used: ヒープメモリ使用量
- Heap Total: ヒープメモリ総量
- External: C++オブジェクトのメモリ使用量
- Array Buffers: ArrayBufferのメモリ使用量

**デフォルト閾値:**
- RSS警告: 400MB
- RSS危険: 480MB
- ヒープ警告: 300MB
- ヒープ危険: 400MB

**設定例:**
```typescript
const memoryMonitor = new MemoryMonitor(
  logger,
  {
    rssWarning: 400 * 1024 * 1024,    // 400MB
    rssCritical: 480 * 1024 * 1024,   // 480MB
    heapWarning: 300 * 1024 * 1024,   // 300MB
    heapCritical: 400 * 1024 * 1024,  // 400MB
  },
  30000  // チェック間隔: 30秒
);

memoryMonitor.start();
```

#### 2.2 自動ガベージコレクション

危険閾値を超えた場合、自動的にガベージコレクションを実行します。

**有効化方法:**
```bash
node --expose-gc dist/index.js
```

**systemdサービスでの設定:**
```ini
[Service]
ExecStart=/usr/bin/node --expose-gc dist/index.js
```

### 3. 非同期処理の最適化

#### 3.1 I/O操作の非同期化

すべてのI/O操作を非同期で実行し、イベントループをブロックしないようにします。

**実装状況:**

| コンポーネント | 操作 | 非同期化 | 備考 |
|--------------|------|---------|------|
| ConfigManager | ファイル読み込み | 同期 | 起動時のみ実行、影響は限定的 |
| Logger | ログ書き込み | 非同期 | winston使用 |
| Logger | ディレクトリ作成 | 同期 | 起動時のみ実行 |
| MainServer | HTTP/WebSocket | 非同期 | すべて非同期 |
| MessageRouter | データ転送 | 非同期 | Socket.io使用 |
| HeartbeatManager | タイマー処理 | 非同期 | setInterval使用 |
| MemoryMonitor | メモリチェック | 非同期 | setInterval使用 |

**ConfigManagerの同期処理について:**
- 設定ファイルの読み込みは起動時に1回のみ実行
- ファイルサイズが小さい（数KB程度）ため、影響は無視できる
- 起動時の処理であり、リクエスト処理には影響しない

#### 3.2 イベントループのブロッキング回避

**実装されている対策:**

1. **長時間実行される処理の分割**
   - 大量のクライアントへの送信を一度に行わず、適切に分割
   - ただし、現在の実装では同期的にループ処理を行っている

2. **タイマーの適切な使用**
   - setIntervalを使用した定期処理
   - setTimeoutを使用した遅延処理

3. **Promise/async-awaitの活用**
   - サーバーの起動・停止処理
   - Socket.ioの非同期操作

**改善の余地:**
- 大量のクライアントへの送信時に、setImmediateやprocess.nextTickを使用した分割処理
- 現在は10クライアント程度を想定しているため、問題にはならない

#### 3.3 CPU集約的な処理の回避

**実装されている対策:**

1. **データ検証の最小化**
   - 必要最小限の検証のみ実行
   - 複雑な検証は避ける

2. **JSON解析の最適化**
   - Socket.ioが自動的に行うため、追加の解析は不要
   - 大きなデータの場合は注意が必要

3. **画像データの処理**
   - Base64エンコードされた画像をそのまま転送
   - サーバー側での画像処理は行わない

## パフォーマンス目標

### Raspberry Pi 4での目標値

| 項目 | 目標値 | 測定方法 |
|-----|--------|---------|
| メモリ使用量 | 512MB以下 | process.memoryUsage() |
| CPU使用率 | 50%以下 | top/htop |
| 起動時間 | 5秒以内 | 起動ログのタイムスタンプ |
| レスポンス時間 | 100ms以内 | ヘルスチェックAPI |
| 同時接続数 | 10クライアント | 統合テスト |
| メッセージスループット | 10msg/秒 | 統合テスト |

## 監視とメンテナンス

### メモリ使用量の監視

```bash
# プロセスのメモリ使用量確認
ps aux | grep node

# リアルタイム監視
top -p $(pgrep -f "node.*index.js")

# ヘルスチェックAPI
curl http://localhost:3001/health
```

### ログの確認

```bash
# メモリ警告の確認
grep "WARNING: Memory usage" logs/server.log

# メモリ危険の確認
grep "CRITICAL: Memory usage" logs/server.log

# ガベージコレクション実行の確認
grep "Forcing garbage collection" logs/server.log
```

### パフォーマンステスト

```bash
# 単体テスト
npm test

# 統合テスト
npm test -- tests/integration

# 負荷テスト（別途実装が必要）
npm run load-test
```

## トラブルシューティング

### メモリ使用量が高い場合

1. **メモリリークの確認**
   ```bash
   # ヒープスナップショットの取得（要heapdump）
   kill -USR2 $(pgrep -f "node.*index.js")
   ```

2. **接続数の確認**
   ```bash
   curl http://localhost:3001/health | jq '.connections'
   ```

3. **ガベージコレクションの強制実行**
   - メモリモニターが自動的に実行
   - --expose-gcフラグが必要

### CPU使用率が高い場合

1. **接続数の確認**
   - 想定以上のクライアントが接続していないか確認

2. **メッセージ頻度の確認**
   - センサーデータの送信頻度が高すぎないか確認

3. **ログレベルの調整**
   - debugログを無効化してオーバーヘッドを削減

## 今後の最適化案

### 短期的な改善

1. **接続プール**
   - データベース接続が必要になった場合の接続プール実装

2. **キャッシング**
   - 頻繁にアクセスされるデータのキャッシング

3. **圧縮**
   - WebSocket通信の圧縮（permessage-deflate）

### 長期的な改善

1. **クラスタリング**
   - 複数プロセスでの負荷分散
   - PM2やclusterモジュールの使用

2. **Redis統合**
   - セッション管理
   - Pub/Subによるメッセージング

3. **ネイティブモジュール**
   - 性能が重要な部分のC++実装

## 参考資料

- [Node.js Performance Best Practices](https://nodejs.org/en/docs/guides/simple-profiling/)
- [V8 Memory Management](https://v8.dev/blog/trash-talk)
- [Socket.io Performance Tips](https://socket.io/docs/v4/performance-tuning/)
