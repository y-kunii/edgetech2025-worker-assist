# Raspberry Pi WebSocket Server

製造現場デジタルツインシステム用のWebSocketサーバー

## 概要

このサーバーは、Electronクライアントアプリケーションと外部センサー/カメラプログラム、ロボット制御プログラム間のデータ中継を担当します。Node.jsとSocket.ioを使用し、Raspberry Pi上で軽量かつ効率的に動作するように設計されています。

## 主要機能

- **WebSocket通信**: Socket.ioによるリアルタイム双方向通信
- **クライアント管理**: 複数クライアントタイプ（Electron、センサー、ロボット）の識別と管理
- **データ中継**: センサーデータとロボット制御指示の効率的なルーティング
- **ハートビート監視**: 接続状態の自動監視とタイムアウト検出
- **ヘルスチェックAPI**: HTTP経由でのサーバー状態確認
- **ログ管理**: 日次ローテーション付きの詳細ログ記録
- **ラズパイ最適化**: 限られたリソースでの効率的な動作
- **systemdサービス**: システム起動時の自動起動とクラッシュ後の自動再起動

## 技術スタック

- **Node.js**: サーバーランタイム
- **Socket.io**: WebSocket通信
- **Express**: HTTPサーバー
- **TypeScript**: 型安全な開発
- **Winston**: ログ管理
- **Jest**: テストフレームワーク

## 必要要件

### ハードウェア
- **Raspberry Pi 4 Model B** (推奨: 2GB RAM以上)
- **microSDカード**: 16GB以上
- **電源**: 5V 3A USB-C電源アダプター
- **ネットワーク**: 有線LAN接続推奨

### ソフトウェア
- **OS**: Raspberry Pi OS (Bullseye以降)
- **Node.js**: 18.x 以上
- **npm**: 9.x 以上

## クイックスタート

### 1. Raspberry Piのセットアップ

```bash
# システムの更新
sudo apt update
sudo apt upgrade -y

# Node.jsのインストール（NodeSource経由）
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

# バージョン確認
node --version  # v18.x.x以上
npm --version   # 9.x.x以上
```

### 2. プロジェクトのセットアップ

```bash
# プロジェクトディレクトリに移動
cd /home/pi/raspberry-pi-websocket-server

# 依存パッケージのインストール
npm install

# TypeScriptのビルド
npm run build
```

### 3. 設定ファイルの作成

```bash
# 設定ファイルをテンプレートからコピー
cp config/config.template.json config/config.json

# 必要に応じて編集
nano config/config.json
```

### 4. サーバーの起動

```bash
# 開発環境（TypeScriptを直接実行）
npm run dev

# 本番環境（ビルド済みJavaScriptを実行）
npm start
```

### 5. 動作確認

```bash
# ヘルスチェック
curl http://localhost:3001/health

# JSONフォーマットで表示
curl -s http://localhost:3001/health | jq
```

## 設定

### 設定ファイル (config/config.json)

```json
{
  "port": 3001,                    // WebSocketサーバーのポート番号
  "cors_origin": "*",              // CORS許可オリジン（本番環境では制限推奨）
  "heartbeat_interval": 30000,     // ハートビート送信間隔（ミリ秒）
  "connection_timeout": 60000,     // 接続タイムアウト（ミリ秒）
  "log_level": "info",             // ログレベル: debug, info, warn, error
  "log_file": "logs/server.log"    // ログファイルパス
}
```

### 環境変数による設定

```bash
export PORT=3001
export LOG_LEVEL=debug
npm start
```

## 開発コマンド

- `npm run build` - TypeScriptビルド
- `npm run build:clean` - クリーンビルド
- `npm start` - サーバー起動
- `npm run dev` - 開発モード（ts-node使用）
- `npm test` - テスト実行
- `npm run test:watch` - テストウォッチモード
- `npm run test:coverage` - テストカバレッジ
- `npm run test:performance` - パフォーマンステスト
- `npm run test:load` - 負荷テスト

## プロジェクト構造

```
raspberry-pi-websocket-server/
├── src/
│   ├── index.ts                 # エントリーポイント
│   ├── server/
│   │   ├── MainServer.ts        # メインサーバー
│   │   ├── ConnectionManager.ts # 接続管理
│   │   ├── MessageRouter.ts     # メッセージルーティング
│   │   └── HeartbeatManager.ts  # ハートビート管理
│   ├── utils/
│   │   ├── Logger.ts            # ロガー
│   │   └── ConfigManager.ts     # 設定管理
│   └── types/
│       └── index.ts             # 型定義
├── config/
│   ├── config.template.json     # 設定ファイルテンプレート
│   └── config.json              # 実際の設定ファイル
├── logs/                        # ログディレクトリ
├── tests/                       # テストファイル
├── scripts/                     # デプロイ・テストスクリプト
├── dist/                        # ビルド出力
└── docs/                        # ドキュメント
```

## WebSocket通信プロトコル

### クライアントタイプ

サーバーは3種類のクライアントタイプを識別します：

- **electron**: Electronクライアントアプリケーション（複数接続可能）
- **sensor**: センサー/カメラプログラム（1接続のみ）
- **robot**: ロボット制御プログラム（1接続のみ）

### 接続フロー

```javascript
// クライアント側の接続例
const socket = io('ws://localhost:3001');

// クライアントタイプの登録
socket.emit('register_client', { client_type: 'electron' });

// 接続確認
socket.on('connect', () => {
  console.log('Connected to server');
});
```

### データフロー

```
センサー/カメラ → WebSocketサーバー → Electronクライアント
                      ↓
Electronクライアント → WebSocketサーバー → ロボット制御プログラム
```

### イベント一覧

#### クライアント → サーバー

| イベント名 | 送信元 | データ型 | 説明 |
|-----------|--------|---------|------|
| `register_client` | 全クライアント | `{ client_type: string }` | クライアントタイプの登録 |
| `sensor_data` | センサープログラム | `SensorData` | センサーデータ送信 |
| `robot_command` | Electronクライアント | `RobotCommand` | ロボット制御指示 |
| `robot_response` | ロボット制御プログラム | `RobotResponse` | ロボット応答 |
| `pong` | 全クライアント | `{ timestamp: number }` | ハートビート応答 |

#### サーバー → クライアント

| イベント名 | 送信先 | データ型 | 説明 |
|-----------|--------|---------|------|
| `sensor_data` | Electronクライアント | `SensorData` | センサーデータ転送 |
| `robot_command` | ロボット制御プログラム | `RobotCommand` | ロボット制御指示転送 |
| `robot_response` | Electronクライアント | `RobotResponse` | ロボット応答転送 |
| `ping` | 全クライアント | `{ timestamp: number }` | ハートビート |
| `error` | 該当クライアント | `ErrorInfo` | エラー通知 |
| `external_disconnected` | Electronクライアント | `{ client_type: string }` | 外部プログラム切断通知 |

### データ型定義

詳細なデータ型定義は `src/types/index.ts` を参照してください。

#### センサーデータ (`SensorData`)

```json
{
  "image": "data:image/jpeg;base64,/9j/4AAQ...", // Base64エンコードされた画像（オプション）
  "worker_status": "screw_tightening",           // 作業者状態
  "robot_status": {
    "state": "operating",                        // ロボット状態
    "grip": "open"                              // グリップ状態
  },
  "screw_count": 3,                             // ネジ締め回数
  "bolt_count": 1,                              // ボルト締め回数
  "work_step": "screw_tightening"               // 現在の作業ステップ
}
```

#### ロボット指示 (`RobotCommand`)

```json
{
  "command": "tool_handover",                   // 指示コマンド
  "data": {                                     // 追加データ（オプション）
    "tool_type": "screwdriver",
    "position": { "x": 100, "y": 200, "z": 50 }
  },
  "timestamp": "2024-01-01T12:00:00.000Z"     // タイムスタンプ
}
```

## ヘルスチェック

サーバーが起動したら、HTTPエンドポイントでヘルスチェックが可能です：

```bash
# 基本的なヘルスチェック
curl http://localhost:3001/health

# JSONフォーマットで表示
curl -s http://localhost:3001/health | jq
```

### ヘルスチェックレスポンス例

```json
{
  "status": "ok",
  "timestamp": "2025-10-14T12:00:00.000Z",
  "connections": {
    "electron": 1,
    "sensor": 1,
    "robot": 1,
    "unknown": 0,
    "total": 3
  },
  "sensor_connected": true,
  "robot_connected": true
}
```

## テスト

### 単体テスト

```bash
# 全単体テスト実行
npm test

# 特定のテストファイルを実行
npm test -- tests/unit/ConnectionManager.test.ts

# ウォッチモード
npm run test:watch

# カバレッジレポート生成
npm run test:coverage
```

### パフォーマンステスト

ラズパイ実機でのパフォーマンステストを実施できます：

```bash
# パフォーマンステストの実行
npm run test:performance
```

このテストは以下を検証します：
- **メモリ使用量**: 512MB以下
- **CPU使用率**: 50%以下（平均）
- **起動時間**: 測定と記録

詳細は [PERFORMANCE_TEST_QUICKSTART.md](PERFORMANCE_TEST_QUICKSTART.md) を参照してください。

### 負荷テスト

複数クライアント接続と高頻度データ送信の負荷テストを実施できます：

```bash
# 負荷テストの実行
npm run test:load
```

このテストは以下を検証します：
- **複数クライアント接続**: 10 Electronクライアント + 1センサー + 1ロボット
- **高頻度データ送信**: 10メッセージ/秒
- **接続時間**: 5秒以内
- **メッセージレイテンシ**: 1000ms以下
- **成功率**: 95%以上

詳細は [LOAD_TEST_QUICKSTART.md](LOAD_TEST_QUICKSTART.md) を参照してください。

## systemdサービスとして実行

Raspberry Pi上でシステム起動時に自動起動させる場合：

### 1. サービスファイルの配置

```bash
# サービスファイルをsystemdディレクトリにコピー
sudo cp raspberry-pi-websocket-server.service /etc/systemd/system/

# パーミッション設定
sudo chmod 644 /etc/systemd/system/raspberry-pi-websocket-server.service
```

### 2. サービスの有効化と起動

```bash
# systemdデーモンをリロード
sudo systemctl daemon-reload

# サービスを有効化（起動時に自動起動）
sudo systemctl enable raspberry-pi-websocket-server

# サービスを起動
sudo systemctl start raspberry-pi-websocket-server

# ステータス確認
sudo systemctl status raspberry-pi-websocket-server
```

### 3. サービス管理コマンド

```bash
# サービスの停止
sudo systemctl stop raspberry-pi-websocket-server

# サービスの再起動
sudo systemctl restart raspberry-pi-websocket-server

# ログの確認
sudo journalctl -u raspberry-pi-websocket-server -f
```

### 4. systemdサービステスト

サービスが正しく設定されているか確認するには：

```bash
# 完全なsystemdサービステスト
sudo ./scripts/test-systemd-service.sh
```

詳細は [SYSTEMD_TEST_QUICKSTART.md](SYSTEMD_TEST_QUICKSTART.md) を参照してください。

## デプロイメント

詳細なデプロイ手順については、[DEPLOYMENT.md](DEPLOYMENT.md) を参照してください。

### クイックスタート

#### 初回デプロイ

```bash
# 1. ビルドとデプロイ
npm run deploy

# 2. systemdサービスのインストール
npm run deploy:service

# 3. サービスの起動
ssh pi@raspberrypi 'sudo systemctl start raspberry-pi-websocket-server'
```

#### 更新デプロイ

```bash
# コード更新後の素早いデプロイ
npm run deploy:quick
```

### デプロイスクリプト

プロジェクトには以下のデプロイスクリプトが含まれています：

- `scripts/deploy.sh` - 完全なデプロイ（初回またはメジャー更新時）
- `scripts/quick-deploy.sh` - 素早い更新デプロイ（開発中）
- `scripts/install-service.sh` - systemdサービスのインストール
- `scripts/setup-raspi.sh` - ラズパイの初期セットアップ

## 監視とメンテナンス

### ログ監視

```bash
# アプリケーションログをリアルタイム表示
tail -f logs/server-$(date +%Y-%m-%d).log

# systemdログをリアルタイム表示
sudo journalctl -u raspberry-pi-websocket-server -f

# 過去のログを検索
grep "ERROR" logs/server-*.log
```

### パフォーマンス監視

```bash
# メモリ使用量の確認
ps aux | grep node

# CPU使用率の確認
top -p $(pgrep -f "node.*index.js")

# ネットワーク接続の確認
netstat -an | grep 3001
```

## トラブルシューティング

### 接続問題

#### サーバーが起動しない

```bash
# 詳細なログを確認
sudo journalctl -u raspberry-pi-websocket-server -n 50

# 手動で起動してエラーを確認
cd ~/raspberry-pi-websocket-server
node dist/index.js
```

#### ポートが使用中

```bash
# ポート3001を使用しているプロセスを確認
sudo lsof -i :3001

# プロセスを終了
sudo kill -9 <PID>
```

#### クライアントが接続できない

```bash
# ファイアウォール設定を確認
sudo ufw status

# ポートを開放（必要な場合）
sudo ufw allow 3001/tcp
```

### パフォーマンス問題

#### メモリ使用量が多い

```bash
# メモリ使用量を確認
free -h
ps aux --sort=-%mem | head

# Node.jsのメモリ制限を設定
node --max-old-space-size=512 dist/index.js
```

#### CPU使用率が高い

- ログレベルを `info` または `warn` に変更
- 不要なデバッグログを削除
- 接続数を確認

### データ問題

#### センサーデータが届かない

```bash
# センサープログラムの接続状態を確認
curl -s http://localhost:3001/health | jq '.sensor_connected'

# ログでエラーを確認
grep "sensor" logs/server-$(date +%Y-%m-%d).log
```

#### ロボット指示が送信されない

```bash
# ロボットプログラムの接続状態を確認
curl -s http://localhost:3001/health | jq '.robot_connected'

# ログでエラーを確認
grep "robot" logs/server-$(date +%Y-%m-%d).log
```

## セキュリティ

### 推奨事項

1. **CORS設定**: 本番環境では `cors_origin` を特定のドメインに制限
2. **ファイアウォール**: 必要なポートのみ開放
3. **認証**: 将来的にJWT認証の実装を検討
4. **TLS/SSL**: 本番環境ではHTTPS/WSSの使用を推奨
5. **定期更新**: 依存パッケージの定期的な更新

## パフォーマンス最適化

### メモリ最適化

- オブジェクトプールの使用
- 不要なデータコピーの削減
- 定期的なガベージコレクション

### CPU最適化

- 非同期処理の活用
- イベントループのブロッキング回避
- データ検証の最適化

### ネットワーク最適化

- データ圧縮の使用（必要に応じて）
- バッチ処理の実装
- 適切なバッファサイズの設定

## ライセンス

MIT

## サポート

問題が発生した場合は、以下を確認してください：

1. ログファイルの内容
2. ヘルスチェックAPIの応答
3. システムリソースの使用状況
4. ネットワーク接続状態

## 関連ドキュメント

- [DEPLOYMENT.md](DEPLOYMENT.md) - デプロイメント手順
- [PROJECT_SETUP.md](PROJECT_SETUP.md) - プロジェクトセットアップ
- [LOAD_TEST_QUICKSTART.md](LOAD_TEST_QUICKSTART.md) - 負荷テストガイド
- [PERFORMANCE_TEST_QUICKSTART.md](PERFORMANCE_TEST_QUICKSTART.md) - パフォーマンステストガイド
- [SYSTEMD_TEST_QUICKSTART.md](SYSTEMD_TEST_QUICKSTART.md) - systemdサービステストガイド
- [docs/](docs/) - 詳細なドキュメント
