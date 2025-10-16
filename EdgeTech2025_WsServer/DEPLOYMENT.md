# デプロイメントガイド

このドキュメントでは、Raspberry Pi WebSocketサーバーをラズパイにデプロイする手順を説明します。

## 前提条件

### ローカルマシン
- Node.js 18以上
- npm
- rsync
- SSH接続設定済み

### Raspberry Pi
- Raspberry Pi OS (Bullseye以降推奨)
- Node.js 18以上
- npm
- SSH接続可能

## 初回デプロイ手順

### 1. ラズパイの準備

ラズパイにNode.jsがインストールされていない場合:

```bash
# ラズパイにSSH接続
ssh pi@raspberrypi

# Node.js 18.xをインストール
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# バージョン確認
node -v
npm -v
```

または、初期セットアップスクリプトを使用:

```bash
# ローカルマシンから実行
ssh pi@raspberrypi 'bash -s' < scripts/setup-raspi.sh
```

### 2. プロジェクトのビルドとデプロイ

```bash
# ローカルマシンで実行
npm run deploy

# または、ホスト名とユーザー名を指定
./scripts/deploy.sh raspberrypi pi
```

このスクリプトは以下を実行します:
- プロジェクトのビルド
- ファイルのラズパイへの転送
- 本番用依存パッケージのインストール
- ログディレクトリの作成
- 設定ファイルの確認

### 3. systemdサービスのインストール

```bash
# ローカルマシンから実行
npm run deploy:service

# または
./scripts/install-service.sh raspberrypi pi
```

### 4. サービスの起動

```bash
# ラズパイにSSH接続して実行
ssh pi@raspberrypi
sudo systemctl start raspberry-pi-websocket-server
sudo systemctl status raspberry-pi-websocket-server
```

### 5. 動作確認

```bash
# ヘルスチェック
curl http://raspberrypi:3001/health

# ログ確認
sudo journalctl -u raspberry-pi-websocket-server -f
```

## 更新デプロイ

コードを更新した後、素早くデプロイする場合:

```bash
# クイックデプロイ（ビルド、転送、再起動）
npm run deploy:quick

# または
./scripts/quick-deploy.sh raspberrypi pi
```

## デプロイスクリプト一覧

### deploy.sh
完全なデプロイを実行します。初回デプロイや大きな変更時に使用。

```bash
./scripts/deploy.sh [host] [user]
```

**実行内容:**
- プロジェクトのビルド
- 全ファイルの転送（node_modules除く）
- 依存パッケージのインストール
- ディレクトリ構造の作成

### quick-deploy.sh
素早い更新デプロイ。開発中の頻繁な更新に使用。

```bash
./scripts/quick-deploy.sh [host] [user]
```

**実行内容:**
- プロジェクトのビルド
- distディレクトリとpackage.jsonのみ転送
- サービスの再起動

### install-service.sh
systemdサービスをインストールします。

```bash
./scripts/install-service.sh [host] [user]
```

**実行内容:**
- サービスファイルのコピー
- systemdのリロード
- サービスの有効化

### setup-raspi.sh
ラズパイの初期セットアップを実行します。

```bash
ssh pi@raspberrypi 'bash -s' < scripts/setup-raspi.sh
```

**実行内容:**
- Node.jsのバージョン確認
- 必要なディレクトリの作成

## サービス管理コマンド

```bash
# サービス起動
sudo systemctl start raspberry-pi-websocket-server

# サービス停止
sudo systemctl stop raspberry-pi-websocket-server

# サービス再起動
sudo systemctl restart raspberry-pi-websocket-server

# サービス状態確認
sudo systemctl status raspberry-pi-websocket-server

# ログ確認（リアルタイム）
sudo journalctl -u raspberry-pi-websocket-server -f

# ログ確認（最新100行）
sudo journalctl -u raspberry-pi-websocket-server -n 100

# 自動起動の有効化
sudo systemctl enable raspberry-pi-websocket-server

# 自動起動の無効化
sudo systemctl disable raspberry-pi-websocket-server
```

## 設定ファイル

### config/config.json

デプロイ前に設定を確認・編集してください:

```json
{
  "port": 3001,
  "cors_origin": "*",
  "heartbeat_interval": 30000,
  "connection_timeout": 60000,
  "log_level": "info",
  "log_file": "logs/server.log"
}
```

本番環境では、`cors_origin`を適切に設定することを推奨します。

## トラブルシューティング

### サービスが起動しない

```bash
# 詳細なログを確認
sudo journalctl -u raspberry-pi-websocket-server -n 50

# 手動で起動してエラーを確認
cd ~/raspberry-pi-websocket-server
node dist/index.js
```

### ポートが使用中

```bash
# ポート3001を使用しているプロセスを確認
sudo lsof -i :3001

# プロセスを終了
sudo kill -9 <PID>
```

### メモリ不足

```bash
# メモリ使用状況を確認
free -h

# プロセスのメモリ使用量を確認
ps aux | grep node
```

### ログファイルが大きくなりすぎた

ログは自動的にローテーションされますが、手動で削除する場合:

```bash
cd ~/raspberry-pi-websocket-server/logs
rm *.log
sudo systemctl restart raspberry-pi-websocket-server
```

## セキュリティ考慮事項

1. **ファイアウォール設定**
   ```bash
   # ポート3001を開放（必要に応じて）
   sudo ufw allow 3001/tcp
   ```

2. **CORS設定**
   本番環境では`config.json`の`cors_origin`を特定のオリジンに制限してください。

3. **SSH鍵認証**
   パスワード認証ではなく、SSH鍵認証を使用することを推奨します。

## バックアップとロールバック

### バックアップ

```bash
# ラズパイ上でバックアップ
cd ~/raspberry-pi-websocket-server
tar -czf ../backup-$(date +%Y%m%d-%H%M%S).tar.gz .
```

### ロールバック

```bash
# バックアップから復元
cd ~
tar -xzf backup-YYYYMMDD-HHMMSS.tar.gz -C raspberry-pi-websocket-server/
sudo systemctl restart raspberry-pi-websocket-server
```

## パフォーマンス監視

```bash
# CPU使用率
top -p $(pgrep -f "node dist/index.js")

# メモリ使用量
ps aux | grep "node dist/index.js"

# ネットワーク接続
sudo netstat -tulpn | grep :3001
```

## 参考リンク

- [Node.js公式サイト](https://nodejs.org/)
- [systemd公式ドキュメント](https://www.freedesktop.org/wiki/Software/systemd/)
- [Raspberry Pi公式サイト](https://www.raspberrypi.org/)
