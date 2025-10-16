#!/bin/bash

# デプロイスクリプト - ラズパイへの転送とセットアップ
# Usage: ./scripts/deploy.sh [raspberry-pi-host] [user]

set -e

# デフォルト設定
RASPI_HOST="${1:-raspberrypi}"
RASPI_USER="${2:-pi}"
REMOTE_DIR="/home/${RASPI_USER}/raspberry-pi-websocket-server"

echo "=========================================="
echo "Raspberry Pi WebSocket Server デプロイ"
echo "=========================================="
echo "ホスト: ${RASPI_USER}@${RASPI_HOST}"
echo "リモートディレクトリ: ${REMOTE_DIR}"
echo ""

# ビルド
echo "📦 プロジェクトをビルド中..."
npm run build

# ラズパイへの転送
echo ""
echo "📤 ファイルをラズパイに転送中..."
rsync -avz --progress \
  --exclude 'node_modules' \
  --exclude '.git' \
  --exclude 'tests' \
  --exclude 'logs/*.log' \
  --exclude '.DS_Store' \
  --exclude '*.log' \
  . ${RASPI_USER}@${RASPI_HOST}:${REMOTE_DIR}/

# リモートでのセットアップ
echo ""
echo "🔧 リモートでセットアップを実行中..."
ssh ${RASPI_USER}@${RASPI_HOST} "bash -s" << 'ENDSSH'
cd ~/raspberry-pi-websocket-server

# 本番用依存パッケージのインストール
echo "📥 依存パッケージをインストール中..."
npm install --production

# ログディレクトリの作成
echo "📁 ログディレクトリを作成中..."
mkdir -p logs

# 設定ファイルの確認
if [ ! -f config/config.json ]; then
  echo "⚠️  config.json が存在しません。テンプレートからコピーします..."
  cp config/config.template.json config/config.json
fi

echo "✅ セットアップ完了"
ENDSSH

echo ""
echo "=========================================="
echo "✅ デプロイ完了！"
echo "=========================================="
echo ""
echo "次のステップ:"
echo "1. systemdサービスをインストール:"
echo "   ./scripts/install-service.sh ${RASPI_HOST} ${RASPI_USER}"
echo ""
echo "2. サーバーを起動:"
echo "   ssh ${RASPI_USER}@${RASPI_HOST} 'sudo systemctl start raspberry-pi-websocket-server'"
echo ""
echo "3. ステータスを確認:"
echo "   ssh ${RASPI_USER}@${RASPI_HOST} 'sudo systemctl status raspberry-pi-websocket-server'"
echo ""
