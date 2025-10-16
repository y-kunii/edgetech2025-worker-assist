#!/bin/bash

# クイックデプロイスクリプト - ビルドして転送、サービス再起動
# 開発中の素早い更新用
# Usage: ./scripts/quick-deploy.sh [raspberry-pi-host] [user]

set -e

# デフォルト設定
RASPI_HOST="${1:-raspberrypi}"
RASPI_USER="${2:-pi}"
REMOTE_DIR="/home/${RASPI_USER}/raspberry-pi-websocket-server"
SERVICE_NAME="raspberry-pi-websocket-server"

echo "🚀 クイックデプロイ開始..."

# ビルド
echo "📦 ビルド中..."
npm run build

# 必要なファイルのみ転送
echo "📤 ファイル転送中..."
rsync -avz --progress \
  --exclude 'node_modules' \
  --exclude '.git' \
  --exclude 'tests' \
  --exclude 'logs/*.log' \
  --exclude '.DS_Store' \
  dist/ ${RASPI_USER}@${RASPI_HOST}:${REMOTE_DIR}/dist/

rsync -avz --progress \
  package.json \
  config/ \
  ${RASPI_USER}@${RASPI_HOST}:${REMOTE_DIR}/

# サービス再起動
echo "🔄 サービス再起動中..."
ssh ${RASPI_USER}@${RASPI_HOST} "sudo systemctl restart ${SERVICE_NAME}"

echo "✅ クイックデプロイ完了！"
echo ""
echo "ステータス確認:"
ssh ${RASPI_USER}@${RASPI_HOST} "sudo systemctl status ${SERVICE_NAME} --no-pager"
