#!/bin/bash

# systemdサービスインストールスクリプト
# Usage: ./scripts/install-service.sh [raspberry-pi-host] [user]

set -e

# デフォルト設定
RASPI_HOST="${1:-raspberrypi}"
RASPI_USER="${2:-pi}"
REMOTE_DIR="/home/${RASPI_USER}/raspberry-pi-websocket-server"
SERVICE_NAME="raspberry-pi-websocket-server"

echo "=========================================="
echo "systemdサービスのインストール"
echo "=========================================="
echo "ホスト: ${RASPI_USER}@${RASPI_HOST}"
echo ""

# リモートでサービスをインストール
ssh ${RASPI_USER}@${RASPI_HOST} "bash -s" << ENDSSH
cd ${REMOTE_DIR}

echo "📋 サービスファイルをコピー中..."
sudo cp ${SERVICE_NAME}.service /etc/systemd/system/

echo "🔄 systemdをリロード中..."
sudo systemctl daemon-reload

echo "✅ サービスを有効化中..."
sudo systemctl enable ${SERVICE_NAME}

echo ""
echo "=========================================="
echo "✅ インストール完了！"
echo "=========================================="
echo ""
echo "サービスコマンド:"
echo "  起動: sudo systemctl start ${SERVICE_NAME}"
echo "  停止: sudo systemctl stop ${SERVICE_NAME}"
echo "  再起動: sudo systemctl restart ${SERVICE_NAME}"
echo "  状態確認: sudo systemctl status ${SERVICE_NAME}"
echo "  ログ確認: sudo journalctl -u ${SERVICE_NAME} -f"
echo ""
ENDSSH

echo "✅ リモートインストール完了"
