#!/bin/bash

# ラズパイ初期セットアップスクリプト
# ラズパイ上で実行するスクリプト
# Usage: ssh pi@raspberrypi 'bash -s' < scripts/setup-raspi.sh

set -e

echo "=========================================="
echo "Raspberry Pi 初期セットアップ"
echo "=========================================="
echo ""

# Node.jsのバージョン確認
echo "📋 Node.jsバージョンを確認中..."
if command -v node &> /dev/null; then
  NODE_VERSION=$(node -v)
  echo "✅ Node.js ${NODE_VERSION} がインストールされています"
else
  echo "❌ Node.jsがインストールされていません"
  echo ""
  echo "Node.jsをインストールしてください:"
  echo "  curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -"
  echo "  sudo apt-get install -y nodejs"
  exit 1
fi

# npmのバージョン確認
if command -v npm &> /dev/null; then
  NPM_VERSION=$(npm -v)
  echo "✅ npm ${NPM_VERSION} がインストールされています"
else
  echo "❌ npmがインストールされていません"
  exit 1
fi

# 必要なディレクトリの作成
echo ""
echo "📁 ディレクトリを作成中..."
mkdir -p ~/raspberry-pi-websocket-server
cd ~/raspberry-pi-websocket-server
mkdir -p logs config

echo ""
echo "=========================================="
echo "✅ 初期セットアップ完了！"
echo "=========================================="
echo ""
echo "次のステップ:"
echo "1. ローカルマシンからデプロイを実行:"
echo "   ./scripts/deploy.sh"
echo ""
