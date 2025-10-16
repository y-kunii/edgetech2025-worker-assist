# C++ センサーアプリ実装

このディレクトリには、C++で実装されたセンサーデータ取得アプリケーションのサンプルが含まれています。

## 必要な依存関係

### Ubuntu/Debian

```bash
# 基本的な開発ツール
sudo apt update
sudo apt install build-essential cmake pkg-config

# WebSocket通信ライブラリ
sudo apt install libwebsocketpp-dev libjsoncpp-dev

# OpenCV（カメラ使用時）
sudo apt install libopencv-dev

# Boost（非同期処理用）
sudo apt install libboost-all-dev

# pthread
sudo apt install libpthread-stubs0-dev
```

### Raspberry Pi OS

```bash
# 上記に加えて、GPIO制御用ライブラリ
sudo apt install wiringpi libwiringpi-dev

# I2C開発ライブラリ
sudo apt install libi2c-dev i2c-tools
```

## ビルド方法

```bash
# ビルドディレクトリ作成
mkdir build
cd build

# CMake設定
cmake ..

# ビルド実行
make

# 実行
./sensor_app
```

## 設定

### WebSocketサーバーURL

環境変数で設定可能：

```bash
export WEBSOCKET_URL=ws://192.168.1.100:3001
./sensor_app
```

### GPIO設定

Raspberry Pi上で実行する場合、GPIO権限が必要：

```bash
# ユーザーをgpioグループに追加
sudo usermod -a -G gpio $USER

# 再ログインまたは
newgrp gpio
```

### I2C設定

I2Cデバイスを使用する場合：

```bash
# I2Cを有効化
sudo raspi-config
# -> Interface Options -> I2C -> Enable

# I2Cデバイス確認
i2cdetect -y 1
```

## 実装の特徴

- **非同期処理**: Boostライブラリを使用した効率的な非同期I/O
- **WebSocket通信**: websocketpp を使用したSocket.io互換通信
- **センサー統合**: カメラ、GPIO、I2Cセンサーの統合サポート
- **エラーハンドリング**: 堅牢なエラー処理と自動復旧
- **リソース管理**: RAIIパターンによる適切なリソース管理

## パフォーマンス

C++実装の利点：

- **低メモリ使用量**: 通常20-50MB程度
- **低CPU使用率**: 効率的なネイティブコード
- **リアルタイム性**: 高精度なタイミング制御
- **長時間安定動作**: メモリリークなしの安定動作

## トラブルシューティング

### ビルドエラー

```bash
# 依存関係の確認
pkg-config --cflags --libs opencv4
pkg-config --cflags --libs jsoncpp

# CMakeキャッシュクリア
rm -rf build
mkdir build && cd build
cmake ..
```

### 実行時エラー

```bash
# ライブラリパス確認
ldd sensor_app

# 権限確認
ls -l /dev/i2c-*
ls -l /dev/video*
```

### GPIO/I2Cアクセスエラー

```bash
# 権限確認
groups $USER

# デバイス確認
ls -l /sys/class/gpio/
i2cdetect -y 1
```