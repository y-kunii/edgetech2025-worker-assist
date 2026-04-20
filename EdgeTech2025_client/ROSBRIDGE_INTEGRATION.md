# EdgeTech2025 ROSBridge統合ガイド

このドキュメントは、EdgeTech2025_ClientをROSBridgeプロトコルでtwin-gateway-rosbridgeに接続する方法を説明します。

## 概要

EdgeTech2025_Clientは、ROSBridgeプロトコルを使用してtwin-gateway-rosbridgeと通信します。これにより、ROSトピックを介してリアルタイムでデータを送受信できます。

## アーキテクチャ

```
┌─────────────────────────────────────────────────────────────────┐
│ twin-gateway-rosbridge Container                                │
│                                                                   │
│  ┌──────────────────┐         ┌──────────────────────┐         │
│  │ twin_bridge_node │ ------> │ rosbridge_websocket  │         │
│  │  (ROS2 Node)     │         │  (Port 9090)         │         │
│  │                  │ <------ │                      │         │
│  └──────────────────┘         └──────────────────────┘         │
│         │                              │                        │
│         │ Publish: /status_data        │ WebSocket              │
│         │ Subscribe: /robot_command    │ (ROSBridge Protocol)   │
│         │                              │                        │
└─────────┼──────────────────────────────┼─────────────────────────┘
          │                              │
          │                              │ ws://localhost:9090
          │                              │
          │                              ▼
┌─────────┴────────────────────────────────────────────────────────┐
│ EdgeTech2025_Client (Electron App)                               │
│                                                                   │
│  ┌──────────────────────────────────────────────────────┐       │
│  │ RosBridgeClient                                       │       │
│  │  - Subscribe: /status_data (StatusData)              │       │
│  │  - Publish: /robot_command (CommandData)             │       │
│  └──────────────────────────────────────────────────────┘       │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

## セットアップ手順

### 1. twin-gateway-rosbridgeのビルドと起動

```bash
cd twin-gateway-rosbridge

# Dockerイメージをビルド
docker build -t twin-gateway-rosbridge:humble .

# コンテナを起動（ROSBridgeサーバーとノードが自動起動）
docker run --rm -it \
  -v "$(pwd)/digital_twin_ws:/workspace/digital_twin_ws" \
  -p 9090:9090 \
  --name twin-gateway-rosbridge \
  twin-gateway-rosbridge:humble
```

起動すると以下のプロセスが開始されます:
- `rosbridge_websocket`: ポート9090でWebSocketサーバー
- `twin_bridge_node.py`: ステータスパブリッシュとコマンドサブスクライブ

### 2. メッセージ定義の確認

別のターミナルで:

```bash
docker exec -it twin-gateway-rosbridge bash

# 環境をセットアップ
source /opt/ros/humble/setup.bash
source /workspace/digital_twin_ws/install/setup.bash

# メッセージ定義を確認
ros2 interface list | grep twin_bridge
ros2 interface show twin_bridge/msg/StatusData
ros2 interface show twin_bridge/msg/CommandData
```

### 3. EdgeTech2025_Clientのビルドと起動

```bash
cd EdgeTech2025_client

# 依存関係をインストール
npm install

# ビルド
npm run build

# 起動
npm start
```

アプリケーションが起動したら:
1. 設定画面でWebSocketサーバーURLを設定: `ws://localhost:9090`
2. 「接続」ボタンをクリック

## 通信プロトコル

### ROSBridge Subscribe（クライアント側）

接続時に自動的に送信:
```json
{
  "op": "subscribe",
  "topic": "/status_data",
  "type": "twin_bridge/msg/StatusData"
}
```

### StatusData受信（サーバー → クライアント）

```json
{
  "op": "publish",
  "topic": "/status_data",
  "msg": {
    "worker_status": "Working",
    "space_status": "Screw_tightening",
    "robot_state": "ready",
    "robot_grip": "open",
    "timestamp": "20241030120000",
    "tool_delivery": 5,
    "status": "Working"
  }
}
```

### CommandData送信（クライアント → サーバー）

```json
{
  "op": "publish",
  "topic": "/robot_command",
  "type": "twin_bridge/msg/CommandData",
  "msg": {
    "command": "tool_handover",
    "timestamp": "20241030120000"
  }
}
```

## テスト方法

### 1. ROSトピックの確認

```bash
docker exec -it twin-gateway-rosbridge bash
source /opt/ros/humble/setup.bash
source /workspace/digital_twin_ws/install/setup.bash

# アクティブなトピックを表示
ros2 topic list

# /status_dataトピックをエコー
ros2 topic echo /status_data

# /robot_commandトピックをエコー
ros2 topic echo /robot_command
```

### 2. 手動でコマンドを送信（テスト用）

```bash
# ROSコマンドでパブリッシュ
ros2 topic pub /robot_command twin_bridge/msg/CommandData \
  "{command: 'tool_handover', timestamp: '$(date +%Y%m%d%H%M%S)'}" --once
```

### 3. WebSocketクライアントでテスト（Python）

twin-gateway-rosbridge/rosbridge_test_code_python/ のテストスクリプトを使用:

```bash
cd twin-gateway-rosbridge/rosbridge_test_code_python

# コンテナをビルド
docker build -t python-websocket:ws .

# コンテナを起動
docker run --rm -it --name rosbridge-python-client \
  --network host \
  -v $(pwd)/src:/app/src \
  python-websocket:ws

# Subscribeテスト
python rosbridge_client_subsclibe.py

# Publishテスト（別ターミナル）
python rosbridge_client_publish.py
```

## トラブルシューティング

### 接続できない場合

1. **ROSBridgeサーバーが起動しているか確認**
   ```bash
   docker logs twin-gateway-rosbridge
   ```

2. **ポート9090が使用可能か確認**
   ```bash
   netstat -an | grep 9090
   ```

3. **ファイアウォール設定を確認**
   - Windowsファイアウォールでポート9090を許可

### メッセージが受信できない場合

1. **トピックが正しくパブリッシュされているか確認**
   ```bash
   ros2 topic echo /status_data
   ```

2. **twin_bridge_nodeが起動しているか確認**
   ```bash
   docker exec -it twin-gateway-rosbridge bash
   ros2 node list
   # /twin_bridge_node が表示されるはず
   ```

3. **メッセージ定義が正しくビルドされているか確認**
   ```bash
   ros2 interface show twin_bridge/msg/StatusData
   ros2 interface show twin_bridge/msg/CommandData
   ```

### コマンドが送信できない場合

1. **EdgeTech2025_Clientのログを確認**
   - Electronアプリの開発者ツール（F12）でコンソールログを確認

2. **ROSトピックでコマンドが受信されているか確認**
   ```bash
   ros2 topic echo /robot_command
   ```

3. **twin_bridge_nodeのログを確認**
   ```bash
   docker logs twin-gateway-rosbridge
   ```

## カスタマイズ

### トピック名を変更する場合

1. **twin_bridge_node.py** のトピック名を変更:
   ```python
   self.status_publisher = self.create_publisher(
       StatusData, 
       '/your_custom_status_topic',  # ここを変更
       10
   )
   ```

2. **EdgeTech2025_Client の main.ts** で設定を変更:
   ```typescript
   const config: RosBridgeClientConfig = {
     subscribeTopic: '/your_custom_status_topic',  // ここを変更
     publishTopic: '/your_custom_command_topic',   // ここを変更
     ...
   };
   ```

### メッセージフィールドを追加する場合

1. **msg定義ファイル**を編集:
   - `StatusData.msg` または `CommandData.msg`

2. **ワークスペースを再ビルド**:
   ```bash
   docker exec -it twin-gateway-rosbridge bash
   cd /workspace/digital_twin_ws
   colcon build
   ```

3. **コンテナを再起動**:
   ```bash
   docker restart twin-gateway-rosbridge
   ```

## 参考リンク

- [ROSBridge Protocol Specification](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [twin-gateway-rosbridge README](../twin-gateway-rosbridge/README.md)
- [EdgeTech2025_Client README](./README.md)
