# CRANE V2+ WebSocket Bridge

CRANE V2+ロボットをros-humble-rosbridge-serverを通じてWebSocketで制御するPythonサンプルコード集です。

## 概要

このパッケージには以下の3つのサンプルコードが含まれています：

1. **crane_plus_websocket_publisher.py** - CRANE V2+のトピックを送信
2. **crane_plus_websocket_subscriber.py** - CRANE V2+のトピックを購読・監視
3. **crane_plus_websocket_controller.py** - インタラクティブな制御システム

## 必要要件

### ROS 2パッケージ
- ROS 2 Humble
- ros-humble-rosbridge-server
- CRANE V2+関連パッケージ（crane_plus_control, crane_plus_description等）

### Pythonパッケージ
```bash
pip install websocket-client rel
```

## 使用方法

### 1. ROSBridge Serverの起動

```bash
# 新しいターミナルで
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 2. CRANE V2+システムの起動

```bash
# ハードウェア接続がある場合
ros2 launch crane_plus_control crane_plus_control.launch.py

# Gazeboシミュレーションの場合
ros2 launch crane_plus_gazebo crane_plus_gazebo.launch.py
```

### 3. WebSocketクライアントの実行

#### パブリッシャー（送信）
```bash
cd /home/kuni/JASA/crane_plus/crane_plus_examples/src
python3 crane_plus_websocket_publisher.py

# または異なるURL指定
python3 crane_plus_websocket_publisher.py --url ws://192.168.1.100:9090
```

#### サブスクライバー（購読・監視）
```bash
python3 crane_plus_websocket_subscriber.py

# 特定のトピックのみ監視
python3 crane_plus_websocket_subscriber.py --topics joint_states controller
```

#### インタラクティブ制御
```bash
python3 crane_plus_websocket_controller.py
```

## 機能説明

### Publisher（送信機能）
- ジョイント状態の定期送信（サイン波パターン）
- アームトラジェクトリ制御コマンド送信
- グリッパー開閉制御
- 自動広告（advertise）機能

### Subscriber（購読機能）
- ジョイント状態の監視
- コントローラー状態の監視
- TF（座標変換）の監視
- ロボット記述（URDF）の受信
- 統計情報表示

### Controller（統合制御）
- 予定義ポーズへの移動
  - Home: [0°, 0°, 0°, 0°]
  - Ready: [0°, -29°, 29°, 0°] 
  - Pickup: [0°, -46°, 46°, 0°]
  - Place: [90°, -29°, 29°, 0°]
  - Inspect: [0°, -17°, 17°, -29°]
- グリッパー制御（開く・閉じる・中立）
- 自動ピック&プレースデモ
- リアルタイム状態監視
- インタラクティブメニュー

## CRANE V2+トピック一覧

### 購読対象トピック
| トピック | メッセージタイプ | 説明 |
|---------|------------------|------|
| `/joint_states` | `sensor_msgs/msg/JointState` | ジョイント状態 |
| `/crane_plus_arm_controller/state` | `control_msgs/msg/JointTrajectoryControllerState` | アーム制御状態 |
| `/crane_plus_gripper_controller/state` | `control_msgs/msg/JointTrajectoryControllerState` | グリッパー制御状態 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 座標変換 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静的座標変換 |
| `/robot_description` | `std_msgs/msg/String` | ロボット記述（URDF） |

### 送信対象トピック
| トピック | メッセージタイプ | 説明 |
|---------|------------------|------|
| `/crane_plus_arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | アーム制御コマンド |
| `/crane_plus_gripper_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | グリッパー制御コマンド |

### ジョイント構成
```
アーム:
- crane_plus_joint1 (ベース回転)
- crane_plus_joint2 (第1リンク)
- crane_plus_joint3 (第2リンク) 
- crane_plus_joint4 (第3リンク)

グリッパー:
- crane_plus_joint_hand (開閉)
```

## トラブルシューティング

### 接続エラー
```
[error] [Errno 111] Connection refused
```
→ ROSBridge Serverが起動していることを確認

### トピックが見つからない
```
[error] Topic not found: /joint_states
```
→ CRANE V2+システムが正常に起動していることを確認

### 依存関係エラー
```
ModuleNotFoundError: No module named 'websocket'
```
→ `pip install websocket-client rel`で依存関係をインストール

## 開発者情報

参考元ファイル:
- `/home/kuni/JASA/edgetech2025-worker-assist/twin-gateway-rosbridge/rosbridge_test_code_python/src/rosbridge_client_publish.py`
- `/home/kuni/JASA/edgetech2025-worker-assist/twin-gateway-rosbridge/rosbridge_test_code_python/src/rosbridge_client_subsclibe.py`

CRANE V2+仕様:
- RT Corporation CRANE V2+ ロボットアーム
- ROS 2 Humble対応
- MoveIt2統合制御

## ライセンス

このサンプルコードはApache License 2.0の下で提供されています。