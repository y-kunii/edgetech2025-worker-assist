# CRANE V2+ WebSocket統合起動ガイド

## 起動方法の使い分け

### 🔧 開発・テスト用（分離起動）
```bash
# Terminal 1: 基本システム
ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0

# Terminal 2: ROSBridge（必要に応じて）
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 3: WebSocketコントローラー
cd /home/kuni/JASA/crane_plus/crane_plus_examples/src
python3 crane_plus_websocket_controller.py
```

### 🚀 本番・自動化用（統合起動）
```bash
# 全て一括起動
ros2 launch crane_plus_examples websocket_demo.launch.py port_name:=/dev/ttyUSB0

# オプション付き起動
ros2 launch crane_plus_examples websocket_demo.launch.py \
    port_name:=/dev/ttyUSB0 \
    rosbridge_port:=9090 \
    enable_websocket_controller:=true
```

## 起動オプション

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `port_name` | `/dev/ttyUSB0` | CRANE V2+のシリアルポート |
| `rosbridge_port` | `9090` | WebSocketポート番号 |
| `enable_websocket_controller` | `true` | WebSocketコントローラーの有効/無効 |

## 各起動方法の特徴

### 分離起動の利点
- **デバッグが容易**: 各コンポーネントを個別に停止・再起動可能
- **開発効率**: 一部だけ変更して再起動
- **ログが見やすい**: 各ターミナルで個別のログ

### 統合起動の利点  
- **運用が簡単**: 1コマンドで全システム起動
- **依存関係管理**: 正しい順序で起動
- **自動化対応**: スクリプトや自動起動に適している

## 推奨の使い分け

### 開発段階
```bash
# コード修正 → 部分再起動を繰り返す
ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0
# ↑は起動したまま

# コントローラーだけ再起動
python3 crane_plus_websocket_controller.py  # 修正 → Ctrl+C → 再実行
```

### 本番運用
```bash
# システム起動時に自動実行
ros2 launch crane_plus_examples websocket_demo.launch.py port_name:=/dev/ttyUSB0

# または systemd で自動起動設定
```

### Web アプリケーション統合時
```bash
# ROSシステムのみ起動
ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# WebアプリケーションはFlask/Django等で独自実装
# controller.pyの機能を参考にWebAPI化
```

## トラブルシューティング

### 起動順序の問題
```bash
# エラー: [ERROR] Failed to connect to WebSocket
# 解決: ROSBridgeの起動を待ってからコントローラー起動

# 手動で順序制御
ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0
sleep 5  # 5秒待機
ros2 launch rosbridge_server rosbridge_websocket_launch.xml  
sleep 3  # 3秒待機
python3 crane_plus_websocket_controller.py
```

### ポート競合
```bash
# エラー: [ERROR] Address already in use
# 解決: 異なるポート使用
ros2 launch crane_plus_examples websocket_demo.launch.py \
    rosbridge_port:=9091
```

### 権限エラー
```bash
# エラー: [ERROR] Permission denied: '/dev/ttyUSB0'
# 解決: ユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER
# ログアウト・ログインが必要
```

## 実際のプロジェクトでの推奨構成

### 最小構成（学習用）
```bash
ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0
python3 crane_plus_websocket_controller.py  # 別ターミナル
```

### 開発構成
```bash
ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
python3 crane_plus_websocket_subscriber.py  # 監視用
python3 your_custom_controller.py          # 開発中のコード
```

### 本番構成
```bash
ros2 launch crane_plus_examples websocket_demo.launch.py port_name:=/dev/ttyUSB0
# または独自のlaunchファイルで全統合
```