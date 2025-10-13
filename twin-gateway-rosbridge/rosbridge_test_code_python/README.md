# rosbridge_test_code_python の使い方

Python から WebSocket 経由で rosbridge に接続し、ROS 2 のトピックを Publish/Subscribe する最小サンプルです。

対象スクリプトは `rosbridge_test_code_python/src` 配下にあります。

--

## 前提
- rosbridge がポート `9090` で起動していること
  - 例（別リポジトリ直下で）:
    - `docker run --rm -it -v "$(pwd)/digital_twin_ws:/workspace/digital_twin_ws" -p 9090:9090 --name twin-gateway-rosbridge twin-gateway-rosbridge:humble`
- 接続先は `ws://localhost:9090` を想定（各スクリプトの `ROSBRIDGE_URL` を編集可）

--

## セットアップ（このディレクトリで実行）
```
# イメージをビルド
docker build -t python-websocket:ws .

# 簡易起動（終了時にコンテナ削除）
docker run --rm -it --name rosbridge-python-client \
  --network host \
  -v "$(pwd)/src:/app/src" \
  python-websocket:ws

# 残す起動（再利用したい場合）
# docker run -it --name rosbridge-python-client \
#   --network host \
#   -v "$(pwd)/src:/app/src" \
#   python-websocket:ws
```

補足
- `--network host` を付けることで、コンテナ内から `ws://localhost:9090` に直接接続できます。
- 作業ディレクトリは `/app/src` です。以降のコマンドはコンテナ内で `/app/src` にいる前提です。

--

## シンプルメッセージ（/simple_topic）

メッセージ型: `twin_bridge/msg/SimpleMsg`（フィールド: `int32 id`, `string text`）。

- Subscribe（受信）
```
python rosbridge_client_subsclibe.py
```

- Publish（送信）
```
python rosbridge_client_publish.py
```

期待される例（Subscribe 側）
```
[client] connecting to ws://localhost:9090 ...
[send] subscribe {'op': 'subscribe', 'topic': '/simple_topic', 'type': 'twin_bridge/msg/SimpleMsg'}
[recv] {"op": "publish", "topic": "/simple_topic", "msg": {"id": 1, "text": "hello twin"}}
```

--

## 検知オブジェクト（/detected_object）

メッセージ型: `twin_bridge/msg/DetectedObject`

- Subscribe（受信）
```
python rosbridge_client_detected_object_sub.py
```

- Publish（送信）
```
python rosbridge_client_detected_object.py
```

Publish スクリプトは 1 秒間隔でモックの検知結果を送信します（frame_id, pose, confidence など）。

--

## 接続先・ポートの変更
- 各スクリプトの先頭にある `ROSBRIDGE_URL = "ws://localhost:9090"` を編集してください。
- 例: 別ホスト上の rosbridge に接続する場合
  - `ROSBRIDGE_URL = "ws://192.168.1.50:9090"`

--

## よくあるトラブルと対処
- rosbridge に接続できない/応答がない
  - rosbridge 側コンテナが起動しているか確認
  - `-p 9090:9090` でポートが公開されているか確認
  - Python クライアント側を `--network host` で起動し、`localhost:9090` に到達できる状態か確認
- メッセージ型不一致エラー
  - twin_bridge パッケージがビルド済みで、rosbridge 側が最新の `install/` を読み込んでいるかを確認

--

## 参考（rosbridge 起動例）
```
docker run --rm -it \
  -v "$(pwd)/digital_twin_ws:/workspace/digital_twin_ws" \
  -p 9090:9090 \
  --name twin-gateway-rosbridge \
  twin-gateway-rosbridge:humble
```

