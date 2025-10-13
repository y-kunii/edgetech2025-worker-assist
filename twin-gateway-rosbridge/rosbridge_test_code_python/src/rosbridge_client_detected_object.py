#!/usr/bin/env python3
# ============================================
# rosbridge_client_detected_object.py
# - twin_bridge/msg/DetectedObject を Publish
# - カメラ検知結果をROS2へ送信
# ============================================

import os
import json
import time
import websocket  # pip install websocket-client
import rel  # websocket-client の軽量イベントループ

ROSBRIDGE_URL = "ws://localhost:9090"
TOPIC = "/detected_object"
TYPE = "twin_bridge/msg/DetectedObject"

INTERVAL = 1.0  # 送信周期 [s]

def on_message(ws, message):
    print("[recv]", message, flush=True)

def send_periodic(ws):
    # 検知結果を模擬生成
    detected_obj = {
        "header": {
            "stamp": {"sec": int(time.time()), "nanosec": 0},
            "frame_id": "camera_link"
        },
        "id": 1,
        "name": "person",
        "pose": {
            "position": {"x": 1.23, "y": 0.45, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.7, "w": 0.7}
        },
        "confidence": 0.95
    }

    pub = {
        "op": "publish",
        "topic": TOPIC,
        "type": TYPE,
        "msg": detected_obj
    }

    ws.send(json.dumps(pub))
    print("[send]", json.dumps(pub, indent=2), flush=True)

    rel.timeout(INTERVAL, send_periodic, ws)

def on_open(ws):
    print("[open] connection established", flush=True)
    rel.timeout(0, send_periodic, ws)

def main():
    url = ROSBRIDGE_URL
    if not (url.startswith("ws://") or url.startswith("wss://")):
        url = "ws://" + url
    print(f"[client] connecting to {url} ...", flush=True)

    ws = websocket.WebSocketApp(
        url,
        on_message=on_message,
        on_open=on_open,
    )

    ws.run_forever(dispatcher=rel)
    rel.signal(2, rel.abort)
    rel.signal(15, rel.abort)
    rel.dispatch()

if __name__ == "__main__":
    main()
