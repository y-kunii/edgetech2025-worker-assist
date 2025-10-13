#!/usr/bin/env python3
# ============================================
# rosbridge_client_detected_object_sub.py
# - twin_bridge/msg/DetectedObject を Subscribe
# - カメラ検知結果を受信して表示
# ============================================

import os
import json
import websocket  # pip install websocket-client
import rel  # websocket-client の軽量イベントループ

ROSBRIDGE_URL = "ws://localhost:9090"
TOPIC = "/detected_object"
TYPE = "twin_bridge/msg/DetectedObject"

def on_message(ws, message):
    try:
        data = json.loads(message)
        if "msg" in data:
            msg = data["msg"]
            print("\n[recv DetectedObject]")
            print(f"  id: {msg.get('id')}")
            print(f"  name: {msg.get('name')}")
            pose = msg.get("pose", {})
            pos = pose.get("position", {})
            ori = pose.get("orientation", {})
            print(f"  position: x={pos.get('x'):.2f}, y={pos.get('y'):.2f}, z={pos.get('z'):.2f}")
            print(f"  orientation: x={ori.get('x'):.2f}, y={ori.get('y'):.2f}, "
                  f"z={ori.get('z'):.2f}, w={ori.get('w'):.2f}")
            print(f"  confidence: {msg.get('confidence'):.2f}")
    except Exception as e:
        print("[error parsing]", e, flush=True)

def on_open(ws):
    print("[open] connection established", flush=True)
    sub = {
        "op": "subscribe",
        "topic": TOPIC,
        "type": TYPE
    }
    ws.send(json.dumps(sub))
    print(f"[send] subscribed to {TOPIC}", flush=True)

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
