import os
import json

import websocket  # pip install websocket-client

# 環境変数で上書き可能
ROSBRIDGE_URL = os.environ.get("ROSBRIDGE_URL", "ws://localhost:9090")
MSG_TYPE = os.environ.get("ROSBRIDGE_MSG_TYPE", "std_msgs/msg/String")


def main():
    url = ROSBRIDGE_URL
    if not (url.startswith("ws://") or url.startswith("wss://")):
        url = "ws://" + url
    print(f"[client] connecting to {url} ...", flush=True)

    payload = {
        "op": "publish",
        "topic": "/ws/pick_and_place_topic",
        "type": MSG_TYPE,
        "msg": {"data": "motion1"},
    }

    ws = None
    try:
        ws = websocket.create_connection(url)
        ws.send(json.dumps(payload))
        print("[send] publish", payload, flush=True)
    except Exception as exc:
        print(f"[error] publish failed: {exc}", flush=True)
    finally:
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
