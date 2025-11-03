import json
import os
import threading
import time
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RosbridgeClient:
    """
    Minimal WebSocket client for rosbridge.
    - Connects to ws URL
    - Reconnects on failure
    - Queues outgoing messages (JSON dict)
    - Provides advertise() and publish() helpers
    Note: Requires `websocket-client` (pip install websocket-client).
    """

    def __init__(self, url: str):
        self.url = url
        self._q: Queue = Queue()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._ws = None
        self._connected = threading.Event()
        self._pending_advertise: list[dict] = []
        self._subs: dict[str, dict] = {}

        # Try import once and keep reference
        try:
            import websocket  # type: ignore
            self._ws_module = websocket
            # Keep a reference to the timeout exception class (varies by version)
            try:
                self._ws_timeout_exc = websocket.WebSocketTimeoutException
            except Exception:
                self._ws_timeout_exc = TimeoutError  # fallback
        except Exception as e:  # pragma: no cover
            self._ws_module = None
            print(f"[rosbridge] websocket-client not available: {e}")

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="rosbridge-ws", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        try:
            if self._ws is not None:
                self._ws.close()
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=3)

    def advertise(self, topic: str, msg_type: str):
        obj = {"op": "advertise", "topic": topic, "type": msg_type}
        # Queue now; if not connected yet, keep a copy to resend after connect
        self._pending_advertise.append(obj)
        self.send(obj)

    def publish(self, topic: str, msg: dict):
        self.send({"op": "publish", "topic": topic, "msg": msg})

    def subscribe(self, topic: str, msg_type: str, callback):
        # Store for re-subscription and callbacks
        self._subs[topic] = {"type": msg_type, "cb": callback}
        self.send({"op": "subscribe", "topic": topic, "type": msg_type})

    def send(self, obj: dict):
        try:
            self._q.put_nowait(json.dumps(obj))
        except Exception as e:
            print(f"[rosbridge] enqueue failed: {e}")

    def _run(self):
        if self._ws_module is None:
            print("[rosbridge] websocket-client is required. Install with: pip install websocket-client")
            return
        backoff = 1.0
        while not self._stop.is_set():
            try:
                self._connected.clear()
                self._ws = self._ws_module.create_connection(self.url, timeout=5)
                print(f"[rosbridge] connected: {self.url}")
                self._connected.set()
                # Re-send pending advertises on each (re)connect
                for adv in self._pending_advertise:
                    try:
                        self._ws.send(json.dumps(adv))
                    except Exception:
                        pass
                # Re-subscribe existing subscriptions on reconnect
                for t, meta in self._subs.items():
                    try:
                        self._ws.send(json.dumps({"op": "subscribe", "topic": t, "type": meta["type"]}))
                    except Exception:
                        pass
                # Main loop: interleave send and receive
                backoff = 1.0
                # non-blocking-ish receive (short timeout, treat timeout as normal idle)
                try:
                    self._ws.settimeout(0.1)
                except Exception:
                    pass
                while not self._stop.is_set():
                    # Try to send one item if available
                    try:
                        payload = self._q.get_nowait()
                        try:
                            self._ws.send(payload)
                        except Exception as e:
                            print(f"[rosbridge] send failed: {e}")
                            break
                    except Empty:
                        pass
                    # Try to receive one message
                    try:
                        incoming = self._ws.recv()
                        if not incoming:
                            continue
                        try:
                            obj = json.loads(incoming)
                        except Exception:
                            continue
                        if isinstance(obj, dict) and obj.get("op") == "publish":
                            topic = obj.get("topic")
                            msg = obj.get("msg")
                            meta = self._subs.get(topic)
                            if meta and callable(meta.get("cb")):
                                try:
                                    meta["cb"](topic, msg)
                                except Exception:
                                    pass
                    except Exception as e:
                        # Treat WebSocket recv timeout as normal idle, not a disconnect
                        if hasattr(self, '_ws_timeout_exc') and isinstance(e, self._ws_timeout_exc):
                            continue
                        # other recv errors cause reconnect
                        break
            except Exception as e:
                if not self._stop.is_set():
                    print(f"[rosbridge] connect failed: {e}. retrying in {backoff:.1f}s")
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 10)
            finally:
                try:
                    if self._ws is not None:
                        self._ws.close()
                except Exception:
                    pass
                self._ws = None


class StatusBridge(Node):
    def __init__(self, ws_url: str, rosbridge_msg_type: str):
        super().__init__('status_bridge')
        self.ws = RosbridgeClient(ws_url)
        self.ws.start()

        # Topics
        self.operating_topic = '/operating_status_topic'
        self.gripper_topic = '/gripper_status_topic'
        self.pick_topic = '/pickand_place_topic'
        # WS-side mapped topic names (avoid loops)
        self.ws_operating_topic = '/ws/operating_status_topic'
        self.ws_gripper_topic = '/ws/gripper_status_topic'
        self.ws_pick_topic = '/ws/pickand_place_topic'
        self.msg_type = rosbridge_msg_type
        # Echo suppression for ROS->WS feedback loop (rosbridge re-publishes to ROS)
        self._echo_guard = {}
        try:
            self._echo_ttl = float(os.environ.get('BRIDGE_ECHO_TTL_SEC', '0.75'))
        except Exception:
            self._echo_ttl = 0.75

        # ROS2: subscriptions to forward to rosbridge (ROS2 -> WS)
        self.operating_subscription = self.create_subscription(
            String, self.operating_topic, self._operating_cb, 10
        )
        self.gripper_subscription = self.create_subscription(
            String, self.gripper_topic, self._gripper_cb, 10
        )
        # ROS2: publisher to emit messages coming from WS (WS -> ROS2)
        self.pick_publisher = self.create_publisher(String, self.pick_topic, 10)
        # queue + timer to publish from ROS executor thread
        self._pick_in_q: Queue = Queue()
        self._pick_timer = self.create_timer(0.05, self._drain_pick_queue)

        # rosbridge: advertise WS-mapped status topics (ROS2 -> WS)
        self.ws.advertise(self.ws_operating_topic, self.msg_type)
        self.ws.advertise(self.ws_gripper_topic, self.msg_type)
        # rosbridge: subscribe WS-mapped pick topic (WS -> ROS2)
        self.ws.subscribe(self.ws_pick_topic, self.msg_type, self._ws_pick_cb)

    # ROS -> rosbridge
    def _operating_cb(self, msg: String):
        self.get_logger().info(f'[Operating Status] {msg.data}')
        if self._is_echo(self.ws_operating_topic, msg.data):
            return
        self._mark_sent(self.ws_operating_topic, msg.data)
        self.ws.publish(self.ws_operating_topic, {"data": msg.data})

    def _gripper_cb(self, msg: String):
        self.get_logger().info(f'[Gripper Status] {msg.data}')
        if self._is_echo(self.ws_gripper_topic, msg.data):
            return
        self._mark_sent(self.ws_gripper_topic, msg.data)
        self.ws.publish(self.ws_gripper_topic, {"data": msg.data})

    # WS -> ROS2 (enqueue, published by timer)
    def _ws_pick_cb(self, topic: str, msg_obj: dict):
        try:
            data = msg_obj.get('data') if isinstance(msg_obj, dict) else None
            if data is not None:
                self._pick_in_q.put_nowait(str(data))
        except Exception:
            pass

    def _drain_pick_queue(self):
        published = 0
        while True:
            try:
                data = self._pick_in_q.get_nowait()
            except Empty:
                break
            msg = String()
            msg.data = data
            self.pick_publisher.publish(msg)
            published += 1
        if published:
            self.get_logger().info(f'[Publish pickand_place_topic from WS] x{published}')

    # --- Echo suppression helpers (ROS -> WS) ---
    def _is_echo(self, topic: str, data: str) -> bool:
        try:
            prev_data, prev_t = self._echo_guard.get(topic, (None, 0.0))
            if prev_data is None:
                return False
            return (prev_data == data) and ((time.monotonic() - prev_t) < self._echo_ttl)
        except Exception:
            return False

    def _mark_sent(self, topic: str, data: str) -> None:
        try:
            self._echo_guard[topic] = (data, time.monotonic())
        except Exception:
            pass

    def destroy_node(self):
        try:
            self.ws.stop()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    ws_url = os.environ.get('ROSBRIDGE_URL', 'ws://localhost:9090')
    # For ROS2 rosbridge, types look like 'std_msgs/msg/String'; for ROS1 it's 'std_msgs/String'.
    msg_type = os.environ.get('ROSBRIDGE_MSG_TYPE', 'std_msgs/msg/String')
    node = StatusBridge(ws_url, msg_type)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
