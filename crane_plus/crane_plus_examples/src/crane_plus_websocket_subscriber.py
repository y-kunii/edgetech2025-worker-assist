#!/usr/bin/env python3
"""
CRANE V2+ WebSocket Subscriber
WebSocketを使用してCRANE V2+のトピックを購読し、ロボットの状態を監視
"""

import json
import websocket
import rel

# ROSBridge設定
ROSBRIDGE_URL = "ws://localhost:9090"

# CRANE V2+監視トピック
TOPICS_TO_SUBSCRIBE = [
    {
        "topic": "/joint_states",
        "type": "sensor_msgs/msg/JointState",
        "description": "ジョイント状態"
    },
    {
        "topic": "/crane_plus_arm_controller/state",
        "type": "control_msgs/msg/JointTrajectoryControllerState", 
        "description": "アームコントローラー状態"
    },
    {
        "topic": "/crane_plus_gripper_controller/state",
        "type": "control_msgs/msg/JointTrajectoryControllerState",
        "description": "グリッパーコントローラー状態"
    },
    {
        "topic": "/robot_description",
        "type": "std_msgs/msg/String",
        "description": "ロボット記述"
    },
    {
        "topic": "/tf",
        "type": "tf2_msgs/msg/TFMessage", 
        "description": "座標変換"
    },
    {
        "topic": "/tf_static",
        "type": "tf2_msgs/msg/TFMessage",
        "description": "静的座標変換"
    }
]

class CranePlusWebSocketSubscriber:
    def __init__(self, url=ROSBRIDGE_URL):
        self.url = url if url.startswith("ws") else f"ws://{url}"
        self.ws = None
        self.connected = False
        self.message_count = {}
        
    def on_message(self, ws, message):
        """受信メッセージを処理"""
        try:
            data = json.loads(message)
            topic = data.get("topic", "unknown")
            
            # メッセージカウント
            self.message_count[topic] = self.message_count.get(topic, 0) + 1
            
            # トピック別処理
            if topic == "/joint_states":
                self.handle_joint_states(data)
            elif "controller/state" in topic:
                self.handle_controller_state(data) 
            elif topic == "/tf" or topic == "/tf_static":
                self.handle_tf_message(data)
            elif topic == "/robot_description":
                self.handle_robot_description(data)
            else:
                print(f"[recv] {topic}: {self.message_count[topic]} messages received", flush=True)
                
        except json.JSONDecodeError:
            print(f"[error] Failed to parse message: {message[:100]}...", flush=True)
        except Exception as e:
            print(f"[error] Message handling error: {e}", flush=True)
            
    def handle_joint_states(self, data):
        """ジョイント状態メッセージを処理"""
        msg = data.get("msg", {})
        joint_names = msg.get("name", [])
        positions = msg.get("position", [])
        velocities = msg.get("velocity", [])
        
        if len(joint_names) == len(positions):
            print(f"[joint_states #{self.message_count['/joint_states']}]")
            for name, pos, vel in zip(joint_names, positions, velocities or [0]*len(positions)):
                if "crane_plus" in name:
                    degree = pos * 180.0 / 3.14159  # ラジアンから度に変換
                    print(f"  {name}: {degree:6.1f}° (vel: {vel:6.3f})")
            print()
            
    def handle_controller_state(self, data):
        """コントローラー状態メッセージを処理"""
        topic = data.get("topic", "")
        msg = data.get("msg", {})
        
        joint_names = msg.get("joint_names", [])
        actual_positions = msg.get("actual", {}).get("positions", [])
        desired_positions = msg.get("desired", {}).get("positions", [])
        error_positions = msg.get("error", {}).get("positions", [])
        
        controller_type = "ARM" if "arm" in topic else "GRIPPER"
        count = self.message_count[topic]
        
        print(f"[{controller_type}_controller #{count}]")
        for i, name in enumerate(joint_names):
            actual = actual_positions[i] if i < len(actual_positions) else 0.0
            desired = desired_positions[i] if i < len(desired_positions) else 0.0
            error = error_positions[i] if i < len(error_positions) else 0.0
            
            actual_deg = actual * 180.0 / 3.14159
            desired_deg = desired * 180.0 / 3.14159
            error_deg = error * 180.0 / 3.14159
            
            print(f"  {name}:")
            print(f"    actual: {actual_deg:6.1f}°, desired: {desired_deg:6.1f}°, error: {error_deg:6.1f}°")
        print()
        
    def handle_tf_message(self, data):
        """TF（座標変換）メッセージを処理"""
        topic = data.get("topic", "")
        msg = data.get("msg", {})
        transforms = msg.get("transforms", [])
        
        count = self.message_count[topic]
        if count % 10 == 1:  # 10メッセージごとに表示
            print(f"[{topic} #{count}] {len(transforms)} transforms")
            for tf in transforms[:3]:  # 最初の3つだけ表示
                header = tf.get("header", {})
                child_frame = tf.get("child_frame_id", "")
                frame_id = header.get("frame_id", "")
                translation = tf.get("transform", {}).get("translation", {})
                
                if "crane_plus" in child_frame or "base_link" in child_frame:
                    x = translation.get("x", 0.0)
                    y = translation.get("y", 0.0) 
                    z = translation.get("z", 0.0)
                    print(f"  {frame_id} -> {child_frame}: ({x:.3f}, {y:.3f}, {z:.3f})")
            print()
            
    def handle_robot_description(self, data):
        """ロボット記述メッセージを処理"""
        msg = data.get("msg", {})
        description = msg.get("data", "")
        
        count = self.message_count["/robot_description"]
        print(f"[robot_description #{count}] URDF length: {len(description)} chars")
        
        # URDFからジョイント情報を抽出
        if "crane_plus_joint" in description:
            joints = []
            for line in description.split('\n'):
                if 'joint name="crane_plus_joint' in line:
                    start = line.find('name="') + 6
                    end = line.find('"', start)
                    if start > 5 and end > start:
                        joints.append(line[start:end])
            
            if joints:
                print(f"  Found joints: {', '.join(joints)}")
        print()
        
    def on_open(self, ws):
        """WebSocket接続開始時の処理"""
        print("[open] WebSocket connection established", flush=True)
        self.connected = True
        
        # 監視対象トピックを購読
        self.subscribe_topics()
        
    def on_error(self, ws, error):
        """エラー処理"""
        print(f"[error] {error}", flush=True)
        
    def on_close(self, ws, close_status_code, close_msg):
        """接続終了処理"""
        print("[close] WebSocket connection closed", flush=True)
        self.connected = False
        
    def subscribe_topics(self):
        """監視対象トピックを購読"""
        print("[subscribe] Subscribing to CRANE V2+ topics...")
        
        for topic_info in TOPICS_TO_SUBSCRIBE:
            subscribe_msg = {
                "op": "subscribe",
                "topic": topic_info["topic"],
                "type": topic_info["type"]
            }
            
            self.ws.send(json.dumps(subscribe_msg))
            print(f"  ✓ {topic_info['topic']} ({topic_info['description']})")
            
        print()
        
    def print_statistics(self):
        """統計情報を定期的に表示"""
        if not self.connected:
            return
            
        print("[statistics] Message count summary:")
        for topic, count in sorted(self.message_count.items()):
            print(f"  {topic}: {count} messages")
        print()
        
        # 10秒後に再度統計表示
        rel.timeout(10.0, self.print_statistics)
        
    def run(self):
        """WebSocket接続を開始"""
        print(f"[client] Connecting to {self.url}...", flush=True)
        print("Monitoring CRANE V2+ robot topics via WebSocket\n")
        
        self.ws = websocket.WebSocketApp(
            self.url,
            on_message=self.on_message,
            on_open=self.on_open,
            on_error=self.on_error,
            on_close=self.on_close
        )
        
        # 統計表示を開始
        rel.timeout(10.0, self.print_statistics)
        
        self.ws.run_forever(dispatcher=rel)
        rel.signal(2, rel.abort)   # Ctrl+C
        rel.signal(15, rel.abort)  # SIGTERM  
        rel.dispatch()


def main():
    """メイン関数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="CRANE V2+ WebSocket Subscriber")
    parser.add_argument("--url", default=ROSBRIDGE_URL,
                       help="ROSBridge WebSocket URL (default: ws://localhost:9090)")
    parser.add_argument("--topics", nargs="+", 
                       help="Specific topics to subscribe (default: all CRANE V2+ topics)")
    
    args = parser.parse_args()
    
    # 特定のトピックが指定された場合はフィルタ
    if args.topics:
        global TOPICS_TO_SUBSCRIBE
        TOPICS_TO_SUBSCRIBE = [
            t for t in TOPICS_TO_SUBSCRIBE 
            if any(topic in t["topic"] for topic in args.topics)
        ]
    
    subscriber = CranePlusWebSocketSubscriber(args.url)
    subscriber.run()


if __name__ == "__main__":
    main()