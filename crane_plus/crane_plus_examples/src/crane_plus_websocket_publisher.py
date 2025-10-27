#!/usr/bin/env python3
"""
CRANE V2+ WebSocket Publisher
WebSocketを使用してCRANE V2+のジョイント状態やトラジェクトリ制御コマンドを送信
"""

import json
import time
import math
import websocket
import rel

# ROSBridge設定
ROSBRIDGE_URL = "ws://localhost:9090"

# CRANE V2+トピック定義
JOINT_STATES_TOPIC = "/joint_states"
ARM_TRAJECTORY_TOPIC = "/crane_plus_arm_controller/joint_trajectory"
GRIPPER_TRAJECTORY_TOPIC = "/crane_plus_gripper_controller/joint_trajectory"

# メッセージタイプ
JOINT_STATES_TYPE = "sensor_msgs/msg/JointState"
TRAJECTORY_TYPE = "trajectory_msgs/msg/JointTrajectory"

# CRANE V2+ジョイント名
ARM_JOINTS = [
    "crane_plus_joint1",
    "crane_plus_joint2", 
    "crane_plus_joint3",
    "crane_plus_joint4"
]

GRIPPER_JOINTS = ["crane_plus_joint_hand"]

class CranePlusWebSocketPublisher:
    def __init__(self, url=ROSBRIDGE_URL):
        self.url = url if url.startswith("ws") else f"ws://{url}"
        self.ws = None
        self.connected = False
        
    def on_message(self, ws, message):
        print(f"[recv] {message}", flush=True)
        
    def on_open(self, ws):
        print("[open] WebSocket connection established", flush=True)
        self.connected = True
        
        # トピックの広告（advertise）
        self.advertise_topics()
        
        # 定期的な送信開始
        rel.timeout(1.0, self.send_periodic_joint_states)
        rel.timeout(2.0, self.send_arm_trajectory_example)
        rel.timeout(4.0, self.send_gripper_control_example)
        
    def on_error(self, ws, error):
        print(f"[error] {error}", flush=True)
        
    def on_close(self, ws, close_status_code, close_msg):
        print("[close] WebSocket connection closed", flush=True)
        self.connected = False
        
    def advertise_topics(self):
        """トピックを広告"""
        topics = [
            {
                "op": "advertise",
                "topic": JOINT_STATES_TOPIC,
                "type": JOINT_STATES_TYPE
            },
            {
                "op": "advertise", 
                "topic": ARM_TRAJECTORY_TOPIC,
                "type": TRAJECTORY_TYPE
            },
            {
                "op": "advertise",
                "topic": GRIPPER_TRAJECTORY_TOPIC, 
                "type": TRAJECTORY_TYPE
            }
        ]
        
        for topic_msg in topics:
            self.ws.send(json.dumps(topic_msg))
            print(f"[advertise] {topic_msg['topic']}", flush=True)
            
    def create_joint_states_msg(self, positions):
        """JointStateメッセージを作成"""
        current_time = time.time()
        sec = int(current_time)
        nanosec = int((current_time - sec) * 1e9)
        
        return {
            "header": {
                "stamp": {
                    "sec": sec,
                    "nanosec": nanosec
                },
                "frame_id": ""
            },
            "name": ARM_JOINTS + GRIPPER_JOINTS,
            "position": positions,
            "velocity": [0.0] * len(positions),
            "effort": [0.0] * len(positions)
        }
        
    def create_trajectory_msg(self, joint_names, positions, duration_sec=2.0):
        """JointTrajectoryメッセージを作成"""
        current_time = time.time()
        sec = int(current_time)
        nanosec = int((current_time - sec) * 1e9)
        
        duration_sec_int = int(duration_sec)
        duration_nanosec = int((duration_sec - duration_sec_int) * 1e9)
        
        return {
            "header": {
                "stamp": {
                    "sec": sec,
                    "nanosec": nanosec
                },
                "frame_id": ""
            },
            "joint_names": joint_names,
            "points": [
                {
                    "positions": positions,
                    "velocities": [0.0] * len(positions),
                    "accelerations": [0.0] * len(positions),
                    "effort": [0.0] * len(positions),
                    "time_from_start": {
                        "sec": duration_sec_int,
                        "nanosec": duration_nanosec
                    }
                }
            ]
        }
        
    def send_periodic_joint_states(self):
        """定期的にジョイント状態を送信"""
        if not self.connected:
            return
            
        # サイン波でジョイント位置をシミュレート
        t = time.time()
        arm_positions = [
            math.sin(t * 0.5) * 0.5,      # joint1
            math.sin(t * 0.3) * 0.3,      # joint2
            math.sin(t * 0.4) * 0.4,      # joint3
            math.sin(t * 0.6) * 0.2       # joint4
        ]
        gripper_position = [math.sin(t * 0.2) * 0.1]  # gripper
        
        all_positions = arm_positions + gripper_position
        
        joint_state_msg = self.create_joint_states_msg(all_positions)
        
        publish_msg = {
            "op": "publish",
            "topic": JOINT_STATES_TOPIC,
            "msg": joint_state_msg
        }
        
        self.ws.send(json.dumps(publish_msg))
        print(f"[send] Joint states: {[f'{p:.3f}' for p in all_positions]}", flush=True)
        
        # 次の送信をスケジュール
        rel.timeout(0.1, self.send_periodic_joint_states)
        
    def send_arm_trajectory_example(self):
        """アーム制御トラジェクトリの例を送信"""
        if not self.connected:
            return
            
        # ホームポジションから45度回転
        target_positions = [
            math.radians(45),   # joint1: 45度
            math.radians(30),   # joint2: 30度  
            math.radians(-30),  # joint3: -30度
            math.radians(45)    # joint4: 45度
        ]
        
        trajectory_msg = self.create_trajectory_msg(ARM_JOINTS, target_positions, 3.0)
        
        publish_msg = {
            "op": "publish",
            "topic": ARM_TRAJECTORY_TOPIC,
            "msg": trajectory_msg
        }
        
        self.ws.send(json.dumps(publish_msg))
        print(f"[send] Arm trajectory to: {[f'{math.degrees(p):.1f}°' for p in target_positions]}", flush=True)
        
        # 6秒後に元の位置に戻す
        rel.timeout(6.0, self.send_arm_home_position)
        
    def send_arm_home_position(self):
        """アームをホームポジションに戻す"""
        if not self.connected:
            return
            
        home_positions = [0.0, 0.0, 0.0, 0.0]  # すべてのジョイントを0度
        
        trajectory_msg = self.create_trajectory_msg(ARM_JOINTS, home_positions, 3.0)
        
        publish_msg = {
            "op": "publish", 
            "topic": ARM_TRAJECTORY_TOPIC,
            "msg": trajectory_msg
        }
        
        self.ws.send(json.dumps(publish_msg))
        print("[send] Arm trajectory to: HOME position", flush=True)
        
    def send_gripper_control_example(self):
        """グリッパー制御の例を送信"""
        if not self.connected:
            return
            
        # グリッパーを開く（30度）
        open_position = [math.radians(30)]
        
        trajectory_msg = self.create_trajectory_msg(GRIPPER_JOINTS, open_position, 2.0)
        
        publish_msg = {
            "op": "publish",
            "topic": GRIPPER_TRAJECTORY_TOPIC, 
            "msg": trajectory_msg
        }
        
        self.ws.send(json.dumps(publish_msg))
        print(f"[send] Gripper open: {math.degrees(open_position[0]):.1f}°", flush=True)
        
        # 3秒後にグリッパーを閉じる
        rel.timeout(3.0, self.send_gripper_close)
        
    def send_gripper_close(self):
        """グリッパーを閉じる"""
        if not self.connected:
            return
            
        close_position = [math.radians(-30)]
        
        trajectory_msg = self.create_trajectory_msg(GRIPPER_JOINTS, close_position, 2.0)
        
        publish_msg = {
            "op": "publish",
            "topic": GRIPPER_TRAJECTORY_TOPIC,
            "msg": trajectory_msg
        }
        
        self.ws.send(json.dumps(publish_msg))
        print(f"[send] Gripper close: {math.degrees(close_position[0]):.1f}°", flush=True)
        
        # 3秒後に再度開く（ループ）
        rel.timeout(3.0, self.send_gripper_control_example)
        
    def run(self):
        """WebSocket接続を開始"""
        print(f"[client] Connecting to {self.url}...", flush=True)
        
        self.ws = websocket.WebSocketApp(
            self.url,
            on_message=self.on_message,
            on_open=self.on_open,
            on_error=self.on_error,
            on_close=self.on_close
        )
        
        self.ws.run_forever(dispatcher=rel)
        rel.signal(2, rel.abort)   # Ctrl+C
        rel.signal(15, rel.abort)  # SIGTERM
        rel.dispatch()


def main():
    """メイン関数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="CRANE V2+ WebSocket Publisher")
    parser.add_argument("--url", default=ROSBRIDGE_URL, 
                       help="ROSBridge WebSocket URL (default: ws://localhost:9090)")
    
    args = parser.parse_args()
    
    publisher = CranePlusWebSocketPublisher(args.url)
    publisher.run()


if __name__ == "__main__":
    main()