#!/usr/bin/env python3
"""
CRANE V2+ WebSocket Controller
WebSocketを使用したCRANE V2+の統合制御例
PublisherとSubscriberを組み合わせた実用的な制御システム
"""

import json
import time
import math
import threading
import websocket
import rel

# ROSBridge設定
ROSBRIDGE_URL = "ws://localhost:9090"

# トピック定義
JOINT_STATES_TOPIC = "/joint_states"
ARM_COMMAND_TOPIC = "/crane_plus_arm_controller/joint_trajectory"
GRIPPER_COMMAND_TOPIC = "/crane_plus_gripper_controller/joint_trajectory"

# メッセージタイプ
JOINT_STATES_TYPE = "sensor_msgs/msg/JointState"
TRAJECTORY_TYPE = "trajectory_msgs/msg/JointTrajectory"

class CranePlusController:
    def __init__(self, url=ROSBRIDGE_URL):
        self.url = url if url.startswith("ws") else f"ws://{url}"
        self.ws = None
        self.connected = False
        
        # ロボット状態
        self.current_joint_positions = {}
        self.target_positions = {}
        
        # 制御シーケンス
        self.sequence_running = False
        self.sequence_step = 0
        
        # 予定義ポーズ
        self.predefined_poses = {
            "home": [0.0, 0.0, 0.0, 0.0],
            "ready": [0.0, -0.5, 0.5, 0.0],
            "pickup": [0.0, -0.8, 0.8, 0.0],
            "place": [1.57, -0.5, 0.5, 0.0],  # 90度回転
            "inspect": [0.0, -0.3, 0.3, -0.5]
        }
        
        self.gripper_poses = {
            "open": 0.5,
            "close": -0.5,
            "neutral": 0.0
        }
        
    def on_message(self, ws, message):
        """メッセージ受信処理"""
        try:
            data = json.loads(message)
            topic = data.get("topic")
            
            if topic == JOINT_STATES_TOPIC:
                self.handle_joint_states(data)
                
        except json.JSONDecodeError:
            pass
        except Exception as e:
            print(f"[error] Message handling: {e}")
            
    def handle_joint_states(self, data):
        """ジョイント状態を更新"""
        msg = data.get("msg", {})
        joint_names = msg.get("name", [])
        positions = msg.get("position", [])
        
        for name, pos in zip(joint_names, positions):
            if "crane_plus_joint" in name:
                self.current_joint_positions[name] = pos
                
        # 現在位置を定期的に表示
        if hasattr(self, '_last_status_time'):
            if time.time() - self._last_status_time > 2.0:
                self.print_current_status()
                self._last_status_time = time.time()
        else:
            self._last_status_time = time.time()
            
    def print_current_status(self):
        """現在のロボット状態を表示"""
        print("\n=== CRANE V2+ Current Status ===")
        arm_joints = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
        
        for joint in arm_joints:
            if joint in self.current_joint_positions:
                pos_deg = self.current_joint_positions[joint] * 180.0 / math.pi
                print(f"{joint}: {pos_deg:6.1f}°")
                
        if "crane_plus_joint_hand" in self.current_joint_positions:
            gripper_deg = self.current_joint_positions["crane_plus_joint_hand"] * 180.0 / math.pi
            print(f"Gripper: {gripper_deg:6.1f}°")
            
        print("=" * 35)
        
    def on_open(self, ws):
        """接続開始時の処理"""
        print("[open] WebSocket connection established")
        self.connected = True
        
        # トピック購読
        self.subscribe_joint_states()
        
        # トピック広告
        self.advertise_control_topics()
        
        # コントロールメニュー表示
        rel.timeout(2.0, self.show_control_menu)
        
    def subscribe_joint_states(self):
        """ジョイント状態を購読"""
        subscribe_msg = {
            "op": "subscribe",
            "topic": JOINT_STATES_TOPIC,
            "type": JOINT_STATES_TYPE
        }
        self.ws.send(json.dumps(subscribe_msg))
        print("[subscribe] Joint states monitoring started")
        
    def advertise_control_topics(self):
        """制御トピックを広告"""
        topics = [
            {"topic": ARM_COMMAND_TOPIC, "type": TRAJECTORY_TYPE},
            {"topic": GRIPPER_COMMAND_TOPIC, "type": TRAJECTORY_TYPE}
        ]
        
        for topic_info in topics:
            advertise_msg = {
                "op": "advertise",
                "topic": topic_info["topic"],
                "type": topic_info["type"]
            }
            self.ws.send(json.dumps(advertise_msg))
            
        print("[advertise] Control topics ready")
        
    def show_control_menu(self):
        """制御メニューを表示"""
        print("\n" + "="*50)
        print("🤖 CRANE V2+ WebSocket Controller Menu")
        print("="*50)
        print("Predefined Poses:")
        for i, (name, pose) in enumerate(self.predefined_poses.items(), 1):
            pose_str = [f"{p*180/math.pi:.0f}°" for p in pose]
            print(f"  {i}. {name.upper()}: {pose_str}")
            
        print("\nGripper Control:")
        print("  6. Open Gripper")
        print("  7. Close Gripper") 
        print("  8. Neutral Gripper")
        
        print("\nSequences:")
        print("  9. Pick and Place Demo")
        print("  0. Home Position")
        
        print("\nPress Ctrl+C to exit")
        print("="*50)
        
        # ユーザー入力を別スレッドで処理
        threading.Thread(target=self.input_handler, daemon=True).start()
        
    def input_handler(self):
        """ユーザー入力処理"""
        while self.connected:
            try:
                choice = input("\nEnter choice (1-9, 0): ").strip()
                
                if choice in ['1', '2', '3', '4', '5']:
                    pose_names = list(self.predefined_poses.keys())
                    pose_name = pose_names[int(choice)-1]
                    self.move_to_pose(pose_name)
                    
                elif choice == '6':
                    self.control_gripper("open")
                elif choice == '7':
                    self.control_gripper("close")
                elif choice == '8':
                    self.control_gripper("neutral")
                    
                elif choice == '9':
                    self.start_pick_and_place_demo()
                elif choice == '0':
                    self.move_to_pose("home")
                    
                else:
                    print("Invalid choice. Please enter 1-9 or 0.")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Input error: {e}")
                
    def move_to_pose(self, pose_name):
        """指定されたポーズに移動"""
        if pose_name not in self.predefined_poses:
            print(f"Unknown pose: {pose_name}")
            return
            
        target_positions = self.predefined_poses[pose_name]
        self.send_arm_trajectory(target_positions, duration=3.0)
        
        pose_str = [f"{p*180/math.pi:.0f}°" for p in target_positions]
        print(f"[command] Moving to {pose_name.upper()}: {pose_str}")
        
    def control_gripper(self, action):
        """グリッパー制御"""
        if action not in self.gripper_poses:
            print(f"Unknown gripper action: {action}")
            return
            
        target_position = self.gripper_poses[action]
        self.send_gripper_trajectory([target_position], duration=2.0)
        
        print(f"[command] Gripper {action.upper()}: {target_position*180/math.pi:.0f}°")
        
    def send_arm_trajectory(self, positions, duration=3.0):
        """アームトラジェクトリ送信"""
        joint_names = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
        trajectory_msg = self.create_trajectory_msg(joint_names, positions, duration)
        
        publish_msg = {
            "op": "publish",
            "topic": ARM_COMMAND_TOPIC,
            "msg": trajectory_msg
        }
        
        self.ws.send(json.dumps(publish_msg))
        
    def send_gripper_trajectory(self, positions, duration=2.0):
        """グリッパートラジェクトリ送信"""
        joint_names = ["crane_plus_joint_hand"]
        trajectory_msg = self.create_trajectory_msg(joint_names, positions, duration)
        
        publish_msg = {
            "op": "publish", 
            "topic": GRIPPER_COMMAND_TOPIC,
            "msg": trajectory_msg
        }
        
        self.ws.send(json.dumps(publish_msg))
        
    def create_trajectory_msg(self, joint_names, positions, duration):
        """JointTrajectoryメッセージ作成"""
        current_time = time.time()
        sec = int(current_time)
        nanosec = int((current_time - sec) * 1e9)
        
        duration_sec = int(duration)
        duration_nanosec = int((duration - duration_sec) * 1e9)
        
        return {
            "header": {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "frame_id": ""
            },
            "joint_names": joint_names,
            "points": [{
                "positions": positions,
                "velocities": [0.0] * len(positions),
                "accelerations": [0.0] * len(positions), 
                "effort": [0.0] * len(positions),
                "time_from_start": {"sec": duration_sec, "nanosec": duration_nanosec}
            }]
        }
        
    def start_pick_and_place_demo(self):
        """ピックアンドプレースデモ開始"""
        if self.sequence_running:
            print("[demo] Demo already running!")
            return
            
        print("[demo] Starting Pick and Place Demo...")
        self.sequence_running = True
        self.sequence_step = 0
        
        rel.timeout(1.0, self.pick_and_place_sequence)
        
    def pick_and_place_sequence(self):
        """ピックアンドプレースシーケンス"""
        if not self.sequence_running:
            return
            
        sequences = [
            ("home", "open", "Starting from home position"),
            ("ready", "open", "Moving to ready position"),
            ("pickup", "open", "Moving to pickup position"),
            ("pickup", "close", "Picking up object"),
            ("ready", "close", "Lifting object"),
            ("place", "close", "Moving to place position"),
            ("place", "open", "Placing object"),
            ("ready", "open", "Moving back to ready"),
            ("home", "neutral", "Returning to home")
        ]
        
        if self.sequence_step < len(sequences):
            arm_pose, gripper_action, description = sequences[self.sequence_step]
            
            print(f"[demo step {self.sequence_step+1}/{len(sequences)}] {description}")
            
            # アーム移動
            self.move_to_pose(arm_pose)
            
            # グリッパー制御（少し遅延）
            rel.timeout(1.0, lambda: self.control_gripper(gripper_action))
            
            self.sequence_step += 1
            
            # 次のステップ
            rel.timeout(4.0, self.pick_and_place_sequence)
        else:
            print("[demo] Pick and Place Demo completed!")
            self.sequence_running = False
            self.sequence_step = 0
            
    def run(self):
        """WebSocket接続開始"""
        print(f"[client] Connecting to {self.url}...")
        print("🤖 CRANE V2+ WebSocket Controller Starting...")
        
        self.ws = websocket.WebSocketApp(
            self.url,
            on_message=self.on_message,
            on_open=self.on_open
        )
        
        self.ws.run_forever(dispatcher=rel)
        rel.signal(2, rel.abort)
        rel.signal(15, rel.abort)
        rel.dispatch()


def main():
    """メイン関数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="CRANE V2+ WebSocket Controller")
    parser.add_argument("--url", default=ROSBRIDGE_URL,
                       help="ROSBridge WebSocket URL")
    
    args = parser.parse_args()
    
    controller = CranePlusController(args.url)
    controller.run()


if __name__ == "__main__":
    main()