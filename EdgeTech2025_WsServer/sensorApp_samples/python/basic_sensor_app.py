#!/usr/bin/env python3
"""
基本的なセンサーデータ送信アプリ（Python版）
Raspberry Pi WebSocketサーバーと連携してセンサーデータを送信
"""

import socketio
import time
import json
import threading
import signal
import sys
import os
from datetime import datetime
import logging
import colorlog

class BasicSensorApp:
    def __init__(self, server_url='ws://localhost:3001'):
        self.server_url = server_url
        self.sio = socketio.Client()
        self.is_connected = False
        self.is_registered = False
        self.running = False
        
        # データ送信スレッド
        self.data_thread = None
        
        # センサーデータの状態
        self.sensor_state = {
            'screw_count': 0,
            'bolt_count': 0,
            'last_worker_status': 'waiting',
            'last_robot_state': 'waiting',
            'last_robot_grip': 'closed'
        }
        
        # ログ設定
        self.setup_logging()
        self.logger.info(f"Initializing sensor app for server: {self.server_url}")
        
        # イベントハンドラー設定
        self.setup_event_handlers()
    
    def setup_logging(self):
        """ログ設定"""
        # カラーログフォーマッター
        formatter = colorlog.ColoredFormatter(
            '%(log_color)s%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
            log_colors={
                'DEBUG': 'cyan',
                'INFO': 'green',
                'WARNING': 'yellow',
                'ERROR': 'red',
                'CRITICAL': 'red,bg_white',
            }
        )
        
        # ハンドラー設定
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)
        
        # ロガー設定
        self.logger = logging.getLogger('SensorApp')
        self.logger.setLevel(logging.INFO)
        self.logger.addHandler(handler)
    
    def setup_event_handlers(self):
        """WebSocketイベントハンドラーの設定"""
        
        @self.sio.event
        def connect():
            self.logger.info("✅ Connected to WebSocket server")
            self.is_connected = True
            
            # センサークライアントとして登録
            self.logger.info("📝 Registering as sensor client...")
            self.sio.emit('register_client', {'client_type': 'sensor'})
        
        @self.sio.event
        def registered(data):
            self.logger.info(f"✅ Registered successfully: {data}")
            self.is_registered = True
            
            # データ送信開始
            self.start_sending_data()
        
        @self.sio.event
        def disconnect():
            self.logger.warning("❌ Disconnected from server")
            self.is_connected = False
            self.is_registered = False
            self.stop_sending_data()
        
        @self.sio.event
        def connect_error(data):
            self.logger.error(f"❌ Connection error: {data}")
        
        @self.sio.event
        def error(data):
            self.logger.error(f"❌ Server error: {data}")
        
        @self.sio.event
        def ping(data):
            self.sio.emit('pong', data)
            self.logger.debug("💓 Heartbeat responded")
    
    def connect(self):
        """WebSocketサーバーに接続"""
        try:
            self.logger.info("Connecting to WebSocket server...")
            self.sio.connect(self.server_url)
        except Exception as e:
            self.logger.error(f"Failed to connect: {e}")
            raise
    
    def start_sending_data(self):
        """データ送信開始"""
        if self.data_thread and self.data_thread.is_alive():
            return
        
        self.logger.info("🚀 Starting sensor data transmission...")
        self.running = True
        self.data_thread = threading.Thread(target=self._data_sender_loop)
        self.data_thread.daemon = True
        self.data_thread.start()
    
    def stop_sending_data(self):
        """データ送信停止"""
        self.running = False
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=2)
        self.logger.info("⏹️ Stopped sensor data transmission")
    
    def _data_sender_loop(self):
        """データ送信ループ"""
        while self.running and self.is_connected and self.is_registered:
            try:
                sensor_data = self.collect_sensor_data()
                self.sio.emit('sensor_data', sensor_data)
                
                self.logger.info(f"📤 Sensor data sent: "
                               f"worker={sensor_data['worker_status']}, "
                               f"robot={sensor_data['robot_status']['state']}, "
                               f"screws={sensor_data['screw_count']}, "
                               f"bolts={sensor_data['bolt_count']}")
                
                time.sleep(1)  # 1秒間隔
                
            except Exception as e:
                self.logger.error(f"Error sending data: {e}")
                time.sleep(1)
    
    def collect_sensor_data(self):
        """センサーデータの収集"""
        # 作業者状態の検出
        worker_status = self.detect_worker_status()
        
        # ロボット状態の取得
        robot_status = self.get_robot_status()
        
        # カウント値の更新
        self.update_counts(worker_status)
        
        return {
            'worker_status': worker_status,
            'robot_status': robot_status,
            'screw_count': self.sensor_state['screw_count'],
            'bolt_count': self.sensor_state['bolt_count'],
            'work_step': worker_status,
            'timestamp': datetime.now().isoformat()
        }
    
    def detect_worker_status(self):
        """作業者状態の検出（モック実装）"""
        import random
        
        states = ['waiting', 'screw_tightening', 'bolt_tightening', 'tool_handover', 'absent']
        
        # 80%の確率で前回と同じ状態を維持
        if random.random() > 0.2:
            return self.sensor_state['last_worker_status']
        
        # 20%の確率で状態変更
        new_status = random.choice(states)
        self.sensor_state['last_worker_status'] = new_status
        
        if new_status != 'waiting':
            self.logger.info(f"👷 Worker status changed to: {new_status}")
        
        return new_status
    
    def get_robot_status(self):
        """ロボット状態の取得（モック実装）"""
        import random
        
        # 90%の確率で前回と同じ状態を維持
        if random.random() > 0.1:
            return {
                'state': self.sensor_state['last_robot_state'],
                'grip': self.sensor_state['last_robot_grip']
            }
        
        # 10%の確率で状態変更
        states = ['waiting', 'operating']
        grips = ['open', 'closed']
        
        self.sensor_state['last_robot_state'] = random.choice(states)
        self.sensor_state['last_robot_grip'] = random.choice(grips)
        
        self.logger.info(f"🤖 Robot status changed: "
                        f"{self.sensor_state['last_robot_state']}, "
                        f"grip: {self.sensor_state['last_robot_grip']}")
        
        return {
            'state': self.sensor_state['last_robot_state'],
            'grip': self.sensor_state['last_robot_grip']
        }
    
    def update_counts(self, worker_status):
        """カウント値の更新"""
        import random
        
        # ネジ締め作業中の場合、5%の確率でカウント増加
        if worker_status == 'screw_tightening' and random.random() > 0.95:
            self.sensor_state['screw_count'] += 1
            self.logger.info(f"🔩 Screw count increased: {self.sensor_state['screw_count']}")
        
        # ボルト締め作業中の場合、3%の確率でカウント増加
        if worker_status == 'bolt_tightening' and random.random() > 0.97:
            self.sensor_state['bolt_count'] += 1
            self.logger.info(f"🔧 Bolt count increased: {self.sensor_state['bolt_count']}")
    
    def disconnect(self):
        """接続切断"""
        self.logger.info("🛑 Stopping sensor app...")
        self.stop_sending_data()
        
        if self.sio.connected:
            self.sio.disconnect()
    
    def run(self):
        """アプリケーション実行"""
        try:
            self.connect()
            self.sio.wait()
        except KeyboardInterrupt:
            self.logger.info("Received keyboard interrupt")
        except Exception as e:
            self.logger.error(f"Application error: {e}")
        finally:
            self.disconnect()

def signal_handler(signum, frame):
    """シグナルハンドラー"""
    print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
    sys.exit(0)

def main():
    """メイン関数"""
    # 環境変数からサーバーURLを取得
    server_url = os.getenv('WEBSOCKET_URL', 'ws://localhost:3001')
    
    print("🚀 Starting Basic Sensor App (Python)")
    print(f"📡 Target server: {server_url}")
    
    # シグナルハンドラー設定
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # センサーアプリ実行
    sensor_app = BasicSensorApp(server_url)
    sensor_app.run()

if __name__ == '__main__':
    main()