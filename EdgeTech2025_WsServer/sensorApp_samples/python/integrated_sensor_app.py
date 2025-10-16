#!/usr/bin/env python3
"""
統合センサーアプリ（Python版）
複数のセンサーを統合してWebSocketサーバーにデータを送信
"""

import socketio
import time
import json
import threading
import signal
import sys
import os
import yaml
from datetime import datetime
import logging
import colorlog
from typing import Optional, Dict, Any

# センサーモジュール
from sensors.camera_sensor import CameraSensor
from sensors.vibration_sensor import create_vibration_sensor
from sensors.torque_sensor import create_torque_sensor

class IntegratedSensorApp:
    def __init__(self, config_file: str = 'config.yaml'):
        """
        統合センサーアプリの初期化
        
        Args:
            config_file: 設定ファイルパス
        """
        # 設定読み込み
        self.config = self.load_config(config_file)
        
        # WebSocket設定
        self.server_url = self.config.get('websocket', {}).get('url', 'ws://localhost:3001')
        self.sio = socketio.Client()
        self.is_connected = False
        self.is_registered = False
        self.running = False
        
        # データ送信スレッド
        self.data_thread = None
        self.send_interval = self.config.get('data', {}).get('send_interval', 1.0)
        
        # センサー
        self.camera_sensor: Optional[CameraSensor] = None
        self.vibration_sensor = None
        self.torque_sensor = None
        
        # データ統合
        self.sensor_data = {
            'worker_status': 'waiting',
            'robot_status': {'state': 'waiting', 'grip': 'closed'},
            'screw_count': 0,
            'bolt_count': 0,
            'work_step': 'waiting',
            'image': None,
            'timestamp': None
        }
        
        # ログ設定
        self.setup_logging()
        self.logger.info(f"Initializing integrated sensor app for server: {self.server_url}")
        
        # センサー初期化
        self.initialize_sensors()
        
        # WebSocketイベントハンドラー設定
        self.setup_websocket_handlers()
    
    def load_config(self, config_file: str) -> Dict[str, Any]:
        """設定ファイルの読み込み"""
        default_config = {
            'websocket': {
                'url': os.getenv('WEBSOCKET_URL', 'ws://localhost:3001')
            },
            'data': {
                'send_interval': 1.0,
                'include_image': True,
                'image_quality': 80
            },
            'sensors': {
                'camera': {
                    'enabled': True,
                    'camera_id': 0,
                    'width': 640,
                    'height': 480
                },
                'vibration': {
                    'enabled': True,
                    'pin': 18,
                    'debounce_time': 1.0,
                    'use_mock': True
                },
                'torque': {
                    'enabled': True,
                    'i2c_address': 0x48,
                    'i2c_bus': 1,
                    'threshold': 50.0,
                    'use_mock': True
                }
            },
            'logging': {
                'level': 'INFO'
            }
        }
        
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    user_config = yaml.safe_load(f)
                    # デフォルト設定とマージ
                    self._merge_config(default_config, user_config)
            except Exception as e:
                print(f"Warning: Failed to load config file {config_file}: {e}")
        
        return default_config
    
    def _merge_config(self, default: dict, user: dict):
        """設定のマージ"""
        for key, value in user.items():
            if key in default and isinstance(default[key], dict) and isinstance(value, dict):
                self._merge_config(default[key], value)
            else:
                default[key] = value
    
    def setup_logging(self):
        """ログ設定"""
        log_level = getattr(logging, self.config.get('logging', {}).get('level', 'INFO'))
        
        # カラーログフォーマッター
        formatter = colorlog.ColoredFormatter(
            '%(log_color)s%(asctime)s - %(name)s - %(levelname)s - %(message)s',
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
        self.logger = logging.getLogger('IntegratedSensorApp')
        self.logger.setLevel(log_level)
        self.logger.addHandler(handler)
        
        # 他のロガーのレベルも設定
        logging.getLogger('CameraSensor').setLevel(log_level)
        logging.getLogger('VibrationSensor').setLevel(log_level)
        logging.getLogger('TorqueSensor').setLevel(log_level)
    
    def initialize_sensors(self):
        """センサーの初期化"""
        sensor_config = self.config.get('sensors', {})
        
        # カメラセンサー
        if sensor_config.get('camera', {}).get('enabled', True):
            try:
                camera_config = sensor_config['camera']
                self.camera_sensor = CameraSensor(
                    camera_id=camera_config.get('camera_id', 0),
                    width=camera_config.get('width', 640),
                    height=camera_config.get('height', 480)
                )
                if self.camera_sensor.is_initialized:
                    self.logger.info("✅ Camera sensor initialized")
                else:
                    self.logger.warning("❌ Camera sensor initialization failed")
                    self.camera_sensor = None
            except Exception as e:
                self.logger.error(f"Camera sensor initialization error: {e}")
                self.camera_sensor = None
        
        # 振動センサー
        if sensor_config.get('vibration', {}).get('enabled', True):
            try:
                vibration_config = sensor_config['vibration']
                self.vibration_sensor = create_vibration_sensor(
                    pin=vibration_config.get('pin', 18),
                    debounce_time=vibration_config.get('debounce_time', 1.0),
                    use_mock=vibration_config.get('use_mock', True)
                )
                
                # コールバック設定
                self.vibration_sensor.set_callback(self._on_screw_detected)
                
                if self.vibration_sensor.is_initialized:
                    self.logger.info("✅ Vibration sensor initialized")
                else:
                    self.logger.warning("❌ Vibration sensor initialization failed")
            except Exception as e:
                self.logger.error(f"Vibration sensor initialization error: {e}")
                self.vibration_sensor = None
        
        # トルクセンサー
        if sensor_config.get('torque', {}).get('enabled', True):
            try:
                torque_config = sensor_config['torque']
                self.torque_sensor = create_torque_sensor(
                    i2c_address=torque_config.get('i2c_address', 0x48),
                    i2c_bus=torque_config.get('i2c_bus', 1),
                    torque_threshold=torque_config.get('threshold', 50.0),
                    use_mock=torque_config.get('use_mock', True)
                )
                
                # コールバック設定
                self.torque_sensor.set_callback(self._on_bolt_detected)
                
                if self.torque_sensor.is_initialized:
                    self.logger.info("✅ Torque sensor initialized")
                else:
                    self.logger.warning("❌ Torque sensor initialization failed")
            except Exception as e:
                self.logger.error(f"Torque sensor initialization error: {e}")
                self.torque_sensor = None
    
    def _on_screw_detected(self, count: int):
        """ネジ締め検出コールバック"""
        self.sensor_data['screw_count'] = count
        self.logger.info(f"🔩 Screw detected: {count}")
    
    def _on_bolt_detected(self, count: int, torque: float):
        """ボルト締め検出コールバック"""
        self.sensor_data['bolt_count'] = count
        self.logger.info(f"🔧 Bolt detected: {count} (torque: {torque:.1f}Nm)")
    
    def setup_websocket_handlers(self):
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
            self.start_data_collection()
        
        @self.sio.event
        def disconnect():
            self.logger.warning("❌ Disconnected from server")
            self.is_connected = False
            self.is_registered = False
            self.stop_data_collection()
        
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
    
    def start_data_collection(self):
        """データ収集開始"""
        if self.data_thread and self.data_thread.is_alive():
            return
        
        self.logger.info("🚀 Starting integrated sensor data collection...")
        self.running = True
        self.data_thread = threading.Thread(target=self._data_collection_loop)
        self.data_thread.daemon = True
        self.data_thread.start()
    
    def stop_data_collection(self):
        """データ収集停止"""
        self.running = False
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=2)
        self.logger.info("⏹️ Stopped sensor data collection")
    
    def _data_collection_loop(self):
        """データ収集ループ"""
        while self.running and self.is_connected and self.is_registered:
            try:
                # 各センサーからデータ収集
                sensor_data = self.collect_all_sensor_data()
                
                # WebSocketサーバーに送信
                self.sio.emit('sensor_data', sensor_data)
                
                self.logger.info(f"📤 Integrated data sent: "
                               f"worker={sensor_data['worker_status']}, "
                               f"screws={sensor_data['screw_count']}, "
                               f"bolts={sensor_data['bolt_count']}")
                
                time.sleep(self.send_interval)
                
            except Exception as e:
                self.logger.error(f"Error in data collection loop: {e}")
                time.sleep(1)
    
    def collect_all_sensor_data(self) -> Dict[str, Any]:
        """全センサーからデータ収集"""
        # カメラから作業者状態検出
        if self.camera_sensor and self.camera_sensor.is_initialized:
            try:
                worker_status = self.camera_sensor.detect_worker_status()
                self.sensor_data['worker_status'] = worker_status
                self.sensor_data['work_step'] = worker_status
                
                # 画像データ（オプション）
                if self.config.get('data', {}).get('include_image', True):
                    image_quality = self.config.get('data', {}).get('image_quality', 80)
                    image_data = self.camera_sensor.capture_image_base64(image_quality)
                    self.sensor_data['image'] = image_data
                
            except Exception as e:
                self.logger.error(f"Camera sensor error: {e}")
        
        # 振動センサーからネジ締め回数取得
        if self.vibration_sensor and self.vibration_sensor.is_initialized:
            try:
                self.sensor_data['screw_count'] = self.vibration_sensor.get_screw_count()
            except Exception as e:
                self.logger.error(f"Vibration sensor error: {e}")
        
        # トルクセンサーからボルト締め回数取得
        if self.torque_sensor and self.torque_sensor.is_initialized:
            try:
                self.sensor_data['bolt_count'] = self.torque_sensor.get_bolt_count()
            except Exception as e:
                self.logger.error(f"Torque sensor error: {e}")
        
        # ロボット状態（実際の実装ではロボットAPIから取得）
        self.sensor_data['robot_status'] = self.get_robot_status()
        
        # タイムスタンプ
        self.sensor_data['timestamp'] = datetime.now().isoformat()
        
        return self.sensor_data.copy()
    
    def get_robot_status(self) -> Dict[str, str]:
        """ロボット状態の取得（モック実装）"""
        # 実際の実装では、ロボットAPIから状態を取得
        import random
        
        # 作業者状態に応じてロボット状態を決定
        worker_status = self.sensor_data.get('worker_status', 'waiting')
        
        if worker_status == 'tool_handover':
            return {'state': 'operating', 'grip': 'open'}
        elif worker_status in ['screw_tightening', 'bolt_tightening']:
            # 作業中は時々ロボットも動作
            if random.random() > 0.7:
                return {'state': 'operating', 'grip': 'closed'}
        
        return {'state': 'waiting', 'grip': 'closed'}
    
    def get_system_status(self) -> Dict[str, Any]:
        """システム状態の取得"""
        return {
            'websocket': {
                'connected': self.is_connected,
                'registered': self.is_registered,
                'server_url': self.server_url
            },
            'sensors': {
                'camera': {
                    'enabled': self.camera_sensor is not None,
                    'initialized': self.camera_sensor.is_initialized if self.camera_sensor else False,
                    'info': self.camera_sensor.get_camera_info() if self.camera_sensor else {}
                },
                'vibration': {
                    'enabled': self.vibration_sensor is not None,
                    'initialized': self.vibration_sensor.is_initialized if self.vibration_sensor else False,
                    'info': self.vibration_sensor.get_sensor_info() if self.vibration_sensor else {}
                },
                'torque': {
                    'enabled': self.torque_sensor is not None,
                    'initialized': self.torque_sensor.is_initialized if self.torque_sensor else False,
                    'info': self.torque_sensor.get_sensor_info() if self.torque_sensor else {}
                }
            },
            'data': self.sensor_data.copy()
        }
    
    def cleanup(self):
        """リソースのクリーンアップ"""
        self.logger.info("🛑 Cleaning up integrated sensor app...")
        
        # データ収集停止
        self.stop_data_collection()
        
        # センサークリーンアップ
        if self.camera_sensor:
            self.camera_sensor.release()
        
        if self.vibration_sensor:
            self.vibration_sensor.cleanup()
        
        if self.torque_sensor:
            self.torque_sensor.cleanup()
        
        # WebSocket切断
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
            self.cleanup()

def create_default_config():
    """デフォルト設定ファイルの作成"""
    config = {
        'websocket': {
            'url': 'ws://localhost:3001'
        },
        'data': {
            'send_interval': 1.0,
            'include_image': True,
            'image_quality': 80
        },
        'sensors': {
            'camera': {
                'enabled': True,
                'camera_id': 0,
                'width': 640,
                'height': 480
            },
            'vibration': {
                'enabled': True,
                'pin': 18,
                'debounce_time': 1.0,
                'use_mock': True
            },
            'torque': {
                'enabled': True,
                'i2c_address': 0x48,
                'i2c_bus': 1,
                'threshold': 50.0,
                'use_mock': True
            }
        },
        'logging': {
            'level': 'INFO'
        }
    }
    
    with open('config.yaml', 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    print("Default config file 'config.yaml' created")

def signal_handler(signum, frame):
    """シグナルハンドラー"""
    print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
    sys.exit(0)

def main():
    """メイン関数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Integrated Sensor App')
    parser.add_argument('--config', default='config.yaml', help='Configuration file path')
    parser.add_argument('--create-config', action='store_true', help='Create default configuration file')
    parser.add_argument('--status', action='store_true', help='Show system status and exit')
    
    args = parser.parse_args()
    
    if args.create_config:
        create_default_config()
        return
    
    print("🚀 Starting Integrated Sensor App (Python)")
    print(f"📋 Config file: {args.config}")
    
    # シグナルハンドラー設定
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # アプリ実行
    app = IntegratedSensorApp(args.config)
    
    if args.status:
        status = app.get_system_status()
        print(json.dumps(status, indent=2))
        app.cleanup()
        return
    
    app.run()

if __name__ == '__main__':
    main()