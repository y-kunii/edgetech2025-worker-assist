#!/usr/bin/env python3
"""
トルクセンサー実装
I2C経由でのトルク値読み取りとボルト締め回数カウント
"""

import time
import threading
import logging
from typing import Optional, Callable, List

# I2C通信ライブラリ（利用可能な場合のみインポート）
try:
    import smbus2 as smbus
    I2C_AVAILABLE = True
except ImportError:
    try:
        import smbus
        I2C_AVAILABLE = True
    except ImportError:
        I2C_AVAILABLE = False
        print("Warning: smbus not available. Using mock implementation.")

class TorqueSensor:
    def __init__(self, i2c_address: int = 0x48, i2c_bus: int = 1, torque_threshold: float = 50.0):
        """
        トルクセンサーの初期化
        
        Args:
            i2c_address: I2Cアドレス
            i2c_bus: I2Cバス番号
            torque_threshold: ボルト締め検出閾値（Nm）
        """
        self.i2c_address = i2c_address
        self.i2c_bus = i2c_bus
        self.torque_threshold = torque_threshold
        self.bolt_count = 0
        self.last_detection_time = 0
        self.debounce_time = 2.0  # 2秒のデバウンス
        self.is_initialized = False
        
        # I2Cバス
        self.bus = None
        
        # トルク値履歴（移動平均用）
        self.torque_history: List[float] = []
        self.history_size = 5
        
        # コールバック関数
        self.on_bolt_detected: Optional[Callable] = None
        
        # ログ設定
        self.logger = logging.getLogger('TorqueSensor')
        
        # I2C初期化
        self.initialize_i2c()
    
    def initialize_i2c(self) -> bool:
        """I2C初期化"""
        if not I2C_AVAILABLE:
            self.logger.warning("I2C not available, using mock implementation")
            self.is_initialized = True  # モック実装として動作
            return True
        
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            
            # センサーの初期化コマンド（センサー固有）
            # 例：設定レジスタへの書き込み
            # self.bus.write_byte_data(self.i2c_address, 0x00, 0x01)
            
            # テスト読み取り
            test_value = self.read_raw_torque()
            if test_value is not None:
                self.is_initialized = True
                self.logger.info(f"Torque sensor initialized on I2C address 0x{self.i2c_address:02X}")
                return True
            else:
                self.logger.error("Failed to read test value from torque sensor")
                return False
                
        except Exception as e:
            self.logger.error(f"I2C initialization error: {e}")
            return False
    
    def read_raw_torque(self) -> Optional[int]:
        """生のトルク値読み取り"""
        if not I2C_AVAILABLE or not self.is_initialized:
            # モック実装
            import random
            return random.randint(0, 1023)  # 10bit ADC値をシミュレート
        
        try:
            # 16bitデータ読み取り（センサー固有の実装）
            # 例：2バイトのデータを読み取り
            data = self.bus.read_i2c_block_data(self.i2c_address, 0x00, 2)
            raw_value = (data[0] << 8) | data[1]
            return raw_value
            
        except Exception as e:
            self.logger.error(f"I2C read error: {e}")
            return None
    
    def read_torque(self) -> float:
        """
        トルク値の読み取り（Nm単位）
        
        Returns:
            トルク値（Nm）
        """
        raw_value = self.read_raw_torque()
        if raw_value is None:
            return 0.0
        
        try:
            # 生値からトルク値への変換（センサー固有の計算式）
            # 例：10bit ADC、0-100Nmレンジの場合
            torque = (raw_value / 1023.0) * 100.0
            
            # 移動平均でノイズ除去
            self.torque_history.append(torque)
            if len(self.torque_history) > self.history_size:
                self.torque_history.pop(0)
            
            averaged_torque = sum(self.torque_history) / len(self.torque_history)
            return averaged_torque
            
        except Exception as e:
            self.logger.error(f"Torque calculation error: {e}")
            return 0.0
    
    def detect_bolt_tightening(self) -> bool:
        """
        ボルト締め動作の検出
        
        Returns:
            ボルト締めが検出されたかどうか
        """
        torque = self.read_torque()
        current_time = time.time()
        
        # 閾値を超えた場合
        if torque > self.torque_threshold:
            # デバウンス処理
            if current_time - self.last_detection_time >= self.debounce_time:
                self.last_detection_time = current_time
                self.bolt_count += 1
                
                self.logger.info(f"🔧 Bolt tightening detected! Torque: {torque:.1f}Nm, Count: {self.bolt_count}")
                
                # コールバック実行
                if self.on_bolt_detected:
                    try:
                        self.on_bolt_detected(self.bolt_count, torque)
                    except Exception as e:
                        self.logger.error(f"Callback error: {e}")
                
                return True
        
        return False
    
    def get_bolt_count(self) -> int:
        """ボルト締め回数の取得"""
        return self.bolt_count
    
    def reset_count(self):
        """カウントのリセット"""
        old_count = self.bolt_count
        self.bolt_count = 0
        self.logger.info(f"Bolt count reset: {old_count} -> 0")
    
    def set_torque_threshold(self, threshold: float):
        """トルク閾値の設定"""
        old_threshold = self.torque_threshold
        self.torque_threshold = threshold
        self.logger.info(f"Torque threshold changed: {old_threshold:.1f} -> {threshold:.1f} Nm")
    
    def set_callback(self, callback: Callable[[int, float], None]):
        """コールバック関数の設定"""
        self.on_bolt_detected = callback
        self.logger.info("Bolt detection callback set")
    
    def calibrate_zero(self, samples: int = 10) -> float:
        """
        ゼロ点校正
        
        Args:
            samples: 校正用サンプル数
            
        Returns:
            校正されたゼロ点オフセット
        """
        self.logger.info(f"Starting zero calibration with {samples} samples...")
        
        torque_values = []
        for i in range(samples):
            torque = self.read_torque()
            torque_values.append(torque)
            self.logger.debug(f"Calibration sample {i+1}: {torque:.3f} Nm")
            time.sleep(0.1)
        
        zero_offset = sum(torque_values) / len(torque_values)
        self.logger.info(f"Zero calibration completed. Offset: {zero_offset:.3f} Nm")
        
        return zero_offset
    
    def get_sensor_info(self) -> dict:
        """センサー情報の取得"""
        return {
            'i2c_address': f"0x{self.i2c_address:02X}",
            'i2c_bus': self.i2c_bus,
            'torque_threshold': self.torque_threshold,
            'bolt_count': self.bolt_count,
            'debounce_time': self.debounce_time,
            'i2c_available': I2C_AVAILABLE,
            'is_initialized': self.is_initialized,
            'last_detection': self.last_detection_time,
            'current_torque': self.read_torque()
        }
    
    def cleanup(self):
        """リソースのクリーンアップ"""
        if I2C_AVAILABLE and self.bus:
            try:
                # センサーのシャットダウンコマンド（必要に応じて）
                # self.bus.write_byte_data(self.i2c_address, 0x00, 0x00)
                self.bus.close()
                self.logger.info("I2C bus closed")
            except Exception as e:
                self.logger.error(f"I2C cleanup error: {e}")
        
        self.is_initialized = False

class MockTorqueSensor(TorqueSensor):
    """モックトルクセンサー（テスト用）"""
    
    def __init__(self, i2c_address: int = 0x48, i2c_bus: int = 1, torque_threshold: float = 50.0):
        self.i2c_address = i2c_address
        self.i2c_bus = i2c_bus
        self.torque_threshold = torque_threshold
        self.bolt_count = 0
        self.last_detection_time = 0
        self.debounce_time = 2.0
        self.is_initialized = True
        self.torque_history: List[float] = []
        self.history_size = 5
        self.on_bolt_detected: Optional[Callable] = None
        
        # モック用パラメータ
        self.base_torque = 10.0  # ベーストルク値
        self.noise_level = 5.0   # ノイズレベル
        
        # ログ設定
        self.logger = logging.getLogger('MockTorqueSensor')
        self.logger.info("Mock torque sensor initialized")
        
        # 自動検出スレッド開始
        self._start_mock_detection()
    
    def _start_mock_detection(self):
        """モック検出の開始"""
        def mock_detection_loop():
            import random
            while self.is_initialized:
                time.sleep(random.uniform(5, 12))  # 5-12秒間隔
                
                if self.is_initialized and random.random() > 0.4:  # 60%の確率
                    self._simulate_bolt_tightening()
        
        thread = threading.Thread(target=mock_detection_loop)
        thread.daemon = True
        thread.start()
    
    def _simulate_bolt_tightening(self):
        """ボルト締めのシミュレート"""
        import random
        
        # 高トルクをシミュレート
        high_torque = self.torque_threshold + random.uniform(10, 30)
        
        # 一時的に高トルク状態を作る
        original_base = self.base_torque
        self.base_torque = high_torque
        
        # 検出処理
        self.detect_bolt_tightening()
        
        # 元に戻す
        time.sleep(1)
        self.base_torque = original_base
    
    def read_raw_torque(self) -> Optional[int]:
        """モック生トルク値"""
        import random
        # ベーストルクにノイズを加える
        torque = self.base_torque + random.uniform(-self.noise_level, self.noise_level)
        # 0-1023の範囲に正規化
        raw_value = int((torque / 100.0) * 1023)
        return max(0, min(1023, raw_value))
    
    def cleanup(self):
        """モッククリーンアップ"""
        self.is_initialized = False
        self.logger.info("Mock torque sensor cleanup completed")

# ファクトリー関数
def create_torque_sensor(i2c_address: int = 0x48, i2c_bus: int = 1, 
                        torque_threshold: float = 50.0, use_mock: bool = False) -> TorqueSensor:
    """
    トルクセンサーの作成
    
    Args:
        i2c_address: I2Cアドレス
        i2c_bus: I2Cバス番号
        torque_threshold: 検出閾値
        use_mock: モック実装を使用するか
        
    Returns:
        トルクセンサーインスタンス
    """
    if use_mock or not I2C_AVAILABLE:
        return MockTorqueSensor(i2c_address, i2c_bus, torque_threshold)
    else:
        return TorqueSensor(i2c_address, i2c_bus, torque_threshold)

# 使用例
if __name__ == '__main__':
    import signal
    import sys
    
    # ログ設定
    logging.basicConfig(level=logging.INFO)
    
    def signal_handler(signum, frame):
        print("\nShutting down...")
        sensor.cleanup()
        sys.exit(0)
    
    # シグナルハンドラー設定
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # センサー初期化
    sensor = create_torque_sensor(use_mock=True)  # モック使用
    
    # コールバック設定
    def on_bolt_detected(count, torque):
        print(f"Callback: Bolt #{count} detected! Torque: {torque:.1f}Nm")
    
    sensor.set_callback(on_bolt_detected)
    
    print("Torque sensor test started. Press Ctrl+C to stop.")
    print("Sensor info:", sensor.get_sensor_info())
    
    try:
        # メインループ
        while True:
            torque = sensor.read_torque()
            detected = sensor.detect_bolt_tightening()
            status = "DETECTED!" if detected else "monitoring"
            
            print(f"Torque: {torque:6.1f}Nm, Count: {sensor.get_bolt_count()}, Status: {status}")
            time.sleep(1)
            
    except KeyboardInterrupt:
        pass
    finally:
        sensor.cleanup()