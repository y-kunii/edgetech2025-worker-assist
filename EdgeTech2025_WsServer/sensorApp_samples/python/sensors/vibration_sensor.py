#!/usr/bin/env python3
"""
振動センサー実装
GPIO経由での振動検出とネジ締め回数カウント
"""

import time
import threading
import logging
from typing import Optional, Callable

# Raspberry Pi GPIO（利用可能な場合のみインポート）
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: RPi.GPIO not available. Using mock implementation.")

class VibrationSensor:
    def __init__(self, pin: int = 18, debounce_time: float = 1.0):
        """
        振動センサーの初期化
        
        Args:
            pin: GPIO ピン番号（BCMモード）
            debounce_time: デバウンス時間（秒）
        """
        self.pin = pin
        self.debounce_time = debounce_time
        self.screw_count = 0
        self.last_detection_time = 0
        self.is_initialized = False
        
        # コールバック関数
        self.on_screw_detected: Optional[Callable] = None
        
        # ログ設定
        self.logger = logging.getLogger('VibrationSensor')
        
        # GPIO初期化
        self.initialize_gpio()
    
    def initialize_gpio(self) -> bool:
        """GPIO初期化"""
        if not GPIO_AVAILABLE:
            self.logger.warning("GPIO not available, using mock implementation")
            self.is_initialized = True  # モック実装として動作
            return True
        
        try:
            # GPIO設定
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            
            # 割り込み設定
            GPIO.add_event_detect(
                self.pin, 
                GPIO.RISING, 
                callback=self._on_vibration_interrupt,
                bouncetime=int(self.debounce_time * 1000)  # ミリ秒
            )
            
            self.is_initialized = True
            self.logger.info(f"Vibration sensor initialized on GPIO pin {self.pin}")
            return True
            
        except Exception as e:
            self.logger.error(f"GPIO initialization error: {e}")
            return False
    
    def _on_vibration_interrupt(self, channel: int):
        """振動検出割り込みハンドラー"""
        current_time = time.time()
        
        # デバウンス処理
        if current_time - self.last_detection_time < self.debounce_time:
            return
        
        self.last_detection_time = current_time
        self.screw_count += 1
        
        self.logger.info(f"🔩 Vibration detected! Screw count: {self.screw_count}")
        
        # コールバック実行
        if self.on_screw_detected:
            try:
                self.on_screw_detected(self.screw_count)
            except Exception as e:
                self.logger.error(f"Callback error: {e}")
    
    def read_vibration_level(self) -> float:
        """
        振動レベルの読み取り
        
        実際の実装では、アナログセンサーからの値を読み取り
        ADC（MCP3008等）を使用してアナログ値をデジタル変換
        
        Returns:
            振動レベル（0.0-1.0）
        """
        if not GPIO_AVAILABLE:
            # モック実装：ランダムな振動レベル
            import random
            return random.uniform(0.0, 0.3)
        
        try:
            # デジタル入力の場合
            digital_value = GPIO.input(self.pin)
            return 1.0 if digital_value else 0.0
            
        except Exception as e:
            self.logger.error(f"Vibration level read error: {e}")
            return 0.0
    
    def detect_screw_tightening(self, threshold: float = 0.5) -> bool:
        """
        ネジ締め動作の検出
        
        Args:
            threshold: 検出閾値
            
        Returns:
            ネジ締めが検出されたかどうか
        """
        vibration_level = self.read_vibration_level()
        
        if vibration_level > threshold:
            current_time = time.time()
            
            # デバウンス処理
            if current_time - self.last_detection_time >= self.debounce_time:
                self.last_detection_time = current_time
                self.screw_count += 1
                
                self.logger.info(f"🔩 Screw tightening detected! Count: {self.screw_count}")
                
                # コールバック実行
                if self.on_screw_detected:
                    try:
                        self.on_screw_detected(self.screw_count)
                    except Exception as e:
                        self.logger.error(f"Callback error: {e}")
                
                return True
        
        return False
    
    def get_screw_count(self) -> int:
        """ネジ締め回数の取得"""
        return self.screw_count
    
    def reset_count(self):
        """カウントのリセット"""
        old_count = self.screw_count
        self.screw_count = 0
        self.logger.info(f"Screw count reset: {old_count} -> 0")
    
    def set_callback(self, callback: Callable[[int], None]):
        """コールバック関数の設定"""
        self.on_screw_detected = callback
        self.logger.info("Screw detection callback set")
    
    def get_sensor_info(self) -> dict:
        """センサー情報の取得"""
        return {
            'pin': self.pin,
            'debounce_time': self.debounce_time,
            'screw_count': self.screw_count,
            'gpio_available': GPIO_AVAILABLE,
            'is_initialized': self.is_initialized,
            'last_detection': self.last_detection_time
        }
    
    def cleanup(self):
        """リソースのクリーンアップ"""
        if GPIO_AVAILABLE and self.is_initialized:
            try:
                GPIO.remove_event_detect(self.pin)
                GPIO.cleanup(self.pin)
                self.logger.info("GPIO cleanup completed")
            except Exception as e:
                self.logger.error(f"GPIO cleanup error: {e}")
        
        self.is_initialized = False

class MockVibrationSensor(VibrationSensor):
    """モック振動センサー（テスト用）"""
    
    def __init__(self, pin: int = 18, debounce_time: float = 1.0):
        self.pin = pin
        self.debounce_time = debounce_time
        self.screw_count = 0
        self.last_detection_time = 0
        self.is_initialized = True
        self.on_screw_detected: Optional[Callable] = None
        
        # ログ設定
        self.logger = logging.getLogger('MockVibrationSensor')
        self.logger.info("Mock vibration sensor initialized")
        
        # 自動検出スレッド開始
        self._start_mock_detection()
    
    def _start_mock_detection(self):
        """モック検出の開始"""
        def mock_detection_loop():
            import random
            while self.is_initialized:
                time.sleep(random.uniform(3, 8))  # 3-8秒間隔
                
                if self.is_initialized and random.random() > 0.3:  # 70%の確率
                    self._simulate_vibration()
        
        thread = threading.Thread(target=mock_detection_loop)
        thread.daemon = True
        thread.start()
    
    def _simulate_vibration(self):
        """振動のシミュレート"""
        current_time = time.time()
        
        if current_time - self.last_detection_time >= self.debounce_time:
            self.last_detection_time = current_time
            self.screw_count += 1
            
            self.logger.info(f"🔩 Mock vibration detected! Count: {self.screw_count}")
            
            if self.on_screw_detected:
                try:
                    self.on_screw_detected(self.screw_count)
                except Exception as e:
                    self.logger.error(f"Callback error: {e}")
    
    def read_vibration_level(self) -> float:
        """モック振動レベル"""
        import random
        return random.uniform(0.0, 0.8)
    
    def cleanup(self):
        """モッククリーンアップ"""
        self.is_initialized = False
        self.logger.info("Mock vibration sensor cleanup completed")

# ファクトリー関数
def create_vibration_sensor(pin: int = 18, debounce_time: float = 1.0, use_mock: bool = False) -> VibrationSensor:
    """
    振動センサーの作成
    
    Args:
        pin: GPIO ピン番号
        debounce_time: デバウンス時間
        use_mock: モック実装を使用するか
        
    Returns:
        振動センサーインスタンス
    """
    if use_mock or not GPIO_AVAILABLE:
        return MockVibrationSensor(pin, debounce_time)
    else:
        return VibrationSensor(pin, debounce_time)

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
    sensor = create_vibration_sensor(use_mock=True)  # モック使用
    
    # コールバック設定
    def on_screw_detected(count):
        print(f"Callback: Screw #{count} detected!")
    
    sensor.set_callback(on_screw_detected)
    
    print("Vibration sensor test started. Press Ctrl+C to stop.")
    print("Sensor info:", sensor.get_sensor_info())
    
    try:
        # メインループ
        while True:
            vibration_level = sensor.read_vibration_level()
            print(f"Vibration level: {vibration_level:.3f}, Count: {sensor.get_screw_count()}")
            time.sleep(2)
            
    except KeyboardInterrupt:
        pass
    finally:
        sensor.cleanup()