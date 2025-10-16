#!/usr/bin/env python3
"""
カメラセンサー実装
OpenCVを使用した画像取得と作業者状態検出
"""

import cv2
import base64
import numpy as np
import logging
from typing import Optional, Tuple

class CameraSensor:
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480):
        """
        カメラセンサーの初期化
        
        Args:
            camera_id: カメラデバイスID
            width: 画像幅
            height: 画像高さ
        """
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.camera = None
        self.is_initialized = False
        
        # 前フレーム（動き検出用）
        self.prev_frame = None
        
        # ログ設定
        self.logger = logging.getLogger('CameraSensor')
        
        # カメラ初期化
        self.initialize_camera()
    
    def initialize_camera(self) -> bool:
        """カメラの初期化"""
        try:
            self.camera = cv2.VideoCapture(self.camera_id)
            
            if not self.camera.isOpened():
                self.logger.error(f"Failed to open camera {self.camera_id}")
                return False
            
            # カメラ設定
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            # テスト撮影
            ret, frame = self.camera.read()
            if not ret:
                self.logger.error("Failed to capture test frame")
                return False
            
            self.is_initialized = True
            self.logger.info(f"Camera initialized: {self.width}x{self.height}")
            return True
            
        except Exception as e:
            self.logger.error(f"Camera initialization error: {e}")
            return False
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """フレーム取得"""
        if not self.is_initialized or not self.camera.isOpened():
            return None
        
        try:
            ret, frame = self.camera.read()
            if ret:
                return frame
            else:
                self.logger.warning("Failed to capture frame")
                return None
        except Exception as e:
            self.logger.error(f"Frame capture error: {e}")
            return None
    
    def capture_image_base64(self, quality: int = 80) -> Optional[str]:
        """Base64エンコードされた画像を取得"""
        frame = self.capture_frame()
        if frame is None:
            return None
        
        try:
            # JPEG圧縮
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            _, buffer = cv2.imencode('.jpg', frame, encode_param)
            
            # Base64エンコード
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            return f"data:image/jpeg;base64,{image_base64}"
            
        except Exception as e:
            self.logger.error(f"Image encoding error: {e}")
            return None
    
    def detect_motion(self, frame: np.ndarray, threshold: float = 0.02) -> bool:
        """動き検出"""
        try:
            # グレースケール変換
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)
            
            # 初回フレーム
            if self.prev_frame is None:
                self.prev_frame = gray
                return False
            
            # フレーム差分
            frame_diff = cv2.absdiff(self.prev_frame, gray)
            thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
            
            # 膨張処理でノイズ除去
            thresh = cv2.dilate(thresh, None, iterations=2)
            
            # 動きの割合を計算
            motion_ratio = np.sum(thresh > 0) / thresh.size
            
            # 前フレーム更新
            self.prev_frame = gray
            
            return motion_ratio > threshold
            
        except Exception as e:
            self.logger.error(f"Motion detection error: {e}")
            return False
    
    def detect_worker_status(self) -> str:
        """
        作業者状態の検出
        
        実際の実装では以下の手法を使用：
        - MediaPipe: 姿勢推定
        - YOLO: 物体検出（工具、作業者）
        - カスタムML: アクション認識
        
        Returns:
            作業者状態 ('waiting', 'screw_tightening', 'bolt_tightening', 'tool_handover', 'absent')
        """
        frame = self.capture_frame()
        if frame is None:
            return 'absent'
        
        try:
            # 簡単な動き検出による状態推定（デモ用）
            motion_detected = self.detect_motion(frame)
            
            if not motion_detected:
                return 'waiting'
            
            # 実際の実装では、ここでより高度な画像解析を行う
            # 例：
            # - 手の位置と動きから工具使用を検出
            # - 作業エリアでの活動パターン分析
            # - 機械学習モデルによる行動分類
            
            # デモ用のランダム状態生成
            import random
            active_states = ['screw_tightening', 'bolt_tightening', 'tool_handover']
            return random.choice(active_states)
            
        except Exception as e:
            self.logger.error(f"Worker status detection error: {e}")
            return 'waiting'
    
    def detect_objects(self, frame: np.ndarray) -> list:
        """
        物体検出（YOLO等を使用）
        
        実際の実装例：
        - 工具（ドライバー、レンチ）の検出
        - 作業者の検出
        - 部品の検出
        
        Args:
            frame: 入力画像
            
        Returns:
            検出された物体のリスト
        """
        # YOLOやその他の物体検出モデルを使用
        # ここではプレースホルダー実装
        detected_objects = []
        
        try:
            # 実際の実装では以下のような処理：
            # net = cv2.dnn.readNet('yolo.weights', 'yolo.cfg')
            # blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            # net.setInput(blob)
            # outputs = net.forward()
            # ... 後処理 ...
            
            pass
            
        except Exception as e:
            self.logger.error(f"Object detection error: {e}")
        
        return detected_objects
    
    def analyze_pose(self, frame: np.ndarray) -> dict:
        """
        姿勢推定（MediaPipe等を使用）
        
        実際の実装例：
        - 手、腕、体の姿勢から作業状態を推定
        - 工具使用時の特徴的な姿勢を検出
        
        Args:
            frame: 入力画像
            
        Returns:
            姿勢情報の辞書
        """
        pose_info = {}
        
        try:
            # MediaPipeを使用した姿勢推定の例：
            # import mediapipe as mp
            # mp_pose = mp.solutions.pose
            # pose = mp_pose.Pose()
            # results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            # ... 姿勢解析 ...
            
            pass
            
        except Exception as e:
            self.logger.error(f"Pose analysis error: {e}")
        
        return pose_info
    
    def get_camera_info(self) -> dict:
        """カメラ情報の取得"""
        if not self.is_initialized:
            return {}
        
        try:
            return {
                'camera_id': self.camera_id,
                'width': int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
                'height': int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                'fps': int(self.camera.get(cv2.CAP_PROP_FPS)),
                'is_opened': self.camera.isOpened()
            }
        except Exception as e:
            self.logger.error(f"Failed to get camera info: {e}")
            return {}
    
    def release(self):
        """リソースの解放"""
        if self.camera and self.camera.isOpened():
            self.camera.release()
            self.logger.info("Camera released")
        self.is_initialized = False

# 使用例
if __name__ == '__main__':
    import time
    
    # ログ設定
    logging.basicConfig(level=logging.INFO)
    
    # カメラセンサー初期化
    camera = CameraSensor()
    
    if not camera.is_initialized:
        print("Failed to initialize camera")
        exit(1)
    
    try:
        print("Camera info:", camera.get_camera_info())
        
        # 10秒間テスト
        for i in range(10):
            status = camera.detect_worker_status()
            image_data = camera.capture_image_base64()
            
            print(f"Frame {i+1}: Status={status}, Image={'Available' if image_data else 'None'}")
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Test interrupted")
    finally:
        camera.release()