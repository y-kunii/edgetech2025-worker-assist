export type WorkerStatus = 'waiting' | 'screw_tightening' | 'tool_handover' | 'bolt_tightening' | 'absent';

export interface RobotStatus {
  state: 'waiting' | 'operating';
  grip: 'open' | 'closed';
}

export interface WorkData {
  timestamp: Date;
  workerStatus: WorkerStatus;
  robotStatus: RobotStatus;
  screwCount: number;
  boltCount: number;
  image?: string; // base64 encoded
}

export interface ThresholdSettings {
  screwThreshold: number;
  boltThreshold: number;
}

export interface SensorData {
  type: string;
  timestamp: string;
  data: {
    image: string;
    worker_status: WorkerStatus;
    robot_status: RobotStatus;
    screw_count: number;
    bolt_count: number;
    work_step: WorkerStatus;
  };
}

export interface RobotCommand {
  id: string;
  type: 'tool_handover' | 'next_task';
  timestamp: string;
  parameters?: Record<string, any>;
}

export interface RobotCommandResponse {
  commandId: string;
  status: 'success' | 'error';
  message: string;
  timestamp: string;
}

// 統計情報の型定義
export interface WorkStatistics {
  totalWorkTime: number; // 分単位
  completedTasks: number;
  averageEfficiency: number; // パーセンテージ
  errorCount: number;
}

// 作業履歴の型定義
export interface WorkHistory {
  id: string;
  timestamp: Date;
  workerStatus: WorkerStatus;
  duration: number; // ミリ秒
  efficiency: number; // パーセンテージ
}

// 効率指標の型定義
export interface EfficiencyMetrics {
  current: number; // 現在の効率（パーセンテージ）
  target: number; // 目標効率（パーセンテージ）
  trend: 'up' | 'down' | 'stable'; // トレンド
  lastUpdated: Date;
}

// 通知の型定義
export interface Notification {
  id: string;
  type: 'info' | 'warning' | 'error' | 'success';
  title: string;
  message: string;
  timestamp: Date;
  read: boolean;
}

// 接続品質の型定義
export interface ConnectionQuality {
  latency: number; // ミリ秒
  dataRate: number; // メッセージ/秒
  stability: 'excellent' | 'good' | 'fair' | 'poor';
  lastUpdated: Date;
}