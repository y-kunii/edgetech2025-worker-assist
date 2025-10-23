// WebSocket通信で使用するデータ型定義

export interface StatusData {
  worker_status: 'Absent' | 'Waiting' | 'Working' | 'Work Completed';
  space_status: 'Nothing' | 'Screw_tightening' | 'Building_blocks' | 'Survey_responses';
  robot_status: {
    state: string;
    grip: 'open' | 'closed';
  };
  timestamp: string; // YYYYMMDDhhmmss
  tool_delivery: number;
  status: 'Absent' | 'Waiting' | 'Working' | 'Work Completed';
}

export interface CommandData {
  command: 'tool_handover' | 'tool_collection' | 'wait';
  timestamp: string; // YYYYMMDDhhmmss
}

// アプリケーション設定
export interface AppSettings {
  wsServerUrl: string;
  imageWatchPath: string;
  refreshInterval: number;
  autoConnect: boolean;
  theme: 'dark' | 'light';
  language: 'ja' | 'en';
  maxHistoryDays: number;
  enableNotifications: boolean;
}

// 設定検証結果
export interface SettingsValidationResult {
  isValid: boolean;
  errors: string[];
}

// 作業履歴エントリ
export interface WorkHistoryEntry {
  id: number;
  timestamp: Date;
  task: 'Screw_tightening' | 'Building_blocks' | 'Survey_responses' | 'Waiting';
  duration: number;
  worker_status: string;
  robot_status: string;
}

// 作業統計
export interface WorkStatistics {
  screwCount: number;
  blocksCount: number;
  surveyCount: number;
  totalWorkTime: number;
  averageWorkTime: number;
  recentPerformance: 'fast' | 'normal' | 'slow' | null;
}

// アプリケーション状態
export interface AppState {
  connection: {
    isConnected: boolean;
    lastHeartbeat: Date | null;
    reconnectAttempts: number;
  };
  currentStatus: StatusData | null;
  liveImage: {
    path: string | null;
    lastUpdated: Date | null;
  };
  workHistory: WorkHistoryEntry[];
  workStatistics: WorkStatistics;
  settings: AppSettings;
}

// データベーステーブル型定義
export interface StatusHistoryRecord {
  id: number;
  timestamp: string;
  worker_status: string;
  space_status: string;
  robot_state: string;
  robot_grip: string;
  tool_delivery: number;
  demo_status: string;
  created_at: string;
}

export interface CommandHistoryRecord {
  id: number;
  timestamp: string;
  command: string;
  success: boolean;
  created_at: string;
}

export interface WorkStatisticsRecord {
  id: number;
  date: string;
  screw_count: number;
  blocks_count: number;
  survey_count: number;
  total_work_time: number;
  average_work_time: number;
  created_at: string;
}