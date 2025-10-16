/**
 * 型定義
 * ラズパイWebSocketサーバーで使用する型定義
 */

// ログレベル型
export type LogLevel = "debug" | "info" | "warn" | "error";

// クライアントタイプ型
export type ClientType = "electron" | "sensor" | "robot" | "unknown";

// サーバー設定型
export interface ServerConfig {
  port: number;                    // WebSocketサーバーのポート番号（デフォルト: 3001）
  cors_origin: string;             // CORS許可オリジン（デフォルト: "*"）
  heartbeat_interval: number;      // ハートビート送信間隔（ミリ秒、デフォルト: 30000）
  connection_timeout: number;      // 接続タイムアウト（ミリ秒、デフォルト: 60000）
  log_level: LogLevel;             // ログレベル（デフォルト: "info"）
  log_file: string;                // ログファイルパス（デフォルト: "logs/server.log"）
}

// 作業者状態型
export type WorkerStatus = "waiting" | "screw_tightening" | "bolt_tightening" | "tool_handover" | "absent";

// ロボット状態型
export interface RobotStatus {
  state: "waiting" | "operating";
  grip: "open" | "closed";
}

// センサーデータ型
export interface SensorData {
  worker_status: WorkerStatus;
  robot_status: RobotStatus;
  screw_count: number;
  bolt_count: number;
  work_step: string;
  image?: string;  // Base64エンコード画像（オプション）
  timestamp?: string;
}

// コマンドタイプ型
export type CommandType = "tool_handover" | "next_task" | "emergency_stop" | "reset";

// ロボット指示型
export interface RobotCommand {
  command: CommandType;
  data?: any;
  timestamp: string;
}

// 応答ステータス型
export type ResponseStatus = "success" | "error" | "emergency_stopped";

// ロボット応答型
export interface RobotResponse {
  command: CommandType;
  status: ResponseStatus;
  timestamp: string;
  data?: any;
}

// ヘルスステータス型
export interface HealthStatus {
  status: "ok" | "error";
  timestamp: string;
  connections: {
    electron: number;
    sensor: number;
    robot: number;
    unknown: number;
    total: number;
  };
  sensor_connected: boolean;
  robot_connected: boolean;
}

// 接続統計型
export interface ConnectionStats {
  total: number;
  byType: {
    electron: number;
    sensor: number;
    robot: number;
    unknown: number;
  };
}

// エラー情報型
export interface ErrorInfo {
  code: string;
  message: string;
  timestamp: string;
}
