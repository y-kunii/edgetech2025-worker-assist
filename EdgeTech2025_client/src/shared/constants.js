"use strict";
// アプリケーション定数
Object.defineProperty(exports, "__esModule", { value: true });
exports.LABELS = exports.UI_COLORS = exports.ROBOT_COMMANDS = exports.WORKER_STATUS = exports.DEMO_TASKS = exports.APP_CONFIG = void 0;
exports.APP_CONFIG = {
    // WebSocket設定
    WS_RECONNECT_INTERVAL: 5000, // 5秒
    WS_MAX_RECONNECT_ATTEMPTS: 10,
    WS_HEARTBEAT_INTERVAL: 30000, // 30秒
    // ファイル監視設定
    IMAGE_WATCH_DEBOUNCE: 500, // 0.5秒
    SUPPORTED_IMAGE_FORMATS: ['.jpg', '.jpeg', '.png', '.bmp'],
    // データベース設定
    DB_FILE_NAME: 'digital_twin.db',
    DATA_RETENTION_DAYS: 30, // 30日間のデータを保持
    // UI設定
    CHART_UPDATE_INTERVAL: 1000, // 1秒
    STATUS_UPDATE_INTERVAL: 1000, // 1秒
    // デフォルト設定
    DEFAULT_SETTINGS: {
        wsServerUrl: 'ws://localhost:9090',
        imageWatchPath: './images',
        refreshInterval: 1000
    }
};
// 作業タスクの定義
exports.DEMO_TASKS = {
    SCREW_TIGHTENING: 'Screw_tightening',
    BUILDING_BLOCKS: 'Building_blocks',
    SURVEY_RESPONSES: 'Survey_responses',
    NOTHING: 'Nothing'
};
// 作業者状態の定義
exports.WORKER_STATUS = {
    ABSENT: 'Absent',
    WAITING: 'Waiting',
    WORKING: 'Working',
    WORK_COMPLETED: 'Work Completed'
};
// ロボット制御コマンド
exports.ROBOT_COMMANDS = {
    TOOL_HANDOVER: 'tool_handover',
    TOOL_COLLECTION: 'tool_collection',
    WAIT: 'wait'
};
// UI色定義
exports.UI_COLORS = {
    SUCCESS: '#4caf50',
    WARNING: '#ff9800',
    ERROR: '#f44336',
    INFO: '#2196f3',
    PRIMARY: '#1976d2',
    SECONDARY: '#dc004e'
};
// 日本語ラベル
exports.LABELS = {
    WORKER_STATUS: {
        [exports.WORKER_STATUS.ABSENT]: '不在',
        [exports.WORKER_STATUS.WAITING]: '待機中',
        [exports.WORKER_STATUS.WORKING]: '作業中',
        [exports.WORKER_STATUS.WORK_COMPLETED]: '作業完了'
    },
    DEMO_TASKS: {
        [exports.DEMO_TASKS.SCREW_TIGHTENING]: 'ネジ締め',
        [exports.DEMO_TASKS.BUILDING_BLOCKS]: '積み木',
        [exports.DEMO_TASKS.SURVEY_RESPONSES]: 'アンケート回答',
        [exports.DEMO_TASKS.NOTHING]: '作業なし'
    },
    ROBOT_COMMANDS: {
        [exports.ROBOT_COMMANDS.TOOL_HANDOVER]: '工具受け渡し',
        [exports.ROBOT_COMMANDS.TOOL_COLLECTION]: '工具回収',
        [exports.ROBOT_COMMANDS.WAIT]: '待機'
    }
};
