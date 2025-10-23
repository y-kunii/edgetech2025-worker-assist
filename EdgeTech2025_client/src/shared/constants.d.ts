export declare const APP_CONFIG: {
    readonly WS_RECONNECT_INTERVAL: 5000;
    readonly WS_MAX_RECONNECT_ATTEMPTS: 10;
    readonly WS_HEARTBEAT_INTERVAL: 30000;
    readonly IMAGE_WATCH_DEBOUNCE: 500;
    readonly SUPPORTED_IMAGE_FORMATS: readonly [".jpg", ".jpeg", ".png", ".bmp"];
    readonly DB_FILE_NAME: "digital_twin.db";
    readonly DATA_RETENTION_DAYS: 30;
    readonly CHART_UPDATE_INTERVAL: 1000;
    readonly STATUS_UPDATE_INTERVAL: 1000;
    readonly DEFAULT_SETTINGS: {
        readonly wsServerUrl: "ws://localhost:9090";
        readonly imageWatchPath: "./images";
        readonly refreshInterval: 1000;
    };
};
export declare const DEMO_TASKS: {
    readonly SCREW_TIGHTENING: "Screw_tightening";
    readonly BUILDING_BLOCKS: "Building_blocks";
    readonly SURVEY_RESPONSES: "Survey_responses";
    readonly NOTHING: "Nothing";
};
export declare const WORKER_STATUS: {
    readonly ABSENT: "Absent";
    readonly WAITING: "Waiting";
    readonly WORKING: "Working";
    readonly WORK_COMPLETED: "Work Completed";
};
export declare const ROBOT_COMMANDS: {
    readonly TOOL_HANDOVER: "tool_handover";
    readonly TOOL_COLLECTION: "tool_collection";
    readonly WAIT: "wait";
};
export declare const UI_COLORS: {
    readonly SUCCESS: "#4caf50";
    readonly WARNING: "#ff9800";
    readonly ERROR: "#f44336";
    readonly INFO: "#2196f3";
    readonly PRIMARY: "#1976d2";
    readonly SECONDARY: "#dc004e";
};
export declare const LABELS: {
    readonly WORKER_STATUS: {
        readonly Absent: "不在";
        readonly Waiting: "待機中";
        readonly Working: "作業中";
        readonly "Work Completed": "作業完了";
    };
    readonly DEMO_TASKS: {
        readonly Screw_tightening: "ネジ締め";
        readonly Building_blocks: "積み木";
        readonly Survey_responses: "アンケート回答";
        readonly Nothing: "作業なし";
    };
    readonly ROBOT_COMMANDS: {
        readonly tool_handover: "工具受け渡し";
        readonly tool_collection: "工具回収";
        readonly wait: "待機";
    };
};
