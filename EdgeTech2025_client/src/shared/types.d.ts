export interface StatusData {
    worker_status: 'Absent' | 'Waiting' | 'Working' | 'Work Completed';
    space_status: 'Nothing' | 'Screw_tightening' | 'Building_blocks' | 'Survey_responses';
    robot_status: {
        state: string;
        grip: 'open' | 'closed';
    };
    timestamp: string;
    tool_delivery: number;
    status: 'Absent' | 'Waiting' | 'Working' | 'Work Completed';
}
export interface CommandData {
    command: 'tool_handover' | 'tool_collection' | 'wait';
    timestamp: string;
}
export interface AppSettings {
    wsServerUrl: string;
    imageWatchPath: string;
    refreshInterval: number;
}
export interface WorkHistoryEntry {
    id: number;
    timestamp: Date;
    task: 'Screw_tightening' | 'Building_blocks' | 'Survey_responses' | 'Waiting';
    duration: number;
    worker_status: string;
    robot_status: string;
}
export interface WorkStatistics {
    screwCount: number;
    blocksCount: number;
    surveyCount: number;
    totalWorkTime: number;
    averageWorkTime: number;
    recentPerformance: 'fast' | 'normal' | 'slow' | null;
}
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
}
