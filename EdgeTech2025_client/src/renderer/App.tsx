import React, { useState, useEffect, useCallback } from 'react';
import {
  Box,
  Container,
  Typography,
  AppBar,
  Toolbar,
  Grid,
  Chip,
  useTheme,
  useMediaQuery,
  IconButton,
  Tooltip
} from '@mui/material';
import {
  Wifi,
  WifiOff,
  AccessTime,
  Factory,
  SignalWifiStatusbar4Bar,
  Settings as SettingsIcon,
  ControlCamera as ControlIcon,
  History as HistoryIcon,
  Storage as StorageIcon
} from '@mui/icons-material';
import LiveVideo from './components/LiveVideo';
import StatusPanel from './components/StatusPanel';
import { CommandHistoryItem } from './components/ControlPanel';
import ControlPanelDialog from './components/ControlPanelDialog';
import CommunicationHistoryDialog from './components/CommunicationHistoryDialog';
import TimeChart from './components/TimeChart';
import WorkStatistics from './components/WorkStatistics';
import { SettingsDialog } from './components/SettingsDialog';
import DatabaseViewer from './components/DatabaseViewer';
import { StatusData, CommandData, WorkHistoryEntry, WorkStatistics as WorkStatsType, AppSettings, RobotAdditionalStatus } from '../shared/types';

interface AppState {
  isConnected: boolean;
  currentTime: string;
  appInfo: {
    name: string;
    version: string;
  };
  currentStatus: StatusData | null;
  commandHistory: CommandHistoryItem[];
  connectionQuality: 'excellent' | 'good' | 'poor' | 'disconnected';
  lastDataReceived: Date | null;
  workHistory: WorkHistoryEntry[];
  workStatistics: WorkStatsType;
  // ロボット追加ステータス（新しいトピックから取得）
  robotStatus: RobotAdditionalStatus;
}

const App: React.FC = () => {
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('md'));

  const [appState, setAppState] = useState<AppState>({
    isConnected: false,
    currentTime: new Date().toLocaleTimeString('ja-JP'),
    appInfo: {
      name: 'Digital Twin Dashboard',
      version: '1.0.0'
    },
    currentStatus: {
      worker_status: 'Waiting',
      space_status: 'Nothing',
      robot_status: {
        state: 'idle',
        grip: 'open'
      },
      timestamp: '',
      tool_delivery: 0,
      status: 'Waiting'
    },
    commandHistory: [],
    connectionQuality: 'disconnected',
    lastDataReceived: null,
    workHistory: [],
    workStatistics: {
      screwCount: 0,
      blocksCount: 0,
      surveyCount: 0,
      totalWorkTime: 0,
      averageWorkTime: 0,
      recentPerformance: null
    },
    robotStatus: {
      operatingStatus: 'idle',
      gripperStatus: 'open'
    }
  });

  const [settingsOpen, setSettingsOpen] = useState(false);
  const [controlPanelOpen, setControlPanelOpen] = useState(false);
  const [communicationHistoryOpen, setCommunicationHistoryOpen] = useState(false);
  const [databaseViewerOpen, setDatabaseViewerOpen] = useState(false);
  const [appSettings, setAppSettings] = useState<AppSettings | null>(null);

  // コマンド送信処理
  const handleCommandSend = useCallback(async (command: CommandData): Promise<boolean> => {
    try {
      if (!window.electronAPI) {
        console.error('Electron API not available');
        return false;
      }

      const result = await window.electronAPI.sendCommand(command);
      
      // コマンド履歴に追加
      const historyItem: CommandHistoryItem = {
        id: Date.now(),
        command: command.command,
        timestamp: new Date(),
        success: result.success
      };

      setAppState(prev => ({
        ...prev,
        commandHistory: [historyItem, ...prev.commandHistory.slice(0, 9)] // 最新10件を保持
      }));

      return result.success;
    } catch (error) {
      console.error('Command send error:', error);
      return false;
    }
  }, []);

  // 現在時刻の更新
  useEffect(() => {
    const timer = setInterval(() => {
      setAppState(prev => ({
        ...prev,
        currentTime: new Date().toLocaleTimeString('ja-JP')
      }));
    }, 1000);

    return () => clearInterval(timer);
  }, []);

  // アプリケーション情報の取得
  useEffect(() => {
    const loadAppInfo = async () => {
      try {
        if (window.electronAPI) {
          const [name, version] = await Promise.all([
            window.electronAPI.getAppName(),
            window.electronAPI.getAppVersion()
          ]);
          
          setAppState(prev => ({
            ...prev,
            appInfo: { name, version }
          }));
        }
      } catch (error) {
        console.error('Failed to load app info:', error);
      }
    };

    loadAppInfo();
  }, []);

  // WebSocket接続とイベントリスナーの設定
  useEffect(() => {
    if (!window.electronAPI) return;

    // 接続状態の監視
    const checkConnectionState = async () => {
      try {
        const connectionState = await window.electronAPI.getConnectionState();
        setAppState(prev => ({
          ...prev,
          isConnected: connectionState.isConnected,
          connectionQuality: connectionState.isConnected ? 'good' : 'disconnected'
        }));
      } catch (error) {
        console.error('Failed to get connection state:', error);
      }
    };

    // 初回チェック
    checkConnectionState();

    // 定期的な接続状態チェック
    const connectionTimer = setInterval(checkConnectionState, 5000);

    // WebSocketイベントリスナーの設定
    const setupEventListeners = () => {
      // 接続イベント
      window.electronAPI.onWebSocketConnected(() => {
        console.log('WebSocket connected');
        setAppState(prev => ({
          ...prev,
          isConnected: true,
          connectionQuality: 'good'
        }));
      });

      // 切断イベント
      window.electronAPI.onWebSocketDisconnected((info) => {
        console.log('WebSocket disconnected:', info);
        setAppState(prev => ({
          ...prev,
          isConnected: false,
          connectionQuality: 'disconnected'
        }));
      });

      // ステータスデータ受信
      window.electronAPI.onWebSocketStatusData((data: StatusData) => {
        console.log('Received status data:', data);
        setAppState(prev => ({
          ...prev,
          currentStatus: data,
          lastDataReceived: new Date()
        }));
      });

      // ロボット動作状態受信
      window.electronAPI.onOperatingStatus((status: string) => {
        console.log('Operating status received:', status);
        setAppState(prev => ({
          ...prev,
          robotStatus: {
            ...prev.robotStatus,
            operatingStatus: status as 'idle' | 'operation'
          },
          lastDataReceived: new Date()
        }));
      });

      // ロボットグリッパー状態受信
      window.electronAPI.onGripperStatus((status: string) => {
        console.log('Gripper status received:', status);
        setAppState(prev => ({
          ...prev,
          robotStatus: {
            ...prev.robotStatus,
            gripperStatus: status as 'open' | 'close'
          },
          lastDataReceived: new Date()
        }));
      });

      // 接続状態変更
      window.electronAPI.onWebSocketConnectionStateChanged((state) => {
        console.log('Connection state changed:', state);
        setAppState(prev => ({
          ...prev,
          isConnected: state.isConnected,
          connectionQuality: state.isConnected ? 'good' : 'disconnected'
        }));
      });

      // エラーイベント
      window.electronAPI.onWebSocketError((error) => {
        console.error('WebSocket error:', error);
        setAppState(prev => ({
          ...prev,
          isConnected: false,
          connectionQuality: 'disconnected'
        }));
      });
    };

    setupEventListeners();

    return () => {
      clearInterval(connectionTimer);
      // イベントリスナーのクリーンアップ
      if (window.electronAPI.removeAllListeners) {
        window.electronAPI.removeAllListeners('websocket:connected');
        window.electronAPI.removeAllListeners('websocket:disconnected');
        window.electronAPI.removeAllListeners('websocket:statusData');
        window.electronAPI.removeAllListeners('robot:operatingStatus');
        window.electronAPI.removeAllListeners('robot:gripperStatus');
        window.electronAPI.removeAllListeners('websocket:connectionStateChanged');
        window.electronAPI.removeAllListeners('websocket:error');
      }
    };
  }, []);

  // 作業履歴と統計データの取得
  useEffect(() => {
    const loadWorkData = async () => {
      if (!window.electronAPI) return;

      try {
        // 作業履歴の取得
        const workHistory = await window.electronAPI.getWorkHistory('24h');
        // 統計データの取得
        const workStatistics = await window.electronAPI.getWorkStatistics();

        setAppState(prev => ({
          ...prev,
          workHistory,
          workStatistics
        }));
      } catch (error) {
        console.error('Failed to load work data:', error);
        // エラー時は空のデータを設定
        setAppState(prev => ({
          ...prev,
          workHistory: [],
          workStatistics: {
            screwCount: 0,
            blocksCount: 0,
            surveyCount: 0,
            totalWorkTime: 0,
            averageWorkTime: 0,
            recentPerformance: null
          }
        }));
      }
    };

    loadWorkData();
  }, []);

  // 統計データの更新
  const updateStatistics = useCallback(async () => {
    if (!window.electronAPI) return;

    try {
      const workStatistics = await window.electronAPI.getWorkStatistics();
      setAppState(prev => ({
        ...prev,
        workStatistics
      }));
    } catch (error) {
      console.error('Failed to update statistics:', error);
    }
  }, []);

  // 設定変更時の処理
  const handleSettingsChanged = useCallback((newSettings: AppSettings) => {
    setAppSettings(newSettings);
    console.log('Settings updated:', newSettings);
  }, []);

  // 初期設定の取得
  useEffect(() => {
    const fetchSettings = async () => {
      if (window.electronAPI && window.electronAPI.getSettings) {
        const settings = await window.electronAPI.getSettings();
        setAppSettings(settings);
      }
    };
    fetchSettings();
  }, []);
  // サンプルデータ定義（テストモード用）
  const sampleWorkHistory: WorkHistoryEntry[] = [
    // 4時間前からのデータを生成
    { id: 1, timestamp: new Date(Date.now() - 1000 * 60 * 240), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 2, timestamp: new Date(Date.now() - 1000 * 60 * 235), task: 'Screw_tightening', duration: 120, worker_status: 'Working', robot_status: 'working' },
    { id: 3, timestamp: new Date(Date.now() - 1000 * 60 * 233), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 4, timestamp: new Date(Date.now() - 1000 * 60 * 225), task: 'Building_blocks', duration: 180, worker_status: 'Working', robot_status: 'working' },
    { id: 5, timestamp: new Date(Date.now() - 1000 * 60 * 222), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 6, timestamp: new Date(Date.now() - 1000 * 60 * 215), task: 'Survey_responses', duration: 60, worker_status: 'Working', robot_status: 'working' },
    { id: 7, timestamp: new Date(Date.now() - 1000 * 60 * 214), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 8, timestamp: new Date(Date.now() - 1000 * 60 * 210), task: 'Screw_tightening', duration: 110, worker_status: 'Working', robot_status: 'working' },
    { id: 9, timestamp: new Date(Date.now() - 1000 * 60 * 208), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 10, timestamp: new Date(Date.now() - 1000 * 60 * 200), task: 'Building_blocks', duration: 190, worker_status: 'Working', robot_status: 'working' },
    { id: 11, timestamp: new Date(Date.now() - 1000 * 60 * 197), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 12, timestamp: new Date(Date.now() - 1000 * 60 * 190), task: 'Screw_tightening', duration: 125, worker_status: 'Working', robot_status: 'working' },
    { id: 13, timestamp: new Date(Date.now() - 1000 * 60 * 188), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 14, timestamp: new Date(Date.now() - 1000 * 60 * 180), task: 'Survey_responses', duration: 55, worker_status: 'Working', robot_status: 'working' },
    { id: 15, timestamp: new Date(Date.now() - 1000 * 60 * 179), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 16, timestamp: new Date(Date.now() - 1000 * 60 * 170), task: 'Building_blocks', duration: 175, worker_status: 'Working', robot_status: 'working' },
    { id: 17, timestamp: new Date(Date.now() - 1000 * 60 * 167), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 18, timestamp: new Date(Date.now() - 1000 * 60 * 160), task: 'Screw_tightening', duration: 115, worker_status: 'Working', robot_status: 'working' },
    { id: 19, timestamp: new Date(Date.now() - 1000 * 60 * 158), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 20, timestamp: new Date(Date.now() - 1000 * 60 * 150), task: 'Survey_responses', duration: 65, worker_status: 'Working', robot_status: 'working' },
    { id: 21, timestamp: new Date(Date.now() - 1000 * 60 * 149), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 22, timestamp: new Date(Date.now() - 1000 * 60 * 140), task: 'Screw_tightening', duration: 130, worker_status: 'Working', robot_status: 'working' },
    { id: 23, timestamp: new Date(Date.now() - 1000 * 60 * 138), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 24, timestamp: new Date(Date.now() - 1000 * 60 * 130), task: 'Building_blocks', duration: 185, worker_status: 'Working', robot_status: 'working' },
    { id: 25, timestamp: new Date(Date.now() - 1000 * 60 * 127), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 26, timestamp: new Date(Date.now() - 1000 * 60 * 120), task: 'Survey_responses', duration: 70, worker_status: 'Working', robot_status: 'working' },
    { id: 27, timestamp: new Date(Date.now() - 1000 * 60 * 119), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 28, timestamp: new Date(Date.now() - 1000 * 60 * 110), task: 'Screw_tightening', duration: 105, worker_status: 'Working', robot_status: 'working' },
    { id: 29, timestamp: new Date(Date.now() - 1000 * 60 * 108), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 30, timestamp: new Date(Date.now() - 1000 * 60 * 100), task: 'Building_blocks', duration: 170, worker_status: 'Working', robot_status: 'working' },
    { id: 31, timestamp: new Date(Date.now() - 1000 * 60 * 97), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 32, timestamp: new Date(Date.now() - 1000 * 60 * 90), task: 'Screw_tightening', duration: 120, worker_status: 'Working', robot_status: 'working' },
    { id: 33, timestamp: new Date(Date.now() - 1000 * 60 * 88), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 34, timestamp: new Date(Date.now() - 1000 * 60 * 80), task: 'Survey_responses', duration: 60, worker_status: 'Working', robot_status: 'working' },
    { id: 35, timestamp: new Date(Date.now() - 1000 * 60 * 79), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 36, timestamp: new Date(Date.now() - 1000 * 60 * 70), task: 'Building_blocks', duration: 180, worker_status: 'Working', robot_status: 'working' },
    { id: 37, timestamp: new Date(Date.now() - 1000 * 60 * 67), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 38, timestamp: new Date(Date.now() - 1000 * 60 * 60), task: 'Screw_tightening', duration: 135, worker_status: 'Working', robot_status: 'working' },
    { id: 39, timestamp: new Date(Date.now() - 1000 * 60 * 58), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 40, timestamp: new Date(Date.now() - 1000 * 60 * 50), task: 'Survey_responses', duration: 50, worker_status: 'Working', robot_status: 'working' },
    { id: 41, timestamp: new Date(Date.now() - 1000 * 60 * 49), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 42, timestamp: new Date(Date.now() - 1000 * 60 * 40), task: 'Building_blocks', duration: 195, worker_status: 'Working', robot_status: 'working' },
    { id: 43, timestamp: new Date(Date.now() - 1000 * 60 * 37), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 44, timestamp: new Date(Date.now() - 1000 * 60 * 30), task: 'Screw_tightening', duration: 100, worker_status: 'Working', robot_status: 'working' },
    { id: 45, timestamp: new Date(Date.now() - 1000 * 60 * 28), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 46, timestamp: new Date(Date.now() - 1000 * 60 * 20), task: 'Survey_responses', duration: 75, worker_status: 'Working', robot_status: 'working' },
    { id: 47, timestamp: new Date(Date.now() - 1000 * 60 * 19), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 48, timestamp: new Date(Date.now() - 1000 * 60 * 10), task: 'Building_blocks', duration: 165, worker_status: 'Working', robot_status: 'working' },
    { id: 49, timestamp: new Date(Date.now() - 1000 * 60 * 7), task: 'Waiting', duration: 0, worker_status: 'Waiting', robot_status: 'idle' },
    { id: 50, timestamp: new Date(Date.now() - 1000 * 60 * 2), task: 'Screw_tightening', duration: 110, worker_status: 'Working', robot_status: 'working' }
  ];

  const sampleWorkStatistics: WorkStatsType = {
    screwCount: 12,
    blocksCount: 10,
    surveyCount: 8,
    totalWorkTime: 3600,
    averageWorkTime: 120,
    recentPerformance: 'fast'
  };

  // 接続品質アイコンの取得
  const getConnectionIcon = () => {
    switch (appState.connectionQuality) {
      case 'excellent':
        return <SignalWifiStatusbar4Bar />;
      case 'good':
        return <Wifi />;
      case 'poor':
        return <Wifi />;
      default:
        return <WifiOff />;
    }
  };

  // 接続品質色の取得
  const getConnectionColor = () => {
    switch (appState.connectionQuality) {
      case 'excellent':
        return 'success';
      case 'good':
        return 'info';
      case 'poor':
        return 'warning';
      default:
        return 'error';
    }
  };

  return (
    <Box sx={{ 
      flexGrow: 1, 
      minHeight: '100vh', 
      display: 'flex', 
      flexDirection: 'column',
      backgroundColor: theme.palette.background.default,
      overflow: 'hidden' // 全体のスクロールを制御
    }}>
      {/* ヘッダー */}
      <AppBar position="static" sx={{ backgroundColor: '#1e1e1e', boxShadow: 3, flexShrink: 0 }}>
        <Toolbar sx={{ minHeight: { xs: 56, sm: 64 } }}>
          <Factory sx={{ mr: 2, color: theme.palette.primary.main }} />
          <Typography 
            variant={isMobile ? "subtitle1" : "h6"} 
            component="div" 
            sx={{ flexGrow: 1, fontWeight: 600 }}
          >
            {isMobile ? 'デジタルツイン' : `${appState.appInfo.name} v${appState.appInfo.version}`}
          </Typography>
          {/* ステータスサマリー（タイトル右側） */}
          <Box sx={{ display: { xs: 'none', sm: 'flex' }, alignItems: 'center', gap: 1, mr: 2 }}>
            {([
              { key: 'Waiting', label: '待機中' },
              { key: 'Ready', label: '準備完了' },
              { key: 'Working', label: '作業中' },
              { key: 'Work Completed', label: '作業完了' }
            ] as { key: string; label: string }[]).map(item => {
              // ヘッダーは優先的に status フィールドを参照する
              const current = appState.currentStatus?.status ?? appState.currentStatus?.worker_status ?? 'Waiting';
              const isActive = current === item.key;
              return (
                <Box
                  key={item.key}
                  sx={{
                    px: 1.2,
                    py: 0.4,
                    borderRadius: 1.5,
                    minWidth: 72,
                    textAlign: 'center',
                    backgroundColor: isActive ? '#2e7d32' : '#424242',
                    color: isActive ? '#fff' : '#bdbdbd',
                    fontWeight: isActive ? 700 : 400,
                    fontSize: '0.8rem',
                    boxShadow: isActive ? '0 2px 8px rgba(46,125,50,0.18)' : 'none',
                    transition: 'background-color 200ms, color 200ms'
                  }}
                >
                  {item.label}
                </Box>
              );
            })}
          </Box>
          
          {/* デモ状況表示はヘッダー移動に伴い不要になったので削除 */}
          
          {/* 接続状態 */}
          <Chip
            icon={getConnectionIcon()}
            label={
              isMobile 
                ? (appState.isConnected ? '接続' : '未接続')
                : (appState.isConnected ? `接続中 (${appState.connectionQuality})` : '未接続')
            }
            color={getConnectionColor() as any}
            sx={{ mr: 2 }}
            size={isMobile ? "small" : "medium"}
          />
          
          {/* 現在時刻 */}
          <Chip
            icon={<AccessTime />}
            label={appState.currentTime}
            variant="outlined"
            size={isMobile ? "small" : "medium"}
            sx={{ mr: 1 }}
          />
          
          {/* 制御パネルボタン */}
          <Tooltip title="制御パネル">
            <IconButton
              color="inherit"
              onClick={() => setControlPanelOpen(true)}
              sx={{ ml: 1 }}
            >
              <ControlIcon />
            </IconButton>
          </Tooltip>
          
          {/* 通信履歴ボタン */}
          <Tooltip title="通信履歴">
            <IconButton
              color="inherit"
              onClick={() => setCommunicationHistoryOpen(true)}
              sx={{ ml: 1 }}
            >
              <HistoryIcon />
            </IconButton>
          </Tooltip>
          
          {/* 設定ボタン */}
          <Tooltip title="設定">
            <IconButton
              color="inherit"
              onClick={() => setSettingsOpen(true)}
              sx={{ ml: 1 }}
            >
              <SettingsIcon />
            </IconButton>
          </Tooltip>

          {/* データベースビューワーボタン */}
          <Tooltip title="データベース">
            <IconButton
              color="inherit"
              onClick={() => setDatabaseViewerOpen(true)}
              sx={{ ml: 1 }}
            >
              <StorageIcon />
            </IconButton>
          </Tooltip>
        </Toolbar>
      </AppBar>

      {/* メインコンテンツ - スクロール可能 */}
      <Box sx={{ 
        flexGrow: 1, 
        overflow: 'auto',
        height: 'calc(100vh - 64px)' // ヘッダーの高さを除く
      }}>
        <Container 
          maxWidth={false} 
          sx={{ 
            py: { xs: 2, sm: 3 },
            px: { xs: 1, sm: 2, md: 3 }
          }}
        >
          <Grid container spacing={{ xs: 2, sm: 3 }}>
            {/* 上段: 状態表示エリア */}
            <Grid item xs={12} sm={6} lg={3}>
              <StatusPanel
                  title="作業者状況"
                  status={appState.currentStatus?.worker_status || 'Waiting'}
                  type="worker"
                  lastUpdated={appState.lastDataReceived}
                  height={460} // 高さを統一
                  spaceStatus={appState.currentStatus?.space_status}
                />
            </Grid>

            <Grid item xs={12} sm={6} lg={3}>
              <StatusPanel
                title="ロボット状況"
                status={appState.robotStatus.operatingStatus ?? 'unknown'}
                type="robot"
                robotGrip={appState.robotStatus.gripperStatus === 'close' ? 'closed' : appState.robotStatus.gripperStatus}
                height={460} // 高さを統一
              />
            </Grid>

            <Grid item xs={12} lg={6}>
              <LiveVideo 
                height={460} // 高さを統一
                title="ライブ映像"
                autoRefresh={true}
                showControls={true}
              />
            </Grid>

            {/* 下段: チャートと統計エリア */}
            <Grid item xs={12} lg={8}>
              <TimeChart
                data={appSettings?.testMode ? sampleWorkHistory : appState.workHistory}
                height={isMobile ? 250 : 420}
                title="作業履歴タイムチャート"
                showControls={true}
              />
            </Grid>

            <Grid item xs={12} lg={4}>
              <WorkStatistics
                statistics={appSettings?.testMode ? sampleWorkStatistics : appState.workStatistics}
                height={isMobile ? 250 : 420}
                title="作業統計"
                showRefreshButton={true}
                onRefresh={updateStatistics}
                autoRefresh={true}
                refreshInterval={30000}
                settings={appSettings}
              />
            </Grid>
          </Grid>
        </Container>
      </Box>

      {/* 制御パネルダイアログ */}
      <ControlPanelDialog
        open={controlPanelOpen}
        onClose={() => setControlPanelOpen(false)}
        onCommandSend={handleCommandSend}
        isConnected={appState.isConnected}
        commandHistory={appState.commandHistory}
      />

      {/* 通信履歴ダイアログ */}
      <CommunicationHistoryDialog
        open={communicationHistoryOpen}
        onClose={() => setCommunicationHistoryOpen(false)}
        isConnected={appState.isConnected}
      />

      {/* 設定ダイアログ */}
      <SettingsDialog
        open={settingsOpen}
        onClose={() => setSettingsOpen(false)}
        onSettingsChanged={handleSettingsChanged}
      />

      {/* データベースビューワーダイアログ */}
      <DatabaseViewer
        open={databaseViewerOpen}
        onClose={() => setDatabaseViewerOpen(false)}
      />
    </Box>
  );
};

export default App;