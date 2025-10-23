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
  History as HistoryIcon
} from '@mui/icons-material';
import LiveVideo from './components/LiveVideo';
import StatusPanel from './components/StatusPanel';
import { CommandHistoryItem } from './components/ControlPanel';
import ControlPanelDialog from './components/ControlPanelDialog';
import CommunicationHistoryDialog from './components/CommunicationHistoryDialog';
import TimeChart from './components/TimeChart';
import WorkStatistics from './components/WorkStatistics';
import { SettingsDialog } from './components/SettingsDialog';
import { StatusData, CommandData, WorkHistoryEntry, WorkStatistics as WorkStatsType, AppSettings } from '../shared/types';

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
    currentStatus: null,
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
    }
  });

  const [settingsOpen, setSettingsOpen] = useState(false);
  const [controlPanelOpen, setControlPanelOpen] = useState(false);
  const [communicationHistoryOpen, setCommunicationHistoryOpen] = useState(false);

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
    // 設定変更に応じてアプリケーションの動作を更新
    console.log('Settings updated:', newSettings);
  }, []);

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
          
          {/* デモ状況表示 */}
          {appState.currentStatus && (
            <Chip
              label={`デモ: ${appState.currentStatus.space_status === 'Nothing' ? '待機中' : '実行中'}`}
              color={appState.currentStatus.space_status === 'Nothing' ? 'default' : 'success'}
              size="small"
              sx={{ mr: 1, display: { xs: 'none', sm: 'flex' } }}
            />
          )}
          
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
                status={appState.currentStatus?.worker_status || 'Absent'}
                type="worker"
                lastUpdated={appState.lastDataReceived}
                height={400} // 高さを統一
              />
            </Grid>

            <Grid item xs={12} sm={6} lg={3}>
              <StatusPanel
                title="ロボット状況"
                status={appState.currentStatus?.robot_status.state || 'idle'}
                type="robot"
                robotGrip={appState.currentStatus?.robot_status.grip}
                lastUpdated={appState.lastDataReceived}
                height={400} // 高さを統一
              />
            </Grid>

            <Grid item xs={12} lg={6}>
              <LiveVideo 
                height={400} // 高さを統一
                title="ライブ映像"
                autoRefresh={true}
                showControls={true}
              />
            </Grid>

            {/* 下段: チャートと統計エリア */}
            <Grid item xs={12} lg={8}>
              <TimeChart
                data={appState.workHistory}
                height={isMobile ? 250 : 380}
                title="作業履歴タイムチャート"
                showControls={true}
              />
            </Grid>

            <Grid item xs={12} lg={4}>
              <WorkStatistics
                statistics={appState.workStatistics}
                height={isMobile ? 250 : 380}
                title="作業統計"
                showRefreshButton={true}
                onRefresh={updateStatistics}
                autoRefresh={true}
                refreshInterval={30000}
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
    </Box>
  );
};

export default App;