import React, { useState, useEffect, useCallback } from 'react';
import { ipcRenderer } from 'electron';
import { RobotCommandLog } from '../../main/services/RobotCommandManager';
import { RobotCommandResponse } from '../../types';
import './RobotControlPanel.css';

interface RobotControlPanelProps {
  isConnected: boolean;
  onToggleManualMode: (enabled: boolean) => void;
}

interface RobotControlState {
  isManualMode: boolean;
  commandLogs: RobotCommandLog[];
  statistics: {
    total: number;
    successful: number;
    failed: number;
    pending: number;
    successRate: number;
  };
  lastError: string | null;
  isLoading: boolean;
}

export const RobotControlPanel: React.FC<RobotControlPanelProps> = ({
  isConnected,
  onToggleManualMode
}) => {
  const [state, setState] = useState<RobotControlState>({
    isManualMode: false,
    commandLogs: [],
    statistics: {
      total: 0,
      successful: 0,
      failed: 0,
      pending: 0,
      successRate: 0
    },
    lastError: null,
    isLoading: false
  });

  // IPC通信でデータを取得
  const fetchCommandLogs = useCallback(async () => {
    try {
      const result = await ipcRenderer.invoke('robot-command-get-logs');
      if (result.success) {
        setState(prev => ({ ...prev, commandLogs: result.logs }));
      }
    } catch (error) {
      console.error('Failed to fetch command logs:', error);
    }
  }, []);

  const fetchStatistics = useCallback(async () => {
    try {
      const result = await ipcRenderer.invoke('robot-command-get-statistics');
      if (result.success) {
        setState(prev => ({ ...prev, statistics: result.statistics }));
      }
    } catch (error) {
      console.error('Failed to fetch statistics:', error);
    }
  }, []);

  // 初期データ読み込み
  useEffect(() => {
    fetchCommandLogs();
    fetchStatistics();
  }, [fetchCommandLogs, fetchStatistics]);

  // IPCイベントリスナー
  useEffect(() => {
    const handleCommandSent = (data: RobotCommandLog) => {
      setState(prev => ({
        ...prev,
        commandLogs: [data, ...prev.commandLogs.filter(log => log.id !== data.id)]
      }));
      fetchStatistics();
    };

    const handleCommandSuccess = (data: RobotCommandLog) => {
      setState(prev => ({
        ...prev,
        commandLogs: prev.commandLogs.map(log => 
          log.id === data.id ? data : log
        ),
        lastError: null
      }));
      fetchStatistics();
    };

    const handleCommandError = (data: RobotCommandLog) => {
      setState(prev => ({
        ...prev,
        commandLogs: prev.commandLogs.map(log => 
          log.id === data.id ? data : log
        ),
        lastError: data.error || 'Unknown error'
      }));
      fetchStatistics();
    };

    const handleThresholdCommandError = (data: { reason: string; error: Error }) => {
      setState(prev => ({
        ...prev,
        lastError: `閾値コマンドエラー (${data.reason}): ${data.error.message}`
      }));
    };

    // IPCイベントリスナーを登録
    ipcRenderer.on('robot-command-sent', (event, data) => handleCommandSent(data));
    ipcRenderer.on('robot-command-success', (event, data) => handleCommandSuccess(data));
    ipcRenderer.on('robot-command-error', (event, data) => handleCommandError(data));
    ipcRenderer.on('threshold-command-error', (event, data) => handleThresholdCommandError(data));

    return () => {
      // クリーンアップ
      ipcRenderer.removeAllListeners('robot-command-sent');
      ipcRenderer.removeAllListeners('robot-command-success');
      ipcRenderer.removeAllListeners('robot-command-error');
      ipcRenderer.removeAllListeners('threshold-command-error');
    };
  }, []);

  // 手動コマンド送信
  const handleManualCommand = async (type: 'tool_handover' | 'next_task') => {
    if (!isConnected) {
      setState(prev => ({ ...prev, lastError: 'WebSocket接続が確立されていません' }));
      return;
    }

    setState(prev => ({ ...prev, isLoading: true, lastError: null }));
    
    try {
      const result = await ipcRenderer.invoke('robot-command-send', type, { 
        manual: true, 
        timestamp: new Date().toISOString() 
      });
      
      if (!result.success) {
        throw new Error(result.error || 'コマンド送信に失敗しました');
      }
    } catch (error) {
      setState(prev => ({ 
        ...prev, 
        lastError: error instanceof Error ? error.message : 'コマンド送信に失敗しました' 
      }));
    } finally {
      setState(prev => ({ ...prev, isLoading: false }));
    }
  };

  // コマンドリトライ
  const handleRetryCommand = async (commandId: string) => {
    setState(prev => ({ ...prev, isLoading: true, lastError: null }));
    
    try {
      const result = await ipcRenderer.invoke('robot-command-retry', commandId);
      
      if (!result.success) {
        throw new Error(result.error || 'リトライに失敗しました');
      }
    } catch (error) {
      setState(prev => ({ 
        ...prev, 
        lastError: error instanceof Error ? error.message : 'リトライに失敗しました' 
      }));
    } finally {
      setState(prev => ({ ...prev, isLoading: false }));
    }
  };

  // コマンドキャンセル
  const handleCancelCommand = async (commandId: string) => {
    try {
      const result = await ipcRenderer.invoke('robot-command-cancel', commandId);
      
      if (!result.success) {
        throw new Error(result.error || 'キャンセルに失敗しました');
      }
    } catch (error) {
      setState(prev => ({ 
        ...prev, 
        lastError: error instanceof Error ? error.message : 'キャンセルに失敗しました' 
      }));
    }
  };

  // 手動モード切り替え
  const handleToggleManualMode = () => {
    const newManualMode = !state.isManualMode;
    setState(prev => ({ ...prev, isManualMode: newManualMode }));
    onToggleManualMode(newManualMode);
  };

  // エラーをクリア
  const clearError = () => {
    setState(prev => ({ ...prev, lastError: null }));
  };

  // ログをクリア
  const handleClearLogs = async () => {
    try {
      const result = await ipcRenderer.invoke('robot-command-clear-logs');
      
      if (!result.success) {
        throw new Error(result.error || 'ログクリアに失敗しました');
      }
      
      setState(prev => ({ ...prev, commandLogs: [] }));
      fetchStatistics();
    } catch (error) {
      setState(prev => ({ 
        ...prev, 
        lastError: error instanceof Error ? error.message : 'ログクリアに失敗しました' 
      }));
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'success': return '✅';
      case 'error': return '❌';
      case 'pending': return '⏳';
      default: return '❓';
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'success': return '#4CAF50';
      case 'error': return '#F44336';
      case 'pending': return '#FF9800';
      default: return '#9E9E9E';
    }
  };

  return (
    <div className="robot-control-panel">
      <div className="control-header">
        <h3>🤖 ロボット制御パネル</h3>
        <div className="connection-status">
          <span className={`status-indicator ${isConnected ? 'connected' : 'disconnected'}`}>
            {isConnected ? '🟢 接続中' : '🔴 切断中'}
          </span>
        </div>
      </div>

      {/* エラー表示 */}
      {state.lastError && (
        <div className="error-banner">
          <div className="error-content">
            <span className="error-icon">⚠️</span>
            <span className="error-message">{state.lastError}</span>
            <button className="error-close" onClick={clearError}>×</button>
          </div>
        </div>
      )}

      {/* 手動制御モード */}
      <div className="manual-control-section">
        <div className="mode-toggle">
          <label className="toggle-switch">
            <input
              type="checkbox"
              checked={state.isManualMode}
              onChange={handleToggleManualMode}
            />
            <span className="toggle-slider"></span>
          </label>
          <span className="mode-label">
            {state.isManualMode ? '🔧 手動制御モード' : '🤖 自動制御モード'}
          </span>
        </div>

        {state.isManualMode && (
          <div className="manual-commands">
            <button
              className="command-button tool-handover"
              onClick={() => handleManualCommand('tool_handover')}
              disabled={!isConnected || state.isLoading}
            >
              🔧 工具受け渡し指示
            </button>
            <button
              className="command-button next-task"
              onClick={() => handleManualCommand('next_task')}
              disabled={!isConnected || state.isLoading}
            >
              ➡️ 次タスク指示
            </button>
          </div>
        )}
      </div>

      {/* 統計情報 */}
      <div className="statistics-section">
        <h4>📊 コマンド統計</h4>
        <div className="stats-grid">
          <div className="stat-item">
            <span className="stat-label">総数</span>
            <span className="stat-value">{state.statistics.total}</span>
          </div>
          <div className="stat-item success">
            <span className="stat-label">成功</span>
            <span className="stat-value">{state.statistics.successful}</span>
          </div>
          <div className="stat-item error">
            <span className="stat-label">失敗</span>
            <span className="stat-value">{state.statistics.failed}</span>
          </div>
          <div className="stat-item pending">
            <span className="stat-label">実行中</span>
            <span className="stat-value">{state.statistics.pending}</span>
          </div>
          <div className="stat-item rate">
            <span className="stat-label">成功率</span>
            <span className="stat-value">{state.statistics.successRate.toFixed(1)}%</span>
          </div>
        </div>
      </div>

      {/* コマンドログ */}
      <div className="command-logs-section">
        <div className="logs-header">
          <h4>📝 コマンドログ</h4>
          <button 
            className="clear-logs-button"
            onClick={handleClearLogs}
            disabled={!state.commandLogs || state.commandLogs.length === 0}
          >
            🗑️ ログクリア
          </button>
        </div>
        
        <div className="logs-container">
          {!state.commandLogs || state.commandLogs.length === 0 ? (
            <div className="no-logs">コマンドログがありません</div>
          ) : (
            (state.commandLogs || []).slice(0, 10).map((log) => (
              <div key={log.id} className="log-entry">
                <div className="log-header">
                  <span className="log-status" style={{ color: getStatusColor(log.status) }}>
                    {getStatusIcon(log.status)} {log.command.type}
                  </span>
                  <span className="log-time">
                    {log.timestamp.toLocaleTimeString()}
                  </span>
                </div>
                
                <div className="log-details">
                  <div className="log-id">ID: {log.id}</div>
                  {log.retryCount > 0 && (
                    <div className="log-retry">リトライ: {log.retryCount}回</div>
                  )}
                  {log.error && (
                    <div className="log-error">エラー: {log.error}</div>
                  )}
                  {log.response && (
                    <div className="log-response">
                      レスポンス: {log.response.message}
                    </div>
                  )}
                </div>

                {log.status === 'error' && (
                  <div className="log-actions">
                    <button
                      className="retry-button"
                      onClick={() => handleRetryCommand(log.id)}
                      disabled={state.isLoading || !isConnected}
                    >
                      🔄 リトライ
                    </button>
                  </div>
                )}

                {log.status === 'pending' && (
                  <div className="log-actions">
                    <button
                      className="cancel-button"
                      onClick={() => handleCancelCommand(log.id)}
                      disabled={state.isLoading}
                    >
                      ❌ キャンセル
                    </button>
                  </div>
                )}
              </div>
            ))
          )}
        </div>
      </div>
    </div>
  );
};