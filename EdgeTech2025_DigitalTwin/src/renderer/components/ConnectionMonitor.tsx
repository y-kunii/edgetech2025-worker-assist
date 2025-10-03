import React, { useState, useEffect } from 'react';
import { WorkDataStore } from '../../stores/WorkDataStore';
import { ConnectionQuality } from '../../types';
import './ConnectionMonitor.css';

interface ConnectionSettings {
  websocketUrl: string;
  reconnectInterval: number;
  maxReconnectAttempts: number;
  heartbeatInterval: number;
}

interface ConnectionMonitorProps {
  workDataStore: WorkDataStore;
  isVisible: boolean;
  onClose: () => void;
}

const ConnectionMonitor: React.FC<ConnectionMonitorProps> = ({
  workDataStore,
  isVisible,
  onClose
}) => {
  const [connectionSettings, setConnectionSettings] = useState<ConnectionSettings>({
    websocketUrl: 'ws://localhost:3001',
    reconnectInterval: 5000,
    maxReconnectAttempts: 10,
    heartbeatInterval: 30000
  });

  const [tempSettings, setTempSettings] = useState<ConnectionSettings>({
    websocketUrl: 'ws://localhost:3001',
    reconnectInterval: 5000,
    maxReconnectAttempts: 10,
    heartbeatInterval: 30000
  });

  const [connectionQuality, setConnectionQuality] = useState<ConnectionQuality | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [hasChanges, setHasChanges] = useState(false);
  const [connectionStats, setConnectionStats] = useState({
    messagesReceived: 0,
    messagesPerSecond: 0,
    averageLatency: 0,
    lastMessageTime: null as Date | null,
    connectionUptime: 0,
    reconnectCount: 0
  });

  // Load settings from localStorage on mount
  useEffect(() => {
    const savedSettings = loadConnectionSettings();
    setConnectionSettings(savedSettings);
    setTempSettings(savedSettings);
  }, []);

  // Listen to store events
  useEffect(() => {
    const handleConnectionStatusChange = (connected: boolean) => {
      setIsConnected(connected);
    };

    const handleConnectionQualityUpdate = (quality: ConnectionQuality) => {
      setConnectionQuality(quality);
    };

    workDataStore.on('connection_status_changed', handleConnectionStatusChange);
    workDataStore.on('connection_quality_updated', handleConnectionQualityUpdate);

    // Get initial state
    const state = workDataStore.getState();
    setIsConnected(state.isConnected);
    setConnectionQuality(state.connectionQuality);

    return () => {
      workDataStore.off('connection_status_changed', handleConnectionStatusChange);
      workDataStore.off('connection_quality_updated', handleConnectionQualityUpdate);
    };
  }, [workDataStore]);

  // Check for changes
  useEffect(() => {
    const hasChanges = 
      tempSettings.websocketUrl !== connectionSettings.websocketUrl ||
      tempSettings.reconnectInterval !== connectionSettings.reconnectInterval ||
      tempSettings.maxReconnectAttempts !== connectionSettings.maxReconnectAttempts ||
      tempSettings.heartbeatInterval !== connectionSettings.heartbeatInterval;
    setHasChanges(hasChanges);
  }, [tempSettings, connectionSettings]);

  // Update connection stats periodically
  useEffect(() => {
    const interval = setInterval(() => {
      if (connectionQuality) {
        setConnectionStats(prev => ({
          ...prev,
          messagesPerSecond: connectionQuality.dataRate,
          averageLatency: connectionQuality.latency,
          lastMessageTime: connectionQuality.lastUpdated,
          connectionUptime: isConnected ? prev.connectionUptime + 1 : 0
        }));
      }
    }, 1000);

    return () => clearInterval(interval);
  }, [connectionQuality, isConnected]);

  const loadConnectionSettings = (): ConnectionSettings => {
    const defaultSettings: ConnectionSettings = {
      websocketUrl: 'ws://localhost:3001',
      reconnectInterval: 5000,
      maxReconnectAttempts: 10,
      heartbeatInterval: 30000
    };

    try {
      if (typeof window !== 'undefined' && window.localStorage) {
        const saved = localStorage.getItem('manufacturing-twin-connection-settings');
        if (saved) {
          const parsed = JSON.parse(saved);
          return { ...defaultSettings, ...parsed };
        }
      }
    } catch (error) {
      console.warn('Failed to load connection settings:', error);
    }

    return defaultSettings;
  };

  const saveConnectionSettings = (settings: ConnectionSettings): void => {
    try {
      if (typeof window !== 'undefined' && window.localStorage) {
        localStorage.setItem('manufacturing-twin-connection-settings', JSON.stringify(settings));
      }
    } catch (error) {
      console.warn('Failed to save connection settings:', error);
    }
  };

  const handleUrlChange = (url: string) => {
    setTempSettings(prev => ({ ...prev, websocketUrl: url }));
  };

  const handleReconnectIntervalChange = (interval: number) => {
    const clampedValue = Math.max(1000, Math.min(60000, interval));
    setTempSettings(prev => ({ ...prev, reconnectInterval: clampedValue }));
  };

  const handleMaxReconnectAttemptsChange = (attempts: number) => {
    const clampedValue = Math.max(1, Math.min(100, attempts));
    setTempSettings(prev => ({ ...prev, maxReconnectAttempts: clampedValue }));
  };

  const handleHeartbeatIntervalChange = (interval: number) => {
    const clampedValue = Math.max(5000, Math.min(300000, interval));
    setTempSettings(prev => ({ ...prev, heartbeatInterval: clampedValue }));
  };

  const handleSave = () => {
    setConnectionSettings(tempSettings);
    saveConnectionSettings(tempSettings);
    setHasChanges(false);
    
    // Emit event to notify about settings change
    workDataStore.emit('connection_settings_updated', tempSettings);
    onClose();
  };

  const handleCancel = () => {
    setTempSettings(connectionSettings);
    setHasChanges(false);
    onClose();
  };

  const handleReset = () => {
    const defaultSettings: ConnectionSettings = {
      websocketUrl: 'ws://localhost:3001',
      reconnectInterval: 5000,
      maxReconnectAttempts: 10,
      heartbeatInterval: 30000
    };
    setTempSettings(defaultSettings);
  };

  const getConnectionStatusText = () => {
    if (!isConnected) return '🔴 切断中';
    if (!connectionQuality) return '🟡 接続中';
    
    switch (connectionQuality.stability) {
      case 'excellent': return '🟢 優秀';
      case 'good': return '🟢 良好';
      case 'fair': return '🟡 普通';
      case 'poor': return '🟠 不良';
      default: return '🔴 不明';
    }
  };

  const getConnectionStatusClass = () => {
    if (!isConnected) return 'status-disconnected';
    if (!connectionQuality) return 'status-connecting';
    
    switch (connectionQuality.stability) {
      case 'excellent':
      case 'good':
        return 'status-connected';
      case 'fair':
        return 'status-warning';
      case 'poor':
        return 'status-error';
      default:
        return 'status-disconnected';
    }
  };

  const formatUptime = (seconds: number): string => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  if (!isVisible) {
    return null;
  }

  return (
    <div className="connection-monitor-overlay">
      <div className="connection-monitor-modal">
        <div className="modal-header">
          <h3>🔗 接続設定・監視</h3>
          <button className="close-button" onClick={handleCancel}>
            ✕
          </button>
        </div>
        
        <div className="modal-content">
          {/* Connection Status Section */}
          <div className="section">
            <h4>接続状態</h4>
            <div className="status-grid">
              <div className={`status-card ${getConnectionStatusClass()}`}>
                <div className="status-label">接続状態</div>
                <div className="status-value">{getConnectionStatusText()}</div>
              </div>
              
              <div className="status-card">
                <div className="status-label">遅延</div>
                <div className="status-value">
                  {connectionQuality ? `${connectionQuality.latency}ms` : '---'}
                </div>
              </div>
              
              <div className="status-card">
                <div className="status-label">データレート</div>
                <div className="status-value">
                  {connectionQuality ? `${connectionQuality.dataRate.toFixed(1)}/秒` : '---'}
                </div>
              </div>
              
              <div className="status-card">
                <div className="status-label">稼働時間</div>
                <div className="status-value">{formatUptime(connectionStats.connectionUptime)}</div>
              </div>
            </div>
          </div>

          {/* Connection Settings Section */}
          <div className="section">
            <h4>接続設定</h4>
            
            <div className="setting-group">
              <label htmlFor="websocket-url">WebSocket URL</label>
              <input
                id="websocket-url"
                type="text"
                value={tempSettings.websocketUrl}
                onChange={(e) => handleUrlChange(e.target.value)}
                placeholder="ws://localhost:3001"
              />
              <div className="setting-description">
                接続先のWebSocketサーバーのURL
              </div>
            </div>

            <div className="setting-group">
              <label htmlFor="reconnect-interval">再接続間隔 (ms)</label>
              <div className="input-group">
                <button 
                  className="decrement-button"
                  onClick={() => handleReconnectIntervalChange(tempSettings.reconnectInterval - 1000)}
                  disabled={tempSettings.reconnectInterval <= 1000}
                >
                  -
                </button>
                <input
                  id="reconnect-interval"
                  type="number"
                  min="1000"
                  max="60000"
                  step="1000"
                  value={tempSettings.reconnectInterval}
                  onChange={(e) => handleReconnectIntervalChange(parseInt(e.target.value) || 1000)}
                />
                <button 
                  className="increment-button"
                  onClick={() => handleReconnectIntervalChange(tempSettings.reconnectInterval + 1000)}
                  disabled={tempSettings.reconnectInterval >= 60000}
                >
                  +
                </button>
              </div>
              <div className="setting-description">
                接続が切断された際の再接続試行間隔 (1-60秒)
              </div>
            </div>

            <div className="setting-group">
              <label htmlFor="max-reconnect-attempts">最大再接続試行回数</label>
              <div className="input-group">
                <button 
                  className="decrement-button"
                  onClick={() => handleMaxReconnectAttemptsChange(tempSettings.maxReconnectAttempts - 1)}
                  disabled={tempSettings.maxReconnectAttempts <= 1}
                >
                  -
                </button>
                <input
                  id="max-reconnect-attempts"
                  type="number"
                  min="1"
                  max="100"
                  value={tempSettings.maxReconnectAttempts}
                  onChange={(e) => handleMaxReconnectAttemptsChange(parseInt(e.target.value) || 1)}
                />
                <button 
                  className="increment-button"
                  onClick={() => handleMaxReconnectAttemptsChange(tempSettings.maxReconnectAttempts + 1)}
                  disabled={tempSettings.maxReconnectAttempts >= 100}
                >
                  +
                </button>
              </div>
              <div className="setting-description">
                再接続を諦めるまでの最大試行回数 (1-100回)
              </div>
            </div>

            <div className="setting-group">
              <label htmlFor="heartbeat-interval">ハートビート間隔 (ms)</label>
              <div className="input-group">
                <button 
                  className="decrement-button"
                  onClick={() => handleHeartbeatIntervalChange(tempSettings.heartbeatInterval - 5000)}
                  disabled={tempSettings.heartbeatInterval <= 5000}
                >
                  -
                </button>
                <input
                  id="heartbeat-interval"
                  type="number"
                  min="5000"
                  max="300000"
                  step="5000"
                  value={tempSettings.heartbeatInterval}
                  onChange={(e) => handleHeartbeatIntervalChange(parseInt(e.target.value) || 5000)}
                />
                <button 
                  className="increment-button"
                  onClick={() => handleHeartbeatIntervalChange(tempSettings.heartbeatInterval + 5000)}
                  disabled={tempSettings.heartbeatInterval >= 300000}
                >
                  +
                </button>
              </div>
              <div className="setting-description">
                接続確認のためのハートビート送信間隔 (5-300秒)
              </div>
            </div>
          </div>
        </div>

        <div className="modal-footer">
          <button className="reset-button" onClick={handleReset}>
            デフォルトに戻す
          </button>
          <div className="action-buttons">
            <button className="cancel-button" onClick={handleCancel}>
              キャンセル
            </button>
            <button 
              className="save-button" 
              onClick={handleSave}
              disabled={!hasChanges}
            >
              保存
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ConnectionMonitor;