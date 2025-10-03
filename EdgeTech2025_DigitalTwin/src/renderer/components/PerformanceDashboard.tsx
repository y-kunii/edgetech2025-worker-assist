/**
 * Performance Dashboard Component
 * Displays real-time performance metrics and system status
 */

import React, { useState, useEffect, useRef } from 'react';
import { PerformanceMonitor, DeviceCapabilities } from '../../utils/performanceOptimizer';
import { DataIntegrationManager } from '../../utils/dataIntegrationManager';
import { AnimationManager } from '../../utils/animationManager';

interface PerformanceDashboardProps {
  isVisible: boolean;
  onClose: () => void;
}

interface PerformanceMetrics {
  memory: {
    current: number;
    average: number;
    peak: number;
  };
  frameRate: {
    current: number;
    average: number;
    min: number;
  };
  operations: Record<string, {
    average: number;
    min: number;
    max: number;
    count: number;
  }>;
  animations: {
    activeAnimations: number;
    queuedAnimations: number;
    performanceMode: string;
    animationsEnabled: boolean;
  };
  dataIntegration: {
    cacheSize: number;
    lastUpdateTimes: Record<string, number>;
  };
}

const PerformanceDashboard: React.FC<PerformanceDashboardProps> = ({
  isVisible,
  onClose
}) => {
  const [metrics, setMetrics] = useState<PerformanceMetrics | null>(null);
  const [deviceCapabilities, setDeviceCapabilities] = useState<Record<string, boolean>>({});
  const intervalRef = useRef<number | null>(null);

  useEffect(() => {
    if (isVisible) {
      // Detect device capabilities
      setDeviceCapabilities(DeviceCapabilities.detect());
      
      // Start metrics collection
      const collectMetrics = () => {
        const performanceMonitor = PerformanceMonitor.getInstance();
        const dataIntegrationManager = DataIntegrationManager.getInstance();
        const animationManager = AnimationManager.getInstance();

        const performanceMetrics = performanceMonitor.getAllMetrics();
        const animationStats = animationManager.getStatistics();

        const newMetrics: PerformanceMetrics = {
          memory: performanceMetrics.memory || { current: 0, average: 0, peak: 0 },
          frameRate: performanceMetrics.frameRate || { current: 0, average: 0, min: 0 },
          operations: performanceMetrics.operations || {},
          animations: animationStats,
          dataIntegration: {
            cacheSize: Object.keys(dataIntegrationManager.getCachedData('') || {}).length,
            lastUpdateTimes: {
              workData: dataIntegrationManager.getLastUpdateTime('currentWorkData') || 0,
              connectionStatus: dataIntegrationManager.getLastUpdateTime('connectionStatus') || 0,
              thresholdSettings: dataIntegrationManager.getLastUpdateTime('thresholdSettings') || 0
            }
          }
        };

        setMetrics(newMetrics);
      };

      // Collect metrics immediately
      collectMetrics();

      // Set up periodic collection
      intervalRef.current = window.setInterval(collectMetrics, 1000);
    }

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, [isVisible]);

  const formatBytes = (bytes: number): string => {
    if (bytes === 0) return '0 B';
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
  };

  const formatTime = (ms: number): string => {
    if (ms < 1000) return `${ms.toFixed(1)}ms`;
    return `${(ms / 1000).toFixed(1)}s`;
  };

  const getPerformanceStatus = (value: number, thresholds: { good: number; fair: number }): string => {
    if (value <= thresholds.good) return 'excellent';
    if (value <= thresholds.fair) return 'good';
    return 'poor';
  };

  if (!isVisible) return null;

  return (
    <div className="performance-dashboard-overlay">
      <div className="performance-dashboard-modal">
        <div className="modal-header">
          <h3>📊 パフォーマンスダッシュボード</h3>
          <button className="close-button" onClick={onClose}>×</button>
        </div>
        
        <div className="modal-content performance-content">
          {metrics ? (
            <>
              {/* Device Capabilities */}
              <div className="performance-section">
                <h4>🖥️ デバイス性能</h4>
                <div className="capabilities-grid">
                  <div className={`capability-item ${deviceCapabilities.webgl ? 'enabled' : 'disabled'}`}>
                    <span className="capability-label">WebGL:</span>
                    <span className="capability-value">{deviceCapabilities.webgl ? '有効' : '無効'}</span>
                  </div>
                  <div className={`capability-item ${deviceCapabilities.highMemory ? 'enabled' : 'disabled'}`}>
                    <span className="capability-label">高メモリ:</span>
                    <span className="capability-value">{deviceCapabilities.highMemory ? '有効' : '無効'}</span>
                  </div>
                  <div className={`capability-item ${deviceCapabilities.highPerformance ? 'enabled' : 'disabled'}`}>
                    <span className="capability-label">高性能:</span>
                    <span className="capability-value">{deviceCapabilities.highPerformance ? '有効' : '無効'}</span>
                  </div>
                  <div className={`capability-item ${deviceCapabilities.touchSupport ? 'enabled' : 'disabled'}`}>
                    <span className="capability-label">タッチ:</span>
                    <span className="capability-value">{deviceCapabilities.touchSupport ? '対応' : '非対応'}</span>
                  </div>
                </div>
              </div>

              {/* Memory Usage */}
              <div className="performance-section">
                <h4>💾 メモリ使用量</h4>
                <div className="metrics-grid">
                  <div className="metric-item">
                    <span className="metric-label">現在:</span>
                    <span className="metric-value">{formatBytes(metrics.memory.current)}</span>
                  </div>
                  <div className="metric-item">
                    <span className="metric-label">平均:</span>
                    <span className="metric-value">{formatBytes(metrics.memory.average)}</span>
                  </div>
                  <div className="metric-item">
                    <span className="metric-label">ピーク:</span>
                    <span className="metric-value">{formatBytes(metrics.memory.peak)}</span>
                  </div>
                </div>
              </div>

              {/* Frame Rate */}
              <div className="performance-section">
                <h4>🎬 フレームレート</h4>
                <div className="metrics-grid">
                  <div className="metric-item">
                    <span className="metric-label">現在:</span>
                    <span className={`metric-value ${getPerformanceStatus(60 - metrics.frameRate.current, { good: 10, fair: 20 })}`}>
                      {metrics.frameRate.current.toFixed(1)} FPS
                    </span>
                  </div>
                  <div className="metric-item">
                    <span className="metric-label">平均:</span>
                    <span className="metric-value">{metrics.frameRate.average.toFixed(1)} FPS</span>
                  </div>
                  <div className="metric-item">
                    <span className="metric-label">最小:</span>
                    <span className="metric-value">{metrics.frameRate.min.toFixed(1)} FPS</span>
                  </div>
                </div>
              </div>

              {/* Operation Performance */}
              <div className="performance-section">
                <h4>⚡ 操作パフォーマンス</h4>
                <div className="operations-list">
                  {Object.entries(metrics.operations).map(([operation, stats]) => (
                    <div key={operation} className="operation-item">
                      <div className="operation-name">{operation}</div>
                      <div className="operation-stats">
                        <span className={`stat-value ${getPerformanceStatus(stats.average, { good: 5, fair: 10 })}`}>
                          平均: {formatTime(stats.average)}
                        </span>
                        <span className="stat-value">
                          最小: {formatTime(stats.min)}
                        </span>
                        <span className="stat-value">
                          最大: {formatTime(stats.max)}
                        </span>
                        <span className="stat-value">
                          回数: {stats.count}
                        </span>
                      </div>
                    </div>
                  ))}
                </div>
              </div>

              {/* Animation Status */}
              <div className="performance-section">
                <h4>🎨 アニメーション状態</h4>
                <div className="metrics-grid">
                  <div className="metric-item">
                    <span className="metric-label">アクティブ:</span>
                    <span className="metric-value">{metrics.animations.activeAnimations}</span>
                  </div>
                  <div className="metric-item">
                    <span className="metric-label">キュー:</span>
                    <span className="metric-value">{metrics.animations.queuedAnimations}</span>
                  </div>
                  <div className="metric-item">
                    <span className="metric-label">モード:</span>
                    <span className="metric-value">{metrics.animations.performanceMode}</span>
                  </div>
                  <div className="metric-item">
                    <span className="metric-label">有効:</span>
                    <span className="metric-value">{metrics.animations.animationsEnabled ? 'はい' : 'いいえ'}</span>
                  </div>
                </div>
              </div>

              {/* Data Integration */}
              <div className="performance-section">
                <h4>🔄 データ統合</h4>
                <div className="metrics-grid">
                  <div className="metric-item">
                    <span className="metric-label">キャッシュサイズ:</span>
                    <span className="metric-value">{metrics.dataIntegration.cacheSize}</span>
                  </div>
                  {Object.entries(metrics.dataIntegration.lastUpdateTimes).map(([key, timestamp]) => (
                    <div key={key} className="metric-item">
                      <span className="metric-label">{key}:</span>
                      <span className="metric-value">
                        {timestamp > 0 ? `${Math.round((Date.now() - timestamp) / 1000)}秒前` : '未更新'}
                      </span>
                    </div>
                  ))}
                </div>
              </div>
            </>
          ) : (
            <div className="loading-metrics">
              <div className="loading-spinner"></div>
              <div>メトリクスを読み込み中...</div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default PerformanceDashboard;