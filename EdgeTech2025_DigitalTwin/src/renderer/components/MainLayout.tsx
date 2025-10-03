import React, { useState, useEffect, useRef, memo } from 'react';
import LiveVideoDisplay from './LiveVideoDisplay';
import StatusIndicator from './StatusIndicator';
import ProgressChart from './ProgressChart';
import WorkStepDisplay from './WorkStepDisplay';
import TimelineAndStats from './TimelineAndStats';
import CurrentTime from './CurrentTime';
import { RobotControlPanel } from './RobotControlPanel';
import ThresholdSettings from './ThresholdSettings';
import ConnectionMonitor from './ConnectionMonitor';
import PerformanceDashboard from './PerformanceDashboard';
import { WorkerStatus, RobotStatus, WorkHistory, WorkStatistics, EfficiencyMetrics } from '../../types';
import { WorkStepManager } from '../../utils/workStepManager';
import { WorkDataStore } from '../../stores/WorkDataStore';
import { AnimationManager } from '../../utils/animationManager';
import { DeviceCapabilities } from '../../utils/performanceOptimizer';

interface MainLayoutProps {
  // Connection status
  isConnected: boolean;
  connectionQuality: 'good' | 'fair' | 'poor';
  
  // Live video data
  currentImage?: string;
  
  // Worker and robot status
  workerStatus: WorkerStatus;
  robotStatus: RobotStatus;
  isWorkerActive: boolean;
  isRobotActive: boolean;
  
  // Progress data
  screwCount: number;
  screwTarget: number;
  boltCount: number;
  boltTarget: number;
  
  // Work step data
  currentWorkStep: WorkerStatus;
  workStepManager?: WorkStepManager;
  
  // Overall efficiency
  overallEfficiency: number;
  
  // Notification count
  notificationCount: number;
  
  // Timeline and statistics data
  workHistory: WorkHistory[];
  statistics: WorkStatistics;
  efficiencyMetrics: EfficiencyMetrics;
  
  // Data store for settings
  workDataStore?: WorkDataStore;
  
  // Performance optimization
  animationManager?: AnimationManager;
  deviceCapabilities?: Record<string, boolean>;
}

const MainLayout: React.FC<MainLayoutProps> = memo(({
  isConnected,
  connectionQuality,
  currentImage,
  workerStatus,
  robotStatus,
  isWorkerActive,
  isRobotActive,
  screwCount,
  screwTarget,
  boltCount,
  boltTarget,
  currentWorkStep,
  workStepManager,
  overallEfficiency,
  notificationCount,
  workHistory,
  statistics,
  efficiencyMetrics,
  workDataStore,
  animationManager,
  deviceCapabilities
}) => {
  const [showRobotControl, setShowRobotControl] = useState(false);
  const [showThresholdSettings, setShowThresholdSettings] = useState(false);
  const [showConnectionMonitor, setShowConnectionMonitor] = useState(false);
  const [showPerformanceDashboard, setShowPerformanceDashboard] = useState(false);
  const [isManualMode, setIsManualMode] = useState(false);
  
  // Refs for animation targets
  const headerRef = useRef<HTMLElement>(null);
  const efficiencyRef = useRef<HTMLDivElement>(null);
  const connectionStatusRef = useRef<HTMLDivElement>(null);
  const getConnectionStatusText = () => {
    if (!isConnected) return '切断';
    switch (connectionQuality) {
      case 'good': return '接続良好';
      case 'fair': return '接続普通';
      case 'poor': return '接続不良';
      default: return '不明';
    }
  };

  const getConnectionStatusClass = () => {
    if (!isConnected) return 'status-disconnected';
    switch (connectionQuality) {
      case 'good': return 'status-connected';
      case 'fair': return 'status-warning';
      case 'poor': return 'status-warning';
      default: return 'status-disconnected';
    }
  };

  const isScrewThresholdReached = screwCount >= screwTarget;
  const isBoltThresholdReached = boltCount >= boltTarget;

  const handleToggleRobotControl = () => {
    setShowRobotControl(!showRobotControl);
  };

  const handleToggleThresholdSettings = () => {
    setShowThresholdSettings(!showThresholdSettings);
  };

  const handleToggleConnectionMonitor = () => {
    setShowConnectionMonitor(!showConnectionMonitor);
  };

  const handleTogglePerformanceDashboard = () => {
    setShowPerformanceDashboard(!showPerformanceDashboard);
  };

  const handleToggleManualMode = (enabled: boolean) => {
    setIsManualMode(enabled);
  };

  // Removed animations for cleaner design

  return (
    <div className="app-container">
      {/* Header */}
      <header className="app-header" ref={headerRef}>
        <div className="header-title">
          <span>製造現場デジタルツイン</span>
        </div>
        
        <div className="header-stats">
          <div className="stat-item" ref={efficiencyRef}>
            <span>効率:</span>
            <span className="stat-value">{overallEfficiency.toFixed(1)}%</span>
          </div>
          
          <div 
            className={`status-indicator ${getConnectionStatusClass()}`}
            ref={connectionStatusRef}
          >
            接続: {getConnectionStatusText()}
          </div>
          
          <CurrentTime format="full" />
        </div>
      </header>

      {/* Main Content Grid */}
      <main className="main-content">
        <LiveVideoDisplay 
          image={currentImage}
          isConnected={isConnected}
        />
        
        <StatusIndicator 
          type="worker"
          status={workerStatus}
          isActive={isWorkerActive}
        />
        
        <StatusIndicator 
          type="robot"
          status={robotStatus}
          isActive={isRobotActive}
        />
        
        <div className="progress-container">
          <ProgressChart 
            type="screw"
            current={screwCount}
            target={screwTarget}
            isThresholdReached={isScrewThresholdReached}
            animationManager={animationManager}
            deviceCapabilities={deviceCapabilities}
          />
          
          <ProgressChart 
            type="bolt"
            current={boltCount}
            target={boltTarget}
            isThresholdReached={isBoltThresholdReached}
            animationManager={animationManager}
            deviceCapabilities={deviceCapabilities}
          />
        </div>
        
        <WorkStepDisplay 
          currentStep={currentWorkStep}
          stepManager={workStepManager}
        />
        
        <TimelineAndStats 
          workHistory={workHistory}
          statistics={statistics}
          efficiencyMetrics={efficiencyMetrics}
          timeRangeMinutes={30}
          maxDataPoints={deviceCapabilities?.highPerformance ? 100 : 50}
          deviceCapabilities={deviceCapabilities}
        />
      </main>

      {/* Footer */}
      <footer className="app-footer">
        <div className="footer-left">
          <button 
            className="status-indicator"
            onClick={handleToggleThresholdSettings}
            style={{ 
              background: showThresholdSettings ? 'var(--color-info)' : 'var(--color-light-gray)',
              border: '1px solid var(--color-info)',
              color: showThresholdSettings ? 'white' : 'var(--color-info)',
              cursor: 'pointer'
            }}
          >
            設定
          </button>
          
          <button 
            className="status-indicator"
            onClick={handleToggleRobotControl}
            style={{ 
              background: showRobotControl ? 'var(--color-info)' : 'var(--color-light-gray)',
              border: '1px solid var(--color-info)',
              color: showRobotControl ? 'white' : 'var(--color-info)',
              cursor: 'pointer'
            }}
          >
            ロボット制御
          </button>
          
          <button 
            className={`status-indicator ${getConnectionStatusClass()}`}
            onClick={handleToggleConnectionMonitor}
            style={{ 
              background: showConnectionMonitor ? 'var(--color-info)' : undefined,
              color: showConnectionMonitor ? 'white' : undefined,
              cursor: 'pointer'
            }}
          >
            接続状態: {isConnected ? '接続中' : '切断中'}
          </button>
          
          {isManualMode && (
            <div className="status-indicator" style={{ 
              background: 'var(--color-warning)',
              border: '1px solid var(--color-warning)',
              color: 'white'
            }}>
              手動制御モード
            </div>
          )}
        </div>
        
        <div className="footer-right">
          <button 
            className="status-indicator"
            onClick={handleTogglePerformanceDashboard}
            style={{ 
              background: showPerformanceDashboard ? 'var(--color-info)' : 'var(--color-light-gray)',
              border: '1px solid var(--color-info)',
              color: showPerformanceDashboard ? 'white' : 'var(--color-info)',
              cursor: 'pointer'
            }}
          >
            パフォーマンス
          </button>
          
          <button 
            className="status-indicator"
            style={{ 
              background: 'var(--color-light-gray)',
              border: '1px solid var(--color-info)',
              color: 'var(--color-info)',
              cursor: 'pointer'
            }}
          >
            ログ
          </button>
          
          <button 
            className="status-indicator"
            style={{ 
              background: notificationCount > 0 ? 'var(--color-warning)' : 'var(--color-light-gray)',
              border: `1px solid ${notificationCount > 0 ? 'var(--color-warning)' : 'var(--color-info)'}`,
              color: notificationCount > 0 ? 'white' : 'var(--color-info)',
              cursor: 'pointer'
            }}
          >
            通知: {notificationCount}件
          </button>
        </div>
      </footer>

      {/* Robot Control Panel Modal */}
      {showRobotControl && (
        <div className="robot-control-overlay">
          <div className="robot-control-modal">
            <div className="modal-header">
              <h3>ロボット制御パネル</h3>
              <button 
                className="close-button"
                onClick={handleToggleRobotControl}
              >
                ×
              </button>
            </div>
            <div className="modal-content">
              <RobotControlPanel
                isConnected={isConnected}
                onToggleManualMode={handleToggleManualMode}
              />
            </div>
          </div>
        </div>
      )}

      {/* Threshold Settings Modal */}
      {showThresholdSettings && workDataStore && (
        <ThresholdSettings
          workDataStore={workDataStore}
          isVisible={showThresholdSettings}
          onClose={handleToggleThresholdSettings}
        />
      )}

      {/* Connection Monitor Modal */}
      {showConnectionMonitor && workDataStore && (
        <ConnectionMonitor
          workDataStore={workDataStore}
          isVisible={showConnectionMonitor}
          onClose={handleToggleConnectionMonitor}
        />
      )}

      {/* Performance Dashboard Modal */}
      {showPerformanceDashboard && (
        <PerformanceDashboard
          isVisible={showPerformanceDashboard}
          onClose={handleTogglePerformanceDashboard}
        />
      )}
    </div>
  );
});

export default MainLayout;