import React from 'react';
import { WorkerStatus, RobotStatus } from '../../types';

interface StatusIndicatorProps {
  type: 'worker' | 'robot';
  status: WorkerStatus | RobotStatus;
  isActive: boolean;
}

const StatusIndicator: React.FC<StatusIndicatorProps> = ({ type, status, isActive }) => {
  const getStatusText = () => {
    if (type === 'worker') {
      const workerStatus = status as WorkerStatus;
      switch (workerStatus) {
        case 'waiting': return '待機中';
        case 'screw_tightening': return 'ネジ締め中';
        case 'bolt_tightening': return 'ボルト締め中';
        case 'tool_handover': return '工具受け渡し中';
        case 'absent': return '不在';
        default: return '不明';
      }
    } else {
      const robotStatus = status as RobotStatus;
      const robotState = typeof robotStatus === 'object' ? robotStatus.state : robotStatus;
      switch (robotState) {
        case 'waiting': return '待機中';
        case 'operating': return '稼働中';
        default: return '不明';
      }
    }
  };

  const getGripStatusText = () => {
    if (type === 'robot') {
      const robotStatus = status as RobotStatus;
      const gripState = typeof robotStatus === 'object' ? robotStatus.grip : 'closed';
      return gripState === 'open' ? 'グリップ: 開' : 'グリップ: 閉';
    }
    return '';
  };

  const getIndicatorColor = () => {
    if (type === 'worker') {
      const workerStatus = status as WorkerStatus;
      switch (workerStatus) {
        case 'screw_tightening':
        case 'bolt_tightening':
          return 'var(--color-success)';
        case 'tool_handover':
          return 'var(--color-info)';
        case 'absent':
          return 'var(--color-medium-gray)';
        default:
          return 'var(--color-medium-gray)';
      }
    } else {
      const robotStatus = status as RobotStatus;
      const robotState = typeof robotStatus === 'object' ? robotStatus.state : robotStatus;
      return robotState === 'operating' ? 'var(--color-success)' : 'var(--color-medium-gray)';
    }
  };

  const isActiveState = () => {
    if (type === 'worker') {
      const workerStatus = status as WorkerStatus;
      return workerStatus === 'screw_tightening' || workerStatus === 'bolt_tightening' || workerStatus === 'tool_handover';
    } else {
      const robotStatus = status as RobotStatus;
      const robotState = typeof robotStatus === 'object' ? robotStatus.state : robotStatus;
      return robotState === 'operating';
    }
  };

  const title = type === 'worker' ? '作業者状態' : 'ロボット状態';

  return (
    <div className={`content-area ${type === 'worker' ? 'worker-status-area' : 'robot-status-area'}`}>
      <div className="area-header">
        {title}
      </div>
      <div className="area-content">
        <div className="status-display">
          <div 
            className={`status-indicator-dot ${isActiveState() ? 'active' : 'inactive'}`}
            style={{ backgroundColor: getIndicatorColor() }}
          />
          <div className="status-text">
            <div 
              className="status-text-primary"
              style={{
                fontSize: 'calc(max(16px, 16px * var(--font-scale)))',
                fontWeight: '600',
                color: 'var(--color-text)',
                marginBottom: '4px'
              }}
            >
              {getStatusText()}
            </div>
            
            {type === 'robot' && (
              <div 
                className="status-text-secondary"
                style={{
                  fontSize: 'calc(max(14px, 14px * var(--font-scale)))',
                  color: 'var(--color-text-light)'
                }}
              >
                {getGripStatusText()}
              </div>
            )}
          </div>
        </div>
        
        <div 
          className="activity-indicator"
          style={{
            fontSize: 'calc(max(14px, 14px * var(--font-scale)))',
            color: 'var(--color-text-light)',
            marginTop: '16px',
            textAlign: 'center'
          }}
        >
          {isActiveState() ? '● 正常動作' : '○ 待機'}
        </div>
      </div>
    </div>
  );
};

export default StatusIndicator;