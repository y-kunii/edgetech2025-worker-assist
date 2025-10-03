import React, { useEffect, useState } from 'react';
import { WorkerStatus } from '../../types';
import { WorkStepManager } from '../../utils/workStepManager';

interface WorkStepDisplayProps {
  currentStep: WorkerStatus;
  stepManager?: WorkStepManager;
}

const WorkStepDisplay: React.FC<WorkStepDisplayProps> = ({ 
  currentStep,
  stepManager
}) => {
  const [timing, setTiming] = useState(stepManager?.getCurrentTiming() || {
    currentStep,
    stepStartTime: new Date(),
    elapsedTime: 0,
    totalStepTime: 0,
    stepCount: 0
  });
  
  const [efficiency, setEfficiency] = useState(stepManager?.calculateCurrentEfficiency() || {
    currentEfficiency: 100,
    averageStepTime: 0,
    expectedStepTime: 0,
    trend: 'stable' as const
  });

  // Update timing and efficiency every second
  useEffect(() => {
    const interval = setInterval(() => {
      if (stepManager) {
        stepManager.updateWorkStep(currentStep);
        const newTiming = stepManager.getCurrentTiming();
        const newEfficiency = stepManager.calculateCurrentEfficiency();
        
        setTiming(newTiming);
        setEfficiency(newEfficiency);
      } else {
        // Fallback for when no step manager is provided
        setTiming(prev => ({
          ...prev,
          currentStep,
          elapsedTime: prev.elapsedTime + 1
        }));
      }
    }, 1000);

    return () => clearInterval(interval);
  }, [currentStep, stepManager]);

  // Update step when it changes
  useEffect(() => {
    if (stepManager) {
      stepManager.updateWorkStep(currentStep);
      setTiming(stepManager.getCurrentTiming());
      setEfficiency(stepManager.calculateCurrentEfficiency());
    }
  }, [currentStep, stepManager]);

  const getStepStatusText = () => {
    if (timing.currentStep === 'absent') return '不在';
    if (timing.currentStep === 'waiting') return '待機中';
    return 'ステップ実行中';
  };

  const getTimeComparisonText = () => {
    if (efficiency.expectedStepTime === 0) return '';
    const remaining = Math.max(0, efficiency.expectedStepTime - timing.elapsedTime);
    if (remaining > 0) {
      return `予定まで ${WorkStepManager.formatTime(remaining)}`;
    } else {
      const overtime = timing.elapsedTime - efficiency.expectedStepTime;
      return `予定より ${WorkStepManager.formatTime(overtime)} 超過`;
    }
  };

  const getProgressPercentage = () => {
    if (efficiency.expectedStepTime === 0) return 0;
    return Math.min(100, (timing.elapsedTime / efficiency.expectedStepTime) * 100);
  };

  return (
    <div className="content-area work-step-area">
      <div className="area-header">
        現在の作業
      </div>
      <div className="area-content">
        {/* Current Step Display */}
        <div style={{ marginBottom: '24px' }}>
          <div style={{
            fontSize: 'calc(max(18px, 18px * var(--font-scale)))',
            fontWeight: '600',
            color: 'var(--color-text)',
            marginBottom: '8px',
            textAlign: 'center'
          }}>
            {WorkStepManager.getStepDisplayName(timing.currentStep)}
          </div>
          <div style={{
            fontSize: 'calc(max(14px, 14px * var(--font-scale)))',
            color: 'var(--color-text-light)',
            marginBottom: '12px',
            textAlign: 'center'
          }}>
            {getStepStatusText()}
          </div>
            
          {/* Progress bar for expected time */}
          {efficiency.expectedStepTime > 0 && (
            <div style={{ marginTop: '12px' }}>
              <div style={{
                width: '100%',
                height: '6px',
                backgroundColor: 'var(--color-light-gray)',
                borderRadius: '3px',
                overflow: 'hidden'
              }}>
                <div style={{
                  width: `${getProgressPercentage()}%`,
                  height: '100%',
                  backgroundColor: timing.elapsedTime <= efficiency.expectedStepTime 
                    ? 'var(--color-success)' 
                    : 'var(--color-warning)',
                  transition: 'width 0.2s ease'
                }} />
              </div>
              <div style={{
                fontSize: 'calc(max(14px, 14px * var(--font-scale)))',
                color: 'var(--color-text-light)',
                marginTop: '8px',
                textAlign: 'center'
              }}>
                {getTimeComparisonText()}
              </div>
            </div>
          )}
        </div>

        {/* Timing and Efficiency Grid */}
        <div style={{ 
          display: 'grid', 
          gridTemplateColumns: '1fr 1fr', 
          gap: '16px',
          width: '100%',
          marginBottom: '16px'
        }}>
          <div style={{ textAlign: 'center' }}>
            <div style={{
              fontSize: 'calc(max(14px, 14px * var(--font-scale)))',
              color: 'var(--color-text-light)',
              marginBottom: '6px'
            }}>
              経過時間
            </div>
            <div style={{
              fontSize: 'calc(max(20px, 20px * var(--font-scale)))',
              fontWeight: '700',
              color: 'var(--color-text)',
              fontFamily: 'monospace'
            }}>
              {WorkStepManager.formatTime(timing.elapsedTime)}
            </div>
          </div>

          <div style={{ textAlign: 'center' }}>
            <div style={{
              fontSize: 'calc(max(14px, 14px * var(--font-scale)))',
              color: 'var(--color-text-light)',
              marginBottom: '6px'
            }}>
              作業効率
            </div>
            <div style={{
              fontSize: 'calc(max(20px, 20px * var(--font-scale)))',
              fontWeight: '700',
              color: WorkStepManager.getEfficiencyColor(efficiency.currentEfficiency)
            }}>
              {efficiency.currentEfficiency.toFixed(1)}%
            </div>
          </div>
        </div>

        {/* Additional Statistics */}
        <div style={{
          display: 'grid',
          gridTemplateColumns: '1fr 1fr',
          gap: '16px',
          fontSize: 'calc(max(14px, 14px * var(--font-scale)))',
          color: 'var(--color-text-light)'
        }}>
          <div style={{ textAlign: 'center' }}>
            <div>実行回数</div>
            <div style={{ 
              fontWeight: '600', 
              color: 'var(--color-text)',
              fontSize: 'calc(max(16px, 16px * var(--font-scale)))'
            }}>
              {timing.stepCount}回
            </div>
          </div>
          
          <div style={{ textAlign: 'center' }}>
            <div>平均時間</div>
            <div style={{ 
              fontWeight: '600', 
              color: 'var(--color-text)',
              fontSize: 'calc(max(16px, 16px * var(--font-scale)))'
            }}>
              {efficiency.averageStepTime > 0 
                ? WorkStepManager.formatTime(efficiency.averageStepTime)
                : '--:--'
              }
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default WorkStepDisplay;