import React, { useState, useEffect, useCallback } from 'react';
import ProgressChart from './ProgressChart';
import { WorkDataStore } from '../../stores/WorkDataStore';
import { 
  calculateProgress, 
  detectProgressChange, 
  ProgressData, 
  CountingState,
  initializeCountingState,
  updateCountingState,
  validateCountIncrement,
  getRecommendedAction
} from '../../utils/progressManager';
import { WorkData, ThresholdSettings } from '../../types';

interface ProgressManagerProps {
  workDataStore: WorkDataStore;
}

const ProgressManager: React.FC<ProgressManagerProps> = ({ workDataStore }) => {
  const [currentProgress, setCurrentProgress] = useState<ProgressData | null>(null);
  const [thresholdSettings, setThresholdSettings] = useState<ThresholdSettings>({ screwThreshold: 5, boltThreshold: 3 });
  const [workData, setWorkData] = useState<WorkData | null>(null);
  const [countingState, setCountingState] = useState<CountingState>(initializeCountingState());
  const [validationErrors, setValidationErrors] = useState<string[]>([]);

  // 進捗データを更新する
  const updateProgress = useCallback((newWorkData: WorkData | null, settings: ThresholdSettings) => {
    const newProgress = calculateProgress(newWorkData, settings);
    
    // カウント状態を更新
    const newCountingState = updateCountingState(countingState, newWorkData);
    setCountingState(newCountingState);
    
    // 進捗の変化を検出
    const changes = detectProgressChange(currentProgress, newProgress);
    
    // カウント増加の妥当性をチェック
    const errors: string[] = [];
    if (newWorkData && changes.screwIncrement > 0) {
      const screwValidation = validateCountIncrement(
        currentProgress?.screwProgress.current || 0,
        newProgress.screwProgress.current,
        newWorkData.workerStatus,
        'screw_tightening'
      );
      if (!screwValidation.isValid && screwValidation.message) {
        errors.push(`ネジ締め: ${screwValidation.message}`);
      }
    }
    
    if (newWorkData && changes.boltIncrement > 0) {
      const boltValidation = validateCountIncrement(
        currentProgress?.boltProgress.current || 0,
        newProgress.boltProgress.current,
        newWorkData.workerStatus,
        'bolt_tightening'
      );
      if (!boltValidation.isValid && boltValidation.message) {
        errors.push(`ボルト締め: ${boltValidation.message}`);
      }
    }
    
    setValidationErrors(errors);
    
    // 閾値達成時の特別処理
    if (changes.screwThresholdReached) {
      console.log('Screw threshold reached!', newProgress.screwProgress);
      const action = getRecommendedAction('screw', true, newProgress.screwProgress.isOverTarget);
      if (action) {
        console.log('Recommended action:', action);
      }
    }
    
    if (changes.boltThresholdReached) {
      console.log('Bolt threshold reached!', newProgress.boltProgress);
      const action = getRecommendedAction('bolt', true, newProgress.boltProgress.isOverTarget);
      if (action) {
        console.log('Recommended action:', action);
      }
    }
    
    setCurrentProgress(newProgress);
  }, [currentProgress, countingState]);

  // WorkDataStoreのイベントリスナーを設定
  useEffect(() => {
    const handleWorkDataUpdate = (data: WorkData) => {
      setWorkData(data);
      updateProgress(data, thresholdSettings);
    };

    const handleThresholdUpdate = (settings: ThresholdSettings) => {
      setThresholdSettings(settings);
      updateProgress(workData, settings);
    };

    const handleScrewThresholdReached = (data: { count: number; threshold: number }) => {
      console.log('Screw threshold reached event:', data);
    };

    const handleBoltThresholdReached = (data: { count: number; threshold: number }) => {
      console.log('Bolt threshold reached event:', data);
    };

    // イベントリスナーを登録
    workDataStore.on('work_data_updated', handleWorkDataUpdate);
    workDataStore.on('threshold_settings_updated', handleThresholdUpdate);
    workDataStore.on('screw_threshold_reached', handleScrewThresholdReached);
    workDataStore.on('bolt_threshold_reached', handleBoltThresholdReached);

    // 初期状態を設定
    const initialState = workDataStore.getState();
    setWorkData(initialState.currentWorkData);
    setThresholdSettings(initialState.thresholdSettings);
    updateProgress(initialState.currentWorkData, initialState.thresholdSettings);

    // クリーンアップ
    return () => {
      workDataStore.off('work_data_updated', handleWorkDataUpdate);
      workDataStore.off('threshold_settings_updated', handleThresholdUpdate);
      workDataStore.off('screw_threshold_reached', handleScrewThresholdReached);
      workDataStore.off('bolt_threshold_reached', handleBoltThresholdReached);
    };
  }, [workDataStore, updateProgress, workData, thresholdSettings]);

  if (!currentProgress) {
    return (
      <div className="progress-manager-loading">
        <div>進捗データを読み込み中...</div>
      </div>
    );
  }

  return (
    <div className="progress-manager">
      {validationErrors.length > 0 && (
        <div className="validation-errors">
          {validationErrors.map((error, index) => (
            <div key={index} className="validation-error">
              ⚠️ {error}
            </div>
          ))}
        </div>
      )}
      
      <ProgressChart
        type="screw"
        current={currentProgress.screwProgress.current}
        target={currentProgress.screwProgress.target}
        isThresholdReached={currentProgress.screwProgress.isThresholdReached}
      />
      <ProgressChart
        type="bolt"
        current={currentProgress.boltProgress.current}
        target={currentProgress.boltProgress.target}
        isThresholdReached={currentProgress.boltProgress.isThresholdReached}
      />
      
      <div className="counting-stats">
        <div className="stat-item">
          <span className="stat-label">ネジ締め増分:</span>
          <span className="stat-value">{countingState.screwIncrements}</span>
        </div>
        <div className="stat-item">
          <span className="stat-label">ボルト締め増分:</span>
          <span className="stat-value">{countingState.boltIncrements}</span>
        </div>
      </div>
    </div>
  );
};

export default ProgressManager;