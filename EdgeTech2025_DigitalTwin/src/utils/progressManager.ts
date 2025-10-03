import { WorkData, ThresholdSettings, WorkerStatus } from '../types';

export interface ProgressData {
  screwProgress: {
    current: number;
    target: number;
    percentage: number;
    remaining: number;
    isThresholdReached: boolean;
    isOverTarget: boolean;
  };
  boltProgress: {
    current: number;
    target: number;
    percentage: number;
    remaining: number;
    isThresholdReached: boolean;
    isOverTarget: boolean;
  };
}

export interface CountingState {
  previousScrewCount: number;
  previousBoltCount: number;
  screwIncrements: number;
  boltIncrements: number;
  lastWorkerStatus: WorkerStatus | null;
}

/**
 * 進捗データを計算する
 */
export function calculateProgress(
  workData: WorkData | null,
  thresholdSettings: ThresholdSettings
): ProgressData {
  const screwCount = workData?.screwCount || 0;
  const boltCount = workData?.boltCount || 0;
  const { screwThreshold, boltThreshold } = thresholdSettings;

  // 安全な除算を行い、0で割ることを防ぐ
  const screwPercentage = screwThreshold > 0 ? Math.min((screwCount / screwThreshold) * 100, 100) : 0;
  const boltPercentage = boltThreshold > 0 ? Math.min((boltCount / boltThreshold) * 100, 100) : 0;

  return {
    screwProgress: {
      current: screwCount,
      target: screwThreshold,
      percentage: screwPercentage,
      remaining: Math.max(screwThreshold - screwCount, 0),
      isThresholdReached: screwCount >= screwThreshold && screwThreshold > 0,
      isOverTarget: screwCount > screwThreshold && screwThreshold > 0
    },
    boltProgress: {
      current: boltCount,
      target: boltThreshold,
      percentage: boltPercentage,
      remaining: Math.max(boltThreshold - boltCount, 0),
      isThresholdReached: boltCount >= boltThreshold && boltThreshold > 0,
      isOverTarget: boltCount > boltThreshold && boltThreshold > 0
    }
  };
}

/**
 * カウント状態を初期化する
 */
export function initializeCountingState(): CountingState {
  return {
    previousScrewCount: 0,
    previousBoltCount: 0,
    screwIncrements: 0,
    boltIncrements: 0,
    lastWorkerStatus: null
  };
}

/**
 * カウント状態を更新する
 */
export function updateCountingState(
  currentState: CountingState,
  workData: WorkData | null
): CountingState {
  if (!workData) {
    return currentState;
  }

  const newState = { ...currentState };
  
  // ネジ締めカウントの増加を検出
  if (workData.screwCount > currentState.previousScrewCount) {
    newState.screwIncrements += (workData.screwCount - currentState.previousScrewCount);
    newState.previousScrewCount = workData.screwCount;
  }
  
  // ボルト締めカウントの増加を検出
  if (workData.boltCount > currentState.previousBoltCount) {
    newState.boltIncrements += (workData.boltCount - currentState.previousBoltCount);
    newState.previousBoltCount = workData.boltCount;
  }
  
  // 作業者状態の変更を記録
  newState.lastWorkerStatus = workData.workerStatus;
  
  return newState;
}

/**
 * 進捗の変化を検出する
 */
export function detectProgressChange(
  previousProgress: ProgressData | null,
  currentProgress: ProgressData
): {
  screwChanged: boolean;
  boltChanged: boolean;
  screwThresholdReached: boolean;
  boltThresholdReached: boolean;
  screwIncrement: number;
  boltIncrement: number;
} {
  if (!previousProgress) {
    return {
      screwChanged: true,
      boltChanged: true,
      screwThresholdReached: currentProgress.screwProgress.isThresholdReached,
      boltThresholdReached: currentProgress.boltProgress.isThresholdReached,
      screwIncrement: currentProgress.screwProgress.current,
      boltIncrement: currentProgress.boltProgress.current
    };
  }

  const screwChanged = previousProgress.screwProgress.current !== currentProgress.screwProgress.current;
  const boltChanged = previousProgress.boltProgress.current !== currentProgress.boltProgress.current;
  
  const screwThresholdReached = !previousProgress.screwProgress.isThresholdReached && 
                                currentProgress.screwProgress.isThresholdReached;
  const boltThresholdReached = !previousProgress.boltProgress.isThresholdReached && 
                               currentProgress.boltProgress.isThresholdReached;

  const screwIncrement = currentProgress.screwProgress.current - previousProgress.screwProgress.current;
  const boltIncrement = currentProgress.boltProgress.current - previousProgress.boltProgress.current;

  return {
    screwChanged,
    boltChanged,
    screwThresholdReached,
    boltThresholdReached,
    screwIncrement,
    boltIncrement
  };
}

/**
 * 進捗に基づいてメッセージを生成する
 */
export function generateProgressMessage(progress: ProgressData): {
  screwMessage: string;
  boltMessage: string;
} {
  const { screwProgress, boltProgress } = progress;

  let screwMessage: string;
  if (screwProgress.target === 0) {
    screwMessage = '目標未設定';
  } else if (screwProgress.isOverTarget) {
    screwMessage = `🎉 目標超過！ (+${screwProgress.current - screwProgress.target})`;
  } else if (screwProgress.isThresholdReached) {
    screwMessage = '🎉 目標達成！';
  } else {
    screwMessage = `🎯 目標まで${screwProgress.remaining}回`;
  }

  let boltMessage: string;
  if (boltProgress.target === 0) {
    boltMessage = '目標未設定';
  } else if (boltProgress.isOverTarget) {
    boltMessage = `🎉 目標超過！ (+${boltProgress.current - boltProgress.target})`;
  } else if (boltProgress.isThresholdReached) {
    boltMessage = '🎉 目標達成！';
  } else {
    boltMessage = `🎯 目標まで${boltProgress.remaining}回`;
  }

  return { screwMessage, boltMessage };
}

/**
 * 進捗の色を取得する
 */
export function getProgressColor(progress: { current: number; target: number; isThresholdReached: boolean; isOverTarget?: boolean }): string {
  if (progress.isOverTarget) {
    return '#9C27B0'; // Purple for over target
  }
  
  if (progress.isThresholdReached) {
    return '#4CAF50'; // Success green
  }
  
  if (progress.target === 0) {
    return '#9E9E9E'; // Gray for no target
  }
  
  const percentage = progress.target > 0 ? (progress.current / progress.target) * 100 : 0;
  
  if (percentage >= 80) {
    return '#2196F3'; // Primary blue
  } else if (percentage >= 50) {
    return '#FF9800'; // Warning orange
  } else {
    return '#F44336'; // Error red
  }
}

/**
 * カウント増加の妥当性をチェックする
 */
export function validateCountIncrement(
  previousCount: number,
  currentCount: number,
  workerStatus: WorkerStatus,
  expectedStatus: WorkerStatus
): {
  isValid: boolean;
  increment: number;
  message?: string;
} {
  const increment = currentCount - previousCount;
  
  if (increment < 0) {
    return {
      isValid: false,
      increment: 0,
      message: 'カウントが減少しました。データエラーの可能性があります。'
    };
  }
  
  if (increment > 10) {
    return {
      isValid: false,
      increment,
      message: '異常に大きなカウント増加が検出されました。'
    };
  }
  
  if (increment > 0 && workerStatus !== expectedStatus) {
    return {
      isValid: false,
      increment,
      message: `作業状態が${expectedStatus}でないのにカウントが増加しました。`
    };
  }
  
  return {
    isValid: true,
    increment
  };
}

/**
 * 閾値達成時の推奨アクションを取得する
 */
export function getRecommendedAction(
  progressType: 'screw' | 'bolt',
  isThresholdReached: boolean,
  isOverTarget: boolean
): string | null {
  if (!isThresholdReached) {
    return null;
  }
  
  if (progressType === 'screw') {
    return isOverTarget 
      ? 'ネジ締め作業が目標を超過しました。次の工程に進んでください。'
      : 'ネジ締め作業が完了しました。ロボットに工具を渡してください。';
  } else {
    return isOverTarget
      ? 'ボルト締め作業が目標を超過しました。作業を完了してください。'
      : 'ボルト締め作業が完了しました。次のタスクに進んでください。';
  }
}

/**
 * 進捗のアニメーション設定を取得する
 */
export function getProgressAnimationConfig(isThresholdReached: boolean) {
  return {
    duration: isThresholdReached ? 1500 : 1000,
    easing: isThresholdReached ? 'easeOutBounce' : 'easeOutQuart',
    delay: isThresholdReached ? 200 : 0
  };
}