import { WorkerStatus } from '../types';

export interface WorkStepTiming {
  currentStep: WorkerStatus;
  stepStartTime: Date;
  elapsedTime: number; // in seconds
  totalStepTime: number; // total time spent in current step type today
  stepCount: number; // number of times this step has been executed
}

export interface WorkStepEfficiency {
  currentEfficiency: number; // percentage
  averageStepTime: number; // average time for current step type
  expectedStepTime: number; // expected time for current step type
  trend: 'improving' | 'declining' | 'stable';
}

// Expected times for each work step (in seconds)
const EXPECTED_STEP_TIMES: Record<WorkerStatus, number> = {
  'waiting': 30, // 30 seconds expected waiting time
  'screw_tightening': 45, // 45 seconds expected for screw tightening
  'bolt_tightening': 60, // 60 seconds expected for bolt tightening
  'tool_handover': 15, // 15 seconds expected for tool handover
  'absent': 0 // No expected time for absent
};

export class WorkStepManager {
  private currentTiming: WorkStepTiming;
  private stepHistory: Array<{
    step: WorkerStatus;
    startTime: Date;
    endTime: Date;
    duration: number;
  }> = [];
  private efficiencyHistory: number[] = [];
  private readonly maxHistorySize = 100;

  constructor(initialStep: WorkerStatus = 'waiting') {
    this.currentTiming = {
      currentStep: initialStep,
      stepStartTime: new Date(),
      elapsedTime: 0,
      totalStepTime: 0,
      stepCount: 0
    };
  }

  /**
   * Update the current work step
   */
  public updateWorkStep(newStep: WorkerStatus): void {
    const now = new Date();
    const previousStep = this.currentTiming.currentStep;
    
    // If step changed, record the previous step in history
    if (previousStep !== newStep) {
      const duration = now.getTime() - this.currentTiming.stepStartTime.getTime();
      
      this.stepHistory.push({
        step: previousStep,
        startTime: this.currentTiming.stepStartTime,
        endTime: now,
        duration: duration / 1000 // convert to seconds
      });
      
      // Limit history size
      if (this.stepHistory.length > this.maxHistorySize) {
        this.stepHistory = this.stepHistory.slice(-this.maxHistorySize);
      }
      
      // Reset timing for new step
      this.currentTiming = {
        currentStep: newStep,
        stepStartTime: now,
        elapsedTime: 0,
        totalStepTime: this.calculateTotalStepTime(newStep),
        stepCount: this.calculateStepCount(newStep)
      };
    }
  }

  /**
   * Update elapsed time for current step
   */
  public updateElapsedTime(): void {
    const now = new Date();
    this.currentTiming.elapsedTime = Math.floor(
      (now.getTime() - this.currentTiming.stepStartTime.getTime()) / 1000
    );
  }

  /**
   * Get current timing information
   */
  public getCurrentTiming(): WorkStepTiming {
    this.updateElapsedTime();
    return { ...this.currentTiming };
  }

  /**
   * Calculate current work efficiency
   */
  public calculateCurrentEfficiency(): WorkStepEfficiency {
    const currentStep = this.currentTiming.currentStep;
    const elapsedTime = this.currentTiming.elapsedTime;
    const expectedTime = EXPECTED_STEP_TIMES[currentStep];
    
    // Calculate efficiency based on expected vs actual time
    let currentEfficiency = 100;
    if (expectedTime > 0 && elapsedTime > 0) {
      // Efficiency decreases as time exceeds expected time
      currentEfficiency = Math.max(0, Math.min(100, 
        (expectedTime / elapsedTime) * 100
      ));
    }
    
    // Calculate average step time for this step type
    const stepHistoryForType = this.stepHistory.filter(h => h.step === currentStep);
    const averageStepTime = stepHistoryForType.length > 0 
      ? stepHistoryForType.reduce((sum, h) => sum + h.duration, 0) / stepHistoryForType.length
      : expectedTime;
    
    // Determine trend
    const recentEfficiencies = this.efficiencyHistory.slice(-5); // Last 5 measurements
    let trend: 'improving' | 'declining' | 'stable' = 'stable';
    
    if (recentEfficiencies.length >= 3) {
      const firstHalf = recentEfficiencies.slice(0, Math.floor(recentEfficiencies.length / 2));
      const secondHalf = recentEfficiencies.slice(Math.floor(recentEfficiencies.length / 2));
      
      const firstAvg = firstHalf.reduce((sum, eff) => sum + eff, 0) / firstHalf.length;
      const secondAvg = secondHalf.reduce((sum, eff) => sum + eff, 0) / secondHalf.length;
      
      const threshold = 5; // 5% threshold for trend detection
      if (secondAvg > firstAvg + threshold) {
        trend = 'improving';
      } else if (secondAvg < firstAvg - threshold) {
        trend = 'declining';
      }
    }
    
    // Store efficiency for trend analysis
    this.efficiencyHistory.push(currentEfficiency);
    if (this.efficiencyHistory.length > 20) {
      this.efficiencyHistory = this.efficiencyHistory.slice(-20);
    }
    
    return {
      currentEfficiency: Math.round(currentEfficiency),
      averageStepTime: Math.round(averageStepTime),
      expectedStepTime: expectedTime,
      trend
    };
  }

  /**
   * Get step statistics
   */
  public getStepStatistics() {
    const stepCounts: Record<WorkerStatus, number> = {
      'waiting': 0,
      'screw_tightening': 0,
      'bolt_tightening': 0,
      'tool_handover': 0,
      'absent': 0
    };
    
    const stepTotalTimes: Record<WorkerStatus, number> = {
      'waiting': 0,
      'screw_tightening': 0,
      'bolt_tightening': 0,
      'tool_handover': 0,
      'absent': 0
    };
    
    this.stepHistory.forEach(entry => {
      stepCounts[entry.step]++;
      stepTotalTimes[entry.step] += entry.duration;
    });
    
    return {
      stepCounts,
      stepTotalTimes,
      totalSteps: this.stepHistory.length,
      totalTime: this.stepHistory.reduce((sum, entry) => sum + entry.duration, 0)
    };
  }

  /**
   * Calculate total time spent in a specific step type
   */
  private calculateTotalStepTime(stepType: WorkerStatus): number {
    return this.stepHistory
      .filter(entry => entry.step === stepType)
      .reduce((total, entry) => total + entry.duration, 0);
  }

  /**
   * Calculate how many times a step has been executed
   */
  private calculateStepCount(stepType: WorkerStatus): number {
    return this.stepHistory.filter(entry => entry.step === stepType).length;
  }

  /**
   * Reset all timing data
   */
  public reset(): void {
    this.stepHistory = [];
    this.efficiencyHistory = [];
    this.currentTiming = {
      currentStep: 'waiting',
      stepStartTime: new Date(),
      elapsedTime: 0,
      totalStepTime: 0,
      stepCount: 0
    };
  }

  /**
   * Get formatted time string
   */
  public static formatTime(seconds: number): string {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins}:${secs.toString().padStart(2, '0')}`;
  }

  /**
   * Get step display name in Japanese
   */
  public static getStepDisplayName(step: WorkerStatus): string {
    switch (step) {
      case 'waiting': return '待機中';
      case 'screw_tightening': return 'ネジ締め作業';
      case 'bolt_tightening': return 'ボルト締め作業';
      case 'tool_handover': return '工具受け渡し';
      case 'absent': return '不在';
      default: return '不明な状態';
    }
  }

  /**
   * Get step icon
   */
  public static getStepIcon(step: WorkerStatus): string {
    switch (step) {
      case 'waiting': return '⏳';
      case 'screw_tightening': return '🔧';
      case 'bolt_tightening': return '🔩';
      case 'tool_handover': return '🤝';
      case 'absent': return '👤';
      default: return '📋';
    }
  }

  /**
   * Get efficiency color based on percentage
   */
  public static getEfficiencyColor(efficiency: number): string {
    if (efficiency >= 90) return 'var(--color-success)';
    if (efficiency >= 75) return 'var(--color-primary-blue)';
    if (efficiency >= 50) return 'var(--color-warning)';
    return 'var(--color-error)';
  }

  /**
   * Get trend icon
   */
  public static getTrendIcon(trend: 'improving' | 'declining' | 'stable'): string {
    switch (trend) {
      case 'improving': return '📈';
      case 'declining': return '📉';
      case 'stable': return '📊';
    }
  }
}