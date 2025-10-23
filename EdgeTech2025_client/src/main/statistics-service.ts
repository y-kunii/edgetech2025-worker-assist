import { DatabaseService } from './database-service';
import { StatusHistoryRecord, WorkStatisticsRecord, WorkStatistics } from '../shared/types';
import { DEMO_TASKS } from '../shared/constants';

export class StatisticsService {
  private dbService: DatabaseService;

  constructor(dbService: DatabaseService) {
    this.dbService = dbService;
  }

  /**
   * 日次作業統計の計算と更新
   */
  public calculateDailyStatistics(date: string): WorkStatisticsRecord | null {
    try {
      // 指定日の開始・終了タイムスタンプを生成（YYYYMMDDhhmmss形式）
      const startTimestamp = `${date}000000`;
      const endTimestamp = `${date}235959`;

      // 指定日の状態履歴を取得
      const dayHistory = this.dbService.getStatusHistory(startTimestamp, endTimestamp);

      if (dayHistory.length === 0) {
        return null;
      }

      // 作業タスク別の実行回数を計算
      const taskCounts = this.calculateTaskCounts(dayHistory);
      
      // 作業時間の計算
      const workTimes = this.calculateWorkTimes(dayHistory);

      // 統計レコードを作成
      const statistics: Omit<WorkStatisticsRecord, 'id' | 'created_at'> = {
        date,
        screw_count: taskCounts.screwCount,
        blocks_count: taskCounts.blocksCount,
        survey_count: taskCounts.surveyCount,
        total_work_time: workTimes.totalWorkTime,
        average_work_time: workTimes.averageWorkTime
      };

      // データベースに保存または更新
      return this.saveOrUpdateDailyStatistics(statistics);
    } catch (error) {
      console.error('Error calculating daily statistics:', error);
      return null;
    }
  }

  /**
   * 作業タスク別の実行回数を計算
   */
  private calculateTaskCounts(history: StatusHistoryRecord[]): {
    screwCount: number;
    blocksCount: number;
    surveyCount: number;
  } {
    let screwCount = 0;
    let blocksCount = 0;
    let surveyCount = 0;

    // 作業状態の変化を追跡して完了回数をカウント
    let previousSpaceStatus = '';
    let previousWorkerStatus = '';

    for (const record of history.reverse()) { // 時系列順にソート
      // 作業完了の判定：作業中 → 作業完了への変化
      if (previousWorkerStatus === 'Working' && record.worker_status === 'Work Completed') {
        switch (previousSpaceStatus) {
          case DEMO_TASKS.SCREW_TIGHTENING:
            screwCount++;
            break;
          case DEMO_TASKS.BUILDING_BLOCKS:
            blocksCount++;
            break;
          case DEMO_TASKS.SURVEY_RESPONSES:
            surveyCount++;
            break;
        }
      }

      previousSpaceStatus = record.space_status;
      previousWorkerStatus = record.worker_status;
    }

    return { screwCount, blocksCount, surveyCount };
  }

  /**
   * 作業時間の計算
   */
  private calculateWorkTimes(history: StatusHistoryRecord[]): {
    totalWorkTime: number;
    averageWorkTime: number;
  } {
    let totalWorkTime = 0;
    let workSessionCount = 0;
    let workStartTime: Date | null = null;

    // 時系列順にソート
    const sortedHistory = history.sort((a, b) => 
      new Date(a.created_at).getTime() - new Date(b.created_at).getTime()
    );

    for (const record of sortedHistory) {
      const recordTime = new Date(record.created_at);

      // 作業開始の検出
      if (record.worker_status === 'Working' && workStartTime === null) {
        workStartTime = recordTime;
      }
      
      // 作業終了の検出
      if (record.worker_status === 'Work Completed' && workStartTime !== null) {
        const workDuration = recordTime.getTime() - workStartTime.getTime();
        totalWorkTime += workDuration;
        workSessionCount++;
        workStartTime = null;
      }
      
      // 作業中断の検出（待機状態に戻った場合）
      if ((record.worker_status === 'Waiting' || record.worker_status === 'Absent') && workStartTime !== null) {
        workStartTime = null;
      }
    }

    const averageWorkTime = workSessionCount > 0 ? totalWorkTime / workSessionCount : 0;

    return {
      totalWorkTime: Math.round(totalWorkTime / 1000), // 秒単位に変換
      averageWorkTime: Math.round(averageWorkTime / 1000) // 秒単位に変換
    };
  }

  /**
   * 日次統計の保存または更新
   */
  private saveOrUpdateDailyStatistics(statistics: Omit<WorkStatisticsRecord, 'id' | 'created_at'>): WorkStatisticsRecord | null {
    try {
      // 既存レコードの確認
      const existingRecord = this.dbService.db.prepare(`
        SELECT * FROM work_statistics WHERE date = ?
      `).get(statistics.date) as WorkStatisticsRecord | undefined;

      if (existingRecord) {
        // 更新
        const updateStmt = this.dbService.db.prepare(`
          UPDATE work_statistics 
          SET screw_count = ?, blocks_count = ?, survey_count = ?, 
              total_work_time = ?, average_work_time = ?, updated_at = CURRENT_TIMESTAMP
          WHERE date = ?
        `);

        updateStmt.run(
          statistics.screw_count,
          statistics.blocks_count,
          statistics.survey_count,
          statistics.total_work_time,
          statistics.average_work_time,
          statistics.date
        );

        return { ...existingRecord, ...statistics };
      } else {
        // 新規作成
        const insertStmt = this.dbService.db.prepare(`
          INSERT INTO work_statistics (date, screw_count, blocks_count, survey_count, total_work_time, average_work_time)
          VALUES (?, ?, ?, ?, ?, ?)
        `);

        const result = insertStmt.run(
          statistics.date,
          statistics.screw_count,
          statistics.blocks_count,
          statistics.survey_count,
          statistics.total_work_time,
          statistics.average_work_time
        );

        return {
          id: result.lastInsertRowid as number,
          ...statistics,
          created_at: new Date().toISOString()
        };
      }
    } catch (error) {
      console.error('Error saving daily statistics:', error);
      return null;
    }
  }

  /**
   * 現在の作業統計を取得
   */
  public getCurrentStatistics(): WorkStatistics {
    try {
      const today = new Date().toISOString().slice(0, 10).replace(/-/g, ''); // YYYYMMDD形式

      // 今日の統計を計算
      const todayStats = this.calculateDailyStatistics(today);

      // 直近の作業評価を計算
      const recentPerformance = this.calculateRecentPerformance();

      return {
        screwCount: todayStats?.screw_count || 0,
        blocksCount: todayStats?.blocks_count || 0,
        surveyCount: todayStats?.survey_count || 0,
        totalWorkTime: todayStats?.total_work_time || 0,
        averageWorkTime: todayStats?.average_work_time || 0,
        recentPerformance
      };
    } catch (error) {
      console.error('Error getting current statistics:', error);
      return {
        screwCount: 0,
        blocksCount: 0,
        surveyCount: 0,
        totalWorkTime: 0,
        averageWorkTime: 0,
        recentPerformance: null
      };
    }
  }

  /**
   * 直近の作業評価を計算（標準時間との比較）
   */
  private calculateRecentPerformance(): 'fast' | 'normal' | 'slow' | null {
    try {
      // 直近10件の作業時間を取得
      const recentHistory = this.dbService.getStatusHistory(undefined, undefined, 100);
      
      if (recentHistory.length < 2) {
        return null;
      }

      const recentWorkTimes = this.calculateWorkTimes(recentHistory);
      
      if (recentWorkTimes.averageWorkTime === 0) {
        return null;
      }

      // 標準時間の定義（秒）
      const STANDARD_WORK_TIMES = {
        [DEMO_TASKS.SCREW_TIGHTENING]: 120, // 2分
        [DEMO_TASKS.BUILDING_BLOCKS]: 180,  // 3分
        [DEMO_TASKS.SURVEY_RESPONSES]: 60   // 1分
      };

      // 全作業の平均標準時間
      const averageStandardTime = Object.values(STANDARD_WORK_TIMES).reduce((a, b) => a + b, 0) / 3;

      // パフォーマンス評価
      const performanceRatio = recentWorkTimes.averageWorkTime / averageStandardTime;

      if (performanceRatio < 0.8) {
        return 'fast';
      } else if (performanceRatio > 1.2) {
        return 'slow';
      } else {
        return 'normal';
      }
    } catch (error) {
      console.error('Error calculating recent performance:', error);
      return null;
    }
  }

  /**
   * 期間別統計の取得
   */
  public getPeriodStatistics(startDate: string, endDate: string): WorkStatisticsRecord[] {
    try {
      const stmt = this.dbService.db.prepare(`
        SELECT * FROM work_statistics 
        WHERE date >= ? AND date <= ?
        ORDER BY date ASC
      `);

      return stmt.all(startDate, endDate) as WorkStatisticsRecord[];
    } catch (error) {
      console.error('Error getting period statistics:', error);
      return [];
    }
  }

  /**
   * 統計データの自動更新（定期実行用）
   */
  public updateAllStatistics(): void {
    try {
      // 過去7日間の統計を更新
      const today = new Date();
      
      for (let i = 0; i < 7; i++) {
        const targetDate = new Date(today);
        targetDate.setDate(today.getDate() - i);
        const dateString = targetDate.toISOString().slice(0, 10).replace(/-/g, '');
        
        this.calculateDailyStatistics(dateString);
      }

      console.log('Statistics updated successfully');
    } catch (error) {
      console.error('Error updating statistics:', error);
    }
  }

  /**
   * 作業効率レポートの生成
   */
  public generateEfficiencyReport(days: number = 7): {
    totalTasks: number;
    averageTasksPerDay: number;
    mostProductiveDay: string | null;
    improvementTrend: 'improving' | 'declining' | 'stable';
  } {
    try {
      const endDate = new Date().toISOString().slice(0, 10).replace(/-/g, '');
      const startDate = new Date();
      startDate.setDate(startDate.getDate() - days);
      const startDateString = startDate.toISOString().slice(0, 10).replace(/-/g, '');

      const periodStats = this.getPeriodStatistics(startDateString, endDate);

      if (periodStats.length === 0) {
        return {
          totalTasks: 0,
          averageTasksPerDay: 0,
          mostProductiveDay: null,
          improvementTrend: 'stable'
        };
      }

      // 総タスク数の計算
      const totalTasks = periodStats.reduce((sum, stat) => 
        sum + stat.screw_count + stat.blocks_count + stat.survey_count, 0
      );

      // 1日あたりの平均タスク数
      const averageTasksPerDay = totalTasks / periodStats.length;

      // 最も生産性の高い日を特定
      let mostProductiveDay: string | null = null;
      let maxTasks = 0;

      periodStats.forEach(stat => {
        const dayTasks = stat.screw_count + stat.blocks_count + stat.survey_count;
        if (dayTasks > maxTasks) {
          maxTasks = dayTasks;
          mostProductiveDay = stat.date;
        }
      });

      // 改善傾向の分析
      const improvementTrend = this.analyzeTrend(periodStats);

      return {
        totalTasks,
        averageTasksPerDay: Math.round(averageTasksPerDay * 100) / 100,
        mostProductiveDay,
        improvementTrend
      };
    } catch (error) {
      console.error('Error generating efficiency report:', error);
      return {
        totalTasks: 0,
        averageTasksPerDay: 0,
        mostProductiveDay: null,
        improvementTrend: 'stable'
      };
    }
  }

  /**
   * 統計データの傾向分析
   */
  private analyzeTrend(stats: WorkStatisticsRecord[]): 'improving' | 'declining' | 'stable' {
    if (stats.length < 3) {
      return 'stable';
    }

    // 前半と後半の平均を比較
    const midPoint = Math.floor(stats.length / 2);
    const firstHalf = stats.slice(0, midPoint);
    const secondHalf = stats.slice(midPoint);

    const firstHalfAvg = firstHalf.reduce((sum, stat) => 
      sum + stat.screw_count + stat.blocks_count + stat.survey_count, 0
    ) / firstHalf.length;

    const secondHalfAvg = secondHalf.reduce((sum, stat) => 
      sum + stat.screw_count + stat.blocks_count + stat.survey_count, 0
    ) / secondHalf.length;

    const improvementRatio = secondHalfAvg / firstHalfAvg;

    if (improvementRatio > 1.1) {
      return 'improving';
    } else if (improvementRatio < 0.9) {
      return 'declining';
    } else {
      return 'stable';
    }
  }
}

// シングルトンインスタンス
let statisticsService: StatisticsService | null = null;

export function getStatisticsService(dbService: DatabaseService): StatisticsService {
  if (!statisticsService) {
    statisticsService = new StatisticsService(dbService);
  }
  return statisticsService;
}