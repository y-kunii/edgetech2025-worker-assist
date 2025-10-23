import { StatusData } from '../shared/types';
import { DatabaseService, getDatabaseService } from './database-service';
import { StatisticsService, getStatisticsService } from './statistics-service';

/**
 * データ検証エラー
 */
export class DataValidationError extends Error {
  constructor(message: string, public field?: string) {
    super(message);
    this.name = 'DataValidationError';
  }
}

/**
 * StatusDataの検証とパース処理を行うクラス
 */
export class DataProcessor {
  private static dbService: DatabaseService;
  private static statisticsService: StatisticsService;

  /**
   * データベースサービスの初期化
   */
  public static initialize(): void {
    this.dbService = getDatabaseService();
    this.statisticsService = getStatisticsService(this.dbService);
  }
  /**
   * 受信したJSONデータをStatusDataとして検証・パース・保存
   */
  public static validateAndParseStatusData(rawData: string): StatusData {
    try {
      const data = JSON.parse(rawData);
      const validatedData = this.validateStatusData(data);
      
      // データベースに保存
      if (this.dbService) {
        const saved = this.dbService.saveStatusData(validatedData);
        if (saved) {
          console.log('Status data saved to database');
          
          // 統計の更新（非同期で実行）
          setTimeout(() => {
            if (this.statisticsService) {
              this.statisticsService.updateAllStatistics();
            }
          }, 0);
        } else {
          console.warn('Failed to save status data to database');
        }
      }
      
      return validatedData;
    } catch (error) {
      if (error instanceof SyntaxError) {
        throw new DataValidationError(`Invalid JSON format: ${error.message}`);
      }
      throw error;
    }
  }

  /**
   * StatusDataオブジェクトの検証
   */
  public static validateStatusData(data: any): StatusData {
    if (!data || typeof data !== 'object') {
      throw new DataValidationError('Data must be an object');
    }

    // worker_statusの検証
    if (!this.isValidWorkerStatus(data.worker_status)) {
      throw new DataValidationError(
        `Invalid worker_status: ${data.worker_status}. Must be one of: Absent, Waiting, Working, Work Completed`,
        'worker_status'
      );
    }

    // space_statusの検証
    if (!this.isValidSpaceStatus(data.space_status)) {
      throw new DataValidationError(
        `Invalid space_status: ${data.space_status}. Must be one of: Nothing, Screw_tightening, Building_blocks, Survey_responses`,
        'space_status'
      );
    }

    // robot_statusの検証
    if (!this.isValidRobotStatus(data.robot_status)) {
      throw new DataValidationError(
        'Invalid robot_status. Must be an object with state and grip properties',
        'robot_status'
      );
    }

    // timestampの検証
    if (!this.isValidTimestamp(data.timestamp)) {
      throw new DataValidationError(
        `Invalid timestamp: ${data.timestamp}. Must be in YYYYMMDDhhmmss format`,
        'timestamp'
      );
    }

    // tool_deliveryの検証
    if (!this.isValidToolDelivery(data.tool_delivery)) {
      throw new DataValidationError(
        `Invalid tool_delivery: ${data.tool_delivery}. Must be a number`,
        'tool_delivery'
      );
    }

    // statusの検証（worker_statusと同じ値セット）
    if (!this.isValidWorkerStatus(data.status)) {
      throw new DataValidationError(
        `Invalid status: ${data.status}. Must be one of: Absent, Waiting, Working, Work Completed`,
        'status'
      );
    }

    return {
      worker_status: data.worker_status,
      space_status: data.space_status,
      robot_status: {
        state: data.robot_status.state,
        grip: data.robot_status.grip
      },
      timestamp: data.timestamp,
      tool_delivery: data.tool_delivery,
      status: data.status
    };
  }

  /**
   * worker_statusの有効性をチェック
   */
  private static isValidWorkerStatus(status: any): status is StatusData['worker_status'] {
    const validStatuses = ['Absent', 'Waiting', 'Working', 'Work Completed'];
    return typeof status === 'string' && validStatuses.includes(status);
  }

  /**
   * space_statusの有効性をチェック
   */
  private static isValidSpaceStatus(status: any): status is StatusData['space_status'] {
    const validStatuses = ['Nothing', 'Screw_tightening', 'Building_blocks', 'Survey_responses'];
    return typeof status === 'string' && validStatuses.includes(status);
  }

  /**
   * robot_statusの有効性をチェック
   */
  private static isValidRobotStatus(robotStatus: any): robotStatus is StatusData['robot_status'] {
    if (!robotStatus || typeof robotStatus !== 'object') {
      return false;
    }

    // stateは文字列であること
    if (typeof robotStatus.state !== 'string') {
      return false;
    }

    // gripは'open'または'closed'であること
    const validGrips = ['open', 'closed'];
    return typeof robotStatus.grip === 'string' && validGrips.includes(robotStatus.grip);
  }

  /**
   * timestampの有効性をチェック（YYYYMMDDhhmmss形式）
   */
  private static isValidTimestamp(timestamp: any): timestamp is string {
    if (typeof timestamp !== 'string') {
      return false;
    }

    // 14桁の数字であることをチェック
    const timestampRegex = /^\d{14}$/;
    if (!timestampRegex.test(timestamp)) {
      return false;
    }

    // 日付として有効かチェック
    const year = parseInt(timestamp.substring(0, 4));
    const month = parseInt(timestamp.substring(4, 6));
    const day = parseInt(timestamp.substring(6, 8));
    const hour = parseInt(timestamp.substring(8, 10));
    const minute = parseInt(timestamp.substring(10, 12));
    const second = parseInt(timestamp.substring(12, 14));

    // 基本的な範囲チェック
    if (year < 2000 || year > 2100) return false;
    if (month < 1 || month > 12) return false;
    if (day < 1 || day > 31) return false;
    if (hour < 0 || hour > 23) return false;
    if (minute < 0 || minute > 59) return false;
    if (second < 0 || second > 59) return false;

    // Dateオブジェクトで実際の日付として有効かチェック
    const date = new Date(year, month - 1, day, hour, minute, second);
    return date.getFullYear() === year &&
           date.getMonth() === month - 1 &&
           date.getDate() === day &&
           date.getHours() === hour &&
           date.getMinutes() === minute &&
           date.getSeconds() === second;
  }

  /**
   * tool_deliveryの有効性をチェック
   */
  private static isValidToolDelivery(toolDelivery: any): toolDelivery is number {
    return typeof toolDelivery === 'number' && !isNaN(toolDelivery);
  }

  /**
   * timestampを日本時間のDateオブジェクトに変換
   */
  public static parseTimestamp(timestamp: string): Date {
    const year = parseInt(timestamp.substring(0, 4));
    const month = parseInt(timestamp.substring(4, 6)) - 1; // Dateオブジェクトは0ベース
    const day = parseInt(timestamp.substring(6, 8));
    const hour = parseInt(timestamp.substring(8, 10));
    const minute = parseInt(timestamp.substring(10, 12));
    const second = parseInt(timestamp.substring(12, 14));

    return new Date(year, month, day, hour, minute, second);
  }

  /**
   * DateオブジェクトをYYYYMMDDhhmmss形式の文字列に変換
   */
  public static formatTimestamp(date: Date): string {
    const year = date.getFullYear().toString();
    const month = (date.getMonth() + 1).toString().padStart(2, '0');
    const day = date.getDate().toString().padStart(2, '0');
    const hour = date.getHours().toString().padStart(2, '0');
    const minute = date.getMinutes().toString().padStart(2, '0');
    const second = date.getSeconds().toString().padStart(2, '0');

    return `${year}${month}${day}${hour}${minute}${second}`;
  }

  /**
   * StatusDataから作業タスクを判定
   */
  public static determineWorkTask(statusData: StatusData): 'Screw_tightening' | 'Building_blocks' | 'Survey_responses' | 'Waiting' | 'Nothing' {
    // space_statusから作業タスクを判定
    switch (statusData.space_status) {
      case 'Screw_tightening':
        return 'Screw_tightening';
      case 'Building_blocks':
        return 'Building_blocks';
      case 'Survey_responses':
        return 'Survey_responses';
      case 'Nothing':
        // worker_statusがWaitingの場合はWaiting、それ以外はNothing
        return statusData.worker_status === 'Waiting' ? 'Waiting' : 'Nothing';
      default:
        return 'Nothing';
    }
  }

  /**
   * 作業状態の変化を検出
   */
  public static detectStatusChange(
    previousStatus: StatusData | null,
    currentStatus: StatusData
  ): {
    hasChanged: boolean;
    changedFields: string[];
    workTaskChanged: boolean;
    previousTask?: string;
    currentTask: string;
  } {
    if (!previousStatus) {
      return {
        hasChanged: true,
        changedFields: ['initial'],
        workTaskChanged: true,
        currentTask: this.determineWorkTask(currentStatus)
      };
    }

    const changedFields: string[] = [];

    // 各フィールドの変化をチェック
    if (previousStatus.worker_status !== currentStatus.worker_status) {
      changedFields.push('worker_status');
    }
    if (previousStatus.space_status !== currentStatus.space_status) {
      changedFields.push('space_status');
    }
    if (previousStatus.robot_status.state !== currentStatus.robot_status.state) {
      changedFields.push('robot_status.state');
    }
    if (previousStatus.robot_status.grip !== currentStatus.robot_status.grip) {
      changedFields.push('robot_status.grip');
    }
    if (previousStatus.tool_delivery !== currentStatus.tool_delivery) {
      changedFields.push('tool_delivery');
    }
    if (previousStatus.status !== currentStatus.status) {
      changedFields.push('status');
    }

    const previousTask = this.determineWorkTask(previousStatus);
    const currentTask = this.determineWorkTask(currentStatus);
    const workTaskChanged = previousTask !== currentTask;

    return {
      hasChanged: changedFields.length > 0,
      changedFields,
      workTaskChanged,
      previousTask,
      currentTask
    };
  }

  /**
   * コマンドデータの保存
   */
  public static saveCommandData(commandData: any, success: boolean): boolean {
    try {
      if (!this.dbService) {
        console.warn('Database service not initialized');
        return false;
      }

      return this.dbService.saveCommandHistory(commandData, success);
    } catch (error) {
      console.error('Error saving command data:', error);
      return false;
    }
  }

  /**
   * 現在の統計データを取得
   */
  public static getCurrentStatistics() {
    if (!this.statisticsService) {
      console.warn('Statistics service not initialized');
      return null;
    }

    return this.statisticsService.getCurrentStatistics();
  }

  /**
   * データベースの統計情報を取得
   */
  public static getDatabaseStats() {
    if (!this.dbService) {
      console.warn('Database service not initialized');
      return null;
    }

    return this.dbService.getDatabaseStats();
  }

  /**
   * データ整合性チェックを実行
   */
  public static validateDatabaseIntegrity() {
    if (!this.dbService) {
      console.warn('Database service not initialized');
      return { isValid: false, errors: ['Database service not initialized'] };
    }

    return this.dbService.validateDataIntegrity();
  }

  /**
   * 古いデータのクリーンアップを実行
   */
  public static cleanupOldData() {
    if (!this.dbService) {
      console.warn('Database service not initialized');
      return { deletedStatusRecords: 0, deletedCommandRecords: 0 };
    }

    return this.dbService.cleanupOldData();
  }

  /**
   * データベースサービスを取得
   */
  public static getDatabaseService(): DatabaseService | null {
    return this.dbService || null;
  }

  /**
   * 統計サービスを取得
   */
  public static getStatisticsService(): StatisticsService | null {
    return this.statisticsService || null;
  }
}