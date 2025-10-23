import Database from 'better-sqlite3';
import path from 'path';
import { app } from 'electron';
import { StatusData, CommandData, StatusHistoryRecord, CommandHistoryRecord, WorkStatisticsRecord } from '../shared/types';
import { APP_CONFIG } from '../shared/constants';

export class DatabaseService {
  public db: Database.Database; // Made public for statistics service access
  private dbPath: string;

  constructor() {
    // データベースファイルをユーザーデータディレクトリに配置
    const userDataPath = app.getPath('userData');
    this.dbPath = path.join(userDataPath, APP_CONFIG.DB_FILE_NAME);
    
    this.db = new Database(this.dbPath);
    this.initializeDatabase();
  }

  /**
   * データベースの初期化とテーブル作成
   */
  private initializeDatabase(): void {
    // WALモードを有効にしてパフォーマンスを向上
    this.db.pragma('journal_mode = WAL');
    
    // 状態履歴テーブルの作成
    this.db.exec(`
      CREATE TABLE IF NOT EXISTS status_history (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp TEXT NOT NULL,
        worker_status TEXT NOT NULL,
        space_status TEXT NOT NULL,
        robot_state TEXT NOT NULL,
        robot_grip TEXT NOT NULL,
        tool_delivery INTEGER NOT NULL,
        demo_status TEXT NOT NULL,
        created_at DATETIME DEFAULT CURRENT_TIMESTAMP
      )
    `);

    // コマンド履歴テーブルの作成
    this.db.exec(`
      CREATE TABLE IF NOT EXISTS command_history (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp TEXT NOT NULL,
        command TEXT NOT NULL,
        success BOOLEAN NOT NULL,
        created_at DATETIME DEFAULT CURRENT_TIMESTAMP
      )
    `);

    // 作業統計テーブルの作成
    this.db.exec(`
      CREATE TABLE IF NOT EXISTS work_statistics (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        date TEXT NOT NULL UNIQUE,
        screw_count INTEGER DEFAULT 0,
        blocks_count INTEGER DEFAULT 0,
        survey_count INTEGER DEFAULT 0,
        total_work_time INTEGER DEFAULT 0,
        average_work_time REAL DEFAULT 0,
        created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
        updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
      )
    `);

    // インデックスの作成（パフォーマンス向上のため）
    this.db.exec(`
      CREATE INDEX IF NOT EXISTS idx_status_timestamp ON status_history(timestamp);
      CREATE INDEX IF NOT EXISTS idx_status_created_at ON status_history(created_at);
      CREATE INDEX IF NOT EXISTS idx_command_timestamp ON command_history(timestamp);
      CREATE INDEX IF NOT EXISTS idx_command_created_at ON command_history(created_at);
      CREATE INDEX IF NOT EXISTS idx_statistics_date ON work_statistics(date);
    `);

    console.log('Database initialized successfully at:', this.dbPath);
  }

  /**
   * 状態データの保存
   */
  public saveStatusData(statusData: StatusData): boolean {
    try {
      const stmt = this.db.prepare(`
        INSERT INTO status_history (
          timestamp, worker_status, space_status, robot_state, 
          robot_grip, tool_delivery, demo_status
        ) VALUES (?, ?, ?, ?, ?, ?, ?)
      `);

      const result = stmt.run(
        statusData.timestamp,
        statusData.worker_status,
        statusData.space_status,
        statusData.robot_status.state,
        statusData.robot_status.grip,
        statusData.tool_delivery,
        statusData.status
      );

      return result.changes > 0;
    } catch (error) {
      console.error('Error saving status data:', error);
      return false;
    }
  }

  /**
   * コマンド履歴の保存
   */
  public saveCommandHistory(commandData: CommandData, success: boolean): boolean {
    try {
      const stmt = this.db.prepare(`
        INSERT INTO command_history (timestamp, command, success)
        VALUES (?, ?, ?)
      `);

      const result = stmt.run(
        commandData.timestamp,
        commandData.command,
        success
      );

      return result.changes > 0;
    } catch (error) {
      console.error('Error saving command history:', error);
      return false;
    }
  }

  /**
   * 状態履歴の取得（時間範囲指定）
   */
  public getStatusHistory(startTime?: string, endTime?: string, limit: number = 1000): StatusHistoryRecord[] {
    try {
      let query = 'SELECT * FROM status_history';
      const params: any[] = [];

      if (startTime || endTime) {
        query += ' WHERE';
        if (startTime) {
          query += ' timestamp >= ?';
          params.push(startTime);
        }
        if (endTime) {
          if (startTime) query += ' AND';
          query += ' timestamp <= ?';
          params.push(endTime);
        }
      }

      query += ' ORDER BY timestamp DESC LIMIT ?';
      params.push(limit);

      const stmt = this.db.prepare(query);
      return stmt.all(...params) as StatusHistoryRecord[];
    } catch (error) {
      console.error('Error getting status history:', error);
      return [];
    }
  }

  /**
   * コマンド履歴の取得
   */
  public getCommandHistory(limit: number = 100): CommandHistoryRecord[] {
    try {
      const stmt = this.db.prepare(`
        SELECT * FROM command_history 
        ORDER BY created_at DESC 
        LIMIT ?
      `);
      return stmt.all(limit) as CommandHistoryRecord[];
    } catch (error) {
      console.error('Error getting command history:', error);
      return [];
    }
  }

  /**
   * 最新の状態データを取得
   */
  public getLatestStatus(): StatusHistoryRecord | null {
    try {
      const stmt = this.db.prepare(`
        SELECT * FROM status_history 
        ORDER BY created_at DESC 
        LIMIT 1
      `);
      return stmt.get() as StatusHistoryRecord || null;
    } catch (error) {
      console.error('Error getting latest status:', error);
      return null;
    }
  }

  /**
   * データ整合性チェック
   */
  public validateDataIntegrity(): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];

    try {
      // 必須フィールドのNULLチェック
      const nullStatusCheck = this.db.prepare(`
        SELECT COUNT(*) as count FROM status_history 
        WHERE timestamp IS NULL OR worker_status IS NULL OR space_status IS NULL
      `).get() as { count: number };

      if (nullStatusCheck.count > 0) {
        errors.push(`Found ${nullStatusCheck.count} status records with NULL required fields`);
      }

      const nullCommandCheck = this.db.prepare(`
        SELECT COUNT(*) as count FROM command_history 
        WHERE timestamp IS NULL OR command IS NULL
      `).get() as { count: number };

      if (nullCommandCheck.count > 0) {
        errors.push(`Found ${nullCommandCheck.count} command records with NULL required fields`);
      }

      // タイムスタンプ形式チェック（YYYYMMDDhhmmss）
      const invalidTimestampCheck = this.db.prepare(`
        SELECT COUNT(*) as count FROM status_history 
        WHERE LENGTH(timestamp) != 14 OR timestamp NOT GLOB '[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]'
      `).get() as { count: number };

      if (invalidTimestampCheck.count > 0) {
        errors.push(`Found ${invalidTimestampCheck.count} records with invalid timestamp format`);
      }

      return {
        isValid: errors.length === 0,
        errors
      };
    } catch (error) {
      errors.push(`Database integrity check failed: ${error}`);
      return {
        isValid: false,
        errors
      };
    }
  }

  /**
   * 古いデータの削除（データ保持期間を超えたもの）
   */
  public cleanupOldData(): { deletedStatusRecords: number; deletedCommandRecords: number } {
    try {
      const cutoffDate = new Date();
      cutoffDate.setDate(cutoffDate.getDate() - APP_CONFIG.DATA_RETENTION_DAYS);
      const cutoffTimestamp = cutoffDate.toISOString();

      const deleteStatusStmt = this.db.prepare(`
        DELETE FROM status_history 
        WHERE created_at < ?
      `);
      const statusResult = deleteStatusStmt.run(cutoffTimestamp);

      const deleteCommandStmt = this.db.prepare(`
        DELETE FROM command_history 
        WHERE created_at < ?
      `);
      const commandResult = deleteCommandStmt.run(cutoffTimestamp);

      console.log(`Cleaned up ${statusResult.changes} status records and ${commandResult.changes} command records`);

      return {
        deletedStatusRecords: statusResult.changes,
        deletedCommandRecords: commandResult.changes
      };
    } catch (error) {
      console.error('Error cleaning up old data:', error);
      return {
        deletedStatusRecords: 0,
        deletedCommandRecords: 0
      };
    }
  }

  /**
   * データベース接続を閉じる
   */
  public close(): void {
    try {
      this.db.close();
      console.log('Database connection closed');
    } catch (error) {
      console.error('Error closing database:', error);
    }
  }

  /**
   * データベースの統計情報を取得
   */
  public getDatabaseStats(): { statusRecords: number; commandRecords: number; statisticsRecords: number } {
    try {
      const statusCount = this.db.prepare('SELECT COUNT(*) as count FROM status_history').get() as { count: number };
      const commandCount = this.db.prepare('SELECT COUNT(*) as count FROM command_history').get() as { count: number };
      const statisticsCount = this.db.prepare('SELECT COUNT(*) as count FROM work_statistics').get() as { count: number };

      return {
        statusRecords: statusCount.count,
        commandRecords: commandCount.count,
        statisticsRecords: statisticsCount.count
      };
    } catch (error) {
      console.error('Error getting database stats:', error);
      return {
        statusRecords: 0,
        commandRecords: 0,
        statisticsRecords: 0
      };
    }
  }
}

// シングルトンインスタンス
let databaseService: DatabaseService | null = null;

export function getDatabaseService(): DatabaseService {
  if (!databaseService) {
    databaseService = new DatabaseService();
  }
  return databaseService;
}

export function closeDatabaseService(): void {
  if (databaseService) {
    databaseService.close();
    databaseService = null;
  }
}