import { EventEmitter } from 'events';
import { CommandData } from '../shared/types';
import { DataProcessor } from './data-processor';

export interface CommandHistoryEntry {
  id: string;
  command: CommandData;
  timestamp: Date;
  success: boolean;
  error?: string;
  responseTime?: number;
}

export interface CommandSendResult {
  success: boolean;
  commandId: string;
  error?: string;
  responseTime: number;
}

/**
 * コマンド送信とフィードバック管理を行うクラス
 */
export class CommandManager extends EventEmitter {
  private commandHistory: CommandHistoryEntry[] = [];
  private pendingCommands: Map<string, {
    command: CommandData;
    timestamp: Date;
    resolve: (result: CommandSendResult) => void;
    reject: (error: Error) => void;
    timeout: NodeJS.Timeout;
  }> = new Map();
  
  private readonly maxHistorySize: number = 1000;
  private readonly commandTimeout: number = 10000; // 10秒

  constructor() {
    super();
  }

  /**
   * コマンドを作成（タイムスタンプ付き）
   */
  public createCommand(command: 'tool_handover' | 'tool_collection' | 'wait'): CommandData {
    const timestamp = DataProcessor.formatTimestamp(new Date());
    
    return {
      command,
      timestamp
    };
  }

  /**
   * コマンドを送信し、結果を追跡
   */
  public async sendCommand(
    command: CommandData,
    sendFunction: (command: CommandData) => Promise<boolean>
  ): Promise<CommandSendResult> {
    const commandId = this.generateCommandId();
    const startTime = Date.now();

    console.log(`Sending command ${commandId}:`, command);

    return new Promise((resolve, reject) => {
      // タイムアウト設定
      const timeout = setTimeout(() => {
        this.pendingCommands.delete(commandId);
        const error = new Error(`Command timeout after ${this.commandTimeout}ms`);
        
        const historyEntry: CommandHistoryEntry = {
          id: commandId,
          command,
          timestamp: new Date(),
          success: false,
          error: error.message,
          responseTime: Date.now() - startTime
        };
        
        this.addToHistory(historyEntry);
        this.emit('commandTimeout', { commandId, command, error: error.message });
        
        reject(error);
      }, this.commandTimeout);

      // 保留中のコマンドとして登録
      this.pendingCommands.set(commandId, {
        command,
        timestamp: new Date(),
        resolve,
        reject,
        timeout
      });

      // 実際の送信処理
      sendFunction(command)
        .then((success) => {
          const responseTime = Date.now() - startTime;
          
          if (success) {
            const result: CommandSendResult = {
              success: true,
              commandId,
              responseTime
            };

            const historyEntry: CommandHistoryEntry = {
              id: commandId,
              command,
              timestamp: new Date(),
              success: true,
              responseTime
            };

            this.addToHistory(historyEntry);
            this.completePendingCommand(commandId, result);
            this.emit('commandSent', { commandId, command, result });
          } else {
            const error = 'Command send failed';
            const result: CommandSendResult = {
              success: false,
              commandId,
              error,
              responseTime
            };

            const historyEntry: CommandHistoryEntry = {
              id: commandId,
              command,
              timestamp: new Date(),
              success: false,
              error,
              responseTime
            };

            this.addToHistory(historyEntry);
            this.completePendingCommand(commandId, result);
            this.emit('commandFailed', { commandId, command, error });
          }
        })
        .catch((error) => {
          const responseTime = Date.now() - startTime;
          const errorMessage = error.message || 'Unknown error';
          
          const result: CommandSendResult = {
            success: false,
            commandId,
            error: errorMessage,
            responseTime
          };

          const historyEntry: CommandHistoryEntry = {
            id: commandId,
            command,
            timestamp: new Date(),
            success: false,
            error: errorMessage,
            responseTime
          };

          this.addToHistory(historyEntry);
          this.completePendingCommand(commandId, result, error);
          this.emit('commandError', { commandId, command, error: errorMessage });
        });
    });
  }

  /**
   * コマンド履歴を取得
   */
  public getCommandHistory(limit?: number): CommandHistoryEntry[] {
    const history = [...this.commandHistory].reverse(); // 新しい順
    return limit ? history.slice(0, limit) : history;
  }

  /**
   * 特定期間のコマンド履歴を取得
   */
  public getCommandHistoryByTimeRange(
    startTime: Date,
    endTime: Date
  ): CommandHistoryEntry[] {
    return this.commandHistory.filter(entry => 
      entry.timestamp >= startTime && entry.timestamp <= endTime
    );
  }

  /**
   * コマンド統計を取得
   */
  public getCommandStatistics(timeRange?: { start: Date; end: Date }) {
    let entries = this.commandHistory;
    
    if (timeRange) {
      entries = this.getCommandHistoryByTimeRange(timeRange.start, timeRange.end);
    }

    const totalCommands = entries.length;
    const successfulCommands = entries.filter(entry => entry.success).length;
    const failedCommands = totalCommands - successfulCommands;
    const successRate = totalCommands > 0 ? (successfulCommands / totalCommands) * 100 : 0;

    // コマンド種別ごとの統計
    const commandCounts = entries.reduce((counts, entry) => {
      const command = entry.command.command;
      counts[command] = (counts[command] || 0) + 1;
      return counts;
    }, {} as Record<string, number>);

    // 平均応答時間
    const responseTimes = entries
      .filter(entry => entry.responseTime !== undefined)
      .map(entry => entry.responseTime!);
    
    const averageResponseTime = responseTimes.length > 0
      ? responseTimes.reduce((sum, time) => sum + time, 0) / responseTimes.length
      : 0;

    return {
      totalCommands,
      successfulCommands,
      failedCommands,
      successRate,
      commandCounts,
      averageResponseTime,
      timeRange: timeRange || null
    };
  }

  /**
   * 保留中のコマンド数を取得
   */
  public getPendingCommandsCount(): number {
    return this.pendingCommands.size;
  }

  /**
   * 保留中のコマンドをすべてキャンセル
   */
  public cancelAllPendingCommands(): void {
    for (const [commandId, pendingCommand] of this.pendingCommands) {
      clearTimeout(pendingCommand.timeout);
      
      const error = new Error('Command cancelled');
      const historyEntry: CommandHistoryEntry = {
        id: commandId,
        command: pendingCommand.command,
        timestamp: pendingCommand.timestamp,
        success: false,
        error: error.message,
        responseTime: Date.now() - pendingCommand.timestamp.getTime()
      };
      
      this.addToHistory(historyEntry);
      pendingCommand.reject(error);
    }
    
    this.pendingCommands.clear();
    this.emit('allCommandsCancelled');
  }

  /**
   * コマンド履歴をクリア
   */
  public clearHistory(): void {
    this.commandHistory = [];
    this.emit('historyCleared');
  }

  /**
   * コマンドIDを生成
   */
  private generateCommandId(): string {
    const timestamp = Date.now();
    const random = Math.random().toString(36).substring(2, 8);
    return `cmd_${timestamp}_${random}`;
  }

  /**
   * 履歴にエントリを追加
   */
  private addToHistory(entry: CommandHistoryEntry): void {
    this.commandHistory.push(entry);
    
    // データベースに保存
    const saved = DataProcessor.saveCommandData(entry.command, entry.success);
    if (!saved) {
      console.warn('Failed to save command to database:', entry.command);
    }
    
    // 履歴サイズ制限
    if (this.commandHistory.length > this.maxHistorySize) {
      this.commandHistory = this.commandHistory.slice(-this.maxHistorySize);
    }
    
    this.emit('historyUpdated', entry);
  }

  /**
   * 保留中のコマンドを完了
   */
  private completePendingCommand(
    commandId: string,
    result: CommandSendResult,
    error?: Error
  ): void {
    const pendingCommand = this.pendingCommands.get(commandId);
    
    if (pendingCommand) {
      clearTimeout(pendingCommand.timeout);
      this.pendingCommands.delete(commandId);
      
      if (error) {
        pendingCommand.reject(error);
      } else {
        pendingCommand.resolve(result);
      }
    }
  }

  /**
   * コマンドデータの検証
   */
  public static validateCommand(command: CommandData): boolean {
    if (!command || typeof command !== 'object') {
      return false;
    }

    // commandフィールドの検証
    const validCommands = ['tool_handover', 'tool_collection', 'wait'];
    if (!validCommands.includes(command.command)) {
      return false;
    }

    // timestampフィールドの検証
    if (typeof command.timestamp !== 'string' || !/^\d{14}$/.test(command.timestamp)) {
      return false;
    }

    return true;
  }
}