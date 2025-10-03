import { EventEmitter } from 'events';
import { RobotCommand, RobotCommandResponse } from '../../types';
import { WebSocketService } from './WebSocketService';

export interface RobotCommandLog {
  id: string;
  command: RobotCommand;
  response?: RobotCommandResponse;
  status: 'pending' | 'success' | 'error' | 'timeout';
  timestamp: Date;
  retryCount: number;
  error?: string;
}

export interface RobotCommandManagerConfig {
  maxRetries: number;
  retryDelay: number;
  commandTimeout: number;
  enableAutoRetry: boolean;
}

export class RobotCommandManager extends EventEmitter {
  private webSocketService: WebSocketService;
  private config: RobotCommandManagerConfig;
  private commandLogs: Map<string, RobotCommandLog> = new Map();
  private pendingCommands: Map<string, NodeJS.Timeout> = new Map();
  private retryTimers: Map<string, NodeJS.Timeout> = new Map();

  constructor(webSocketService: WebSocketService, config?: Partial<RobotCommandManagerConfig>) {
    super();
    this.webSocketService = webSocketService;
    this.config = {
      maxRetries: 3,
      retryDelay: 2000,
      commandTimeout: 10000,
      enableAutoRetry: true,
      ...config
    };
  }

  /**
   * ロボットに指示を送信
   */
  public async sendCommand(
    type: 'tool_handover' | 'next_task',
    parameters?: Record<string, any>
  ): Promise<RobotCommandResponse> {
    const command: RobotCommand = {
      id: this.generateCommandId(),
      type,
      timestamp: new Date().toISOString(),
      parameters
    };

    console.log('Sending robot command:', command);

    // コマンドログを作成
    const commandLog: RobotCommandLog = {
      id: command.id,
      command,
      status: 'pending',
      timestamp: new Date(),
      retryCount: 0
    };

    this.commandLogs.set(command.id, commandLog);
    this.emit('command_sent', commandLog);

    try {
      const response = await this.executeCommand(command);
      this.handleCommandSuccess(command.id, response);
      return response;
    } catch (error) {
      this.handleCommandError(command.id, error as Error);
      throw error;
    }
  }

  /**
   * 閾値達成時の自動指示送信
   */
  public async sendThresholdCommand(
    reason: 'screw_threshold_reached' | 'bolt_threshold_reached',
    data: any
  ): Promise<void> {
    try {
      let commandType: 'tool_handover' | 'next_task';
      let parameters: Record<string, any>;

      if (reason === 'screw_threshold_reached') {
        commandType = 'tool_handover';
        parameters = {
          reason: 'screw_work_completed',
          screwCount: data.screwCount,
          timestamp: data.timestamp
        };
      } else {
        commandType = 'next_task';
        parameters = {
          reason: 'bolt_work_completed',
          boltCount: data.boltCount,
          timestamp: data.timestamp
        };
      }

      const response = await this.sendCommand(commandType, parameters);
      
      // 成功通知を送信
      this.emit('threshold_command_success', {
        reason,
        command: commandType,
        response,
        data
      });

      console.log(`Threshold command sent successfully: ${commandType}`, response);
    } catch (error) {
      // エラー通知を送信
      this.emit('threshold_command_error', {
        reason,
        error: error as Error,
        data
      });

      console.error(`Failed to send threshold command for ${reason}:`, error);
    }
  }

  /**
   * コマンドを実行
   */
  private async executeCommand(command: RobotCommand): Promise<RobotCommandResponse> {
    return new Promise((resolve, reject) => {
      // WebSocket接続チェック
      if (!this.webSocketService.isConnected()) {
        reject(new Error('WebSocket not connected'));
        return;
      }

      // タイムアウトタイマーを設定
      const timeoutTimer = setTimeout(() => {
        this.pendingCommands.delete(command.id);
        reject(new Error(`Command timeout: ${command.id}`));
      }, this.config.commandTimeout);

      this.pendingCommands.set(command.id, timeoutTimer);

      // レスポンスハンドラーを設定
      const responseHandler = (response: RobotCommandResponse) => {
        if (response.commandId === command.id) {
          clearTimeout(timeoutTimer);
          this.pendingCommands.delete(command.id);
          this.webSocketService.off('robot-response', responseHandler);

          if (response.status === 'success') {
            resolve(response);
          } else {
            reject(new Error(response.message));
          }
        }
      };

      this.webSocketService.on('robot-response', responseHandler);

      // コマンドを送信
      const success = this.webSocketService.sendRobotCommand('robot_command', {
        ...command,
        timestamp: new Date().toISOString()
      });

      if (!success) {
        clearTimeout(timeoutTimer);
        this.pendingCommands.delete(command.id);
        this.webSocketService.off('robot-response', responseHandler);
        reject(new Error('Failed to send command via WebSocket'));
      }
    });
  }

  /**
   * コマンド成功を処理
   */
  private handleCommandSuccess(commandId: string, response: RobotCommandResponse): void {
    const commandLog = this.commandLogs.get(commandId);
    if (commandLog) {
      commandLog.status = 'success';
      commandLog.response = response;
      this.emit('command_success', commandLog);
    }
  }

  /**
   * コマンドエラーを処理
   */
  private handleCommandError(commandId: string, error: Error): void {
    const commandLog = this.commandLogs.get(commandId);
    if (!commandLog) return;

    commandLog.status = 'error';
    commandLog.error = error.message;

    // 自動リトライが有効で、最大リトライ回数に達していない場合
    if (this.config.enableAutoRetry && commandLog.retryCount < this.config.maxRetries) {
      this.scheduleRetry(commandLog);
    } else {
      this.emit('command_error', commandLog);
    }
  }

  /**
   * リトライをスケジュール
   */
  private scheduleRetry(commandLog: RobotCommandLog): void {
    const delay = this.config.retryDelay * Math.pow(2, commandLog.retryCount); // 指数バックオフ
    
    console.log(`Scheduling retry for command ${commandLog.id} in ${delay}ms (attempt ${commandLog.retryCount + 1})`);

    const retryTimer = setTimeout(async () => {
      this.retryTimers.delete(commandLog.id);
      commandLog.retryCount++;
      commandLog.status = 'pending';

      try {
        const response = await this.executeCommand(commandLog.command);
        this.handleCommandSuccess(commandLog.id, response);
      } catch (error) {
        this.handleCommandError(commandLog.id, error as Error);
      }
    }, delay);

    this.retryTimers.set(commandLog.id, retryTimer);
    this.emit('command_retry_scheduled', commandLog);
  }

  /**
   * 手動でコマンドをリトライ
   */
  public async retryCommand(commandId: string): Promise<RobotCommandResponse> {
    const commandLog = this.commandLogs.get(commandId);
    if (!commandLog) {
      throw new Error(`Command not found: ${commandId}`);
    }

    if (commandLog.status === 'pending') {
      throw new Error(`Command is already pending: ${commandId}`);
    }

    // 既存のリトライタイマーをクリア
    const existingTimer = this.retryTimers.get(commandId);
    if (existingTimer) {
      clearTimeout(existingTimer);
      this.retryTimers.delete(commandId);
    }

    commandLog.retryCount++;
    commandLog.status = 'pending';
    commandLog.error = undefined;

    try {
      const response = await this.executeCommand(commandLog.command);
      this.handleCommandSuccess(commandId, response);
      return response;
    } catch (error) {
      this.handleCommandError(commandId, error as Error);
      throw error;
    }
  }

  /**
   * コマンドをキャンセル
   */
  public cancelCommand(commandId: string): boolean {
    const pendingTimer = this.pendingCommands.get(commandId);
    const retryTimer = this.retryTimers.get(commandId);

    if (pendingTimer) {
      clearTimeout(pendingTimer);
      this.pendingCommands.delete(commandId);
    }

    if (retryTimer) {
      clearTimeout(retryTimer);
      this.retryTimers.delete(commandId);
    }

    const commandLog = this.commandLogs.get(commandId);
    if (commandLog && commandLog.status === 'pending') {
      commandLog.status = 'error';
      commandLog.error = 'Command cancelled by user';
      this.emit('command_cancelled', commandLog);
      return true;
    }

    return false;
  }

  /**
   * コマンドログを取得
   */
  public getCommandLogs(): RobotCommandLog[] {
    return Array.from(this.commandLogs.values()).sort(
      (a, b) => b.timestamp.getTime() - a.timestamp.getTime()
    );
  }

  /**
   * 特定のコマンドログを取得
   */
  public getCommandLog(commandId: string): RobotCommandLog | undefined {
    return this.commandLogs.get(commandId);
  }

  /**
   * 統計情報を取得
   */
  public getStatistics() {
    const logs = Array.from(this.commandLogs.values());
    const total = logs.length;
    const successful = logs.filter(log => log.status === 'success').length;
    const failed = logs.filter(log => log.status === 'error').length;
    const pending = logs.filter(log => log.status === 'pending').length;

    return {
      total,
      successful,
      failed,
      pending,
      successRate: total > 0 ? (successful / total) * 100 : 0,
      averageRetries: total > 0 ? logs.reduce((sum, log) => sum + log.retryCount, 0) / total : 0
    };
  }

  /**
   * 設定を更新
   */
  public updateConfig(newConfig: Partial<RobotCommandManagerConfig>): void {
    this.config = { ...this.config, ...newConfig };
    this.emit('config_updated', this.config);
  }

  /**
   * ログをクリア
   */
  public clearLogs(): void {
    this.commandLogs.clear();
    this.emit('logs_cleared');
  }

  /**
   * コマンドIDを生成
   */
  private generateCommandId(): string {
    return `cmd_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * マネージャーを破棄
   */
  public destroy(): void {
    // 全てのタイマーをクリア
    this.pendingCommands.forEach(timer => clearTimeout(timer));
    this.retryTimers.forEach(timer => clearTimeout(timer));
    
    this.pendingCommands.clear();
    this.retryTimers.clear();
    this.commandLogs.clear();
    
    this.removeAllListeners();
  }
}