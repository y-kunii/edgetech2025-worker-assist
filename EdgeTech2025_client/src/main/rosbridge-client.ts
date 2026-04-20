import WebSocket from 'ws';
import { EventEmitter } from 'events';
import { StatusData, CommandData } from '../shared/types';
import { DataProcessor, DataValidationError } from './data-processor';
import { CommandManager, CommandSendResult } from './command-manager';

export interface RosBridgeClientConfig {
  url: string;
  reconnectInterval: number;
  maxReconnectAttempts: number;
  heartbeatInterval: number;
  subscribeTopic?: string;
  subscribeType?: string;
  publishTopic?: string;
  publishType?: string;
  // 追加のSubscribeトピック
  additionalSubscriptions?: Array<{
    topic: string;
    type: string;
  }>;
}

export interface ConnectionState {
  isConnected: boolean;
  lastHeartbeat: Date | null;
  reconnectAttempts: number;
  error: string | null;
}

/**
 * ROSBridge Protocol Message Types
 */
interface RosBridgeSubscribe {
  op: 'subscribe';
  topic: string;
  type?: string;
  throttle_rate?: number;
  queue_length?: number;
}

interface RosBridgePublish {
  op: 'publish';
  topic: string;
  type?: string;
  msg: any;
}

interface RosBridgeMessage {
  op: string;
  topic?: string;
  msg?: any;
  type?: string;
}

/**
 * ROSBridge WebSocketクライアント
 * EventEmitterを継承し、標準的なイベント駆動型のインターフェースを提供
 */
export class RosBridgeClient extends EventEmitter {
  private ws: WebSocket | null = null;
  private config: RosBridgeClientConfig;
  private connectionState: ConnectionState;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private heartbeatTimer: NodeJS.Timeout | null = null;
  private isReconnecting: boolean = false;
  private lastStatusData: StatusData | null = null;
  private commandManager: CommandManager;
  private isSubscribed: boolean = false;

  constructor(config: RosBridgeClientConfig) {
    super();
    
    // デフォルト値の設定
    this.config = {
      subscribeTopic: '/action_analysis_topic',
      subscribeType: 'twin_bridge/msg/ActionAnalysisMsg',
      publishTopic: '/ws/pick_and_place_topic',
      publishType: 'std_msgs/msg/String',
      additionalSubscriptions: [
        {
          topic: '/ws/operating_status_topic',
          type: 'std_msgs/msg/String'
        },
        {
          topic: '/ws/gripper_status_topic',
          type: 'std_msgs/msg/String'
        }
      ],
      ...config
    };

    this.connectionState = {
      isConnected: false,
      lastHeartbeat: null,
      reconnectAttempts: 0,
      error: null
    };
    
    this.commandManager = new CommandManager();
    this.setupCommandManagerEvents();
  }

  /**
   * ROSBridge WebSocket接続を開始
   */
  public connect(): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      console.log('ROSBridge WebSocket is already connected');
      return;
    }

    try {
      console.log(`Connecting to ROSBridge server: ${this.config.url}`);
      this.ws = new WebSocket(this.config.url);
      
      this.setupEventHandlers();
    } catch (error) {
      console.error('Failed to create ROSBridge WebSocket connection:', error);
      this.handleConnectionError(error as Error);
    }
  }

  /**
   * ROSBridge WebSocket接続を切断
   */
  public disconnect(): void {
    console.log('Disconnecting ROSBridge WebSocket...');
    this.isReconnecting = false;
    this.isSubscribed = false;
    
    this.clearTimers();
    
    // 保留中のコマンドをキャンセル
    this.commandManager.cancelAllPendingCommands();
    
    if (this.ws) {
      // Unsubscribe before closing
      if (this.config.subscribeTopic) {
        this.unsubscribe(this.config.subscribeTopic);
      }
      
      this.ws.close(1000, 'Client disconnect');
      this.ws = null;
    }
    
    this.updateConnectionState({
      isConnected: false,
      lastHeartbeat: null,
      reconnectAttempts: 0,
      error: null
    });
  }

  /**
   * ROSトピックをサブスクライブ
   */
  private subscribe(topic: string, type?: string): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      console.error('Cannot subscribe: WebSocket is not connected');
      return;
    }

    const subscribeMsg: RosBridgeSubscribe = {
      op: 'subscribe',
      topic: topic,
    };

    if (type) {
      subscribeMsg.type = type;
    }

    try {
      const message = JSON.stringify(subscribeMsg);
      console.log('Sending ROSBridge subscribe:', message);
      this.ws.send(message);
      this.isSubscribed = true;
      this.emit('subscribed', { topic, type });
    } catch (error) {
      console.error('Failed to send subscribe message:', error);
      this.emit('error', error);
    }
  }

  /**
   * ROSトピックのサブスクライブを解除
   */
  private unsubscribe(topic: string): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    const unsubscribeMsg = {
      op: 'unsubscribe',
      topic: topic,
    };

    try {
      const message = JSON.stringify(unsubscribeMsg);
      console.log('Sending ROSBridge unsubscribe:', message);
      this.ws.send(message);
      this.isSubscribed = false;
    } catch (error) {
      console.error('Failed to send unsubscribe message:', error);
    }
  }

  /**
   * ROSトピックにメッセージをパブリッシュ
   */
  private publish(topic: string, msg: any, type?: string): Promise<boolean> {
    return new Promise((resolve, reject) => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
        const error = new Error('WebSocket is not connected');
        console.error('Cannot publish:', error.message);
        reject(error);
        return;
      }

      const publishMsg: RosBridgePublish = {
        op: 'publish',
        topic: topic,
        msg: msg,
      };

      if (type) {
        publishMsg.type = type;
      }

      try {
        const message = JSON.stringify(publishMsg);
        console.log('========================================');
        console.log('[PUBLISH] Sending to ROSBridge:');
        console.log('  Topic:', topic);
        console.log('  Type:', type);
        console.log('  Message:', JSON.stringify(msg, null, 2));
        console.log('  Full ROSBridge message:', message);
        console.log('========================================');
        
        this.ws.send(message, (error) => {
          if (error) {
            console.error('[PUBLISH ERROR] Failed to publish message:', error);
            reject(error);
          } else {
            console.log('[PUBLISH SUCCESS] Message sent successfully to', topic);
            resolve(true);
          }
        });
      } catch (error) {
        console.error('[PUBLISH ERROR] Failed to serialize publish message:', error);
        reject(error);
      }
    });
  }

  /**
   * コマンドデータを送信（履歴管理付き）
   */
  public async sendCommand(command: CommandData): Promise<CommandSendResult> {
    // コマンドの検証
    if (!CommandManager.validateCommand(command)) {
      throw new Error('Invalid command data');
    }

    // 実際に送信されるROSメッセージ（std_msgs/String形式）
    const actualRosMessage = {
      data: command.command  // "motion1" ~ "motion6"
    };

    // CommandManagerを使用して送信（TOPIC、TYPE、実際のメッセージ情報を含める）
    return this.commandManager.sendCommand(
      command, 
      async (cmd) => {
        return this.sendRawCommand(cmd);
      },
      {
        topic: this.config.publishTopic,
        type: this.config.publishType,
        direction: 'publish',
        message: actualRosMessage  // 実際に送信されるメッセージを記録
      }
    );
  }

  /**
   * 新しいコマンドを作成して送信
   */
  public async createAndSendCommand(commandType: string): Promise<CommandSendResult> {
    const command = this.commandManager.createCommand(commandType);
    return this.sendCommand(command);
  }

  /**
   * 生のコマンドデータをROSBridge経由で送信（内部使用）
   * std_msgs/String形式でコマンド文字列のみを送信
   */
  private sendRawCommand(command: CommandData): Promise<boolean> {
    if (!this.config.publishTopic) {
      return Promise.reject(new Error('Publish topic is not configured'));
    }

    // std_msgs/String形式：文字列のみ（"motion1" ~ "motion6"）
    const rosMsg = {
      data: command.command  // "motion1" ~ "motion6"
    };

    return this.publish(
      this.config.publishTopic,
      rosMsg,
      this.config.publishType
    );
  }

  /**
   * 現在の接続状態を取得
   */
  public getConnectionState(): ConnectionState {
    return { ...this.connectionState };
  }

  /**
   * 最後に受信したStatusDataを取得
   */
  public getLastStatusData(): StatusData | null {
    return this.lastStatusData ? { ...this.lastStatusData } : null;
  }

  /**
   * コマンドマネージャーを取得
   */
  public getCommandManager(): CommandManager {
    return this.commandManager;
  }

  /**
   * 設定を更新
   */
  public updateConfig(newConfig: Partial<RosBridgeClientConfig>): void {
    const wasConnected = this.connectionState.isConnected;
    const topicChanged = newConfig.subscribeTopic && newConfig.subscribeTopic !== this.config.subscribeTopic;

    this.config = { ...this.config, ...newConfig };
    console.log('ROSBridge config updated:', this.config);

    // トピックが変更された場合は再サブスクライブ
    if (wasConnected && topicChanged && this.config.subscribeTopic) {
      this.subscribe(this.config.subscribeTopic, this.config.subscribeType);
    }
  }

  /**
   * WebSocketイベントハンドラーを設定
   */
  private setupEventHandlers(): void {
    if (!this.ws) return;

    this.ws.on('open', () => {
      console.log('ROSBridge WebSocket connected successfully');
      this.updateConnectionState({
        isConnected: true,
        lastHeartbeat: new Date(),
        reconnectAttempts: 0,
        error: null
      });
      
      // 接続後、自動的にトピックをサブスクライブ
      if (this.config.subscribeTopic) {
        this.subscribe(this.config.subscribeTopic, this.config.subscribeType);
      }
      
      // 追加のトピックもサブスクライブ
      if (this.config.additionalSubscriptions) {
        this.config.additionalSubscriptions.forEach(sub => {
          this.subscribe(sub.topic, sub.type);
        });
      }
      
      this.startHeartbeat();
      this.emit('connected');
    });

    this.ws.on('message', (data: WebSocket.Data) => {
      try {
        const message = data.toString();
        console.log('Received ROSBridge message:', message);
        
        // ハートビート更新
        this.updateConnectionState({
          ...this.connectionState,
          lastHeartbeat: new Date()
        });

        // ROSBridgeメッセージをパース
        const rosBridgeMsg: RosBridgeMessage = JSON.parse(message);

        // publishメッセージのみ処理
        if (rosBridgeMsg.op === 'publish' && rosBridgeMsg.msg) {
          this.handlePublishedMessage(rosBridgeMsg);
        }

      } catch (error) {
        console.error('Failed to process received message:', error);
        this.emit('error', error);
      }
    });

    this.ws.on('close', (code: number, reason: Buffer) => {
      const reasonString = reason.toString();
      console.log(`ROSBridge WebSocket closed: ${code} - ${reasonString}`);
      
      this.isSubscribed = false;
      
      this.updateConnectionState({
        isConnected: false,
        lastHeartbeat: null,
        reconnectAttempts: this.connectionState.reconnectAttempts,
        error: `Connection closed: ${code} - ${reasonString}`
      });
      
      this.clearTimers();
      this.emit('disconnected', { code, reason: reasonString });
      
      // 自動再接続を試行（意図的な切断でない場合）
      if (code !== 1000 && !this.isReconnecting) {
        this.attemptReconnect();
      }
    });

    this.ws.on('error', (error: Error) => {
      console.error('ROSBridge WebSocket error:', error);
      this.handleConnectionError(error);
    });
  }

  /**
   * ROSBridgeから受信したパブリッシュメッセージを処理
   */
  private handlePublishedMessage(rosBridgeMsg: RosBridgeMessage): void {
    try {
      // 受信履歴を記録
      this.logReceivedMessage(rosBridgeMsg);
      
      // トピックごとに処理を分岐
      const topic = rosBridgeMsg.topic;
      
      if (topic === this.config.subscribeTopic) {
        // /action_analysis_topic: StatusDataに変換
        this.handleActionAnalysisMessage(rosBridgeMsg);
      } else if (topic === '/ws/operating_status_topic') {
        // /ws/operating_status_topic: std_msgs/String
        this.handleOperatingStatusMessage(rosBridgeMsg);
      } else if (topic === '/ws/gripper_status_topic') {
        // /ws/gripper_status_topic: std_msgs/String
        this.handleGripperStatusMessage(rosBridgeMsg);
      } else {
        console.log('Received message from unknown topic:', topic);
      }

    } catch (error) {
      console.error('Failed to handle published message:', error);
      this.emit('error', error);
    }
  }

  /**
   * アクション分析メッセージを処理（元の処理）
   */
  private handleActionAnalysisMessage(rosBridgeMsg: RosBridgeMessage): void {
    // ROSメッセージをStatusDataに変換
    const statusData = this.convertRosMessageToStatusData(rosBridgeMsg.msg);
    
    if (!statusData) {
      console.warn('Failed to convert ROS message to StatusData');
      return;
    }

    console.log('Converted status data:', statusData);

    // 状態変化の検出
    const changeInfo = DataProcessor.detectStatusChange(this.lastStatusData, statusData);
    console.log('Status change detected:', changeInfo);

    // データベースに状態データを保存（RosBridge経由の受信経路でも保存されるように）
    try {
      const dbService = DataProcessor.getDatabaseService();
      if (dbService) {
        const saved = dbService.saveStatusData(statusData);
        if (saved) {
          console.log('Status data saved to database (via RosBridgeClient)');
          // 統計を更新
          const statsService = DataProcessor.getStatisticsService();
          if (statsService) {
            // 非同期で統計更新を実行
            setTimeout(() => statsService.updateAllStatistics(), 0);
          }
        } else {
          console.warn('Failed to save status data to database (via RosBridgeClient)');
        }
      } else {
        console.warn('Database service not initialized; skipping status save');
      }
    } catch (err) {
      console.error('Error while saving status data from RosBridgeClient:', err);
    }

    // Status変化に応じた自動コマンド送信をチェック
    if (changeInfo.hasChanged && changeInfo.changedFields.includes('status') && this.lastStatusData) {
      const motionCommand = this.determineMotionCommand(
        this.lastStatusData.status,
        statusData.status,
        statusData.space_status
      );
      
      if (motionCommand) {
        console.log(`Auto-sending motion command: ${motionCommand}`);
        this.emit('autoMotionCommand', {
          command: motionCommand,
          previousStatus: this.lastStatusData.status,
          currentStatus: statusData.status,
          spaceStatus: statusData.space_status
        });
      }
    }

    // 前回のデータを更新
    this.lastStatusData = statusData;

    // イベントを発行
    this.emit('statusData', statusData);
    
    if (changeInfo.hasChanged) {
      this.emit('statusDataChanged', {
        statusData,
        changeInfo
      });
    }

    if (changeInfo.workTaskChanged) {
      this.emit('workTaskChanged', {
        previousTask: changeInfo.previousTask,
        currentTask: changeInfo.currentTask,
        statusData
      });
    }
  }

  /**
   * Status変化に応じたmotionコマンドを判定
   */
  private determineMotionCommand(
    previousStatus: string,
    currentStatus: string,
    spaceStatus: string
  ): string | null {
    // Waiting → Ready に変更: 待機中から準備完了に切り替わったタイミングでモーションを送信
    if (previousStatus === 'Waiting' && currentStatus === 'Ready') {
      if (spaceStatus === 'Screw_tightening') return 'motion1';
      if (spaceStatus === 'Building_blocks') return 'motion2';
      if (spaceStatus === 'Survey_responses') return 'motion3';
    }

    // Working → Work Completed
    if (previousStatus === 'Working' && currentStatus === 'Work Completed') {
      if (spaceStatus === 'Screw_tightening') return 'motion4';
      if (spaceStatus === 'Building_blocks') return 'motion5';
      if (spaceStatus === 'Survey_responses') return 'motion6';
    }

    return null;
  }

  /**
   * 動作状態メッセージを処理
   */
  private handleOperatingStatusMessage(rosBridgeMsg: RosBridgeMessage): void {
    const status = rosBridgeMsg.msg?.data; // "idle" or "operation"
    console.log('Operating status received:', status);
    this.emit('operatingStatus', status);
  }

  /**
   * グリッパー状態メッセージを処理
   */
  private handleGripperStatusMessage(rosBridgeMsg: RosBridgeMessage): void {
    const status = rosBridgeMsg.msg?.data; // "open" or "close"
    console.log('Gripper status received:', status);
    this.emit('gripperStatus', status);
  }

  /**
   * ROSメッセージをStatusDataに変換
   */
  private convertRosMessageToStatusData(rosMsg: any): StatusData | null {
    try {
      // ROSメッセージの構造に応じて変換
      // 必要に応じてフィールドマッピングを調整
      // 正規化を行って入力の大文字/小文字や表記揺れに対応
      const rawWorkerStatus = rosMsg.worker_status || rosMsg.status || 'Waiting';
      const rawStatus = rosMsg.status || rosMsg.worker_status || 'Waiting';
      const statusData: StatusData = {
        worker_status: this.normalizeStatus(rawWorkerStatus) as StatusData['worker_status'],
        space_status: rosMsg.space_status || 'Nothing',
        robot_status: {
          state: rosMsg.robot_status?.state || rosMsg.robot_state || 'idle',
          grip: rosMsg.robot_status?.grip || rosMsg.robot_grip || 'open',
        },
        timestamp: rosMsg.timestamp || new Date().toISOString(),
        tool_delivery: rosMsg.tool_delivery || 0,
        status: this.normalizeStatus(rawStatus) as StatusData['status'],
      };

      return statusData;
    } catch (error) {
      console.error('Failed to convert ROS message:', error);
      return null;
    }
  }

  /**
   * ステータス文字列を正規化して、コード内で期待する表記に揃える
   * 例: 'ready' -> 'Ready', 'work completed' -> 'Work Completed'
   */
  private normalizeStatus(value: any): string {
    // デフォルトは Waiting（待機）を返す
    if (!value || typeof value !== 'string') return 'Waiting';
    const v = value.trim().toLowerCase();
  // 旧フォーマットの 'absent' は新しいステータス体系では Waiting として扱う（互換性維持）
  if (v === 'absent') return 'Waiting';
  if (v === 'ready') return 'Ready';
    if (v === 'waiting') return 'Waiting';
    if (v === 'working') return 'Working';
    if (v === 'work completed' || v === 'work_completed' || v === 'workcompleted' || v === 'completed') return 'Work Completed';
    // フォールバック: 先頭大文字化
    return v.charAt(0).toUpperCase() + v.slice(1);
  }

  /**
   * 接続エラーを処理
   */
  private handleConnectionError(error: Error): void {
    this.updateConnectionState({
      isConnected: false,
      lastHeartbeat: null,
      reconnectAttempts: this.connectionState.reconnectAttempts,
      error: error.message
    });
    
    this.emit('error', error);
    
    if (!this.isReconnecting) {
      this.attemptReconnect();
    }
  }

  /**
   * 受信メッセージを履歴として記録
   */
  private logReceivedMessage(rosBridgeMsg: RosBridgeMessage): void {
    const logEntry = {
      id: this.generateMessageId(),
      timestamp: new Date(),
      direction: 'subscribe' as const,
      topic: rosBridgeMsg.topic || this.config.subscribeTopic || 'unknown',
      type: rosBridgeMsg.type || this.config.subscribeType || 'unknown',
      message: rosBridgeMsg.msg,
      success: true
    };
    
    console.log('[Subscribe] Received message:', {
      topic: logEntry.topic,
      type: logEntry.type,
      timestamp: logEntry.timestamp.toISOString()
    });
    
    // CommandManagerに記録
    this.commandManager.logReceivedMessage(logEntry);
    
    // イベントとして履歴を発行
    this.emit('messageReceived', logEntry);
  }

  /**
   * メッセージIDを生成
   */
  private generateMessageId(): string {
    return `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * 自動再接続を試行
   */
  private attemptReconnect(): void {
    if (this.connectionState.reconnectAttempts >= this.config.maxReconnectAttempts) {
      console.log('Max reconnect attempts reached');
      this.emit('maxReconnectAttemptsReached');
      return;
    }

    this.isReconnecting = true;
    const attempts = this.connectionState.reconnectAttempts + 1;
    
    this.updateConnectionState({
      ...this.connectionState,
      reconnectAttempts: attempts
    });

    // 指数バックオフで再接続間隔を調整
    const delay = Math.min(
      this.config.reconnectInterval * Math.pow(2, attempts - 1),
      30000 // 最大30秒
    );

    console.log(`Attempting ROSBridge reconnect ${attempts}/${this.config.maxReconnectAttempts} in ${delay}ms`);
    
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, delay);
  }

  /**
   * ハートビート監視を開始
   */
  private startHeartbeat(): void {
    this.clearHeartbeatTimer();
    
    // 最初のheartbeatタイムスタンプを設定
    this.updateConnectionState({
      ...this.connectionState,
      lastHeartbeat: new Date()
    });
    
    this.heartbeatTimer = setInterval(() => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
        console.warn('Cannot send heartbeat: WebSocket is not connected');
        return;
      }

      const now = new Date();
      const lastHeartbeat = this.connectionState.lastHeartbeat;
      
      if (lastHeartbeat) {
        const timeSinceLastHeartbeat = now.getTime() - lastHeartbeat.getTime();
        
        // ハートビートタイムアウトをチェック（2倍の時間で判定）
        if (timeSinceLastHeartbeat > this.config.heartbeatInterval * 2) {
          console.warn('Heartbeat timeout detected');
          this.ws?.close(1001, 'Heartbeat timeout');
          return;
        }
      }

      // ROSBridgeにping/heartbeatメッセージを送信
      // ROSBridgeは任意のメッセージで接続維持できるため、空のsubscribeでも可
      // または、status topicをadvertiseして接続を維持
      try {
        // 軽量なpingとして、現在のsubscribeトピックの再確認を送信
        // これによりサーバーが応答し、接続が維持される
        const pingMsg = {
          op: 'ping'
        };
        
        this.ws.send(JSON.stringify(pingMsg), (error) => {
          if (error) {
            console.error('Failed to send heartbeat ping:', error);
          } else {
            // heartbeat送信成功時にタイムスタンプを更新
            this.updateConnectionState({
              ...this.connectionState,
              lastHeartbeat: new Date()
            });
          }
        });
      } catch (error) {
        console.error('Failed to send heartbeat:', error);
      }
    }, this.config.heartbeatInterval);
  }

  /**
   * CommandManagerのイベントハンドラーを設定
   */
  private setupCommandManagerEvents(): void {
    this.commandManager.on('commandSent', (data) => {
      this.emit('commandSent', data);
    });

    this.commandManager.on('commandFailed', (data) => {
      this.emit('commandFailed', data);
    });

    this.commandManager.on('commandError', (data) => {
      this.emit('commandError', data);
    });

    this.commandManager.on('commandTimeout', (data) => {
      this.emit('commandTimeout', data);
    });

    this.commandManager.on('historyUpdated', (entry) => {
      this.emit('commandHistoryUpdated', entry);
    });

    this.commandManager.on('allCommandsCancelled', () => {
      this.emit('allCommandsCancelled');
    });

    this.commandManager.on('historyCleared', () => {
      this.emit('commandHistoryCleared');
    });
  }

  /**
   * 接続状態を更新
   */
  private updateConnectionState(newState: Partial<ConnectionState>): void {
    this.connectionState = { ...this.connectionState, ...newState };
    this.emit('connectionStateChanged', this.connectionState);
  }

  /**
   * タイマーをクリア
   */
  private clearTimers(): void {
    this.clearReconnectTimer();
    this.clearHeartbeatTimer();
  }

  private clearReconnectTimer(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  private clearHeartbeatTimer(): void {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }
}
