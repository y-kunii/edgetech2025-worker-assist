import { ipcMain, IpcMainInvokeEvent } from 'electron';
import { WebSocketService } from '../services/WebSocketService';

let webSocketService: WebSocketService | null = null;

/**
 * WebSocketサービスを設定
 */
export function setWebSocketService(service: WebSocketService): void {
  webSocketService = service;
}

/**
 * WebSocket関連のIPCハンドラーを登録
 */
export function registerWebSocketHandlers(): void {
  // WebSocket接続
  ipcMain.handle('websocket-connect', async (event: IpcMainInvokeEvent) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }
    
    try {
      webSocketService.connect();
      return { success: true };
    } catch (error) {
      console.error('Failed to connect WebSocket:', error);
      return { 
        success: false, 
        error: error instanceof Error ? error.message : 'Unknown error' 
      };
    }
  });

  // WebSocket切断
  ipcMain.handle('websocket-disconnect', async (event: IpcMainInvokeEvent) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }
    
    try {
      webSocketService.disconnect();
      return { success: true };
    } catch (error) {
      console.error('Failed to disconnect WebSocket:', error);
      return { 
        success: false, 
        error: error instanceof Error ? error.message : 'Unknown error' 
      };
    }
  });

  // 接続状態取得
  ipcMain.handle('websocket-get-status', async (event: IpcMainInvokeEvent) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }
    
    return webSocketService.getConnectionStatus();
  });

  // 接続確認
  ipcMain.handle('websocket-is-connected', async (event: IpcMainInvokeEvent) => {
    if (!webSocketService) {
      return false;
    }
    
    return webSocketService.isConnected();
  });

  // ロボットコマンド送信（レガシー）
  ipcMain.handle('websocket-send-robot-command', async (
    event: IpcMainInvokeEvent, 
    command: string, 
    data?: any
  ) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }
    
    try {
      const success = webSocketService.sendRobotCommand(command, data);
      return { success };
    } catch (error) {
      console.error('Failed to send robot command:', error);
      return { 
        success: false, 
        error: error instanceof Error ? error.message : 'Unknown error' 
      };
    }
  });

  // ロボットコマンド管理関連のハンドラー

  // ロボットコマンド送信（新しいAPI）
  ipcMain.handle('robot-command-send', async (
    event: IpcMainInvokeEvent,
    type: 'tool_handover' | 'next_task',
    parameters?: Record<string, any>
  ) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }

    try {
      const response = await webSocketService.getRobotCommandManager().sendCommand(type, parameters);
      return { success: true, response };
    } catch (error) {
      console.error('Failed to send robot command:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  });

  // コマンドリトライ
  ipcMain.handle('robot-command-retry', async (
    event: IpcMainInvokeEvent,
    commandId: string
  ) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }

    try {
      const response = await webSocketService.getRobotCommandManager().retryCommand(commandId);
      return { success: true, response };
    } catch (error) {
      console.error('Failed to retry robot command:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  });

  // コマンドキャンセル
  ipcMain.handle('robot-command-cancel', async (
    event: IpcMainInvokeEvent,
    commandId: string
  ) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }

    try {
      const success = webSocketService.getRobotCommandManager().cancelCommand(commandId);
      return { success };
    } catch (error) {
      console.error('Failed to cancel robot command:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  });

  // コマンドログ取得
  ipcMain.handle('robot-command-get-logs', async (event: IpcMainInvokeEvent) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }

    try {
      const logs = webSocketService.getRobotCommandManager().getCommandLogs();
      return { success: true, logs };
    } catch (error) {
      console.error('Failed to get robot command logs:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  });

  // 特定のコマンドログ取得
  ipcMain.handle('robot-command-get-log', async (
    event: IpcMainInvokeEvent,
    commandId: string
  ) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }

    try {
      const log = webSocketService.getRobotCommandManager().getCommandLog(commandId);
      return { success: true, log };
    } catch (error) {
      console.error('Failed to get robot command log:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  });

  // コマンド統計取得
  ipcMain.handle('robot-command-get-statistics', async (event: IpcMainInvokeEvent) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }

    try {
      const statistics = webSocketService.getRobotCommandManager().getStatistics();
      return { success: true, statistics };
    } catch (error) {
      console.error('Failed to get robot command statistics:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  });

  // コマンド設定更新
  ipcMain.handle('robot-command-update-config', async (
    event: IpcMainInvokeEvent,
    config: any
  ) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }

    try {
      webSocketService.getRobotCommandManager().updateConfig(config);
      return { success: true };
    } catch (error) {
      console.error('Failed to update robot command config:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  });

  // コマンドログクリア
  ipcMain.handle('robot-command-clear-logs', async (event: IpcMainInvokeEvent) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }

    try {
      webSocketService.getRobotCommandManager().clearLogs();
      return { success: true };
    } catch (error) {
      console.error('Failed to clear robot command logs:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  });

  // データストア関連のハンドラー
  
  // 現在の状態取得
  ipcMain.handle('data-store-get-state', async (event: IpcMainInvokeEvent) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }
    
    return webSocketService.getDataProcessor().getCurrentState();
  });

  // 閾値設定更新
  ipcMain.handle('data-store-update-threshold', async (
    event: IpcMainInvokeEvent, 
    settings: any
  ) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }
    
    try {
      const success = webSocketService.getDataProcessor().updateThresholdSettings(settings);
      return { success };
    } catch (error) {
      console.error('Failed to update threshold settings:', error);
      return { 
        success: false, 
        error: error instanceof Error ? error.message : 'Unknown error' 
      };
    }
  });

  // 通知追加
  ipcMain.handle('data-store-add-notification', async (
    event: IpcMainInvokeEvent, 
    notification: any
  ) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }
    
    try {
      webSocketService.getDataProcessor().addNotification(notification);
      return { success: true };
    } catch (error) {
      console.error('Failed to add notification:', error);
      return { 
        success: false, 
        error: error instanceof Error ? error.message : 'Unknown error' 
      };
    }
  });

  // 統計情報リセット
  ipcMain.handle('data-store-reset-statistics', async (event: IpcMainInvokeEvent) => {
    if (!webSocketService) {
      throw new Error('WebSocket service not initialized');
    }
    
    try {
      webSocketService.getDataProcessor().resetStatistics();
      return { success: true };
    } catch (error) {
      console.error('Failed to reset statistics:', error);
      return { 
        success: false, 
        error: error instanceof Error ? error.message : 'Unknown error' 
      };
    }
  });
}

/**
 * WebSocket関連のIPCハンドラーを削除
 */
export function unregisterWebSocketHandlers(): void {
  ipcMain.removeHandler('websocket-connect');
  ipcMain.removeHandler('websocket-disconnect');
  ipcMain.removeHandler('websocket-get-status');
  ipcMain.removeHandler('websocket-is-connected');
  ipcMain.removeHandler('websocket-send-robot-command');
  ipcMain.removeHandler('robot-command-send');
  ipcMain.removeHandler('robot-command-retry');
  ipcMain.removeHandler('robot-command-cancel');
  ipcMain.removeHandler('robot-command-get-logs');
  ipcMain.removeHandler('robot-command-get-log');
  ipcMain.removeHandler('robot-command-get-statistics');
  ipcMain.removeHandler('robot-command-update-config');
  ipcMain.removeHandler('robot-command-clear-logs');
  ipcMain.removeHandler('data-store-get-state');
  ipcMain.removeHandler('data-store-update-threshold');
  ipcMain.removeHandler('data-store-add-notification');
  ipcMain.removeHandler('data-store-reset-statistics');
}