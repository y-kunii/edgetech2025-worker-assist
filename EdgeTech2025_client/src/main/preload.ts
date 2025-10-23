import { contextBridge, ipcRenderer, IpcRendererEvent } from 'electron';
import { AppSettings, StatusData, CommandData, WorkHistoryEntry, WorkStatistics } from '../shared/types';
import { WebSocketClientConfig, ConnectionState } from './websocket-client';
import { CommandHistoryEntry, CommandSendResult } from './command-manager';

// レンダラープロセスで使用可能なAPIを定義
const electronAPI = {
  // アプリケーション情報
  getAppVersion: (): Promise<string> => ipcRenderer.invoke('app:getVersion'),
  getAppName: (): Promise<string> => ipcRenderer.invoke('app:getName'),
  isAppInitialized: (): Promise<boolean> => ipcRenderer.invoke('app:isInitialized'),
  performHealthCheck: (): Promise<{ success: boolean; errors: string[]; warnings: string[] }> => 
    ipcRenderer.invoke('app:performHealthCheck'),
  reinitializeApp: (): Promise<{ success: boolean; errors: string[]; warnings: string[] }> => 
    ipcRenderer.invoke('app:reinitialize'),
  shutdownApp: (): Promise<boolean> => ipcRenderer.invoke('app:shutdown'),
  
  // 設定管理
  getSettings: (): Promise<AppSettings> => ipcRenderer.invoke('settings:get'),
  saveSettings: (settings: Partial<AppSettings>): Promise<boolean> => ipcRenderer.invoke('settings:save', settings),
  resetSettings: (): Promise<boolean> => ipcRenderer.invoke('settings:reset'),
  validateSettings: (settings: AppSettings): Promise<{ isValid: boolean; errors: string[] }> => 
    ipcRenderer.invoke('settings:validate', settings),
  getSetting: (key: keyof AppSettings): Promise<any> => ipcRenderer.invoke('settings:getSetting', key),
  updateSetting: (key: keyof AppSettings, value: any): Promise<boolean> => 
    ipcRenderer.invoke('settings:updateSetting', key, value),
  getDefaultSettings: (): Promise<AppSettings> => ipcRenderer.invoke('settings:getDefaults'),
  settingsFileExists: (): Promise<boolean> => ipcRenderer.invoke('settings:fileExists'),
  getSettingsPath: (): Promise<string> => ipcRenderer.invoke('settings:getPath'),
  
  // WebSocket通信
  connectWebSocket: (config: WebSocketClientConfig): Promise<{ success: boolean; error?: string }> => 
    ipcRenderer.invoke('websocket:connect', config),
  disconnectWebSocket: (): Promise<{ success: boolean; error?: string }> => 
    ipcRenderer.invoke('websocket:disconnect'),
  sendCommand: (command: CommandData): Promise<CommandSendResult> => 
    ipcRenderer.invoke('websocket:sendCommand', command),
  createAndSendCommand: (commandType: 'tool_handover' | 'tool_collection' | 'wait'): Promise<CommandSendResult> => 
    ipcRenderer.invoke('websocket:createAndSendCommand', commandType),
  getConnectionState: (): Promise<ConnectionState> => 
    ipcRenderer.invoke('websocket:getConnectionState'),
  getLastStatusData: (): Promise<StatusData | null> => 
    ipcRenderer.invoke('websocket:getLastStatusData'),

  // コマンド履歴管理
  getCommandHistory: (limit?: number): Promise<CommandHistoryEntry[]> => 
    ipcRenderer.invoke('command:getHistory', limit),
  getCommandStatistics: (timeRange?: { start: Date; end: Date }): Promise<any> => 
    ipcRenderer.invoke('command:getStatistics', timeRange),
  getPendingCommandsCount: (): Promise<number> => 
    ipcRenderer.invoke('command:getPendingCount'),
  clearCommandHistory: (): Promise<boolean> => 
    ipcRenderer.invoke('command:clearHistory'),
  
  // ファイル監視
  startImageWatch: (path: string): Promise<boolean> => ipcRenderer.invoke('filewatch:start', path),
  stopImageWatch: (): Promise<void> => ipcRenderer.invoke('filewatch:stop'),
  getLatestImage: (): Promise<string | null> => ipcRenderer.invoke('filewatch:getLatestImage'),
  isImageWatchActive: (): Promise<boolean> => ipcRenderer.invoke('filewatch:isActive'),
  updateImageWatchConfig: (config: any): Promise<boolean> => ipcRenderer.invoke('filewatch:updateConfig', config),
  
  // データベース操作（後で実装）
  saveStatusData: (data: StatusData): Promise<boolean> => ipcRenderer.invoke('db:saveStatus', data),
  getWorkHistory: (timeRange: string): Promise<WorkHistoryEntry[]> => ipcRenderer.invoke('db:getWorkHistory', timeRange),
  getWorkStatistics: (): Promise<WorkStatistics> => ipcRenderer.invoke('db:getWorkStatistics'),
  
  // イベントリスナー
  onWebSocketConnected: (callback: () => void) => {
    ipcRenderer.on('websocket:connected', (_event: IpcRendererEvent) => callback());
  },
  onWebSocketDisconnected: (callback: (info: any) => void) => {
    ipcRenderer.on('websocket:disconnected', (_event: IpcRendererEvent, info: any) => callback(info));
  },
  onWebSocketStatusData: (callback: (data: StatusData) => void) => {
    ipcRenderer.on('websocket:statusData', (_event: IpcRendererEvent, data: StatusData) => callback(data));
  },
  onWebSocketStatusDataChanged: (callback: (changeData: any) => void) => {
    ipcRenderer.on('websocket:statusDataChanged', (_event: IpcRendererEvent, changeData: any) => callback(changeData));
  },
  onWebSocketWorkTaskChanged: (callback: (taskChangeData: any) => void) => {
    ipcRenderer.on('websocket:workTaskChanged', (_event: IpcRendererEvent, taskChangeData: any) => callback(taskChangeData));
  },
  onWebSocketDataValidationError: (callback: (errorData: any) => void) => {
    ipcRenderer.on('websocket:dataValidationError', (_event: IpcRendererEvent, errorData: any) => callback(errorData));
  },

  // コマンド関連のイベントリスナー
  onCommandSent: (callback: (data: any) => void) => {
    ipcRenderer.on('command:sent', (_event: IpcRendererEvent, data: any) => callback(data));
  },
  onCommandFailed: (callback: (data: any) => void) => {
    ipcRenderer.on('command:failed', (_event: IpcRendererEvent, data: any) => callback(data));
  },
  onCommandError: (callback: (data: any) => void) => {
    ipcRenderer.on('command:error', (_event: IpcRendererEvent, data: any) => callback(data));
  },
  onCommandTimeout: (callback: (data: any) => void) => {
    ipcRenderer.on('command:timeout', (_event: IpcRendererEvent, data: any) => callback(data));
  },
  onCommandHistoryUpdated: (callback: (entry: CommandHistoryEntry) => void) => {
    ipcRenderer.on('command:historyUpdated', (_event: IpcRendererEvent, entry: CommandHistoryEntry) => callback(entry));
  },
  onAllCommandsCancelled: (callback: () => void) => {
    ipcRenderer.on('command:allCancelled', (_event: IpcRendererEvent) => callback());
  },
  onCommandHistoryCleared: (callback: () => void) => {
    ipcRenderer.on('command:historyCleared', (_event: IpcRendererEvent) => callback());
  },
  onWebSocketError: (callback: (error: string) => void) => {
    ipcRenderer.on('websocket:error', (_event: IpcRendererEvent, error: string) => callback(error));
  },
  onWebSocketConnectionStateChanged: (callback: (state: ConnectionState) => void) => {
    ipcRenderer.on('websocket:connectionStateChanged', (_event: IpcRendererEvent, state: ConnectionState) => callback(state));
  },
  onWebSocketMaxReconnectAttemptsReached: (callback: () => void) => {
    ipcRenderer.on('websocket:maxReconnectAttemptsReached', (_event: IpcRendererEvent) => callback());
  },
  onImageUpdate: (callback: (imagePath: string) => void) => {
    ipcRenderer.on('filewatch:imageUpdate', (_event: IpcRendererEvent, imagePath: string) => callback(imagePath));
  },
  onFileWatcherStarted: (callback: (data: any) => void) => {
    ipcRenderer.on('filewatch:started', (_event: IpcRendererEvent, data: any) => callback(data));
  },
  onFileWatcherStopped: (callback: () => void) => {
    ipcRenderer.on('filewatch:stopped', (_event: IpcRendererEvent) => callback());
  },
  onFileWatcherError: (callback: (error: string) => void) => {
    ipcRenderer.on('filewatch:error', (_event: IpcRendererEvent, error: string) => callback(error));
  },
  onFileWatcherReady: (callback: () => void) => {
    ipcRenderer.on('filewatch:ready', (_event: IpcRendererEvent) => callback());
  },
  
  // イベントリスナーの削除
  removeAllListeners: (channel: string) => {
    ipcRenderer.removeAllListeners(channel);
  }
};

// contextBridgeを使用してAPIを安全に公開
contextBridge.exposeInMainWorld('electronAPI', electronAPI);

// TypeScript用の型定義
declare global {
  interface Window {
    electronAPI: typeof electronAPI;
  }
}