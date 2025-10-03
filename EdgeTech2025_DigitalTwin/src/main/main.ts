import { app, BrowserWindow } from 'electron';
import * as path from 'path';
import { WebSocketService } from './services/WebSocketService';
import { registerWebSocketHandlers, setWebSocketService, unregisterWebSocketHandlers } from './ipc/websocketHandlers';

let mainWindow: BrowserWindow;
let webSocketService: WebSocketService;

function createWindow(): void {
  // Create the browser window
  mainWindow = new BrowserWindow({
    height: 1080,
    width: 1920,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
    },
  });

  // Load the app
  mainWindow.loadFile(path.join(__dirname, 'index.html'));

  // Open DevTools in development
  if (process.env.NODE_ENV === 'development') {
    mainWindow.webContents.openDevTools();
  }

  // Initialize WebSocket service
  initializeWebSocketService();
}

function initializeWebSocketService(): void {
  // WebSocket接続設定（環境変数またはデフォルト値を使用）
  const wsConfig = {
    url: process.env.WEBSOCKET_URL || 'http://localhost:3001',
    reconnectionAttempts: 10,
    reconnectionDelay: 1000,
    timeout: 5000
  };

  // WebSocketサービスを初期化
  webSocketService = new WebSocketService(wsConfig);
  webSocketService.setMainWindow(mainWindow);
  
  // IPCハンドラーを登録
  registerWebSocketHandlers();
  setWebSocketService(webSocketService);

  console.log('WebSocket service initialized with config:', wsConfig);
}

// This method will be called when Electron has finished initialization
app.whenReady().then(createWindow);

// Quit when all windows are closed, except on macOS
app.on('window-all-closed', () => {
  // Clean up WebSocket service
  if (webSocketService) {
    webSocketService.destroy();
  }
  unregisterWebSocketHandlers();
  
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

app.on('activate', () => {
  // On macOS it's common to re-create a window in the app when the
  // dock icon is clicked and there are no other windows open
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});