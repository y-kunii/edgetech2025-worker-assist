/**
 * エントリーポイント
 * Raspberry Pi WebSocket Server
 */

import { ConfigManager } from './utils/ConfigManager';
import { MainServer } from './server/MainServer';

/**
 * メイン関数
 */
async function main(): Promise<void> {
  let server: MainServer | null = null;

  try {
    // 設定の読み込み
    console.log('Loading configuration...');
    const config = ConfigManager.load();

    // MainServerのインスタンス化
    console.log('Initializing server...');
    server = new MainServer(config);

    // グローバルエラーハンドラーをセットアップ
    server.setupGlobalErrorHandlers();

    // シグナルハンドリングをセットアップ
    setupSignalHandlers(server);

    // サーバーを起動
    console.log('Starting server...');
    await server.start();

    console.log('='.repeat(50));
    console.log('Raspberry Pi WebSocket Server is running!');
    console.log(`Port: ${config.port}`);
    console.log(`WebSocket URL: ws://localhost:${config.port}`);
    console.log(`Health Check: http://localhost:${config.port}/health`);
    console.log('='.repeat(50));

  } catch (error) {
    console.error('Failed to start server:', error);
    process.exit(1);
  }
}

/**
 * シグナルハンドラーをセットアップ
 * SIGTERM/SIGINTシグナルを受信した際に優雅なシャットダウンを実行
 */
function setupSignalHandlers(server: MainServer): void {
  let isShuttingDown = false;

  const gracefulShutdown = async (signal: string): Promise<void> => {
    // 既にシャットダウン中の場合は無視
    if (isShuttingDown) {
      console.log(`${signal} received again, but shutdown is already in progress...`);
      return;
    }

    isShuttingDown = true;
    console.log('');
    console.log('='.repeat(50));
    console.log(`${signal} received. Starting graceful shutdown...`);
    console.log('='.repeat(50));

    try {
      // サーバーを停止
      await server.stop();
      
      console.log('Graceful shutdown completed successfully');
      process.exit(0);
    } catch (error) {
      console.error('Error during graceful shutdown:', error);
      process.exit(1);
    }
  };

  // SIGTERMハンドラー
  process.on('SIGTERM', () => {
    gracefulShutdown('SIGTERM');
  });

  // SIGINTハンドラー（Ctrl+C）
  process.on('SIGINT', () => {
    gracefulShutdown('SIGINT');
  });

  console.log('Signal handlers (SIGTERM, SIGINT) registered');
}

// メイン関数を実行
main();
