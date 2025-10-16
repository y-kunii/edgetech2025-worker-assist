/**
 * Logger
 * winstonを使用したロガー実装
 */

import winston from 'winston';
import DailyRotateFile from 'winston-daily-rotate-file';
import path from 'path';
import fs from 'fs';
import { LogLevel } from '../types';

export class Logger {
  private logger: winston.Logger;

  constructor(logLevel: LogLevel = 'info', logFile: string = 'logs/server.log') {
    // ログディレクトリの自動作成
    const logDir = path.dirname(logFile);
    if (!fs.existsSync(logDir)) {
      fs.mkdirSync(logDir, { recursive: true });
    }

    // ログフォーマット定義
    const logFormat = winston.format.combine(
      winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
      winston.format.errors({ stack: true }),
      winston.format.printf(({ timestamp, level, message, stack, ...meta }) => {
        let log = `${timestamp} [${level.toUpperCase()}]: ${message}`;
        
        // メタデータがある場合は追加
        if (Object.keys(meta).length > 0) {
          log += ` ${JSON.stringify(meta)}`;
        }
        
        // スタックトレースがある場合は追加
        if (stack) {
          log += `\n${stack}`;
        }
        
        return log;
      })
    );

    // Daily Rotate File トランスポート設定
    const fileRotateTransport = new DailyRotateFile({
      filename: logFile.replace('.log', '-%DATE%.log'),
      datePattern: 'YYYY-MM-DD',
      maxSize: '20m',
      maxFiles: '14d',
      format: logFormat,
      level: logLevel,
      createSymlink: false,
      auditFile: path.join(logDir, '.audit.json'),
    });

    // Loggerインスタンス作成
    this.logger = winston.createLogger({
      level: logLevel,
      format: logFormat,
      transports: [
        // コンソール出力
        new winston.transports.Console({
          format: winston.format.combine(
            winston.format.colorize(),
            logFormat
          ),
        }),
        // ファイル出力（ローテーション付き）
        fileRotateTransport,
      ],
    });
  }

  /**
   * DEBUGレベルのログを記録
   */
  debug(message: string, meta?: object): void {
    this.logger.debug(message, meta);
  }

  /**
   * INFOレベルのログを記録
   */
  info(message: string, meta?: object): void {
    this.logger.info(message, meta);
  }

  /**
   * WARNレベルのログを記録
   */
  warn(message: string, meta?: object): void {
    this.logger.warn(message, meta);
  }

  /**
   * ERRORレベルのログを記録
   */
  error(message: string, error?: Error, meta?: object): void {
    if (error) {
      this.logger.error(message, { ...meta, error: error.message, stack: error.stack });
    } else {
      this.logger.error(message, meta);
    }
  }

  /**
   * Loggerインスタンスを取得（テスト用）
   */
  getLogger(): winston.Logger {
    return this.logger;
  }
}
