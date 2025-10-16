/**
 * ConfigManager
 * サーバー設定の管理を担当するクラス
 */

import { ServerConfig } from '../types';
import * as fs from 'fs';
import * as path from 'path';

export class ConfigManager {
  /**
   * デフォルト設定
   */
  private static readonly DEFAULT_CONFIG: ServerConfig = {
    port: 3001,
    cors_origin: "*",
    heartbeat_interval: 30000,      // 30秒
    connection_timeout: 60000,      // 60秒
    log_level: "info",
    log_file: "logs/server.log"
  };

  /**
   * デフォルト設定を取得
   */
  static getDefault(): ServerConfig {
    return { ...this.DEFAULT_CONFIG };
  }

  /**
   * 設定ファイルを読み込む
   * @param configPath 設定ファイルのパス（省略時: config/config.json）
   * @returns サーバー設定
   */
  static load(configPath?: string): ServerConfig {
    const filePath = configPath || path.join(process.cwd(), 'config', 'config.json');

    try {
      // ファイルの存在確認
      if (!fs.existsSync(filePath)) {
        console.warn(`[ConfigManager] 設定ファイルが見つかりません: ${filePath}`);
        console.warn('[ConfigManager] デフォルト設定を使用します');
        return this.getDefault();
      }

      // ファイル読み込み
      const fileContent = fs.readFileSync(filePath, 'utf-8');
      const loadedConfig = JSON.parse(fileContent);

      // 設定の検証
      if (!this.validate(loadedConfig)) {
        console.error('[ConfigManager] 設定ファイルの形式が不正です');
        console.warn('[ConfigManager] デフォルト設定を使用します');
        return this.getDefault();
      }

      // デフォルト設定とマージ（不足している項目をデフォルト値で補完）
      const config: ServerConfig = {
        ...this.DEFAULT_CONFIG,
        ...loadedConfig
      };

      console.info('[ConfigManager] 設定ファイルを読み込みました:', filePath);
      return config;

    } catch (error) {
      if (error instanceof SyntaxError) {
        console.error('[ConfigManager] 設定ファイルのJSON形式が不正です:', error.message);
      } else {
        console.error('[ConfigManager] 設定ファイルの読み込みに失敗しました:', error);
      }
      console.warn('[ConfigManager] デフォルト設定を使用します');
      return this.getDefault();
    }
  }

  /**
   * 設定の検証
   * @param config 検証する設定オブジェクト
   * @returns 検証結果（true: 正常, false: 不正）
   */
  static validate(config: any): boolean {
    if (!config || typeof config !== 'object' || Array.isArray(config)) {
      return false;
    }

    // 各フィールドの型チェック
    if (config.port !== undefined && (typeof config.port !== 'number' || config.port < 1 || config.port > 65535)) {
      console.error('[ConfigManager] port は 1-65535 の数値である必要があります');
      return false;
    }

    if (config.cors_origin !== undefined && typeof config.cors_origin !== 'string') {
      console.error('[ConfigManager] cors_origin は文字列である必要があります');
      return false;
    }

    if (config.heartbeat_interval !== undefined && (typeof config.heartbeat_interval !== 'number' || config.heartbeat_interval < 0)) {
      console.error('[ConfigManager] heartbeat_interval は 0 以上の数値である必要があります');
      return false;
    }

    if (config.connection_timeout !== undefined && (typeof config.connection_timeout !== 'number' || config.connection_timeout < 0)) {
      console.error('[ConfigManager] connection_timeout は 0 以上の数値である必要があります');
      return false;
    }

    if (config.log_level !== undefined) {
      const validLogLevels = ['debug', 'info', 'warn', 'error'];
      if (!validLogLevels.includes(config.log_level)) {
        console.error('[ConfigManager] log_level は debug, info, warn, error のいずれかである必要があります');
        return false;
      }
    }

    if (config.log_file !== undefined && typeof config.log_file !== 'string') {
      console.error('[ConfigManager] log_file は文字列である必要があります');
      return false;
    }

    return true;
  }
}
