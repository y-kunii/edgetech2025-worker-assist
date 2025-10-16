/**
 * ConfigManager 単体テスト
 */

import { ConfigManager } from '../../src/utils/ConfigManager';
import { ServerConfig } from '../../src/types';
import * as fs from 'fs';
import * as path from 'path';

describe('ConfigManager', () => {
  const testConfigDir = path.join(__dirname, '../test-configs');
  const validConfigPath = path.join(testConfigDir, 'valid-config.json');
  const invalidJsonPath = path.join(testConfigDir, 'invalid-json.json');
  const invalidConfigPath = path.join(testConfigDir, 'invalid-config.json');
  const nonExistentPath = path.join(testConfigDir, 'non-existent.json');

  // テスト前にテスト用設定ファイルを作成
  beforeAll(() => {
    if (!fs.existsSync(testConfigDir)) {
      fs.mkdirSync(testConfigDir, { recursive: true });
    }

    // 正常な設定ファイル
    const validConfig: ServerConfig = {
      port: 4000,
      cors_origin: "http://localhost:3000",
      heartbeat_interval: 20000,
      connection_timeout: 40000,
      log_level: "debug",
      log_file: "logs/test.log"
    };
    fs.writeFileSync(validConfigPath, JSON.stringify(validConfig, null, 2));

    // 不正なJSON形式
    fs.writeFileSync(invalidJsonPath, '{ invalid json }');

    // 不正な設定値
    const invalidConfig = {
      port: 99999,  // 範囲外
      cors_origin: 123,  // 型が不正
      log_level: "invalid"  // 不正な値
    };
    fs.writeFileSync(invalidConfigPath, JSON.stringify(invalidConfig, null, 2));
  });

  // テスト後にテスト用ファイルを削除
  afterAll(() => {
    if (fs.existsSync(validConfigPath)) fs.unlinkSync(validConfigPath);
    if (fs.existsSync(invalidJsonPath)) fs.unlinkSync(invalidJsonPath);
    if (fs.existsSync(invalidConfigPath)) fs.unlinkSync(invalidConfigPath);
    if (fs.existsSync(testConfigDir)) fs.rmdirSync(testConfigDir);
  });

  describe('getDefault', () => {
    test('デフォルト設定を返すこと', () => {
      const config = ConfigManager.getDefault();

      expect(config).toEqual({
        port: 3001,
        cors_origin: "*",
        heartbeat_interval: 30000,
        connection_timeout: 60000,
        log_level: "info",
        log_file: "logs/server.log"
      });
    });

    test('デフォルト設定のコピーを返すこと（元のオブジェクトを変更しない）', () => {
      const config1 = ConfigManager.getDefault();
      const config2 = ConfigManager.getDefault();

      config1.port = 9999;

      expect(config2.port).toBe(3001);
    });
  });

  describe('load', () => {
    test('正常な設定ファイルを読み込めること', () => {
      const config = ConfigManager.load(validConfigPath);

      expect(config.port).toBe(4000);
      expect(config.cors_origin).toBe("http://localhost:3000");
      expect(config.heartbeat_interval).toBe(20000);
      expect(config.connection_timeout).toBe(40000);
      expect(config.log_level).toBe("debug");
      expect(config.log_file).toBe("logs/test.log");
    });

    test('設定ファイルが存在しない場合、デフォルト設定を返すこと', () => {
      const consoleWarnSpy = jest.spyOn(console, 'warn').mockImplementation();

      const config = ConfigManager.load(nonExistentPath);

      expect(config).toEqual(ConfigManager.getDefault());
      expect(consoleWarnSpy).toHaveBeenCalledWith(
        expect.stringContaining('設定ファイルが見つかりません')
      );

      consoleWarnSpy.mockRestore();
    });

    test('不正なJSON形式の場合、デフォルト設定を返すこと', () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();
      const consoleWarnSpy = jest.spyOn(console, 'warn').mockImplementation();

      const config = ConfigManager.load(invalidJsonPath);

      expect(config).toEqual(ConfigManager.getDefault());
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        expect.stringContaining('JSON形式が不正です'),
        expect.any(String)
      );

      consoleErrorSpy.mockRestore();
      consoleWarnSpy.mockRestore();
    });

    test('不正な設定値の場合、デフォルト設定を返すこと', () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();
      const consoleWarnSpy = jest.spyOn(console, 'warn').mockImplementation();

      const config = ConfigManager.load(invalidConfigPath);

      expect(config).toEqual(ConfigManager.getDefault());

      consoleErrorSpy.mockRestore();
      consoleWarnSpy.mockRestore();
    });

    test('部分的な設定ファイルの場合、デフォルト値で補完すること', () => {
      const partialConfigPath = path.join(testConfigDir, 'partial-config.json');
      const partialConfig = {
        port: 5000,
        log_level: "warn"
      };
      fs.writeFileSync(partialConfigPath, JSON.stringify(partialConfig, null, 2));

      const config = ConfigManager.load(partialConfigPath);

      expect(config.port).toBe(5000);
      expect(config.log_level).toBe("warn");
      expect(config.cors_origin).toBe("*");  // デフォルト値
      expect(config.heartbeat_interval).toBe(30000);  // デフォルト値

      fs.unlinkSync(partialConfigPath);
    });
  });

  describe('validate', () => {
    test('正常な設定を検証できること', () => {
      const validConfig: ServerConfig = {
        port: 3001,
        cors_origin: "*",
        heartbeat_interval: 30000,
        connection_timeout: 60000,
        log_level: "info",
        log_file: "logs/server.log"
      };

      expect(ConfigManager.validate(validConfig)).toBe(true);
    });

    test('nullまたはundefinedの場合、falseを返すこと', () => {
      expect(ConfigManager.validate(null)).toBe(false);
      expect(ConfigManager.validate(undefined)).toBe(false);
    });

    test('オブジェクトでない場合、falseを返すこと', () => {
      expect(ConfigManager.validate("string")).toBe(false);
      expect(ConfigManager.validate(123)).toBe(false);
      expect(ConfigManager.validate([])).toBe(false);
    });

    test('portが範囲外の場合、falseを返すこと', () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      expect(ConfigManager.validate({ port: 0 })).toBe(false);
      expect(ConfigManager.validate({ port: 99999 })).toBe(false);
      expect(ConfigManager.validate({ port: -1 })).toBe(false);

      consoleErrorSpy.mockRestore();
    });

    test('portが数値でない場合、falseを返すこと', () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      expect(ConfigManager.validate({ port: "3001" })).toBe(false);

      consoleErrorSpy.mockRestore();
    });

    test('cors_originが文字列でない場合、falseを返すこと', () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      expect(ConfigManager.validate({ cors_origin: 123 })).toBe(false);

      consoleErrorSpy.mockRestore();
    });

    test('heartbeat_intervalが負の数の場合、falseを返すこと', () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      expect(ConfigManager.validate({ heartbeat_interval: -1 })).toBe(false);

      consoleErrorSpy.mockRestore();
    });

    test('connection_timeoutが負の数の場合、falseを返すこと', () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      expect(ConfigManager.validate({ connection_timeout: -1 })).toBe(false);

      consoleErrorSpy.mockRestore();
    });

    test('log_levelが不正な値の場合、falseを返すこと', () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      expect(ConfigManager.validate({ log_level: "invalid" })).toBe(false);
      expect(ConfigManager.validate({ log_level: "INFO" })).toBe(false);

      consoleErrorSpy.mockRestore();
    });

    test('log_levelが正常な値の場合、trueを返すこと', () => {
      expect(ConfigManager.validate({ log_level: "debug" })).toBe(true);
      expect(ConfigManager.validate({ log_level: "info" })).toBe(true);
      expect(ConfigManager.validate({ log_level: "warn" })).toBe(true);
      expect(ConfigManager.validate({ log_level: "error" })).toBe(true);
    });

    test('log_fileが文字列でない場合、falseを返すこと', () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      expect(ConfigManager.validate({ log_file: 123 })).toBe(false);

      consoleErrorSpy.mockRestore();
    });

    test('空のオブジェクトの場合、trueを返すこと（すべてオプショナル）', () => {
      expect(ConfigManager.validate({})).toBe(true);
    });
  });
});
