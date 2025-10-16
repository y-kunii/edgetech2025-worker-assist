/**
 * Logger 単体テスト
 */

import { Logger } from '../../src/utils/Logger';
import * as fs from 'fs';
import * as path from 'path';

describe('Logger', () => {
  const testLogDir = path.join(__dirname, '../test-logs');
  const testLogFile = path.join(testLogDir, 'test.log');
  let loggers: Logger[] = [];

  // テスト前にログディレクトリをクリーンアップ
  beforeEach(() => {
    loggers = [];
    if (fs.existsSync(testLogDir)) {
      // ディレクトリ内のファイルを削除
      const files = fs.readdirSync(testLogDir);
      files.forEach(file => {
        try {
          fs.unlinkSync(path.join(testLogDir, file));
        } catch (e) {
          // ファイルが使用中の場合は無視
        }
      });
      try {
        fs.rmdirSync(testLogDir);
      } catch (e) {
        // ディレクトリが使用中の場合は無視
      }
    }
  });

  // テスト後にロガーをクローズしてログディレクトリを削除
  afterEach((done) => {
    // すべてのロガーをクローズ
    loggers.forEach(logger => {
      const winstonLogger = logger.getLogger();
      winstonLogger.close();
    });
    
    // ログファイルへの書き込みが完了するまで待つ
    setTimeout(() => {
      if (fs.existsSync(testLogDir)) {
        const files = fs.readdirSync(testLogDir);
        files.forEach(file => {
          try {
            fs.unlinkSync(path.join(testLogDir, file));
          } catch (e) {
            // ファイルが使用中の場合は無視
          }
        });
        try {
          fs.rmdirSync(testLogDir);
        } catch (e) {
          // ディレクトリが使用中の場合は無視
        }
      }
      done();
    }, 200);
  });

  describe('コンストラクタ', () => {
    test('ログディレクトリが存在しない場合、自動的に作成すること', () => {
      expect(fs.existsSync(testLogDir)).toBe(false);

      const logger = new Logger('info', testLogFile);
      loggers.push(logger);

      expect(fs.existsSync(testLogDir)).toBe(true);
    });

    test('デフォルトパラメータでLoggerを作成できること', () => {
      const logger = new Logger();
      loggers.push(logger);

      expect(logger).toBeInstanceOf(Logger);
      expect(logger.getLogger()).toBeDefined();
    });

    test('カスタムログレベルでLoggerを作成できること', () => {
      const logger = new Logger('debug', testLogFile);
      loggers.push(logger);

      expect(logger.getLogger().level).toBe('debug');
    });
  });

  describe('debug', () => {
    test('DEBUGレベルのログを記録できること', (done) => {
      const logger = new Logger('debug', testLogFile);
      loggers.push(logger);
      const message = 'Debug message';

      logger.debug(message);

      // ログファイルへの書き込みを待つ
      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        expect(logFiles.length).toBeGreaterThan(0);

        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain('DEBUG');
        expect(logContent).toContain(message);
        done();
      }, 100);
    });

    test('メタデータ付きでDEBUGログを記録できること', (done) => {
      const logger = new Logger('debug', testLogFile);
      loggers.push(logger);
      const message = 'Debug with meta';
      const meta = { userId: 123, action: 'test' };

      logger.debug(message, meta);

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain(message);
        expect(logContent).toContain('userId');
        expect(logContent).toContain('123');
        done();
      }, 100);
    });

    test('ログレベルがinfoの場合、DEBUGログは記録されないこと', (done) => {
      const logger = new Logger('info', testLogFile);
      loggers.push(logger);

      logger.debug('This should not be logged');

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        if (logFiles.length > 0) {
          const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
          expect(logContent).not.toContain('This should not be logged');
        }
        done();
      }, 100);
    });
  });

  describe('info', () => {
    test('INFOレベルのログを記録できること', (done) => {
      const logger = new Logger('info', testLogFile);
      loggers.push(logger);
      const message = 'Info message';

      logger.info(message);

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        expect(logFiles.length).toBeGreaterThan(0);

        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain('INFO');
        expect(logContent).toContain(message);
        done();
      }, 100);
    });

    test('メタデータ付きでINFOログを記録できること', (done) => {
      const logger = new Logger('info', testLogFile);
      loggers.push(logger);
      const message = 'Info with meta';
      const meta = { clientType: 'electron', socketId: 'abc123' };

      logger.info(message, meta);

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain(message);
        expect(logContent).toContain('clientType');
        expect(logContent).toContain('electron');
        done();
      }, 100);
    });
  });

  describe('warn', () => {
    test('WARNレベルのログを記録できること', (done) => {
      const logger = new Logger('warn', testLogFile);
      loggers.push(logger);
      const message = 'Warning message';

      logger.warn(message);

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        expect(logFiles.length).toBeGreaterThan(0);

        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain('WARN');
        expect(logContent).toContain(message);
        done();
      }, 100);
    });

    test('メタデータ付きでWARNログを記録できること', (done) => {
      const logger = new Logger('warn', testLogFile);
      loggers.push(logger);
      const message = 'Warn with meta';
      const meta = { reason: 'timeout', duration: 5000 };

      logger.warn(message, meta);

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain(message);
        expect(logContent).toContain('timeout');
        done();
      }, 100);
    });

    test('ログレベルがerrorの場合、WARNログは記録されないこと', (done) => {
      const logger = new Logger('error', testLogFile);
      loggers.push(logger);

      logger.warn('This should not be logged');

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        if (logFiles.length > 0) {
          const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
          expect(logContent).not.toContain('This should not be logged');
        }
        done();
      }, 100);
    });
  });

  describe('error', () => {
    test('ERRORレベルのログを記録できること', (done) => {
      const logger = new Logger('error', testLogFile);
      loggers.push(logger);
      const message = 'Error message';

      logger.error(message);

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        expect(logFiles.length).toBeGreaterThan(0);

        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain('ERROR');
        expect(logContent).toContain(message);
        done();
      }, 100);
    });

    test('Errorオブジェクト付きでERRORログを記録できること', (done) => {
      const logger = new Logger('error', testLogFile);
      loggers.push(logger);
      const message = 'Error with exception';
      const error = new Error('Test error');

      logger.error(message, error);

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain('ERROR');
        expect(logContent).toContain(message);
        expect(logContent).toContain('Test error');
        // スタックトレースが含まれていることを確認（at キーワードで判定）
        expect(logContent).toContain('at ');
        done();
      }, 100);
    });

    test('Errorオブジェクトとメタデータ付きでERRORログを記録できること', (done) => {
      const logger = new Logger('error', testLogFile);
      loggers.push(logger);
      const message = 'Error with exception and meta';
      const error = new Error('Test error with meta');
      const meta = { clientId: 'client123', operation: 'send_data' };

      logger.error(message, error, meta);

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain(message);
        expect(logContent).toContain('Test error with meta');
        expect(logContent).toContain('clientId');
        expect(logContent).toContain('client123');
        done();
      }, 100);
    });

    test('メタデータのみでERRORログを記録できること', (done) => {
      const logger = new Logger('error', testLogFile);
      loggers.push(logger);
      const message = 'Error with meta only';
      const meta = { code: 'SENSOR_NOT_CONNECTED', timestamp: Date.now() };

      logger.error(message, undefined, meta);

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain(message);
        expect(logContent).toContain('SENSOR_NOT_CONNECTED');
        done();
      }, 100);
    });
  });

  describe('ログファイル出力', () => {
    test('複数のログレベルのメッセージを記録できること', (done) => {
      const logger = new Logger('debug', testLogFile);
      loggers.push(logger);

      logger.debug('Debug message');
      logger.info('Info message');
      logger.warn('Warn message');
      logger.error('Error message');

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        expect(logFiles.length).toBeGreaterThan(0);

        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        expect(logContent).toContain('DEBUG');
        expect(logContent).toContain('INFO');
        expect(logContent).toContain('WARN');
        expect(logContent).toContain('ERROR');
        done();
      }, 100);
    });

    test('タイムスタンプが含まれること', (done) => {
      const logger = new Logger('info', testLogFile);
      loggers.push(logger);

      logger.info('Message with timestamp');

      setTimeout(() => {
        const logFiles = fs.readdirSync(testLogDir).filter(f => f.endsWith('.log'));
        const logContent = fs.readFileSync(path.join(testLogDir, logFiles[0]), 'utf-8');
        
        // タイムスタンプフォーマット: YYYY-MM-DD HH:mm:ss
        expect(logContent).toMatch(/\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}/);
        done();
      }, 100);
    });
  });

  describe('getLogger', () => {
    test('winstonのLoggerインスタンスを取得できること', () => {
      const logger = new Logger('info', testLogFile);
      loggers.push(logger);
      const winstonLogger = logger.getLogger();

      expect(winstonLogger).toBeDefined();
      expect(typeof winstonLogger.info).toBe('function');
      expect(typeof winstonLogger.error).toBe('function');
    });
  });
});
