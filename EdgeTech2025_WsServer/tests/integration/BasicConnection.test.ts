/**
 * 統合テスト: 基本接続テスト
 * 要件: 1.2, 3.1, 3.2, 3.3, 3.4, 3.5
 */

import { io as ioClient } from 'socket.io-client';
import { MainServer } from '../../src/server/MainServer';
import { ServerConfig } from '../../src/types';

describe('Integration Test: Basic Connection', () => {
  let server: MainServer;
  let serverConfig: ServerConfig;
  const TEST_PORT = 3002;

  beforeAll(async () => {
    // テスト用サーバー設定
    serverConfig = {
      port: TEST_PORT,
      cors_origin: '*',
      heartbeat_interval: 30000,
      connection_timeout: 60000,
      log_level: 'error', // テスト中はエラーのみ
      log_file: 'logs/test.log',
    };

    // サーバーを起動
    server = new MainServer(serverConfig);
    await server.start();
  });

  afterAll(async () => {
    // サーバーを停止
    await server.stop();
  });

  describe('要件 1.2, 3.1: クライアント接続の受け入れ', () => {
    test('Electronクライアントが正常に接続できること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        expect(client.connected).toBe(true);
        client.disconnect();
        done();
      });

      client.on('connect_error', (error) => {
        done(error);
      });
    });

    test('センサークライアントが正常に接続できること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        expect(client.connected).toBe(true);
        client.disconnect();
        done();
      });

      client.on('connect_error', (error) => {
        done(error);
      });
    });

    test('ロボットクライアントが正常に接続できること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        expect(client.connected).toBe(true);
        client.disconnect();
        done();
      });

      client.on('connect_error', (error) => {
        done(error);
      });
    });

    test('複数のクライアントが同時に接続できること', (done) => {
      const client1 = ioClient(`http://localhost:${TEST_PORT}`);
      const client2 = ioClient(`http://localhost:${TEST_PORT}`);
      const client3 = ioClient(`http://localhost:${TEST_PORT}`);

      let connectedCount = 0;

      const checkAllConnected = () => {
        connectedCount++;
        if (connectedCount === 3) {
          expect(client1.connected).toBe(true);
          expect(client2.connected).toBe(true);
          expect(client3.connected).toBe(true);
          
          client1.disconnect();
          client2.disconnect();
          client3.disconnect();
          done();
        }
      };

      client1.on('connect', checkAllConnected);
      client2.on('connect', checkAllConnected);
      client3.on('connect', checkAllConnected);
    });
  });

  describe('要件 3.2, 3.3, 3.4, 3.5: クライアントタイプの識別と登録', () => {
    test('Electronクライアントとして登録できること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('registered', (data) => {
        expect(data.client_type).toBe('electron');
        expect(data.timestamp).toBeDefined();
        client.disconnect();
        done();
      });
    });

    test('センサークライアントとして登録できること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'sensor' });
      });

      client.on('registered', (data) => {
        expect(data.client_type).toBe('sensor');
        expect(data.timestamp).toBeDefined();
        client.disconnect();
        done();
      });
    });

    test('ロボットクライアントとして登録できること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'robot' });
      });

      client.on('registered', (data) => {
        expect(data.client_type).toBe('robot');
        expect(data.timestamp).toBeDefined();
        client.disconnect();
        done();
      });
    });

    test('クライアントタイプ未指定の場合はunknownとして扱われること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', {});
      });

      client.on('registered', (data) => {
        expect(data.client_type).toBe('unknown');
        client.disconnect();
        done();
      });
    });

    test('不正なクライアントタイプの場合はunknownとして扱われること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'invalid_type' });
      });

      client.on('registered', (data) => {
        expect(data.client_type).toBe('unknown');
        client.disconnect();
        done();
      });
    });
  });

  describe('要件 3.2, 3.3, 3.4: 複数クライアントタイプの同時接続', () => {
    test('Electron、センサー、ロボットクライアントが同時に接続できること', (done) => {
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);
      const sensorClient = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);

      let registeredCount = 0;
      const registeredTypes: string[] = [];

      const checkAllRegistered = (clientType: string) => {
        registeredCount++;
        registeredTypes.push(clientType);

        if (registeredCount === 3) {
          expect(registeredTypes).toContain('electron');
          expect(registeredTypes).toContain('sensor');
          expect(registeredTypes).toContain('robot');

          electronClient.disconnect();
          sensorClient.disconnect();
          robotClient.disconnect();
          done();
        }
      };

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });

      electronClient.on('registered', (data) => {
        checkAllRegistered(data.client_type);
      });

      sensorClient.on('connect', () => {
        sensorClient.emit('register_client', { client_type: 'sensor' });
      });

      sensorClient.on('registered', (data) => {
        checkAllRegistered(data.client_type);
      });

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });

      robotClient.on('registered', (data) => {
        checkAllRegistered(data.client_type);
      });
    });

    test('複数のElectronクライアントが同時に接続できること', (done) => {
      const electron1 = ioClient(`http://localhost:${TEST_PORT}`);
      const electron2 = ioClient(`http://localhost:${TEST_PORT}`);
      const electron3 = ioClient(`http://localhost:${TEST_PORT}`);

      let registeredCount = 0;

      const checkAllRegistered = () => {
        registeredCount++;
        if (registeredCount === 3) {
          electron1.disconnect();
          electron2.disconnect();
          electron3.disconnect();
          done();
        }
      };

      electron1.on('connect', () => {
        electron1.emit('register_client', { client_type: 'electron' });
      });
      electron1.on('registered', checkAllRegistered);

      electron2.on('connect', () => {
        electron2.emit('register_client', { client_type: 'electron' });
      });
      electron2.on('registered', checkAllRegistered);

      electron3.on('connect', () => {
        electron3.emit('register_client', { client_type: 'electron' });
      });
      electron3.on('registered', checkAllRegistered);
    });
  });

  describe('要件 1.2: 接続確認メッセージの送信', () => {
    test('接続後に登録確認メッセージを受信すること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('registered', (data) => {
        expect(data).toHaveProperty('client_type');
        expect(data).toHaveProperty('timestamp');
        expect(typeof data.timestamp).toBe('string');
        client.disconnect();
        done();
      });
    });
  });

  describe('要件 1.3: クライアント切断とリソース解放', () => {
    test('クライアントが切断した後、再接続できること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('registered', () => {
        // 一度切断
        client.disconnect();

        // 少し待ってから再接続
        setTimeout(() => {
          const client2 = ioClient(`http://localhost:${TEST_PORT}`);

          client2.on('connect', () => {
            expect(client2.connected).toBe(true);
            client2.disconnect();
            done();
          });
        }, 100);
      });
    });

    test('複数クライアントが順次接続・切断できること', (done) => {
      const client1 = ioClient(`http://localhost:${TEST_PORT}`);

      client1.on('connect', () => {
        client1.emit('register_client', { client_type: 'electron' });
      });

      client1.on('registered', () => {
        client1.disconnect();

        setTimeout(() => {
          const client2 = ioClient(`http://localhost:${TEST_PORT}`);

          client2.on('connect', () => {
            client2.emit('register_client', { client_type: 'sensor' });
          });

          client2.on('registered', () => {
            client2.disconnect();

            setTimeout(() => {
              const client3 = ioClient(`http://localhost:${TEST_PORT}`);

              client3.on('connect', () => {
                expect(client3.connected).toBe(true);
                client3.disconnect();
                done();
              });
            }, 100);
          });
        }, 100);
      });
    });
  });
});
