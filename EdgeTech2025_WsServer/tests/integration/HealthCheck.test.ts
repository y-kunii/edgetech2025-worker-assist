/**
 * 統合テスト: ヘルスチェックテスト
 * 要件: 5.1, 5.2, 5.3, 5.4
 */

import { io as ioClient } from 'socket.io-client';
import { MainServer } from '../../src/server/MainServer';
import { ServerConfig, HealthStatus } from '../../src/types';

describe('Integration Test: Health Check', () => {
  let server: MainServer;
  let serverConfig: ServerConfig;
  const TEST_PORT = 3006;

  beforeAll(async () => {
    serverConfig = {
      port: TEST_PORT,
      cors_origin: '*',
      heartbeat_interval: 30000,
      connection_timeout: 60000,
      log_level: 'error',
      log_file: 'logs/test.log',
    };

    server = new MainServer(serverConfig);
    await server.start();
  });

  afterAll(async () => {
    await server.stop();
  });

  describe('要件 5.1: ヘルスチェックエンドポイント', () => {
    test('/healthエンドポイントにGETリクエストを送信すると稼働状態が返されること', async () => {
      const response = await fetch(`http://localhost:${TEST_PORT}/health`);
      
      expect(response.status).toBe(200);
      
      const data = await response.json() as HealthStatus;
      expect(data).toHaveProperty('status');
      expect(data).toHaveProperty('timestamp');
      expect(data).toHaveProperty('connections');
      expect(data).toHaveProperty('sensor_connected');
      expect(data).toHaveProperty('robot_connected');
    });

    test('サーバーが正常に動作している場合、HTTPステータスコード200を返すこと', async () => {
      const response = await fetch(`http://localhost:${TEST_PORT}/health`);
      
      expect(response.status).toBe(200);
      
      const data = await response.json() as HealthStatus;
      expect(data.status).toBe('ok');
    });

    test('ヘルスチェック応答がJSON形式であること', async () => {
      const response = await fetch(`http://localhost:${TEST_PORT}/health`);
      
      const contentType = response.headers.get('content-type');
      expect(contentType).toContain('application/json');
    });
  });

  describe('要件 5.2: ヘルスチェック応答の内容', () => {
    test('ヘルスチェック応答にstatus、timestamp、connectionsが含まれること', async () => {
      const response = await fetch(`http://localhost:${TEST_PORT}/health`);
      const data = await response.json() as HealthStatus;

      expect(data.status).toBeDefined();
      expect(typeof data.status).toBe('string');
      
      expect(data.timestamp).toBeDefined();
      expect(typeof data.timestamp).toBe('string');
      
      expect(data.connections).toBeDefined();
      expect(typeof data.connections).toBe('object');
    });

    test('connections情報にタイプ別の接続数が含まれること', async () => {
      const response = await fetch(`http://localhost:${TEST_PORT}/health`);
      const data = await response.json() as HealthStatus;

      expect(data.connections).toHaveProperty('electron');
      expect(data.connections).toHaveProperty('sensor');
      expect(data.connections).toHaveProperty('robot');
      expect(data.connections).toHaveProperty('unknown');
      expect(data.connections).toHaveProperty('total');

      expect(typeof data.connections.electron).toBe('number');
      expect(typeof data.connections.sensor).toBe('number');
      expect(typeof data.connections.robot).toBe('number');
      expect(typeof data.connections.unknown).toBe('number');
      expect(typeof data.connections.total).toBe('number');
    });

    test('sensor_connectedとrobot_connectedがboolean値であること', async () => {
      const response = await fetch(`http://localhost:${TEST_PORT}/health`);
      const data = await response.json() as HealthStatus;

      expect(typeof data.sensor_connected).toBe('boolean');
      expect(typeof data.robot_connected).toBe('boolean');
    });

    test('timestampがISO 8601形式であること', async () => {
      const response = await fetch(`http://localhost:${TEST_PORT}/health`);
      const data = await response.json() as HealthStatus;

      // ISO 8601形式の検証
      const isoDateRegex = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z$/;
      expect(data.timestamp).toMatch(isoDateRegex);
      
      // 有効な日付であることを確認
      const date = new Date(data.timestamp);
      expect(date.toString()).not.toBe('Invalid Date');
    });
  });

  describe('要件 5.3: 接続状態の報告', () => {
    test('クライアントが接続していない場合、接続数が0であること', async () => {
      const response = await fetch(`http://localhost:${TEST_PORT}/health`);
      const data = await response.json() as HealthStatus;

      expect(data.connections.total).toBe(0);
      expect(data.connections.electron).toBe(0);
      expect(data.connections.sensor).toBe(0);
      expect(data.connections.robot).toBe(0);
      expect(data.sensor_connected).toBe(false);
      expect(data.robot_connected).toBe(false);
    });

    test('Electronクライアントが接続している場合、接続数が正しく報告されること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('registered', async () => {
        // 少し待ってからヘルスチェック
        setTimeout(async () => {
          const response = await fetch(`http://localhost:${TEST_PORT}/health`);
          const data = await response.json() as HealthStatus;

          expect(data.connections.electron).toBe(1);
          expect(data.connections.total).toBe(1);
          expect(data.sensor_connected).toBe(false);
          expect(data.robot_connected).toBe(false);

          client.disconnect();
          done();
        }, 100);
      });
    });

    test('センサークライアントが接続している場合、sensor_connectedがtrueであること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'sensor' });
      });

      client.on('registered', async () => {
        setTimeout(async () => {
          const response = await fetch(`http://localhost:${TEST_PORT}/health`);
          const data = await response.json() as HealthStatus;

          expect(data.connections.sensor).toBe(1);
          expect(data.connections.total).toBe(1);
          expect(data.sensor_connected).toBe(true);
          expect(data.robot_connected).toBe(false);

          client.disconnect();
          done();
        }, 100);
      });
    });

    test('ロボットクライアントが接続している場合、robot_connectedがtrueであること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'robot' });
      });

      client.on('registered', async () => {
        setTimeout(async () => {
          const response = await fetch(`http://localhost:${TEST_PORT}/health`);
          const data = await response.json() as HealthStatus;

          expect(data.connections.robot).toBe(1);
          expect(data.connections.total).toBe(1);
          expect(data.sensor_connected).toBe(false);
          expect(data.robot_connected).toBe(true);

          client.disconnect();
          done();
        }, 100);
      });
    });

    test('複数のクライアントが接続している場合、接続数が正しく報告されること', (done) => {
      const electron1 = ioClient(`http://localhost:${TEST_PORT}`);
      const electron2 = ioClient(`http://localhost:${TEST_PORT}`);
      const sensorClient = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);

      let registeredCount = 0;

      const checkAllRegistered = async () => {
        registeredCount++;
        if (registeredCount === 4) {
          setTimeout(async () => {
            const response = await fetch(`http://localhost:${TEST_PORT}/health`);
            const data = await response.json() as HealthStatus;

            expect(data.connections.electron).toBe(2);
            expect(data.connections.sensor).toBe(1);
            expect(data.connections.robot).toBe(1);
            expect(data.connections.total).toBe(4);
            expect(data.sensor_connected).toBe(true);
            expect(data.robot_connected).toBe(true);

            electron1.disconnect();
            electron2.disconnect();
            sensorClient.disconnect();
            robotClient.disconnect();
            done();
          }, 100);
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

      sensorClient.on('connect', () => {
        sensorClient.emit('register_client', { client_type: 'sensor' });
      });
      sensorClient.on('registered', checkAllRegistered);

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });
      robotClient.on('registered', checkAllRegistered);
    });
  });

  describe('要件 5.4: 接続状態の動的更新', () => {
    test('クライアントが切断した後、接続数が減少すること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('registered', async () => {
        // 接続中の状態を確認
        const response1 = await fetch(`http://localhost:${TEST_PORT}/health`);
        const data1 = await response1.json() as HealthStatus;
        expect(data1.connections.electron).toBe(1);

        // 切断
        client.disconnect();

        // 少し待ってから再度確認
        setTimeout(async () => {
          const response2 = await fetch(`http://localhost:${TEST_PORT}/health`);
          const data2 = await response2.json() as HealthStatus;
          expect(data2.connections.electron).toBe(0);
          expect(data2.connections.total).toBe(0);
          done();
        }, 200);
      });
    });

    test('センサークライアントが切断した後、sensor_connectedがfalseになること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'sensor' });
      });

      client.on('registered', async () => {
        // 接続中の状態を確認
        const response1 = await fetch(`http://localhost:${TEST_PORT}/health`);
        const data1 = await response1.json() as HealthStatus;
        expect(data1.sensor_connected).toBe(true);

        // 切断
        client.disconnect();

        // 少し待ってから再度確認
        setTimeout(async () => {
          const response2 = await fetch(`http://localhost:${TEST_PORT}/health`);
          const data2 = await response2.json() as HealthStatus;
          expect(data2.sensor_connected).toBe(false);
          done();
        }, 200);
      });
    });

    test('ロボットクライアントが切断した後、robot_connectedがfalseになること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'robot' });
      });

      client.on('registered', async () => {
        // 接続中の状態を確認
        const response1 = await fetch(`http://localhost:${TEST_PORT}/health`);
        const data1 = await response1.json() as HealthStatus;
        expect(data1.robot_connected).toBe(true);

        // 切断
        client.disconnect();

        // 少し待ってから再度確認
        setTimeout(async () => {
          const response2 = await fetch(`http://localhost:${TEST_PORT}/health`);
          const data2 = await response2.json() as HealthStatus;
          expect(data2.robot_connected).toBe(false);
          done();
        }, 200);
      });
    });

    test('複数回ヘルスチェックを実行しても正しい状態が返されること', async () => {
      // 1回目
      const response1 = await fetch(`http://localhost:${TEST_PORT}/health`);
      const data1 = await response1.json() as HealthStatus;
      expect(data1.status).toBe('ok');

      // 2回目
      const response2 = await fetch(`http://localhost:${TEST_PORT}/health`);
      const data2 = await response2.json() as HealthStatus;
      expect(data2.status).toBe('ok');

      // 3回目
      const response3 = await fetch(`http://localhost:${TEST_PORT}/health`);
      const data3 = await response3.json() as HealthStatus;
      expect(data3.status).toBe('ok');

      // タイムスタンプが異なることを確認（各リクエストで新しい状態を取得）
      expect(data1.timestamp).not.toBe(data2.timestamp);
      expect(data2.timestamp).not.toBe(data3.timestamp);
    });
  });

  describe('要件 5.1, 5.2: ヘルスチェックの信頼性', () => {
    test('同時に複数のヘルスチェックリクエストを処理できること', async () => {
      const requests = [
        fetch(`http://localhost:${TEST_PORT}/health`),
        fetch(`http://localhost:${TEST_PORT}/health`),
        fetch(`http://localhost:${TEST_PORT}/health`),
        fetch(`http://localhost:${TEST_PORT}/health`),
        fetch(`http://localhost:${TEST_PORT}/health`),
      ];

      const responses = await Promise.all(requests);

      for (const response of responses) {
        expect(response.status).toBe(200);
        const data = await response.json() as HealthStatus;
        expect(data.status).toBe('ok');
        expect(data).toHaveProperty('connections');
      }
    });

    test('クライアント接続中でもヘルスチェックが正常に動作すること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('registered', async () => {
        // 複数回ヘルスチェックを実行
        const response1 = await fetch(`http://localhost:${TEST_PORT}/health`);
        const data1 = await response1.json() as HealthStatus;
        expect(data1.connections.electron).toBeGreaterThanOrEqual(1);

        const response2 = await fetch(`http://localhost:${TEST_PORT}/health`);
        const data2 = await response2.json() as HealthStatus;
        expect(data2.connections.electron).toBeGreaterThanOrEqual(1);

        client.disconnect();
        done();
      });
    });
  });
});
