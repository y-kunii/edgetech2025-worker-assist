/**
 * 統合テスト: ハートビートテスト
 * 要件: 1.3, 9.6
 */

import { io as ioClient } from 'socket.io-client';
import { MainServer } from '../../src/server/MainServer';
import { ServerConfig } from '../../src/types';

describe('Integration Test: Heartbeat', () => {
  let server: MainServer;
  let serverConfig: ServerConfig;
  const TEST_PORT = 3005;

  beforeAll(async () => {
    serverConfig = {
      port: TEST_PORT,
      cors_origin: '*',
      heartbeat_interval: 1000, // 1秒（テスト用に短く設定）
      connection_timeout: 3000, // 3秒（テスト用に短く設定）
      log_level: 'error',
      log_file: 'logs/test.log',
    };

    server = new MainServer(serverConfig);
    await server.start();
  });

  afterAll(async () => {
    await server.stop();
  });

  describe('要件 1.3: ハートビート送信', () => {
    test('サーバーから定期的にpingイベントが送信されること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);
      let pingCount = 0;

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('registered', () => {
        // pingイベントを待つ
      });

      client.on('ping', (data) => {
        pingCount++;
        expect(data).toHaveProperty('timestamp');
        expect(typeof data.timestamp).toBe('number');

        // 2回以上pingを受信したら成功
        if (pingCount >= 2) {
          client.disconnect();
          done();
        }
      });

      // タイムアウト処理
      setTimeout(() => {
        if (pingCount < 2) {
          client.disconnect();
          done(new Error('Did not receive enough ping events'));
        }
      }, 5000);
    }, 10000);

    test('複数のクライアントが同時にpingを受信すること', (done) => {
      const client1 = ioClient(`http://localhost:${TEST_PORT}`);
      const client2 = ioClient(`http://localhost:${TEST_PORT}`);
      const client3 = ioClient(`http://localhost:${TEST_PORT}`);

      let client1PingReceived = false;
      let client2PingReceived = false;
      let client3PingReceived = false;

      const checkAllReceived = () => {
        if (client1PingReceived && client2PingReceived && client3PingReceived) {
          client1.disconnect();
          client2.disconnect();
          client3.disconnect();
          done();
        }
      };

      client1.on('connect', () => {
        client1.emit('register_client', { client_type: 'electron' });
      });
      client1.on('ping', () => {
        client1PingReceived = true;
        checkAllReceived();
      });

      client2.on('connect', () => {
        client2.emit('register_client', { client_type: 'sensor' });
      });
      client2.on('ping', () => {
        client2PingReceived = true;
        checkAllReceived();
      });

      client3.on('connect', () => {
        client3.emit('register_client', { client_type: 'robot' });
      });
      client3.on('ping', () => {
        client3PingReceived = true;
        checkAllReceived();
      });

      // タイムアウト処理
      setTimeout(() => {
        client1.disconnect();
        client2.disconnect();
        client3.disconnect();
        if (!client1PingReceived || !client2PingReceived || !client3PingReceived) {
          done(new Error('Not all clients received ping'));
        }
      }, 5000);
    }, 10000);
  });

  describe('要件 1.3: Pong応答の処理', () => {
    test('クライアントからのpong応答が正しく処理されること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);
      let testCompleted = false;

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('registered', () => {
        // pingを受信したらpongを返す
      });

      client.on('ping', (data) => {
        if (!testCompleted) {
          testCompleted = true;
          // pongを送信
          client.emit('pong', { timestamp: data.timestamp });
          
          // pongが正常に処理されたことを確認するため、少し待ってから切断
          setTimeout(() => {
            client.disconnect();
            done();
          }, 100);
        }
      });

      // タイムアウト処理
      setTimeout(() => {
        if (!testCompleted) {
          testCompleted = true;
          if (client.connected) {
            client.disconnect();
          }
          done(new Error('Did not receive ping event'));
        }
      }, 3000);
    }, 10000);

    test('複数のクライアントがpong応答を送信できること', (done) => {
      const client1 = ioClient(`http://localhost:${TEST_PORT}`);
      const client2 = ioClient(`http://localhost:${TEST_PORT}`);

      let client1PongSent = false;
      let client2PongSent = false;

      const checkBothSent = () => {
        if (client1PongSent && client2PongSent) {
          setTimeout(() => {
            client1.disconnect();
            client2.disconnect();
            done();
          }, 100);
        }
      };

      client1.on('connect', () => {
        client1.emit('register_client', { client_type: 'electron' });
      });
      client1.on('ping', (data) => {
        client1.emit('pong', { timestamp: data.timestamp });
        client1PongSent = true;
        checkBothSent();
      });

      client2.on('connect', () => {
        client2.emit('register_client', { client_type: 'sensor' });
      });
      client2.on('ping', (data) => {
        client2.emit('pong', { timestamp: data.timestamp });
        client2PongSent = true;
        checkBothSent();
      });

      // タイムアウト処理
      setTimeout(() => {
        client1.disconnect();
        client2.disconnect();
        if (!client1PongSent || !client2PongSent) {
          done(new Error('Not all clients sent pong'));
        }
      }, 5000);
    }, 10000);
  });

  describe('要件 9.6: タイムアウト検出', () => {
    test('pong応答がない場合、接続がタイムアウトすること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`, {
        reconnection: false, // 再接続を無効化
      });

      let pingReceived = false;

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('registered', () => {
        // pingを受信してもpongを返さない（タイムアウトをシミュレート）
      });

      client.on('ping', () => {
        pingReceived = true;
        // pongを送信しない（タイムアウトさせる）
      });

      client.on('disconnect', (reason) => {
        // タイムアウトによる切断を確認
        if (pingReceived) {
          expect(reason).toBeDefined();
          done();
        }
      });

      // タイムアウト処理（connection_timeout + timeout check interval + 余裕）
      // 3000ms timeout + 10000ms check interval + 2000ms余裕 = 15000ms
      setTimeout(() => {
        if (client.connected) {
          client.disconnect();
          done(new Error('Client was not disconnected due to timeout'));
        }
      }, 15000);
    }, 20000);

    test('pong応答を送信し続ける限り接続が維持されること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);
      let pingCount = 0;

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      client.on('ping', (data) => {
        pingCount++;
        // pongを送信
        client.emit('pong', { timestamp: data.timestamp });

        // 3回以上pingを受信したら成功（接続が維持されている）
        if (pingCount >= 3) {
          expect(client.connected).toBe(true);
          client.disconnect();
          done();
        }
      });

      client.on('disconnect', (reason) => {
        if (pingCount < 3) {
          done(new Error(`Client disconnected prematurely: ${reason}`));
        }
      });

      // タイムアウト処理
      setTimeout(() => {
        if (pingCount < 3) {
          client.disconnect();
          done(new Error('Did not receive enough pings'));
        }
      }, 5000);
    }, 10000);
  });

  describe('要件 1.3, 9.6: ハートビートとタイムアウトの統合', () => {
    test('一部のクライアントがタイムアウトしても他のクライアントは影響を受けないこと', (done) => {
      const timeoutClient = ioClient(`http://localhost:${TEST_PORT}`, {
        reconnection: false,
      });
      const normalClient = ioClient(`http://localhost:${TEST_PORT}`);

      let timeoutClientDisconnected = false;
      let normalClientPingCount = 0;

      timeoutClient.on('connect', () => {
        timeoutClient.emit('register_client', { client_type: 'electron' });
      });

      timeoutClient.on('ping', () => {
        // pongを送信しない（タイムアウトさせる）
      });

      timeoutClient.on('disconnect', () => {
        timeoutClientDisconnected = true;
      });

      normalClient.on('connect', () => {
        normalClient.emit('register_client', { client_type: 'sensor' });
      });

      normalClient.on('ping', (data) => {
        normalClientPingCount++;
        // pongを送信
        normalClient.emit('pong', { timestamp: data.timestamp });

        // タイムアウトクライアントが切断され、通常クライアントは接続を維持していることを確認
        if (timeoutClientDisconnected && normalClientPingCount >= 2) {
          expect(normalClient.connected).toBe(true);
          normalClient.disconnect();
          done();
        }
      });

      // タイムアウト処理（connection_timeout + timeout check interval + 余裕）
      setTimeout(() => {
        if (timeoutClient.connected) {
          timeoutClient.disconnect();
        }
        if (normalClient.connected) {
          normalClient.disconnect();
        }
        if (!timeoutClientDisconnected) {
          done(new Error('Timeout client was not disconnected'));
        } else if (normalClientPingCount < 2) {
          done(new Error('Normal client did not receive enough pings'));
        }
      }, 18000);
    }, 20000);
  });

  describe('要件 1.3: ハートビートの再開', () => {
    test('クライアントが再接続した場合、ハートビートが再開されること', (done) => {
      const client = ioClient(`http://localhost:${TEST_PORT}`);

      client.on('connect', () => {
        client.emit('register_client', { client_type: 'electron' });
      });

      let firstConnectionPingReceived = false;

      client.on('ping', () => {
        if (!firstConnectionPingReceived) {
          firstConnectionPingReceived = true;
          // 一度切断
          client.disconnect();

          // 再接続
          setTimeout(() => {
            const client2 = ioClient(`http://localhost:${TEST_PORT}`);

            client2.on('connect', () => {
              client2.emit('register_client', { client_type: 'electron' });
            });

            client2.on('ping', () => {
              // 再接続後もpingを受信できることを確認
              expect(client2.connected).toBe(true);
              client2.disconnect();
              done();
            });

            // タイムアウト処理（heartbeat_interval + 余裕）
            setTimeout(() => {
              if (client2.connected) {
                client2.disconnect();
              }
              done(new Error('Did not receive ping after reconnection'));
            }, 3000);
          }, 500);
        }
      });

      // タイムアウト処理
      setTimeout(() => {
        if (client.connected) {
          client.disconnect();
        }
        if (!firstConnectionPingReceived) {
          done(new Error('Did not receive ping in first connection'));
        }
      }, 8000);
    }, 20000);
  });
});
