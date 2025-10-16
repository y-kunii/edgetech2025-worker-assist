/**
 * 統合テスト: データ中継テスト
 * 要件: 2.1, 2.2, 2.3, 4.1, 4.2, 4.3, 4.4
 */

import { io as ioClient } from 'socket.io-client';
import { MainServer } from '../../src/server/MainServer';
import { ServerConfig, SensorData, RobotCommand, RobotResponse } from '../../src/types';

describe('Integration Test: Data Routing', () => {
  let server: MainServer;
  let serverConfig: ServerConfig;
  const TEST_PORT = 3003;

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

  describe('要件 2.1, 2.2, 2.3: センサーデータの転送', () => {
    test('センサーデータがElectronクライアントに転送されること', (done) => {
      const sensorClient = ioClient(`http://localhost:${TEST_PORT}`);
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);

      const testSensorData: SensorData = {
        worker_status: 'screw_tightening',
        robot_status: {
          state: 'operating',
          grip: 'closed',
        },
        screw_count: 5,
        bolt_count: 3,
        work_step: 'step_1',
        timestamp: new Date().toISOString(),
      };

      let sensorRegistered = false;
      let electronRegistered = false;

      const checkBothRegistered = () => {
        if (sensorRegistered && electronRegistered) {
          // 両方登録完了したらセンサーデータを送信
          sensorClient.emit('sensor_data', testSensorData);
        }
      };

      sensorClient.on('connect', () => {
        sensorClient.emit('register_client', { client_type: 'sensor' });
      });

      sensorClient.on('registered', () => {
        sensorRegistered = true;
        checkBothRegistered();
      });

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });

      electronClient.on('registered', () => {
        electronRegistered = true;
        checkBothRegistered();
      });

      electronClient.on('sensor_data', (data: SensorData) => {
        expect(data.worker_status).toBe(testSensorData.worker_status);
        expect(data.robot_status.state).toBe(testSensorData.robot_status.state);
        expect(data.robot_status.grip).toBe(testSensorData.robot_status.grip);
        expect(data.screw_count).toBe(testSensorData.screw_count);
        expect(data.bolt_count).toBe(testSensorData.bolt_count);
        expect(data.work_step).toBe(testSensorData.work_step);

        sensorClient.disconnect();
        electronClient.disconnect();
        done();
      });
    });

    test('センサーデータが複数のElectronクライアントに転送されること', (done) => {
      const sensorClient = ioClient(`http://localhost:${TEST_PORT}`);
      const electron1 = ioClient(`http://localhost:${TEST_PORT}`);
      const electron2 = ioClient(`http://localhost:${TEST_PORT}`);

      const testSensorData: SensorData = {
        worker_status: 'waiting',
        robot_status: {
          state: 'waiting',
          grip: 'open',
        },
        screw_count: 0,
        bolt_count: 0,
        work_step: 'idle',
      };

      let registeredCount = 0;
      let receivedCount = 0;

      const checkAllRegistered = () => {
        registeredCount++;
        if (registeredCount === 3) {
          sensorClient.emit('sensor_data', testSensorData);
        }
      };

      const checkAllReceived = () => {
        receivedCount++;
        if (receivedCount === 2) {
          sensorClient.disconnect();
          electron1.disconnect();
          electron2.disconnect();
          done();
        }
      };

      sensorClient.on('connect', () => {
        sensorClient.emit('register_client', { client_type: 'sensor' });
      });
      sensorClient.on('registered', checkAllRegistered);

      electron1.on('connect', () => {
        electron1.emit('register_client', { client_type: 'electron' });
      });
      electron1.on('registered', checkAllRegistered);
      electron1.on('sensor_data', (data: SensorData) => {
        expect(data.worker_status).toBe(testSensorData.worker_status);
        checkAllReceived();
      });

      electron2.on('connect', () => {
        electron2.emit('register_client', { client_type: 'electron' });
      });
      electron2.on('registered', checkAllRegistered);
      electron2.on('sensor_data', (data: SensorData) => {
        expect(data.worker_status).toBe(testSensorData.worker_status);
        checkAllReceived();
      });
    });

    test('画像データを含むセンサーデータが転送されること', (done) => {
      const sensorClient = ioClient(`http://localhost:${TEST_PORT}`);
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);

      const testSensorData: SensorData = {
        worker_status: 'bolt_tightening',
        robot_status: {
          state: 'operating',
          grip: 'closed',
        },
        screw_count: 10,
        bolt_count: 5,
        work_step: 'step_2',
        image: 'base64encodedimagedata==',
      };

      let sensorRegistered = false;
      let electronRegistered = false;

      const checkBothRegistered = () => {
        if (sensorRegistered && electronRegistered) {
          sensorClient.emit('sensor_data', testSensorData);
        }
      };

      sensorClient.on('connect', () => {
        sensorClient.emit('register_client', { client_type: 'sensor' });
      });
      sensorClient.on('registered', () => {
        sensorRegistered = true;
        checkBothRegistered();
      });

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });
      electronClient.on('registered', () => {
        electronRegistered = true;
        checkBothRegistered();
      });

      electronClient.on('sensor_data', (data: SensorData) => {
        expect(data.image).toBe(testSensorData.image);
        expect(data.worker_status).toBe(testSensorData.worker_status);

        sensorClient.disconnect();
        electronClient.disconnect();
        done();
      });
    });
  });

  describe('要件 4.1, 4.2: ロボット指示の転送', () => {
    test('ロボット指示がロボット制御プログラムに転送されること', (done) => {
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);

      const testCommand: RobotCommand = {
        command: 'tool_handover',
        timestamp: new Date().toISOString(),
      };

      let electronRegistered = false;
      let robotRegistered = false;

      const checkBothRegistered = () => {
        if (electronRegistered && robotRegistered) {
          electronClient.emit('robot_command', testCommand);
        }
      };

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });
      electronClient.on('registered', () => {
        electronRegistered = true;
        checkBothRegistered();
      });

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });
      robotClient.on('registered', () => {
        robotRegistered = true;
        checkBothRegistered();
      });

      robotClient.on('robot_command', (command: RobotCommand) => {
        expect(command.command).toBe(testCommand.command);
        expect(command.timestamp).toBe(testCommand.timestamp);

        electronClient.disconnect();
        robotClient.disconnect();
        done();
      });
    });

    test('各種ロボットコマンドタイプが転送されること', (done) => {
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);

      const commands: RobotCommand[] = [
        { command: 'tool_handover', timestamp: new Date().toISOString() },
        { command: 'next_task', timestamp: new Date().toISOString() },
        { command: 'emergency_stop', timestamp: new Date().toISOString() },
        { command: 'reset', timestamp: new Date().toISOString() },
      ];

      let currentCommandIndex = 0;
      let electronRegistered = false;
      let robotRegistered = false;

      const checkBothRegistered = () => {
        if (electronRegistered && robotRegistered) {
          electronClient.emit('robot_command', commands[0]);
        }
      };

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });
      electronClient.on('registered', () => {
        electronRegistered = true;
        checkBothRegistered();
      });

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });
      robotClient.on('registered', () => {
        robotRegistered = true;
        checkBothRegistered();
      });

      robotClient.on('robot_command', (command: RobotCommand) => {
        expect(command.command).toBe(commands[currentCommandIndex].command);
        currentCommandIndex++;

        if (currentCommandIndex < commands.length) {
          electronClient.emit('robot_command', commands[currentCommandIndex]);
        } else {
          electronClient.disconnect();
          robotClient.disconnect();
          done();
        }
      });
    });

    test('データを含むロボット指示が転送されること', (done) => {
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);

      const testCommand: RobotCommand = {
        command: 'tool_handover',
        data: { tool_type: 'screwdriver', position: { x: 100, y: 200 } },
        timestamp: new Date().toISOString(),
      };

      let electronRegistered = false;
      let robotRegistered = false;

      const checkBothRegistered = () => {
        if (electronRegistered && robotRegistered) {
          electronClient.emit('robot_command', testCommand);
        }
      };

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });
      electronClient.on('registered', () => {
        electronRegistered = true;
        checkBothRegistered();
      });

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });
      robotClient.on('registered', () => {
        robotRegistered = true;
        checkBothRegistered();
      });

      robotClient.on('robot_command', (command: RobotCommand) => {
        expect(command.command).toBe(testCommand.command);
        expect(command.data).toEqual(testCommand.data);

        electronClient.disconnect();
        robotClient.disconnect();
        done();
      });
    });
  });

  describe('要件 4.3, 4.4: ロボット応答の転送', () => {
    test('ロボット応答がElectronクライアントに転送されること', (done) => {
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);

      const testResponse: RobotResponse = {
        command: 'tool_handover',
        status: 'success',
        timestamp: new Date().toISOString(),
      };

      let electronRegistered = false;
      let robotRegistered = false;

      const checkBothRegistered = () => {
        if (electronRegistered && robotRegistered) {
          robotClient.emit('robot_response', testResponse);
        }
      };

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });
      electronClient.on('registered', () => {
        electronRegistered = true;
        checkBothRegistered();
      });

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });
      robotClient.on('registered', () => {
        robotRegistered = true;
        checkBothRegistered();
      });

      electronClient.on('robot_response', (response: RobotResponse) => {
        expect(response.command).toBe(testResponse.command);
        expect(response.status).toBe(testResponse.status);
        expect(response.timestamp).toBe(testResponse.timestamp);

        electronClient.disconnect();
        robotClient.disconnect();
        done();
      });
    });

    test('各種ステータスのロボット応答が転送されること', (done) => {
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);

      const responses: RobotResponse[] = [
        { command: 'tool_handover', status: 'success', timestamp: new Date().toISOString() },
        { command: 'next_task', status: 'error', timestamp: new Date().toISOString() },
        { command: 'emergency_stop', status: 'emergency_stopped', timestamp: new Date().toISOString() },
      ];

      let currentResponseIndex = 0;
      let electronRegistered = false;
      let robotRegistered = false;

      const checkBothRegistered = () => {
        if (electronRegistered && robotRegistered) {
          robotClient.emit('robot_response', responses[0]);
        }
      };

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });
      electronClient.on('registered', () => {
        electronRegistered = true;
        checkBothRegistered();
      });

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });
      robotClient.on('registered', () => {
        robotRegistered = true;
        checkBothRegistered();
      });

      electronClient.on('robot_response', (response: RobotResponse) => {
        expect(response.command).toBe(responses[currentResponseIndex].command);
        expect(response.status).toBe(responses[currentResponseIndex].status);
        currentResponseIndex++;

        if (currentResponseIndex < responses.length) {
          robotClient.emit('robot_response', responses[currentResponseIndex]);
        } else {
          electronClient.disconnect();
          robotClient.disconnect();
          done();
        }
      });
    });

    test('データを含むロボット応答が転送されること', (done) => {
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);

      const testResponse: RobotResponse = {
        command: 'tool_handover',
        status: 'success',
        timestamp: new Date().toISOString(),
        data: { completion_time: 5.2, position: { x: 100, y: 200 } },
      };

      let electronRegistered = false;
      let robotRegistered = false;

      const checkBothRegistered = () => {
        if (electronRegistered && robotRegistered) {
          robotClient.emit('robot_response', testResponse);
        }
      };

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });
      electronClient.on('registered', () => {
        electronRegistered = true;
        checkBothRegistered();
      });

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });
      robotClient.on('registered', () => {
        robotRegistered = true;
        checkBothRegistered();
      });

      electronClient.on('robot_response', (response: RobotResponse) => {
        expect(response.command).toBe(testResponse.command);
        expect(response.status).toBe(testResponse.status);
        expect(response.data).toEqual(testResponse.data);

        electronClient.disconnect();
        robotClient.disconnect();
        done();
      });
    });

    test('ロボット応答が複数のElectronクライアントに転送されること', (done) => {
      const electron1 = ioClient(`http://localhost:${TEST_PORT}`);
      const electron2 = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);

      const testResponse: RobotResponse = {
        command: 'reset',
        status: 'success',
        timestamp: new Date().toISOString(),
      };

      let registeredCount = 0;
      let receivedCount = 0;

      const checkAllRegistered = () => {
        registeredCount++;
        if (registeredCount === 3) {
          robotClient.emit('robot_response', testResponse);
        }
      };

      const checkAllReceived = () => {
        receivedCount++;
        if (receivedCount === 2) {
          electron1.disconnect();
          electron2.disconnect();
          robotClient.disconnect();
          done();
        }
      };

      electron1.on('connect', () => {
        electron1.emit('register_client', { client_type: 'electron' });
      });
      electron1.on('registered', checkAllRegistered);
      electron1.on('robot_response', (response: RobotResponse) => {
        expect(response.command).toBe(testResponse.command);
        checkAllReceived();
      });

      electron2.on('connect', () => {
        electron2.emit('register_client', { client_type: 'electron' });
      });
      electron2.on('registered', checkAllRegistered);
      electron2.on('robot_response', (response: RobotResponse) => {
        expect(response.command).toBe(testResponse.command);
        checkAllReceived();
      });

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });
      robotClient.on('registered', checkAllRegistered);
    });
  });

  describe('要件 2.1, 4.1: エンドツーエンドデータフロー', () => {
    test('センサーデータとロボット指示が同時に正しく転送されること', (done) => {
      const sensorClient = ioClient(`http://localhost:${TEST_PORT}`);
      const robotClient = ioClient(`http://localhost:${TEST_PORT}`);
      const electronClient = ioClient(`http://localhost:${TEST_PORT}`);

      const testSensorData: SensorData = {
        worker_status: 'tool_handover',
        robot_status: { state: 'operating', grip: 'open' },
        screw_count: 7,
        bolt_count: 4,
        work_step: 'step_3',
      };

      const testCommand: RobotCommand = {
        command: 'next_task',
        timestamp: new Date().toISOString(),
      };

      let registeredCount = 0;
      let sensorDataReceived = false;
      let commandReceived = false;

      const checkAllRegistered = () => {
        registeredCount++;
        if (registeredCount === 3) {
          // 両方のデータを送信
          sensorClient.emit('sensor_data', testSensorData);
          electronClient.emit('robot_command', testCommand);
        }
      };

      const checkBothReceived = () => {
        if (sensorDataReceived && commandReceived) {
          sensorClient.disconnect();
          robotClient.disconnect();
          electronClient.disconnect();
          done();
        }
      };

      sensorClient.on('connect', () => {
        sensorClient.emit('register_client', { client_type: 'sensor' });
      });
      sensorClient.on('registered', checkAllRegistered);

      robotClient.on('connect', () => {
        robotClient.emit('register_client', { client_type: 'robot' });
      });
      robotClient.on('registered', checkAllRegistered);
      robotClient.on('robot_command', (command: RobotCommand) => {
        expect(command.command).toBe(testCommand.command);
        commandReceived = true;
        checkBothReceived();
      });

      electronClient.on('connect', () => {
        electronClient.emit('register_client', { client_type: 'electron' });
      });
      electronClient.on('registered', checkAllRegistered);
      electronClient.on('sensor_data', (data: SensorData) => {
        expect(data.worker_status).toBe(testSensorData.worker_status);
        sensorDataReceived = true;
        checkBothReceived();
      });
    });
  });
});
