import { WorkDataStore } from '../stores/WorkDataStore';
import { WorkData, ThresholdSettings, WorkerStatus, RobotStatus } from '../types';

describe('WorkDataStore', () => {
  let workDataStore: WorkDataStore;

  beforeEach(() => {
    workDataStore = new WorkDataStore();
  });

  afterEach(() => {
    workDataStore.destroy();
  });

  describe('initialization', () => {
    it('should initialize with default state', () => {
      const state = workDataStore.getState();
      
      expect(state.currentWorkData).toBeNull();
      expect(state.thresholdSettings.screwThreshold).toBe(5);
      expect(state.thresholdSettings.boltThreshold).toBe(3);
      expect(state.workHistory).toHaveLength(0);
      expect(state.statistics.totalWorkTime).toBe(0);
      expect(state.statistics.completedTasks).toBe(0);
      expect(state.statistics.averageEfficiency).toBe(0);
      expect(state.isConnected).toBe(false);
      expect(state.notifications).toHaveLength(0);
      expect(state.connectionQuality).toBeDefined();
    });
  });

  describe('updateSensorData', () => {
    it('should update sensor data successfully', () => {
      const validSensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: {
            state: 'waiting',
            grip: 'open'
          },
          screw_count: 3,
          bolt_count: 1,
          work_step: 'screw_tightening'
        }
      };

      const success = workDataStore.updateSensorData(validSensorData);
      expect(success).toBe(true);

      const state = workDataStore.getState();
      expect(state.currentWorkData).not.toBeNull();
      expect(state.currentWorkData?.workerStatus).toBe('screw_tightening');
      expect(state.currentWorkData?.screwCount).toBe(3);
      expect(state.currentWorkData?.boltCount).toBe(1);
    });

    it('should add work data to history', () => {
      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      workDataStore.updateSensorData(sensorData);

      const state = workDataStore.getState();
      expect(state.workHistory).toHaveLength(1);
      expect(state.workHistory[0].workerStatus).toBe('screw_tightening');
    });

    it('should emit threshold reached events', (done) => {
      // Set low thresholds for testing
      workDataStore.updateThresholdSettings({
        screwThreshold: 2,
        boltThreshold: 1
      });

      workDataStore.on('screw_threshold_reached', (data) => {
        expect(data.count).toBe(2);
        expect(data.threshold).toBe(2);
        done();
      });

      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 2,
          bolt_count: 0
        }
      };

      workDataStore.updateSensorData(sensorData);
    });

    it('should emit bolt threshold reached events', (done) => {
      workDataStore.updateThresholdSettings({
        screwThreshold: 5,
        boltThreshold: 1
      });

      workDataStore.on('bolt_threshold_reached', (data) => {
        expect(data.count).toBe(1);
        expect(data.threshold).toBe(1);
        done();
      });

      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'bolt_tightening',
          robot_status: { state: 'operating', grip: 'closed' },
          screw_count: 0,
          bolt_count: 1
        }
      };

      workDataStore.updateSensorData(sensorData);
    });

    it('should reject invalid sensor data', () => {
      const invalidSensorData = {
        type: 'invalid_type',
        data: { invalid_field: 'invalid_value' }
      };

      const success = workDataStore.updateSensorData(invalidSensorData);
      expect(success).toBe(false);

      const state = workDataStore.getState();
      expect(state.currentWorkData).toBeNull();
    });

    it('should update statistics on work completion', () => {
      const sensorData1 = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 1,
          bolt_count: 0
        }
      };

      const sensorData2 = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:01:00Z', // 1 minute later
        data: {
          worker_status: 'waiting',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 1,
          bolt_count: 0
        }
      };

      workDataStore.updateSensorData(sensorData1);
      workDataStore.updateSensorData(sensorData2);

      const state = workDataStore.getState();
      expect(state.statistics.totalWorkTime).toBeGreaterThan(0);
    });
  });

  describe('updateThresholdSettings', () => {
    it('should update threshold settings successfully', () => {
      const newSettings: ThresholdSettings = {
        screwThreshold: 10,
        boltThreshold: 5
      };

      const success = workDataStore.updateThresholdSettings(newSettings);
      expect(success).toBe(true);

      const state = workDataStore.getState();
      expect(state.thresholdSettings.screwThreshold).toBe(10);
      expect(state.thresholdSettings.boltThreshold).toBe(5);
    });

    it('should emit threshold_settings_updated event', (done) => {
      const newSettings: ThresholdSettings = {
        screwThreshold: 8,
        boltThreshold: 4
      };

      workDataStore.on('threshold_settings_updated', (settings) => {
        expect(settings.screwThreshold).toBe(8);
        expect(settings.boltThreshold).toBe(4);
        done();
      });

      workDataStore.updateThresholdSettings(newSettings);
    });

    it('should reject invalid threshold settings', () => {
      const invalidSettings = {
        screwThreshold: -1,
        boltThreshold: 0
      };

      const success = workDataStore.updateThresholdSettings(invalidSettings);
      expect(success).toBe(false);

      const state = workDataStore.getState();
      expect(state.thresholdSettings.screwThreshold).toBe(5); // Should remain default
      expect(state.thresholdSettings.boltThreshold).toBe(3); // Should remain default
    });
  });

  describe('updateConnectionStatus', () => {
    it('should update connection status', () => {
      workDataStore.updateConnectionStatus(true);
      expect(workDataStore.getState().isConnected).toBe(true);

      workDataStore.updateConnectionStatus(false);
      expect(workDataStore.getState().isConnected).toBe(false);
    });

    it('should emit connection_status_changed event', (done) => {
      workDataStore.on('connection_status_changed', (isConnected) => {
        expect(isConnected).toBe(true);
        done();
      });

      workDataStore.updateConnectionStatus(true);
    });
  });

  describe('updateConnectionQuality', () => {
    it('should update connection quality metrics', () => {
      const qualityMetrics = {
        latency: 50,
        dataRate: 1.5,
        stability: 'good' as const,
        lastUpdated: new Date()
      };

      workDataStore.updateConnectionQuality(qualityMetrics);

      const state = workDataStore.getState();
      expect(state.connectionQuality.latency).toBe(50);
      expect(state.connectionQuality.dataRate).toBe(1.5);
      expect(state.connectionQuality.stability).toBe('good');
    });

    it('should emit connection_quality_updated event', (done) => {
      const qualityMetrics = {
        latency: 25,
        dataRate: 2.0,
        stability: 'excellent' as const,
        lastUpdated: new Date()
      };

      workDataStore.on('connection_quality_updated', (quality) => {
        expect(quality.latency).toBe(25);
        expect(quality.dataRate).toBe(2.0);
        expect(quality.stability).toBe('excellent');
        done();
      });

      workDataStore.updateConnectionQuality(qualityMetrics);
    });
  });

  describe('addNotification', () => {
    it('should add notification successfully', () => {
      const notification = {
        type: 'info' as const,
        title: 'Test Notification',
        message: 'This is a test message'
      };

      workDataStore.addNotification(notification);

      const state = workDataStore.getState();
      expect(state.notifications).toHaveLength(1);
      expect(state.notifications[0].title).toBe('Test Notification');
      expect(state.notifications[0].read).toBe(false);
      expect(state.notifications[0].id).toBeDefined();
      expect(state.notifications[0].timestamp).toBeInstanceOf(Date);
    });

    it('should emit notification_added event', (done) => {
      const notification = {
        type: 'warning' as const,
        title: 'Warning',
        message: 'This is a warning'
      };

      workDataStore.on('notification_added', (addedNotification) => {
        expect(addedNotification.title).toBe('Warning');
        expect(addedNotification.type).toBe('warning');
        done();
      });

      workDataStore.addNotification(notification);
    });

    it('should limit notification count', () => {
      // Add more than the limit (assuming limit is 50)
      for (let i = 0; i < 55; i++) {
        workDataStore.addNotification({
          type: 'info',
          title: `Notification ${i}`,
          message: `Message ${i}`
        });
      }

      const state = workDataStore.getState();
      expect(state.notifications.length).toBeLessThanOrEqual(50);
    });
  });

  describe('markNotificationAsRead', () => {
    it('should mark notification as read', () => {
      const notification = {
        type: 'info' as const,
        title: 'Test Notification',
        message: 'This is a test message'
      };

      workDataStore.addNotification(notification);
      const state = workDataStore.getState();
      const notificationId = state.notifications[0].id;

      const success = workDataStore.markNotificationAsRead(notificationId);
      expect(success).toBe(true);

      const updatedState = workDataStore.getState();
      expect(updatedState.notifications[0].read).toBe(true);
    });

    it('should return false for non-existent notification', () => {
      const success = workDataStore.markNotificationAsRead('non-existent-id');
      expect(success).toBe(false);
    });
  });

  describe('markAllNotificationsAsRead', () => {
    it('should mark all notifications as read', () => {
      // Add multiple notifications
      for (let i = 0; i < 3; i++) {
        workDataStore.addNotification({
          type: 'info',
          title: `Notification ${i}`,
          message: `Message ${i}`
        });
      }

      workDataStore.markAllNotificationsAsRead();

      const state = workDataStore.getState();
      expect(state.notifications.every(n => n.read)).toBe(true);
    });

    it('should emit notifications_marked_read event', (done) => {
      workDataStore.addNotification({
        type: 'info',
        title: 'Test',
        message: 'Test'
      });

      workDataStore.on('notifications_marked_read', () => {
        done();
      });

      workDataStore.markAllNotificationsAsRead();
    });
  });

  describe('clearNotifications', () => {
    it('should clear all notifications', () => {
      // Add notifications
      workDataStore.addNotification({
        type: 'info',
        title: 'Test 1',
        message: 'Message 1'
      });
      workDataStore.addNotification({
        type: 'warning',
        title: 'Test 2',
        message: 'Message 2'
      });

      workDataStore.clearNotifications();

      const state = workDataStore.getState();
      expect(state.notifications).toHaveLength(0);
    });

    it('should emit notifications_cleared event', (done) => {
      workDataStore.addNotification({
        type: 'info',
        title: 'Test',
        message: 'Test'
      });

      workDataStore.on('notifications_cleared', () => {
        done();
      });

      workDataStore.clearNotifications();
    });
  });

  describe('clearWorkHistory', () => {
    it('should clear work history', () => {
      // Add some work data
      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      workDataStore.updateSensorData(sensorData);
      expect(workDataStore.getState().workHistory).toHaveLength(1);

      workDataStore.clearWorkHistory();
      expect(workDataStore.getState().workHistory).toHaveLength(0);
    });

    it('should emit work_history_cleared event', (done) => {
      workDataStore.on('work_history_cleared', () => {
        done();
      });

      workDataStore.clearWorkHistory();
    });
  });

  describe('resetStatistics', () => {
    it('should reset statistics to default values', () => {
      // Update some statistics first
      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      workDataStore.updateSensorData(sensorData);
      
      // Reset statistics
      workDataStore.resetStatistics();

      const state = workDataStore.getState();
      expect(state.statistics.totalWorkTime).toBe(0);
      expect(state.statistics.completedTasks).toBe(0);
      expect(state.statistics.averageEfficiency).toBe(0);
    });

    it('should emit statistics_reset event', (done) => {
      workDataStore.on('statistics_reset', () => {
        done();
      });

      workDataStore.resetStatistics();
    });
  });

  describe('getUnreadNotificationCount', () => {
    it('should return correct unread notification count', () => {
      expect(workDataStore.getUnreadNotificationCount()).toBe(0);

      // Add notifications
      workDataStore.addNotification({
        type: 'info',
        title: 'Test 1',
        message: 'Message 1'
      });
      workDataStore.addNotification({
        type: 'warning',
        title: 'Test 2',
        message: 'Message 2'
      });

      expect(workDataStore.getUnreadNotificationCount()).toBe(2);

      // Mark one as read
      const state = workDataStore.getState();
      workDataStore.markNotificationAsRead(state.notifications[0].id);

      expect(workDataStore.getUnreadNotificationCount()).toBe(1);
    });
  });

  describe('getCurrentProgress', () => {
    it('should return current progress information', () => {
      const sensorData = {
        type: 'sensor_data',
        timestamp: '2024-01-01T12:00:00Z',
        data: {
          worker_status: 'screw_tightening',
          robot_status: { state: 'waiting', grip: 'open' },
          screw_count: 3,
          bolt_count: 1
        }
      };

      workDataStore.updateSensorData(sensorData);

      const progress = workDataStore.getCurrentProgress();

      expect(progress.screwProgress.current).toBe(3);
      expect(progress.screwProgress.target).toBe(5);
      expect(progress.screwProgress.percentage).toBe(60);
      expect(progress.boltProgress.current).toBe(1);
      expect(progress.boltProgress.target).toBe(3);
      expect(progress.boltProgress.percentage).toBeCloseTo(33.33, 1);
    });
  });

  describe('destroy', () => {
    it('should clean up resources', () => {
      const eventSpy = jest.fn();
      workDataStore.on('test_event', eventSpy);

      workDataStore.destroy();

      // Verify that event listeners are removed
      workDataStore.emit('test_event');
      expect(eventSpy).not.toHaveBeenCalled();
    });
  });
});