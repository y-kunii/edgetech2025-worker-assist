import React, { useEffect, useState, useCallback } from 'react';
import { WorkDataStore } from '../../stores/WorkDataStore';
import { EfficiencyIndicator } from './EfficiencyIndicator';
import { NotificationSystem } from './NotificationSystem';
import { EfficiencyMetrics, Notification } from '../../types';
import './EfficiencyManager.css';

interface EfficiencyManagerProps {
  workDataStore: WorkDataStore;
  lowEfficiencyThreshold?: number;
  criticalEfficiencyThreshold?: number;
  className?: string;
}

export const EfficiencyManager: React.FC<EfficiencyManagerProps> = ({
  workDataStore,
  lowEfficiencyThreshold = 70,
  criticalEfficiencyThreshold = 50,
  className
}) => {
  const [efficiencyMetrics, setEfficiencyMetrics] = useState<EfficiencyMetrics>({
    current: 0,
    target: 85,
    trend: 'stable',
    lastUpdated: new Date()
  });
  const [notifications, setNotifications] = useState<Notification[]>([]);

  // WorkDataStoreからの状態更新を監視
  useEffect(() => {
    const handleEfficiencyUpdate = (metrics: EfficiencyMetrics) => {
      setEfficiencyMetrics(metrics);
    };

    const handleNotificationAdded = (notification: Notification) => {
      setNotifications(prev => [notification, ...prev]);
    };

    // イベントリスナーを登録
    workDataStore.on('efficiency_metrics_updated', handleEfficiencyUpdate);
    workDataStore.on('notification_added', handleNotificationAdded);

    // 初期状態を取得
    const state = workDataStore.getState();
    setEfficiencyMetrics(state.efficiencyMetrics);
    setNotifications(state.notifications);

    // クリーンアップ
    return () => {
      workDataStore.off('efficiency_metrics_updated', handleEfficiencyUpdate);
      workDataStore.off('notification_added', handleNotificationAdded);
    };
  }, [workDataStore]);

  // アラートハンドラー
  const handleAlert = useCallback((alertType: 'low_efficiency' | 'efficiency_drop', message: string) => {
    const notificationType = alertType === 'low_efficiency' ? 
      (efficiencyMetrics.current < criticalEfficiencyThreshold ? 'error' : 'warning') : 
      'warning';

    workDataStore.addNotification({
      type: notificationType,
      title: alertType === 'low_efficiency' ? '効率低下アラート' : '効率急降下アラート',
      message
    });
  }, [workDataStore, efficiencyMetrics.current, criticalEfficiencyThreshold]);

  // 通知管理ハンドラー
  const handleMarkAsRead = useCallback((notificationId: string) => {
    workDataStore.markNotificationAsRead(notificationId);
  }, [workDataStore]);

  const handleMarkAllAsRead = useCallback(() => {
    workDataStore.markAllNotificationsAsRead();
  }, [workDataStore]);

  return (
    <div className={`efficiency-manager ${className || ''}`}>
      <div className="efficiency-manager-header">
        <h2>効率監視システム</h2>
        <NotificationSystem
          notifications={notifications}
          onMarkAsRead={handleMarkAsRead}
          onMarkAllAsRead={handleMarkAllAsRead}
          maxVisible={5}
        />
      </div>
      
      <div className="efficiency-manager-content">
        <EfficiencyIndicator
          metrics={efficiencyMetrics}
          onAlert={handleAlert}
          lowEfficiencyThreshold={lowEfficiencyThreshold}
          criticalEfficiencyThreshold={criticalEfficiencyThreshold}
        />
        
        <div className="efficiency-manager-info">
          <div className="threshold-info">
            <h4>アラート設定</h4>
            <div className="threshold-item">
              <span className="threshold-label">低効率閾値:</span>
              <span className="threshold-value">{lowEfficiencyThreshold}%</span>
            </div>
            <div className="threshold-item">
              <span className="threshold-label">危険閾値:</span>
              <span className="threshold-value">{criticalEfficiencyThreshold}%</span>
            </div>
            <div className="threshold-item">
              <span className="threshold-label">目標効率:</span>
              <span className="threshold-value">{efficiencyMetrics.target}%</span>
            </div>
          </div>
          
          <div className="efficiency-tips">
            <h4>効率改善のヒント</h4>
            <ul>
              {efficiencyMetrics.current < criticalEfficiencyThreshold && (
                <>
                  <li>作業手順を見直してください</li>
                  <li>機器の動作状況を確認してください</li>
                  <li>作業環境を最適化してください</li>
                </>
              )}
              {efficiencyMetrics.current >= criticalEfficiencyThreshold && efficiencyMetrics.current < lowEfficiencyThreshold && (
                <>
                  <li>作業のペースを調整してください</li>
                  <li>無駄な動作を減らしてください</li>
                  <li>ツールの配置を最適化してください</li>
                </>
              )}
              {efficiencyMetrics.current >= lowEfficiencyThreshold && efficiencyMetrics.current < efficiencyMetrics.target && (
                <>
                  <li>現在の作業ペースを維持してください</li>
                  <li>さらなる効率化を検討してください</li>
                </>
              )}
              {efficiencyMetrics.current >= efficiencyMetrics.target && (
                <>
                  <li>素晴らしい効率です！</li>
                  <li>この調子を維持してください</li>
                </>
              )}
            </ul>
          </div>
        </div>
      </div>
    </div>
  );
};

export default EfficiencyManager;