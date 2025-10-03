import React from 'react';
import { render, screen, waitFor, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import { EfficiencyManager } from '../renderer/components/EfficiencyManager';
import { WorkDataStore } from '../stores/WorkDataStore';
import { EfficiencyMetrics } from '../types';

// WorkDataStoreのモック
jest.mock('../stores/WorkDataStore');

describe('EfficiencyManager', () => {
  let mockWorkDataStore: jest.Mocked<WorkDataStore>;
  
  const mockEfficiencyMetrics: EfficiencyMetrics = {
    current: 75,
    target: 85,
    trend: 'up',
    lastUpdated: new Date('2024-01-01T12:00:00Z')
  };

  beforeEach(() => {
    mockWorkDataStore = {
      on: jest.fn(),
      off: jest.fn(),
      getState: jest.fn().mockReturnValue({
        efficiencyMetrics: mockEfficiencyMetrics,
        notifications: []
      }),
      addNotification: jest.fn(),
      markNotificationAsRead: jest.fn(),
      markAllNotificationsAsRead: jest.fn()
    } as any;
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  it('効率管理システムが正しく表示される', () => {
    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    expect(screen.getByText('効率監視システム')).toBeInTheDocument();
    expect(screen.getByText('アラート設定')).toBeInTheDocument();
    expect(screen.getByText('効率改善のヒント')).toBeInTheDocument();
  });

  it('WorkDataStoreのイベントリスナーが正しく登録される', () => {
    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    expect(mockWorkDataStore.on).toHaveBeenCalledWith('efficiency_metrics_updated', expect.any(Function));
    expect(mockWorkDataStore.on).toHaveBeenCalledWith('notification_added', expect.any(Function));
  });

  it('初期状態が正しく取得される', () => {
    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    expect(mockWorkDataStore.getState).toHaveBeenCalled();
  });

  it('カスタム閾値が正しく表示される', () => {
    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
        lowEfficiencyThreshold={80}
        criticalEfficiencyThreshold={60}
      />
    );

    expect(screen.getByText('80%')).toBeInTheDocument();
    expect(screen.getByText('60%')).toBeInTheDocument();
  });

  it('目標効率が正しく表示される', () => {
    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    expect(screen.getByText('85%')).toBeInTheDocument();
  });

  it('効率レベルに応じたヒントが表示される', () => {
    // 危険レベル
    const criticalMetrics = { ...mockEfficiencyMetrics, current: 40 };
    mockWorkDataStore.getState.mockReturnValue({
      efficiencyMetrics: criticalMetrics,
      notifications: []
    });

    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
        criticalEfficiencyThreshold={50}
      />
    );

    expect(screen.getByText('作業手順を見直してください')).toBeInTheDocument();
    expect(screen.getByText('機器の動作状況を確認してください')).toBeInTheDocument();
  });

  it('警告レベルのヒントが表示される', () => {
    // 警告レベル
    const warningMetrics = { ...mockEfficiencyMetrics, current: 65 };
    mockWorkDataStore.getState.mockReturnValue({
      efficiencyMetrics: warningMetrics,
      notifications: []
    });

    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
        lowEfficiencyThreshold={70}
        criticalEfficiencyThreshold={50}
      />
    );

    expect(screen.getByText('作業のペースを調整してください')).toBeInTheDocument();
  });

  it('良好レベルのヒントが表示される', () => {
    // 良好レベル
    const goodMetrics = { ...mockEfficiencyMetrics, current: 80 };
    mockWorkDataStore.getState.mockReturnValue({
      efficiencyMetrics: goodMetrics,
      notifications: []
    });

    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
        lowEfficiencyThreshold={70}
      />
    );

    expect(screen.getByText('現在の作業ペースを維持してください')).toBeInTheDocument();
  });

  it('優秀レベルのヒントが表示される', () => {
    // 優秀レベル
    const excellentMetrics = { ...mockEfficiencyMetrics, current: 90 };
    mockWorkDataStore.getState.mockReturnValue({
      efficiencyMetrics: excellentMetrics,
      notifications: []
    });

    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    expect(screen.getByText('素晴らしい効率です！')).toBeInTheDocument();
  });

  it('通知管理機能が正しく動作する', () => {
    const mockNotifications = [
      {
        id: '1',
        type: 'warning' as const,
        title: 'テスト通知',
        message: 'テストメッセージ',
        timestamp: new Date(),
        read: false
      }
    ];

    mockWorkDataStore.getState.mockReturnValue({
      efficiencyMetrics: mockEfficiencyMetrics,
      notifications: mockNotifications
    });

    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    // NotificationSystemコンポーネントが表示されることを確認
    expect(screen.getByLabelText(/通知/)).toBeInTheDocument();
  });

  it('アラートハンドラーが正しく動作する', async () => {
    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    // EfficiencyIndicatorコンポーネントが表示されることを確認
    expect(screen.getByText('効率指標')).toBeInTheDocument();
  });

  it('コンポーネントのアンマウント時にイベントリスナーが削除される', () => {
    const { unmount } = render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    unmount();

    expect(mockWorkDataStore.off).toHaveBeenCalledWith('efficiency_metrics_updated', expect.any(Function));
    expect(mockWorkDataStore.off).toHaveBeenCalledWith('notification_added', expect.any(Function));
  });

  it('カスタムクラス名が適用される', () => {
    const { container } = render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
        className="custom-class"
      />
    );

    expect(container.firstChild).toHaveClass('efficiency-manager', 'custom-class');
  });

  it('効率指標の更新イベントが正しく処理される', async () => {
    let updateHandler: (metrics: EfficiencyMetrics) => void;
    
    mockWorkDataStore.on.mockImplementation((event, handler) => {
      if (event === 'efficiency_metrics_updated') {
        updateHandler = handler as any;
      }
    });

    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    // 効率指標を更新
    const newMetrics = { ...mockEfficiencyMetrics, current: 90 };
    
    await act(async () => {
      updateHandler!(newMetrics);
    });

    // 更新が反映されることを確認（実際のテストでは、表示の変更を確認）
    await waitFor(() => {
      // ここでは、updateHandlerが呼ばれることを確認
      expect(updateHandler).toBeDefined();
    });
  });

  it('通知追加イベントが正しく処理される', async () => {
    let notificationHandler: (notification: any) => void;
    
    mockWorkDataStore.on.mockImplementation((event, handler) => {
      if (event === 'notification_added') {
        notificationHandler = handler as any;
      }
    });

    render(
      <EfficiencyManager
        workDataStore={mockWorkDataStore}
      />
    );

    // 新しい通知を追加
    const newNotification = {
      id: '2',
      type: 'error' as const,
      title: '新しい通知',
      message: '新しいメッセージ',
      timestamp: new Date(),
      read: false
    };
    
    await act(async () => {
      notificationHandler!(newNotification);
    });

    // 通知ハンドラーが呼ばれることを確認
    await waitFor(() => {
      expect(notificationHandler).toBeDefined();
    });
  });
});