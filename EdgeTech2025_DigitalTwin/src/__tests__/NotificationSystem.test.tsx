import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { NotificationSystem } from '../renderer/components/NotificationSystem';
import { Notification } from '../types';

// モックデータ
const mockNotifications: Notification[] = [
  {
    id: '1',
    type: 'success',
    title: 'ネジ締め完了',
    message: 'ネジ締め作業が目標回数に達しました。',
    timestamp: new Date('2024-01-01T12:00:00Z'),
    read: false
  },
  {
    id: '2',
    type: 'warning',
    title: '接続が不安定',
    message: 'WebSocket接続が不安定です。',
    timestamp: new Date('2024-01-01T11:55:00Z'),
    read: false
  },
  {
    id: '3',
    type: 'info',
    title: '作業開始',
    message: '新しい作業セッションが開始されました。',
    timestamp: new Date('2024-01-01T11:50:00Z'),
    read: true
  }
];

describe('NotificationSystem', () => {
  const mockOnMarkAsRead = jest.fn();
  const mockOnMarkAllAsRead = jest.fn();

  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('通知カウンターが正しく表示される', () => {
    render(
      <NotificationSystem
        notifications={mockNotifications}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    const toggleButton = screen.getByRole('button', { name: /通知.*2件未読/ });
    expect(toggleButton).toBeInTheDocument();
    
    const badge = screen.getByText('2');
    expect(badge).toBeInTheDocument();
  });

  it('通知リストが展開・折りたたみできる', async () => {
    render(
      <NotificationSystem
        notifications={mockNotifications}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    const toggleButton = screen.getByLabelText(/通知.*2件未読/);
    
    // 初期状態では通知リストは表示されていない（通知リスト内のクローズボタンを確認）
    expect(screen.queryAllByLabelText('通知を閉じる')).toHaveLength(0);
    
    // クリックして展開
    fireEvent.click(toggleButton);
    
    await waitFor(() => {
      expect(screen.getAllByLabelText('通知を閉じる')).toHaveLength(3);
      expect(screen.getByText('ネジ締め作業が目標回数に達しました。')).toBeInTheDocument();
      expect(screen.getByText('WebSocket接続が不安定です。')).toBeInTheDocument();
      expect(screen.getByText('新しい作業セッションが開始されました。')).toBeInTheDocument();
    });
  });

  it('通知タイプに応じたアイコンが表示される', async () => {
    render(
      <NotificationSystem
        notifications={mockNotifications}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    const toggleButton = screen.getByLabelText(/通知.*2件未読/);
    fireEvent.click(toggleButton);

    await waitFor(() => {
      // 通知リスト内のアイコンを確認（クラス名で特定）
      const notificationIcons = document.querySelectorAll('.notification-icon');
      expect(notificationIcons).toHaveLength(3);
      expect(notificationIcons[0]).toHaveTextContent('✅');
      expect(notificationIcons[1]).toHaveTextContent('⚠️');
      expect(notificationIcons[2]).toHaveTextContent('ℹ️');
    });
  });

  it('通知をクリックすると既読になる', async () => {
    render(
      <NotificationSystem
        notifications={mockNotifications}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    const toggleButton = screen.getByLabelText(/通知.*2件未読/);
    fireEvent.click(toggleButton);

    await waitFor(() => {
      const notificationItems = document.querySelectorAll('.notification-item');
      fireEvent.click(notificationItems[0]);
      
      expect(mockOnMarkAsRead).toHaveBeenCalledWith('1');
    });
  });

  it('すべて既読ボタンが機能する', () => {
    render(
      <NotificationSystem
        notifications={mockNotifications}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    const markAllReadButton = screen.getByText('すべて既読');
    fireEvent.click(markAllReadButton);

    expect(mockOnMarkAllAsRead).toHaveBeenCalled();
  });

  it('通知の閉じるボタンが機能する', async () => {
    render(
      <NotificationSystem
        notifications={mockNotifications}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    const toggleButton = screen.getByLabelText(/通知.*2件未読/);
    fireEvent.click(toggleButton);

    await waitFor(() => {
      const closeButtons = screen.getAllByLabelText('通知を閉じる');
      fireEvent.click(closeButtons[0]);
      
      // 未読の通知を閉じた場合、既読にマークされる
      expect(mockOnMarkAsRead).toHaveBeenCalledWith('1');
    });
  });

  it('未読通知がない場合、バッジが表示されない', () => {
    const readNotifications = mockNotifications.map(n => ({ ...n, read: true }));
    
    render(
      <NotificationSystem
        notifications={readNotifications}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    expect(screen.queryByText('すべて既読')).not.toBeInTheDocument();
    expect(screen.queryByText('3')).not.toBeInTheDocument();
  });

  it('通知が空の場合、適切なメッセージが表示される', async () => {
    render(
      <NotificationSystem
        notifications={[]}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    const toggleButton = screen.getByLabelText('通知');
    fireEvent.click(toggleButton);

    await waitFor(() => {
      expect(screen.getByText('通知はありません')).toBeInTheDocument();
    });
  });

  it('maxVisibleプロパティが正しく動作する', async () => {
    const manyNotifications = Array.from({ length: 10 }, (_, i) => ({
      id: `${i + 1}`,
      type: 'info' as const,
      title: `通知 ${i + 1}`,
      message: `メッセージ ${i + 1}`,
      timestamp: new Date(),
      read: false
    }));

    render(
      <NotificationSystem
        notifications={manyNotifications}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
        maxVisible={3}
      />
    );

    const toggleButton = screen.getByLabelText(/通知.*10件未読/);
    fireEvent.click(toggleButton);

    await waitFor(() => {
      // 展開時は全通知が表示される
      expect(screen.getByText('全10件の通知')).toBeInTheDocument();
    });
  });

  it('時間表示が正しくフォーマットされる', async () => {
    const recentNotification: Notification = {
      id: '1',
      type: 'info',
      title: 'テスト通知',
      message: 'テストメッセージ',
      timestamp: new Date(Date.now() - 30 * 1000), // 30秒前
      read: false
    };

    render(
      <NotificationSystem
        notifications={[recentNotification]}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    const toggleButton = screen.getByLabelText(/通知.*1件未読/);
    fireEvent.click(toggleButton);

    await waitFor(() => {
      expect(screen.getByText('たった今')).toBeInTheDocument();
    });
  });

  it('アクセシビリティ属性が正しく設定される', () => {
    render(
      <NotificationSystem
        notifications={mockNotifications}
        onMarkAsRead={mockOnMarkAsRead}
        onMarkAllAsRead={mockOnMarkAllAsRead}
      />
    );

    const toggleButton = screen.getByLabelText(/通知.*2件未読/);
    expect(toggleButton).toHaveAttribute('aria-label');
  });
});