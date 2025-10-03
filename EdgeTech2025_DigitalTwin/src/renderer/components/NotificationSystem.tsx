import React, { useState, useEffect, useCallback, useMemo } from 'react';
import { Notification } from '../../types';
import './NotificationSystem.css';

interface NotificationSystemProps {
  notifications: Notification[];
  onMarkAsRead: (notificationId: string) => void;
  onMarkAllAsRead: () => void;
  maxVisible?: number;
}

interface NotificationItemProps {
  notification: Notification;
  onMarkAsRead: (notificationId: string) => void;
  onClose: () => void;
}

const NotificationItem: React.FC<NotificationItemProps> = ({ 
  notification, 
  onMarkAsRead, 
  onClose 
}) => {
  const [isVisible, setIsVisible] = useState(false);
  const [isClosing, setIsClosing] = useState(false);

  useEffect(() => {
    // アニメーション用の遅延
    const timer = setTimeout(() => setIsVisible(true), 50);
    return () => clearTimeout(timer);
  }, []);

  const handleClose = useCallback(() => {
    setIsClosing(true);
    setTimeout(() => {
      onClose();
      if (!notification.read) {
        onMarkAsRead(notification.id);
      }
    }, 300);
  }, [notification.id, notification.read, onMarkAsRead, onClose]);

  const handleClick = useCallback(() => {
    if (!notification.read) {
      onMarkAsRead(notification.id);
    }
  }, [notification.id, notification.read, onMarkAsRead]);

  const getNotificationIcon = (type: Notification['type']): string => {
    switch (type) {
      case 'success': return '✅';
      case 'warning': return '⚠️';
      case 'error': return '❌';
      case 'info': return 'ℹ️';
      default: return 'ℹ️';
    }
  };

  const formatTimestamp = (timestamp: Date): string => {
    const now = new Date();
    const diff = now.getTime() - timestamp.getTime();
    const minutes = Math.floor(diff / (1000 * 60));
    
    if (minutes < 1) return 'たった今';
    if (minutes < 60) return `${minutes}分前`;
    
    const hours = Math.floor(minutes / 60);
    if (hours < 24) return `${hours}時間前`;
    
    const days = Math.floor(hours / 24);
    return `${days}日前`;
  };

  return (
    <div 
      className={`notification-item notification-${notification.type} ${
        isVisible ? 'notification-visible' : ''
      } ${isClosing ? 'notification-closing' : ''} ${
        notification.read ? 'notification-read' : 'notification-unread'
      }`}
      onClick={handleClick}
    >
      <div className="notification-content">
        <div className="notification-header">
          <span className="notification-icon">
            {getNotificationIcon(notification.type)}
          </span>
          <span className="notification-title">{notification.title}</span>
          <button 
            className="notification-close"
            onClick={(e) => {
              e.stopPropagation();
              handleClose();
            }}
            aria-label="通知を閉じる"
          >
            ×
          </button>
        </div>
        <div className="notification-message">{notification.message}</div>
        <div className="notification-timestamp">
          {formatTimestamp(notification.timestamp)}
        </div>
      </div>
      {!notification.read && <div className="notification-unread-indicator" />}
    </div>
  );
};

export const NotificationSystem: React.FC<NotificationSystemProps> = ({
  notifications,
  onMarkAsRead,
  onMarkAllAsRead,
  maxVisible = 5
}) => {
  const [isExpanded, setIsExpanded] = useState(false);
  const [visibleNotifications, setVisibleNotifications] = useState<string[]>([]);
  const [showPopups, setShowPopups] = useState(false);

  // 新しい通知が追加されたときの処理
  useEffect(() => {
    setVisibleNotifications(prev => {
      const newNotifications = notifications
        .filter(n => !prev.includes(n.id))
        .slice(0, maxVisible);
      
      if (newNotifications.length > 0) {
        // 新しい通知があった場合、ポップアップを表示
        setShowPopups(true);
        return [
          ...newNotifications.map(n => n.id),
          ...prev.slice(0, maxVisible - newNotifications.length)
        ];
      }
      return prev;
    });
  }, [notifications, maxVisible]);

  const handleRemoveNotification = useCallback((notificationId: string) => {
    setVisibleNotifications(prev => prev.filter(id => id !== notificationId));
  }, []);

  const unreadCount = notifications.filter(n => !n.read).length;
  
  // 表示する通知を決定
  const displayNotifications = useMemo(() => {
    if (isExpanded) {
      return notifications;
    }
    return notifications
      .filter(n => visibleNotifications.includes(n.id))
      .slice(0, maxVisible);
  }, [notifications, visibleNotifications, maxVisible, isExpanded]);

  const allNotifications = displayNotifications;

  return (
    <div className="notification-system">
      {/* 通知カウンターとコントロール */}
      <div className="notification-controls">
        <button 
          className={`notification-toggle ${unreadCount > 0 ? 'has-unread' : ''}`}
          onClick={() => {
            setIsExpanded(!isExpanded);
            if (!isExpanded) {
              setShowPopups(false);
            }
          }}
          aria-label={`通知 ${unreadCount > 0 ? `(${unreadCount}件未読)` : ''}`}
        >
          🔔
          {unreadCount > 0 && (
            <span className="notification-badge">{unreadCount}</span>
          )}
        </button>
        
        {unreadCount > 0 && (
          <button 
            className="notification-mark-all-read"
            onClick={onMarkAllAsRead}
            title="すべて既読にする"
          >
            すべて既読
          </button>
        )}
      </div>

      {/* 通知リスト */}
      <div className={`notification-list ${isExpanded ? 'expanded' : ''}`}>
        {isExpanded && (
          <>
            {allNotifications.length === 0 ? (
              <div className="notification-empty">
                通知はありません
              </div>
            ) : (
              <>
                {allNotifications.map(notification => (
                  <NotificationItem
                    key={notification.id}
                    notification={notification}
                    onMarkAsRead={onMarkAsRead}
                    onClose={() => handleRemoveNotification(notification.id)}
                  />
                ))}
                
                {notifications.length > 0 && (
                  <div className="notification-summary">
                    全{notifications.length}件の通知
                  </div>
                )}
              </>
            )}
          </>
        )}
      </div>

      {/* ポップアップ通知（最新の未読通知のみ、リストが展開されていない時のみ） */}
      {!isExpanded && showPopups && (
        <div className="notification-popup-container">
          {notifications
            .filter(n => !n.read && visibleNotifications.includes(n.id))
            .slice(0, 3)
            .map(notification => (
            <div 
              key={`popup-${notification.id}`}
              className={`notification-popup notification-popup-${notification.type}`}
            >
              <div className="notification-popup-content">
                <span className="notification-popup-icon">
                  {getNotificationIcon(notification.type)}
                </span>
                <div className="notification-popup-text">
                  <div className="notification-popup-title">{notification.title}</div>
                  <div className="notification-popup-message">{notification.message}</div>
                </div>
                <button 
                  className="notification-popup-close"
                  onClick={() => {
                    onMarkAsRead(notification.id);
                    handleRemoveNotification(notification.id);
                  }}
                >
                  ×
                </button>
              </div>
            </div>
            ))
          }
        </div>
      )}
    </div>
  );
};

// ヘルパー関数をエクスポート
const getNotificationIcon = (type: Notification['type']): string => {
  switch (type) {
    case 'success': return '✅';
    case 'warning': return '⚠️';
    case 'error': return '❌';
    case 'info': return 'ℹ️';
    default: return 'ℹ️';
  }
};

export default NotificationSystem;