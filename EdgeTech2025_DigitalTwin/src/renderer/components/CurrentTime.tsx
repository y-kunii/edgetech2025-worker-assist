import React, { useState, useEffect } from 'react';

interface CurrentTimeProps {
  className?: string;
  format?: 'full' | 'time-only' | 'compact';
}

const CurrentTime: React.FC<CurrentTimeProps> = ({ 
  className = '', 
  format = 'full' 
}) => {
  const [currentTime, setCurrentTime] = useState(new Date());

  useEffect(() => {
    // 1秒ごとに時刻を更新
    const timer = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);

    return () => clearInterval(timer);
  }, []);

  const formatTime = (date: Date): string => {
    const options: Intl.DateTimeFormatOptions = {
      timeZone: 'Asia/Tokyo',
      hour12: false
    };

    switch (format) {
      case 'time-only':
        return date.toLocaleTimeString('ja-JP', {
          ...options,
          hour: '2-digit',
          minute: '2-digit',
          second: '2-digit'
        });
      
      case 'compact':
        return date.toLocaleTimeString('ja-JP', {
          ...options,
          hour: '2-digit',
          minute: '2-digit'
        });
      
      case 'full':
      default:
        return date.toLocaleString('ja-JP', {
          ...options,
          year: 'numeric',
          month: '2-digit',
          day: '2-digit',
          hour: '2-digit',
          minute: '2-digit',
          second: '2-digit'
        });
    }
  };

  const getDateParts = (date: Date) => {
    const dateStr = date.toLocaleDateString('ja-JP', {
      timeZone: 'Asia/Tokyo',
      year: 'numeric',
      month: '2-digit',
      day: '2-digit'
    });
    
    const timeStr = date.toLocaleTimeString('ja-JP', {
      timeZone: 'Asia/Tokyo',
      hour12: false,
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    });

    return { dateStr, timeStr };
  };

  if (format === 'full') {
    const { dateStr, timeStr } = getDateParts(currentTime);
    
    return (
      <div className={`current-time-display ${className}`}>
        <div className="current-date">{dateStr}</div>
        <div className="current-time">{timeStr}</div>
      </div>
    );
  }

  return (
    <div className={`current-time-display ${className}`}>
      <div className="current-time">{formatTime(currentTime)}</div>
    </div>
  );
};

export default CurrentTime;