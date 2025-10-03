import React, { useState, useCallback, useRef, useEffect } from 'react';

interface LiveVideoDisplayProps {
  image?: string;
  isConnected: boolean;
}

interface ImageCache {
  [key: string]: {
    url: string;
    timestamp: number;
  };
}

const LiveVideoDisplay: React.FC<LiveVideoDisplayProps> = ({ image, isConnected }) => {
  const [imageError, setImageError] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [imageSize, setImageSize] = useState({ width: 0, height: 0 });
  const [lastUpdateTime, setLastUpdateTime] = useState<Date | null>(null);
  const [errorCount, setErrorCount] = useState(0);
  const [retryAttempt, setRetryAttempt] = useState(0);
  const [frameRate, setFrameRate] = useState(0);
  
  const imgRef = useRef<HTMLImageElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const imageCache = useRef<ImageCache>({});
  const frameCountRef = useRef(0);
  const lastFrameTimeRef = useRef<number>(Date.now());
  const errorTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const retryTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  // フレームレート計算
  const updateFrameRate = useCallback(() => {
    const now = Date.now();
    const timeDiff = now - lastFrameTimeRef.current;
    
    if (timeDiff >= 1000) { // 1秒ごとに更新
      setFrameRate(frameCountRef.current);
      frameCountRef.current = 0;
      lastFrameTimeRef.current = now;
    } else {
      frameCountRef.current++;
    }
  }, []);

  // 画像キャッシュの管理
  const getCachedImageUrl = useCallback((imageData: string) => {
    const hash = btoa(imageData.substring(0, 100)); // 簡易ハッシュ
    const cached = imageCache.current[hash];
    
    if (cached && Date.now() - cached.timestamp < 5000) { // 5秒間キャッシュ
      return cached.url;
    }
    
    const url = `data:image/jpeg;base64,${imageData}`;
    imageCache.current[hash] = {
      url,
      timestamp: Date.now()
    };
    
    // 古いキャッシュを削除
    Object.keys(imageCache.current).forEach(key => {
      if (Date.now() - imageCache.current[key].timestamp > 10000) {
        delete imageCache.current[key];
      }
    });
    
    return url;
  }, []);

  // 画像読み込み成功時のハンドラー
  const handleImageLoad = useCallback((event: React.SyntheticEvent<HTMLImageElement>) => {
    const img = event.currentTarget;
    setImageSize({ width: img.naturalWidth, height: img.naturalHeight });
    setImageError(false);
    setIsLoading(false);
    setErrorCount(0);
    setRetryAttempt(0);
    setLastUpdateTime(new Date());
    updateFrameRate();
    
    // エラータイムアウトをクリア
    if (errorTimeoutRef.current) {
      clearTimeout(errorTimeoutRef.current);
      errorTimeoutRef.current = null;
    }
  }, [updateFrameRate]);

  // 画像読み込みエラー時のハンドラー
  const handleImageError = useCallback(() => {
    setIsLoading(false);
    setErrorCount(prev => prev + 1);
    
    // 連続エラーの場合は遅延を増加
    const delay = Math.min(1000 * Math.pow(2, retryAttempt), 10000);
    
    if (retryAttempt < 3) {
      // 自動リトライ
      retryTimeoutRef.current = setTimeout(() => {
        setRetryAttempt(prev => prev + 1);
        setImageError(false);
        setIsLoading(true);
      }, delay);
    } else {
      // 最大リトライ回数に達した場合
      setImageError(true);
      
      // 10秒後にリトライカウントをリセット
      errorTimeoutRef.current = setTimeout(() => {
        setRetryAttempt(0);
        setErrorCount(0);
      }, 10000);
    }
  }, [retryAttempt]);

  // 新しい画像データが来た時の処理
  useEffect(() => {
    if (image && isConnected) {
      setIsLoading(true);
      
      // 前回のリトライタイムアウトをクリア
      if (retryTimeoutRef.current) {
        clearTimeout(retryTimeoutRef.current);
        retryTimeoutRef.current = null;
      }
    } else if (!isConnected) {
      // 接続が切断された場合の処理
      setImageError(false);
      setIsLoading(false);
      setErrorCount(0);
      setRetryAttempt(0);
      setLastUpdateTime(null);
      setFrameRate(0);
      frameCountRef.current = 0;
    }
  }, [image, isConnected]);

  // コンポーネントのクリーンアップ
  useEffect(() => {
    return () => {
      if (errorTimeoutRef.current) {
        clearTimeout(errorTimeoutRef.current);
      }
      if (retryTimeoutRef.current) {
        clearTimeout(retryTimeoutRef.current);
      }
    };
  }, []);

  // レスポンシブスケーリングの計算
  const getImageStyles = useCallback(() => {
    if (!containerRef.current) return {};
    
    return {
      maxWidth: '100%',
      maxHeight: '100%',
      width: 'auto',
      height: 'auto',
      objectFit: 'contain' as const,
      borderRadius: '8px',
      transition: 'all 0.3s ease',
      boxShadow: '0 4px 12px rgba(0, 0, 0, 0.1)',
    };
  }, []);

  // 手動リトライ機能
  const handleManualRetry = useCallback(() => {
    setImageError(false);
    setIsLoading(true);
    setRetryAttempt(0);
    setErrorCount(0);
  }, []);

  // 接続状態に応じた表示内容
  const renderContent = () => {
    if (!isConnected) {
      return (
        <div className="flex-center live-video-placeholder disconnected">
          <div className="placeholder-content">
            <div className="placeholder-icon">🔌</div>
            <div className="placeholder-text">接続が切断されています</div>
            <div className="placeholder-subtext">WebSocket接続を確認してください</div>
            {errorCount > 0 && (
              <div className="error-stats">
                エラー回数: {errorCount}
              </div>
            )}
          </div>
        </div>
      );
    }

    if (!image) {
      return (
        <div className="flex-center live-video-placeholder waiting">
          <div className="placeholder-content">
            <div className="placeholder-icon">📹</div>
            <div className="placeholder-text">映像待機中...</div>
            <div className="placeholder-subtext">カメラからの映像を待機しています</div>
            <div className="waiting-animation">
              <div className="dot"></div>
              <div className="dot"></div>
              <div className="dot"></div>
            </div>
          </div>
        </div>
      );
    }

    if (imageError) {
      return (
        <div className="flex-center live-video-placeholder error">
          <div className="placeholder-content">
            <div className="placeholder-icon">⚠️</div>
            <div className="placeholder-text">画像読み込みエラー</div>
            <div className="placeholder-subtext">
              {retryAttempt >= 3 
                ? '最大リトライ回数に達しました' 
                : '画像データの形式を確認してください'
              }
            </div>
            <div className="error-stats">
              エラー回数: {errorCount} | リトライ: {retryAttempt}/3
            </div>
            <button 
              className="retry-button"
              onClick={handleManualRetry}
              disabled={isLoading}
            >
              {isLoading ? '再試行中...' : '手動で再試行'}
            </button>
          </div>
        </div>
      );
    }

    return (
      <div className="live-video-container">
        {isLoading && (
          <div className="live-video-loading">
            <div className="loading-spinner"></div>
            <span>読み込み中...</span>
          </div>
        )}
        <img
          ref={imgRef}
          src={getCachedImageUrl(image)}
          alt="Live video with skeleton overlay"
          style={getImageStyles()}
          onLoad={handleImageLoad}
          onError={handleImageError}
          className={`live-video-image ${isLoading ? 'loading' : ''}`}
        />
        <div className="image-overlay-info">
          {imageSize.width > 0 && (
            <div className="image-info">
              {imageSize.width} × {imageSize.height}
            </div>
          )}
          {frameRate > 0 && (
            <div className="frame-rate">
              {frameRate} FPS
            </div>
          )}
          {lastUpdateTime && (
            <div className="last-update">
              更新: {lastUpdateTime.toLocaleTimeString()}
            </div>
          )}
        </div>
      </div>
    );
  };

  return (
    <div className="content-area live-video-area">
      <div className="area-header" style={{ marginBottom: '12px', paddingBottom: '8px' }}>
        <span style={{ fontSize: 'calc(max(20px, 20px * var(--font-scale)))', fontWeight: '600' }}>📹 ライブ映像</span>
        {isConnected && (
          <div className="connection-indicator connected" style={{ fontSize: 'calc(max(14px, 14px * var(--font-scale)))' }}>
            <span className="indicator-dot"></span>
            接続中
          </div>
        )}
        {!isConnected && (
          <div className="connection-indicator disconnected" style={{ fontSize: 'calc(max(14px, 14px * var(--font-scale)))' }}>
            <span className="indicator-dot"></span>
            切断
          </div>
        )}
      </div>
      <div className="area-content" ref={containerRef}>
        {renderContent()}
      </div>
    </div>
  );
};

export default LiveVideoDisplay;