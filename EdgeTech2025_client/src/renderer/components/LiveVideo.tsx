import React, { useState, useEffect, useCallback } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Alert,
  CircularProgress,
  IconButton,
  Tooltip,
  Chip
} from '@mui/material';
import {
  Refresh,
  FolderOpen,
  Image as ImageIcon,
  Error as ErrorIcon,
  CheckCircle
} from '@mui/icons-material';

interface LiveVideoProps {
  title?: string;
  height?: number;
  autoRefresh?: boolean;
  showControls?: boolean;
}

interface ImageState {
  imagePath: string | null;
  lastUpdated: Date | null;
  isLoading: boolean;
  error: string | null;
  isWatcherActive: boolean;
}

const LiveVideo: React.FC<LiveVideoProps> = ({
  title = 'ライブ映像',
  height = 300,
  autoRefresh = true,
  showControls = true
}) => {
  const [imageState, setImageState] = useState<ImageState>({
    imagePath: null,
    lastUpdated: null,
    isLoading: true,
    error: null,
    isWatcherActive: false
  });

  // 画像の更新処理
  const updateImage = useCallback(async () => {
    try {
      setImageState(prev => ({ ...prev, isLoading: true, error: null }));
      
      if (window.electronAPI) {
        const latestImage = await window.electronAPI.getLatestImage();
        const isActive = await window.electronAPI.isImageWatchActive();
        
        setImageState(prev => ({
          ...prev,
          imagePath: latestImage,
          lastUpdated: latestImage ? new Date() : null,
          isLoading: false,
          isWatcherActive: isActive
        }));
      }
    } catch (error) {
      console.error('Failed to update image:', error);
      setImageState(prev => ({
        ...prev,
        error: 'Failed to load image',
        isLoading: false
      }));
    }
  }, []);

  // ファイル監視の開始
  const startImageWatch = useCallback(async () => {
    try {
      if (window.electronAPI) {
        const success = await window.electronAPI.startImageWatch('./images');
        if (success) {
          setImageState(prev => ({ ...prev, isWatcherActive: true }));
          await updateImage();
        } else {
          setImageState(prev => ({ 
            ...prev, 
            error: 'Failed to start image watcher',
            isWatcherActive: false 
          }));
        }
      }
    } catch (error) {
      console.error('Failed to start image watcher:', error);
      setImageState(prev => ({ 
        ...prev, 
        error: 'Failed to start image watcher',
        isWatcherActive: false 
      }));
    }
  }, [updateImage]);

  // 手動リフレッシュ
  const handleManualRefresh = useCallback(async () => {
    await updateImage();
  }, [updateImage]);

  // 画像パスの変換（file://プロトコルを追加）
  const getImageUrl = useCallback((imagePath: string | null): string | null => {
    if (!imagePath) return null;
    
    // 既にfile://で始まっている場合はそのまま返す
    if (imagePath.startsWith('file://')) {
      return imagePath;
    }
    
    // Windowsパスの処理
    if (imagePath.includes('\\')) {
      return `file:///${imagePath.replace(/\\/g, '/')}`;
    }
    
    // Unix系パスの処理
    return `file://${imagePath}`;
  }, []);

  // 初期化とイベントリスナーの設定
  useEffect(() => {
    let mounted = true;

    const initializeImageWatch = async () => {
      if (!window.electronAPI) {
        setImageState(prev => ({ 
          ...prev, 
          error: 'Electron API not available',
          isLoading: false 
        }));
        return;
      }

      // 初期画像の取得
      await updateImage();

      // 自動リフレッシュが有効な場合、ファイル監視を開始
      if (autoRefresh) {
        await startImageWatch();
      }
    };

    // 画像更新イベントリスナー
    const handleImageUpdate = (imagePath: string) => {
      if (mounted) {
        console.log('Image updated:', imagePath);
        setImageState(prev => ({
          ...prev,
          imagePath,
          lastUpdated: new Date(),
          error: null
        }));
      }
    };

    // ファイル監視エラーイベントリスナー
    const handleWatcherError = (error: string) => {
      if (mounted) {
        console.error('File watcher error:', error);
        setImageState(prev => ({
          ...prev,
          error: `File watcher error: ${error}`,
          isWatcherActive: false
        }));
      }
    };

    // ファイル監視開始イベントリスナー
    const handleWatcherStarted = (data: any) => {
      if (mounted) {
        console.log('File watcher started:', data);
        setImageState(prev => ({
          ...prev,
          isWatcherActive: true,
          error: null
        }));
      }
    };

    // ファイル監視停止イベントリスナー
    const handleWatcherStopped = () => {
      if (mounted) {
        console.log('File watcher stopped');
        setImageState(prev => ({
          ...prev,
          isWatcherActive: false
        }));
      }
    };

    // イベントリスナーの登録
    if (window.electronAPI) {
      window.electronAPI.onImageUpdate(handleImageUpdate);
      window.electronAPI.onFileWatcherError(handleWatcherError);
      window.electronAPI.onFileWatcherStarted(handleWatcherStarted);
      window.electronAPI.onFileWatcherStopped(handleWatcherStopped);
    }

    initializeImageWatch();

    return () => {
      mounted = false;
    };
  }, [autoRefresh, startImageWatch, updateImage]);

  // 画像の読み込みエラーハンドリング
  const handleImageError = useCallback(() => {
    setImageState(prev => ({
      ...prev,
      error: 'Failed to load image file'
    }));
  }, []);

  // 画像の読み込み成功ハンドリング
  const handleImageLoad = useCallback(() => {
    setImageState(prev => ({
      ...prev,
      error: null
    }));
  }, []);

  const imageUrl = getImageUrl(imageState.imagePath);

  return (
    <Card sx={{ height: height, display: 'flex', flexDirection: 'column' }}>
      <CardContent sx={{ flexGrow: 1, display: 'flex', flexDirection: 'column' }}>
        <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 2 }}>
          <Typography variant="h6" component="div">
            {title}
          </Typography>
          
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            {/* 監視状態インジケーター */}
            <Chip
              icon={imageState.isWatcherActive ? <CheckCircle /> : <ErrorIcon />}
              label={imageState.isWatcherActive ? '監視中' : '停止中'}
              color={imageState.isWatcherActive ? 'success' : 'default'}
              size="small"
            />
            
            {/* 手動リフレッシュボタン */}
            {showControls && (
              <Tooltip title="手動更新">
                <IconButton 
                  onClick={handleManualRefresh}
                  disabled={imageState.isLoading}
                  size="small"
                >
                  <Refresh />
                </IconButton>
              </Tooltip>
            )}
          </Box>
        </Box>

        {/* 最終更新時刻 */}
        {imageState.lastUpdated && (
          <Typography variant="caption" color="text.secondary" sx={{ mb: 1, display: 'block' }}>
            最終更新: {imageState.lastUpdated.toLocaleString('ja-JP')}
          </Typography>
        )}

        {/* 画像表示エリア */}
        <Box
          sx={{
            flexGrow: 1,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            backgroundColor: '#2a2a2a',
            borderRadius: 1,
            position: 'relative',
            overflow: 'hidden',
            minHeight: 150
          }}
        >
          {imageState.isLoading && (
            <Box sx={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 2 }}>
              <CircularProgress />
              <Typography variant="body2" color="text.secondary">
                画像を読み込み中...
              </Typography>
            </Box>
          )}

          {imageState.error && !imageState.isLoading && (
            <Box sx={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 2 }}>
              <ErrorIcon sx={{ fontSize: 48, color: 'error.main' }} />
              <Typography variant="body2" color="error" textAlign="center">
                {imageState.error}
              </Typography>
            </Box>
          )}

          {!imageState.imagePath && !imageState.isLoading && !imageState.error && (
            <Box sx={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 2 }}>
              <ImageIcon sx={{ fontSize: 48, color: 'text.secondary' }} />
              <Typography variant="body2" color="text.secondary" textAlign="center">
                画像ファイルが見つかりません
              </Typography>
            </Box>
          )}

          {imageUrl && !imageState.isLoading && !imageState.error && (
            <img
              src={imageUrl}
              alt="Live video feed"
              style={{
                maxWidth: '100%',
                maxHeight: '100%',
                objectFit: 'contain'
              }}
              onError={handleImageError}
              onLoad={handleImageLoad}
            />
          )}
        </Box>

        {/* エラー表示 */}
        {imageState.error && (
          <Alert severity="error" sx={{ mt: 2 }}>
            {imageState.error}
          </Alert>
        )}
      </CardContent>
    </Card>
  );
};

export default LiveVideo;