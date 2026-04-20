import React, { useState, useEffect } from 'react';
import {
  Dialog,
  DialogTitle,
  DialogContent,
  IconButton,
  Box,
  List,
  ListItem,
  ListItemText,
  Typography,
  Chip,
  Divider,
  Paper,
  Button,
  Tooltip
} from '@mui/material';
import {
  Close as CloseIcon,
  Refresh as RefreshIcon,
  Clear as ClearIcon,
  Send as SendIcon,
  CallReceived as ReceiveIcon,
  Error as ErrorIcon,
  CheckCircle as SuccessIcon
} from '@mui/icons-material';

interface CommunicationEntry {
  id: number;
  timestamp: Date;
  type: 'sent' | 'received' | 'error';
  direction: 'outgoing' | 'incoming';
  message: string;
  status?: 'success' | 'error' | 'pending';
  responseTime?: number;
  // ROSBridge通信情報
  topic?: string;
  msgType?: string;
  rosBridgeDirection?: 'publish' | 'subscribe';
}

interface CommunicationHistoryDialogProps {
  open: boolean;
  onClose: () => void;
  isConnected: boolean;
}

const CommunicationHistoryDialog: React.FC<CommunicationHistoryDialogProps> = ({
  open,
  onClose,
  isConnected
}) => {
  const [communicationHistory, setCommunicationHistory] = useState<CommunicationEntry[]>([]);

  // 実際の通信履歴データの取得
  useEffect(() => {
    if (open) {
      loadCommunicationHistory();
    }
  }, [open]);

  const loadCommunicationHistory = async () => {
    if (!window.electronAPI) {
      console.error('Electron API not available');
      return;
    }

    try {
      // コマンド履歴の取得
      const commandHistory = await window.electronAPI.getCommandHistory(50);

      // コマンド履歴を通信履歴形式に変換
      const entries: CommunicationEntry[] = commandHistory.map((cmd, index) => {
        // directionの判定：cmd.direction === 'subscribe' なら受信、それ以外は送信
        const isIncoming = cmd.direction === 'subscribe';
        
        return {
          id: index,
          timestamp: cmd.timestamp,
          type: cmd.success ? (isIncoming ? 'received' : 'sent') : 'error',
          direction: isIncoming ? 'incoming' as const : 'outgoing' as const,
          message: cmd.message ? JSON.stringify(cmd.message) : JSON.stringify({
            command: cmd.command,
            timestamp: cmd.timestamp
          }),
          status: cmd.success ? 'success' : 'error',
          responseTime: cmd.responseTime,
          // ROSBridge情報を追加
          topic: cmd.topic,
          msgType: cmd.type,
          rosBridgeDirection: cmd.direction
        };
      });

      setCommunicationHistory(entries);
    } catch (error) {
      console.error('Failed to load communication history:', error);
      setCommunicationHistory([]);
    }
  };

  const handleRefresh = () => {
    loadCommunicationHistory();
  };

  const handleClear = () => {
    setCommunicationHistory([]);
  };

  const getEntryIcon = (entry: CommunicationEntry) => {
    if (entry.direction === 'outgoing') {
      return entry.status === 'error' ? <ErrorIcon color="error" /> : <SendIcon color="primary" />;
    } else {
      return <ReceiveIcon color="success" />;
    }
  };

  const getEntryColor = (entry: CommunicationEntry) => {
    if (entry.direction === 'outgoing') {
      return entry.status === 'error' ? 'error' : 'primary';
    }
    return 'success';
  };

  const formatMessage = (message: string) => {
    try {
      const parsed = JSON.parse(message);
      return JSON.stringify(parsed, null, 2);
    } catch {
      return message;
    }
  };

  return (
    <Dialog
      open={open}
      onClose={onClose}
      maxWidth="md"
      fullWidth
      PaperProps={{
        sx: {
          backgroundColor: '#1e1e1e',
          color: '#ffffff',
          height: '80vh'
        }
      }}
    >
      <DialogTitle sx={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        pb: 1
      }}>
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
          通信履歴
          <Chip
            label={isConnected ? '接続中' : '未接続'}
            color={isConnected ? 'success' : 'error'}
            size="small"
          />
        </Box>
        <Box>
          <Tooltip title="更新">
            <IconButton onClick={handleRefresh} sx={{ color: '#ffffff', mr: 1 }}>
              <RefreshIcon />
            </IconButton>
          </Tooltip>
          <Tooltip title="履歴をクリア">
            <IconButton onClick={handleClear} sx={{ color: '#ffffff', mr: 1 }}>
              <ClearIcon />
            </IconButton>
          </Tooltip>
          <IconButton onClick={onClose} sx={{ color: '#ffffff' }}>
            <CloseIcon />
          </IconButton>
        </Box>
      </DialogTitle>

      <DialogContent sx={{ p: 0, display: 'flex', flexDirection: 'column' }}>
        {/* 統計情報 */}
        <Box sx={{ p: 2, backgroundColor: '#2a2a2a' }}>
          <Typography variant="body2" color="text.secondary">
            総通信数: {communicationHistory.length} |
            送信: {communicationHistory.filter(e => e.direction === 'outgoing').length} |
            受信: {communicationHistory.filter(e => e.direction === 'incoming').length} |
            エラー: {communicationHistory.filter(e => e.status === 'error').length}
          </Typography>
        </Box>

        {/* 通信履歴リスト */}
        <Box sx={{ flexGrow: 1, overflow: 'auto' }}>
          {communicationHistory.length === 0 ? (
            <Box sx={{ p: 4, textAlign: 'center' }}>
              <Typography color="text.secondary">
                通信履歴がありません
              </Typography>
              <Button
                variant="outlined"
                onClick={handleRefresh}
                sx={{ mt: 2 }}
                startIcon={<RefreshIcon />}
              >
                履歴を更新
              </Button>
            </Box>
          ) : (
            <List sx={{ p: 0 }}>
              {communicationHistory.map((entry, index) => (
                <React.Fragment key={entry.id}>
                  <ListItem
                    sx={{
                      flexDirection: 'column',
                      alignItems: 'stretch',
                      py: 2,
                      backgroundColor: entry.direction === 'outgoing' ? '#1a237e20' : '#2e7d3220'
                    }}
                  >
                    {/* ヘッダー行 */}
                    <Box sx={{ display: 'flex', alignItems: 'center', mb: 1, width: '100%' }}>
                      {getEntryIcon(entry)}
                      <Box sx={{ ml: 1, flexGrow: 1 }}>
                        <Typography variant="body2" sx={{ fontWeight: 'bold' }}>
                          {entry.direction === 'outgoing' ? '送信 (Publish)' : '受信 (Subscribe)'}
                        </Typography>
                        <Typography variant="caption" color="text.secondary">
                          {entry.timestamp.toLocaleString('ja-JP')}
                        </Typography>
                        {/* TOPIC と TYPE を表示 */}
                        {entry.topic && (
                          <Box sx={{ mt: 0.5 }}>
                            <Typography variant="caption" color="info.main" sx={{ mr: 2 }}>
                              TOPIC: {entry.topic}
                            </Typography>
                            {entry.msgType && (
                              <Typography variant="caption" color="warning.main">
                                TYPE: {entry.msgType}
                              </Typography>
                            )}
                          </Box>
                        )}
                      </Box>
                      <Box sx={{ display: 'flex', gap: 1 }}>
                        <Chip
                          label={entry.direction === 'outgoing' ? '送信' : '受信'}
                          color={getEntryColor(entry) as any}
                          size="small"
                        />
                        {entry.status === 'error' && (
                          <Chip label="エラー" color="error" size="small" />
                        )}
                        {entry.responseTime && (
                          <Chip
                            label={`${entry.responseTime}ms`}
                            variant="outlined"
                            size="small"
                          />
                        )}
                      </Box>
                    </Box>

                    {/* メッセージ内容 */}
                    <Paper
                      sx={{
                        p: 2,
                        backgroundColor: '#0a0a0a',
                        border: '1px solid #333',
                        width: '100%'
                      }}
                    >
                      <Typography
                        variant="caption"
                        sx={{ color: '#888', mb: 1, display: 'block' }}
                      >
                        メッセージ内容:
                      </Typography>
                      <Typography
                        variant="body2"
                        component="pre"
                        sx={{
                          fontFamily: 'monospace',
                          fontSize: '0.75rem',
                          whiteSpace: 'pre-wrap',
                          wordBreak: 'break-all',
                          margin: 0,
                          color: entry.status === 'error' ? '#f44336' : '#ffffff'
                        }}
                      >
                        {formatMessage(entry.message)}
                      </Typography>
                    </Paper>
                  </ListItem>
                  {index < communicationHistory.length - 1 && <Divider />}
                </React.Fragment>
              ))}
            </List>
          )}
        </Box>
      </DialogContent>
    </Dialog>
  );
};

export default CommunicationHistoryDialog;