import React, { useState, useCallback } from 'react';
import {
  Card,
  CardContent,
  Typography,
  Box,
  Button,
  ButtonGroup,
  Alert,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  Chip,
  Divider,
  CircularProgress,
  Tooltip
} from '@mui/material';
import {
  Send,
  History,
  CheckCircle,
  Error,
  Schedule,
  PanTool,
  Handshake,
  Stop
} from '@mui/icons-material';
import { CommandData } from '../../shared/types';

export interface ControlPanelProps {
  onCommandSend: (command: CommandData) => Promise<boolean>;
  isConnected: boolean;
  commandHistory?: CommandHistoryItem[];
  maxHistoryItems?: number;
}

export interface CommandHistoryItem {
  id: number;
  command: 'tool_handover' | 'tool_collection' | 'wait';
  timestamp: Date;
  success: boolean;
}

// コマンド設定
const commandConfig = {
  'tool_handover': {
    label: 'ツール受け渡し',
    icon: <Handshake />,
    color: 'primary' as const,
    description: 'ロボットにツールを受け渡します'
  },
  'tool_collection': {
    label: 'ツール回収',
    icon: <PanTool />,
    color: 'secondary' as const,
    description: 'ロボットからツールを回収します'
  },
  'wait': {
    label: '待機',
    icon: <Stop />,
    color: 'warning' as const,
    description: 'ロボットを待機状態にします'
  }
};

const ControlPanel: React.FC<ControlPanelProps> = ({
  onCommandSend,
  isConnected,
  commandHistory = [],
  maxHistoryItems = 5
}) => {
  const [selectedCommand, setSelectedCommand] = useState<'tool_handover' | 'tool_collection' | 'wait' | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [lastResult, setLastResult] = useState<{ success: boolean; message: string } | null>(null);

  // コマンド送信処理
  const handleCommandSend = useCallback(async () => {
    if (!selectedCommand || !isConnected || isLoading) return;

    setIsLoading(true);
    setLastResult(null);

    try {
      const commandData: CommandData = {
        command: selectedCommand,
        timestamp: new Date().toISOString().replace(/[-:T.]/g, '').slice(0, 14)
      };

      const success = await onCommandSend(commandData);
      
      setLastResult({
        success,
        message: success 
          ? `${commandConfig[selectedCommand].label}コマンドを送信しました`
          : `${commandConfig[selectedCommand].label}コマンドの送信に失敗しました`
      });

      if (success) {
        setSelectedCommand(null);
      }
    } catch (error) {
      console.error('Command send error:', error);
      setLastResult({
        success: false,
        message: 'コマンド送信中にエラーが発生しました'
      });
    } finally {
      setIsLoading(false);
    }
  }, [selectedCommand, isConnected, isLoading, onCommandSend]);

  // コマンド選択処理
  const handleCommandSelect = useCallback((command: 'tool_handover' | 'tool_collection' | 'wait') => {
    if (isLoading) return;
    setSelectedCommand(selectedCommand === command ? null : command);
    setLastResult(null);
  }, [selectedCommand, isLoading]);

  // 履歴表示用のアイテム
  const displayHistory = commandHistory.slice(0, maxHistoryItems);

  return (
    <Card>
      <CardContent>
        <Typography variant="h6" component="div" sx={{ mb: 2 }}>
          制御パネル
        </Typography>

        {/* 接続状態表示 */}
        <Box sx={{ mb: 2 }}>
          <Chip
            icon={isConnected ? <CheckCircle /> : <Error />}
            label={isConnected ? 'WebSocket接続中' : 'WebSocket未接続'}
            color={isConnected ? 'success' : 'error'}
            size="small"
          />
        </Box>

        {/* マテハン指示選択UI */}
        <Box sx={{ mb: 3 }}>
          <Typography variant="subtitle2" sx={{ mb: 1 }}>
            マテハン指示選択
          </Typography>
          
          <ButtonGroup
            orientation="vertical"
            variant="outlined"
            fullWidth
            disabled={!isConnected || isLoading}
          >
            {Object.entries(commandConfig).map(([command, config]) => (
              <Tooltip key={command} title={config.description} placement="right">
                <Button
                  startIcon={config.icon}
                  onClick={() => handleCommandSelect(command as any)}
                  variant={selectedCommand === command ? 'contained' : 'outlined'}
                  color={selectedCommand === command ? config.color : 'inherit'}
                  sx={{
                    justifyContent: 'flex-start',
                    textAlign: 'left',
                    py: 1.5,
                    '&:not(:last-child)': {
                      borderBottom: 'none'
                    }
                  }}
                >
                  {config.label}
                </Button>
              </Tooltip>
            ))}
          </ButtonGroup>
        </Box>

        {/* 送信ボタン */}
        <Box sx={{ mb: 3 }}>
          <Button
            variant="contained"
            color="primary"
            fullWidth
            startIcon={isLoading ? <CircularProgress size={20} /> : <Send />}
            onClick={handleCommandSend}
            disabled={!selectedCommand || !isConnected || isLoading}
            sx={{ py: 1.5 }}
          >
            {isLoading ? '送信中...' : 'コマンド送信'}
          </Button>
        </Box>

        {/* 送信結果フィードバック */}
        {lastResult && (
          <Alert 
            severity={lastResult.success ? 'success' : 'error'} 
            sx={{ mb: 2 }}
            onClose={() => setLastResult(null)}
          >
            {lastResult.message}
          </Alert>
        )}

        {/* 送信履歴表示 */}
        <Box>
          <Typography variant="subtitle2" sx={{ mb: 1, display: 'flex', alignItems: 'center' }}>
            <History sx={{ mr: 1, fontSize: '1rem' }} />
            送信履歴
          </Typography>
          
          {displayHistory.length === 0 ? (
            <Typography variant="body2" color="text.secondary" sx={{ textAlign: 'center', py: 2 }}>
              送信履歴がありません
            </Typography>
          ) : (
            <List dense sx={{ maxHeight: 200, overflow: 'auto' }}>
              {displayHistory.map((item, index) => (
                <React.Fragment key={item.id}>
                  <ListItem sx={{ px: 0 }}>
                    <ListItemIcon sx={{ minWidth: 36 }}>
                      {item.success ? (
                        <CheckCircle color="success" fontSize="small" />
                      ) : (
                        <Error color="error" fontSize="small" />
                      )}
                    </ListItemIcon>
                    <ListItemText
                      primary={
                        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                          <Typography variant="body2">
                            {commandConfig[item.command].label}
                          </Typography>
                          <Chip
                            label={item.success ? '成功' : '失敗'}
                            color={item.success ? 'success' : 'error'}
                            size="small"
                          />
                        </Box>
                      }
                      secondary={
                        <Typography variant="caption" color="text.secondary">
                          {item.timestamp.toLocaleString('ja-JP')}
                        </Typography>
                      }
                    />
                  </ListItem>
                  {index < displayHistory.length - 1 && <Divider />}
                </React.Fragment>
              ))}
            </List>
          )}
        </Box>

        {/* 選択中のコマンド情報 */}
        {selectedCommand && (
          <Box sx={{ mt: 2, p: 2, backgroundColor: '#2a2a2a', borderRadius: 1 }}>
            <Typography variant="body2" color="text.secondary">
              選択中: <strong>{commandConfig[selectedCommand].label}</strong>
            </Typography>
            <Typography variant="caption" color="text.secondary">
              {commandConfig[selectedCommand].description}
            </Typography>
          </Box>
        )}
      </CardContent>
    </Card>
  );
};

export default ControlPanel;