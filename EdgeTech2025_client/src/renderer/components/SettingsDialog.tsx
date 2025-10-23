import React, { useState, useEffect } from 'react';
import {
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Button,
  TextField,
  FormControlLabel,
  Switch,
  Select,
  MenuItem,
  FormControl,
  InputLabel,
  Box,
  Typography,
  Alert,
  Divider,
  Grid,
  Chip
} from '@mui/material';
import { AppSettings } from '../../shared/types';

interface SettingsDialogProps {
  open: boolean;
  onClose: () => void;
  onSettingsChanged?: (settings: AppSettings) => void;
}

export const SettingsDialog: React.FC<SettingsDialogProps> = ({
  open,
  onClose,
  onSettingsChanged
}) => {
  const [settings, setSettings] = useState<AppSettings | null>(null);
  const [originalSettings, setOriginalSettings] = useState<AppSettings | null>(null);
  const [validationErrors, setValidationErrors] = useState<string[]>([]);
  const [isSaving, setIsSaving] = useState(false);
  const [hasChanges, setHasChanges] = useState(false);

  // 設定を読み込み
  useEffect(() => {
    if (open) {
      loadSettings();
    }
  }, [open]);

  // 変更検知
  useEffect(() => {
    if (settings && originalSettings) {
      const changed = JSON.stringify(settings) !== JSON.stringify(originalSettings);
      setHasChanges(changed);
    }
  }, [settings, originalSettings]);

  const loadSettings = async () => {
    try {
      const currentSettings = await window.electronAPI.getSettings();
      setSettings(currentSettings);
      setOriginalSettings({ ...currentSettings });
      setValidationErrors([]);
    } catch (error) {
      console.error('Failed to load settings:', error);
    }
  };

  const handleSettingChange = (key: keyof AppSettings, value: any) => {
    if (!settings) return;
    
    const newSettings = { ...settings, [key]: value };
    setSettings(newSettings);
    
    // リアルタイム検証
    validateSettings(newSettings);
  };

  const validateSettings = async (settingsToValidate: AppSettings) => {
    try {
      const validation = await window.electronAPI.validateSettings(settingsToValidate);
      setValidationErrors(validation.errors);
    } catch (error) {
      console.error('Validation failed:', error);
      setValidationErrors(['設定の検証に失敗しました']);
    }
  };

  const handleSave = async () => {
    if (!settings || validationErrors.length > 0) return;

    setIsSaving(true);
    try {
      const success = await window.electronAPI.saveSettings(settings);
      if (success) {
        setOriginalSettings({ ...settings });
        setHasChanges(false);
        onSettingsChanged?.(settings);
        onClose();
      } else {
        setValidationErrors(['設定の保存に失敗しました']);
      }
    } catch (error) {
      console.error('Failed to save settings:', error);
      setValidationErrors(['設定の保存中にエラーが発生しました']);
    } finally {
      setIsSaving(false);
    }
  };

  const handleReset = async () => {
    try {
      const defaultSettings = await window.electronAPI.getDefaultSettings();
      setSettings(defaultSettings);
      setValidationErrors([]);
    } catch (error) {
      console.error('Failed to load default settings:', error);
    }
  };

  const handleCancel = () => {
    if (originalSettings) {
      setSettings({ ...originalSettings });
      setHasChanges(false);
    }
    setValidationErrors([]);
    onClose();
  };

  if (!settings) {
    return null;
  }

  return (
    <Dialog 
      open={open} 
      onClose={handleCancel}
      maxWidth="md"
      fullWidth
      PaperProps={{
        sx: { minHeight: '600px' }
      }}
    >
      <DialogTitle>
        <Typography variant="h6">アプリケーション設定</Typography>
        {hasChanges && (
          <Chip 
            label="未保存の変更があります" 
            color="warning" 
            size="small" 
            sx={{ ml: 2 }}
          />
        )}
      </DialogTitle>
      
      <DialogContent dividers>
        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 3 }}>
          
          {/* エラー表示 */}
          {validationErrors.length > 0 && (
            <Alert severity="error">
              <Typography variant="subtitle2">設定エラー:</Typography>
              <ul style={{ margin: 0, paddingLeft: '20px' }}>
                {validationErrors.map((error, index) => (
                  <li key={index}>{error}</li>
                ))}
              </ul>
            </Alert>
          )}

          {/* WebSocket設定 */}
          <Box>
            <Typography variant="h6" gutterBottom>WebSocket接続設定</Typography>
            <Grid container spacing={2}>
              <Grid item xs={12}>
                <TextField
                  fullWidth
                  label="WebSocketサーバーURL"
                  value={settings.wsServerUrl}
                  onChange={(e) => handleSettingChange('wsServerUrl', e.target.value)}
                  placeholder="ws://localhost:9090"
                  helperText="ROSブリッジサーバーのWebSocket URL"
                />
              </Grid>
              <Grid item xs={12}>
                <FormControlLabel
                  control={
                    <Switch
                      checked={settings.autoConnect}
                      onChange={(e) => handleSettingChange('autoConnect', e.target.checked)}
                    />
                  }
                  label="起動時に自動接続"
                />
              </Grid>
            </Grid>
          </Box>

          <Divider />

          {/* ファイル監視設定 */}
          <Box>
            <Typography variant="h6" gutterBottom>ファイル監視設定</Typography>
            <Grid container spacing={2}>
              <Grid item xs={12}>
                <TextField
                  fullWidth
                  label="画像監視フォルダパス"
                  value={settings.imageWatchPath}
                  onChange={(e) => handleSettingChange('imageWatchPath', e.target.value)}
                  placeholder="./images"
                  helperText="ライブ映像画像が保存されるフォルダのパス"
                />
              </Grid>
              <Grid item xs={6}>
                <TextField
                  fullWidth
                  type="number"
                  label="更新間隔 (ミリ秒)"
                  value={settings.refreshInterval}
                  onChange={(e) => handleSettingChange('refreshInterval', parseInt(e.target.value))}
                  inputProps={{ min: 100, max: 10000, step: 100 }}
                  helperText="100-10000ms"
                />
              </Grid>
            </Grid>
          </Box>

          <Divider />

          {/* UI設定 */}
          <Box>
            <Typography variant="h6" gutterBottom>UI設定</Typography>
            <Grid container spacing={2}>
              <Grid item xs={6}>
                <FormControl fullWidth>
                  <InputLabel>テーマ</InputLabel>
                  <Select
                    value={settings.theme}
                    onChange={(e) => handleSettingChange('theme', e.target.value)}
                    label="テーマ"
                  >
                    <MenuItem value="dark">ダーク</MenuItem>
                    <MenuItem value="light">ライト</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item xs={6}>
                <FormControl fullWidth>
                  <InputLabel>言語</InputLabel>
                  <Select
                    value={settings.language}
                    onChange={(e) => handleSettingChange('language', e.target.value)}
                    label="言語"
                  >
                    <MenuItem value="ja">日本語</MenuItem>
                    <MenuItem value="en">English</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item xs={12}>
                <FormControlLabel
                  control={
                    <Switch
                      checked={settings.enableNotifications}
                      onChange={(e) => handleSettingChange('enableNotifications', e.target.checked)}
                    />
                  }
                  label="通知を有効にする"
                />
              </Grid>
            </Grid>
          </Box>

          <Divider />

          {/* データ管理設定 */}
          <Box>
            <Typography variant="h6" gutterBottom>データ管理設定</Typography>
            <Grid container spacing={2}>
              <Grid item xs={6}>
                <TextField
                  fullWidth
                  type="number"
                  label="履歴保持日数"
                  value={settings.maxHistoryDays}
                  onChange={(e) => handleSettingChange('maxHistoryDays', parseInt(e.target.value))}
                  inputProps={{ min: 1, max: 365 }}
                  helperText="1-365日"
                />
              </Grid>
            </Grid>
          </Box>

        </Box>
      </DialogContent>
      
      <DialogActions sx={{ p: 2, gap: 1 }}>
        <Button 
          onClick={handleReset}
          color="secondary"
        >
          デフォルトに戻す
        </Button>
        <Box sx={{ flexGrow: 1 }} />
        <Button 
          onClick={handleCancel}
          color="inherit"
        >
          キャンセル
        </Button>
        <Button 
          onClick={handleSave}
          variant="contained"
          disabled={!hasChanges || validationErrors.length > 0 || isSaving}
        >
          {isSaving ? '保存中...' : '保存'}
        </Button>
      </DialogActions>
    </Dialog>
  );
};