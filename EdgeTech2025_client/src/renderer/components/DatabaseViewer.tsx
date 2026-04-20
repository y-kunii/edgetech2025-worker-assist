import React, { useState, useEffect } from 'react';
import {
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  Button,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  TextField,
  Box,
  Typography,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  Alert,
  CircularProgress,
  Chip,
  IconButton,
  Tooltip
} from '@mui/material';
import {
  Refresh,
  Storage,
  Info
} from '@mui/icons-material';

interface DatabaseViewerProps {
  open: boolean;
  onClose: () => void;
}

type TableType = 'status_history' | 'command_history' | 'work_statistics';

interface DatabaseStats {
  statusRecords: number;
  commandRecords: number;
  statisticsRecords: number;
}

export const DatabaseViewer: React.FC<DatabaseViewerProps> = ({ open, onClose }) => {
  const [selectedTable, setSelectedTable] = useState<TableType>('status_history');
  const [recordLimit, setRecordLimit] = useState<number>(50);
  const [tableData, setTableData] = useState<any[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [dbStats, setDbStats] = useState<DatabaseStats | null>(null);

  // データベース統計を取得
  const loadDatabaseStats = async () => {
    try {
      const stats = await window.electronAPI.getDatabaseStats();
      setDbStats(stats);
    } catch (error) {
      console.error('Failed to load database stats:', error);
    }
  };

  // テーブルデータを取得
  const loadTableData = async () => {
    setIsLoading(true);
    setError(null);
    
    try {
      let data: any[] = [];
      
      switch (selectedTable) {
        case 'status_history':
          data = await window.electronAPI.getDatabaseStatusHistory(recordLimit);
          break;
        case 'command_history':
          data = await window.electronAPI.getDatabaseCommandHistory(recordLimit);
          break;
        case 'work_statistics':
          const statsData = await window.electronAPI.getWorkStatistics();
          // WorkStatisticsは配列として返されるはず
          data = Array.isArray(statsData) ? statsData : [];
          break;
      }
      
      setTableData(data);
    } catch (error) {
      console.error('Failed to load table data:', error);
      setError('データの取得に失敗しました');
    } finally {
      setIsLoading(false);
    }
  };

  // ダイアログが開いたら初期データを読み込み
  useEffect(() => {
    if (open) {
      loadDatabaseStats();
      loadTableData();
    }
  }, [open, selectedTable, recordLimit]);

  // 手動更新
  const handleRefresh = () => {
    loadDatabaseStats();
    loadTableData();
  };

  // テーブルのカラム定義
  const getTableColumns = (): string[] => {
    switch (selectedTable) {
      case 'status_history':
        return ['id', 'timestamp', 'worker_status', 'space_status', 'robot_state', 'robot_grip', 'tool_delivery', 'demo_status', 'created_at'];
      case 'command_history':
        return ['id', 'timestamp', 'command', 'success', 'created_at'];
      case 'work_statistics':
        return ['id', 'date', 'screw_count', 'blocks_count', 'survey_count', 'total_work_time', 'average_work_time', 'created_at'];
      default:
        return [];
    }
  };

  // テーブル名の日本語表示
  const getTableDisplayName = (table: TableType): string => {
    switch (table) {
      case 'status_history':
        return '状態履歴';
      case 'command_history':
        return 'コマンド履歴';
      case 'work_statistics':
        return '作業統計';
      default:
        return table;
    }
  };

  // セル値のフォーマット
  const formatCellValue = (value: any, column: string): React.ReactNode => {
    if (value === null || value === undefined) {
      return <Typography variant="body2" color="text.secondary">-</Typography>;
    }

    // Boolean値
    if (typeof value === 'boolean') {
      return (
        <Chip
          label={value ? '成功' : '失敗'}
          color={value ? 'success' : 'error'}
          size="small"
        />
      );
    }

    // タイムスタンプのフォーマット
    if (column === 'timestamp' && typeof value === 'string' && value.length === 14) {
      const year = value.substring(0, 4);
      const month = value.substring(4, 6);
      const day = value.substring(6, 8);
      const hour = value.substring(8, 10);
      const minute = value.substring(10, 12);
      const second = value.substring(12, 14);
      return `${year}-${month}-${day} ${hour}:${minute}:${second}`;
    }

    // created_atのフォーマット
    if (column === 'created_at' || column === 'updated_at') {
      try {
        const date = new Date(value);
        return date.toLocaleString('ja-JP');
      } catch {
        return value;
      }
    }

    return value.toString();
  };

  const columns = getTableColumns();

  return (
    <Dialog
      open={open}
      onClose={onClose}
      maxWidth="xl"
      fullWidth
      PaperProps={{
        sx: { height: '90vh' }
      }}
    >
      <DialogTitle>
        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
          <Storage />
          <Typography variant="h6">データベース内容確認</Typography>
        </Box>
      </DialogTitle>

      <DialogContent dividers>
        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 3 }}>
          
          {/* データベース統計 */}
          {dbStats && (
            <Alert severity="info" icon={<Info />}>
              <Typography variant="subtitle2" sx={{ mb: 1 }}>
                データベース統計
              </Typography>
              <Box sx={{ display: 'flex', gap: 3 }}>
                <Typography variant="body2">
                  状態履歴: <strong>{dbStats.statusRecords.toLocaleString()}</strong> 件
                </Typography>
                <Typography variant="body2">
                  コマンド履歴: <strong>{dbStats.commandRecords.toLocaleString()}</strong> 件
                </Typography>
                <Typography variant="body2">
                  作業統計: <strong>{dbStats.statisticsRecords.toLocaleString()}</strong> 件
                </Typography>
              </Box>
            </Alert>
          )}

          {/* 表示設定 */}
          <Paper sx={{ p: 2 }}>
            <Box sx={{ display: 'flex', gap: 2, alignItems: 'center', flexWrap: 'wrap' }}>
              <FormControl sx={{ minWidth: 200 }}>
                <InputLabel>テーブル選択</InputLabel>
                <Select
                  value={selectedTable}
                  label="テーブル選択"
                  onChange={(e) => setSelectedTable(e.target.value as TableType)}
                >
                  <MenuItem value="status_history">状態履歴</MenuItem>
                  <MenuItem value="command_history">コマンド履歴</MenuItem>
                  <MenuItem value="work_statistics">作業統計</MenuItem>
                </Select>
              </FormControl>

              {selectedTable !== 'work_statistics' && (
                <TextField
                  type="number"
                  label="表示件数"
                  value={recordLimit}
                  onChange={(e) => setRecordLimit(parseInt(e.target.value) || 50)}
                  inputProps={{ min: 1, max: 1000 }}
                  sx={{ width: 150 }}
                />
              )}

              <Tooltip title="更新">
                <IconButton onClick={handleRefresh} color="primary">
                  <Refresh />
                </IconButton>
              </Tooltip>

              <Typography variant="body2" color="text.secondary" sx={{ ml: 'auto' }}>
                {tableData.length} 件表示中
              </Typography>
            </Box>
          </Paper>

          {/* エラー表示 */}
          {error && (
            <Alert severity="error" onClose={() => setError(null)}>
              {error}
            </Alert>
          )}

          {/* テーブルデータ表示 */}
          <Paper sx={{ flexGrow: 1, overflow: 'hidden', display: 'flex', flexDirection: 'column' }}>
            {isLoading ? (
              <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', p: 4 }}>
                <CircularProgress />
              </Box>
            ) : (
              <TableContainer sx={{ flexGrow: 1, maxHeight: 'calc(90vh - 400px)' }}>
                <Table stickyHeader size="small">
                  <TableHead>
                    <TableRow>
                      {columns.map((column) => (
                        <TableCell key={column} sx={{ fontWeight: 'bold', backgroundColor: 'action.hover' }}>
                          {column}
                        </TableCell>
                      ))}
                    </TableRow>
                  </TableHead>
                  <TableBody>
                    {tableData.length === 0 ? (
                      <TableRow>
                        <TableCell colSpan={columns.length} align="center">
                          <Typography variant="body2" color="text.secondary" sx={{ py: 4 }}>
                            データがありません
                          </Typography>
                        </TableCell>
                      </TableRow>
                    ) : (
                      tableData.map((row, index) => (
                        <TableRow key={row.id || index} hover>
                          {columns.map((column) => (
                            <TableCell key={column}>
                              {formatCellValue(row[column], column)}
                            </TableCell>
                          ))}
                        </TableRow>
                      ))
                    )}
                  </TableBody>
                </Table>
              </TableContainer>
            )}
          </Paper>

          {/* 注釈 */}
          <Typography variant="caption" color="text.secondary">
            ※ 最新のレコードから順に表示されます
          </Typography>
        </Box>
      </DialogContent>

      <DialogActions>
        <Button onClick={onClose}>閉じる</Button>
      </DialogActions>
    </Dialog>
  );
};

export default DatabaseViewer;
