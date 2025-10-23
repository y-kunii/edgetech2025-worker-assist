import React from 'react';
import {
  Dialog,
  DialogTitle,
  DialogContent,
  IconButton,
  Box
} from '@mui/material';
import { Close as CloseIcon } from '@mui/icons-material';
import ControlPanel, { CommandHistoryItem } from './ControlPanel';
import { CommandData } from '../../shared/types';

interface ControlPanelDialogProps {
  open: boolean;
  onClose: () => void;
  onCommandSend: (command: CommandData) => Promise<boolean>;
  isConnected: boolean;
  commandHistory: CommandHistoryItem[];
}

const ControlPanelDialog: React.FC<ControlPanelDialogProps> = ({
  open,
  onClose,
  onCommandSend,
  isConnected,
  commandHistory
}) => {
  return (
    <Dialog
      open={open}
      onClose={onClose}
      maxWidth="sm"
      fullWidth
      PaperProps={{
        sx: {
          backgroundColor: '#1e1e1e',
          color: '#ffffff'
        }
      }}
    >
      <DialogTitle sx={{ 
        display: 'flex', 
        justifyContent: 'space-between', 
        alignItems: 'center',
        pb: 1
      }}>
        制御パネル
        <IconButton
          onClick={onClose}
          sx={{ color: '#ffffff' }}
        >
          <CloseIcon />
        </IconButton>
      </DialogTitle>
      <DialogContent sx={{ p: 2 }}>
        <Box sx={{ height: 400 }}>
          <ControlPanel
            onCommandSend={onCommandSend}
            isConnected={isConnected}
            commandHistory={commandHistory}
            maxHistoryItems={10}
          />
        </Box>
      </DialogContent>
    </Dialog>
  );
};

export default ControlPanelDialog;