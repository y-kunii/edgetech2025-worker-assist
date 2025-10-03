import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { RobotControlPanel } from '../renderer/components/RobotControlPanel';

// Mock electron module
jest.mock('electron', () => ({
  ipcRenderer: {
    invoke: jest.fn(),
    on: jest.fn(),
    removeListener: jest.fn()
  }
}));

describe('RobotControlPanel', () => {
  const mockOnToggleManualMode = jest.fn();
  const { ipcRenderer } = require('electron');

  beforeEach(() => {
    jest.clearAllMocks();
    
    // Mock IPC responses
    (ipcRenderer.invoke as jest.Mock).mockImplementation((channel: string) => {
      switch (channel) {
        case 'robot-command-get-logs':
          return Promise.resolve({ success: true, logs: [] });
        case 'robot-command-get-statistics':
          return Promise.resolve({
            success: true,
            statistics: {
              total: 0,
              successful: 0,
              failed: 0,
              pending: 0,
              successRate: 0
            }
          });
        default:
          return Promise.resolve({ success: true });
      }
    });
  });

  it('should render robot control panel', async () => {
    render(
      <RobotControlPanel
        isConnected={true}
        onToggleManualMode={mockOnToggleManualMode}
      />
    );

    expect(screen.getByText('🤖 ロボット制御パネル')).toBeInTheDocument();
    expect(screen.getByText('🟢 接続中')).toBeInTheDocument();
  });

  it('should show disconnected status when not connected', () => {
    render(
      <RobotControlPanel
        isConnected={false}
        onToggleManualMode={mockOnToggleManualMode}
      />
    );

    expect(screen.getByText('🔴 切断中')).toBeInTheDocument();
  });

  it('should toggle manual mode', async () => {
    render(
      <RobotControlPanel
        isConnected={true}
        onToggleManualMode={mockOnToggleManualMode}
      />
    );

    const toggleSwitch = screen.getByRole('checkbox');
    expect(toggleSwitch).not.toBeChecked();

    fireEvent.click(toggleSwitch);

    expect(mockOnToggleManualMode).toHaveBeenCalledWith(true);
  });

  it('should show manual commands when in manual mode', async () => {
    render(
      <RobotControlPanel
        isConnected={true}
        onToggleManualMode={mockOnToggleManualMode}
      />
    );

    const toggleSwitch = screen.getByRole('checkbox');
    fireEvent.click(toggleSwitch);

    await waitFor(() => {
      expect(screen.getByText('🔧 工具受け渡し指示')).toBeInTheDocument();
      expect(screen.getByText('➡️ 次タスク指示')).toBeInTheDocument();
    });
  });

  it('should disable manual commands when disconnected', async () => {
    render(
      <RobotControlPanel
        isConnected={false}
        onToggleManualMode={mockOnToggleManualMode}
      />
    );

    const toggleSwitch = screen.getByRole('checkbox');
    fireEvent.click(toggleSwitch);

    await waitFor(() => {
      const toolHandoverButton = screen.getByText('🔧 工具受け渡し指示');
      const nextTaskButton = screen.getByText('➡️ 次タスク指示');
      
      expect(toolHandoverButton).toBeDisabled();
      expect(nextTaskButton).toBeDisabled();
    });
  });

  it('should send manual command when button is clicked', async () => {
    (ipcRenderer.invoke as jest.Mock).mockResolvedValueOnce({ success: true });

    render(
      <RobotControlPanel
        isConnected={true}
        onToggleManualMode={mockOnToggleManualMode}
      />
    );

    // Enable manual mode
    const toggleSwitch = screen.getByRole('checkbox');
    fireEvent.click(toggleSwitch);

    await waitFor(() => {
      const toolHandoverButton = screen.getByText('🔧 工具受け渡し指示');
      fireEvent.click(toolHandoverButton);
    });

    expect(ipcRenderer.invoke).toHaveBeenCalledWith(
      'robot-command-send',
      'tool_handover',
      expect.objectContaining({
        manual: true,
        timestamp: expect.any(String)
      })
    );
  });

  it('should display error when command fails', async () => {
    (ipcRenderer.invoke as jest.Mock).mockResolvedValueOnce({ 
      success: false, 
      error: 'Command failed' 
    });

    render(
      <RobotControlPanel
        isConnected={true}
        onToggleManualMode={mockOnToggleManualMode}
      />
    );

    // Enable manual mode
    const toggleSwitch = screen.getByRole('checkbox');
    fireEvent.click(toggleSwitch);

    await waitFor(() => {
      const toolHandoverButton = screen.getByText('🔧 工具受け渡し指示');
      fireEvent.click(toolHandoverButton);
    });

    await waitFor(() => {
      expect(screen.getByText('Command failed')).toBeInTheDocument();
    });
  });

  it('should display statistics', async () => {
    (ipcRenderer.invoke as jest.Mock).mockImplementation((channel: string) => {
      if (channel === 'robot-command-get-statistics') {
        return Promise.resolve({
          success: true,
          statistics: {
            total: 10,
            successful: 8,
            failed: 2,
            pending: 0,
            successRate: 80
          }
        });
      }
      return Promise.resolve({ success: true, logs: [] });
    });

    render(
      <RobotControlPanel
        isConnected={true}
        onToggleManualMode={mockOnToggleManualMode}
      />
    );

    await waitFor(() => {
      expect(screen.getByText('10')).toBeInTheDocument(); // total
      expect(screen.getByText('8')).toBeInTheDocument();  // successful
      expect(screen.getByText('2')).toBeInTheDocument();  // failed
      expect(screen.getByText('80.0%')).toBeInTheDocument(); // success rate
    });
  });

  it('should clear logs when clear button is clicked', async () => {
    (ipcRenderer.invoke as jest.Mock).mockResolvedValueOnce({ success: true });

    render(
      <RobotControlPanel
        isConnected={true}
        onToggleManualMode={mockOnToggleManualMode}
      />
    );

    const clearButton = screen.getByText('🗑️ ログクリア');
    fireEvent.click(clearButton);

    await waitFor(() => {
      expect(ipcRenderer.invoke).toHaveBeenCalledWith('robot-command-clear-logs');
    });
  });
});