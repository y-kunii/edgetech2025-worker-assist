import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import ConnectionMonitor from '../renderer/components/ConnectionMonitor';
import { WorkDataStore } from '../stores/WorkDataStore';

// Mock localStorage
const localStorageMock = {
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
};
Object.defineProperty(window, 'localStorage', {
  value: localStorageMock
});

describe('ConnectionMonitor', () => {
  let workDataStore: WorkDataStore;
  let mockOnClose: jest.Mock;

  beforeEach(() => {
    workDataStore = new WorkDataStore();
    mockOnClose = jest.fn();
    localStorageMock.getItem.mockClear();
    localStorageMock.setItem.mockClear();
    jest.clearAllTimers();
    jest.useFakeTimers();
  });

  afterEach(() => {
    workDataStore.destroy();
    jest.runOnlyPendingTimers();
    jest.useRealTimers();
  });

  it('renders connection monitor modal when visible', () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    expect(screen.getByText('🔗 接続設定・監視')).toBeInTheDocument();
    expect(screen.getAllByText('接続状態')).toHaveLength(2); // Header and status card
    expect(screen.getByText('接続設定')).toBeInTheDocument();
  });

  it('does not render when not visible', () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={false}
        onClose={mockOnClose}
      />
    );

    expect(screen.queryByText('🔗 接続設定・監視')).not.toBeInTheDocument();
  });

  it('displays connection status information', () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    // Check for status cards
    expect(screen.getByText('遅延')).toBeInTheDocument();
    expect(screen.getByText('データレート')).toBeInTheDocument();
    expect(screen.getByText('稼働時間')).toBeInTheDocument();
  });

  it('displays default connection settings', () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const urlInput = screen.getByLabelText('WebSocket URL') as HTMLInputElement;
    const reconnectInput = screen.getByLabelText('再接続間隔 (ms)') as HTMLInputElement;
    const maxAttemptsInput = screen.getByLabelText('最大再接続試行回数') as HTMLInputElement;
    const heartbeatInput = screen.getByLabelText('ハートビート間隔 (ms)') as HTMLInputElement;

    expect(urlInput.value).toBe('ws://localhost:3001');
    expect(reconnectInput.value).toBe('5000');
    expect(maxAttemptsInput.value).toBe('10');
    expect(heartbeatInput.value).toBe('30000');
  });

  it('allows changing WebSocket URL', async () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const urlInput = screen.getByLabelText('WebSocket URL');
    fireEvent.change(urlInput, { target: { value: 'ws://localhost:8080' } });

    await waitFor(() => {
      expect((urlInput as HTMLInputElement).value).toBe('ws://localhost:8080');
    });
  });

  it('allows changing reconnect interval with buttons', async () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const reconnectInput = screen.getByLabelText('再接続間隔 (ms)') as HTMLInputElement;
    const incrementButtons = screen.getAllByText('+');
    const reconnectIncrementButton = incrementButtons[0]; // First increment button

    fireEvent.click(reconnectIncrementButton);

    await waitFor(() => {
      expect(reconnectInput.value).toBe('6000');
    });
  });

  it('prevents setting reconnect interval below minimum', async () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const reconnectInput = screen.getByLabelText('再接続間隔 (ms)');
    fireEvent.change(reconnectInput, { target: { value: '500' } });

    await waitFor(() => {
      expect((reconnectInput as HTMLInputElement).value).toBe('1000');
    });
  });

  it('prevents setting reconnect interval above maximum', async () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const reconnectInput = screen.getByLabelText('再接続間隔 (ms)');
    fireEvent.change(reconnectInput, { target: { value: '70000' } });

    await waitFor(() => {
      expect((reconnectInput as HTMLInputElement).value).toBe('60000');
    });
  });

  it('shows save button as disabled when no changes are made', () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const saveButton = screen.getByText('保存');
    expect(saveButton).toBeDisabled();
  });

  it('enables save button when changes are made', async () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const urlInput = screen.getByLabelText('WebSocket URL');
    const saveButton = screen.getByText('保存');

    fireEvent.change(urlInput, { target: { value: 'ws://localhost:8080' } });

    await waitFor(() => {
      expect(saveButton).not.toBeDisabled();
    });
  });

  it('saves settings and closes modal when save button is clicked', async () => {
    const emitSpy = jest.spyOn(workDataStore, 'emit');

    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const urlInput = screen.getByLabelText('WebSocket URL');
    const saveButton = screen.getByText('保存');

    fireEvent.change(urlInput, { target: { value: 'ws://localhost:8080' } });
    fireEvent.click(saveButton);

    await waitFor(() => {
      expect(emitSpy).toHaveBeenCalledWith('connection_settings_updated', expect.objectContaining({
        websocketUrl: 'ws://localhost:8080'
      }));
      expect(mockOnClose).toHaveBeenCalled();
    });

    emitSpy.mockRestore();
  });

  it('resets to default values when reset button is clicked', async () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const urlInput = screen.getByLabelText('WebSocket URL') as HTMLInputElement;
    const reconnectInput = screen.getByLabelText('再接続間隔 (ms)') as HTMLInputElement;
    const resetButton = screen.getByText('デフォルトに戻す');

    // Change values first
    fireEvent.change(urlInput, { target: { value: 'ws://localhost:8080' } });
    fireEvent.change(reconnectInput, { target: { value: '10000' } });

    // Reset to defaults
    fireEvent.click(resetButton);

    await waitFor(() => {
      expect(urlInput.value).toBe('ws://localhost:3001');
      expect(reconnectInput.value).toBe('5000');
    });
  });

  it('closes modal without saving when cancel button is clicked', async () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const urlInput = screen.getByLabelText('WebSocket URL');
    const cancelButton = screen.getByText('キャンセル');

    // Make changes
    fireEvent.change(urlInput, { target: { value: 'ws://localhost:8080' } });
    
    // Cancel
    fireEvent.click(cancelButton);

    expect(mockOnClose).toHaveBeenCalled();
  });

  it('closes modal when close button (×) is clicked', () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const closeButton = screen.getByText('✕');
    fireEvent.click(closeButton);

    expect(mockOnClose).toHaveBeenCalled();
  });

  it('formats uptime correctly', async () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    // Initially should show 00:00:00
    expect(screen.getByText('00:00:00')).toBeInTheDocument();

    // Advance time by smaller increments to avoid timeout
    jest.advanceTimersByTime(1000); // 1 second

    await waitFor(() => {
      expect(screen.getByText('00:00:01')).toBeInTheDocument();
    }, { timeout: 1000 });
  });

  it('updates connection quality display when store emits updates', async () => {
    render(
      <ConnectionMonitor
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    // Simulate connection quality update
    const newQuality = {
      latency: 50,
      dataRate: 1.5,
      stability: 'fair' as const,
      lastUpdated: new Date()
    };

    workDataStore.updateConnectionQuality(newQuality);

    await waitFor(() => {
      expect(screen.getByText('50ms')).toBeInTheDocument();
      expect(screen.getByText('1.5/秒')).toBeInTheDocument();
    });
  });
});