import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import ThresholdSettings from '../renderer/components/ThresholdSettings';
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

describe('ThresholdSettings', () => {
  let workDataStore: WorkDataStore;
  let mockOnClose: jest.Mock;

  beforeEach(() => {
    workDataStore = new WorkDataStore();
    mockOnClose = jest.fn();
    localStorageMock.getItem.mockClear();
    localStorageMock.setItem.mockClear();
  });

  afterEach(() => {
    workDataStore.destroy();
  });

  it('renders threshold settings modal when visible', () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    expect(screen.getByText('閾値設定')).toBeInTheDocument();
    expect(screen.getByLabelText('ネジ締め目標回数')).toBeInTheDocument();
    expect(screen.getByLabelText('ボルト締め目標回数')).toBeInTheDocument();
  });

  it('does not render when not visible', () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={false}
        onClose={mockOnClose}
      />
    );

    expect(screen.queryByText('閾値設定')).not.toBeInTheDocument();
  });

  it('displays current threshold values', () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const screwInput = screen.getByLabelText('ネジ締め目標回数') as HTMLInputElement;
    const boltInput = screen.getByLabelText('ボルト締め目標回数') as HTMLInputElement;

    expect(screwInput.value).toBe('5'); // Default value
    expect(boltInput.value).toBe('3'); // Default value
  });

  it('allows changing screw threshold with increment/decrement buttons', async () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const screwInput = screen.getByLabelText('ネジ締め目標回数') as HTMLInputElement;
    const incrementButtons = screen.getAllByText('+');
    const screwIncrementButton = incrementButtons[0]; // First increment button is for screw

    fireEvent.click(screwIncrementButton);

    await waitFor(() => {
      expect(screwInput.value).toBe('6');
    });
  });

  it('allows changing bolt threshold with increment/decrement buttons', async () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const boltInput = screen.getByLabelText('ボルト締め目標回数') as HTMLInputElement;
    const decrementButtons = screen.getAllByText('-');
    const boltDecrementButton = decrementButtons[1]; // Second decrement button is for bolt

    fireEvent.click(boltDecrementButton);

    await waitFor(() => {
      expect(boltInput.value).toBe('2');
    });
  });

  it('prevents setting values below 1', async () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const screwInput = screen.getByLabelText('ネジ締め目標回数') as HTMLInputElement;
    
    // Try to set value to 0
    fireEvent.change(screwInput, { target: { value: '0' } });

    await waitFor(() => {
      expect(screwInput.value).toBe('1'); // Should be clamped to minimum
    });
  });

  it('prevents setting values above 20', async () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const boltInput = screen.getByLabelText('ボルト締め目標回数') as HTMLInputElement;
    
    // Try to set value to 25
    fireEvent.change(boltInput, { target: { value: '25' } });

    await waitFor(() => {
      expect(boltInput.value).toBe('20'); // Should be clamped to maximum
    });
  });

  it('shows save button as disabled when no changes are made', () => {
    render(
      <ThresholdSettings
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
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const screwInput = screen.getByLabelText('ネジ締め目標回数');
    const saveButton = screen.getByText('保存');

    fireEvent.change(screwInput, { target: { value: '7' } });

    await waitFor(() => {
      expect(saveButton).not.toBeDisabled();
    });
  });

  it('saves settings and closes modal when save button is clicked', async () => {
    const updateSpy = jest.spyOn(workDataStore, 'updateThresholdSettings');
    updateSpy.mockReturnValue(true);

    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const screwInput = screen.getByLabelText('ネジ締め目標回数');
    const saveButton = screen.getByText('保存');

    fireEvent.change(screwInput, { target: { value: '7' } });
    fireEvent.click(saveButton);

    await waitFor(() => {
      expect(updateSpy).toHaveBeenCalledWith({
        screwThreshold: 7,
        boltThreshold: 3
      });
      expect(mockOnClose).toHaveBeenCalled();
    });

    updateSpy.mockRestore();
  });

  it('resets to default values when reset button is clicked', async () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const screwInput = screen.getByLabelText('ネジ締め目標回数') as HTMLInputElement;
    const boltInput = screen.getByLabelText('ボルト締め目標回数') as HTMLInputElement;
    const resetButton = screen.getByText('デフォルトに戻す');

    // Change values first
    fireEvent.change(screwInput, { target: { value: '10' } });
    fireEvent.change(boltInput, { target: { value: '8' } });

    // Reset to defaults
    fireEvent.click(resetButton);

    await waitFor(() => {
      expect(screwInput.value).toBe('5');
      expect(boltInput.value).toBe('3');
    });
  });

  it('closes modal without saving when cancel button is clicked', async () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const screwInput = screen.getByLabelText('ネジ締め目標回数');
    const cancelButton = screen.getByText('キャンセル');

    // Make changes
    fireEvent.change(screwInput, { target: { value: '10' } });
    
    // Cancel
    fireEvent.click(cancelButton);

    expect(mockOnClose).toHaveBeenCalled();
  });

  it('closes modal when close button (×) is clicked', () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const closeButton = screen.getByText('✕');
    fireEvent.click(closeButton);

    expect(mockOnClose).toHaveBeenCalled();
  });

  it('displays current vs new settings description', async () => {
    render(
      <ThresholdSettings
        workDataStore={workDataStore}
        isVisible={true}
        onClose={mockOnClose}
      />
    );

    const screwInput = screen.getByLabelText('ネジ締め目標回数');
    
    // Change screw threshold
    fireEvent.change(screwInput, { target: { value: '8' } });

    await waitFor(() => {
      expect(screen.getByText(/現在: 5回 → 新しい設定: 8回/)).toBeInTheDocument();
    });
  });
});