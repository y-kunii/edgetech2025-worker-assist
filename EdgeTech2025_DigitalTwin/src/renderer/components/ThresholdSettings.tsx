import React, { useState, useEffect } from 'react';
import { WorkDataStore } from '../../stores/WorkDataStore';
import { ThresholdSettings as ThresholdSettingsType } from '../../types';
import './ThresholdSettings.css';

interface ThresholdSettingsProps {
  workDataStore: WorkDataStore;
  isVisible: boolean;
  onClose: () => void;
}

const ThresholdSettings: React.FC<ThresholdSettingsProps> = ({ 
  workDataStore, 
  isVisible, 
  onClose 
}) => {
  const [settings, setSettings] = useState<ThresholdSettingsType>({ screwThreshold: 5, boltThreshold: 3 });
  const [tempSettings, setTempSettings] = useState<ThresholdSettingsType>({ screwThreshold: 5, boltThreshold: 3 });
  const [hasChanges, setHasChanges] = useState(false);

  useEffect(() => {
    const initialState = workDataStore.getState();
    setSettings(initialState.thresholdSettings);
    setTempSettings(initialState.thresholdSettings);
  }, [workDataStore]);

  useEffect(() => {
    const hasChanges = 
      tempSettings.screwThreshold !== settings.screwThreshold ||
      tempSettings.boltThreshold !== settings.boltThreshold;
    setHasChanges(hasChanges);
  }, [tempSettings, settings]);

  const handleScrewThresholdChange = (value: number) => {
    const clampedValue = Math.max(1, Math.min(20, value));
    setTempSettings(prev => ({ ...prev, screwThreshold: clampedValue }));
  };

  const handleBoltThresholdChange = (value: number) => {
    const clampedValue = Math.max(1, Math.min(20, value));
    setTempSettings(prev => ({ ...prev, boltThreshold: clampedValue }));
  };

  const handleSave = () => {
    const success = workDataStore.updateThresholdSettings(tempSettings);
    if (success) {
      setSettings(tempSettings);
      setHasChanges(false);
      onClose();
    }
  };

  const handleCancel = () => {
    setTempSettings(settings);
    setHasChanges(false);
    onClose();
  };

  const handleReset = () => {
    const defaultSettings = { screwThreshold: 5, boltThreshold: 3 };
    setTempSettings(defaultSettings);
  };

  if (!isVisible) {
    return null;
  }

  return (
    <div className="threshold-settings-overlay">
      <div className="threshold-settings-modal">
        <div className="modal-header">
          <h3>閾値設定</h3>
          <button className="close-button" onClick={handleCancel}>
            ✕
          </button>
        </div>
        
        <div className="modal-content">
          <div className="setting-group">
            <label htmlFor="screw-threshold">ネジ締め目標回数</label>
            <div className="input-group">
              <button 
                className="decrement-button"
                onClick={() => handleScrewThresholdChange(tempSettings.screwThreshold - 1)}
                disabled={tempSettings.screwThreshold <= 1}
              >
                -
              </button>
              <input
                id="screw-threshold"
                type="number"
                min="1"
                max="20"
                value={tempSettings.screwThreshold}
                onChange={(e) => handleScrewThresholdChange(parseInt(e.target.value) || 1)}
              />
              <button 
                className="increment-button"
                onClick={() => handleScrewThresholdChange(tempSettings.screwThreshold + 1)}
                disabled={tempSettings.screwThreshold >= 20}
              >
                +
              </button>
            </div>
            <div className="setting-description">
              現在: {settings.screwThreshold}回 → 新しい設定: {tempSettings.screwThreshold}回
            </div>
          </div>

          <div className="setting-group">
            <label htmlFor="bolt-threshold">ボルト締め目標回数</label>
            <div className="input-group">
              <button 
                className="decrement-button"
                onClick={() => handleBoltThresholdChange(tempSettings.boltThreshold - 1)}
                disabled={tempSettings.boltThreshold <= 1}
              >
                -
              </button>
              <input
                id="bolt-threshold"
                type="number"
                min="1"
                max="20"
                value={tempSettings.boltThreshold}
                onChange={(e) => handleBoltThresholdChange(parseInt(e.target.value) || 1)}
              />
              <button 
                className="increment-button"
                onClick={() => handleBoltThresholdChange(tempSettings.boltThreshold + 1)}
                disabled={tempSettings.boltThreshold >= 20}
              >
                +
              </button>
            </div>
            <div className="setting-description">
              現在: {settings.boltThreshold}回 → 新しい設定: {tempSettings.boltThreshold}回
            </div>
          </div>
        </div>

        <div className="modal-footer">
          <button className="reset-button" onClick={handleReset}>
            デフォルトに戻す
          </button>
          <div className="action-buttons">
            <button className="cancel-button" onClick={handleCancel}>
              キャンセル
            </button>
            <button 
              className="save-button" 
              onClick={handleSave}
              disabled={!hasChanges}
            >
              保存
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ThresholdSettings;