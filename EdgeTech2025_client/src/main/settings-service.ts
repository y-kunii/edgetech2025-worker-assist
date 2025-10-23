import * as fs from 'fs';
import * as path from 'path';
import { app } from 'electron';

export interface AppSettings {
  wsServerUrl: string;
  imageWatchPath: string;
  refreshInterval: number;
  autoConnect: boolean;
  theme: 'dark' | 'light';
  language: 'ja' | 'en';
  maxHistoryDays: number;
  enableNotifications: boolean;
}

export interface SettingsValidationResult {
  isValid: boolean;
  errors: string[];
}

export class SettingsService {
  private static instance: SettingsService | null = null;
  private settingsPath: string;
  private currentSettings: AppSettings;
  private readonly defaultSettings: AppSettings = {
    wsServerUrl: 'ws://localhost:9090',
    imageWatchPath: './images',
    refreshInterval: 1000,
    autoConnect: true,
    theme: 'dark',
    language: 'ja',
    maxHistoryDays: 30,
    enableNotifications: true
  };

  private constructor() {
    // 設定ファイルのパスを決定（ユーザーデータディレクトリ内）
    const userDataPath = app.getPath('userData');
    this.settingsPath = path.join(userDataPath, 'settings.json');
    
    // 設定を読み込み
    this.currentSettings = this.loadSettings();
  }

  public static getInstance(): SettingsService {
    if (!SettingsService.instance) {
      SettingsService.instance = new SettingsService();
    }
    return SettingsService.instance;
  }

  /**
   * 設定を取得
   */
  public getSettings(): AppSettings {
    return { ...this.currentSettings };
  }

  /**
   * 設定を保存
   */
  public async saveSettings(settings: Partial<AppSettings>): Promise<boolean> {
    try {
      // 新しい設定をマージ
      const newSettings = { ...this.currentSettings, ...settings };
      
      // 設定を検証
      const validation = this.validateSettings(newSettings);
      if (!validation.isValid) {
        console.error('Settings validation failed:', validation.errors);
        return false;
      }

      // ファイルに保存
      await this.writeSettingsFile(newSettings);
      
      // メモリ上の設定を更新
      this.currentSettings = newSettings;
      
      console.log('Settings saved successfully');
      return true;
    } catch (error) {
      console.error('Failed to save settings:', error);
      return false;
    }
  }

  /**
   * 設定をリセット（デフォルト値に戻す）
   */
  public async resetSettings(): Promise<boolean> {
    try {
      await this.writeSettingsFile(this.defaultSettings);
      this.currentSettings = { ...this.defaultSettings };
      console.log('Settings reset to defaults');
      return true;
    } catch (error) {
      console.error('Failed to reset settings:', error);
      return false;
    }
  }

  /**
   * 特定の設定値を取得
   */
  public getSetting<K extends keyof AppSettings>(key: K): AppSettings[K] {
    return this.currentSettings[key];
  }

  /**
   * 特定の設定値を更新
   */
  public async updateSetting<K extends keyof AppSettings>(
    key: K, 
    value: AppSettings[K]
  ): Promise<boolean> {
    const partialSettings = { [key]: value } as Partial<AppSettings>;
    return await this.saveSettings(partialSettings);
  }

  /**
   * 設定の検証
   */
  public validateSettings(settings: AppSettings): SettingsValidationResult {
    const errors: string[] = [];

    // WebSocket URL の検証
    if (!settings.wsServerUrl || !this.isValidWebSocketUrl(settings.wsServerUrl)) {
      errors.push('Invalid WebSocket server URL');
    }

    // 画像監視パスの検証
    if (!settings.imageWatchPath || settings.imageWatchPath.trim() === '') {
      errors.push('Image watch path cannot be empty');
    }

    // リフレッシュ間隔の検証
    if (!Number.isInteger(settings.refreshInterval) || 
        settings.refreshInterval < 100 || 
        settings.refreshInterval > 10000) {
      errors.push('Refresh interval must be between 100 and 10000 milliseconds');
    }

    // 履歴保持日数の検証
    if (!Number.isInteger(settings.maxHistoryDays) || 
        settings.maxHistoryDays < 1 || 
        settings.maxHistoryDays > 365) {
      errors.push('Max history days must be between 1 and 365');
    }

    // テーマの検証
    if (!['dark', 'light'].includes(settings.theme)) {
      errors.push('Theme must be either "dark" or "light"');
    }

    // 言語の検証
    if (!['ja', 'en'].includes(settings.language)) {
      errors.push('Language must be either "ja" or "en"');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  /**
   * 設定ファイルの存在確認
   */
  public settingsFileExists(): boolean {
    return fs.existsSync(this.settingsPath);
  }

  /**
   * 設定ファイルのパスを取得
   */
  public getSettingsPath(): string {
    return this.settingsPath;
  }

  /**
   * デフォルト設定を取得
   */
  public getDefaultSettings(): AppSettings {
    return { ...this.defaultSettings };
  }

  /**
   * 設定ファイルを読み込み
   */
  private loadSettings(): AppSettings {
    try {
      if (!fs.existsSync(this.settingsPath)) {
        console.log('Settings file not found, using defaults');
        return { ...this.defaultSettings };
      }

      const settingsData = fs.readFileSync(this.settingsPath, 'utf-8');
      const loadedSettings = JSON.parse(settingsData) as Partial<AppSettings>;
      
      // デフォルト設定とマージ（新しい設定項目への対応）
      const mergedSettings = { ...this.defaultSettings, ...loadedSettings };
      
      // 設定を検証
      const validation = this.validateSettings(mergedSettings);
      if (!validation.isValid) {
        console.warn('Loaded settings are invalid, using defaults:', validation.errors);
        return { ...this.defaultSettings };
      }

      console.log('Settings loaded successfully');
      return mergedSettings;
    } catch (error) {
      console.error('Failed to load settings, using defaults:', error);
      return { ...this.defaultSettings };
    }
  }

  /**
   * 設定ファイルに書き込み
   */
  private async writeSettingsFile(settings: AppSettings): Promise<void> {
    try {
      // ディレクトリが存在しない場合は作成
      const settingsDir = path.dirname(this.settingsPath);
      if (!fs.existsSync(settingsDir)) {
        fs.mkdirSync(settingsDir, { recursive: true });
      }

      // 設定をJSONファイルに保存
      const settingsJson = JSON.stringify(settings, null, 2);
      fs.writeFileSync(this.settingsPath, settingsJson, 'utf-8');
    } catch (error) {
      throw new Error(`Failed to write settings file: ${error}`);
    }
  }

  /**
   * WebSocket URLの妥当性チェック
   */
  private isValidWebSocketUrl(url: string): boolean {
    try {
      const parsedUrl = new URL(url);
      return parsedUrl.protocol === 'ws:' || parsedUrl.protocol === 'wss:';
    } catch {
      return false;
    }
  }
}