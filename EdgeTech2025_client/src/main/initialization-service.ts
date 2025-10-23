import { app } from 'electron';
import { SettingsService } from './settings-service';
import { DataProcessor } from './data-processor';
import * as fs from 'fs';
import * as path from 'path';

export interface InitializationResult {
    success: boolean;
    errors: string[];
    warnings: string[];
}

export class InitializationService {
    private static instance: InitializationService | null = null;
    private settingsService: SettingsService;
    private isInitialized: boolean = false;

    private constructor() {
        this.settingsService = SettingsService.getInstance();
    }

    public static getInstance(): InitializationService {
        if (!InitializationService.instance) {
            InitializationService.instance = new InitializationService();
        }
        return InitializationService.instance;
    }

    /**
     * アプリケーションの初期化処理
     */
    public async initialize(): Promise<InitializationResult> {
        const result: InitializationResult = {
            success: true,
            errors: [],
            warnings: []
        };

        try {
            console.log('Starting application initialization...');

            // 1. ユーザーデータディレクトリの確認・作成
            await this.ensureUserDataDirectory(result);

            // 2. 設定ファイルの初期化
            await this.initializeSettings(result);

            // 3. データベースの初期化
            await this.initializeDatabase(result);

            // 4. 画像フォルダの確認・作成
            await this.ensureImageDirectory(result);

            // 5. ログディレクトリの確認・作成
            await this.ensureLogDirectory(result);

            // 6. 一時ファイルのクリーンアップ
            await this.cleanupTemporaryFiles(result);

            this.isInitialized = result.success;

            if (result.success) {
                console.log('Application initialization completed successfully');
            } else {
                console.error('Application initialization failed:', result.errors);
            }

            return result;
        } catch (error) {
            result.success = false;
            result.errors.push(`Initialization failed: ${error}`);
            console.error('Critical initialization error:', error);
            return result;
        }
    }

    /**
     * アプリケーションの終了処理
     */
    public async shutdown(): Promise<InitializationResult> {
        const result: InitializationResult = {
            success: true,
            errors: [],
            warnings: []
        };

        try {
            console.log('Starting application shutdown...');

            // 1. 設定の最終保存
            await this.finalizeSettings(result);

            // 2. データベースの安全な切断
            await this.shutdownDatabase(result);

            // 3. 一時ファイルのクリーンアップ
            await this.cleanupTemporaryFiles(result);

            // 4. ログファイルの整理
            await this.finalizeLogFiles(result);

            console.log('Application shutdown completed');
            return result;
        } catch (error) {
            result.success = false;
            result.errors.push(`Shutdown failed: ${error}`);
            console.error('Critical shutdown error:', error);
            return result;
        }
    }

    /**
     * 初期化状態の確認
     */
    public isAppInitialized(): boolean {
        return this.isInitialized;
    }

    /**
     * アプリケーションの健全性チェック
     */
    public async performHealthCheck(): Promise<InitializationResult> {
        const result: InitializationResult = {
            success: true,
            errors: [],
            warnings: []
        };

        try {
            // 設定ファイルの確認
            if (!this.settingsService.settingsFileExists()) {
                result.warnings.push('Settings file does not exist');
            }

            // データベースの確認
            const dbStats = DataProcessor.getDatabaseStats();
            if (!dbStats || dbStats.statusRecords < 0) {
                result.errors.push('Database connection issue detected');
                result.success = false;
            }

            // 画像ディレクトリの確認
            const settings = this.settingsService.getSettings();
            if (!fs.existsSync(settings.imageWatchPath)) {
                result.warnings.push('Image watch directory does not exist');
            }

            return result;
        } catch (error) {
            result.success = false;
            result.errors.push(`Health check failed: ${error}`);
            return result;
        }
    }

    /**
     * ユーザーデータディレクトリの確認・作成
     */
    private async ensureUserDataDirectory(result: InitializationResult): Promise<void> {
        try {
            const userDataPath = app.getPath('userData');

            if (!fs.existsSync(userDataPath)) {
                fs.mkdirSync(userDataPath, { recursive: true });
                console.log(`Created user data directory: ${userDataPath}`);
            }

            // 書き込み権限の確認
            const testFile = path.join(userDataPath, 'test-write.tmp');
            fs.writeFileSync(testFile, 'test');
            fs.unlinkSync(testFile);

        } catch (error) {
            result.errors.push(`Failed to ensure user data directory: ${error}`);
            result.success = false;
        }
    }

    /**
     * 設定の初期化
     */
    private async initializeSettings(result: InitializationResult): Promise<void> {
        try {
            const settings = this.settingsService.getSettings();
            const validation = this.settingsService.validateSettings(settings);

            if (!validation.isValid) {
                result.warnings.push('Settings validation failed, using defaults');
                await this.settingsService.resetSettings();
            }

            console.log('Settings initialized successfully');
        } catch (error) {
            result.errors.push(`Failed to initialize settings: ${error}`);
            result.success = false;
        }
    }

    /**
     * データベースの初期化
     */
    private async initializeDatabase(result: InitializationResult): Promise<void> {
        try {
            DataProcessor.initialize();

            // データベースの整合性チェック
            const integrity = DataProcessor.validateDatabaseIntegrity();
            if (!integrity.isValid) {
                result.warnings.push('Database integrity issues detected');
                result.warnings.push(...integrity.errors);
            }

            console.log('Database initialized successfully');
        } catch (error) {
            result.errors.push(`Failed to initialize database: ${error}`);
            result.success = false;
        }
    }

    /**
     * 画像ディレクトリの確認・作成
     */
    private async ensureImageDirectory(result: InitializationResult): Promise<void> {
        try {
            const settings = this.settingsService.getSettings();
            const imagePath = path.resolve(settings.imageWatchPath);

            if (!fs.existsSync(imagePath)) {
                fs.mkdirSync(imagePath, { recursive: true });
                console.log(`Created image directory: ${imagePath}`);
            }

            // 読み取り権限の確認
            fs.accessSync(imagePath, fs.constants.R_OK);

        } catch (error) {
            result.warnings.push(`Image directory issue: ${error}`);
        }
    }

    /**
     * ログディレクトリの確認・作成
     */
    private async ensureLogDirectory(result: InitializationResult): Promise<void> {
        try {
            const userDataPath = app.getPath('userData');
            const logPath = path.join(userDataPath, 'logs');

            if (!fs.existsSync(logPath)) {
                fs.mkdirSync(logPath, { recursive: true });
                console.log(`Created log directory: ${logPath}`);
            }

        } catch (error) {
            result.warnings.push(`Failed to ensure log directory: ${error}`);
        }
    }

    /**
     * 一時ファイルのクリーンアップ
     */
    private async cleanupTemporaryFiles(result: InitializationResult): Promise<void> {
        try {
            const userDataPath = app.getPath('userData');
            const tempPath = path.join(userDataPath, 'temp');

            if (fs.existsSync(tempPath)) {
                const files = fs.readdirSync(tempPath);
                for (const file of files) {
                    const filePath = path.join(tempPath, file);
                    const stats = fs.statSync(filePath);

                    // 24時間以上古いファイルを削除
                    const ageInHours = (Date.now() - stats.mtime.getTime()) / (1000 * 60 * 60);
                    if (ageInHours > 24) {
                        fs.unlinkSync(filePath);
                        console.log(`Cleaned up temporary file: ${file}`);
                    }
                }
            }

        } catch (error) {
            result.warnings.push(`Failed to cleanup temporary files: ${error}`);
        }
    }

    /**
     * 設定の最終保存
     */
    private async finalizeSettings(result: InitializationResult): Promise<void> {
        try {
            // 現在の設定を保存（最後の状態を保持）
            const currentSettings = this.settingsService.getSettings();
            await this.settingsService.saveSettings(currentSettings);

            console.log('Settings finalized');
        } catch (error) {
            result.warnings.push(`Failed to finalize settings: ${error}`);
        }
    }

    /**
     * データベースの安全な切断
     */
    private async shutdownDatabase(result: InitializationResult): Promise<void> {
        try {
            // 最後のデータ整合性チェック
            const integrity = DataProcessor.validateDatabaseIntegrity();
            if (!integrity.isValid) {
                result.warnings.push('Database integrity issues detected during shutdown');
            }

            // データベース接続の切断
            const { closeDatabaseService } = await import('./database-service');
            closeDatabaseService();

            console.log('Database shutdown completed');
        } catch (error) {
            result.warnings.push(`Failed to shutdown database: ${error}`);
        }
    }

    /**
     * ログファイルの整理
     */
    private async finalizeLogFiles(result: InitializationResult): Promise<void> {
        try {
            const userDataPath = app.getPath('userData');
            const logPath = path.join(userDataPath, 'logs');

            if (fs.existsSync(logPath)) {
                const files = fs.readdirSync(logPath);

                // 古いログファイルの削除（30日以上）
                for (const file of files) {
                    const filePath = path.join(logPath, file);
                    const stats = fs.statSync(filePath);

                    const ageInDays = (Date.now() - stats.mtime.getTime()) / (1000 * 60 * 60 * 24);
                    if (ageInDays > 30) {
                        fs.unlinkSync(filePath);
                        console.log(`Cleaned up old log file: ${file}`);
                    }
                }
            }

        } catch (error) {
            result.warnings.push(`Failed to finalize log files: ${error}`);
        }
    }
}