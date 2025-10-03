/**
 * Data Integration Manager for optimized data flow and integration
 */

import { EventEmitter } from 'events';
import { WorkData, WorkerStatus, RobotStatus, ThresholdSettings } from '../types';
import { PerformanceMonitor, DataFlowOptimizer } from './performanceOptimizer';

export interface DataIntegrationConfig {
  batchUpdateDelay: number;
  maxHistorySize: number;
  enablePerformanceMonitoring: boolean;
  enableDataValidation: boolean;
}

export class DataIntegrationManager extends EventEmitter {
  private static instance: DataIntegrationManager;
  private config: DataIntegrationConfig;
  private performanceMonitor: PerformanceMonitor;
  private dataCache: Map<string, any> = new Map();
  private lastUpdateTimes: Map<string, number> = new Map();
  private validationRules: Map<string, (data: any) => boolean> = new Map();

  private constructor(config: Partial<DataIntegrationConfig> = {}) {
    super();
    
    this.config = {
      batchUpdateDelay: 16, // ~60fps
      maxHistorySize: 1000,
      enablePerformanceMonitoring: true,
      enableDataValidation: true,
      ...config
    };
    
    this.performanceMonitor = PerformanceMonitor.getInstance();
    this.setupValidationRules();
    this.setupDataFlowOptimization();
  }

  static getInstance(config?: Partial<DataIntegrationConfig>): DataIntegrationManager {
    if (!DataIntegrationManager.instance) {
      DataIntegrationManager.instance = new DataIntegrationManager(config);
    }
    return DataIntegrationManager.instance;
  }

  /**
   * Setup validation rules for different data types
   */
  private setupValidationRules(): void {
    // WorkData validation
    this.validationRules.set('workData', (data: WorkData) => {
      return (
        data &&
        typeof data.timestamp === 'object' &&
        data.timestamp instanceof Date &&
        typeof data.workerStatus === 'string' &&
        ['waiting', 'screw_tightening', 'bolt_tightening', 'tool_handover', 'absent'].includes(data.workerStatus) &&
        typeof data.robotStatus === 'object' &&
        typeof data.robotStatus.state === 'string' &&
        ['waiting', 'operating'].includes(data.robotStatus.state) &&
        typeof data.robotStatus.grip === 'string' &&
        ['open', 'closed'].includes(data.robotStatus.grip) &&
        typeof data.screwCount === 'number' &&
        data.screwCount >= 0 &&
        typeof data.boltCount === 'number' &&
        data.boltCount >= 0
      );
    });

    // Threshold settings validation
    this.validationRules.set('thresholdSettings', (data: ThresholdSettings) => {
      return (
        data &&
        typeof data.screwThreshold === 'number' &&
        data.screwThreshold >= 1 &&
        data.screwThreshold <= 20 &&
        typeof data.boltThreshold === 'number' &&
        data.boltThreshold >= 1 &&
        data.boltThreshold <= 20
      );
    });

    // Connection quality validation
    this.validationRules.set('connectionQuality', (data: any) => {
      return (
        data &&
        typeof data.latency === 'number' &&
        data.latency >= 0 &&
        typeof data.dataRate === 'number' &&
        data.dataRate >= 0 &&
        typeof data.stability === 'string' &&
        ['excellent', 'good', 'fair', 'poor'].includes(data.stability)
      );
    });
  }

  /**
   * Setup data flow optimization
   */
  private setupDataFlowOptimization(): void {
    // Subscribe to batched updates for different data types
    DataFlowOptimizer.subscribe('workData', (data) => {
      this.emit('workDataUpdated', data);
    });

    DataFlowOptimizer.subscribe('connectionStatus', (data) => {
      this.emit('connectionStatusUpdated', data);
    });

    DataFlowOptimizer.subscribe('thresholdSettings', (data) => {
      this.emit('thresholdSettingsUpdated', data);
    });

    DataFlowOptimizer.subscribe('statistics', (data) => {
      this.emit('statisticsUpdated', data);
    });
  }

  /**
   * Validate data using registered rules
   */
  private validateData(type: string, data: any): boolean {
    if (!this.config.enableDataValidation) {
      return true;
    }

    const validator = this.validationRules.get(type);
    if (!validator) {
      console.warn(`No validation rule found for data type: ${type}`);
      return true;
    }

    try {
      return validator(data);
    } catch (error) {
      console.error(`Validation error for ${type}:`, error);
      return false;
    }
  }

  /**
   * Process and integrate work data with optimization
   */
  public integrateWorkData(data: WorkData): boolean {
    if (this.config.enablePerformanceMonitoring) {
      this.performanceMonitor.startMeasure('integrateWorkData');
    }

    try {
      // Validate data
      if (!this.validateData('workData', data)) {
        console.error('Invalid work data:', data);
        return false;
      }

      // Check if data has actually changed to avoid unnecessary updates
      const cacheKey = 'currentWorkData';
      const cachedData = this.dataCache.get(cacheKey);
      
      if (cachedData && this.isWorkDataEqual(cachedData, data)) {
        return true; // No change, skip update
      }

      // Cache the new data
      this.dataCache.set(cacheKey, { ...data });
      this.lastUpdateTimes.set(cacheKey, Date.now());

      // Batch the update to optimize rendering
      DataFlowOptimizer.batchUpdate('workData', data, this.config.batchUpdateDelay);

      return true;
    } catch (error) {
      console.error('Error integrating work data:', error);
      return false;
    } finally {
      if (this.config.enablePerformanceMonitoring) {
        this.performanceMonitor.endMeasure('integrateWorkData');
      }
    }
  }

  /**
   * Process and integrate connection status
   */
  public integrateConnectionStatus(isConnected: boolean, quality?: any): void {
    if (this.config.enablePerformanceMonitoring) {
      this.performanceMonitor.startMeasure('integrateConnectionStatus');
    }

    try {
      const connectionData = {
        isConnected,
        quality: quality && this.validateData('connectionQuality', quality) ? quality : null,
        timestamp: new Date()
      };

      // Check for changes
      const cacheKey = 'connectionStatus';
      const cachedData = this.dataCache.get(cacheKey);
      
      if (cachedData && 
          cachedData.isConnected === isConnected && 
          JSON.stringify(cachedData.quality) === JSON.stringify(quality)) {
        return; // No change
      }

      this.dataCache.set(cacheKey, connectionData);
      this.lastUpdateTimes.set(cacheKey, Date.now());

      DataFlowOptimizer.batchUpdate('connectionStatus', connectionData, this.config.batchUpdateDelay);
    } catch (error) {
      console.error('Error integrating connection status:', error);
    } finally {
      if (this.config.enablePerformanceMonitoring) {
        this.performanceMonitor.endMeasure('integrateConnectionStatus');
      }
    }
  }

  /**
   * Process and integrate threshold settings
   */
  public integrateThresholdSettings(settings: ThresholdSettings): boolean {
    if (this.config.enablePerformanceMonitoring) {
      this.performanceMonitor.startMeasure('integrateThresholdSettings');
    }

    try {
      if (!this.validateData('thresholdSettings', settings)) {
        console.error('Invalid threshold settings:', settings);
        return false;
      }

      const cacheKey = 'thresholdSettings';
      const cachedData = this.dataCache.get(cacheKey);
      
      if (cachedData && 
          cachedData.screwThreshold === settings.screwThreshold &&
          cachedData.boltThreshold === settings.boltThreshold) {
        return true; // No change
      }

      this.dataCache.set(cacheKey, { ...settings });
      this.lastUpdateTimes.set(cacheKey, Date.now());

      DataFlowOptimizer.batchUpdate('thresholdSettings', settings, this.config.batchUpdateDelay);
      
      return true;
    } catch (error) {
      console.error('Error integrating threshold settings:', error);
      return false;
    } finally {
      if (this.config.enablePerformanceMonitoring) {
        this.performanceMonitor.endMeasure('integrateThresholdSettings');
      }
    }
  }

  /**
   * Process and integrate statistics data
   */
  public integrateStatistics(statistics: any): void {
    if (this.config.enablePerformanceMonitoring) {
      this.performanceMonitor.startMeasure('integrateStatistics');
    }

    try {
      const cacheKey = 'statistics';
      this.dataCache.set(cacheKey, { ...statistics, timestamp: new Date() });
      this.lastUpdateTimes.set(cacheKey, Date.now());

      DataFlowOptimizer.batchUpdate('statistics', statistics, this.config.batchUpdateDelay);
    } catch (error) {
      console.error('Error integrating statistics:', error);
    } finally {
      if (this.config.enablePerformanceMonitoring) {
        this.performanceMonitor.endMeasure('integrateStatistics');
      }
    }
  }

  /**
   * Check if two WorkData objects are equal
   */
  private isWorkDataEqual(data1: WorkData, data2: WorkData): boolean {
    return (
      data1.workerStatus === data2.workerStatus &&
      data1.robotStatus.state === data2.robotStatus.state &&
      data1.robotStatus.grip === data2.robotStatus.grip &&
      data1.screwCount === data2.screwCount &&
      data1.boltCount === data2.boltCount &&
      data1.image === data2.image
    );
  }

  /**
   * Get cached data
   */
  public getCachedData(key: string): any {
    return this.dataCache.get(key);
  }

  /**
   * Get last update time for a data type
   */
  public getLastUpdateTime(key: string): number | undefined {
    return this.lastUpdateTimes.get(key);
  }

  /**
   * Get data freshness (time since last update)
   */
  public getDataFreshness(key: string): number {
    const lastUpdate = this.lastUpdateTimes.get(key);
    return lastUpdate ? Date.now() - lastUpdate : Infinity;
  }

  /**
   * Clear old cached data
   */
  public clearOldCache(maxAge: number = 300000): void { // 5 minutes default
    const now = Date.now();
    const keysToDelete: string[] = [];

    this.lastUpdateTimes.forEach((timestamp, key) => {
      if (now - timestamp > maxAge) {
        keysToDelete.push(key);
      }
    });

    keysToDelete.forEach(key => {
      this.dataCache.delete(key);
      this.lastUpdateTimes.delete(key);
    });
  }

  /**
   * Get performance metrics
   */
  public getPerformanceMetrics(): any {
    if (!this.config.enablePerformanceMonitoring) {
      return null;
    }
    
    return this.performanceMonitor.getAllMetrics();
  }

  /**
   * Reset performance metrics
   */
  public resetPerformanceMetrics(): void {
    // Clear internal metrics
    this.performanceMonitor = PerformanceMonitor.getInstance();
  }

  /**
   * Destroy the manager and clean up resources
   */
  public destroy(): void {
    this.removeAllListeners();
    this.dataCache.clear();
    this.lastUpdateTimes.clear();
    this.validationRules.clear();
  }
}