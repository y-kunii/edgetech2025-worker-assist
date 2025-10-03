/**
 * Performance optimization utilities for the manufacturing digital twin demo
 */

// Performance monitoring
export class PerformanceMonitor {
  private static instance: PerformanceMonitor;
  private metrics: Map<string, number[]> = new Map();
  private memoryUsage: number[] = [];
  private frameRates: number[] = [];
  private lastFrameTime = 0;

  static getInstance(): PerformanceMonitor {
    if (!PerformanceMonitor.instance) {
      PerformanceMonitor.instance = new PerformanceMonitor();
    }
    return PerformanceMonitor.instance;
  }

  /**
   * Start measuring performance for a specific operation
   */
  startMeasure(name: string): void {
    performance.mark(`${name}-start`);
  }

  /**
   * End measuring performance for a specific operation
   */
  endMeasure(name: string): number {
    performance.mark(`${name}-end`);
    performance.measure(name, `${name}-start`, `${name}-end`);
    
    const measure = performance.getEntriesByName(name, 'measure')[0];
    const duration = measure.duration;
    
    if (!this.metrics.has(name)) {
      this.metrics.set(name, []);
    }
    
    const measurements = this.metrics.get(name)!;
    measurements.push(duration);
    
    // Keep only last 100 measurements
    if (measurements.length > 100) {
      measurements.shift();
    }
    
    // Clean up performance entries
    performance.clearMarks(`${name}-start`);
    performance.clearMarks(`${name}-end`);
    performance.clearMeasures(name);
    
    return duration;
  }

  /**
   * Get average performance for an operation
   */
  getAveragePerformance(name: string): number {
    const measurements = this.metrics.get(name);
    if (!measurements || measurements.length === 0) return 0;
    
    return measurements.reduce((sum, val) => sum + val, 0) / measurements.length;
  }

  /**
   * Monitor memory usage
   */
  recordMemoryUsage(): void {
    if ('memory' in performance) {
      const memory = (performance as any).memory;
      this.memoryUsage.push(memory.usedJSHeapSize);
      
      // Keep only last 100 measurements
      if (this.memoryUsage.length > 100) {
        this.memoryUsage.shift();
      }
    }
  }

  /**
   * Get current memory usage statistics
   */
  getMemoryStats(): { current: number; average: number; peak: number } {
    if (this.memoryUsage.length === 0) {
      return { current: 0, average: 0, peak: 0 };
    }
    
    const current = this.memoryUsage[this.memoryUsage.length - 1];
    const average = this.memoryUsage.reduce((sum, val) => sum + val, 0) / this.memoryUsage.length;
    const peak = Math.max(...this.memoryUsage);
    
    return { current, average, peak };
  }

  /**
   * Monitor frame rate
   */
  recordFrameRate(): void {
    const now = performance.now();
    if (this.lastFrameTime > 0) {
      const frameTime = now - this.lastFrameTime;
      const fps = 1000 / frameTime;
      this.frameRates.push(fps);
      
      // Keep only last 100 measurements
      if (this.frameRates.length > 100) {
        this.frameRates.shift();
      }
    }
    this.lastFrameTime = now;
  }

  /**
   * Get frame rate statistics
   */
  getFrameRateStats(): { current: number; average: number; min: number } {
    if (this.frameRates.length === 0) {
      return { current: 0, average: 0, min: 0 };
    }
    
    const current = this.frameRates[this.frameRates.length - 1];
    const average = this.frameRates.reduce((sum, val) => sum + val, 0) / this.frameRates.length;
    const min = Math.min(...this.frameRates);
    
    return { current, average, min };
  }

  /**
   * Get all performance metrics
   */
  getAllMetrics(): Record<string, any> {
    const metrics: Record<string, any> = {};
    
    for (const [name, measurements] of this.metrics.entries()) {
      metrics[name] = {
        average: measurements.reduce((sum, val) => sum + val, 0) / measurements.length,
        min: Math.min(...measurements),
        max: Math.max(...measurements),
        count: measurements.length
      };
    }
    
    return {
      operations: metrics,
      memory: this.getMemoryStats(),
      frameRate: this.getFrameRateStats()
    };
  }
}

// Animation optimization utilities
export class AnimationOptimizer {
  private static rafId: number | null = null;
  private static callbacks: Set<() => void> = new Set();
  private static isRunning = false;

  /**
   * Batch animation updates to a single RAF
   */
  static addCallback(callback: () => void): void {
    this.callbacks.add(callback);
    this.startLoop();
  }

  /**
   * Remove animation callback
   */
  static removeCallback(callback: () => void): void {
    this.callbacks.delete(callback);
    if (this.callbacks.size === 0) {
      this.stopLoop();
    }
  }

  private static startLoop(): void {
    if (this.isRunning) return;
    
    this.isRunning = true;
    const loop = () => {
      if (this.callbacks.size === 0) {
        this.stopLoop();
        return;
      }
      
      // Execute all callbacks
      this.callbacks.forEach(callback => {
        try {
          callback();
        } catch (error) {
          console.error('Animation callback error:', error);
        }
      });
      
      this.rafId = requestAnimationFrame(loop);
    };
    
    this.rafId = requestAnimationFrame(loop);
  }

  private static stopLoop(): void {
    if (this.rafId) {
      cancelAnimationFrame(this.rafId);
      this.rafId = null;
    }
    this.isRunning = false;
  }
}

// Memory optimization utilities
export class MemoryOptimizer {
  private static objectPools: Map<string, any[]> = new Map();
  private static weakRefs: Set<any> = new Set();

  /**
   * Create an object pool for reusing objects
   */
  static createPool<T>(name: string, factory: () => T, initialSize = 10): void {
    const pool: T[] = [];
    for (let i = 0; i < initialSize; i++) {
      pool.push(factory());
    }
    this.objectPools.set(name, pool);
  }

  /**
   * Get an object from the pool
   */
  static getFromPool<T>(name: string, factory?: () => T): T | null {
    const pool = this.objectPools.get(name);
    if (!pool || pool.length === 0) {
      return factory ? factory() : null;
    }
    return pool.pop() as T;
  }

  /**
   * Return an object to the pool
   */
  static returnToPool(name: string, object: any): void {
    const pool = this.objectPools.get(name);
    if (pool && pool.length < 50) { // Limit pool size
      // Reset object properties if needed
      if (typeof object.reset === 'function') {
        object.reset();
      }
      pool.push(object);
    }
  }

  /**
   * Add a weak reference for cleanup
   */
  static addWeakRef(object: any): void {
    // Use a simple reference for compatibility
    this.weakRefs.add(object);
  }

  /**
   * Clean up dead weak references
   */
  static cleanupWeakRefs(): void {
    // Simple cleanup - in a real implementation, this would use WeakRef
    // For now, just clear old references periodically
    if (this.weakRefs.size > 100) {
      this.weakRefs.clear();
    }
  }

  /**
   * Force garbage collection if available
   */
  static forceGC(): void {
    if ('gc' in window && typeof (window as any).gc === 'function') {
      (window as any).gc();
    }
  }
}

// Data flow optimization utilities
export class DataFlowOptimizer {
  private static updateQueue: Map<string, any> = new Map();
  private static batchTimeout: number | null = null;
  private static subscribers: Map<string, Set<(data: any) => void>> = new Map();

  /**
   * Batch data updates to reduce re-renders
   */
  static batchUpdate(key: string, data: any, delay = 16): void {
    this.updateQueue.set(key, data);
    
    if (this.batchTimeout) {
      clearTimeout(this.batchTimeout);
    }
    
    this.batchTimeout = window.setTimeout(() => {
      this.flushUpdates();
    }, delay);
  }

  /**
   * Subscribe to batched updates
   */
  static subscribe(key: string, callback: (data: any) => void): () => void {
    if (!this.subscribers.has(key)) {
      this.subscribers.set(key, new Set());
    }
    
    this.subscribers.get(key)!.add(callback);
    
    // Return unsubscribe function
    return () => {
      const subs = this.subscribers.get(key);
      if (subs) {
        subs.delete(callback);
        if (subs.size === 0) {
          this.subscribers.delete(key);
        }
      }
    };
  }

  private static flushUpdates(): void {
    const updates = new Map(this.updateQueue);
    this.updateQueue.clear();
    this.batchTimeout = null;
    
    updates.forEach((data, key) => {
      const subs = this.subscribers.get(key);
      if (subs) {
        subs.forEach(callback => {
          try {
            callback(data);
          } catch (error) {
            console.error(`Error in subscriber for ${key}:`, error);
          }
        });
      }
    });
  }
}

// Device capability detection
export class DeviceCapabilities {
  private static capabilities: Record<string, boolean> | null = null;

  /**
   * Detect device capabilities
   */
  static detect(): Record<string, boolean> {
    if (this.capabilities) {
      return this.capabilities;
    }

    const canvas = document.createElement('canvas');
    const gl = canvas.getContext('webgl') || canvas.getContext('experimental-webgl');
    
    this.capabilities = {
      // Hardware acceleration
      webgl: !!gl,
      hardwareAcceleration: !!gl,
      
      // Memory
      highMemory: (navigator as any).deviceMemory ? (navigator as any).deviceMemory >= 4 : true,
      
      // Network
      fastConnection: (navigator as any).connection ? 
        ['4g', 'wifi'].includes(((navigator as any).connection as any).effectiveType) : true,
      
      // Touch
      touchSupport: 'ontouchstart' in window,
      
      // Performance
      highPerformance: navigator.hardwareConcurrency ? navigator.hardwareConcurrency >= 4 : true,
      
      // Screen
      highDPI: window.devicePixelRatio > 1,
      largeScreen: window.innerWidth >= 1024,
      
      // Features
      intersectionObserver: 'IntersectionObserver' in window,
      resizeObserver: 'ResizeObserver' in window,
      requestIdleCallback: 'requestIdleCallback' in window
    };
    
    return this.capabilities;
  }

  /**
   * Check if device supports high-performance features
   */
  static supportsHighPerformance(): boolean {
    const caps = this.detect();
    return caps.webgl && caps.highMemory && caps.highPerformance;
  }

  /**
   * Check if device is mobile
   */
  static isMobile(): boolean {
    const caps = this.detect();
    return caps.touchSupport && !caps.largeScreen;
  }
}