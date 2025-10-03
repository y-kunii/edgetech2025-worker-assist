/**
 * Animation Manager for optimized animations and visual effects
 */

import { AnimationOptimizer, DeviceCapabilities, PerformanceMonitor } from './performanceOptimizer';

export interface AnimationConfig {
  enableAnimations: boolean;
  reducedMotion: boolean;
  performanceMode: 'high' | 'medium' | 'low';
  maxConcurrentAnimations: number;
}

export class AnimationManager {
  private static instance: AnimationManager;
  private config: AnimationConfig;
  private activeAnimations: Map<string, Animation> = new Map();
  private animationQueue: Array<() => void> = [];
  private performanceMonitor: PerformanceMonitor;
  private frameCallback: (() => void) | null = null;

  private constructor() {
    this.performanceMonitor = PerformanceMonitor.getInstance();
    this.config = this.detectOptimalConfig();
    this.setupFrameCallback();
  }

  static getInstance(): AnimationManager {
    if (!AnimationManager.instance) {
      AnimationManager.instance = new AnimationManager();
    }
    return AnimationManager.instance;
  }

  /**
   * Detect optimal animation configuration based on device capabilities
   */
  private detectOptimalConfig(): AnimationConfig {
    const capabilities = DeviceCapabilities.detect();
    const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

    let performanceMode: 'high' | 'medium' | 'low' = 'medium';
    let maxConcurrentAnimations = 5;

    if (capabilities.highPerformance && capabilities.webgl && capabilities.highMemory) {
      performanceMode = 'high';
      maxConcurrentAnimations = 10;
    } else if (capabilities.touchSupport || !capabilities.highMemory) {
      performanceMode = 'low';
      maxConcurrentAnimations = 3;
    }

    return {
      enableAnimations: !prefersReducedMotion,
      reducedMotion: prefersReducedMotion,
      performanceMode,
      maxConcurrentAnimations
    };
  }

  /**
   * Setup frame callback for performance monitoring
   */
  private setupFrameCallback(): void {
    this.frameCallback = () => {
      this.performanceMonitor.recordFrameRate();
      this.performanceMonitor.recordMemoryUsage();
    };
    
    AnimationOptimizer.addCallback(this.frameCallback);
  }

  /**
   * Create an optimized fade animation
   */
  public createFadeAnimation(
    element: HTMLElement,
    options: {
      from?: number;
      to?: number;
      duration?: number;
      easing?: string;
      onComplete?: () => void;
    } = {}
  ): string {
    const animationId = `fade-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    
    if (!this.config.enableAnimations) {
      // Skip animation, apply final state immediately
      element.style.opacity = (options.to ?? 1).toString();
      options.onComplete?.();
      return animationId;
    }

    const {
      from = (() => {
        try {
          return parseFloat(getComputedStyle(element).opacity) || 0;
        } catch {
          return 0;
        }
      })(),
      to = 1,
      duration = this.getOptimizedDuration(300),
      easing = 'ease-out',
      onComplete
    } = options;

    if (this.activeAnimations.size >= this.config.maxConcurrentAnimations) {
      this.queueAnimation(() => this.createFadeAnimation(element, options));
      return animationId;
    }

    try {
      const animation = element.animate([
        { opacity: from },
        { opacity: to }
      ], {
        duration,
        easing,
        fill: 'forwards'
      });

      this.activeAnimations.set(animationId, animation);

      animation.addEventListener('finish', () => {
        this.activeAnimations.delete(animationId);
        onComplete?.();
        this.processQueue();
      });

      animation.addEventListener('cancel', () => {
        this.activeAnimations.delete(animationId);
        this.processQueue();
      });
    } catch (error) {
      console.warn('Animation creation failed:', error);
      // Apply final state immediately
      element.style.opacity = to.toString();
      onComplete?.();
    }

    return animationId;
  }

  /**
   * Create an optimized scale animation
   */
  public createScaleAnimation(
    element: HTMLElement,
    options: {
      from?: number;
      to?: number;
      duration?: number;
      easing?: string;
      onComplete?: () => void;
    } = {}
  ): string {
    const animationId = `scale-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    
    if (!this.config.enableAnimations) {
      element.style.transform = `scale(${options.to ?? 1})`;
      options.onComplete?.();
      return animationId;
    }

    const {
      from = 1,
      to = 1.1,
      duration = this.getOptimizedDuration(200),
      easing = 'ease-out',
      onComplete
    } = options;

    if (this.activeAnimations.size >= this.config.maxConcurrentAnimations) {
      this.queueAnimation(() => this.createScaleAnimation(element, options));
      return animationId;
    }

    const animation = element.animate([
      { transform: `scale(${from})` },
      { transform: `scale(${to})` }
    ], {
      duration,
      easing,
      fill: 'forwards'
    });

    this.activeAnimations.set(animationId, animation);

    animation.addEventListener('finish', () => {
      this.activeAnimations.delete(animationId);
      onComplete?.();
      this.processQueue();
    });

    animation.addEventListener('cancel', () => {
      this.activeAnimations.delete(animationId);
      this.processQueue();
    });

    return animationId;
  }

  /**
   * Create an optimized pulse animation
   */
  public createPulseAnimation(
    element: HTMLElement,
    options: {
      scale?: number;
      duration?: number;
      iterations?: number;
      onComplete?: () => void;
    } = {}
  ): string {
    const animationId = `pulse-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    
    if (!this.config.enableAnimations) {
      options.onComplete?.();
      return animationId;
    }

    const {
      scale = 1.05,
      duration = this.getOptimizedDuration(1000),
      iterations = Infinity,
      onComplete
    } = options;

    if (this.activeAnimations.size >= this.config.maxConcurrentAnimations) {
      this.queueAnimation(() => this.createPulseAnimation(element, options));
      return animationId;
    }

    const animation = element.animate([
      { transform: 'scale(1)' },
      { transform: `scale(${scale})` },
      { transform: 'scale(1)' }
    ], {
      duration,
      iterations,
      easing: 'ease-in-out'
    });

    this.activeAnimations.set(animationId, animation);

    animation.addEventListener('finish', () => {
      this.activeAnimations.delete(animationId);
      onComplete?.();
      this.processQueue();
    });

    animation.addEventListener('cancel', () => {
      this.activeAnimations.delete(animationId);
      this.processQueue();
    });

    return animationId;
  }

  /**
   * Create an optimized slide animation
   */
  public createSlideAnimation(
    element: HTMLElement,
    options: {
      direction: 'up' | 'down' | 'left' | 'right';
      distance?: number;
      duration?: number;
      easing?: string;
      onComplete?: () => void;
    }
  ): string {
    const animationId = `slide-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    
    if (!this.config.enableAnimations) {
      options.onComplete?.();
      return animationId;
    }

    const {
      direction,
      distance = 20,
      duration = this.getOptimizedDuration(300),
      easing = 'ease-out',
      onComplete
    } = options;

    if (this.activeAnimations.size >= this.config.maxConcurrentAnimations) {
      this.queueAnimation(() => this.createSlideAnimation(element, options));
      return animationId;
    }

    let fromTransform = '';
    let toTransform = '';

    switch (direction) {
      case 'up':
        fromTransform = `translateY(${distance}px)`;
        toTransform = 'translateY(0)';
        break;
      case 'down':
        fromTransform = `translateY(-${distance}px)`;
        toTransform = 'translateY(0)';
        break;
      case 'left':
        fromTransform = `translateX(${distance}px)`;
        toTransform = 'translateX(0)';
        break;
      case 'right':
        fromTransform = `translateX(-${distance}px)`;
        toTransform = 'translateX(0)';
        break;
    }

    const animation = element.animate([
      { transform: fromTransform, opacity: 0 },
      { transform: toTransform, opacity: 1 }
    ], {
      duration,
      easing,
      fill: 'forwards'
    });

    this.activeAnimations.set(animationId, animation);

    animation.addEventListener('finish', () => {
      this.activeAnimations.delete(animationId);
      onComplete?.();
      this.processQueue();
    });

    animation.addEventListener('cancel', () => {
      this.activeAnimations.delete(animationId);
      this.processQueue();
    });

    return animationId;
  }

  /**
   * Create celebration animation for threshold achievement
   */
  public createCelebrationAnimation(
    element: HTMLElement,
    options: {
      duration?: number;
      onComplete?: () => void;
    } = {}
  ): string {
    const animationId = `celebration-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    
    if (!this.config.enableAnimations || this.config.performanceMode === 'low') {
      // Simple flash effect for low performance
      element.style.backgroundColor = '#4CAF50';
      setTimeout(() => {
        element.style.backgroundColor = '';
        options.onComplete?.();
      }, 100);
      return animationId;
    }

    const {
      duration = this.getOptimizedDuration(1500),
      onComplete
    } = options;

    if (this.activeAnimations.size >= this.config.maxConcurrentAnimations) {
      this.queueAnimation(() => this.createCelebrationAnimation(element, options));
      return animationId;
    }

    // Multi-stage celebration animation
    const animation = element.animate([
      { transform: 'scale(1)', backgroundColor: 'transparent', boxShadow: '0 0 0 rgba(76, 175, 80, 0)' },
      { transform: 'scale(1.1)', backgroundColor: 'rgba(76, 175, 80, 0.2)', boxShadow: '0 0 20px rgba(76, 175, 80, 0.5)' },
      { transform: 'scale(1.05)', backgroundColor: 'rgba(76, 175, 80, 0.1)', boxShadow: '0 0 15px rgba(76, 175, 80, 0.3)' },
      { transform: 'scale(1)', backgroundColor: 'transparent', boxShadow: '0 0 0 rgba(76, 175, 80, 0)' }
    ], {
      duration,
      easing: 'ease-in-out'
    });

    this.activeAnimations.set(animationId, animation);

    animation.addEventListener('finish', () => {
      this.activeAnimations.delete(animationId);
      onComplete?.();
      this.processQueue();
    });

    animation.addEventListener('cancel', () => {
      this.activeAnimations.delete(animationId);
      this.processQueue();
    });

    return animationId;
  }

  /**
   * Cancel an animation
   */
  public cancelAnimation(animationId: string): void {
    const animation = this.activeAnimations.get(animationId);
    if (animation) {
      animation.cancel();
    }
  }

  /**
   * Cancel all animations
   */
  public cancelAllAnimations(): void {
    this.activeAnimations.forEach(animation => animation.cancel());
    this.activeAnimations.clear();
    this.animationQueue.length = 0;
  }

  /**
   * Queue an animation for later execution
   */
  private queueAnimation(animationFn: () => void): void {
    this.animationQueue.push(animationFn);
  }

  /**
   * Process queued animations
   */
  private processQueue(): void {
    if (this.animationQueue.length > 0 && this.activeAnimations.size < this.config.maxConcurrentAnimations) {
      const nextAnimation = this.animationQueue.shift();
      if (nextAnimation) {
        nextAnimation();
      }
    }
  }

  /**
   * Get optimized duration based on performance mode
   */
  private getOptimizedDuration(baseDuration: number): number {
    if (this.config.reducedMotion) {
      return Math.min(baseDuration * 0.3, 100); // Very short animations
    }

    switch (this.config.performanceMode) {
      case 'low':
        return baseDuration * 0.5; // Faster animations
      case 'high':
        return baseDuration * 1.2; // Slightly longer, smoother animations
      default:
        return baseDuration;
    }
  }

  /**
   * Update configuration
   */
  public updateConfig(newConfig: Partial<AnimationConfig>): void {
    this.config = { ...this.config, ...newConfig };
  }

  /**
   * Get current configuration
   */
  public getConfig(): AnimationConfig {
    return { ...this.config };
  }

  /**
   * Get animation statistics
   */
  public getStatistics(): {
    activeAnimations: number;
    queuedAnimations: number;
    performanceMode: string;
    animationsEnabled: boolean;
  } {
    return {
      activeAnimations: this.activeAnimations.size,
      queuedAnimations: this.animationQueue.length,
      performanceMode: this.config.performanceMode,
      animationsEnabled: this.config.enableAnimations
    };
  }

  /**
   * Destroy the animation manager
   */
  public destroy(): void {
    this.cancelAllAnimations();
    
    if (this.frameCallback) {
      AnimationOptimizer.removeCallback(this.frameCallback);
    }
  }
}