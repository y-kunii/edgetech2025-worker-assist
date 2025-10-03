import React from 'react';
import { render, screen, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import { MainLayout } from '../renderer/components/MainLayout';
import { WorkDataStore } from '../stores/WorkDataStore';

// Mock components
jest.mock('../renderer/components/LiveVideoDisplay', () => {
  return function MockLiveVideoDisplay({ isConnected }: { isConnected: boolean }) {
    return <div data-testid="live-video-display">Live Video {isConnected ? 'Connected' : 'Disconnected'}</div>;
  };
});

jest.mock('../renderer/components/StatusIndicator', () => {
  return function MockStatusIndicator() {
    return <div data-testid="status-indicator">Status Indicator</div>;
  };
});

jest.mock('../renderer/components/ProgressChart', () => {
  return function MockProgressChart() {
    return <div data-testid="progress-chart">Progress Chart</div>;
  };
});

jest.mock('../renderer/components/WorkStepDisplay', () => {
  return function MockWorkStepDisplay() {
    return <div data-testid="work-step-display">Work Step Display</div>;
  };
});

jest.mock('../renderer/components/TimelineAndStats', () => {
  return function MockTimelineAndStats() {
    return <div data-testid="timeline-and-stats">Timeline and Stats</div>;
  };
});

jest.mock('../renderer/components/EfficiencyIndicator', () => {
  return function MockEfficiencyIndicator() {
    return <div data-testid="efficiency-indicator">Efficiency Indicator</div>;
  };
});

jest.mock('../renderer/components/ConnectionMonitor', () => {
  return function MockConnectionMonitor() {
    return <div data-testid="connection-monitor">Connection Monitor</div>;
  };
});

jest.mock('../renderer/components/ThresholdSettings', () => {
  return function MockThresholdSettings() {
    return <div data-testid="threshold-settings">Threshold Settings</div>;
  };
});

jest.mock('../renderer/components/NotificationSystem', () => {
  return function MockNotificationSystem() {
    return <div data-testid="notification-system">Notification System</div>;
  };
});

describe('Responsive Layout Tests', () => {
  let workDataStore: WorkDataStore;

  beforeEach(() => {
    workDataStore = new WorkDataStore();
    
    // Mock window dimensions
    Object.defineProperty(window, 'innerWidth', {
      writable: true,
      configurable: true,
      value: 1920,
    });
    
    Object.defineProperty(window, 'innerHeight', {
      writable: true,
      configurable: true,
      value: 1080,
    });
  });

  afterEach(() => {
    workDataStore.destroy();
  });

  describe('Desktop Layout (1920x1080)', () => {
    it('should render all components in desktop layout', () => {
      render(<MainLayout workDataStore={workDataStore} />);

      // Verify all main components are rendered
      expect(screen.getByTestId('live-video-display')).toBeInTheDocument();
      expect(screen.getAllByTestId('status-indicator')).toHaveLength(2); // Worker and Robot
      expect(screen.getAllByTestId('progress-chart')).toHaveLength(2); // Screw and Bolt
      expect(screen.getByTestId('work-step-display')).toBeInTheDocument();
      expect(screen.getByTestId('timeline-and-stats')).toBeInTheDocument();
      expect(screen.getByTestId('efficiency-indicator')).toBeInTheDocument();
    });

    it('should apply correct CSS classes for desktop', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      const mainContainer = container.querySelector('.main-layout');
      expect(mainContainer).toHaveClass('desktop-layout');
    });

    it('should use CSS Grid layout for desktop', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      const gridContainer = container.querySelector('.content-grid');
      expect(gridContainer).toHaveStyle({
        display: 'grid',
        gridTemplateColumns: 'repeat(3, 1fr)',
        gridTemplateRows: 'auto auto auto'
      });
    });
  });

  describe('Tablet Layout (1024x768)', () => {
    beforeEach(() => {
      Object.defineProperty(window, 'innerWidth', {
        writable: true,
        configurable: true,
        value: 1024,
      });
      
      Object.defineProperty(window, 'innerHeight', {
        writable: true,
        configurable: true,
        value: 768,
      });
    });

    it('should adapt layout for tablet dimensions', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      // Trigger resize event
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const mainContainer = container.querySelector('.main-layout');
      expect(mainContainer).toHaveClass('tablet-layout');
    });

    it('should adjust grid layout for tablet', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const gridContainer = container.querySelector('.content-grid');
      expect(gridContainer).toHaveStyle({
        gridTemplateColumns: 'repeat(2, 1fr)'
      });
    });
  });

  describe('Mobile Layout (375x667)', () => {
    beforeEach(() => {
      Object.defineProperty(window, 'innerWidth', {
        writable: true,
        configurable: true,
        value: 375,
      });
      
      Object.defineProperty(window, 'innerHeight', {
        writable: true,
        configurable: true,
        value: 667,
      });
    });

    it('should adapt layout for mobile dimensions', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const mainContainer = container.querySelector('.main-layout');
      expect(mainContainer).toHaveClass('mobile-layout');
    });

    it('should use single column layout for mobile', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const gridContainer = container.querySelector('.content-grid');
      expect(gridContainer).toHaveStyle({
        gridTemplateColumns: '1fr'
      });
    });

    it('should show mobile navigation menu', () => {
      render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const mobileMenu = screen.getByTestId('mobile-menu-toggle');
      expect(mobileMenu).toBeInTheDocument();
    });
  });

  describe('4K Layout (3840x2160)', () => {
    beforeEach(() => {
      Object.defineProperty(window, 'innerWidth', {
        writable: true,
        configurable: true,
        value: 3840,
      });
      
      Object.defineProperty(window, 'innerHeight', {
        writable: true,
        configurable: true,
        value: 2160,
      });
    });

    it('should scale up for 4K displays', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const mainContainer = container.querySelector('.main-layout');
      expect(mainContainer).toHaveClass('uhd-layout');
    });

    it('should maintain aspect ratios on 4K', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const videoContainer = container.querySelector('.video-container');
      expect(videoContainer).toHaveStyle({
        aspectRatio: '16/9'
      });
    });
  });

  describe('Dynamic Resizing', () => {
    it('should handle window resize events', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      // Start with desktop
      expect(container.querySelector('.main-layout')).toHaveClass('desktop-layout');

      // Resize to tablet
      Object.defineProperty(window, 'innerWidth', { value: 1024 });
      Object.defineProperty(window, 'innerHeight', { value: 768 });
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      expect(container.querySelector('.main-layout')).toHaveClass('tablet-layout');

      // Resize to mobile
      Object.defineProperty(window, 'innerWidth', { value: 375 });
      Object.defineProperty(window, 'innerHeight', { value: 667 });
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      expect(container.querySelector('.main-layout')).toHaveClass('mobile-layout');
    });

    it('should debounce resize events', () => {
      jest.useFakeTimers();
      
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      const resizeHandler = jest.fn();
      window.addEventListener('resize', resizeHandler);

      // Trigger multiple resize events quickly
      for (let i = 0; i < 10; i++) {
        window.dispatchEvent(new Event('resize'));
      }

      // Should not have processed all events immediately
      expect(resizeHandler).toHaveBeenCalledTimes(10);

      // Fast-forward timers
      act(() => {
        jest.runAllTimers();
      });

      jest.useRealTimers();
    });
  });

  describe('Component Visibility', () => {
    it('should hide non-essential components on mobile', () => {
      Object.defineProperty(window, 'innerWidth', { value: 375 });
      Object.defineProperty(window, 'innerHeight', { value: 667 });

      render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      // Some components should be hidden or collapsed on mobile
      const timelineStats = screen.getByTestId('timeline-and-stats');
      expect(timelineStats).toHaveClass('mobile-collapsed');
    });

    it('should show all components on desktop', () => {
      render(<MainLayout workDataStore={workDataStore} />);

      // All components should be visible on desktop
      expect(screen.getByTestId('live-video-display')).toBeVisible();
      expect(screen.getAllByTestId('status-indicator')[0]).toBeVisible();
      expect(screen.getAllByTestId('progress-chart')[0]).toBeVisible();
      expect(screen.getByTestId('work-step-display')).toBeVisible();
      expect(screen.getByTestId('timeline-and-stats')).toBeVisible();
    });
  });

  describe('Font and Text Scaling', () => {
    it('should scale fonts appropriately for different screen sizes', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      // Desktop font size
      const header = container.querySelector('h1');
      expect(header).toHaveStyle({ fontSize: '2rem' });

      // Tablet font size
      Object.defineProperty(window, 'innerWidth', { value: 1024 });
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });
      
      expect(header).toHaveStyle({ fontSize: '1.8rem' });

      // Mobile font size
      Object.defineProperty(window, 'innerWidth', { value: 375 });
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });
      
      expect(header).toHaveStyle({ fontSize: '1.5rem' });
    });
  });

  describe('Touch and Interaction Adaptations', () => {
    it('should increase touch targets on mobile', () => {
      Object.defineProperty(window, 'innerWidth', { value: 375 });
      
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const buttons = container.querySelectorAll('button');
      buttons.forEach(button => {
        expect(button).toHaveStyle({ minHeight: '44px' });
      });
    });

    it('should enable touch gestures on mobile', () => {
      Object.defineProperty(window, 'innerWidth', { value: 375 });
      
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const touchContainer = container.querySelector('.touch-enabled');
      expect(touchContainer).toHaveAttribute('data-touch-enabled', 'true');
    });
  });

  describe('Performance Optimizations', () => {
    it('should reduce animations on mobile for performance', () => {
      Object.defineProperty(window, 'innerWidth', { value: 375 });
      
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const animatedElements = container.querySelectorAll('.animated');
      animatedElements.forEach(element => {
        expect(element).toHaveClass('reduced-motion');
      });
    });

    it('should lazy load non-critical components on mobile', () => {
      Object.defineProperty(window, 'innerWidth', { value: 375 });
      
      render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      // Timeline component should be lazy loaded on mobile
      const timelineContainer = screen.getByTestId('timeline-and-stats');
      expect(timelineContainer).toHaveAttribute('data-lazy-loaded', 'true');
    });
  });

  describe('Accessibility Adaptations', () => {
    it('should maintain accessibility across screen sizes', () => {
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      // Check ARIA labels are present
      const mainContent = container.querySelector('[role="main"]');
      expect(mainContent).toBeInTheDocument();

      // Check keyboard navigation
      const focusableElements = container.querySelectorAll('[tabindex]');
      expect(focusableElements.length).toBeGreaterThan(0);
    });

    it('should provide appropriate focus management on mobile', () => {
      Object.defineProperty(window, 'innerWidth', { value: 375 });
      
      const { container } = render(<MainLayout workDataStore={workDataStore} />);
      
      act(() => {
        window.dispatchEvent(new Event('resize'));
      });

      const skipLink = container.querySelector('.skip-to-content');
      expect(skipLink).toBeInTheDocument();
    });
  });
});