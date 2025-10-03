import React from 'react';
import { render, screen } from '@testing-library/react';
import StatusIndicator from '../renderer/components/StatusIndicator';
import { WorkerStatus, RobotStatus } from '../types';

describe('StatusIndicator Component', () => {
  describe('Worker Status Display', () => {
    it('should render worker waiting status correctly', () => {
      render(
        <StatusIndicator 
          type="worker" 
          status="waiting" as WorkerStatus
          isActive={false} 
        />
      );
      
      expect(screen.getByText('👤 作業者状態')).toBeInTheDocument();
      expect(screen.getByText('待機中')).toBeInTheDocument();
      expect(screen.getByText('⏸️ 非アクティブ')).toBeInTheDocument();
    });

    it('should render worker screw tightening status with active state', () => {
      render(
        <StatusIndicator 
          type="worker" 
          status="screw_tightening" as WorkerStatus
          isActive={true} 
        />
      );
      
      expect(screen.getByText('👤 作業者状態')).toBeInTheDocument();
      expect(screen.getByText('ネジ締め中')).toBeInTheDocument();
      expect(screen.getByText('⚡ アクティブ')).toBeInTheDocument();
    });

    it('should render worker bolt tightening status', () => {
      render(
        <StatusIndicator 
          type="worker" 
          status="bolt_tightening" as WorkerStatus
          isActive={true} 
        />
      );
      
      expect(screen.getByText('ボルト締め中')).toBeInTheDocument();
    });

    it('should render worker tool handover status', () => {
      render(
        <StatusIndicator 
          type="worker" 
          status="tool_handover" as WorkerStatus
          isActive={true} 
        />
      );
      
      expect(screen.getByText('工具受け渡し')).toBeInTheDocument();
    });

    it('should render worker absent status', () => {
      render(
        <StatusIndicator 
          type="worker" 
          status="absent" as WorkerStatus
          isActive={false} 
        />
      );
      
      expect(screen.getByText('不在')).toBeInTheDocument();
    });

    it('should show activity indicator when worker is active', () => {
      render(
        <StatusIndicator 
          type="worker" 
          status="screw_tightening" as WorkerStatus
          isActive={true} 
        />
      );
      
      const activityIndicator = screen.getByText('⚡ アクティブ');
      expect(activityIndicator).toBeInTheDocument();
      expect(activityIndicator).toHaveClass('status-connected');
    });
  });

  describe('Robot Status Display', () => {
    it('should render robot waiting status with closed grip', () => {
      const robotStatus: RobotStatus = { state: 'waiting', grip: 'closed' };
      
      render(
        <StatusIndicator 
          type="robot" 
          status={robotStatus}
          isActive={false} 
        />
      );
      
      expect(screen.getByText('🤖 ロボット状態')).toBeInTheDocument();
      expect(screen.getByText('待機中')).toBeInTheDocument();
      expect(screen.getByText('グリップ閉')).toBeInTheDocument();
      expect(screen.getByText('⏸️ 非アクティブ')).toBeInTheDocument();
    });

    it('should render robot operating status with open grip', () => {
      const robotStatus: RobotStatus = { state: 'operating', grip: 'open' };
      
      render(
        <StatusIndicator 
          type="robot" 
          status={robotStatus}
          isActive={true} 
        />
      );
      
      expect(screen.getByText('🤖 ロボット状態')).toBeInTheDocument();
      expect(screen.getByText('稼働中')).toBeInTheDocument();
      expect(screen.getByText('グリップ開')).toBeInTheDocument();
      expect(screen.getByText('⚡ アクティブ')).toBeInTheDocument();
    });

    it('should show activity indicator when robot is active', () => {
      const robotStatus: RobotStatus = { state: 'operating', grip: 'closed' };
      
      render(
        <StatusIndicator 
          type="robot" 
          status={robotStatus}
          isActive={true} 
        />
      );
      
      const activityIndicator = screen.getByText('⚡ アクティブ');
      expect(activityIndicator).toBeInTheDocument();
      expect(activityIndicator).toHaveClass('status-connected');
    });
  });

  describe('Visual Elements', () => {
    it('should render SVG illustrations for worker status', () => {
      const { container } = render(
        <StatusIndicator 
          type="worker" 
          status="screw_tightening" as WorkerStatus
          isActive={true} 
        />
      );
      
      const svgElement = container.querySelector('svg.status-illustration');
      expect(svgElement).toBeInTheDocument();
      expect(svgElement).toHaveClass('status-illustration');
    });

    it('should render SVG illustrations for robot status', () => {
      const robotStatus: RobotStatus = { state: 'operating', grip: 'open' };
      
      const { container } = render(
        <StatusIndicator 
          type="robot" 
          status={robotStatus}
          isActive={true} 
        />
      );
      
      const svgElement = container.querySelector('svg.status-illustration');
      expect(svgElement).toBeInTheDocument();
      expect(svgElement).toHaveClass('status-illustration');
    });

    it('should apply active class to illustration container when active', () => {
      const { container } = render(
        <StatusIndicator 
          type="worker" 
          status="screw_tightening" as WorkerStatus
          isActive={true} 
        />
      );
      
      const illustrationContainer = container.querySelector('.status-illustration-container');
      expect(illustrationContainer).toHaveClass('active');
    });

    it('should not apply active class when inactive', () => {
      const { container } = render(
        <StatusIndicator 
          type="worker" 
          status="waiting" as WorkerStatus
          isActive={false} 
        />
      );
      
      const illustrationContainer = container.querySelector('.status-illustration-container');
      expect(illustrationContainer).not.toHaveClass('active');
    });
  });

  describe('Animation and Styling', () => {
    it('should show activity dots when active', () => {
      const { container } = render(
        <StatusIndicator 
          type="worker" 
          status="screw_tightening" as WorkerStatus
          isActive={true} 
        />
      );
      
      const activityDots = container.querySelector('.activity-dots');
      expect(activityDots).toBeInTheDocument();
      
      const dots = container.querySelectorAll('.activity-dots .dot');
      expect(dots).toHaveLength(3);
    });

    it('should not show activity dots when inactive', () => {
      const { container } = render(
        <StatusIndicator 
          type="worker" 
          status="waiting" as WorkerStatus
          isActive={false} 
        />
      );
      
      const activityDots = container.querySelector('.activity-dots');
      expect(activityDots).not.toBeInTheDocument();
    });
  });
});