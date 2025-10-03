import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import MainLayout from './components/MainLayout';
import ErrorBoundary from './components/ErrorBoundary';
import { WorkerStatus, RobotStatus, WorkHistory, WorkStatistics, EfficiencyMetrics } from '../types';
import { WorkStepManager } from '../utils/workStepManager';
import { WorkDataStore } from '../stores/WorkDataStore';
import { DataIntegrationManager } from '../utils/dataIntegrationManager';
import { AnimationManager } from '../utils/animationManager';
import { PerformanceMonitor, DeviceCapabilities, MemoryOptimizer } from '../utils/performanceOptimizer';
import './styles/layout.css';

const App: React.FC = () => {
  // Performance and optimization managers
  const performanceMonitorRef = useRef<PerformanceMonitor>(PerformanceMonitor.getInstance());
  const dataIntegrationManagerRef = useRef<DataIntegrationManager>(DataIntegrationManager.getInstance());
  const animationManagerRef = useRef<AnimationManager>(AnimationManager.getInstance());
  
  // Work step manager instance
  const workStepManagerRef = useRef<WorkStepManager>(new WorkStepManager());
  
  // Work data store instance
  const workDataStoreRef = useRef<WorkDataStore>(new WorkDataStore());
  
  // Device capabilities detection
  const deviceCapabilities = useMemo(() => DeviceCapabilities.detect(), []);
  
  // Mock data state for demonstration
  const [mockData, setMockData] = useState({
    isConnected: true,
    connectionQuality: 'good' as 'good' | 'fair' | 'poor',
    currentImage: undefined as string | undefined,
    workerStatus: 'waiting' as WorkerStatus,
    robotStatus: { state: 'waiting', grip: 'closed' } as RobotStatus,
    isWorkerActive: false,
    isRobotActive: false,
    screwCount: 3,
    boltCount: 1,
    currentWorkStep: 'waiting' as WorkerStatus,
    overallEfficiency: 92,
    notificationCount: 0
  });

  // Get threshold settings from store
  const [thresholdSettings, setThresholdSettings] = useState(() => {
    return workDataStoreRef.current.getState().thresholdSettings;
  });

  // Mock work history data
  const [workHistory] = useState<WorkHistory[]>([
    {
      id: '1',
      timestamp: new Date(Date.now() - 10 * 60 * 1000), // 10分前
      workerStatus: 'screw_tightening',
      duration: 30000,
      efficiency: 90
    },
    {
      id: '2',
      timestamp: new Date(Date.now() - 8 * 60 * 1000), // 8分前
      workerStatus: 'bolt_tightening',
      duration: 25000,
      efficiency: 85
    },
    {
      id: '3',
      timestamp: new Date(Date.now() - 5 * 60 * 1000), // 5分前
      workerStatus: 'tool_handover',
      duration: 5000,
      efficiency: 95
    },
    {
      id: '4',
      timestamp: new Date(Date.now() - 2 * 60 * 1000), // 2分前
      workerStatus: 'waiting',
      duration: 10000,
      efficiency: 80
    }
  ]);

  // Mock statistics data
  const [statistics] = useState<WorkStatistics>({
    totalWorkTime: 45.5, // 45分30秒
    completedTasks: 12,
    averageEfficiency: 87.5,
    errorCount: 1
  });

  // Mock efficiency metrics
  const [efficiencyMetrics] = useState<EfficiencyMetrics>({
    current: 92,
    target: 85,
    trend: 'up',
    lastUpdated: new Date()
  });

  // Performance monitoring and optimization setup
  useEffect(() => {
    const performanceMonitor = performanceMonitorRef.current;
    const dataIntegrationManager = dataIntegrationManagerRef.current;
    
    // Start performance monitoring
    performanceMonitor.startMeasure('appInitialization');
    
    // Setup memory optimization
    MemoryOptimizer.createPool('workData', () => ({
      timestamp: new Date(),
      workerStatus: 'waiting' as WorkerStatus,
      robotStatus: { state: 'waiting', grip: 'closed' } as RobotStatus,
      screwCount: 0,
      boltCount: 0
    }), 10);
    
    // Setup data integration event listeners
    const handleWorkDataUpdate = (workData: any) => {
      setMockData(prev => ({
        ...prev,
        workerStatus: workData.workerStatus,
        robotStatus: workData.robotStatus,
        screwCount: workData.screwCount,
        boltCount: workData.boltCount,
        isWorkerActive: workData.workerStatus !== 'waiting' && workData.workerStatus !== 'absent',
        isRobotActive: workData.robotStatus.state === 'operating'
      }));
    };
    
    const handleConnectionStatusUpdate = (connectionData: any) => {
      setMockData(prev => ({
        ...prev,
        isConnected: connectionData.isConnected,
        connectionQuality: connectionData.quality?.stability === 'excellent' ? 'good' :
                          connectionData.quality?.stability === 'good' ? 'good' :
                          connectionData.quality?.stability === 'fair' ? 'fair' : 'poor'
      }));
    };
    
    dataIntegrationManager.on('workDataUpdated', handleWorkDataUpdate);
    dataIntegrationManager.on('connectionStatusUpdated', handleConnectionStatusUpdate);
    
    performanceMonitor.endMeasure('appInitialization');
    
    return () => {
      dataIntegrationManager.off('workDataUpdated', handleWorkDataUpdate);
      dataIntegrationManager.off('connectionStatusUpdated', handleConnectionStatusUpdate);
    };
  }, []);

  // Threshold update handler
  const handleThresholdUpdate = useCallback((settings: any) => {
    setThresholdSettings(settings);
    dataIntegrationManagerRef.current.integrateThresholdSettings(settings);
  }, []);

  // Listen to threshold settings changes with optimization
  useEffect(() => {
    const workDataStore = workDataStoreRef.current;
    
    workDataStore.on('threshold_settings_updated', handleThresholdUpdate);
    
    return () => {
      workDataStore.off('threshold_settings_updated', handleThresholdUpdate);
    };
  }, []);

  // Optimized simulation with performance monitoring
  useEffect(() => {
    const performanceMonitor = performanceMonitorRef.current;
    const dataIntegrationManager = dataIntegrationManagerRef.current;
    
    // Adjust update frequency based on device capabilities
    const updateInterval = deviceCapabilities.highPerformance ? 1500 : 3000;
    
    const interval = setInterval(() => {
      performanceMonitor.startMeasure('mockDataUpdate');
      
      setMockData(prev => {
        // Simulate worker status changes with reduced frequency for better performance
        const workerStatuses: WorkerStatus[] = ['waiting', 'screw_tightening', 'bolt_tightening', 'tool_handover'];
        const newWorkerStatus = Math.random() > 0.85 ? 
          workerStatuses[Math.floor(Math.random() * workerStatuses.length)] : 
          prev.workerStatus;
        
        // Simulate robot status changes
        const robotStates = ['waiting', 'operating'] as const;
        const gripStates = ['open', 'closed'] as const;
        const newRobotStatus: RobotStatus = {
          state: Math.random() > 0.75 ? robotStates[Math.floor(Math.random() * robotStates.length)] : prev.robotStatus.state,
          grip: Math.random() > 0.7 ? gripStates[Math.floor(Math.random() * gripStates.length)] : prev.robotStatus.grip
        };
        
        const newData = {
          ...prev,
          workerStatus: newWorkerStatus,
          robotStatus: newRobotStatus,
          isWorkerActive: newWorkerStatus !== 'waiting' && newWorkerStatus !== 'absent',
          isRobotActive: newRobotStatus.state === 'operating',
          currentWorkStep: newWorkerStatus,
          // Simulate efficiency fluctuation with smoother changes
          overallEfficiency: Math.max(80, Math.min(100, prev.overallEfficiency + (Math.random() - 0.5) * 0.5))
        };
        
        // Integrate data through the optimization manager
        dataIntegrationManager.integrateWorkData({
          timestamp: new Date(),
          workerStatus: newWorkerStatus,
          robotStatus: newRobotStatus,
          screwCount: prev.screwCount,
          boltCount: prev.boltCount
        });
        
        performanceMonitor.endMeasure('mockDataUpdate');
        return newData;
      });
    }, updateInterval);

    return () => clearInterval(interval);
  }, [deviceCapabilities.highPerformance]);

  // Memory cleanup and performance monitoring
  useEffect(() => {
    const cleanupInterval = setInterval(() => {
      // Clean up old cached data
      dataIntegrationManagerRef.current.clearOldCache();
      
      // Clean up weak references
      MemoryOptimizer.cleanupWeakRefs();
      
      // Force garbage collection on low-memory devices
      if (!deviceCapabilities.highMemory) {
        MemoryOptimizer.forceGC();
      }
    }, 60000); // Every minute

    return () => clearInterval(cleanupInterval);
  }, [deviceCapabilities.highMemory]);

  // Memoized props to prevent unnecessary re-renders
  const mainLayoutProps = useMemo(() => ({
    isConnected: mockData.isConnected,
    connectionQuality: mockData.connectionQuality,
    currentImage: mockData.currentImage,
    workerStatus: mockData.workerStatus,
    robotStatus: mockData.robotStatus,
    isWorkerActive: mockData.isWorkerActive,
    isRobotActive: mockData.isRobotActive,
    screwCount: mockData.screwCount,
    screwTarget: thresholdSettings.screwThreshold,
    boltCount: mockData.boltCount,
    boltTarget: thresholdSettings.boltThreshold,
    currentWorkStep: mockData.currentWorkStep,
    workStepManager: workStepManagerRef.current,
    overallEfficiency: mockData.overallEfficiency,
    notificationCount: mockData.notificationCount,
    workHistory,
    statistics,
    efficiencyMetrics,
    workDataStore: workDataStoreRef.current,
    animationManager: animationManagerRef.current,
    deviceCapabilities
  }), [
    mockData.isConnected,
    mockData.connectionQuality,
    mockData.currentImage,
    mockData.workerStatus,
    mockData.robotStatus,
    mockData.isWorkerActive,
    mockData.isRobotActive,
    mockData.screwCount,
    thresholdSettings.screwThreshold,
    mockData.boltCount,
    thresholdSettings.boltThreshold,
    mockData.currentWorkStep,
    mockData.overallEfficiency,
    mockData.notificationCount,
    workHistory,
    statistics,
    efficiencyMetrics,
    deviceCapabilities
  ]);

  return (
    <ErrorBoundary>
      <MainLayout {...mainLayoutProps} />
    </ErrorBoundary>
  );
};

export default App;