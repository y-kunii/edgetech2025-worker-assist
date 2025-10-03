export type WorkerStatus = 'waiting' | 'screw_tightening' | 'tool_handover' | 'bolt_tightening' | 'absent';

export interface RobotStatus {
  state: 'waiting' | 'operating';
  grip: 'open' | 'closed';
}

export interface SensorData {
  type: string;
  timestamp: string;
  data: {
    image: string;
    worker_status: WorkerStatus;
    robot_status: RobotStatus;
    screw_count: number;
    bolt_count: number;
    work_step: WorkerStatus;
  };
}

export class MockDataGenerator {
  private screwCount = 0;
  private boltCount = 0;
  private currentWorkerStatus: WorkerStatus = 'waiting';
  private currentRobotStatus: RobotStatus = { state: 'waiting', grip: 'open' };
  private lastStatusChange = Date.now();
  private statusDuration = 5000; // 5 seconds per status
  
  private workerStatusCycle: WorkerStatus[] = [
    'waiting',
    'screw_tightening',
    'tool_handover', 
    'bolt_tightening',
    'waiting'
  ];
  
  private currentStatusIndex = 0;

  generateData(): SensorData {
    this.updateStatus();
    
    return {
      type: 'sensor_data',
      timestamp: new Date().toISOString(),
      data: {
        image: this.generateMockImage(),
        worker_status: this.currentWorkerStatus,
        robot_status: this.currentRobotStatus,
        screw_count: this.screwCount,
        bolt_count: this.boltCount,
        work_step: this.currentWorkerStatus
      }
    };
  }

  private updateStatus(): void {
    const now = Date.now();
    
    // Change status every 5 seconds
    if (now - this.lastStatusChange > this.statusDuration) {
      this.currentStatusIndex = (this.currentStatusIndex + 1) % this.workerStatusCycle.length;
      this.currentWorkerStatus = this.workerStatusCycle[this.currentStatusIndex];
      this.lastStatusChange = now;
      
      // Update counts based on status
      if (this.currentWorkerStatus === 'screw_tightening') {
        this.screwCount++;
        this.currentRobotStatus = { state: 'waiting', grip: 'open' };
      } else if (this.currentWorkerStatus === 'bolt_tightening') {
        this.boltCount++;
        this.currentRobotStatus = { state: 'waiting', grip: 'open' };
      } else if (this.currentWorkerStatus === 'tool_handover') {
        this.currentRobotStatus = { state: 'operating', grip: 'closed' };
      } else {
        this.currentRobotStatus = { state: 'waiting', grip: 'open' };
      }
      
      console.log(`Status changed to: ${this.currentWorkerStatus}, Screw: ${this.screwCount}, Bolt: ${this.boltCount}`);
    }
  }

  private generateMockImage(): string {
    // Generate a simple base64 encoded placeholder image
    // In a real scenario, this would be actual camera data with skeleton overlay
    const canvas = this.createMockCanvas();
    return canvas;
  }

  private createMockCanvas(): string {
    // Create a simple base64 encoded SVG as mock image data
    const svg = `
      <svg width="640" height="480" xmlns="http://www.w3.org/2000/svg">
        <rect width="640" height="480" fill="#f0f0f0"/>
        <text x="320" y="240" text-anchor="middle" font-family="Arial" font-size="24" fill="#333">
          Mock Camera Feed
        </text>
        <text x="320" y="280" text-anchor="middle" font-family="Arial" font-size="16" fill="#666">
          Worker Status: ${this.currentWorkerStatus}
        </text>
        <text x="320" y="310" text-anchor="middle" font-family="Arial" font-size="16" fill="#666">
          Robot: ${this.currentRobotStatus.state} (${this.currentRobotStatus.grip})
        </text>
        <circle cx="320" cy="200" r="30" fill="#2196F3" opacity="0.7"/>
        <line x1="320" y1="230" x2="320" y2="350" stroke="#2196F3" stroke-width="3"/>
        <line x1="290" y1="260" x2="350" y2="260" stroke="#2196F3" stroke-width="3"/>
        <line x1="320" y1="350" x2="290" y2="420" stroke="#2196F3" stroke-width="3"/>
        <line x1="320" y1="350" x2="350" y2="420" stroke="#2196F3" stroke-width="3"/>
      </svg>
    `;
    
    return `data:image/svg+xml;base64,${Buffer.from(svg).toString('base64')}`;
  }

  // Method to reset counts for testing
  resetCounts(): void {
    this.screwCount = 0;
    this.boltCount = 0;
    console.log('Counts reset to 0');
  }

  // Method to set specific status for testing
  setWorkerStatus(status: WorkerStatus): void {
    this.currentWorkerStatus = status;
    this.currentStatusIndex = this.workerStatusCycle.indexOf(status);
    console.log(`Worker status manually set to: ${status}`);
  }
}