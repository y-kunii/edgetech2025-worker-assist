import express, { Request, Response } from 'express';
import { createServer } from 'http';
import { Server } from 'socket.io';
import { MockDataGenerator } from './mockDataGenerator';

const app = express();
const server = createServer(app);
const io = new Server(server, {
  cors: {
    origin: "*",
    methods: ["GET", "POST"]
  }
});

const PORT = process.env.PORT || 3001;
const mockDataGenerator = new MockDataGenerator();

// HTTP endpoint for health check
app.get('/health', (req: Request, res: Response) => {
  res.json({ status: 'OK', message: 'Mock WebSocket Server is running' });
});

// WebSocket connection handling
io.on('connection', (socket) => {
  console.log(`Client connected: ${socket.id}`);
  
  // Send initial data immediately
  socket.emit('sensor_data', mockDataGenerator.generateData());
  
  // Send data every 2 seconds
  const dataInterval = setInterval(() => {
    const data = mockDataGenerator.generateData();
    socket.emit('sensor_data', data);
    console.log(`Sent data to ${socket.id}:`, {
      timestamp: data.timestamp,
      workerStatus: data.data.worker_status,
      robotStatus: data.data.robot_status,
      screwCount: data.data.screw_count,
      boltCount: data.data.bolt_count
    });
  }, 2000);
  
  // Handle robot commands from client
  socket.on('robot_command', (command) => {
    console.log(`Received robot command from ${socket.id}:`, command);
    
    // Simulate command processing
    setTimeout(() => {
      socket.emit('robot_command_response', {
        commandId: command.id,
        status: 'success',
        message: 'Command executed successfully',
        timestamp: new Date().toISOString()
      });
    }, 500);
  });
  
  // Handle client disconnect
  socket.on('disconnect', () => {
    console.log(`Client disconnected: ${socket.id}`);
    clearInterval(dataInterval);
  });
});

server.listen(PORT, () => {
  console.log(`Mock WebSocket Server running on port ${PORT}`);
  console.log(`Health check available at http://localhost:${PORT}/health`);
});