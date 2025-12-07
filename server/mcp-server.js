// server/mcp-server.js
// MCP (Model Context Protocol) Server for Interactive Robotics Demos
const WebSocket = require('ws');
const http = require('http');
const fs = require('fs');
const path = require('path');

class MCPServer {
  constructor(port = 8080) {
    this.port = port;
    this.wss = null;
    this.clients = new Set();
  }

  start() {
    // Create HTTP server for serving static files
    const server = http.createServer((req, res) => {
      if (req.url === '/' || req.url === '/mcp') {
        // Serve MCP interface
        res.writeHead(200, { 'Content-Type': 'text/html' });
        res.end(this.getMCPInterfaceHTML());
      } else {
        // 404 for other routes
        res.writeHead(404);
        res.end('Not Found');
      }
    });

    // Create WebSocket server
    this.wss = new WebSocket.Server({ server });

    this.wss.on('connection', (ws, req) => {
      console.log('New MCP client connected');
      this.clients.add(ws);

      ws.on('message', (message) => {
        console.log('Received message:', message.toString());
        try {
          const data = JSON.parse(message.toString());
          const response = this.handleMCPRequest(data);
          ws.send(JSON.stringify(response));
        } catch (error) {
          console.error('Error processing MCP message:', error);
          ws.send(JSON.stringify({
            error: 'Invalid JSON message',
            details: error.message
          }));
        }
      });

      ws.on('close', () => {
        console.log('MCP client disconnected');
        this.clients.delete(ws);
      });

      ws.on('error', (error) => {
        console.error('WebSocket error:', error);
        this.clients.delete(ws);
      });
    });

    server.listen(this.port, () => {
      console.log(`MCP Server running on ws://localhost:${this.port}`);
    });
  }

  handleMCPRequest(data) {
    // Handle different types of MCP requests
    switch (data.method) {
      case 'robot.get_state':
        return this.handleGetRobotState(data);
      case 'robot.set_joint_angles':
        return this.handleSetJointAngles(data);
      case 'robot.simulate_step':
        return this.handleSimulateStep(data);
      case 'robot.balance_control':
        return this.handleBalanceControl(data);
      case 'visualization.update':
        return this.handleVisualizationUpdate(data);
      default:
        return {
          error: 'Unknown method',
          method: data.method
        };
    }
  }

  handleGetRobotState(data) {
    // Return current robot state
    return {
      id: data.id,
      result: {
        joint_angles: [0.1, 0.2, 0.3, -0.1, -0.2, -0.3], // Example joint angles
        position: { x: 0, y: 0, z: 0.8 },
        orientation: { w: 1, x: 0, y: 0, z: 0 },
        sensors: {
          imu: { roll: 0.01, pitch: -0.02, yaw: 0.005 },
          force_torque: { left_foot: [10, 5, 200], right_foot: [12, -3, 195] },
          encoders: [0.1, 0.2, 0.3, -0.1, -0.2, -0.3]
        },
        status: 'running'
      }
    };
  }

  handleSetJointAngles(data) {
    // Set joint angles for the robot
    const { joint_angles } = data.params || {};
    if (!Array.isArray(joint_angles)) {
      return {
        error: 'Invalid joint angles',
        id: data.id
      };
    }

    // In a real implementation, this would send commands to the robot
    console.log('Setting joint angles:', joint_angles);

    return {
      id: data.id,
      result: {
        success: true,
        message: 'Joint angles set successfully',
        new_angles: joint_angles
      }
    };
  }

  handleSimulateStep(data) {
    // Simulate one step of robot dynamics
    const { time_step = 0.01 } = data.params || {};

    // In a real implementation, this would run physics simulation
    console.log(`Simulating step with time step: ${time_step}s`);

    return {
      id: data.id,
      result: {
        success: true,
        time_step: time_step,
        new_state: this.handleGetRobotState({}).result
      }
    };
  }

  handleBalanceControl(data) {
    // Handle balance control commands
    const { target_com, control_mode = 'zmp' } = data.params || {};

    // In a real implementation, this would run balance control algorithms
    console.log(`Running ${control_mode} balance control with target CoM:`, target_com);

    return {
      id: data.id,
      result: {
        success: true,
        control_mode: control_mode,
        message: 'Balance control command processed'
      }
    };
  }

  handleVisualizationUpdate(data) {
    // Handle visualization updates
    const { robot_config, highlight_joints } = data.params || {};

    // This would update visualization in a real implementation
    console.log('Updating visualization for robot config:', robot_config);

    return {
      id: data.id,
      result: {
        success: true,
        message: 'Visualization updated'
      }
    };
  }

  getMCPInterfaceHTML() {
    return `
<!DOCTYPE html>
<html>
<head>
  <title>Physical AI & Humanoid Robotics - MCP Interface</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; }
    .container { max-width: 800px; margin: 0 auto; }
    .panel { border: 1px solid #ccc; padding: 15px; margin: 10px 0; border-radius: 5px; }
    .status { padding: 10px; margin: 10px 0; border-radius: 3px; }
    .connected { background-color: #d4edda; color: #155724; }
    .disconnected { background-color: #f8d7da; color: #721c24; }
    button { padding: 8px 15px; margin: 5px; }
    input, select { padding: 5px; margin: 5px; }
    .joint-controls { display: flex; flex-wrap: wrap; }
    .joint-control { margin: 5px; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Physical AI & Humanoid Robotics - MCP Interface</h1>

    <div class="panel">
      <h3>Connection Status</h3>
      <div id="status" class="status disconnected">Disconnected</div>
      <button id="connectBtn">Connect to MCP Server</button>
      <button id="disconnectBtn" disabled>Disconnect</button>
    </div>

    <div class="panel">
      <h3>Robot State</h3>
      <div id="robotState">Connect to view robot state</div>
      <button id="getStateBtn" disabled>Get Robot State</button>
    </div>

    <div class="panel">
      <h3>Joint Control</h3>
      <div class="joint-controls">
        <div class="joint-control">
          <label>Joint 1: <input type="number" id="joint1" value="0" step="0.1" min="-3.14" max="3.14"></label>
        </div>
        <div class="joint-control">
          <label>Joint 2: <input type="number" id="joint2" value="0" step="0.1" min="-3.14" max="3.14"></label>
        </div>
        <div class="joint-control">
          <label>Joint 3: <input type="number" id="joint3" value="0" step="0.1" min="-3.14" max="3.14"></label>
        </div>
        <div class="joint-control">
          <label>Joint 4: <input type="number" id="joint4" value="0" step="0.1" min="-3.14" max="3.14"></label>
        </div>
        <div class="joint-control">
          <label>Joint 5: <input type="number" id="joint5" value="0" step="0.1" min="-3.14" max="3.14"></label>
        </div>
        <div class="joint-control">
          <label>Joint 6: <input type="number" id="joint6" value="0" step="0.1" min="-3.14" max="3.14"></label>
        </div>
      </div>
      <button id="setJointsBtn" disabled>Set Joint Angles</button>
    </div>

    <div class="panel">
      <h3>Simulation Control</h3>
      <button id="simulateBtn" disabled>Run Simulation Step</button>
      <button id="balanceBtn" disabled>Run Balance Control</button>
    </div>
  </div>

  <script>
    class MCPClient {
      constructor() {
        this.ws = null;
        this.isConnected = false;
        this.requestId = 1;
        this.callbacks = new Map();

        this.connectBtn = document.getElementById('connectBtn');
        this.disconnectBtn = document.getElementById('disconnectBtn');
        this.statusDiv = document.getElementById('status');
        this.getStateBtn = document.getElementById('getStateBtn');
        this.setJointsBtn = document.getElementById('setJointsBtn');
        this.simulateBtn = document.getElementById('simulateBtn');
        this.balanceBtn = document.getElementById('balanceBtn');
        this.robotStateDiv = document.getElementById('robotState');

        this.setupEventListeners();
      }

      setupEventListeners() {
        this.connectBtn.addEventListener('click', () => this.connect());
        this.disconnectBtn.addEventListener('click', () => this.disconnect());
        this.getStateBtn.addEventListener('click', () => this.getRobotState());
        this.setJointsBtn.addEventListener('click', () => this.setJointAngles());
        this.simulateBtn.addEventListener('click', () => this.simulateStep());
        this.balanceBtn.addEventListener('click', () => this.balanceControl());
      }

      connect() {
        try {
          this.ws = new WebSocket('ws://localhost:8080');

          this.ws.onopen = () => {
            console.log('Connected to MCP server');
            this.isConnected = true;
            this.updateUI();
          };

          this.ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            this.handleResponse(data);
          };

          this.ws.onclose = () => {
            console.log('Disconnected from MCP server');
            this.isConnected = false;
            this.updateUI();
          };

          this.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
            this.isConnected = false;
            this.updateUI();
          };
        } catch (error) {
          console.error('Connection error:', error);
        }
      }

      disconnect() {
        if (this.ws) {
          this.ws.close();
          this.ws = null;
        }
        this.isConnected = false;
        this.updateUI();
      }

      updateUI() {
        if (this.isConnected) {
          this.statusDiv.textContent = 'Connected to MCP Server';
          this.statusDiv.className = 'status connected';
          this.connectBtn.disabled = true;
          this.disconnectBtn.disabled = false;
          this.getStateBtn.disabled = false;
          this.setJointsBtn.disabled = false;
          this.simulateBtn.disabled = false;
          this.balanceBtn.disabled = false;
        } else {
          this.statusDiv.textContent = 'Disconnected from MCP Server';
          this.statusDiv.className = 'status disconnected';
          this.connectBtn.disabled = false;
          this.disconnectBtn.disabled = true;
          this.getStateBtn.disabled = true;
          this.setJointsBtn.disabled = true;
          this.simulateBtn.disabled = true;
          this.balanceBtn.disabled = true;
        }
      }

      sendRequest(method, params = {}) {
        if (!this.isConnected || !this.ws) {
          console.error('Not connected to MCP server');
          return;
        }

        const id = this.requestId++;
        const message = { id, method, params };

        return new Promise((resolve, reject) => {
          this.callbacks.set(id, { resolve, reject });
          this.ws.send(JSON.stringify(message));

          // Set timeout for response
          setTimeout(() => {
            if (this.callbacks.has(id)) {
              this.callbacks.get(id).reject(new Error('Request timeout'));
              this.callbacks.delete(id);
            }
          }, 5000);
        });
      }

      handleResponse(data) {
        if (data.id && this.callbacks.has(data.id)) {
          const callback = this.callbacks.get(data.id);
          if (data.error) {
            callback.reject(new Error(data.error));
          } else {
            callback.resolve(data.result);
          }
          this.callbacks.delete(data.id);
        }
      }

      async getRobotState() {
        try {
          const result = await this.sendRequest('robot.get_state');
          this.robotStateDiv.innerHTML = '<pre>' + JSON.stringify(result, null, 2) + '</pre>';
        } catch (error) {
          this.robotStateDiv.textContent = 'Error getting robot state: ' + error.message;
        }
      }

      async setJointAngles() {
        const joint1 = parseFloat(document.getElementById('joint1').value);
        const joint2 = parseFloat(document.getElementById('joint2').value);
        const joint3 = parseFloat(document.getElementById('joint3').value);
        const joint4 = parseFloat(document.getElementById('joint4').value);
        const joint5 = parseFloat(document.getElementById('joint5').value);
        const joint6 = parseFloat(document.getElementById('joint6').value);

        const jointAngles = [joint1, joint2, joint3, joint4, joint5, joint6];

        try {
          const result = await this.sendRequest('robot.set_joint_angles', { joint_angles: jointAngles });
          alert('Joint angles set: ' + JSON.stringify(result));
        } catch (error) {
          alert('Error setting joint angles: ' + error.message);
        }
      }

      async simulateStep() {
        try {
          const result = await this.sendRequest('robot.simulate_step', { time_step: 0.01 });
          alert('Simulation step completed: ' + JSON.stringify(result));
        } catch (error) {
          alert('Error running simulation: ' + error.message);
        }
      }

      async balanceControl() {
        try {
          const result = await this.sendRequest('robot.balance_control', {
            control_mode: 'zmp',
            target_com: { x: 0, y: 0, z: 0.8 }
          });
          alert('Balance control completed: ' + JSON.stringify(result));
        } catch (error) {
          alert('Error running balance control: ' + error.message);
        }
      }
    }

    // Initialize the MCP client when the page loads
    document.addEventListener('DOMContentLoaded', () => {
      new MCPClient();
    });
  </script>
</body>
</html>
    `;
  }
}

// If this file is run directly, start the server
if (require.main === module) {
  const server = new MCPServer(8080);
  server.start();
}

module.exports = MCPServer;