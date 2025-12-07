---
sidebar_position: 1
---

# MCP Server Integration for Interactive Demos

Model-Context Protocol (MCP) integration enables interactive demonstrations of humanoid robotics concepts through web interfaces and remote access. This section explains how to set up and use MCP for interactive humanoid robotics demonstrations.

## Overview of MCP in Humanoid Robotics

MCP (Model-Context Protocol) enables:
- **Interactive demonstrations**: Real-time interaction with humanoid robot models
- **Remote access**: Access to simulation environments from web browsers
- **Context sharing**: Integration between AI models and robotics environments
- **Educational tools**: Interactive learning experiences for robotics concepts

## MCP Server Architecture

### Basic MCP Server Structure

```python
# humanoid_mcp_server.py
import asyncio
import json
from aiohttp import web, WSMsgType
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class RobotState:
    """Represents the current state of the humanoid robot"""
    joint_positions: List[float]
    joint_velocities: List[float]
    com_position: List[float]  # Center of mass
    com_velocity: List[float]
    orientation: List[float]  # Quaternion [x, y, z, w]
    angular_velocity: List[float]
    balance_metrics: Dict[str, float]

@dataclass
class ControlCommand:
    """Represents a control command for the robot"""
    joint_positions: Optional[List[float]] = None
    joint_velocities: Optional[List[float]] = None
    torques: Optional[List[float]] = None
    balance_mode: Optional[bool] = None
    walk_command: Optional[Dict] = None

class HumanoidMCPHandler:
    """Handles MCP communication for humanoid robotics demos"""

    def __init__(self):
        self.robots = {}  # Robot ID to state mapping
        self.simulations = {}  # Active simulation instances
        self.connected_clients = set()
        self.default_robot_id = "humanoid_001"

        # Initialize default robot state
        self.robots[self.default_robot_id] = RobotState(
            joint_positions=[0.0] * 20,  # 20 DOF humanoid
            joint_velocities=[0.0] * 20,
            com_position=[0.0, 0.0, 0.85],  # 85cm CoM height
            com_velocity=[0.0, 0.0, 0.0],
            orientation=[0.0, 0.0, 0.0, 1.0],  # No rotation (quaternion)
            angular_velocity=[0.0, 0.0, 0.0],
            balance_metrics={
                "zmp_x": 0.0,
                "zmp_y": 0.0,
                "capture_point_x": 0.0,
                "capture_point_y": 0.0,
                "stability": 1.0
            }
        )

    async def handle_websocket(self, request):
        """Handle WebSocket connections for real-time interaction"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        self.connected_clients.add(ws)
        logger.info(f"New client connected. Total clients: {len(self.connected_clients)}")

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self.process_message(msg.data, ws)
                elif msg.type == WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
                    break
        finally:
            self.connected_clients.remove(ws)
            logger.info(f"Client disconnected. Total clients: {len(self.connected_clients)}")

        return ws

    async def process_message(self, message_str: str, ws):
        """Process incoming messages from clients"""
        try:
            message = json.loads(message_str)
            msg_type = message.get("type")

            if msg_type == "get_robot_state":
                robot_id = message.get("robot_id", self.default_robot_id)
                await self.send_robot_state(robot_id, ws)

            elif msg_type == "send_control":
                robot_id = message.get("robot_id", self.default_robot_id)
                command = ControlCommand(**message.get("command", {}))
                await self.execute_control_command(robot_id, command)
                await self.send_robot_state(robot_id, ws)

            elif msg_type == "start_simulation":
                robot_id = message.get("robot_id", self.default_robot_id)
                await self.start_simulation(robot_id)
                await self.send_robot_state(robot_id, ws)

            elif msg_type == "stop_simulation":
                robot_id = message.get("robot_id", self.default_robot_id)
                await self.stop_simulation(robot_id)

            elif msg_type == "reset_robot":
                robot_id = message.get("robot_id", self.default_robot_id)
                await self.reset_robot(robot_id)
                await self.send_robot_state(robot_id, ws)

            else:
                error_msg = {"type": "error", "message": f"Unknown message type: {msg_type}"}
                await ws.send_str(json.dumps(error_msg))

        except json.JSONDecodeError:
            error_msg = {"type": "error", "message": "Invalid JSON message"}
            await ws.send_str(json.dumps(error_msg))
        except Exception as e:
            logger.error(f"Error processing message: {e}")
            error_msg = {"type": "error", "message": f"Processing error: {str(e)}"}
            await ws.send_str(json.dumps(error_msg))

    async def send_robot_state(self, robot_id: str, ws):
        """Send current robot state to client"""
        if robot_id in self.robots:
            state = self.robots[robot_id]
            state_dict = {
                "type": "robot_state",
                "robot_id": robot_id,
                "state": asdict(state)
            }
            await ws.send_str(json.dumps(state_dict))
        else:
            error_msg = {"type": "error", "message": f"Robot {robot_id} not found"}
            await ws.send_str(json.dumps(error_msg))

    async def execute_control_command(self, robot_id: str, command: ControlCommand):
        """Execute control command on the robot"""
        if robot_id not in self.robots:
            logger.error(f"Robot {robot_id} not found")
            return

        robot_state = self.robots[robot_id]

        # Apply joint position commands
        if command.joint_positions:
            robot_state.joint_positions = command.joint_positions

        # Apply joint velocity commands
        if command.joint_velocities:
            robot_state.joint_velocities = command.joint_velocities

        # Apply torque commands
        if command.torques:
            # In simulation, torques would be applied to physics engine
            # For this demo, we'll simulate the effect
            pass

        # Handle balance mode
        if command.balance_mode is not None:
            if command.balance_mode:
                # Apply simple balance control
                self.apply_balance_control(robot_state)

        # Handle walk command
        if command.walk_command:
            self.execute_walk_command(robot_state, command.walk_command)

        # Update derived metrics
        self.update_balance_metrics(robot_state)

        logger.info(f"Executed control command for robot {robot_id}")

    def apply_balance_control(self, robot_state: RobotState):
        """Apply simple balance control to robot state"""
        # Simple PD control based on CoM position
        kp = 10.0
        kd = 1.0

        # Get current CoM error from desired position (0,0,0.85)
        com_error_x = -robot_state.com_position[0]  # Negative to correct
        com_error_y = -robot_state.com_position[1]

        # Apply control to joint positions (simplified)
        for i in range(min(4, len(robot_state.joint_positions))):  # Adjust first 4 joints
            control_input = kp * com_error_x + kd * robot_state.com_velocity[0]
            robot_state.joint_positions[i] += control_input * 0.01  # Small adjustment

    def execute_walk_command(self, robot_state: RobotState, walk_command: Dict):
        """Execute walking command"""
        # Extract walk parameters
        distance = walk_command.get("distance", 0.0)
        speed = walk_command.get("speed", 0.5)
        step_height = walk_command.get("step_height", 0.1)

        # Update CoM position based on walk command (simplified)
        robot_state.com_position[0] += speed * 0.01  # Move forward
        robot_state.com_position[2] = 0.85 + step_height * 0.1  # Simulate step height

        logger.info(f"Executed walk command: distance={distance}, speed={speed}")

    def update_balance_metrics(self, robot_state: RobotState):
        """Update balance-related metrics"""
        # Calculate ZMP (Zero Moment Point) - simplified
        h = 0.85  # Assumed CoM height
        g = 9.81  # Gravity

        zmp_x = robot_state.com_position[0] - (h * robot_state.com_velocity[0]) / g
        zmp_y = robot_state.com_position[1] - (h * robot_state.com_velocity[1]) / g

        # Calculate capture point
        omega = (g / h) ** 0.5
        capture_point_x = robot_state.com_position[0] + robot_state.com_velocity[0] / omega
        capture_point_y = robot_state.com_position[1] + robot_state.com_velocity[1] / omega

        # Update metrics
        robot_state.balance_metrics.update({
            "zmp_x": zmp_x,
            "zmp_y": zmp_y,
            "capture_point_x": capture_point_x,
            "capture_point_y": capture_point_y,
            "stability": 1.0 / (1.0 + abs(zmp_x) + abs(zmp_y))  # Simple stability metric
        })

    async def start_simulation(self, robot_id: str):
        """Start physics simulation for the robot"""
        if robot_id not in self.simulations:
            # In a real implementation, this would start a physics simulation
            self.simulations[robot_id] = {
                "running": True,
                "step_count": 0,
                "last_update": asyncio.get_event_loop().time()
            }
            logger.info(f"Started simulation for robot {robot_id}")

    async def stop_simulation(self, robot_id: str):
        """Stop physics simulation for the robot"""
        if robot_id in self.simulations:
            self.simulations[robot_id]["running"] = False
            del self.simulations[robot_id]
            logger.info(f"Stopped simulation for robot {robot_id}")

    async def reset_robot(self, robot_id: str):
        """Reset robot to initial state"""
        if robot_id in self.robots:
            # Reset to initial state
            self.robots[robot_id] = RobotState(
                joint_positions=[0.0] * 20,
                joint_velocities=[0.0] * 20,
                com_position=[0.0, 0.0, 0.85],
                com_velocity=[0.0, 0.0, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                angular_velocity=[0.0, 0.0, 0.0],
                balance_metrics={
                    "zmp_x": 0.0,
                    "zmp_y": 0.0,
                    "capture_point_x": 0.0,
                    "capture_point_y": 0.0,
                    "stability": 1.0
                }
            )
            logger.info(f"Reset robot {robot_id}")

    async def broadcast_state_update(self, robot_id: str):
        """Broadcast state update to all connected clients"""
        if robot_id in self.robots and self.connected_clients:
            state = self.robots[robot_id]
            state_dict = {
                "type": "robot_state_update",
                "robot_id": robot_id,
                "state": asdict(state)
            }
            message = json.dumps(state_dict)

            disconnected_clients = []
            for ws in self.connected_clients:
                try:
                    await ws.send_str(message)
                except Exception:
                    disconnected_clients.append(ws)

            # Remove disconnected clients
            for ws in disconnected_clients:
                self.connected_clients.discard(ws)

class HumanoidMCPApp:
    """Main application class for the humanoid MCP server"""

    def __init__(self):
        self.app = web.Application()
        self.handler = HumanoidMCPHandler()
        self.setup_routes()

    def setup_routes(self):
        """Set up application routes"""
        self.app.router.add_get('/ws', self.handler.handle_websocket)
        self.app.router.add_get('/', self.serve_index)
        self.app.router.add_static('/static', 'static/', show_index=True)

    async def serve_index(self, request):
        """Serve the main index page for the interactive demo"""
        html_content = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Humanoid Robotics Interactive Demo</title>
            <style>
                body { font-family: Arial, sans-serif; margin: 20px; }
                .container { max-width: 1200px; margin: 0 auto; }
                .panel {
                    border: 1px solid #ccc;
                    padding: 20px;
                    margin: 10px 0;
                    border-radius: 5px;
                }
                .controls { display: flex; gap: 10px; flex-wrap: wrap; }
                .control-group { margin: 10px 0; }
                button {
                    padding: 10px 15px;
                    margin: 5px;
                    border: none;
                    border-radius: 3px;
                    cursor: pointer;
                }
                button:hover { background-color: #e0e0e0; }
                #connect-btn { background-color: #4CAF50; color: white; }
                #disconnect-btn { background-color: #f44336; color: white; }
                .status { padding: 10px; margin: 10px 0; border-radius: 3px; }
                .connected { background-color: #dff0d8; color: #3c763d; }
                .disconnected { background-color: #f2dede; color: #a94442; }
                canvas { border: 1px solid #ddd; background-color: #f9f9f9; }
                .metrics-grid {
                    display: grid;
                    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
                    gap: 10px;
                    margin-top: 10px;
                }
                .metric {
                    padding: 10px;
                    background-color: #f0f0f0;
                    border-radius: 3px;
                    text-align: center;
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>Humanoid Robotics Interactive Demo</h1>

                <div id="connection-status" class="status disconnected">
                    Disconnected from server
                </div>

                <div class="controls">
                    <button id="connect-btn">Connect</button>
                    <button id="disconnect-btn">Disconnect</button>
                    <button id="start-sim-btn">Start Simulation</button>
                    <button id="stop-sim-btn">Stop Simulation</button>
                    <button id="reset-robot-btn">Reset Robot</button>
                    <button id="balance-mode-btn">Toggle Balance Mode</button>
                </div>

                <div class="panel">
                    <h3>Robot Visualization</h3>
                    <canvas id="robot-canvas" width="800" height="400"></canvas>
                </div>

                <div class="panel">
                    <h3>Balance Metrics</h3>
                    <div id="metrics-container" class="metrics-grid">
                        <div class="metric">ZMP X: <span id="zmp-x">0.00</span></div>
                        <div class="metric">ZMP Y: <span id="zmp-y">0.00</span></div>
                        <div class="metric">Capture Point X: <span id="capture-x">0.00</span></div>
                        <div class="metric">Capture Point Y: <span id="capture-y">0.00</span></div>
                        <div class="metric">Stability: <span id="stability">1.00</span></div>
                        <div class="metric">CoM X: <span id="com-x">0.00</span></div>
                        <div class="metric">CoM Y: <span id="com-y">0.00</span></div>
                        <div class="metric">CoM Z: <span id="com-z">0.85</span></div>
                    </div>
                </div>

                <div class="panel">
                    <h3>Joint Control</h3>
                    <div class="control-group">
                        <label>Joint 1: <input type="range" id="joint-0" min="-1.57" max="1.57" step="0.01" value="0"></label>
                        <span id="joint-0-value">0.00</span>
                    </div>
                    <div class="control-group">
                        <label>Joint 2: <input type="range" id="joint-1" min="-1.57" max="1.57" step="0.01" value="0"></label>
                        <span id="joint-1-value">0.00</span>
                    </div>
                    <div class="control-group">
                        <label>Joint 3: <input type="range" id="joint-2" min="-1.57" max="1.57" step="0.01" value="0"></label>
                        <span id="joint-2-value">0.00</span>
                    </div>
                    <div class="control-group">
                        <button id="send-joints-btn">Send Joint Positions</button>
                    </div>
                </div>

                <div class="panel">
                    <h3>Walking Control</h3>
                    <div class="control-group">
                        <label>Distance: <input type="number" id="walk-distance" value="1.0" step="0.1" min="0"></label> m
                        <label>Speed: <input type="number" id="walk-speed" value="0.5" step="0.1" min="0.1" max="2.0"></label> m/s
                        <button id="walk-btn">Start Walking</button>
                    </div>
                </div>
            </div>

            <script>
                class HumanoidDemo {
                    constructor() {
                        this.ws = null;
                        this.isConnected = false;
                        this.balanceMode = false;
                        this.robotState = null;

                        this.initializeElements();
                        this.setupEventListeners();
                    }

                    initializeElements() {
                        this.connectBtn = document.getElementById('connect-btn');
                        this.disconnectBtn = document.getElementById('disconnect-btn');
                        this.startSimBtn = document.getElementById('start-sim-btn');
                        this.stopSimBtn = document.getElementById('stop-sim-btn');
                        this.resetRobotBtn = document.getElementById('reset-robot-btn');
                        this.balanceModeBtn = document.getElementById('balance-mode-btn');
                        this.sendJointsBtn = document.getElementById('send-joints-btn');
                        this.walkBtn = document.getElementById('walk-btn');

                        this.connectionStatus = document.getElementById('connection-status');
                        this.canvas = document.getElementById('robot-canvas');
                        this.ctx = this.canvas.getContext('2d');

                        // Metric elements
                        this.zmpX = document.getElementById('zmp-x');
                        this.zmpY = document.getElementById('zmp-y');
                        this.captureX = document.getElementById('capture-x');
                        this.captureY = document.getElementById('capture-y');
                        this.stability = document.getElementById('stability');
                        this.comX = document.getElementById('com-x');
                        this.comY = document.getElementById('com-y');
                        this.comZ = document.getElementById('com-z');

                        // Joint sliders
                        for (let i = 0; i < 3; i++) {
                            const slider = document.getElementById(`joint-${i}`);
                            const valueDisplay = document.getElementById(`joint-${i}-value`);
                            slider.addEventListener('input', () => {
                                valueDisplay.textContent = parseFloat(slider.value).toFixed(2);
                            });
                        }
                    }

                    setupEventListeners() {
                        this.connectBtn.addEventListener('click', () => this.connect());
                        this.disconnectBtn.addEventListener('click', () => this.disconnect());
                        this.startSimBtn.addEventListener('click', () => this.startSimulation());
                        this.stopSimBtn.addEventListener('click', () => this.stopSimulation());
                        this.resetRobotBtn.addEventListener('click', () => this.resetRobot());
                        this.balanceModeBtn.addEventListener('click', () => this.toggleBalanceMode());
                        this.sendJointsBtn.addEventListener('click', () => this.sendJointPositions());
                        this.walkBtn.addEventListener('click', () => this.startWalking());
                    }

                    connect() {
                        if (this.isConnected) return;

                        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
                        const wsUrl = `${protocol}//${window.location.host}/ws`;

                        this.ws = new WebSocket(wsUrl);

                        this.ws.onopen = () => {
                            this.isConnected = true;
                            this.connectionStatus.textContent = 'Connected to server';
                            this.connectionStatus.className = 'status connected';
                            console.log('Connected to MCP server');
                        };

                        this.ws.onmessage = (event) => {
                            const data = JSON.parse(event.data);
                            this.handleMessage(data);
                        };

                        this.ws.onclose = () => {
                            this.isConnected = false;
                            this.connectionStatus.textContent = 'Disconnected from server';
                            this.connectionStatus.className = 'status disconnected';
                            console.log('Disconnected from MCP server');
                        };

                        this.ws.onerror = (error) => {
                            console.error('WebSocket error:', error);
                            this.connectionStatus.textContent = 'Connection error';
                            this.connectionStatus.className = 'status disconnected';
                        };
                    }

                    disconnect() {
                        if (this.ws) {
                            this.ws.close();
                        }
                    }

                    handleMessage(data) {
                        if (data.type === 'robot_state' || data.type === 'robot_state_update') {
                            this.robotState = data.state;
                            this.updateMetricsDisplay();
                            this.drawRobot();
                        } else if (data.type === 'error') {
                            console.error('Server error:', data.message);
                        }
                    }

                    updateMetricsDisplay() {
                        if (!this.robotState) return;

                        this.zmpX.textContent = this.robotState.balance_metrics.zmp_x.toFixed(2);
                        this.zmpY.textContent = this.robotState.balance_metrics.zmp_y.toFixed(2);
                        this.captureX.textContent = this.robotState.balance_metrics.capture_point_x.toFixed(2);
                        this.captureY.textContent = this.robotState.balance_metrics.capture_point_y.toFixed(2);
                        this.stability.textContent = this.robotState.balance_metrics.stability.toFixed(2);
                        this.comX.textContent = this.robotState.com_position[0].toFixed(2);
                        this.comY.textContent = this.robotState.com_position[1].toFixed(2);
                        this.comZ.textContent = this.robotState.com_position[2].toFixed(2);
                    }

                    drawRobot() {
                        if (!this.robotState) return;

                        const ctx = this.ctx;
                        const canvas = this.canvas;

                        // Clear canvas
                        ctx.clearRect(0, 0, canvas.width, canvas.height);

                        // Draw ground
                        ctx.fillStyle = '#e0e0e0';
                        ctx.fillRect(0, canvas.height - 50, canvas.width, 50);

                        // Draw robot (simplified representation)
                        const centerX = canvas.width / 2;
                        const groundY = canvas.height - 50;

                        // Draw CoM
                        const comX = centerX + this.robotState.com_position[0] * 100; // Scale factor
                        const comY = groundY - this.robotState.com_position[2] * 100; // Scale factor
                        ctx.fillStyle = 'red';
                        ctx.beginPath();
                        ctx.arc(comX, comY, 8, 0, Math.PI * 2);
                        ctx.fill();
                        ctx.fillStyle = 'black';
                        ctx.font = '12px Arial';
                        ctx.fillText('CoM', comX - 15, comY - 10);

                        // Draw ZMP
                        const zmpX = centerX + this.robotState.balance_metrics.zmp_x * 100;
                        const zmpY = groundY;
                        ctx.fillStyle = 'blue';
                        ctx.beginPath();
                        ctx.arc(zmpX, zmpY, 6, 0, Math.PI * 2);
                        ctx.fill();
                        ctx.fillStyle = 'black';
                        ctx.fillText('ZMP', zmpX - 15, zmpY - 10);

                        // Draw capture point
                        const captureX = centerX + this.robotState.balance_metrics.capture_point_x * 100;
                        const captureY = groundY + 20;
                        ctx.fillStyle = 'green';
                        ctx.beginPath();
                        ctx.arc(captureX, captureY, 6, 0, Math.PI * 2);
                        ctx.fill();
                        ctx.fillStyle = 'black';
                        ctx.fillText('Capture', captureX - 20, captureY - 10);

                        // Draw simple humanoid stick figure
                        ctx.strokeStyle = 'black';
                        ctx.lineWidth = 3;

                        // Torso
                        ctx.beginPath();
                        ctx.moveTo(centerX, groundY - 100);
                        ctx.lineTo(centerX, groundY - 150);
                        ctx.stroke();

                        // Head
                        ctx.beginPath();
                        ctx.arc(centerX, groundY - 160, 10, 0, Math.PI * 2);
                        ctx.stroke();

                        // Arms
                        ctx.beginPath();
                        ctx.moveTo(centerX, groundY - 140);
                        ctx.lineTo(centerX - 30, groundY - 130);
                        ctx.moveTo(centerX, groundY - 140);
                        ctx.lineTo(centerX + 30, groundY - 130);
                        ctx.stroke();

                        // Legs
                        ctx.beginPath();
                        ctx.moveTo(centerX, groundY - 100);
                        ctx.lineTo(centerX - 20, groundY - 50);
                        ctx.moveTo(centerX, groundY - 100);
                        ctx.lineTo(centerX + 20, groundY - 50);
                        ctx.stroke();
                    }

                    startSimulation() {
                        if (!this.isConnected) return;

                        const message = {
                            type: 'start_simulation',
                            robot_id: 'humanoid_001'
                        };
                        this.ws.send(JSON.stringify(message));
                    }

                    stopSimulation() {
                        if (!this.isConnected) return;

                        const message = {
                            type: 'stop_simulation',
                            robot_id: 'humanoid_001'
                        };
                        this.ws.send(JSON.stringify(message));
                    }

                    resetRobot() {
                        if (!this.isConnected) return;

                        const message = {
                            type: 'reset_robot',
                            robot_id: 'humanoid_001'
                        };
                        this.ws.send(JSON.stringify(message));
                    }

                    toggleBalanceMode() {
                        this.balanceMode = !this.balanceMode;
                        this.balanceModeBtn.textContent =
                            this.balanceMode ? 'Balance Mode ON' : 'Balance Mode OFF';

                        if (!this.isConnected) return;

                        const message = {
                            type: 'send_control',
                            robot_id: 'humanoid_001',
                            command: {
                                balance_mode: this.balanceMode
                            }
                        };
                        this.ws.send(JSON.stringify(message));
                    }

                    sendJointPositions() {
                        if (!this.isConnected) return;

                        const jointPositions = [];
                        for (let i = 0; i < 3; i++) {
                            const slider = document.getElementById(`joint-${i}`);
                            jointPositions.push(parseFloat(slider.value));
                        }

                        const message = {
                            type: 'send_control',
                            robot_id: 'humanoid_001',
                            command: {
                                joint_positions: jointPositions
                            }
                        };
                        this.ws.send(JSON.stringify(message));
                    }

                    startWalking() {
                        if (!this.isConnected) return;

                        const distance = parseFloat(document.getElementById('walk-distance').value);
                        const speed = parseFloat(document.getElementById('walk-speed').value);

                        const message = {
                            type: 'send_control',
                            robot_id: 'humanoid_001',
                            command: {
                                walk_command: {
                                    distance: distance,
                                    speed: speed,
                                    step_height: 0.1
                                }
                            }
                        };
                        this.ws.send(JSON.stringify(message));
                    }
                }

                // Initialize the demo when page loads
                window.addEventListener('load', () => {
                    new HumanoidDemo();
                });
            </script>
        </body>
        </html>
        """
        return web.Response(text=html_content, content_type='text/html')

def main():
    """Main function to run the MCP server"""
    app = HumanoidMCPApp()

    # Run the application
    web.run_app(app.app, host='localhost', port=8080)
    logger.info("Humanoid MCP server started on http://localhost:8080")

if __name__ == "__main__":
    main()
```

## MCP Client Integration

### JavaScript Client for Browser Interaction

```html
<!-- humanoid_demo_client.html -->
<!DOCTYPE html>
<html>
<head>
    <title>Humanoid Robotics MCP Client</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .container { max-width: 1000px; margin: 0 auto; }
        .panel {
            border: 1px solid #ccc;
            padding: 20px;
            margin: 10px 0;
            border-radius: 5px;
            background-color: #f9f9f9;
        }
        .status { padding: 10px; margin: 10px 0; border-radius: 3px; }
        .connected { background-color: #dff0d8; color: #3c763d; }
        .disconnected { background-color: #f2dede; color: #a94442; }
        button {
            padding: 10px 15px;
            margin: 5px;
            border: none;
            border-radius: 3px;
            cursor: pointer;
        }
        button:hover { background-color: #e0e0e0; }
        .metrics-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 10px;
            margin-top: 10px;
        }
        .metric {
            padding: 10px;
            background-color: #e8f4fd;
            border-radius: 3px;
            text-align: center;
        }
        canvas { border: 1px solid #ddd; background-color: #fff; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Humanoid Robotics MCP Client</h1>

        <div id="connection-status" class="status disconnected">
            Disconnected from server
        </div>

        <div>
            <button id="connect-btn">Connect to MCP Server</button>
            <button id="disconnect-btn">Disconnect</button>
        </div>

        <div class="panel">
            <h3>Robot Controls</h3>
            <button id="start-sim">Start Simulation</button>
            <button id="stop-sim">Stop Simulation</button>
            <button id="reset-robot">Reset Robot</button>
            <button id="balance-mode">Toggle Balance Mode</button>
        </div>

        <div class="panel">
            <h3>Robot Visualization</h3>
            <canvas id="robot-canvas" width="800" height="400"></canvas>
        </div>

        <div class="panel">
            <h3>Balance Metrics</h3>
            <div id="metrics-container" class="metrics-grid">
                <div class="metric">ZMP X: <span id="zmp-x">0.00</span></div>
                <div class="metric">ZMP Y: <span id="zmp-y">0.00</span></div>
                <div class="metric">Capture Point X: <span id="capture-x">0.00</span></div>
                <div class="metric">Capture Point Y: <span id="capture-y">0.00</span></div>
                <div class="metric">Stability: <span id="stability">1.00</span></div>
            </div>
        </div>

        <div class="panel">
            <h3>Joint Control</h3>
            <div>
                <label>Joint 1: <input type="range" id="joint-0" min="-1.57" max="1.57" step="0.01" value="0"></label>
                <span id="joint-0-value">0.00</span>
            </div>
            <div>
                <label>Joint 2: <input type="range" id="joint-1" min="-1.57" max="1.57" step="0.01" value="0"></label>
                <span id="joint-1-value">0.00</span>
            </div>
            <div>
                <button id="send-joints">Send Joint Positions</button>
            </div>
        </div>

        <div class="panel">
            <h3>Walking Control</h3>
            <div>
                <label>Distance: <input type="number" id="walk-distance" value="1.0" step="0.1"> m</label>
                <label>Speed: <input type="number" id="walk-speed" value="0.5" step="0.1"> m/s</label>
                <button id="start-walking">Start Walking</button>
            </div>
        </div>
    </div>

    <script>
        class HumanoidMCPClient {
            constructor() {
                this.ws = null;
                this.isConnected = false;
                this.robotState = null;

                this.initializeElements();
                this.setupEventListeners();
            }

            initializeElements() {
                this.connectBtn = document.getElementById('connect-btn');
                this.disconnectBtn = document.getElementById('disconnect-btn');
                this.startSimBtn = document.getElementById('start-sim');
                this.stopSimBtn = document.getElementById('stop-sim');
                this.resetRobotBtn = document.getElementById('reset-robot');
                this.balanceModeBtn = document.getElementById('balance-mode');
                this.sendJointsBtn = document.getElementById('send-joints');
                this.startWalkingBtn = document.getElementById('start-walking');

                this.connectionStatus = document.getElementById('connection-status');
                this.canvas = document.getElementById('robot-canvas');
                this.ctx = this.canvas.getContext('2d');

                // Metric displays
                this.zmpX = document.getElementById('zmp-x');
                this.zmpY = document.getElementById('zmp-y');
                this.captureX = document.getElementById('capture-x');
                this.captureY = document.getElementById('capture-y');
                this.stability = document.getElementById('stability');

                // Joint sliders
                for (let i = 0; i < 2; i++) {
                    const slider = document.getElementById(`joint-${i}`);
                    const valueDisplay = document.getElementById(`joint-${i}-value`);
                    slider.addEventListener('input', () => {
                        valueDisplay.textContent = slider.value;
                    });
                }
            }

            setupEventListeners() {
                this.connectBtn.addEventListener('click', () => this.connect());
                this.disconnectBtn.addEventListener('click', () => this.disconnect());
                this.startSimBtn.addEventListener('click', () => this.startSimulation());
                this.stopSimBtn.addEventListener('click', () => this.stopSimulation());
                this.resetRobotBtn.addEventListener('click', () => this.resetRobot());
                this.balanceModeBtn.addEventListener('click', () => this.toggleBalanceMode());
                this.sendJointsBtn.addEventListener('click', () => this.sendJointPositions());
                this.startWalkingBtn.addEventListener('click', () => this.startWalking());
            }

            connect() {
                if (this.isConnected) return;

                // Use the same protocol as the page (http/https)
                const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
                const wsUrl = `${protocol}//${window.location.host}/ws`;

                this.ws = new WebSocket(wsUrl);

                this.ws.onopen = () => {
                    this.isConnected = true;
                    this.connectionStatus.textContent = 'Connected to Humanoid MCP Server';
                    this.connectionStatus.className = 'status connected';
                    console.log('Connected to MCP server');

                    // Request initial robot state
                    this.requestRobotState();
                };

                this.ws.onmessage = (event) => {
                    const data = JSON.parse(event.data);
                    this.handleMessage(data);
                };

                this.ws.onclose = () => {
                    this.isConnected = false;
                    this.connectionStatus.textContent = 'Disconnected from server';
                    this.connectionStatus.className = 'status disconnected';
                    console.log('Disconnected from MCP server');
                };

                this.ws.onerror = (error) => {
                    console.error('WebSocket error:', error);
                    this.connectionStatus.textContent = 'Connection error';
                    this.connectionStatus.className = 'status disconnected';
                };
            }

            disconnect() {
                if (this.ws) {
                    this.ws.close();
                    this.isConnected = false;
                }
            }

            handleMessage(data) {
                if (data.type === 'robot_state' || data.type === 'robot_state_update') {
                    this.robotState = data.state;
                    this.updateDisplay();
                    this.drawRobot();
                } else if (data.type === 'error') {
                    console.error('Server error:', data.message);
                    alert('Server error: ' + data.message);
                }
            }

            requestRobotState() {
                if (!this.isConnected) return;

                const message = {
                    type: 'get_robot_state',
                    robot_id: 'humanoid_001'
                };
                this.ws.send(JSON.stringify(message));
            }

            updateDisplay() {
                if (!this.robotState) return;

                // Update metrics
                this.zmpX.textContent = this.robotState.balance_metrics.zmp_x.toFixed(2);
                this.zmpY.textContent = this.robotState.balance_metrics.zmp_y.toFixed(2);
                this.captureX.textContent = this.robotState.balance_metrics.capture_point_x.toFixed(2);
                this.captureY.textContent = this.robotState.balance_metrics.capture_point_y.toFixed(2);
                this.stability.textContent = this.robotState.balance_metrics.stability.toFixed(2);
            }

            drawRobot() {
                if (!this.robotState) return;

                const ctx = this.ctx;
                const canvas = this.canvas;

                // Clear canvas
                ctx.clearRect(0, 0, canvas.width, canvas.height);

                // Draw ground
                ctx.fillStyle = '#e0e0e0';
                ctx.fillRect(0, canvas.height - 50, canvas.width, 50);

                // Draw CoM
                const centerX = canvas.width / 2;
                const groundY = canvas.height - 50;
                const comX = centerX + this.robotState.com_position[0] * 100; // Scale factor
                const comY = groundY - this.robotState.com_position[2] * 100; // Scale factor

                ctx.fillStyle = 'red';
                ctx.beginPath();
                ctx.arc(comX, comY, 8, 0, Math.PI * 2);
                ctx.fill();
                ctx.fillStyle = 'black';
                ctx.font = '12px Arial';
                ctx.fillText('CoM', comX - 15, comY - 10);

                // Draw ZMP
                const zmpX = centerX + this.robotState.balance_metrics.zmp_x * 100;
                const zmpY = groundY;
                ctx.fillStyle = 'blue';
                ctx.beginPath();
                ctx.arc(zmpX, zmpY, 6, 0, Math.PI * 2);
                ctx.fill();
                ctx.fillStyle = 'black';
                ctx.fillText('ZMP', zmpX - 15, zmpY - 10);

                // Draw simple humanoid
                ctx.strokeStyle = 'black';
                ctx.lineWidth = 3;

                // Torso
                ctx.beginPath();
                ctx.moveTo(centerX, groundY - 100);
                ctx.lineTo(centerX, groundY - 150);
                ctx.stroke();

                // Head
                ctx.beginPath();
                ctx.arc(centerX, groundY - 160, 10, 0, Math.PI * 2);
                ctx.stroke();

                // Arms
                ctx.beginPath();
                ctx.moveTo(centerX, groundY - 140);
                ctx.lineTo(centerX - 30, groundY - 130);
                ctx.moveTo(centerX, groundY - 140);
                ctx.lineTo(centerX + 30, groundY - 130);
                ctx.stroke();

                // Legs
                ctx.beginPath();
                ctx.moveTo(centerX, groundY - 100);
                ctx.lineTo(centerX - 20, groundY - 50);
                ctx.moveTo(centerX, groundY - 100);
                ctx.lineTo(centerX + 20, groundY - 50);
                ctx.stroke();
            }

            startSimulation() {
                if (!this.isConnected) return;

                const message = {
                    type: 'start_simulation',
                    robot_id: 'humanoid_001'
                };
                this.ws.send(JSON.stringify(message));
            }

            stopSimulation() {
                if (!this.isConnected) return;

                const message = {
                    type: 'stop_simulation',
                    robot_id: 'humanoid_001'
                };
                this.ws.send(JSON.stringify(message));
            }

            resetRobot() {
                if (!this.isConnected) return;

                const message = {
                    type: 'reset_robot',
                    robot_id: 'humanoid_001'
                };
                this.ws.send(JSON.stringify(message));
            }

            toggleBalanceMode() {
                if (!this.isConnected) return;

                const message = {
                    type: 'send_control',
                    robot_id: 'humanoid_001',
                    command: {
                        balance_mode: !this.robotState?.balance_metrics?.balance_mode
                    }
                };
                this.ws.send(JSON.stringify(message));
            }

            sendJointPositions() {
                if (!this.isConnected) return;

                const jointPositions = [];
                for (let i = 0; i < 2; i++) {
                    const slider = document.getElementById(`joint-${i}`);
                    jointPositions.push(parseFloat(slider.value));
                }

                const message = {
                    type: 'send_control',
                    robot_id: 'humanoid_001',
                    command: {
                        joint_positions: jointPositions
                    }
                };
                this.ws.send(JSON.stringify(message));
            }

            startWalking() {
                if (!this.isConnected) return;

                const distance = parseFloat(document.getElementById('walk-distance').value);
                const speed = parseFloat(document.getElementById('walk-speed').value);

                const message = {
                    type: 'send_control',
                    robot_id: 'humanoid_001',
                    command: {
                        walk_command: {
                            distance: distance,
                            speed: speed
                        }
                    }
                };
                this.ws.send(JSON.stringify(message));
            }
        }

        // Initialize client when page loads
        window.addEventListener('load', () => {
            new HumanoidMCPClient();
        });
    </script>
</body>
</html>
```

## MCP Configuration for Production

### Docker Configuration

```dockerfile
# Dockerfile for Humanoid MCP Server
FROM python:3.9-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Expose port
EXPOSE 8080

# Run the application
CMD ["python", "humanoid_mcp_server.py"]
```

### Docker Compose Configuration

```yaml
# docker-compose.yml
version: '3.8'

services:
  humanoid-mcp:
    build: .
    ports:
      - "8080:8080"
    environment:
      - PYTHONUNBUFFERED=1
    volumes:
      - ./logs:/app/logs
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
      - ./ssl:/etc/nginx/ssl
    depends_on:
      - humanoid-mcp
    restart: unless-stopped
```

### Nginx Configuration

```nginx
# nginx.conf
events {
    worker_connections 1024;
}

http {
    upstream humanoid_mcp_backend {
        server humanoid-mcp:8080;
    }

    server {
        listen 80;
        server_name localhost;

        # Serve static files
        location /static/ {
            alias /app/static/;
            expires 1d;
        }

        # WebSocket upgrade for MCP
        location /ws {
            proxy_pass http://humanoid_mcp_backend;
            proxy_http_version 1.1;
            proxy_set_header Upgrade $http_upgrade;
            proxy_set_header Connection "upgrade";
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
            proxy_set_header X-Forwarded-Proto $scheme;

            # WebSocket timeout
            proxy_read_timeout 86400;
            proxy_send_timeout 86400;
        }

        # Main application
        location / {
            proxy_pass http://humanoid_mcp_backend;
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
            proxy_set_header X-Forwarded-Proto $scheme;
        }
    }
}
```

## MCP Security Considerations

### Authentication Middleware

```python
# auth_middleware.py
import jwt
import asyncio
from aiohttp import web
from functools import wraps

class MCPAuthMiddleware:
    """Authentication middleware for MCP server"""

    def __init__(self, secret_key: str, algorithm: str = 'HS256'):
        self.secret_key = secret_key
        self.algorithm = algorithm
        self.valid_tokens = set()  # In production, use a proper token store

    def generate_token(self, user_id: str, permissions: list) -> str:
        """Generate authentication token"""
        import time
        payload = {
            'user_id': user_id,
            'permissions': permissions,
            'exp': time.time() + 3600,  # 1 hour expiration
            'iat': time.time()
        }
        return jwt.encode(payload, self.secret_key, algorithm=self.algorithm)

    def verify_token(self, token: str) -> dict:
        """Verify authentication token"""
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])
            return payload
        except jwt.ExpiredSignatureError:
            raise web.HTTPUnauthorized(text="Token expired")
        except jwt.InvalidTokenError:
            raise web.HTTPUnauthorized(text="Invalid token")

    def require_auth(self, permissions: list = None):
        """Decorator to require authentication"""
        def decorator(handler):
            @wraps(handler)
            async def wrapped(request):
                # Check for authorization header
                auth_header = request.headers.get('Authorization')
                if not auth_header or not auth_header.startswith('Bearer '):
                    raise web.HTTPUnauthorized(text="Authorization header required")

                token = auth_header.split(' ')[1]
                payload = self.verify_token(token)

                # Check permissions if specified
                if permissions:
                    user_perms = payload.get('permissions', [])
                    if not any(perm in user_perms for perm in permissions):
                        raise web.HTTPForbidden(text="Insufficient permissions")

                # Add user info to request
                request['user'] = payload

                return await handler(request)
            return wrapped
        return decorator

# Usage example in the main server
# auth_middleware = MCPAuthMiddleware(secret_key='your-secret-key')
#
# @auth_middleware.require_auth(permissions=['read_robot_state', 'send_control'])
# async def protected_handler(request):
#     # Handler code here
#     pass
```

## MCP Performance Optimization

### Caching Layer

```python
# cache_layer.py
import asyncio
import time
from typing import Any, Optional
from collections import OrderedDict

class LRUCache:
    """Simple LRU cache for MCP server"""

    def __init__(self, max_size: int = 1000, ttl: int = 300):  # 5 min default TTL
        self.max_size = max_size
        self.ttl = ttl
        self.cache = OrderedDict()

    def get(self, key: str) -> Optional[Any]:
        """Get value from cache"""
        if key in self.cache:
            value, timestamp = self.cache[key]
            if time.time() - timestamp < self.ttl:
                # Move to end (most recently used)
                self.cache.move_to_end(key)
                return value
            else:
                # Expired, remove
                del self.cache[key]
        return None

    def set(self, key: str, value: Any):
        """Set value in cache"""
        if key in self.cache:
            self.cache.move_to_end(key)
        elif len(self.cache) >= self.max_size:
            # Remove least recently used
            self.cache.popitem(last=False)

        self.cache[key] = (value, time.time())

    def clear_expired(self):
        """Remove expired entries"""
        current_time = time.time()
        expired_keys = []

        for key, (value, timestamp) in self.cache.items():
            if current_time - timestamp >= self.ttl:
                expired_keys.append(key)

        for key in expired_keys:
            del self.cache[key]

class MCPResponseCache:
    """Response caching for MCP server"""

    def __init__(self):
        self.cache = LRUCache(max_size=1000, ttl=60)  # 1 minute TTL for responses

    async def get_cached_response(self, request_hash: str) -> Optional[dict]:
        """Get cached response"""
        return self.cache.get(request_hash)

    async def cache_response(self, request_hash: str, response: dict):
        """Cache response"""
        self.cache.set(request_hash, response)

    def generate_request_hash(self, request_data: dict) -> str:
        """Generate hash for request data"""
        import hashlib
        import json
        json_str = json.dumps(request_data, sort_keys=True)
        return hashlib.md5(json_str.encode()).hexdigest()
```

## Summary

MCP (Model-Context Protocol) integration provides powerful interactive capabilities for humanoid robotics demonstrations:

1. **Real-time Interaction**: WebSocket-based communication for immediate feedback
2. **Web-based Access**: Browser-based interface for easy access
3. **Visualization**: Real-time rendering of robot state and metrics
4. **Control Interface**: Direct manipulation of robot parameters
5. **Scalability**: Server-based architecture for multiple users
6. **Security**: Authentication and authorization mechanisms
7. **Performance**: Caching and optimization techniques

The MCP server architecture enables rich interactive experiences that can help students, researchers, and enthusiasts understand complex humanoid robotics concepts through direct manipulation and visualization. The system can be deployed in various configurations from local development to production cloud environments, making it accessible for educational and research purposes.

The integration of simulation, visualization, and control in a web-based interface provides an engaging way to explore humanoid robotics without requiring specialized hardware or software installations.