---
sidebar_position: 1
---

# Python Code Examples for Humanoid Robotics

This section provides practical Python code examples for implementing key concepts in humanoid robotics. These examples demonstrate fundamental algorithms and techniques used in real humanoid robot systems.

## Kinematics Example

### Forward Kinematics Implementation

```python
import numpy as np
import math

class RobotArm:
    """
    Simple 2-link planar robot arm for demonstrating forward kinematics
    """
    def __init__(self, link_lengths):
        """
        Initialize robot arm with link lengths [L1, L2]
        """
        self.L1, self.L2 = link_lengths

    def forward_kinematics(self, theta1, theta2):
        """
        Calculate end-effector position given joint angles
        """
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        return x, y

# Example usage
arm = RobotArm([1.0, 0.8])  # Link lengths 1.0 and 0.8 meters
theta1 = math.pi / 4  # 45 degrees
theta2 = math.pi / 6  # 30 degrees

x, y = arm.forward_kinematics(theta1, theta2)
print(f"End-effector position: ({x:.3f}, {y:.3f})")
```

### Inverse Kinematics Implementation

```python
import numpy as np

class RobotArmIK:
    """
    2-link planar robot arm with inverse kinematics
    """
    def __init__(self, link_lengths):
        self.L1, self.L2 = link_lengths

    def inverse_kinematics(self, x, y):
        """
        Calculate joint angles to reach target position (x, y)
        Returns (theta1, theta2) or None if position is unreachable
        """
        # Calculate distance from origin to target
        r = math.sqrt(x**2 + y**2)

        # Check if target is reachable
        if r > (self.L1 + self.L2):
            print("Target position is unreachable")
            return None

        if r < abs(self.L1 - self.L2):
            print("Target position is inside workspace")
            return None

        # Calculate angle of second link
        cos_theta2 = (self.L1**2 + self.L2**2 - r**2) / (2 * self.L1 * self.L2)
        theta2 = math.acos(cos_theta2)

        # Calculate angle of first link
        k1 = self.L1 + self.L2 * math.cos(theta2)
        k2 = self.L2 * math.sin(theta2)

        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        return theta1, theta2

# Example usage
arm_ik = RobotArmIK([1.0, 0.8])
target_x, target_y = 1.2, 0.8

angles = arm_ik.inverse_kinematics(target_x, target_y)
if angles:
    theta1, theta2 = angles
    print(f"Joint angles: θ1={theta1:.3f} rad, θ2={theta2:.3f} rad")

    # Verify with forward kinematics
    x_verify, y_verify = RobotArm([1.0, 0.8]).forward_kinematics(theta1, theta2)
    print(f"Verification - Target: ({target_x}, {target_y}), Reached: ({x_verify:.3f}, {y_verify:.3f})")
```

## Control Systems Example

### PID Controller Implementation

```python
class PIDController:
    """
    PID (Proportional-Integral-Derivative) controller
    """
    def __init__(self, kp, ki, kd, output_limits=(-100, 100)):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.output_limits = output_limits

        self.previous_error = 0
        self.integral = 0
        self.previous_time = None

    def update(self, setpoint, current_value, dt=None):
        """
        Update PID controller and return control output
        """
        import time

        # Calculate time step
        if dt is None:
            current_time = time.time()
            if self.previous_time is None:
                dt = 0.01  # Default time step
            else:
                dt = current_time - self.previous_time
            self.previous_time = current_time
        else:
            current_time = None  # Not using system time

        # Calculate error
        error = setpoint - current_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term

        # Apply output limits
        output = max(self.output_limits[0], min(output, self.output_limits[1]))

        # Store values for next iteration
        self.previous_error = error

        return output

# Example usage: Joint position control
pid = PIDController(kp=10.0, ki=0.1, kd=0.5, output_limits=(-50, 50))

# Simulate controlling a joint to reach target position
target_position = 1.5  # radians
current_position = 0.0
dt = 0.01  # 100 Hz control loop

for i in range(1000):  # Simulate 10 seconds
    control_output = pid.update(target_position, current_position, dt)

    # Simulate robot dynamics (simplified)
    current_position += control_output * dt * 0.1  # Simple integration

    if i % 100 == 0:  # Print every second
        print(f"Time: {i*dt:.2f}s, Position: {current_position:.3f}, Error: {target_position-current_position:.3f}")
```

## Balance Control Example

### Inverted Pendulum Model

```python
import numpy as np

class InvertedPendulum:
    """
    Simple inverted pendulum model for balance control
    """
    def __init__(self, com_height=1.0, gravity=9.81):
        self.h = com_height  # Center of mass height
        self.g = gravity     # Gravitational acceleration
        self.omega = np.sqrt(self.g / self.h)  # Natural frequency

    def compute_zmp(self, x_com, x_com_dot, x_com_ddot):
        """
        Compute Zero Moment Point (ZMP) from CoM state
        """
        zmp = x_com + (self.h * x_com_ddot) / self.g
        return zmp

    def compute_capture_point(self, x_com, x_com_dot):
        """
        Compute capture point for balance recovery
        """
        capture_point = x_com + (x_com_dot / self.omega)
        return capture_point

# Example: Balance recovery
pendulum = InvertedPendulum(com_height=0.9)  # Typical humanoid CoM height

# Initial perturbed state
x_com = 0.05  # 5 cm perturbation
x_com_dot = 0.1  # 0.1 m/s velocity
x_com_ddot = -0.5  # -0.5 m/s² acceleration

zmp = pendulum.compute_zmp(x_com, x_com_dot, x_com_ddot)
capture_point = pendulum.compute_capture_point(x_com, x_com_dot)

print(f"Current CoM position: {x_com:.3f} m")
print(f"ZMP position: {zmp:.3f} m")
print(f"Capture point: {capture_point:.3f} m")
print(f"To recover balance, step to: {capture_point:.3f} m")
```

## Sensor Processing Example

### IMU Data Processing

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUProcessor:
    """
    Process IMU data (accelerometer and gyroscope)
    """
    def __init__(self):
        # Initialize orientation as identity rotation
        self.orientation = R.from_quat([0, 0, 0, 1])
        self.gravity = np.array([0, 0, -9.81])  # Gravity vector in world frame

    def integrate_gyroscope(self, gyro_data, dt):
        """
        Integrate gyroscope data to update orientation
        gyro_data: [wx, wy, wz] angular velocities
        dt: time step
        """
        # Convert angular velocity to rotation vector
        rotation_vector = np.array(gyro_data) * dt

        # Create rotation from small angle approximation
        if np.linalg.norm(rotation_vector) > 1e-6:
            angle = np.linalg.norm(rotation_vector)
            axis = rotation_vector / angle
            d_rotation = R.from_rotvec(axis * angle)
        else:
            d_rotation = R.from_quat([0, 0, 0, 1])  # Identity

        # Update orientation
        self.orientation = d_rotation * self.orientation

    def remove_gravity(self, accel_data):
        """
        Remove gravity from accelerometer data
        accel_data: [ax, ay, az] in sensor frame
        Returns: [ax, ay, az] with gravity removed
        """
        # Convert accelerometer reading to world frame
        accel_world = self.orientation.apply(accel_data)

        # Remove gravity
        linear_accel = accel_world - self.gravity

        # Convert back to sensor frame
        linear_accel_sensor = self.orientation.inv().apply(linear_accel)

        return linear_accel_sensor

# Example usage
imu = IMUProcessor()

# Simulate IMU readings
gyro_data = [0.1, -0.05, 0.02]  # rad/s
accel_data = [0.5, 0.2, -9.3]   # m/s²
dt = 0.01  # 100 Hz

# Update orientation
imu.integrate_gyroscope(gyro_data, dt)

# Process accelerometer data
linear_accel = imu.remove_gravity(accel_data)
print(f"Linear acceleration (gravity removed): [{linear_accel[0]:.3f}, {linear_accel[1]:.3f}, {linear_accel[2]:.3f}] m/s²")

# Current orientation
euler_angles = imu.orientation.as_euler('xyz', degrees=True)
print(f"Current orientation (Euler): [{euler_angles[0]:.2f}, {euler_angles[1]:.2f}, {euler_angles[2]:.2f}] degrees")
```

## Path Planning Example

### A* Path Planning

```python
import heapq
import numpy as np

class AStarPlanner:
    """
    A* path planning algorithm for grid-based navigation
    """
    def __init__(self, grid):
        self.grid = np.array(grid)
        self.rows, self.cols = self.grid.shape

    def heuristic(self, pos1, pos2):
        """Calculate Manhattan distance heuristic"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def get_neighbors(self, pos):
        """Get valid neighboring positions"""
        neighbors = []
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1),  # 4-connectivity
                       (-1,-1), (-1,1), (1,-1), (1,1)]:  # 8-connectivity
            r, c = pos[0] + dr, pos[1] + dc
            if 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r][c] == 0:
                neighbors.append((r, c))
        return neighbors

    def plan(self, start, goal):
        """
        Plan path from start to goal using A*
        Returns path as list of (row, col) tuples
        """
        # Priority queue: (f_score, g_score, position)
        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[2]

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # Reverse to get start-to-goal path

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + 1  # Assuming uniform cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], tentative_g, neighbor))

        return None  # No path found

# Example usage
# Create a simple grid (0 = free space, 1 = obstacle)
grid = [
    [0, 0, 0, 0, 1, 0, 0],
    [0, 1, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0]
]

planner = AStarPlanner(grid)
start = (0, 0)
goal = (6, 6)

path = planner.plan(start, goal)
if path:
    print(f"Path found with {len(path)} steps:")
    for i, pos in enumerate(path):
        print(f"Step {i}: {pos}")
else:
    print("No path found")
```

## Machine Learning Example

### Simple Neural Network for Control

```python
import numpy as np

class SimpleNeuralNetwork:
    """
    Simple neural network for learning control policies
    """
    def __init__(self, input_size, hidden_size, output_size):
        # Initialize weights randomly
        self.W1 = np.random.randn(input_size, hidden_size) * 0.5
        self.b1 = np.zeros((1, hidden_size))
        self.W2 = np.random.randn(hidden_size, output_size) * 0.5
        self.b2 = np.zeros((1, output_size))

    def sigmoid(self, x):
        """Sigmoid activation function"""
        return 1 / (1 + np.exp(-np.clip(x, -250, 250)))  # Clip to prevent overflow

    def forward(self, X):
        """Forward pass through the network"""
        self.z1 = np.dot(X, self.W1) + self.b1
        self.a1 = self.sigmoid(self.z1)
        self.z2 = np.dot(self.a1, self.W2) + self.b2
        self.output = self.sigmoid(self.z2)
        return self.output

    def train(self, X, y, epochs=1000, learning_rate=0.1):
        """Simple training using gradient descent"""
        for epoch in range(epochs):
            # Forward pass
            output = self.forward(X)

            # Calculate error
            error = y - output

            # Backward pass (simplified gradient calculation)
            d_output = error * (output * (1 - output))
            error_hidden = d_output.dot(self.W2.T)
            d_hidden = error_hidden * (self.a1 * (1 - self.a1))

            # Update weights
            self.W2 += self.a1.T.dot(d_output) * learning_rate
            self.b2 += np.sum(d_output, axis=0, keepdims=True) * learning_rate
            self.W1 += X.T.dot(d_hidden) * learning_rate
            self.b1 += np.sum(d_hidden, axis=0, keepdims=True) * learning_rate

# Example: Learning to balance (simplified)
# Input: [position_error, velocity_error]
# Output: [control_output]

nn = SimpleNeuralNetwork(input_size=2, hidden_size=10, output_size=1)

# Training data (simplified example)
X_train = np.array([
    [0.1, 0.0],   # Small position error, no velocity
    [-0.1, 0.0],  # Small negative position error
    [0.0, 0.1],   # Positive velocity (falling forward)
    [0.0, -0.1],  # Negative velocity (falling backward)
    [0.2, 0.05],  # Position and velocity errors
])
y_train = np.array([[0.2], [-0.2], [0.1], [-0.1], [0.25]])  # Desired control outputs

print("Training neural network for balance control...")
nn.train(X_train, y_train, epochs=1000, learning_rate=0.1)

# Test the trained network
test_input = np.array([[0.05, 0.02]])  # Small position and velocity errors
control_output = nn.forward(test_input)
print(f"Input: {test_input.flatten()}, Control output: {control_output.flatten()[0]:.3f}")
```

## Visualization Example

### Robot Configuration Visualization

```python
import matplotlib.pyplot as plt
import numpy as np

def visualize_robot_arm(link_lengths, joint_angles):
    """
    Visualize a 2D robot arm configuration
    """
    # Calculate joint positions
    x1 = link_lengths[0] * np.cos(joint_angles[0])
    y1 = link_lengths[0] * np.sin(joint_angles[0])

    x2 = x1 + link_lengths[1] * np.cos(joint_angles[0] + joint_angles[1])
    y2 = y1 + link_lengths[1] * np.sin(joint_angles[0] + joint_angles[1])

    # Plot the arm
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    # Base of the robot
    ax.plot(0, 0, 'ro', markersize=10, label='Base')

    # First link
    ax.plot([0, x1], [0, y1], 'b-', linewidth=3, label='Link 1')
    ax.plot(x1, y1, 'bo', markersize=8)

    # Second link
    ax.plot([x1, x2], [y1, y2], 'g-', linewidth=3, label='Link 2')
    ax.plot(x2, y2, 'go', markersize=8, label='End-effector')

    # Set equal aspect ratio and limits
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()
    ax.set_title('Robot Arm Configuration')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    # Set axis limits
    max_range = sum(link_lengths) * 1.2
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)

    plt.tight_layout()
    plt.show()

# Example: Visualize the robot arm from previous examples
link_lengths = [1.0, 0.8]
joint_angles = [np.pi/4, np.pi/6]  # 45° and 30°

print("Visualizing robot arm configuration...")
print(f"Link lengths: {link_lengths}")
print(f"Joint angles: {np.degrees(joint_angles[0]):.1f}°, {np.degrees(joint_angles[1]):.1f}°")

# Note: In a real implementation, you would call visualize_robot_arm(link_lengths, joint_angles)
# but we're showing the function definition here
```

## Summary

These Python examples demonstrate fundamental concepts in humanoid robotics:

1. **Kinematics**: Forward and inverse kinematics calculations
2. **Control**: PID controller implementation for joint control
3. **Balance**: Inverted pendulum model for balance control
4. **Sensors**: IMU data processing and integration
5. **Planning**: A* path planning algorithm
6. **Learning**: Simple neural network for control
7. **Visualization**: Robot configuration visualization

Each example provides a practical implementation that can be extended and adapted for real humanoid robot applications. The code demonstrates how theoretical concepts translate into working implementations that can be used in actual robot systems.

These examples serve as building blocks for more complex humanoid robot behaviors and can be combined to create sophisticated robotic systems capable of perception, planning, control, and learning.