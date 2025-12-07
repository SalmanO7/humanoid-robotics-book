---
sidebar_position: 3
---

# Sensor-Actuator Integration

## Overview

Sensor-actuator integration is a critical aspect of humanoid robotics that enables robots to perceive their environment and respond appropriately. This integration forms the foundation of closed-loop control systems that allow humanoid robots to interact with the physical world effectively.

## Integration Architecture

### Sensor-Actuator Loop

The fundamental architecture of sensor-actuator integration involves:

1. **Sensing Phase**: Sensors collect data about the environment and robot state
2. **Processing Phase**: Data is processed and interpreted by control algorithms
3. **Actuation Phase**: Actuators execute commands based on processed sensor data
4. **Feedback Phase**: Sensor data verifies actuator outcomes and adjusts future actions

### Real-Time Requirements

Humanoid robots require real-time sensor-actuator integration to maintain stability and responsiveness:

- Low latency communication between sensors and actuators
- Synchronized data acquisition and actuator commands
- Predictable timing for safety-critical operations

## Communication Protocols

### CAN Bus Integration

Controller Area Network (CAN) bus provides robust communication for sensor-actuator networks:

```python
import can
import time

class SensorActuatorBus:
    def __init__(self, channel='can0', bitrate=500000):
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan')

    def send_command(self, actuator_id, command_data):
        """Send command to specific actuator"""
        msg = can.Message(
            arbitration_id=actuator_id,
            data=command_data,
            is_extended_id=True
        )
        self.bus.send(msg)

    def read_sensor(self, sensor_id):
        """Read data from specific sensor"""
        # Listen for sensor data with timeout
        msg = self.bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == sensor_id:
            return msg.data
        return None
```

### EtherCAT for High-Performance Systems

For high-performance humanoid robots, EtherCAT provides deterministic communication:

- Microsecond synchronization accuracy
- Distributed clock mechanism
- Real-time performance guarantees

## Sensor Fusion

### Multi-Sensor Integration

Effective integration combines data from multiple sensor types:

- **Inertial Measurement Units (IMUs)**: Provide orientation and acceleration data
- **Force/Torque Sensors**: Measure interaction forces at joints and end-effectors
- **Vision Systems**: Provide environmental perception
- **Tactile Sensors**: Enable fine manipulation feedback

### Kalman Filtering

Sensor fusion often employs Kalman filtering to combine multiple sensor inputs:

```python
import numpy as np
from scipy.linalg import inv

class SensorFusion:
    def __init__(self, dim_state, dim_measurement):
        self.dim_state = dim_state
        self.dim_measurement = dim_measurement
        self.x = np.zeros((dim_state, 1))  # State estimate
        self.P = np.eye(dim_state)        # Error covariance
        self.Q = np.eye(dim_state) * 0.1  # Process noise
        self.R = np.eye(dim_measurement) * 0.1  # Measurement noise

    def predict(self, F, B, u):
        """Prediction step"""
        self.x = F @ self.x + B @ u
        self.P = F @ self.P @ F.T + self.Q

    def update(self, H, z):
        """Update step with measurement z"""
        y = z - H @ self.x  # Innovation
        S = H @ self.P @ H.T + self.R  # Innovation covariance
        K = self.P @ H.T @ inv(S)  # Kalman gain
        self.x = self.x + K @ y
        self.P = (np.eye(self.dim_state) - K @ H) @ self.P
```

## Control Integration

### Impedance Control

Impedance control integrates sensor feedback to achieve desired dynamic behavior:

```python
class ImpedanceController:
    def __init__(self, mass, damping, stiffness):
        self.M = mass      # Desired inertia
        self.D = damping   # Desired damping
        self.K = stiffness # Desired stiffness

    def compute_force(self, pos_error, vel_error, vel_current):
        """Compute desired force based on impedance model"""
        return self.K @ pos_error + self.D @ vel_error - self.M @ vel_current
```

### Force Control

Force control integrates force sensor feedback for compliant manipulation:

```python
class ForceController:
    def __init__(self, kp=10.0, ki=1.0, kd=0.1):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.error_integral = 0
        self.prev_error = 0

    def compute_command(self, desired_force, actual_force, dt):
        """Compute control command based on force error"""
        error = desired_force - actual_force

        self.error_integral += error * dt
        error_derivative = (error - self.prev_error) / dt

        command = (self.kp * error +
                  self.ki * self.error_integral +
                  self.kd * error_derivative)

        self.prev_error = error
        return command
```

## Safety Considerations

### Fail-Safe Mechanisms

Sensor-actuator integration must include safety mechanisms:

- **Sensor failure detection**: Identify when sensors provide invalid data
- **Actuator limit enforcement**: Prevent commands that exceed actuator capabilities
- **Emergency stop integration**: Immediate response to safety-critical conditions

### Redundancy

Critical sensor-actuator paths should include redundancy:

- Multiple sensors for critical measurements
- Backup actuator control paths
- Voting algorithms for sensor validation

## Practical Implementation

### Example: Joint Control with Integrated Sensors

```python
class JointController:
    def __init__(self, joint_id, encoder_resolution=4096):
        self.joint_id = joint_id
        self.encoder_resolution = encoder_resolution
        self.position = 0
        self.velocity = 0
        self.torque = 0

        # PID controller for position control
        self.pid = {
            'kp': 100.0,  # Proportional gain
            'ki': 10.0,   # Integral gain
            'kd': 10.0    # Derivative gain
        }
        self.integral_error = 0
        self.prev_error = 0

    def update_sensors(self, encoder_count, torque_sensor):
        """Update internal state from sensors"""
        self.position = (encoder_count / self.encoder_resolution) * 2 * np.pi
        self.torque = torque_sensor

    def compute_torque_command(self, desired_position, dt):
        """Compute torque command based on position error"""
        error = desired_position - self.position

        self.integral_error += error * dt
        derivative_error = (error - self.prev_error) / dt

        torque_cmd = (self.pid['kp'] * error +
                     self.pid['ki'] * self.integral_error +
                     self.pid['kd'] * derivative_error)

        self.prev_error = error
        return np.clip(torque_cmd, -100, 100)  # Limit torque output
```

## Integration Challenges

### Latency Management

Minimizing communication delays between sensors and actuators is crucial:

- Use high-speed communication protocols
- Optimize data processing pipelines
- Implement predictive control for compensation

### Synchronization

Ensuring sensor and actuator data are temporally aligned:

- Use shared system clock
- Implement timestamp-based synchronization
- Account for communication delays in control algorithms

## Best Practices

1. **Modular Design**: Separate sensor processing, control algorithms, and actuator commands
2. **Calibration**: Regular calibration of sensors and actuators
3. **Validation**: Extensive testing of integrated systems before deployment
4. **Monitoring**: Continuous monitoring of sensor-actuator health
5. **Documentation**: Clear documentation of integration interfaces and protocols

## Summary

Sensor-actuator integration is fundamental to humanoid robotics, enabling robots to perceive their environment and respond appropriately. Successful integration requires careful attention to communication protocols, real-time requirements, safety considerations, and system validation. The examples provided demonstrate practical approaches to implementing robust sensor-actuator integration in humanoid robots.

## See Also

- [Sensors](./sensors.md) - Detailed information about different types of sensors used in humanoid robots
- [Actuators](./actuators.md) - Comprehensive guide to actuator types and applications
- [Control Systems](../fundamentals/control-systems.md) - Understanding control theory for sensor-actuator systems
- [Kinematics](../fundamentals/kinematics.md) - How sensor-actuator integration relates to robot kinematics
- [Perception in AI](../ai-humanoid/perception.md) - How sensor data feeds into AI perception systems