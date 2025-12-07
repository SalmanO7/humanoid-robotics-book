---
sidebar_position: 2
---

# ROS Integration Examples for Humanoid Robotics

Robot Operating System (ROS) provides the framework for communication and coordination in humanoid robotics systems. This section provides practical ROS code examples for implementing key concepts in humanoid robotics.

## Overview of ROS in Humanoid Robotics

ROS is widely used in humanoid robotics for:
- **Message passing**: Communication between different robot components
- **Hardware abstraction**: Interface with sensors and actuators
- **Package management**: Organize and share robot software
- **Visualization tools**: Debug and monitor robot behavior
- **Simulation**: Test algorithms in simulation before deployment

## Basic ROS Publisher and Subscriber

### Publisher Example - Joint State Publisher

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import math

class JointStatePublisher:
    """
    Publish joint states for a simple robot
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('joint_state_publisher', anonymous=True)

        # Create publisher for joint states
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Set publishing rate (100 Hz)
        self.rate = rospy.Rate(100)

        # Initialize joint names and positions
        self.joint_names = ['hip_joint', 'knee_joint', 'ankle_joint']
        self.joint_positions = [0.0, 0.0, 0.0]

    def update_joints(self):
        """
        Update joint positions with simple oscillating motion
        """
        current_time = rospy.Time.now()

        # Update joint positions with oscillating motion
        self.joint_positions[0] = 0.5 * math.sin(current_time.to_sec())
        self.joint_positions[1] = 0.3 * math.sin(current_time.to_sec() * 1.5)
        self.joint_positions[2] = 0.2 * math.sin(current_time.to_sec() * 2.0)

    def run(self):
        """
        Main publishing loop
        """
        while not rospy.is_shutdown():
            # Create joint state message
            msg = JointState()
            msg.name = self.joint_names
            msg.position = self.joint_positions
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"

            # Publish the message
            self.pub.publish(msg)

            # Update joint positions
            self.update_joints()

            # Sleep to maintain rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = JointStatePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
```

### Subscriber Example - IMU Data Processor

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUProcessor:
    """
    Process IMU data and estimate orientation
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('imu_processor', anonymous=True)

        # Subscribe to IMU topic
        self.sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # Initialize orientation
        self.orientation = R.from_quat([0, 0, 0, 1])
        self.last_time = None

    def imu_callback(self, msg):
        """
        Process incoming IMU message
        """
        current_time = msg.header.stamp.to_sec()

        if self.last_time is not None:
            dt = current_time - self.last_time

            # Extract angular velocity
            angular_vel = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            # Integrate angular velocity to update orientation
            rotation_vector = angular_vel * dt
            if np.linalg.norm(rotation_vector) > 1e-6:
                angle = np.linalg.norm(rotation_vector)
                axis = rotation_vector / angle
                d_rotation = R.from_rotvec(axis * angle)
                self.orientation = d_rotation * self.orientation

        self.last_time = current_time

        # Print current orientation
        euler_angles = self.orientation.as_euler('xyz', degrees=True)
        rospy.loginfo(f"Current orientation: [{euler_angles[0]:.2f}, {euler_angles[1]:.2f}, {euler_angles[2]:.2f}] degrees")

    def run(self):
        """
        Keep the node running
        """
        rospy.spin()

if __name__ == '__main__':
    processor = IMUProcessor()
    processor.run()
```

## Service Example - Robot Control Service

```python
#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool
from humanoid_robot_msgs.srv import SetJointPosition, SetJointPositionRequest, SetJointPositionResponse

class RobotController:
    """
    Robot controller with service interface
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robot_controller', anonymous=True)

        # Initialize joint positions
        self.joint_positions = {}

        # Create services
        self.enable_service = rospy.Service('/robot/enable', SetBool, self.enable_callback)
        self.move_joint_service = rospy.Service('/robot/move_joint', SetJointPosition, self.move_joint_callback)

        rospy.loginfo("Robot controller services are ready")

    def enable_callback(self, req):
        """
        Enable/disable robot
        """
        if req.data:
            rospy.loginfo("Robot enabled")
            # Add code to enable robot hardware
        else:
            rospy.loginfo("Robot disabled")
            # Add code to disable robot hardware

        return True, "Robot enabled" if req.data else "Robot disabled"

    def move_joint_callback(self, req):
        """
        Move specified joint to target position
        """
        joint_name = req.joint_name
        target_position = req.position

        # Store the target position (in real implementation, send to hardware)
        self.joint_positions[joint_name] = target_position

        rospy.loginfo(f"Moving {joint_name} to {target_position:.3f} radians")

        # In real implementation, you would:
        # 1. Plan the trajectory
        # 2. Send commands to the joint controller
        # 3. Monitor the execution

        return True, f"Successfully moved {joint_name} to {target_position:.3f} radians"

    def run(self):
        """
        Keep the node running
        """
        rospy.spin()

if __name__ == '__main__':
    controller = RobotController()
    controller.run()
```

## Action Example - Walking Action Server

```python
#!/usr/bin/env python3

import rospy
import actionlib
from humanoid_robot_msgs.msg import WalkAction, WalkGoal, WalkResult, WalkFeedback

class WalkingServer:
    """
    Action server for walking control
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('walking_server')

        # Create action server
        self._as = actionlib.SimpleActionServer(
            'walk_action',
            WalkAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()

        rospy.loginfo("Walking action server started")

    def execute_cb(self, goal):
        """
        Execute walking action
        """
        # Helper variables
        success = True
        rate = rospy.Rate(10)  # 10 Hz control loop

        # Start walking
        rospy.loginfo(f"Starting to walk {goal.distance} meters at {goal.speed} m/s")

        # Initialize feedback
        feedback = WalkFeedback()
        result = WalkResult()

        # Simulate walking progress
        current_distance = 0.0
        step_size = goal.speed / 10.0  # 10 steps per second at target speed

        while current_distance < goal.distance:
            # Check for preemption
            if self._as.is_preempt_requested():
                rospy.loginfo("Walk action preempted")
                self._as.set_preempted()
                success = False
                break

            # Update progress
            current_distance += step_size
            if current_distance > goal.distance:
                current_distance = goal.distance

            # Publish feedback
            feedback.distance_traveled = current_distance
            feedback.remaining_distance = goal.distance - current_distance
            self._as.publish_feedback(feedback)

            # Sleep to maintain rate
            rate.sleep()

            # Simulate balance control (in real implementation, run balance controller)
            # Check for stability, adjust as needed

        if success:
            result.success = True
            result.distance_traveled = current_distance
            rospy.loginfo(f"Successfully walked {current_distance:.2f} meters")
            self._as.set_succeeded(result)
        else:
            result.success = False
            result.distance_traveled = current_distance
            self._as.set_aborted(result)

if __name__ == '__main__':
    server = WalkingServer()
    rospy.spin()
```

## ROS Launch File Example

```xml
<!-- humanoid_robot.launch -->
<launch>
  <!-- Robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

  <!-- Joint state publisher -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="true" />
  </node>

  <!-- IMU processor -->
  <node name="imu_processor" pkg="humanoid_robot" type="imu_processor.py" output="screen" />

  <!-- Robot controller -->
  <node name="robot_controller" pkg="humanoid_robot" type="robot_controller.py" output="screen" />

  <!-- Walking server -->
  <node name="walking_server" pkg="humanoid_robot" type="walking_server.py" output="screen" />

  <!-- Visualization -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find humanoid_robot)/config/humanoid_robot.rviz" />

  <!-- Load robot description -->
  <param name="robot_description" command="$(find xacro)/xacro $(find humanoid_robot)/urdf/humanoid_robot.xacro" />

  <!-- TF publisher -->
  <node name="tf_publisher" pkg="tf" type="static_transform_publisher" args="0 0 0 0 0 0 base_link torso 100" />

</launch>
```

## TF (Transform) Example - Coordinate Frame Management

```python
#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped
from std_msgs.msg import Header

class CoordinateFrameManager:
    """
    Manage coordinate frames for humanoid robot
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('coordinate_frame_manager', anonymous=True)

        # Create TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Create TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Set publishing rate
        self.rate = rospy.Rate(100)

    def broadcast_transforms(self):
        """
        Broadcast coordinate frame transforms
        """
        current_time = rospy.Time.now()

        # Example: Transform from base_link to torso
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "base_link"
        t.child_frame_id = "torso"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.8  # Torso height
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

        # Example: Transform from torso to head
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "torso"
        t.child_frame_id = "head"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.3  # Head height above torso
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

        # Example: Transform from base_link to left_foot
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "base_link"
        t.child_frame_id = "left_foot"
        t.transform.translation.x = 0.1  # Slightly forward
        t.transform.translation.y = 0.1  # Slightly to the side
        t.transform.translation.z = 0.0  # On the ground
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

    def lookup_transform(self, target_frame, source_frame):
        """
        Lookup transform between two frames
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0)
            )
            return transform
        except tf2_ros.LookupException as e:
            rospy.logwarn(f"Transform lookup failed: {e}")
            return None

    def run(self):
        """
        Main loop
        """
        while not rospy.is_shutdown():
            # Broadcast transforms
            self.broadcast_transforms()

            # Example: Lookup transform from head to base_link
            transform = self.lookup_transform("base_link", "head")
            if transform:
                rospy.loginfo(f"Head position relative to base: "
                             f"({transform.transform.translation.x:.3f}, "
                             f"{transform.transform.translation.y:.3f}, "
                             f"{transform.transform.translation.z:.3f})")

            self.rate.sleep()

if __name__ == '__main__':
    manager = CoordinateFrameManager()
    manager.run()
```

## ROS Parameter Server Example

```python
#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
from humanoid_robot.cfg import BalanceParamsConfig

class BalanceController:
    """
    Balance controller with dynamic reconfiguration
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('balance_controller', anonymous=True)

        # Load parameters from parameter server
        self.com_height = rospy.get_param('~com_height', 0.85)  # Default 85cm
        self.control_frequency = rospy.get_param('~control_frequency', 100)  # Default 100Hz
        self.stability_threshold = rospy.get_param('~stability_threshold', 0.05)  # Default 5cm

        # Initialize dynamic reconfigure server
        self.srv = Server(BalanceParamsConfig, self.reconfigure_callback)

        # Initialize balance parameters
        self.kp = rospy.get_param('~balance_kp', 10.0)
        self.kd = rospy.get_param('~balance_kd', 1.0)
        self.max_control_effort = rospy.get_param('~max_control_effort', 50.0)

        rospy.loginfo(f"Balance controller initialized with parameters:")
        rospy.loginfo(f"  CoM Height: {self.com_height}m")
        rospy.loginfo(f"  Control Frequency: {self.control_frequency}Hz")
        rospy.loginfo(f"  KP: {self.kp}, KD: {self.kd}")

    def reconfigure_callback(self, config, level):
        """
        Callback for dynamic reconfiguration
        """
        rospy.loginfo(f"Reconfiguring balance parameters: "
                     f"KP={config.kp}, KI={config.ki}, KD={config.kd}")

        # Update parameters
        self.kp = config.kp
        self.ki = config.ki
        self.kd = config.kd
        self.com_height = config.com_height

        return config

    def balance_step(self, com_error, com_velocity):
        """
        Perform one step of balance control
        """
        # Simple PD control
        control_output = self.kp * com_error + self.kd * com_velocity

        # Limit control effort
        control_output = max(-self.max_control_effort, min(control_output, self.max_control_effort))

        return control_output

    def run(self):
        """
        Main control loop
        """
        rate = rospy.Rate(self.control_frequency)

        # Simulate balance control
        com_position = 0.0
        com_velocity = 0.0

        while not rospy.is_shutdown():
            # Simulate CoM error (in real robot, this comes from sensors)
            com_error = com_position  # Deviation from desired position

            # Calculate control output
            control_effort = self.balance_step(com_error, com_velocity)

            # Simulate system response (in real robot, this affects actuators)
            com_velocity += control_effort * 0.01  # Simplified dynamics
            com_position += com_velocity * 0.01

            # Check stability
            if abs(com_position) > self.stability_threshold:
                rospy.logwarn(f"Balance error too large: {com_position:.3f}m")

            rate.sleep()

if __name__ == '__main__':
    controller = BalanceController()
    controller.run()
```

## ROS Node for Sensor Fusion

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
import numpy as np
from scipy.spatial.transform import Rotation as R

class SensorFusionNode:
    """
    Fuse data from multiple sensors for state estimation
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('sensor_fusion_node', anonymous=True)

        # Subscribe to sensor topics
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        # Publish fused state
        self.com_pub = rospy.Publisher('/state_estimator/com_position', Vector3Stamped, queue_size=10)
        self.com_vel_pub = rospy.Publisher('/state_estimator/com_velocity', Vector3Stamped, queue_size=10)
        self.balance_pub = rospy.Publisher('/state_estimator/balance_metric', Float64, queue_size=10)

        # Initialize state variables
        self.orientation = R.from_quat([0, 0, 0, 1])
        self.com_position = np.array([0.0, 0.0, 0.85])  # Initial CoM position
        self.com_velocity = np.array([0.0, 0.0, 0.0])

        # Last IMU time for integration
        self.last_imu_time = None

        # Joint state cache
        self.joint_positions = {}
        self.joint_velocities = {}

        rospy.loginfo("Sensor fusion node initialized")

    def imu_callback(self, msg):
        """
        Process IMU data
        """
        current_time = msg.header.stamp.to_sec()

        # Update orientation from quaternion
        self.orientation = R.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # Get linear acceleration in world frame
        accel_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Convert to world frame
        accel_world = self.orientation.apply(accel_body)

        # Remove gravity
        gravity = np.array([0, 0, -9.81])
        linear_acceleration = accel_world - gravity

        # Integrate to update velocity and position
        if self.last_imu_time is not None:
            dt = current_time - self.last_imu_time

            # Update velocity (with simple low-pass filtering)
            alpha = 0.1  # Filter coefficient
            self.com_velocity = (1-alpha) * self.com_velocity + alpha * (self.com_velocity + linear_acceleration * dt)

            # Update position
            self.com_position += self.com_velocity * dt

        self.last_imu_time = current_time

        # Publish CoM state
        self.publish_state()

    def joint_callback(self, msg):
        """
        Process joint state data
        """
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def publish_state(self):
        """
        Publish estimated CoM state
        """
        # Publish CoM position
        com_msg = Vector3Stamped()
        com_msg.header.stamp = rospy.Time.now()
        com_msg.header.frame_id = "world"
        com_msg.vector.x = self.com_position[0]
        com_msg.vector.y = self.com_position[1]
        com_msg.vector.z = self.com_position[2]
        self.com_pub.publish(com_msg)

        # Publish CoM velocity
        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.header.frame_id = "world"
        vel_msg.vector.x = self.com_velocity[0]
        vel_msg.vector.y = self.com_velocity[1]
        vel_msg.vector.z = self.com_velocity[2]
        self.com_vel_pub.publish(vel_msg)

        # Publish balance metric (distance from center)
        balance_metric = np.sqrt(self.com_position[0]**2 + self.com_position[1]**2)
        balance_msg = Float64()
        balance_msg.data = balance_metric
        self.balance_pub.publish(balance_msg)

    def run(self):
        """
        Keep node running
        """
        rospy.spin()

if __name__ == '__main__':
    fusion_node = SensorFusionNode()
    fusion_node.run()
```

## ROS Package Structure

A typical ROS package for humanoid robotics would have this structure:

```
humanoid_robot/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── balance_params.yaml
│   └── controller_params.yaml
├── launch/
│   ├── humanoid_robot.launch
│   └── simulation.launch
├── src/
│   ├── joint_state_publisher.py
│   ├── imu_processor.py
│   ├── robot_controller.py
│   └── walking_server.py
├── scripts/
│   └── startup.sh
├── urdf/
│   ├── humanoid_robot.urdf
│   └── humanoid_robot.xacro
└── rviz/
    └── humanoid_robot.rviz
```

## Common ROS Messages for Humanoid Robotics

### Custom Message Example - Balance State

```python
# balance_state.msg
Header header
float64 com_x
float64 com_y
float64 com_z
float64 com_velocity_x
float64 com_velocity_y
float64 com_velocity_z
float64 zmp_x
float64 zmp_y
float64 capture_point_x
float64 capture_point_y
float64 balance_metric
bool is_stable
```

### Custom Message Example - Walking Command

```python
# walk_command.msg
float64 distance
float64 speed
float64 step_height
float64 step_width
bool turn_in_place
float64 turn_angle
```

## Summary

These ROS examples demonstrate fundamental concepts in humanoid robotics:

1. **Publishers/Subscribers**: For real-time sensor and actuator communication
2. **Services**: For synchronous robot control commands
3. **Actions**: For long-running tasks with feedback
4. **TF**: For coordinate frame management and transformations
5. **Parameter Server**: For configuration and dynamic reconfiguration
6. **Sensor Fusion**: For combining multiple sensor inputs
7. **Launch Files**: For starting multiple nodes together

ROS provides the communication infrastructure that enables complex humanoid robot behaviors by allowing different software components to work together seamlessly. The examples show how to implement key robotics concepts using ROS patterns and best practices.

These examples can be adapted and extended for specific humanoid robot platforms, providing a foundation for building sophisticated robotic systems with ROS.