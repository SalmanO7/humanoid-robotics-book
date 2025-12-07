---
sidebar_position: 3
---

# Simulation Examples for Humanoid Robotics

Simulation is essential for developing and testing humanoid robotics algorithms before deployment on real robots. This section provides practical examples using popular simulation environments and techniques.

## Overview of Simulation in Humanoid Robotics

Simulation environments provide:
- **Safe testing**: Experiment with control algorithms without risk to hardware
- **Rapid prototyping**: Test ideas quickly without physical setup
- **Environment variety**: Test in different scenarios and conditions
- **Sensor simulation**: Test perception algorithms with realistic sensor data
- **Physics simulation**: Accurate modeling of robot dynamics and interactions

## PyBullet Simulation Example

### Basic Humanoid Robot Simulation

```python
import pybullet as p
import pybullet_data
import numpy as np
import time

class PyBulletHumanoid:
    """
    Basic humanoid robot simulation using PyBullet
    """
    def __init__(self, urdf_path=None):
        # Connect to physics server
        self.physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set gravity
        p.setGravity(0, 0, -9.81)

        # Load plane
        self.plane_id = p.loadURDF("plane.urdf")

        # Load humanoid robot (using simplified model)
        if urdf_path:
            self.robot_id = p.loadURDF(urdf_path, [0, 0, 1])
        else:
            # Create a simple humanoid-like robot
            self.create_simple_humanoid()

        # Get joint information
        self.joint_info = {}
        self.joint_names = []
        self.joint_indices = []

        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]

            # Only consider revolute joints (motors)
            if joint_type == p.JOINT_REVOLUTE:
                self.joint_names.append(joint_name)
                self.joint_indices.append(i)
                self.joint_info[joint_name] = {
                    'index': i,
                    'type': joint_type,
                    'lower_limit': joint_info[8],
                    'upper_limit': joint_info[9],
                    'max_force': joint_info[10],
                    'max_velocity': joint_info[11]
                }

        print(f"Loaded robot with {len(self.joint_names)} joints: {self.joint_names}")

    def create_simple_humanoid(self):
        """
        Create a simplified humanoid robot using primitive shapes
        """
        # Create links and joints (simplified model)
        # This is a placeholder - in practice you'd load a proper URDF
        # For this example, we'll use a pre-built model
        self.robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])  # Using R2D2 as placeholder

    def get_robot_state(self):
        """
        Get current robot state (joint positions, velocities, etc.)
        """
        joint_states = p.getJointStates(self.robot_id, self.joint_indices)
        positions = [state[0] for state in joint_states]
        velocities = [state[1] for state in joint_states]
        forces = [state[3] for state in joint_states]

        # Get base position and orientation
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)

        return {
            'joint_positions': positions,
            'joint_velocities': velocities,
            'joint_forces': forces,
            'base_position': pos,
            'base_orientation': orn,
            'base_linear_velocity': lin_vel,
            'base_angular_velocity': ang_vel
        }

    def set_joint_positions(self, joint_positions):
        """
        Set target joint positions (position control)
        """
        for i, pos in enumerate(joint_positions):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=self.joint_indices[i],
                controlMode=p.POSITION_CONTROL,
                targetPosition=pos,
                force=self.joint_info[self.joint_names[i]]['max_force']
            )

    def set_joint_velocities(self, joint_velocities):
        """
        Set target joint velocities (velocity control)
        """
        for i, vel in enumerate(joint_velocities):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=self.joint_indices[i],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=vel,
                force=self.joint_info[self.joint_names[i]]['max_force']
            )

    def set_joint_torques(self, joint_torques):
        """
        Apply joint torques (torque control)
        """
        p.setJointMotorControlArray(
            bodyUniqueId=self.robot_id,
            jointIndices=self.joint_indices,
            controlMode=p.TORQUE_CONTROL,
            forces=joint_torques
        )

    def run_simulation(self, steps=1000):
        """
        Run simulation loop
        """
        p.setRealTimeSimulation(0)  # Use stepping instead of real-time

        for step in range(steps):
            # Get current state
            state = self.get_robot_state()

            # Example: Simple balance controller
            # Keep the robot upright by adjusting joint positions based on orientation
            pos, orn = state['base_position'], state['base_orientation']

            # Convert quaternion to Euler angles for simple balance
            euler = p.getEulerFromQuaternion(orn)
            roll, pitch, yaw = euler

            # Simple balance control (proportional to tilt)
            target_angles = [0.0] * len(self.joint_names)  # Default to zero
            target_angles[0] = -pitch * 0.5  # Adjust first joint based on pitch
            target_angles[1] = -roll * 0.5   # Adjust second joint based on roll

            # Apply control
            self.set_joint_positions(target_angles)

            # Step simulation
            p.stepSimulation()

            # Print info every 100 steps
            if step % 100 == 0:
                print(f"Step {step}: Position={pos}, Orientation={euler}")

            time.sleep(1./240.)  # Sleep to slow down visualization

    def reset_robot(self, position=[0, 0, 1], orientation=[0, 0, 0, 1]):
        """
        Reset robot to initial position
        """
        p.resetBasePositionAndOrientation(self.robot_id, position, orientation)
        for i in self.joint_indices:
            p.resetJointState(self.robot_id, i, targetValue=0, targetVelocity=0)

    def disconnect(self):
        """
        Disconnect from physics server
        """
        p.disconnect()

# Example usage
if __name__ == "__main__":
    sim = PyBulletHumanoid()

    print("Running humanoid simulation...")
    sim.run_simulation(steps=2400)  # 10 seconds at 240 Hz

    sim.disconnect()
```

### Advanced Balance Control in PyBullet

```python
import pybullet as p
import pybullet_data
import numpy as np
from scipy.spatial.transform import Rotation as R

class AdvancedBalanceController:
    """
    Advanced balance controller using inverted pendulum model
    """
    def __init__(self):
        # Connect to physics server
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set gravity
        p.setGravity(0, 0, -9.81)

        # Load plane
        self.plane_id = p.loadURDF("plane.urdf")

        # Load robot
        self.robot_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

        # Balance control parameters
        self.com_height = 0.8  # Center of mass height
        self.gravity = 9.81
        self.omega = np.sqrt(self.gravity / self.com_height)

        # PID controller parameters
        self.kp = 100.0
        self.kd = 10.0
        self.ki = 1.0

        # Integration terms
        self.integral_error = np.array([0.0, 0.0])

        # Previous error for derivative term
        self.prev_error = np.array([0.0, 0.0])

    def compute_com_state(self):
        """
        Compute center of mass position and velocity
        """
        # For this example, we'll approximate CoM from base link
        # In a real humanoid, you'd compute CoM from all links
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)

        # Get link masses and positions for accurate CoM calculation
        total_mass = 0
        com_pos = np.array([0.0, 0.0, 0.0])

        # This is a simplified calculation - in reality, compute from all links
        com_pos = np.array(pos)  # Approximate CoM as base position

        # For velocity, use base velocity
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)
        com_vel = np.array(lin_vel)

        return com_pos, com_vel

    def compute_zmp(self, com_pos, com_acc):
        """
        Compute Zero Moment Point
        """
        zmp_x = com_pos[0] - (self.com_height * com_acc[0]) / self.gravity
        zmp_y = com_pos[1] - (self.com_height * com_acc[1]) / self.gravity

        return np.array([zmp_x, zmp_y])

    def balance_control_step(self):
        """
        Perform one step of balance control
        """
        # Get current state
        com_pos, com_vel = self.compute_com_state()

        # Estimate acceleration (in practice, use IMU or state estimation)
        # For simulation, we'll use numerical differentiation
        if not hasattr(self, 'prev_com_vel'):
            self.prev_com_vel = com_vel
            self.prev_time = time.time()
            return  # Skip first iteration

        current_time = time.time()
        dt = current_time - self.prev_time

        if dt > 0:
            com_acc = (np.array(com_vel) - np.array(self.prev_com_vel)) / dt
        else:
            com_acc = np.array([0.0, 0.0, 0.0])

        # Compute current ZMP
        current_zmp = self.compute_zmp(com_pos, com_acc)

        # Desired ZMP (at origin for balance)
        desired_zmp = np.array([0.0, 0.0])

        # Compute error
        error = desired_zmp - current_zmp

        # PID control
        self.integral_error += error * dt
        derivative_error = (error - self.prev_error) / dt if dt > 0 else np.array([0.0, 0.0])

        control_output = (self.kp * error +
                         self.ki * self.integral_error +
                         self.kd * derivative_error)

        # Store for next iteration
        self.prev_error = error
        self.prev_com_vel = com_vel
        self.prev_time = current_time

        return control_output, current_zmp

    def run_balance_simulation(self, steps=2400):
        """
        Run balance simulation
        """
        for step in range(steps):
            # Perform balance control
            control_output, current_zmp = self.balance_control_step()

            # Apply control to robot joints (simplified)
            # In a real humanoid, you'd map the control to specific joints
            # For this example, we'll apply forces to keep robot balanced

            # Print balance status
            if step % 100 == 0:
                com_pos, com_vel = self.compute_com_state()
                print(f"Step {step}: CoM=({com_pos[0]:.3f}, {com_pos[1]:.3f}), "
                      f"ZMP=({current_zmp[0]:.3f}, {current_zmp[1]:.3f}), "
                      f"Control=({control_output[0]:.3f}, {control_output[1]:.3f})")

            # Step simulation
            p.stepSimulation()
            time.sleep(1./240.)

    def disconnect(self):
        p.disconnect()

# Example usage
if __name__ == "__main__":
    controller = AdvancedBalanceController()
    print("Running advanced balance control simulation...")
    controller.run_balance_simulation()
    controller.disconnect()
```

## Gazebo Simulation Example

### Gazebo Plugin for Humanoid Robot

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class HumanoidController : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the model pointer for convenience
      this->model = _parent;

      // Initialize joints
      this->hipJoint = this->model->GetJoint("hip_joint");
      this->kneeJoint = this->model->GetJoint("knee_joint");
      this->ankleJoint = this->model->GetJoint("ankle_joint");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&HumanoidController::OnUpdate, this));

      // Initialize PID controllers
      this->hipPID.Init(1.0, 0.1, 0.01, 0, 0);
      this->kneePID.Init(1.0, 0.1, 0.01, 0, 0);
      this->anklePID.Init(1.0, 0.1, 0.01, 0, 0);

      // Set target positions
      this->targetHipPos = 0.0;
      this->targetKneePos = 0.0;
      this->targetAnklePos = 0.0;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply PID control to joints
      double hipError = this->targetHipPos - this->hipJoint->Position(0);
      double kneeError = this->targetKneePos - this->kneeJoint->Position(0);
      double ankleError = this->targetAnklePos - this->ankleJoint->Position(0);

      double hipForce = this->hipPID.Update(hipError, 0);
      double kneeForce = this->kneePID.Update(kneeError, 0);
      double ankleForce = this->anklePID.Update(ankleError, 0);

      this->hipJoint->SetForce(0, hipForce);
      this->kneeJoint->SetForce(0, kneeForce);
      this->ankleJoint->SetForce(0, ankleForce);
    }

    private: physics::ModelPtr model;
    private: physics::JointPtr hipJoint;
    private: physics::JointPtr kneeJoint;
    private: physics::JointPtr ankleJoint;
    private: common::PID hipPID;
    private: common::PID kneePID;
    private: common::PID anklePID;
    private: double targetHipPos;
    private: double targetKneePos;
    private: double targetAnklePos;
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(HumanoidController)
}
```

## Webots Simulation Example

### Webots Robot Controller in Python

```python
"""HumanoidRobot controller."""

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Accelerometer, Camera, LED, DistanceSensor
import numpy as np

class HumanoidRobot:
    def __init__(self):
        # Create robot instance
        self.robot = Robot()

        # Get the time step of the current world
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize motors
        self.left_hip_motor = self.robot.getDevice('left_hip_motor')
        self.right_hip_motor = self.robot.getDevice('right_hip_motor')
        self.left_knee_motor = self.robot.getDevice('left_knee_motor')
        self.right_knee_motor = self.robot.getDevice('right_knee_motor')
        self.left_ankle_motor = self.robot.getDevice('left_ankle_motor')
        self.right_ankle_motor = self.robot.getDevice('right_ankle_motor')

        # Set motor parameters
        self.left_hip_motor.setPosition(float('inf'))
        self.right_hip_motor.setPosition(float('inf'))
        self.left_knee_motor.setPosition(float('inf'))
        self.right_knee_motor.setPosition(float('inf'))
        self.left_ankle_motor.setPosition(float('inf'))
        self.right_ankle_motor.setPosition(float('inf'))

        self.left_hip_motor.setForce(0)
        self.right_hip_motor.setForce(0)
        self.left_knee_motor.setForce(0)
        self.right_knee_motor.setForce(0)
        self.left_ankle_motor.setForce(0)
        self.right_ankle_motor.setForce(0)

        # Initialize sensors
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.timestep)

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.timestep)

        self.accelerometer = self.robot.getDevice('accelerometer')
        self.accelerometer.enable(self.timestep)

        # Balance control parameters
        self.balance_kp = 10.0
        self.balance_kd = 1.0

    def get_sensor_data(self):
        """Get current sensor data"""
        # Get IMU data
        imu_values = self.imu.getRollPitchYaw()
        roll, pitch, yaw = imu_values

        # Get accelerometer data
        acc_values = self.accelerometer.getValues()

        # Get gyroscope data
        gyro_values = self.gyro.getValues()

        # Get GPS data
        gps_values = self.gps.getValues()

        return {
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'acceleration': acc_values,
            'angular_velocity': gyro_values,
            'position': gps_values
        }

    def balance_control(self, sensor_data):
        """Simple balance control based on pitch angle"""
        pitch_error = -sensor_data['pitch']  # Negative to correct forward lean
        pitch_rate = -sensor_data['angular_velocity'][1]  # Y-axis rotation rate

        # Simple PD control
        control_output = self.balance_kp * pitch_error + self.balance_kd * pitch_rate

        # Limit control output
        control_output = max(-1.0, min(1.0, control_output))

        return control_output

    def step(self):
        """Execute one simulation step"""
        # Get sensor data
        sensor_data = self.get_sensor_data()

        # Compute balance control
        balance_output = self.balance_control(sensor_data)

        # Apply control to joints (simplified)
        # In a real implementation, you'd have more sophisticated control
        self.left_hip_motor.setTorque(balance_output * 10.0)
        self.right_hip_motor.setTorque(balance_output * 10.0)
        self.left_knee_motor.setTorque(balance_output * 5.0)
        self.right_knee_motor.setTorque(balance_output * 5.0)

        # Continue for the next step
        return self.robot.step(self.timestep)

    def run(self):
        """Main control loop"""
        print("Starting humanoid robot simulation...")

        while True:
            if self.step() == -1:
                break

            # Print balance status every 100 steps
            if self.robot.getTime() % 1.0 < 0.01:  # Print every second
                sensor_data = self.get_sensor_data()
                print(f"Time: {self.robot.getTime():.2f}s, "
                      f"Pitch: {sensor_data['pitch']:.3f}, "
                      f"Roll: {sensor_data['roll']:.3f}")

# Example usage (when run as Webots controller)
# humanoid = HumanoidRobot()
# humanoid.run()
```

## MuJoCo Simulation Example

### MuJoCo Humanoid Environment

```python
import mujoco
import mujoco.viewer
import numpy as np

class MujocoHumanoid:
    """
    Humanoid simulation using MuJoCo physics engine
    """
    def __init__(self, model_path=None):
        if model_path:
            self.model = mujoco.MjModel.from_xml_path(model_path)
        else:
            # Create a simple humanoid model in memory
            self.model = self.create_simple_humanoid_model()

        self.data = mujoco.MjData(self.model)

        # Control parameters
        self.balance_kp = 100.0
        self.balance_kd = 10.0

        # Joint indices for key joints
        self.joint_names = [
            'abdomen_y', 'abdomen_z', 'abdomen_x',  # Torso
            'right_hip_x', 'right_hip_z', 'right_hip_y',  # Right leg
            'right_knee',
            'right_ankle_x', 'right_ankle_y',
            'left_hip_x', 'left_hip_z', 'left_hip_y',  # Left leg
            'left_knee',
            'left_ankle_x', 'left_ankle_y',
            'right_shoulder1', 'right_shoulder2', 'right_elbow',  # Right arm
            'left_shoulder1', 'left_shoulder2', 'left_elbow'  # Left arm
        ]

        # Get actuator indices
        self.actuator_names = []
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self.actuator_names.append(name)

    def create_simple_humanoid_model(self):
        """
        Create a simple humanoid XML model
        """
        # This is a simplified example - in practice you'd load a proper XML
        xml_string = """
        <mujoco model="humanoid">
          <compiler angle="degree" coordinate="local" inertiafromgeom="true"/>
          <default>
            <joint armature="1" damping="1" limited="true"/>
            <geom conaffinity="1" condim="1" contype="1" margin="0.001" material="geom" rgba="0.8 0.6 .4 1"/>
            <motor ctrllimited="true" ctrlrange="-.4 .4"/>
          </default>
          <option integrator="RK4" iterations="50" solver="PGS" timestep="0.003">
            <flag energy="enable" forward="disable" inverse="disable" velocity="disable"/>
          </option>
          <size nkey="5" nuser_geom="1"/>
          <visual>
            <map fogend="5" fogstart="3"/>
          </visual>
          <asset>
            <texture builtin="gradient" height="100" rgb1=".4 .5 .6" rgb2="0 0 0" type="skybox" width="100"/>
            <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1" name="texgeom" random="0.01" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" type="cube" width="127"/>
            <texture builtin="checker" height="100" name="texplane" rgb1="0 0 0" rgb2="0.8 0.8 0.8" type="2d" width="100"/>
            <material name="MatPlane" reflectance="0.5" shininess="1" specular="1" texrepeat="60 60" texture="texplane"/>
            <material name="geom" texture="texgeom" texuniform="true"/>
          </asset>
          <worldbody>
            <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
            <geom conaffinity="1" condim="3" name="floor" pos="0 0 0" rgba="0.8 0.9 0.8 1" size="20 20 .125" type="plane" material="MatPlane"/>
            <body name="torso" pos="0 0 1.4">
              <camera name="track" mode="trackcom" pos="0 -4 0" xyaxes="1 0 0 0 0 1"/>
              <geom name="torso1" pos="0 0 0.165" size="0.166 0.233" type="capsule"/>
              <joint name="abdomen_y" pos="0 0 0.065" axis="0 1 0" range="-70 30" damping="5" stiffness="20"/>
              <body name="head" pos="0 0 0.19">
                <geom name="head" size="0.091" type="sphere"/>
              </body>
              <body name="lifoot" pos="0 0 -0.605">
                <geom name="foot_l" size="0.075 0.3225" axisangle="1 0 0 -90" type="capsule"/>
              </body>
              <body name="rifoot" pos="0 0 -0.605">
                <geom name="foot_r" size="0.075 0.3225" axisangle="1 0 0 -90" type="capsule"/>
              </body>
            </body>
          </worldbody>
          <actuator>
            <motor name="abdomen_y_motor" joint="abdomen_y" gear="200"/>
          </actuator>
        </mujoco>
        """

        import tempfile
        import os
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_string)
            temp_path = f.name

        model = mujoco.MjModel.from_xml_path(temp_path)
        os.unlink(temp_path)
        return model

    def get_sensor_data(self):
        """
        Get current sensor readings
        """
        # Get joint positions and velocities
        joint_positions = self.data.qpos
        joint_velocities = self.data.qvel

        # Get accelerometer data (simulate IMU on torso)
        torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'torso')
        torso_accel = self.data.sensordata[:3]  # First 3 values are accelerometer

        # Get gyroscope data
        gyro_data = self.data.sensordata[3:6]   # Next 3 values are gyroscope

        # Get center of mass info
        com_pos = self.data.subtree_com[torso_id]

        return {
            'joint_positions': joint_positions,
            'joint_velocities': joint_velocities,
            'accelerometer': torso_accel,
            'gyroscope': gyro_data,
            'com_position': com_pos,
            'qpos': self.data.qpos,
            'qvel': self.data.qvel
        }

    def balance_control(self, sensor_data):
        """
        Compute balance control torques
        """
        # Simple PD control based on torso orientation
        # In practice, you'd use more sophisticated balance control

        # Get torso orientation from quaternion
        torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'torso')
        torso_quat = self.data.xquat[torso_id]

        # Convert quaternion to Euler angles for simple control
        # (simplified - in practice use proper quaternion math)
        roll = np.arctan2(2*(torso_quat[0]*torso_quat[1] + torso_quat[2]*torso_quat[3]),
                         1 - 2*(torso_quat[1]**2 + torso_quat[2]**2))
        pitch = np.arcsin(2*(torso_quat[0]*torso_quat[2] - torso_quat[3]*torso_quat[1]))

        # Compute control torques based on orientation error
        pitch_error = -pitch  # Negative to correct forward lean
        roll_error = -roll    # Negative to correct sideways lean

        # Simple PD control
        pitch_torque = self.balance_kp * pitch_error
        roll_torque = self.balance_kp * roll_error

        return np.array([pitch_torque, roll_torque])

    def step_simulation(self, control_torques=None):
        """
        Step the simulation forward
        """
        if control_torques is not None:
            # Apply control torques to actuators
            if len(control_torques) <= len(self.data.ctrl):
                self.data.ctrl[:len(control_torques)] = control_torques

        # Step simulation
        mujoco.mj_step(self.model, self.data)

    def run_simulation(self, steps=1000):
        """
        Run simulation loop
        """
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            for step in range(steps):
                # Get sensor data
                sensor_data = self.get_sensor_data()

                # Compute balance control
                control_torques = self.balance_control(sensor_data)

                # Apply control and step simulation
                self.step_simulation(control_torques)

                # Sync viewer
                viewer.sync()

                # Print info every 100 steps
                if step % 100 == 0:
                    print(f"Step {step}: COM=({sensor_data['com_position'][0]:.3f}, "
                          f"{sensor_data['com_position'][1]:.3f}, {sensor_data['com_position'][2]:.3f})")

    def reset_simulation(self):
        """
        Reset simulation to initial state
        """
        mujoco.mj_resetData(self.model, self.data)

# Example usage
if __name__ == "__main__":
    sim = MujocoHumanoid()
    print("Running MuJoCo humanoid simulation...")
    sim.run_simulation(steps=3000)
```

## Unity ML-Agents Example

### Unity Environment for Humanoid Learning

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class HumanoidAgent : Agent
{
    [Header("Joint Objects")]
    public Transform leftHip;
    public Transform rightHip;
    public Transform leftKnee;
    public Transform rightKnee;
    public Transform leftAnkle;
    public Transform rightAnkle;
    public Transform torso;
    public Transform head;

    [Header("Environment")]
    public Transform ground;
    public float groundY;

    [Header("Agent Stats")]
    public float agentHeight;
    public float agentWidth;
    public float agentMass;

    [Header("Rewards")]
    public float upReward;
    public float forwardReward;
    public float velocityReward;
    public float balanceReward;

    private Vector3 previousPosition;
    private float previousVelocity;

    public override void OnEpisodeBegin()
    {
        // Reset agent to initial position
        transform.position = new Vector3(0, 1.5f, 0);
        transform.rotation = Quaternion.identity;

        // Reset joint positions
        ResetJoints();

        // Initialize tracking variables
        previousPosition = transform.position;
        previousVelocity = 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Agent position and velocity
        sensor.AddObservation(transform.position);
        sensor.AddObservation(transform.forward);
        sensor.AddObservation(transform.up);

        // Joint angles
        sensor.AddObservation(leftHip.localEulerAngles);
        sensor.AddObservation(rightHip.localEulerAngles);
        sensor.AddObservation(leftKnee.localEulerAngles);
        sensor.AddObservation(rightKnee.localEulerAngles);
        sensor.AddObservation(leftAnkle.localEulerAngles);
        sensor.AddObservation(rightAnkle.localEulerAngles);

        // Velocities
        Vector3 currentVelocity = (transform.position - previousPosition) / Time.fixedDeltaTime;
        sensor.AddObservation(currentVelocity);
        previousPosition = transform.position;

        // Balance information
        sensor.AddObservation(torso.up);
        sensor.AddObservation(head.up);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Apply actions to joints
        float[] continuousActions = actions.ContinuousActions.ToArray();

        // Map actions to joint torques or positions
        ApplyJointActions(continuousActions);

        // Calculate rewards
        CalculateRewards();

        // Check for termination conditions
        CheckTermination();
    }

    private void ApplyJointActions(float[] actions)
    {
        // Apply actions to joints (simplified example)
        // In practice, you'd have more sophisticated mapping
        if (actions.Length >= 6)
        {
            // Apply torques to hip joints
            ApplyTorqueToJoint(leftHip, actions[0]);
            ApplyTorqueToJoint(rightHip, actions[1]);

            // Apply torques to knee joints
            ApplyTorqueToJoint(leftKnee, actions[2]);
            ApplyTorqueToJoint(rightKnee, actions[3]);

            // Apply torques to ankle joints
            ApplyTorqueToJoint(leftAnkle, actions[4]);
            ApplyTorqueToJoint(rightAnkle, actions[5]);
        }
    }

    private void ApplyTorqueToJoint(Transform joint, float torque)
    {
        // Apply torque to joint (simplified)
        // In practice, use proper physics and joint constraints
        joint.GetComponent<Rigidbody>().AddTorque(Vector3.up * torque * 10f);
    }

    private void CalculateRewards()
    {
        // Reward for staying upright
        float upReward = Vector3.Dot(transform.up, Vector3.up);
        SetReward(upReward * this.upReward);

        // Reward for moving forward
        Vector3 currentVelocity = (transform.position - previousPosition) / Time.fixedDeltaTime;
        float forwardReward = Vector3.Dot(currentVelocity, transform.forward);
        AddReward(forwardReward * this.forwardReward);

        // Reward for maintaining velocity
        float velocityReward = Mathf.Abs(currentVelocity.magnitude - previousVelocity);
        AddReward(velocityReward * this.velocityReward);
        previousVelocity = currentVelocity.magnitude;

        // Reward for balance
        float balanceReward = Vector3.Dot(torso.up, Vector3.up);
        AddReward(balanceReward * this.balanceReward);
    }

    private void CheckTermination()
    {
        // Check if agent has fallen
        if (transform.position.y < groundY + 0.5f)
        {
            SetReward(-1f);
            EndEpisode();
        }

        // Check if agent has moved too far
        if (Mathf.Abs(transform.position.x) > 20f || Mathf.Abs(transform.position.z) > 20f)
        {
            EndEpisode();
        }

        // Terminate if episode is too long
        if (HasTimedOut())
        {
            EndEpisode();
        }
    }

    private void ResetJoints()
    {
        // Reset joint positions to neutral
        leftHip.localRotation = Quaternion.identity;
        rightHip.localRotation = Quaternion.identity;
        leftKnee.localRotation = Quaternion.identity;
        rightKnee.localRotation = Quaternion.identity;
        leftAnkle.localRotation = Quaternion.identity;
        rightAnkle.localRotation = Quaternion.identity;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Simple heuristic for testing (arrow keys)
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");  // Forward/back
        continuousActionsOut[1] = Input.GetAxis("Horizontal"); // Left/right
        continuousActionsOut[2] = Input.GetKey(KeyCode.Q) ? 1f : 0f; // Jump
        continuousActionsOut[3] = Input.GetKey(KeyCode.E) ? 1f : 0f; // Crouch
    }
}
```

## Simulation Integration with Real Robot

### Simulation-to-Reality Transfer Example

```python
import numpy as np
import pickle
import os

class Sim2RealTransfer:
    """
    Tools for transferring policies from simulation to reality
    """
    def __init__(self):
        self.sim_model_params = {}
        self.real_model_params = {}
        self.domain_randomization = True
        self.imitation_learning = True

    def randomize_simulation(self):
        """
        Apply domain randomization to simulation parameters
        """
        if self.domain_randomization:
            # Randomize physical parameters within reasonable bounds
            randomized_params = {
                'mass_multiplier': np.random.uniform(0.8, 1.2),
                'friction_coefficient': np.random.uniform(0.5, 1.5),
                'com_offset': np.random.uniform(-0.05, 0.05, 3),
                'sensor_noise': np.random.uniform(0.001, 0.01),
                'actuator_delay': np.random.uniform(0.0, 0.02)
            }
            return randomized_params
        return {}

    def adapt_policy(self, sim_policy, sim_state, real_state):
        """
        Adapt policy trained in simulation for real robot
        """
        # State mapping from sim to real
        state_diff = real_state - sim_state

        # Adapt policy based on state differences
        adapted_policy = sim_policy.copy()
        adapted_policy += state_diff * 0.1  # Simple adaptation factor

        return adapted_policy

    def train_with_domain_randomization(self, env, policy_network, episodes=1000):
        """
        Train policy with domain randomization
        """
        for episode in range(episodes):
            # Randomize environment parameters
            random_params = self.randomize_simulation()
            env.update_parameters(random_params)

            # Run episode with randomized parameters
            state = env.reset()
            done = False
            episode_reward = 0

            while not done:
                action = policy_network.get_action(state)
                next_state, reward, done, info = env.step(action)
                episode_reward += reward
                state = next_state

            print(f"Episode {episode}: Reward = {episode_reward:.2f}, "
                  f"Params = {random_params}")

    def system_identification(self, real_robot, sim_robot):
        """
        Identify system parameters for better sim-to-real transfer
        """
        # Collect data from real robot
        real_data = self.collect_real_robot_data(real_robot)

        # Compare with simulation
        sim_data = self.collect_sim_data(sim_robot)

        # Compute parameter differences
        param_diffs = self.compute_parameter_differences(real_data, sim_data)

        # Update simulation to match real robot better
        self.update_simulation_parameters(param_diffs)

        return param_diffs

    def collect_real_robot_data(self, robot):
        """
        Collect data from real robot for system identification
        """
        data = {
            'joint_positions': [],
            'joint_velocities': [],
            'torques': [],
            'imu_data': [],
            'timestamps': []
        }

        # Collect data for system identification
        for i in range(1000):  # Collect 1000 samples
            joint_pos = robot.get_joint_positions()
            joint_vel = robot.get_joint_velocities()
            torques = robot.get_joint_torques()
            imu_data = robot.get_imu_data()

            data['joint_positions'].append(joint_pos)
            data['joint_velocities'].append(joint_vel)
            data['torques'].append(torques)
            data['imu_data'].append(imu_data)
            data['timestamps'].append(i * 0.01)  # 100 Hz

        return data

    def compute_parameter_differences(self, real_data, sim_data):
        """
        Compute differences between real and simulated data
        """
        # Compute differences in dynamics, friction, etc.
        differences = {}

        # Example: Compare joint position responses
        real_pos = np.array(real_data['joint_positions'])
        sim_pos = np.array(sim_data['joint_positions'])

        pos_error = np.mean(np.abs(real_pos - sim_pos), axis=0)
        differences['position_error'] = pos_error.tolist()

        return differences

    def save_transfer_model(self, model, filepath):
        """
        Save model for sim-to-real transfer
        """
        with open(filepath, 'wb') as f:
            pickle.dump(model, f)

    def load_transfer_model(self, filepath):
        """
        Load model for sim-to-real transfer
        """
        with open(filepath, 'rb') as f:
            return pickle.load(f)

# Example usage
if __name__ == "__main__":
    transfer_tool = Sim2RealTransfer()

    print("Simulation-to-reality transfer tools initialized")
    print("Features available:")
    print("- Domain randomization")
    print("- Policy adaptation")
    print("- System identification")
    print("- Model saving/loading")
```

## Summary

These simulation examples demonstrate various approaches to humanoid robotics simulation:

1. **PyBullet**: Physics-based simulation with good performance and Python API
2. **Gazebo**: ROS-integrated simulation with realistic sensors
3. **Webots**: Cross-platform simulation with built-in controllers
4. **MuJoCo**: High-performance physics simulation for research
5. **Unity ML-Agents**: Game engine-based simulation for learning
6. **Sim-to-Real Transfer**: Techniques for bridging simulation and reality

Simulation is crucial for humanoid robotics development, allowing for safe testing, rapid prototyping, and algorithm validation before deployment on expensive hardware. The examples show how to implement basic simulation environments and control systems that can be extended for specific humanoid robot platforms.

Each simulation environment has its strengths and is suitable for different aspects of humanoid robotics development, from basic kinematics and dynamics validation to complex learning and control algorithm development.