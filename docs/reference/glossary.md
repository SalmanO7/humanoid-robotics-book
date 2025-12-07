---
sidebar_position: 1
---

# Glossary of Humanoid Robotics Terms

This glossary provides definitions for key terms used throughout the Physical AI & Humanoid Robotics book.

## A

**Actuator** - A component of a robot that converts energy (usually electrical) into physical motion. Common types include electric motors, hydraulic actuators, and pneumatic actuators.

**Admittance Control** - A control strategy where the robot's motion is determined by applied forces, making it compliant to environmental contacts.

**Angular Momentum** - The rotational equivalent of linear momentum, calculated as the cross product of the position vector and linear momentum vector.

**Ankle Strategy** - A balance recovery mechanism where a bipedal robot uses ankle torques to maintain balance under small perturbations.

## B

**Balance Control** - Control strategies designed to maintain the robot's center of mass within its support polygon to prevent falling.

**Balance Recovery** - The process of returning to a stable state after experiencing a disturbance that threatens stability.

**Bipedal Locomotion** - Two-legged walking motion, characteristic of human and humanoid robot locomotion.

**Body Schema** - Internal representation of the robot's body configuration and spatial relationships between its parts.

## C

**Capture Point** - A location where a robot should step to halt its motion. It's calculated based on the robot's current center of mass position and velocity.

**Center of Mass (CoM)** - The point where the total mass of the robot can be considered to be concentrated for the purpose of dynamic analysis.

**Center of Pressure (CoP)** - The point where the ground reaction forces can be considered to act, typically used in balance analysis.

**Compliance Control** - Control strategy that allows controlled flexibility in robot motion, often using spring-damper models.

**Configuration Space (C-space)** - The space of all possible joint configurations of a robot.

**Control Lyapunov Function** - A mathematical function used to prove the stability of control systems.

**Cyclic Gait** - A walking pattern that repeats the same sequence of movements for each step.

## D

**Degrees of Freedom (DOF)** - The number of independent parameters that define the configuration of a mechanical system.

**Dynamical Movement Primitives (DMPs)** - A method for representing and generating movements using autonomous nonlinear dynamical systems.

**Dynamic Balance** - Balance maintained during motion, where the center of mass may be outside the support polygon for brief periods.

**Dynamic Model** - Mathematical representation of how forces and torques affect robot motion, including inertial, Coriolis, and gravitational effects.

**Dynamic Walking** - Walking gait that uses controlled falling to achieve efficient locomotion, similar to human walking.

## E

**End-Effector** - The terminal device of a robot manipulator that interacts with the environment, such as a gripper or tool.

**Exteroceptive Sensors** - Sensors that measure properties of the external environment, such as cameras, LIDAR, or touch sensors.

**Equilibrium Point** - A state where all forces and torques acting on the robot sum to zero.

## F

**Feedback Linearization** - A control technique that transforms a nonlinear system into a linear one through feedback.

**Force Control** - Control strategy that regulates the forces applied by the robot to the environment.

**Forward Kinematics** - The calculation of the position and orientation of the end-effector given the joint angles.

**Friction Compensation** - Control techniques that account for and compensate the effects of friction in robotic joints.

## G

**Gait** - The pattern of limb movements used for locomotion.

**Gait Phase** - A specific stage of the walking cycle, such as stance phase or swing phase.

**Gaussian Process** - A probabilistic model used for regression and classification tasks, particularly useful for modeling uncertainty.

**Generalized Coordinates** - A set of parameters that define the configuration of a mechanical system relative to a reference configuration.

## H

**Hardware-in-the-Loop (HIL)** - A testing method where real hardware components are integrated with simulated environments.

**Hierarchical Control** - Control architecture organized in multiple levels, each responsible for different aspects of robot behavior.

**Hip Strategy** - A balance recovery mechanism where a bipedal robot uses hip torques to maintain balance under moderate perturbations.

**Humanoid Robot** - A robot with physical features resembling the human body, typically including a head, torso, two arms, and two legs.

## I

**Impedance Control** - A control strategy that regulates the dynamic relationship between position and force at the robot's contact points.

**Inverse Dynamics** - The calculation of required joint torques to achieve a desired motion trajectory.

**Inverse Kinematics** - The calculation of joint angles required to achieve a desired end-effector position and orientation.

**Inverted Pendulum Model** - A simplified model of balance where the robot is represented as a point mass on a massless rod.

**Inertial Measurement Unit (IMU)** - A device that measures specific force, angular rate, and sometimes magnetic field, used for navigation and motion tracking.

## J

**Jacobian Matrix** - A matrix that describes the relationship between joint velocities and end-effector velocities.

**Joint Space** - The space defined by the robot's joint angles.

**Jerk** - The rate of change of acceleration; important for smooth motion planning.

## K

**Kinematics** - The study of motion without considering the forces that cause it.

**Kinetic Energy** - The energy possessed by an object due to its motion.

**Kinodynamic Planning** - Motion planning that considers both kinematic and dynamic constraints.

## L

**Legged Locomotion** - Movement using legs, as opposed to wheeled or tracked locomotion.

**Linear Inverted Pendulum Model (LIPM)** - A simplified balance model assuming constant center of mass height.

**Limit Cycle** - A periodic orbit to which neighboring trajectories converge, often used to describe stable walking patterns.

**Lyapunov Stability** - A mathematical concept used to prove the stability of dynamical systems.

## M

**Manipulability** - A measure of how easily a robot can move in different directions in task space.

**Model Predictive Control (MPC)** - An advanced control method that uses a model of the system to predict future behavior and optimize control actions.

**Multi-Body Dynamics** - The study of motion of interconnected rigid bodies under the influence of forces.

**Machine Learning** - A field of artificial intelligence focused on algorithms that can learn patterns from data.

## N

**Null Space** - The space of joint motions that do not affect the task-space motion of the end-effector.

**Neural Network** - A computational model inspired by biological neural networks, used for pattern recognition and control.

**Nonholonomic Constraint** - A constraint that cannot be integrated to form a constraint on positions alone.

## O

**Operational Space** - The space in which a robot performs its primary tasks, often end-effector position and orientation.

**Operational Space Control** - Control method that operates in task space rather than joint space.

**Odometry** - The use of data from motion sensors to estimate change in position over time.

**Optimization-Based Control** - Control strategies that formulate control problems as optimization problems.

## P

**Path Planning** - The process of determining a route from start to goal while avoiding obstacles.

**Pendulum Planning** - Motion planning based on pendulum models of robot dynamics.

**Perception** - The process of interpreting sensor data to understand the environment.

**PID Controller** - A control loop feedback mechanism using Proportional, Integral, and Derivative terms.

**Poincaré Map** - A mathematical tool used to analyze periodic orbits in dynamical systems.

**Proprioceptive Sensors** - Sensors that measure the internal state of the robot, such as joint encoders or force sensors.

## R

**Reinforcement Learning** - A type of machine learning where an agent learns to make decisions by performing actions and receiving rewards.

**Rigid Body** - An idealized solid body in which deformation is neglected, used in dynamic modeling.

**Robot Operating System (ROS)** - Flexible framework for writing robot software.

**Rolling Constraint** - A constraint that prevents sliding between contacting surfaces.

**RRT (Rapidly-exploring Random Tree)** - A motion planning algorithm that builds a tree of feasible configurations.

## S

**Sensor Fusion** - The process of combining data from multiple sensors to improve accuracy and reliability.

**Simultaneous Localization and Mapping (SLAM)** - The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location.

**Singularity** - A configuration where the robot loses one or more degrees of freedom, often causing control problems.

**Stability** - The property of a system to return to equilibrium after being disturbed.

**Static Balance** - Balance maintained when the center of mass is always within the support polygon.

**Static Walking** - Walking gait where the center of mass is always within the support polygon.

**Stiffness Control** - Control strategy that regulates the apparent stiffness of the robot at contact points.

**Support Polygon** - The convex hull of all ground contact points, used in balance analysis.

**Swing Leg** - The leg that is not in contact with the ground during walking.

## T

**Task Space** - The space in which the robot's primary task is defined, such as end-effector position.

**Trajectory Planning** - The process of determining a time-parameterized path for robot motion.

**Tactile Sensors** - Sensors that detect touch, pressure, or other physical contact.

**Torque Control** - Control strategy that directly regulates the torques applied by the robot's actuators.

**Trunk** - The main body section of a humanoid robot, typically connecting the head, arms, and legs.

## V

**Velocity Product Terms** - Coriolis and centrifugal forces that appear in robot dynamics equations.

**Virtual Model Control** - A control approach that uses virtual components to achieve desired robot behavior.

**Visual Servoing** - Robot control using visual feedback to guide motion.

## W

**Walking Pattern Generator** - A system that creates the coordinated joint motions needed for walking.

**Whole-Body Control** - Control approach that coordinates all degrees of freedom to achieve multiple tasks simultaneously.

**ZMP (Zero Moment Point)** - A point where the net moment of ground reaction forces is zero, used in balance control.

## X, Y, Z

**Yaw** - Rotation about the vertical axis.

**ZMP-based Control** - Control strategy based on regulating the Zero Moment Point position.