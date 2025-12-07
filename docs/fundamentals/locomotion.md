---
sidebar_position: 4
---

# Locomotion in Humanoid Robotics

Locomotion is one of the most challenging aspects of humanoid robotics. This chapter explores the principles, techniques, and control strategies for enabling humanoid robots to move effectively in various environments.

## Introduction to Humanoid Locomotion

Humanoid locomotion aims to replicate human-like movement patterns, including walking, running, and other forms of bipedal gait. Unlike wheeled or tracked robots, humanoid robots must maintain balance while moving, making locomotion significantly more complex.

## Types of Locomotion

### Static Walking

Static walking maintains stability at all times by keeping the center of mass (CoM) within the support polygon formed by the feet.

**Characteristics:**
- Always statically stable
- Slower than human walking
- Lower energy efficiency
- Safer for uncertain environments

### Dynamic Walking

Dynamic walking uses controlled falling to achieve more human-like, efficient movement.

**Characteristics:**
- Periods of instability are controlled
- More energy efficient
- Faster than static walking
- Requires sophisticated balance control

### Running

Running involves flight phases where both feet are off the ground.

**Characteristics:**
- High-speed locomotion
- Complex balance requirements
- High impact forces
- Limited to research platforms

## Gait Phases

### Single Support Phase

During single support, only one foot is in contact with the ground:
- **Stance leg**: Supports the robot's weight
- **Swing leg**: Moves forward to prepare for next step
- **Balance challenge**: Maintaining stability with single point of contact

### Double Support Phase

During double support, both feet are in contact with the ground:
- **Stability**: Maximum stability
- **Transition**: Shift from one stance leg to the other
- **Duration**: Brief in human-like walking

### Flight Phase (Running)

In running gaits, both feet leave the ground:
- **Aerial phase**: Robot is unsupported
- **Ballistic motion**: Trajectory determined by initial conditions
- **Landing preparation**: Positioning for stable landing

## Mathematical Models for Locomotion

### Inverted Pendulum Model

The simplest model treats the robot as a point mass on a massless leg:
```
ẍ = g * θ
```

Where θ is the tilt angle from vertical.

### Linear Inverted Pendulum Model (LIPM)

Assumes constant CoM height:
```
ẍ_CoM = ω² * (x_CoM - x_ZMP)
```

Where ω² = g/h and h is the CoM height.

### Spring-Loaded Inverted Pendulum (SLIP)

Models the leg as a spring:
```
m*ẍ = -k*(l - l₀) * cos(θ) - mg
m*ÿ = -k*(l - l₀) * sin(θ)
```

Where k is the leg stiffness and l₀ is the rest length.

## Walking Pattern Generation

### ZMP-Based Walking

Zero Moment Point (ZMP) is critical for stable walking:
```
x_ZMP = x_CoM + (z_CoM - z_support) * ẍ_CoM / g
```

**ZMP trajectory planning:**
- Must remain within support polygon
- Smooth transitions between phases
- Anticipatory control for stability

### Preview Control

Uses future reference trajectories to improve tracking:
```
u(k) = Kx(k) + Σ L(i) * r(k+i)
```

Where r is the reference trajectory and L are preview gains.

### Footstep Planning

Determines where and when to place feet:
- **Stability**: Ensure future ZMP remains stable
- **Efficiency**: Minimize energy consumption
- **Obstacle avoidance**: Navigate around obstacles
- **Terrain adaptation**: Adjust for uneven surfaces

## Balance Control Strategies

### Ankle Strategy

Small perturbations are corrected by ankle torques:
- **Range**: Small disturbances
- **Speed**: Fast response
- **Energy**: Low consumption

### Hip Strategy

Larger perturbations are corrected by hip movements:
- **Range**: Moderate disturbances
- **Speed**: Moderate response
- **Energy**: Higher consumption

### Stepping Strategy

Large perturbations require stepping:
- **Range**: Large disturbances
- **Speed**: Slower recovery
- **Energy**: Highest consumption

## Control Architectures

### Hierarchical Control

Organizes locomotion control at multiple levels:
1. **High level**: Step planning and gait selection
2. **Mid level**: Balance and trajectory generation
3. **Low level**: Joint control and actuator commands

### Central Pattern Generators (CPGs)

Neural network models that generate rhythmic patterns:
- **Biological inspiration**: Mimics neural circuits in animals
- **Adaptability**: Can adapt to different speeds and terrains
- **Robustness**: Inherent stability properties

### Model-Free Approaches

Learning-based methods that don't rely on explicit models:
- **Reinforcement learning**: Learn optimal policies through trial and error
- **Imitation learning**: Learn from human demonstrations
- **Adaptive control**: Adjust parameters based on experience

## Terrain Adaptation

### Flat Ground Walking

Standard walking on level surfaces:
- **Predictable**: Consistent contact conditions
- **Optimized**: Can use periodic gaits
- **Efficient**: Minimal adaptation required

### Uneven Terrain

Walking on surfaces with obstacles or variations:
- **Adaptation**: Modify step placement and height
- **Perception**: Detect terrain features
- **Planning**: Adjust gait parameters

### Stair Climbing

Ascending and descending stairs:
- **Step height**: Account for vertical displacement
- **Stability**: Maintain balance during transitions
- **Foot placement**: Precise positioning required

### Rough Terrain

Natural environments with irregular surfaces:
- **Compliance**: Leg compliance for ground adaptation
- **Perception**: Real-time terrain assessment
- **Gait adaptation**: Modify walking pattern as needed

## Performance Metrics

### Stability Metrics

Quantify the stability of walking:
- **ZMP deviation**: Distance from reference ZMP
- **CoM stability**: Center of mass position relative to support
- **Capture point**: Ability to stop motion by stepping

### Efficiency Metrics

Measure energy consumption:
- **Cost of transport**: Energy per unit weight per unit distance
- **Specific resistance**: Power consumption relative to speed
- **Walking speed**: Achieved velocity

### Robustness Metrics

Assess response to disturbances:
- **Recovery time**: Time to return to stable walking
- **Disturbance tolerance**: Maximum disturbance handled
- **Failure rate**: Frequency of falls or stops

## Challenges and Solutions

### Computational Complexity

Real-time control requires efficient algorithms:
- **Model simplification**: Use approximate models when possible
- **Parallel computation**: Distribute calculations across processors
- **Pre-computation**: Calculate trajectories offline when possible

### Sensor Noise and Delays

Real sensors have limitations:
- **Filtering**: Use appropriate filters to reduce noise
- **Prediction**: Compensate for sensor delays
- **Redundancy**: Use multiple sensors for critical measurements

### Model Uncertainties

Real robots differ from models:
- **Adaptive control**: Adjust parameters online
- **Robust control**: Design for worst-case scenarios
- **Learning**: Update models based on experience

## Recent Advances

### Learning-Based Approaches

Machine learning is revolutionizing locomotion:
- **Deep reinforcement learning**: Learn complex gaits end-to-end
- **Imitation learning**: Transfer human walking patterns
- **Transfer learning**: Apply learned skills to new robots

### Hybrid Approaches

Combining model-based and learning methods:
- **Model-based initialization**: Start with stable controllers
- **Learning refinement**: Improve performance through experience
- **Safety constraints**: Maintain stability during learning

## Summary

Humanoid locomotion is a complex, multi-faceted challenge that requires sophisticated control strategies, mathematical modeling, and real-time computation. Success requires balancing stability, efficiency, and adaptability while handling the inherent uncertainties of real-world environments. Modern approaches increasingly combine classical control theory with machine learning to achieve more robust and capable walking behaviors.