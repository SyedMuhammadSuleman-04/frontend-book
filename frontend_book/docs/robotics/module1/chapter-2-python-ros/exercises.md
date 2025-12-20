---
sidebar_label: 'Chapter 2 Exercises'
title: 'Chapter 2 Exercises'
---

# Chapter 2 Exercises

This section provides hands-on exercises to practice integrating Python AI agents with ROS 2 systems.

## Exercise 1: Simple AI Controller

### Objective
Create a simple AI controller that navigates a robot in a basic environment using sensor data.

### Scenario
Create a robot controller that:
- Moves forward until it detects an obstacle
- Turns to avoid the obstacle
- Continues navigation after clearing the obstacle

### Steps
1. Create a ROS 2 node that subscribes to laser scan data
2. Implement basic obstacle detection logic
3. Create twist commands to control the robot's movement
4. Test the controller in simulation

### Requirements
- Use a laser scan topic (e.g., `/scan`) for obstacle detection
- Publish velocity commands to `/cmd_vel`
- Implement safety checks to prevent collisions
- Add logging to track the robot's behavior

### Solution Hints
- Use minimum distance from laser scan to detect obstacles
- Implement a simple state machine for navigation states
- Consider hysteresis to prevent oscillation near obstacle boundaries

## Exercise 2: Machine Learning Classification for Object Detection

### Objective
Use a scikit-learn classifier to identify different types of objects from sensor data and control the robot accordingly.

### Scenario
Train a classifier to distinguish between different objects (e.g., walls, furniture, other robots) and make navigation decisions based on object classification.

### Steps
1. Collect sample sensor data representing different object types
2. Train a classifier (e.g., SVM, Random Forest) to identify objects
3. Integrate the classifier into a ROS 2 node
4. Make navigation decisions based on object classification

### Requirements
- Use a machine learning library (scikit-learn recommended)
- Process sensor data in real-time
- Adapt robot behavior based on object classification
- Include model persistence (save/load trained model)

### Solution Hints
- Use clustering or feature extraction from sensor data
- Consider point cloud data or processed laser scan features
- Train classifier on labeled sensor data samples

## Exercise 3: Deep Learning for Visual Navigation

### Objective
Use a deep learning model to process camera images and control robot navigation.

### Scenario
Create a robot controller that uses a pre-trained image classification or segmentation model to identify navigable paths and obstacles.

### Steps
1. Create a node that subscribes to camera image data
2. Process images through a deep learning model
3. Convert model outputs to navigation commands
4. Implement safety measures to prevent misclassification issues

### Requirements
- Subscribe to image topic (e.g., `/camera/image_raw`)
- Use a deep learning framework (TensorFlow/PyTorch)
- Include image preprocessing for model input
- Implement fallback behavior for poor lighting/visibility

### Solution Hints
- Use CV bridge to convert ROS images to OpenCV format
- Consider using pre-trained models for feature extraction
- Implement rate limiting for computational efficiency

## Exercise 4: Reinforcement Learning Navigation

### Objective
Implement a basic reinforcement learning algorithm to train a robot to navigate to goals while avoiding obstacles.

### Scenario
Train an RL agent to navigate in a simulated environment with rewards for reaching goals and penalties for collisions.

### Steps
1. Implement a Q-learning algorithm with discretized state space
2. Create reward function based on distance to goal and obstacle proximity
3. Integrate with ROS 2 simulation environment
4. Train the agent to navigate effectively

### Requirements
- Implement state discretization from sensor data
- Design appropriate reward function
- Include exploration-exploitation balance
- Track learning progress and convergence

### Solution Hints
- Discretize continuous sensor space into manageable state space
- Use epsilon-greedy for exploration
- Consider using neural networks for large state spaces (Deep Q-Network)

## Exercise 5: Multi-Sensor Fusion AI Agent

### Objective
Create an AI agent that combines multiple sensor inputs (e.g., laser, camera, IMU) for enhanced decision making.

### Scenario
Build an agent that uses laser data for close-range obstacle detection, camera for distant object recognition, and IMU for localization to make navigation decisions.

### Steps
1. Subscribe to multiple sensor topics
2. Implement sensor fusion techniques
3. Create a unified perception system
4. Design behavior based on fused sensor data

### Requirements
- Synchronize data from multiple sensors
- Implement appropriate fusion algorithm
- Handle sensor failure gracefully
- Compare performance with single-sensor approaches

### Solution Hints
- Use ROS 2 message filters for sensor synchronization
- Consider weighted fusion or voting systems
- Implement sensor validation and outlier detection

## Exercise 6: Adaptive Behavior Learning

### Objective
Create an AI agent that learns to adapt its behavior based on environmental feedback.

### Scenario
Design an agent that modifies its navigation strategy based on terrain difficulty, success rates, or other environmental factors.

### Steps
1. Implement behavior selection mechanism
2. Track performance metrics for different behaviors
3. Switch behaviors based on learned effectiveness
4. Test adaptation in varying environments

### Requirements
- Implement multiple behavior modes
- Track success/failure metrics
- Learn optimal behavior selection
- Handle concept drift over time

### Solution Hints
- Use contextual bandits or multi-armed bandit algorithms
- Consider meta-learning approaches
- Implement forgetting mechanisms for changing environments

## Exercise 7: Human-Robot Interaction AI

### Objective
Create an AI system that responds appropriately to human presence and gestures.

### Scenario
Implement an AI agent that detects humans and responds appropriately (e.g., stops, moves around, acknowledges).

### Steps
1. Implement human detection using vision or other sensors
2. Create appropriate responses to human presence
3. Ensure safety in human-robot interactions
4. Test with simulated human interactions

### Requirements
- Detect humans in sensor data
- Implement appropriate interaction behaviors
- Prioritize safety in all interactions
- Include social navigation principles

### Solution Hints
- Use human detection models or simple proximity detection
- Implement right-of-way behaviors
- Consider social norms and expectations

## Summary

These exercises cover essential aspects of AI-robot integration:
- Basic sensor-based control
- Machine learning integration
- Deep learning for perception
- Reinforcement learning for adaptation
- Multi-sensor fusion
- Adaptive behavior learning
- Human-robot interaction

Completing these exercises will provide practical experience in connecting AI algorithms with robotic systems through ROS 2.