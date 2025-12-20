---
sidebar_label: 'Chapter 1 Exercises'
title: 'Chapter 1 Exercises'
---

# Chapter 1 Exercises

This section provides hands-on exercises to reinforce your understanding of ROS 2 core concepts: nodes, topics, services, and actions.

## Exercise 1: Simple Publisher-Subscriber

### Objective
Create a publisher node that publishes messages containing the current time, and a subscriber node that receives and displays these messages.

### Steps
1. Create a new ROS 2 package called `time_publisher`
2. Create a publisher node that publishes the current time every second
3. Create a subscriber node that receives and displays the time
4. Test that the nodes communicate properly

### Expected Outcome
- Publisher node running and publishing time messages
- Subscriber node receiving and displaying messages
- Proper separation of publisher and subscriber functionality

### Solution Hints
- Use `std_msgs.msg.String` for the message type
- Use `datetime.now()` to get the current time
- Implement proper node lifecycle management

## Exercise 2: Service-Based Calculator

### Objective
Create a service server that performs basic mathematical operations and a client that uses the service.

### Steps
1. Define a custom service file for a calculator (e.g., `Calculator.srv`)
2. Implement a service server that can add, subtract, multiply, and divide
3. Create a client that sends requests to the server
4. Test all operations

### Expected Outcome
- Custom service definition
- Working service server with all operations
- Client that can send requests and receive responses
- Error handling for invalid operations

### Solution Hints
- Use `int64` for numbers and a string for the operation
- Include error handling for division by zero
- Use appropriate service types

## Exercise 3: Action-Based Progress Tracker

### Objective
Create an action server that simulates a long-running task with progress feedback and a client that tracks the progress.

### Steps
1. Define a custom action for a progress-tracking task
2. Implement an action server that simulates a task with feedback
3. Create an action client that sends goals and tracks progress
4. Test cancellation functionality

### Expected Outcome
- Custom action definition
- Working action server with progress feedback
- Client that tracks progress and can cancel tasks
- Proper goal handling and result reporting

### Solution Hints
- Use the action pattern for long-running tasks
- Include feedback messages to show progress
- Implement proper cancellation handling

## Exercise 4: Multi-Node System

### Objective
Design and implement a simple robotic system using multiple nodes with different communication patterns.

### Scenario
Create a simple "robotic vacuum" system with:
- Sensor node: publishes room status (clean/dirty)
- Navigation node: receives room status and sends movement commands
- Cleaner node: receives movement commands and reports cleaning status

### Steps
1. Design the communication patterns between nodes
2. Create three separate nodes for each component
3. Implement appropriate message types
4. Test the system integration

### Expected Outcome
- Three interconnected nodes
- Proper use of topics for continuous communication
- System that demonstrates ROS 2 concepts in a realistic scenario

### Solution Hints
- Use topics for continuous sensor data and movement commands
- Consider how nodes should be structured for modularity
- Implement proper error handling

## Exercise 5: Quality of Service (QoS) Exploration

### Objective
Explore how Quality of Service settings affect communication between nodes.

### Steps
1. Create a publisher with different QoS profiles (reliability, durability)
2. Create subscribers with matching and mismatched QoS profiles
3. Observe how different QoS settings affect message delivery
4. Document findings about QoS behavior

### Expected Outcome
- Understanding of QoS profiles and their impact
- Knowledge of how to configure QoS for different requirements
- Experience with matching QoS profiles between publishers and subscribers

### Solution Hints
- Experiment with reliability (reliable vs best_effort)
- Try different durability settings (volatile vs transient_local)
- Use QoS profiles to match communication requirements

## Summary

These exercises reinforce the core concepts of ROS 2:
- **Topics**: For asynchronous, continuous communication
- **Services**: For synchronous request-response interactions
- **Actions**: For long-running tasks with feedback

Completing these exercises will give you hands-on experience with ROS 2 communication patterns and prepare you for more advanced robotics applications.