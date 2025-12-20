# Claude Agent Context: Digital Twin Simulation for Humanoid Robots

**Feature**: 002-digital-twin-sim
**Created**: 2025-12-20
**Last Updated**: 2025-12-20

## Technology Stack

### Primary Technologies
- **Gazebo Garden**: Physics-based simulation engine for robotics
  - Used for: Physics simulation, gravity, collision detection
  - Version: Garden Citadel
  - Integration: With ROS 2 Humble Hawksbill

- **Unity 2022.3 LTS**: 3D environment creation and visualization
  - Used for: Virtual environment building and interaction
  - Version: 2022.3 LTS (Personal Edition for education)
  - Integration: With ROS 2 via Unity Robotics Hub

- **ROS 2 Humble Hawksbill**: Robotics middleware framework
  - Used for: Communication between simulation components
  - Version: Humble Hawksbill (LTS until 2027)
  - Integration: With Gazebo and Unity through ROS bridges

### Sensor Simulation Technologies
- **LiDAR Simulation**: sensor_msgs/LaserScan format
- **Camera Simulation**: sensor_msgs/Image format
- **IMU Simulation**: sensor_msgs/Imu format

## Key Concepts

### Physics Simulation
- Gravity parameters and their effects
- Collision detection and response
- Friction coefficients and material properties
- Realistic force and momentum transfer

### Environment Modeling
- 3D scene creation in Unity
- Terrain and object placement
- Lighting and material properties
- Interactive element design

### Sensor Data Simulation
- Realistic noise patterns in sensor data
- Proper data formatting for ROS 2 topics
- Integration with perception pipelines
- Validation against real-world sensor behavior

## Architecture Patterns

### Simulation Architecture
- Gazebo handles physics simulation and robot dynamics
- Unity handles environment visualization and advanced graphics
- ROS 2 middleware coordinates communication between components
- Sensor data flows through standard ROS 2 topics

### Educational Content Structure
- Module 2: Digital Twin Simulation
- Chapter 1: Simulation Fundamentals (Gazebo)
- Chapter 2: Virtual Environments (Unity)
- Chapter 3: Sensor Simulation (LiDAR, cameras, IMUs)

## Implementation Notes

### Performance Considerations
- Minimum 8GB RAM, 4-core CPU, dedicated GPU recommended
- Simple geometries for initial testing
- Physics update rates based on simulation requirements
- Monitoring CPU and GPU usage during simulation

### Integration Points
- ROS 2 bridge for Unity-Gazebo communication
- Standard message types for sensor data
- Coordinate frame management
- Time synchronization between systems

## Troubleshooting Guide

### Common Issues
- Gazebo startup problems (check GPU drivers)
- ROS 2 communication issues (ensure proper sourcing)
- Performance problems (adjust visual quality settings)

### Validation Steps
- Physics parameters configuration accuracy
- Sensor data format compliance
- Environment interaction responsiveness
- ROS 2 topic communication verification