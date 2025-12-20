# Quickstart Guide: Digital Twin Simulation for Humanoid Robots

**Feature**: 002-digital-twin-sim
**Created**: 2025-12-20
**Status**: Draft

## Overview

This quickstart guide provides a step-by-step introduction to setting up and using the Digital Twin Simulation environment for humanoid robots. The guide covers the three main components: Gazebo physics simulation, Unity virtual environments, and sensor simulation.

## Prerequisites

Before starting, ensure you have:

### System Requirements
- Operating System: Ubuntu 22.04 LTS or Windows 10/11
- RAM: 8GB minimum (16GB recommended)
- CPU: 4-core minimum (8-core recommended)
- GPU: Dedicated graphics card with OpenGL 4.3+ support
- Storage: 20GB free space

### Software Requirements
- ROS 2 Humble Hawksbill (LTS)
- Gazebo Garden
- Unity 2022.3 LTS (Personal Edition)
- Python 3.8 or higher
- Git

## Setup Process

### 1. Install ROS 2 Humble Hawksbill

**On Ubuntu:**
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/rosSigning.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-gazebo-ros-pkgs

# Source ROS 2
source /opt/ros/humble/setup.bash
```

**On Windows:**
Follow the official ROS 2 Windows installation guide: https://docs.ros.org/en/humble/Installation/Windows-Install.html

### 2. Install Gazebo Garden

**On Ubuntu:**
```bash
sudo apt install gazebo
# Or install from source for latest features
```

**On Windows:**
Download from: https://gazebosim.org/docs/garden/install/

### 3. Install Unity 2022.3 LTS

1. Go to https://unity.com/download
2. Download Unity Hub
3. Install Unity 2022.3 LTS through Unity Hub
4. Install the Robotics package if available

### 4. Setup Unity-ROS 2 Bridge

```bash
# Clone the Unity Robotics Hub
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
cd Unity-Robotics-Hub
git checkout main
```

## Chapter 1: Gazebo Simulation Fundamentals

### 1.1 Launch Gazebo with Physics Parameters

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch Gazebo with default physics
ros2 launch gazebo_ros gazebo.launch.py
```

### 1.2 Create a Simple Robot Model

Create a basic URDF file (`simple_robot.urdf`):

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.083"/>
    </inertial>
  </link>
</robot>
```

### 1.3 Spawn Robot in Gazebo

```bash
# Spawn the robot
ros2 run gazebo_ros spawn_entity.py -entity simple_robot -file simple_robot.urdf -z 1.0
```

### 1.4 Test Physics Simulation

Observe how the robot falls due to gravity and interacts with the ground plane.

## Chapter 2: Virtual Environment Creation in Unity

### 2.1 Create Basic Unity Scene

1. Open Unity Hub and create a new 3D project
2. Create a basic environment with terrain, objects, and lighting
3. Set up the camera to view the environment

### 2.2 Configure Environment Parameters

```csharp
// Example Unity C# script for environment configuration
using UnityEngine;

public class EnvironmentController : MonoBehaviour
{
    public float gravity = -9.81f;
    public float friction = 0.5f;

    void Start()
    {
        Physics.gravity = new Vector3(0, gravity, 0);
    }
}
```

### 2.3 Add Interactive Elements

Create objects that can interact with simulated robots, such as:
- Movable boxes
- Ramps
- Obstacles
- Targets

## Chapter 3: Sensor Simulation Implementation

### 3.1 Configure LiDAR Sensor

```bash
# Launch LiDAR simulation node
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node
```

### 3.2 Configure Camera Sensor

```bash
# Launch camera simulation
ros2 launch gazebo_ros camera.launch.py
```

### 3.3 Configure IMU Sensor

```bash
# Launch IMU simulation
ros2 launch gazebo_ros imu.launch.py
```

### 3.4 View Sensor Data

```bash
# Monitor LiDAR data
ros2 topic echo /robot/sensors/lidar sensor_msgs/msg/LaserScan

# Monitor camera data
ros2 topic echo /robot/sensors/camera/image_raw sensor_msgs/msg/Image

# Monitor IMU data
ros2 topic echo /robot/sensors/imu sensor_msgs/msg/Imu
```

## Integration Example

### Connecting Gazebo and ROS 2

```bash
# Terminal 1: Launch Gazebo
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Spawn robot
source /opt/ros/humble/setup.bash
ros2 run gazebo_ros spawn_entity.py -entity simple_robot -file simple_robot.urdf

# Terminal 3: View robot state
source /opt/ros/humble/setup.bash
ros2 topic echo /robot/state gazebo_msgs/msg/ModelStates
```

## Troubleshooting

### Common Issues

1. **Gazebo won't start**
   - Check if NVIDIA drivers are properly installed (for GPU acceleration)
   - Try running with software rendering: `gazebo --verbose --render-engine=ogre`

2. **ROS 2 nodes can't communicate**
   - Ensure all terminals have sourced ROS 2: `source /opt/ros/humble/setup.bash`
   - Check if RMW implementation is consistent across terminals

3. **Simulation runs slowly**
   - Reduce visual quality settings in Gazebo
   - Close unnecessary applications to free up system resources
   - Check if real-time factor is below 1.0

### Performance Tips

- Use simple geometries (boxes, spheres) for initial testing
- Limit the number of objects in the simulation environment
- Adjust physics update rates based on simulation requirements
- Monitor CPU and GPU usage during simulation

## Next Steps

After completing this quickstart guide, proceed to:
1. Chapter 1: Detailed Gazebo simulation concepts
2. Chapter 2: Advanced Unity environment creation
3. Chapter 3: Complex sensor simulation and integration
4. Hands-on exercises combining all components