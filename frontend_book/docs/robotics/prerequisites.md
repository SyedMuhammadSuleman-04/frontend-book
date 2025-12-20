---
sidebar_label: 'Prerequisites and Setup Guide'
title: 'Prerequisites and Setup Guide'
---

# Prerequisites and Setup Guide

This guide provides comprehensive instructions for setting up your development environment to work with ROS 2, Python AI integration, and humanoid robot modeling.

## System Requirements

### Hardware Requirements
- **Processor**: Multi-core processor (Intel i5 or equivalent recommended)
- **Memory**: 8GB RAM minimum, 16GB recommended
- **Storage**: 20GB free disk space
- **Graphics**: GPU with OpenGL 3.3+ support (for visualization)

### Operating System Support
- Ubuntu 22.04 LTS (Recommended for ROS 2 Humble Hawksbill)
- Windows 10/11 (with WSL2 for optimal ROS 2 support)
- macOS (with Docker for ROS 2 development)

## Software Prerequisites

### ROS 2 Installation
1. **Install ROS 2 Humble Hawksbill** (LTS version):
   ```bash
   # For Ubuntu
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update
   sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

2. **Set up ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Python Environment
1. **Install Python 3.8 or higher**:
   ```bash
   sudo apt install python3 python3-pip python3-colcon-common-extensions
   ```

2. **Set up Python virtual environment** (recommended):
   ```bash
   python3 -m venv ros2_env
   source ros2_env/bin/activate
   pip install --upgrade pip
   ```

### Development Tools
- **Git**: Version control system
- **VS Code**: Recommended IDE with ROS extension
- **CMake**: Build system
- **Build tools**: `build-essential`, `python3-dev`

## ROS 2 Packages and Dependencies

### Core Packages
```bash
# Essential ROS 2 packages
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs ros-humble-nav-msgs
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher ros-humble-gazebo-ros-pkgs
```

### Python Dependencies
```bash
pip install numpy scipy matplotlib
pip install opencv-python  # For computer vision tasks
pip install tensorflow pytorch  # For AI/ML (optional)
```

## Simulation Environment Setup

### Gazebo Installation
```bash
sudo apt install ros-humble-gazebo-classic-ros-pkgs ros-humble-gazebo-plugins
# Or for Gazebo Garden (newer version)
sudo apt install ros-humble-gazebo-dev
```

### RViz Configuration
RViz comes with the desktop installation. You can launch it with:
```bash
rviz2
```

## Development Environment Configuration

### Workspace Setup
1. **Create a ROS 2 workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

2. **Environment variables**:
   Add to your `~/.bashrc`:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   export ROS_DOMAIN_ID=0  # Set your domain ID
   ```

### Testing the Installation
1. **Verify ROS 2 installation**:
   ```bash
   ros2 topic list
   ros2 node list
   ```

2. **Test Python integration**:
   ```python
   import rclpy
   rclpy.init()
   node = rclpy.create_node('test_node')
   print("ROS 2 Python integration working!")
   node.destroy_node()
   rclpy.shutdown()
   ```

## Validation Checklist

Before proceeding with the tutorials, verify:

- [ ] ROS 2 Humble Hawksbill is installed and sourced
- [ ] `ros2` command is available in terminal
- [ ] Python can import rclpy
- [ ] Basic ROS 2 commands work (e.g., `ros2 topic list`)
- [ ] You can access the documentation website locally

## Next Steps

Once your environment is properly set up:

1. Verify all components work together
2. Run the basic ROS 2 tutorials to confirm installation
3. Proceed to Chapter 1: ROS 2 Core Concepts
4. Follow along with the examples in this documentation

Remember to restart your terminal or source your environment after making changes to `~/.bashrc`.