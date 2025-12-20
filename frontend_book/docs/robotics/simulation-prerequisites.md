# Simulation Tools Prerequisites

This guide outlines the prerequisites and setup requirements for working with digital twin simulation tools including Gazebo, Unity, and ROS 2.

## System Requirements

### Minimum Hardware Specifications
- Operating System: Ubuntu 22.04 LTS or Windows 10/11
- RAM: 8GB minimum (16GB recommended)
- CPU: 4-core minimum (8-core recommended)
- GPU: Dedicated graphics card with OpenGL 4.3+ support
- Storage: 20GB free space for all tools

### Recommended Specifications for Optimal Performance
- RAM: 16GB or more
- CPU: 8-core or more
- GPU: NVIDIA GTX 1060 or equivalent/AMD Radeon RX 580 or equivalent with 6GB+ VRAM
- Storage: SSD with 30GB+ free space

## Software Installation

### ROS 2 Humble Hawksbill (LTS)
ROS 2 Humble Hawksbill is the Long Term Support version with extended support until 2027, making it ideal for educational use.

**Installation Steps:**
1. Update your Ubuntu packages:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   ```

2. Add the ROS 2 repository:
   ```bash
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/rosSigning.key | sudo apt-key add -
   echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
   ```

3. Install ROS 2 Humble:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-colcon-common-extensions
   ```

4. Source ROS 2 in your shell:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

### Gazebo Garden
Gazebo Garden (Garden Citadel) is the latest stable version with full ROS 2 Humble support.

**Installation Steps:**
```bash
sudo apt install gazebo
# Or install from source for latest features
```

### Unity 2022.3 LTS
Unity 2022.3 LTS (Long Term Support) is recommended for educational use with the Personal Edition.

**Installation Steps:**
1. Go to https://unity.com/download
2. Download Unity Hub
3. Install Unity 2022.3 LTS through Unity Hub
4. Install the Robotics package if available

## Verification Steps

### Verify ROS 2 Installation
```bash
# Check ROS 2 version
ros2 --version

# Verify core packages
ros2 pkg list | grep std_msgs
```

### Verify Gazebo Installation
```bash
# Launch Gazebo to test
gazebo --version
gazebo
```

### Verify Unity Installation
1. Launch Unity Hub
2. Create a new 3D project to verify functionality
3. Check that the interface loads correctly

## ROS 2-Gazebo Integration

Install the ROS 2 Gazebo packages:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Troubleshooting Common Issues

### ROS 2 Environment Not Sourced
If ROS 2 commands are not recognized, ensure you've sourced the setup file:
```bash
source /opt/ros/humble/setup.bash
```

You can also add this to your `~/.bashrc` file to make it permanent:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Gazebo Not Launching
If Gazebo fails to launch, check:
1. GPU drivers are properly installed
2. Try running with software rendering: `gazebo --verbose --render-engine=ogre`

### Performance Issues
- Close unnecessary applications to free up system resources
- Check if real-time factor is below 1.0 in Gazebo
- Use simple geometries (boxes, spheres) for initial testing

## Next Steps

Once you've completed the prerequisites setup, you can proceed to [Module 2: The Digital Twin (Gazebo & Unity)](./module2/index.md) to begin learning about simulation fundamentals.