# Prerequisites Guide: Isaac AI Brain Module

This guide outlines all the prerequisites needed to successfully complete the Isaac AI Brain module (Module 3), including software installations, hardware requirements, and foundational knowledge.

## Hardware Prerequisites

### Minimum Hardware Requirements
- **GPU**: NVIDIA RTX series graphics card (RTX 2060 or equivalent with 6GB+ VRAM)
- **CPU**: Multi-core processor (Intel i5 or AMD Ryzen 5 or better)
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 100GB+ free disk space
- **OS**: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS (recommended for Isaac Sim compatibility)

### Recommended Hardware Requirements
- **GPU**: NVIDIA RTX 3070 or higher with 8GB+ VRAM (for optimal performance)
- **CPU**: Intel i7 or AMD Ryzen 7 or better
- **RAM**: 32GB or more
- **Storage**: SSD with 200GB+ free space for faster loading

## Software Prerequisites

### Operating System
- **Ubuntu 20.04 LTS** or **Ubuntu 22.04 LTS** (recommended)
- Ensure system is fully updated:
  ```bash
  sudo apt update && sudo apt upgrade
  ```

### NVIDIA Software Stack
1. **NVIDIA GPU Drivers** (version 495 or higher):
   ```bash
   # Check current driver
   nvidia-smi

   # Install latest drivers if needed
   sudo apt install nvidia-driver-535  # or latest version
   ```

2. **CUDA Toolkit** (version 11.8 or higher):
   ```bash
   # Download from NVIDIA Developer website
   # Follow CUDA installation guide for your OS
   nvcc --version
   ```

3. **cuDNN** (for deep learning acceleration):
   ```bash
   # Install from NVIDIA Developer website
   # Verify installation
   cat /usr/local/cuda/include/cudnn_version.h | grep CUDNN_MAJOR -A 2
   ```

### ROS 2 Installation
1. **ROS 2 Humble Hawksbill** (full desktop installation):
   ```bash
   # Add ROS 2 GPG key and repository
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install ROS 2 Humble
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

2. **Initialize rosdep**:
   ```bash
   sudo rosdep init
   rosdep update
   ```

3. **Set up ROS 2 environment**:
   ```bash
   # Add to ~/.bashrc
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Isaac Sim Installation
1. **Download Isaac Sim** from NVIDIA Developer website
2. **Install prerequisites**:
   ```bash
   # Install required packages
   sudo apt install -y libssl-dev libffi-dev libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavdevice-dev
   ```

3. **Verify Isaac Sim installation**:
   ```bash
   # Launch Isaac Sim to verify installation
   cd /path/to/isaac-sim
   ./isaac-sim.sh
   ```

### Isaac ROS Packages
```bash
# Add NVIDIA package repository
sudo apt update
sudo apt install -y software-properties-common
wget https://repo.download.nvidia.com/client/ubuntu/$(lsb_release -cs)/nvidia-ml.list
sudo cp nvidia-ml.list /etc/apt/sources.list.d/nvidia-ml.list
sudo apt update

# Install Isaac ROS common packages
sudo apt install -y ros-humble-isaac-ros-common
sudo apt install -y ros-humble-isaac-ros-dev

# Install specific perception packages
sudo apt install -y ros-humble-isaac-ros-visual-slam
sudo apt install -y ros-humble-isaac-ros-detection2d
sudo apt install -y ros-humble-isaac-ros-segmentation
sudo apt install -y ros-humble-isaac-ros-image-ros
```

### Navigation Stack (Nav2)
```bash
# Install Nav2 packages
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-nav2-gui-plugins
sudo apt install -y ros-humble-nav2-lifecycle-manager
sudo apt install -y ros-humble-dwb-core ros-humble-dwb-msgs
sudo apt install -y ros-humble-dwb-plugins ros-humble-nav-2d-utils
sudo apt install -y ros-humble-nav-cpp-tools ros-humble-nav-rotation-recovery
sudo apt install -y ros-humble-nav-smoother-lifecycle
```

## Knowledge Prerequisites

### Programming Skills
1. **Python Programming** (intermediate level):
   - Object-oriented programming concepts
   - File I/O operations
   - Error handling
   - Working with libraries and modules
   - Basic understanding of data structures

2. **ROS 2 Concepts**:
   - Nodes, topics, services, actions
   - Launch files and parameters
   - TF (Transform) system
   - Message types and interfaces
   - Lifecycle nodes

3. **Basic C++** (helpful but not required):
   - Understanding of basic syntax
   - Object-oriented concepts

### Robotics Concepts
1. **Basic Robot Control**:
   - Understanding of robot kinematics
   - Basic concepts of robot navigation
   - Understanding of sensors and their applications

2. **Mathematical Foundations**:
   - Linear algebra (vectors, matrices, transformations)
   - Basic calculus
   - Probability and statistics
   - Geometry concepts

### Simulation Concepts
1. **Basic Simulation Understanding**:
   - Physics simulation concepts
   - Sensor simulation
   - Environment modeling

2. **USD (Universal Scene Description)**:
   - Basic understanding of 3D scene representation
   - File formats and structure (helpful but not required)

## Module-Specific Prerequisites

### From Module 1 (ROS 2 Core Concepts)
- Understanding of ROS 2 nodes, topics, and services
- Experience with rclpy for Python-based ROS 2 development
- Knowledge of coordinate frames and TF
- Experience with basic ROS 2 tools (ros2 run, ros2 launch, etc.)

### From Module 2 (Simulation Fundamentals)
- Basic understanding of Gazebo or similar simulators
- Experience with robot models (URDF)
- Understanding of sensor integration in simulation
- Basic understanding of physics simulation concepts

## Verification of Prerequisites

### System Verification Script
Create a verification script to check all prerequisites:

```python
#!/usr/bin/env python3
# verify_prerequisites.py
import subprocess
import sys
import os

def check_command(cmd, description):
    """Check if a command is available and working"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✓ {description}: Available")
            return True
        else:
            print(f"✗ {description}: Not available")
            print(f"  Error: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print(f"✗ {description}: Command timed out")
        return False
    except Exception as e:
        print(f"✗ {description}: Error - {str(e)}")
        return False

def check_python_packages():
    """Check if required Python packages are available"""
    required_packages = [
        'rclpy',
        'numpy',
        'scipy',
        'cv2',
        'yaml'
    ]

    missing_packages = []
    for package in required_packages:
        try:
            if package == 'cv2':
                import cv2
            else:
                __import__(package)
            print(f"✓ Python package '{package}': Available")
        except ImportError:
            print(f"✗ Python package '{package}': Not available")
            missing_packages.append(package)

    return len(missing_packages) == 0

def main():
    print("Isaac AI Brain Module Prerequisites Verification")
    print("=" * 50)

    all_checks = []

    # Check system commands
    all_checks.append(check_command("nvidia-smi", "NVIDIA GPU Drivers"))
    all_checks.append(check_command("nvcc --version", "CUDA Toolkit"))
    all_checks.append(check_command("ros2 --version", "ROS 2 Humble"))
    all_checks.append(check_command("python3 --version", "Python 3"))

    # Check Isaac ROS packages
    all_checks.append(check_command("ros2 pkg list | grep isaac", "Isaac ROS Packages"))

    # Check Nav2 packages
    all_checks.append(check_command("ros2 pkg list | grep nav2", "Navigation2 (Nav2)"))

    # Check Python packages
    python_check = check_python_packages()
    all_checks.append(python_check)

    print("\n" + "=" * 50)
    if all(all_checks):
        print("🎉 All prerequisites are satisfied!")
        print("You are ready to start the Isaac AI Brain module.")
    else:
        print("❌ Some prerequisites are missing or not working properly.")
        print("Please review the above output and address any issues before proceeding.")

    return 0 if all(all_checks) else 1

if __name__ == "__main__":
    sys.exit(main())
```

### Manual Verification Steps

1. **GPU Verification**:
   ```bash
   nvidia-smi
   # Should show your GPU and driver version
   ```

2. **CUDA Verification**:
   ```bash
   nvcc --version
   # Should show CUDA version
   ```

3. **ROS 2 Verification**:
   ```bash
   ros2 --version
   # Should show ROS 2 version
   ```

4. **Isaac ROS Verification**:
   ```bash
   ros2 pkg list | grep isaac
   # Should show Isaac ROS packages
   ```

5. **Python Packages Verification**:
   ```bash
   python3 -c "import rclpy; print('rclpy OK')"
   python3 -c "import cv2; print('cv2 OK')"
   ```

## Optional Prerequisites (Recommended)

### Development Tools
- **VS Code** with ROS extension
- **Git** for version control
- **Docker** (for containerized development)
- **Gazebo** (for comparison with Isaac Sim)

### Additional Python Libraries
```bash
pip3 install --user numpy scipy matplotlib opencv-python
```

## Troubleshooting Common Issues

### GPU Driver Issues
- **Issue**: CUDA not working properly
- **Solution**: Ensure GPU drivers are up to date and match CUDA version

### ROS 2 Installation Issues
- **Issue**: ROS 2 commands not found
- **Solution**: Verify ROS 2 installation and check environment setup

### Isaac Sim Installation Issues
- **Issue**: Isaac Sim fails to launch
- **Solution**: Check GPU driver compatibility and system requirements

## Next Steps

Once all prerequisites are verified:

1. **Complete the verification script** to ensure all requirements are met
2. **Set up your development workspace**:
   ```bash
   mkdir -p ~/isaac_ros_ws/src
   cd ~/isaac_ros_ws
   colcon build
   source install/setup.bash
   ```
3. **Test basic functionality** before proceeding to Chapter 1
4. **Review the Module 3 overview** to understand the learning objectives

## Resources

- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Navigation2 Documentation](https://navigation.ros.org/)

This prerequisites guide ensures that students have all necessary components installed and configured correctly before beginning the Isaac AI Brain module, preventing common setup issues that could impede learning.