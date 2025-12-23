# Isaac ROS Installation and Integration Guide

This guide provides step-by-step instructions for installing and configuring Isaac ROS packages and integrating them with ROS 2 Humble Hawksbill.

## System Requirements

### Hardware Requirements
- **GPU**: NVIDIA GPU with CUDA support (RTX series recommended)
- **Memory**: 16GB RAM minimum, 32GB recommended
- **Storage**: 20GB free space for Isaac ROS packages
- **OS**: Ubuntu 20.04 or 22.04 LTS

### Software Requirements
- **ROS 2**: Humble Hawksbill distribution
- **CUDA**: Version 11.8 or higher
- **NVIDIA Drivers**: Version 495 or higher
- **Isaac Sim**: For simulation and testing

## Prerequisites

### ROS 2 Humble Installation
Before installing Isaac ROS, ensure ROS 2 Humble is properly installed:

1. **Set up ROS 2 environment**:
   ```bash
   # Add ROS 2 GPG key and repository
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install ROS 2 Humble
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

   # Initialize rosdep
   sudo rosdep init
   rosdep update
   ```

2. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

### CUDA Installation
Verify CUDA is properly installed:
```bash
nvcc --version
nvidia-smi
```

## Installing Isaac ROS Packages

### Adding NVIDIA Package Repositories
1. **Add NVIDIA ML repository**:
   ```bash
   sudo apt update
   sudo apt install -y software-properties-common
   wget https://repo.download.nvidia.com/client/ubuntu/$(lsb_release -cs)/nvidia-ml.list
   sudo cp nvidia-ml.list /etc/apt/sources.list.d/nvidia-ml.list
   sudo apt update
   ```

### Installing Core Isaac ROS Packages
1. **Install Isaac ROS common packages**:
   ```bash
   sudo apt install -y ros-humble-isaac-ros-common
   sudo apt install -y ros-humble-isaac-ros-dev
   ```

2. **Install specific perception packages**:
   ```bash
   # Visual SLAM packages
   sudo apt install -y ros-humble-isaac-ros-visual-slam
   sudo apt install -y ros-humble-isaac-ros-gps-vo
   sudo apt install -y ros-humble-isaac-ros-seesaw
   sudo apt install -y ros-humble-isaac-ros-rotator-forearm

   # Detection packages
   sudo apt install -y ros-humble-isaac-ros-detection2d
   sudo apt install -y ros-humble-isaac-ros-detection3d
   sudo apt install -y ros-humble-isaac-ros-segmentation

   # Image processing packages
   sudo apt install -y ros-humble-isaac-ros-image-ros
   sudo apt install -y ros-humble-isaac-ros-image-rectification
   sudo apt install -y ros-humble-isaac-ros-stereo-image-proc
   ```

3. **Install sensor bridge packages**:
   ```bash
   sudo apt install -y ros-humble-isaac-ros-sensors
   sudo apt install -y ros-humble-isaac-ros-micro-ros
   ```

### Installing Additional Dependencies
```bash
# Install additional dependencies
sudo apt install -y ros-humble-vision-opencv
sudo apt install -y ros-humble-cv-bridge
sudo apt install -y ros-humble-image-transport
sudo apt install -y ros-humble-compressed-image-transport
sudo apt install -y ros-humble-compressed-depth-image-transport
sudo apt install -y ros-humble-tensor-rt
```

## Verification and Testing

### Verify Package Installation
1. **Check installed packages**:
   ```bash
   apt list --installed | grep isaac-ros
   ```

2. **Verify ROS 2 can find Isaac ROS packages**:
   ```bash
   ros2 pkg list | grep isaac
   ```

### Basic Functionality Test
1. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Test Isaac ROS nodes**:
   ```bash
   # Check available Isaac ROS nodes
   ros2 run --list | grep isaac
   ```

3. **Run a simple Isaac ROS example**:
   ```bash
   # This will run a basic Isaac ROS node to test functionality
   ros2 run isaac_ros_visual_slam visual_slam_node --ros-args --log-level info
   ```

## Isaac ROS Integration with ROS 2

### Workspace Setup
1. **Create a workspace for Isaac ROS projects**:
   ```bash
   mkdir -p ~/isaac_ros_ws/src
   cd ~/isaac_ros_ws
   colcon build
   source install/setup.bash
   ```

2. **Set up environment variables**:
   ```bash
   # Add to ~/.bashrc for persistent setup
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/isaac_ros_ws/install/setup.bash" >> ~/.bashrc
   echo "export CUDA_VISIBLE_DEVICES=0" >> ~/.bashrc
   ```

### Package Configuration

#### Creating Isaac ROS Launch Files
Create launch files to easily start Isaac ROS nodes:

```python
# example_isaac_ros_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam_node',
        parameters=[{
            'enable_rectification': True,
            'use_sim_time': True  # Set to True if using Isaac Sim
        }],
        remappings=[
            ('/visual_slam_node/left/image', '/camera/rgb/left/image_raw'),
            ('/visual_slam_node/right/image', '/camera/rgb/right/image_raw'),
            ('/visual_slam_node/left/camera_info', '/camera/rgb/left/camera_info'),
            ('/visual_slam_node/right/camera_info', '/camera/rgb/right/camera_info'),
            ('/visual_slam_node/imu', '/imu/data')
        ]
    )

    return LaunchDescription([
        visual_slam_node
    ])
```

#### Parameter Configuration
Create parameter files for Isaac ROS nodes:

```yaml
# isaac_ros_params.yaml
visual_slam_node:
  ros__parameters:
    # Input parameters
    enable_rectification: True
    rectified_left_topic_name: "/camera/rgb/left/image_rect_color"
    rectified_right_topic_name: "/camera/rgb/right/image_rect_color"
    left_camera_info_topic_name: "/camera/rgb/left/camera_info"
    right_camera_info_topic_name: "/camera/rgb/right/camera_info"
    imu_topic_name: "/imu/data"

    # Algorithm parameters
    max_num_corners: 1000
    min_num_corners: 100
    tracking_rate: 30.0
    mapping_rate: 1.0

    # GPU parameters
    enable_imu_fusion: True
    use_sim_time: False
```

## Integration with Isaac Sim

### Sensor Bridge Setup
1. **Configure Isaac Sim to publish sensor data**:
   - Set up camera topics in Isaac Sim to match ROS expectations
   - Configure LiDAR and IMU publishing rates
   - Verify coordinate frame conventions match ROS standards

2. **Test Isaac Sim to Isaac ROS integration**:
   ```bash
   # Launch Isaac Sim with your robot and sensors
   # In another terminal, launch Isaac ROS perception nodes
   ros2 launch your_package isaac_ros_perception.launch.py
   ```

### Coordinate Frame Integration
Ensure proper coordinate frame conventions:
- **ROS Standard**: X forward, Y left, Z up
- **Isaac Sim**: Verify coordinate system matches
- **Transformations**: Set up proper TF trees

## Troubleshooting Common Issues

### Package Installation Issues
- **Issue**: Package not found during installation
  - **Solution**: Verify repository setup and run `sudo apt update`
- **Issue**: Dependency conflicts
  - **Solution**: Check ROS 2 Humble compatibility requirements
- **Issue**: CUDA version mismatch
  - **Solution**: Ensure CUDA 11.8+ is installed and properly configured

### Runtime Issues
- **Issue**: Isaac ROS nodes fail to start
  - **Solution**: Check GPU availability and CUDA setup
- **Issue**: Performance issues
  - **Solution**: Verify GPU acceleration is working properly
- **Issue**: Sensor data not received
  - **Solution**: Check topic names and coordinate frames

### Integration Issues
- **Issue**: Isaac Sim and Isaac ROS not communicating
  - **Solution**: Verify topic names and message types match
- **Issue**: Coordinate frame mismatches
  - **Solution**: Check TF tree and frame conventions
- **Issue**: Timing synchronization problems
  - **Solution**: Verify clock synchronization settings

## Best Practices

### Performance Optimization
- **GPU Utilization**: Monitor GPU usage and optimize parameters
- **Memory Management**: Use appropriate buffer sizes
- **Threading**: Configure appropriate thread counts for your hardware

### Configuration Management
- **Parameter Files**: Use YAML files for node configuration
- **Launch Files**: Create modular launch files for different scenarios
- **Environment Setup**: Use consistent environment variables

### Testing and Validation
- **Unit Tests**: Test individual nodes before integration
- **Integration Tests**: Validate complete pipeline functionality
- **Performance Tests**: Monitor real-time performance requirements

## Resources

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/index.html)
- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [Isaac ROS GitHub Repository](https://github.com/NVIDIA-ISAAC-ROS)
- [CUDA Installation Guide](https://developer.nvidia.com/cuda-downloads)

## Next Steps

After successful installation:
1. Test Isaac ROS perception nodes with Isaac Sim
2. Implement your first perception pipeline
3. Validate performance with your specific robot configuration
4. Move on to advanced perception techniques