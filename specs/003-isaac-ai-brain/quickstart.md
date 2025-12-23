# Quick Start: NVIDIA Isaac™ AI-Robot Brain

## Overview
This quick start guide will help you set up the environment for the Isaac AI-Robot Brain module. This module covers advanced perception, navigation, and training for humanoid robots using NVIDIA Isaac Sim, Isaac ROS, and Nav2.

## Prerequisites
Before starting with this module, ensure you have:

1. **System Requirements**:
   - NVIDIA RTX graphics card (recommended RTX 3070 or higher with 8GB+ VRAM)
   - Ubuntu 20.04/22.04 or Windows 10/11
   - At least 16GB RAM
   - 100GB+ free disk space for Isaac Sim

2. **Software Prerequisites**:
   - ROS 2 Humble Hawksbill installed and operational
   - Basic understanding of ROS 2 concepts (nodes, topics, services)
   - Python 3.8+ and basic Python programming skills
   - Git for version control
   - Docker (optional, for containerized workflows)

## Setting Up Isaac Sim

### Step 1: Install Isaac Sim
1. Download Isaac Sim from NVIDIA Developer website
2. Follow the installation guide for your operating system
3. Ensure your NVIDIA drivers are up to date (driver version 495 or higher)

### Step 2: Verify Installation
```bash
# Launch Isaac Sim to verify installation
cd /path/to/isaac-sim
./isaac-sim.sh
```

### Step 3: Set Up Isaac Sim Environment
1. Launch Isaac Sim
2. Go to "File" → "New Scene" to create a blank environment
3. Save the scene as `humanoid_lab.usd` in your project directory

## Setting Up Isaac ROS

### Step 1: Install Isaac ROS Dependencies
```bash
# Add NVIDIA package repositories
sudo apt update
sudo apt install -y software-properties-common
wget https://repo.download.nvidia.com/client/ubuntu/$(lsb_release -cs)/nvidia-ml.list
sudo cp nvidia-ml.list /etc/apt/sources.list.d/nvidia-ml.list
sudo apt update

# Install Isaac ROS packages
sudo apt install -y ros-humble-isaac-ros-dev
sudo apt install -y ros-humble-isaac-ros-common
```

### Step 2: Source ROS 2 Environment
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source Isaac ROS packages
source /opt/ros/humble/install/setup.bash
```

## Setting Up Navigation (Nav2)

### Step 1: Install Nav2 Packages
```bash
# Install Nav2 packages
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-nav2-gui-plugins
```

### Step 2: Test Basic Navigation
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch basic navigation
ros2 launch nav2_bringup navigation_launch.py
```

## Creating Your First Isaac Simulation

### Step 1: Import a Humanoid Robot
1. Open Isaac Sim
2. Go to "Window" → "Asset Browser"
3. Import a humanoid robot model (e.g., from NVIDIA Isaac Gym examples)
4. Position the robot in your scene

### Step 2: Add Sensors to Your Robot
1. Select your humanoid robot in the scene
2. Right-click and choose "Add" → "Sensors"
3. Add a camera sensor and LiDAR sensor
4. Configure sensor parameters as needed

### Step 3: Configure Physics
1. Ensure the Physics Scene is properly configured
2. Set gravity to -9.81 m/s²
3. Adjust friction and restitution coefficients as needed

## Running Your First Perception Pipeline

### Step 1: Create a Simple Perception Node
Create a Python file `simple_perception.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import cv2
from cv_bridge import CvBridge

class SimplePerceptionNode(Node):
    def __init__(self):
        super().__init__('simple_perception')
        self.bridge = CvBridge()

        # Subscribe to camera feed from Isaac Sim
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Simple Perception Node Started')

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Simple object detection (example)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Add your perception logic here

        # Log detection results
        self.get_logger().info('Processed image frame')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Run the Perception Pipeline
```bash
# Make the script executable
chmod +x simple_perception.py

# Run the perception node
python3 simple_perception.py
```

## Running Navigation with Isaac Sim

### Step 1: Create a Navigation Launch File
Create `isaac_navigation.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': os.path.join(get_package_share_directory('your_package'), 'maps', 'humanoid_lab.yaml')}]
        ),
        Node(
            package='nav2_localization',
            executable='amcl',
            name='amcl'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server'
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server'
        )
    ])
```

## Next Steps

1. **Chapter 1**: Dive deeper into NVIDIA Isaac Sim for photorealistic simulation and synthetic data generation
2. **Chapter 2**: Explore Isaac ROS for hardware-accelerated VSLAM and perception pipelines
3. **Chapter 3**: Implement humanoid navigation with Nav2 for path planning and bipedal movement

## Troubleshooting

### Common Issues:

1. **Isaac Sim won't launch**:
   - Verify NVIDIA drivers are up to date
   - Check if CUDA is properly installed
   - Ensure sufficient VRAM is available

2. **ROS 2 packages not found**:
   - Verify ROS 2 Humble is properly installed
   - Check that environment is sourced correctly
   - Confirm Isaac ROS packages are installed

3. **Navigation fails**:
   - Verify robot configuration is correct
   - Check that maps are properly configured
   - Ensure sensors are publishing data

## Resources

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/book_intro.html)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages.html)
- [ROS 2 Navigation (Nav2) Documentation](https://navigation.ros.org/)
- [NVIDIA Developer Portal](https://developer.nvidia.com/)