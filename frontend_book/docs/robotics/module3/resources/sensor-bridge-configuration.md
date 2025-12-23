# Isaac Sim to ROS 2 Sensor Bridge Configuration

This guide provides detailed instructions for configuring the sensor bridge between Isaac Sim and ROS 2 to enable real-time sensor data exchange.

## Understanding the Sensor Bridge

### What is the Sensor Bridge?
The Isaac Sim to ROS 2 sensor bridge enables:
- **Real-time sensor data streaming** from Isaac Sim to ROS 2
- **Synchronized multi-sensor data** for perception pipelines
- **Hardware-in-the-loop testing** for robotics applications
- **Ground truth data generation** for training and validation

### Supported Sensor Types
- **Cameras**: RGB, depth, stereo, fisheye
- **LiDAR**: 3D scanning and solid-state LiDAR
- **IMU**: Accelerometer, gyroscope, magnetometer
- **Force/Torque**: Joint and contact sensors
- **GPS**: Global positioning system
- **Other**: Custom sensor types

## Prerequisites

### Required Packages
```bash
# Install Isaac ROS sensor bridge packages
sudo apt install -y ros-humble-isaac-ros-sensors
sudo apt install -y ros-humble-isaac-ros-cameras
sudo apt install -y ros-humble-isaac-ros-pointcloud
sudo apt install -y ros-humble-ros-gz-bridge
```

### Isaac Sim Extensions
Ensure the following extensions are enabled in Isaac Sim:
- **ROS2 Bridge**: Enables ROS 2 communication
- **Sensors**: Provides sensor simulation capabilities
- **Isaac ROS Bridge**: Specialized bridge for Isaac ROS packages

## Configuration Steps

### 1. Setting Up Isaac Sim Scene

#### Enable ROS Bridge Extension
1. In Isaac Sim, go to **Window** → **Extensions**
2. Search for "ROS" and enable **ROS2 Bridge**
3. Configure ROS domain ID (default is 0)
4. Verify that the bridge is running

#### Configure Robot with Sensors
1. Import your humanoid robot model into Isaac Sim
2. Add required sensors to the robot:
   - RGB camera
   - Depth camera
   - LiDAR
   - IMU
3. Position sensors appropriately on the robot
4. Configure sensor parameters (resolution, range, etc.)

### 2. ROS Bridge Configuration

#### Creating a Launch File for Sensor Bridge

```python
# sensor_bridge_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    config_file = LaunchConfiguration('config_file')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('your_robot_sensors'),
            'config',
            'sensor_bridge_params.yaml'
        ),
        description='Full path to config file to load'
    )

    # Isaac Sim ROS Bridge node
    ros_bridge_node = Node(
        package='isaac_ros_bridges',
        executable='ros_bridge_node',
        parameters=[config_file],
        remappings=[
            ('/camera/rgb/image_raw', '/camera/rgb/image_rect_color'),
            ('/camera/depth/image_raw', '/camera/depth/image_rect_raw'),
            ('/lidar/points', '/lidar/points'),
            ('/imu/data', '/imu/data_raw'),
            ('/odom', '/odometry/filtered')
        ]
    )

    # Image Rectification node (for camera data)
    image_rect_node = Node(
        package='isaac_ros_image_rectifier',
        executable='image_rectification_node',
        parameters=[config_file],
        remappings=[
            ('left_image', '/camera/rgb/image_raw'),
            ('right_image', '/camera/depth/image_raw'),
            ('left_camera_info', '/camera/rgb/camera_info'),
            ('right_camera_info', '/camera/depth/camera_info'),
            ('left_rectified_image', '/camera/rgb/image_rect_color'),
            ('right_rectified_image', '/camera/depth/image_rect_raw')
        ]
    )

    # Point Cloud conversion node
    pointcloud_node = Node(
        package='isaac_ros_pointcloud_utils',
        executable='isaac_ros_pointcloud_utils',
        parameters=[config_file],
        remappings=[
            ('/velodyne_points', '/lidar/points'),
            ('/pointcloud', '/lidar/processed_points')
        ]
    )

    return LaunchDescription([
        config_file_arg,
        ros_bridge_node,
        image_rect_node,
        pointcloud_node
    ])
```

### 3. Parameter Configuration

#### Sensor Bridge Parameters

```yaml
# config/sensor_bridge_params.yaml
/**:
  ros__parameters:
    # Global parameters
    use_sim_time: true
    publish_frequency: 30.0  # Hz
    tf_publish_frequency: 100.0  # Hz

# ROS Bridge parameters
ros_bridge_node:
  ros__parameters:
    # Camera parameters
    camera_rgb_topic: "/camera/rgb/image_raw"
    camera_depth_topic: "/camera/depth/image_raw"
    camera_info_topic: "/camera/rgb/camera_info"

    # LiDAR parameters
    lidar_topic: "/lidar/points"
    lidar_frame_id: "lidar_link"

    # IMU parameters
    imu_topic: "/imu/data"
    imu_frame_id: "imu_link"

    # Odometry parameters
    odom_topic: "/odometry/filtered"
    odom_frame_id: "odom"
    base_frame_id: "base_link"

# Image rectification parameters
image_rect_node:
  ros__parameters:
    # Rectification parameters
    alpha: 0.0  # 0=fully rectified, 1=original image
    use_sim_time: true

# Point Cloud parameters
pointcloud_node:
  ros__parameters:
    input_topic: "/lidar/points"
    output_topic: "/lidar/processed_points"
    processing_rate: 10.0
    use_sim_time: true
```

## Isaac Sim Configuration

### Configuring Sensors in Isaac Sim

#### Camera Configuration
1. **In Isaac Sim viewport**, select your camera sensor
2. **In the Property panel**, configure:
   - **Resolution**: Set appropriate width and height (e.g., 640x480)
   - **Focal Length**: Set appropriate FOV (e.g., 35mm equivalent)
   - **Clipping Planes**: Near: 0.1m, Far: 100m
3. **In the Isaac ROS extension**, configure:
   - **Topic Name**: `/camera/rgb/image_raw`
   - **Frame ID**: `camera_rgb_optical_frame`
   - **Format**: `rgb8` or `bgr8`

#### LiDAR Configuration
1. **Select your LiDAR sensor** in Isaac Sim
2. **Configure LiDAR properties**:
   - **Range**: Maximum detection distance (e.g., 50m)
   - **Angular Resolution**: Horizontal and vertical resolution
   - **Field of View**: Horizontal and vertical FOV
   - **Rotation Rate**: How fast the LiDAR spins (e.g., 10Hz)
3. **Set ROS topic** to `/lidar/points`
4. **Set frame ID** to `lidar_link`

#### IMU Configuration
1. **Add IMU sensor** to your robot (typically on the torso)
2. **Configure IMU properties**:
   - **Update Rate**: How frequently the IMU updates (e.g., 100Hz)
   - **Noise Parameters**: Set realistic noise levels
   - **Range Limits**: Maximum measurable values
3. **Set ROS topic** to `/imu/data`
4. **Set frame ID** to `imu_link`

### Coordinate Frame Configuration

#### Setting Up TF Frames
Ensure proper TF tree structure:
```
map
└── odom
    └── base_link
        ├── camera_rgb_optical_frame
        ├── camera_depth_optical_frame
        ├── lidar_link
        ├── imu_link
        └── [other sensor frames]
```

#### Isaac Sim TF Configuration
1. **In Isaac Sim**, ensure each sensor has the correct parent frame
2. **Verify coordinate conventions** match ROS standards:
   - **X**: Forward
   - **Y**: Left
   - **Z**: Up

## Testing the Sensor Bridge

### Verification Steps

#### 1. Launch Isaac Sim with ROS Bridge
```bash
# Terminal 1: Launch Isaac Sim
cd /path/to/isaac-sim
./isaac-sim.sh

# In Isaac Sim, enable the ROS2 Bridge extension
# Configure your scene with sensors
```

#### 2. Launch ROS Bridge Nodes
```bash
# Terminal 2: Source ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Launch the sensor bridge
ros2 launch your_robot_sensors sensor_bridge_launch.py
```

#### 3. Verify Sensor Data
```bash
# Check available topics
ros2 topic list | grep camera
ros2 topic list | grep lidar
ros2 topic list | grep imu

# Check camera data
ros2 topic echo /camera/rgb/image_rect_color --field data --field header.stamp

# Check LiDAR data
ros2 topic echo /lidar/points --field header.stamp

# Check IMU data
ros2 topic echo /imu/data --field header.stamp
```

#### 4. Visualize in RViz
```bash
# Terminal 3: Launch RViz
ros2 run rviz2 rviz2

# Add displays for:
# - Image (for camera data)
# - PointCloud2 (for LiDAR data)
# - Imu (for IMU data)
# - TF (to verify transforms)
```

## Advanced Configuration

### Multi-Robot Sensor Bridge
For multiple robots, use different ROS namespaces:

```yaml
# multi_robot_sensor_bridge.yaml
robot1:
  ros_bridge_node:
    ros__parameters:
      camera_rgb_topic: "/robot1/camera/rgb/image_raw"
      lidar_topic: "/robot1/lidar/points"
      imu_topic: "/robot1/imu/data"

robot2:
  ros_bridge_node:
    ros__parameters:
      camera_rgb_topic: "/robot2/camera/rgb/image_raw"
      lidar_topic: "/robot2/lidar/points"
      imu_topic: "/robot2/imu/data"
```

### Synchronization Configuration
```yaml
# synchronization_params.yaml
/**:
  ros__parameters:
    # Synchronization parameters
    sync_tolerance: 0.01  # seconds
    sync_queue_size: 10

    # Buffer sizes
    image_buffer_size: 5
    lidar_buffer_size: 5
    imu_buffer_size: 100
```

## Troubleshooting Common Issues

### No Sensor Data Issues
- **Issue**: No data published on sensor topics
  - **Solution**: Verify Isaac Sim ROS bridge extension is enabled
  - **Solution**: Check that sensors are properly configured in Isaac Sim
  - **Solution**: Verify topic names match between Isaac Sim and ROS

### Timing Synchronization Issues
- **Issue**: Sensor data timestamps are not synchronized
  - **Solution**: Ensure Isaac Sim and ROS use the same clock source
  - **Solution**: Use `use_sim_time: true` parameter
  - **Solution**: Check that Isaac Sim is publishing at consistent rates

### TF Frame Issues
- **Issue**: TF frames not published or incorrect
  - **Solution**: Verify frame names match between Isaac Sim and ROS
  - **Solution**: Check that TF tree is properly connected
  - **Solution**: Ensure coordinate frame conventions match

### Performance Issues
- **Issue**: High CPU/GPU usage
  - **Solution**: Reduce sensor resolution or update rates
  - **Solution**: Optimize buffer sizes
  - **Solution**: Use appropriate image compression

## Performance Optimization

### Data Compression
```yaml
# compression_params.yaml
/**:
  ros__parameters:
    # Image compression
    image_transport: "compressed"
    compressed_format: "jpeg"
    compressed_quality: 80

    # Point cloud compression
    pointcloud_compression: true
    pointcloud_compression_level: 6
```

### Threading Configuration
```yaml
# threading_params.yaml
/**:
  ros__parameters:
    # Threading parameters
    num_threads: 4
    sensor_processing_threads: 2
    image_processing_threads: 2
```

## Quality Assurance

### Data Quality Checks
1. **Verify data rates**: Check that sensors publish at expected rates
2. **Validate data ranges**: Ensure sensor data is within expected ranges
3. **Check synchronization**: Verify temporal alignment between sensors
4. **Validate transforms**: Ensure TF frames are correct and consistent

### Performance Monitoring
- **CPU Usage**: Monitor processor utilization
- **GPU Usage**: Monitor GPU utilization for rendering
- **Memory Usage**: Monitor memory consumption
- **Network Usage**: Monitor bandwidth for remote connections

## Integration with Perception Pipelines

### Connecting to Isaac ROS Perception
```python
# perception_integration_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include sensor bridge launch
    sensor_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('your_robot_sensors'),
            '/launch/sensor_bridge_launch.py'
        ])
    )

    # Include perception pipeline launch
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('your_robot_perception'),
            '/launch/perception_pipeline_launch.py'
        ])
    )

    return LaunchDescription([
        sensor_bridge_launch,
        perception_launch
    ])
```

## Resources

- [Isaac Sim ROS Bridge Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_ros.html)
- [ROS 2 Sensor Integration Guide](http://wiki.ros.org/sensors)
- [Isaac ROS Bridge Package](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_bridges/index.html)

## Next Steps

After configuring the sensor bridge:
1. Test with your specific robot model
2. Validate sensor data quality
3. Integrate with perception pipelines
4. Optimize performance for your use case
5. Test in various simulation scenarios