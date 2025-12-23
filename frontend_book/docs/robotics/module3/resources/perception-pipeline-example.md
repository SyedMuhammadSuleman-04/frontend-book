# Isaac ROS Perception Pipeline Example

This document provides a complete example of implementing a perception pipeline using Isaac ROS packages with hardware acceleration.

## Pipeline Overview

The perception pipeline demonstrates:
- Hardware-accelerated object detection
- Sensor fusion from multiple modalities
- Real-time processing capabilities
- Integration with ROS 2 ecosystem

## Required Components

### Hardware Requirements
- NVIDIA GPU with CUDA support (RTX series recommended)
- Compatible robot with RGB camera, LiDAR, and IMU sensors
- Sufficient computational resources for real-time processing

### Software Requirements
- ROS 2 Humble Hawksbill
- Isaac ROS packages installed
- Isaac Sim (for simulation testing)
- OpenCV and other standard ROS packages

## Complete Perception Pipeline Implementation

### 1. Perception Pipeline Launch File

```python
# perception_pipeline_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('your_robot_perception'),
            'config',
            'perception_pipeline_params.yaml'
        ),
        description='Full path to params file to load'
    )

    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        parameters=[params_file],
        remappings=[
            ('/visual_slam_node/left/image', '/camera/rgb/left/image_raw'),
            ('/visual_slam_node/right/image', '/camera/rgb/right/image_raw'),
            ('/visual_slam_node/left/camera_info', '/camera/rgb/left/camera_info'),
            ('/visual_slam_node/right/camera_info', '/camera/rgb/right/camera_info'),
            ('/visual_slam_node/imu', '/imu/data'),
            ('/visual_slam_node/visual_slam/trajectory', '/slam/trajectory'),
            ('/visual_slam_node/map_pose_stamped', '/slam/map_pose')
        ]
    )

    # Isaac ROS Detection2D node for object detection
    detection_node = Node(
        package='isaac_ros_detection2d',
        executable='detection2_dnode',
        parameters=[params_file],
        remappings=[
            ('/image', '/camera/rgb/image_rect_color'),
            ('/detections', '/detections'),
            ('/tensor_pub/image_tensor', '/tensor_pub/image_tensor'),
            ('/tensor_pub/tensor_array', '/tensor_pub/tensor_array')
        ]
    )

    # Isaac ROS Segmentation node
    segmentation_node = Node(
        package='isaac_ros_segmentation',
        executable='segmentation_node',
        parameters=[params_file],
        remappings=[
            ('/image', '/camera/rgb/image_rect_color'),
            ('/segmentation', '/segmentation/mask'),
            ('/colored_segmentation', '/segmentation/colored')
        ]
    )

    # Isaac ROS Point Cloud Node for LiDAR processing
    point_cloud_node = Node(
        package='isaac_ros_pointcloud_utils',
        executable='isaac_ros_pointcloud_utils',
        parameters=[params_file],
        remappings=[
            ('/velodyne_points', '/lidar/points'),
            ('/pointcloud', '/lidar/processed_points')
        ]
    )

    # Sensor fusion node (custom implementation)
    fusion_node = Node(
        package='your_robot_perception',
        executable='sensor_fusion_node',
        parameters=[params_file],
        remappings=[
            ('/camera_detections', '/detections'),
            ('/lidar_points', '/lidar/processed_points'),
            ('/imu_data', '/imu/data'),
            ('/fused_perception', '/perception/fused_output')
        ]
    )

    return LaunchDescription([
        params_file_arg,
        visual_slam_node,
        detection_node,
        segmentation_node,
        point_cloud_node,
        fusion_node
    ])
```

### 2. Parameter Configuration File

```yaml
# config/perception_pipeline_params.yaml
/**:
  ros__parameters:
    # Global parameters
    use_sim_time: true
    processing_frequency: 30.0  # Hz

# Visual SLAM parameters
visual_slam_node:
  ros__parameters:
    enable_rectification: true
    rectified_left_topic_name: "/camera/rgb/left/image_rect_color"
    rectified_right_topic_name: "/camera/rgb/right/image_rect_color"
    left_camera_info_topic_name: "/camera/rgb/left/camera_info"
    right_camera_info_topic_name: "/camera/rgb/right/camera_info"
    imu_topic_name: "/imu/data"
    max_num_corners: 1000
    min_num_corners: 100
    tracking_rate: 30.0
    mapping_rate: 1.0
    enable_imu_fusion: true

# Detection2D parameters
detection_node:
  ros__parameters:
    engine_file_path: "/path/to/yolov5s.plan"  # TensorRT engine
    input_tensor_names: ["input_tensor"]
    input_binding_names: ["input"]
    output_tensor_names: ["output_tensor"]
    output_binding_names: ["output"]
    input_image_topic: "/camera/rgb/image_rect_color"
    output_detections_topic: "/detections"
    tensorrt_precision: "FP16"
    max_batch_size: 1
    input_layer_width: 640
    input_layer_height: 640
    input_type: "image"

# Segmentation parameters
segmentation_node:
  ros__parameters:
    engine_file_path: "/path/to/segmentation.plan"  # TensorRT engine
    input_tensor_names: ["input"]
    input_binding_names: ["input"]
    output_tensor_names: ["output"]
    output_binding_names: ["output"]
    input_image_topic: "/camera/rgb/image_rect_color"
    output_segmentation_topic: "/segmentation/mask"
    tensorrt_precision: "FP16"
    max_batch_size: 1

# Point Cloud processing parameters
point_cloud_node:
  ros__parameters:
    input_topic: "/lidar/points"
    output_topic: "/lidar/processed_points"
    processing_rate: 10.0

# Custom sensor fusion parameters
fusion_node:
  ros__parameters:
    camera_detections_topic: "/detections"
    lidar_points_topic: "/lidar/processed_points"
    imu_topic: "/imu/data"
    output_topic: "/perception/fused_output"
    sync_tolerance: 0.05  # seconds
    fusion_rate: 10.0  # Hz
```

### 3. Custom Sensor Fusion Node Implementation

```python
#!/usr/bin/env python3
# sensor_fusion_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import numpy as np
import message_filters
from tf2_ros import TransformListener, Buffer
from visualization_msgs.msg import MarkerArray, Marker
import threading

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Declare parameters
        self.declare_parameter('sync_tolerance', 0.05)
        self.declare_parameter('fusion_rate', 10.0)

        self.sync_tolerance = self.get_parameter('sync_tolerance').value
        self.fusion_rate = self.get_parameter('fusion_rate').value

        # Initialize data storage
        self.camera_detections = None
        self.lidar_points = None
        self.imu_data = None

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscribers
        self.camera_sub = message_filters.Subscriber(
            self, Detection2DArray, 'camera_detections'
        )
        self.lidar_sub = message_filters.Subscriber(
            self, PointCloud2, 'lidar_points'
        )
        self.imu_sub = message_filters.Subscriber(
            self, Imu, 'imu_data'
        )

        # Create approximate time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.camera_sub, self.lidar_sub, self.imu_sub],
            queue_size=10,
            slop=self.sync_tolerance
        )
        self.ts.registerCallback(self.fusion_callback)

        # Create publisher for fused output
        self.fused_pub = self.create_publisher(
            MarkerArray, 'fused_perception', 10
        )

        # Create publisher for visualization
        self.viz_pub = self.create_publisher(
            MarkerArray, 'perception_visualization', 10
        )

        # Create timer for fusion processing
        self.fusion_timer = self.create_timer(
            1.0 / self.fusion_rate, self.process_fusion
        )

        self.get_logger().info('Sensor Fusion Node initialized')

    def fusion_callback(self, camera_detections, lidar_points, imu_data):
        """Callback for synchronized sensor data"""
        self.camera_detections = camera_detections
        self.lidar_points = lidar_points
        self.imu_data = imu_data

    def process_fusion(self):
        """Process sensor fusion and publish results"""
        if self.camera_detections is None or self.lidar_points is None:
            return

        try:
            # Perform sensor fusion
            fused_results = self.perform_fusion(
                self.camera_detections,
                self.lidar_points,
                self.imu_data
            )

            # Publish fused results
            self.publish_fused_results(fused_results)

        except Exception as e:
            self.get_logger().error(f'Error in fusion processing: {e}')

    def perform_fusion(self, camera_detections, lidar_points, imu_data):
        """Perform the actual sensor fusion"""
        # Convert LiDAR points to numpy array
        lidar_array = self.pointcloud2_to_array(lidar_points)

        # Project camera detections to 3D space
        projected_detections = self.project_detections_to_3d(
            camera_detections, lidar_array
        )

        # Fuse with IMU data for motion compensation
        fused_objects = self.fuse_with_imu(
            projected_detections, imu_data
        )

        return fused_objects

    def pointcloud2_to_array(self, pointcloud2_msg):
        """Convert PointCloud2 message to numpy array"""
        import sensor_msgs.point_cloud2 as pc2
        points = pc2.read_points(
            pointcloud2_msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )
        return np.array(list(points))

    def project_detections_to_3d(self, detections, lidar_points):
        """Project 2D camera detections to 3D using LiDAR data"""
        projected_objects = []

        for detection in detections.detections:
            # Get bounding box center in 2D
            center_x = detection.bbox.center.x
            center_y = detection.bbox.center.y

            # Project to 3D using LiDAR data
            # This is a simplified approach - in practice, you'd use camera calibration
            # and more sophisticated projection methods
            if len(lidar_points) > 0:
                # Find corresponding 3D point for the 2D detection
                # (This is a placeholder - actual implementation would be more complex)
                avg_point = np.mean(lidar_points, axis=0)

                projected_obj = {
                    'class': detection.results[0].hypothesis.class_id if detection.results else 'unknown',
                    'confidence': detection.results[0].hypothesis.score if detection.results else 0.0,
                    'position': Point(x=avg_point[0], y=avg_point[1], z=avg_point[2]),
                    'bbox_3d': None  # Would compute 3D bounding box
                }

                projected_objects.append(projected_obj)

        return projected_objects

    def fuse_with_imu(self, projected_detections, imu_data):
        """Fuse projected detections with IMU data for motion compensation"""
        # Apply IMU-based motion compensation
        # This would adjust object positions based on robot motion
        compensated_objects = []

        for obj in projected_detections:
            # Apply motion compensation based on IMU data
            compensated_obj = obj.copy()
            # Apply compensation logic here
            compensated_objects.append(compensated_obj)

        return compensated_objects

    def publish_fused_results(self, fused_objects):
        """Publish fused perception results"""
        marker_array = MarkerArray()

        for i, obj in enumerate(fused_objects):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"

            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position = obj['position']
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5

            marker.color.r = 1.0 if obj['class'] == 'person' else 0.0
            marker.color.g = 1.0 if obj['class'] == 'vehicle' else 0.0
            marker.color.b = 1.0 if obj['class'] == 'obstacle' else 0.5
            marker.color.a = 0.8

            marker.text = f"{obj['class']}: {obj['confidence']:.2f}"

            marker_array.markers.append(marker)

        self.fused_pub.publish(marker_array)
        self.viz_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Package.xml for the Perception Package

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>your_robot_perception</name>
  <version>0.1.0</version>
  <description>Perception pipeline using Isaac ROS packages</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>message_filters</depend>
  <depend>tf2_ros</depend>
  <depend>visualization_msgs</depend>
  <depend>isaac_ros_visual_slam</depend>
  <depend>isaac_ros_detection2d</depend>
  <depend>isaac_ros_segmentation</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 5. Setup.py for the Perception Package

```python
from setuptools import find_packages, setup

package_name = 'your_robot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/perception_pipeline_launch.py']),
        ('share/' + package_name + '/config',
            ['config/perception_pipeline_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Perception pipeline using Isaac ROS packages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion_node = your_robot_perception.sensor_fusion_node:main',
        ],
    },
)
```

## Running the Perception Pipeline

### 1. Building the Package

```bash
cd ~/isaac_ros_ws
colcon build --packages-select your_robot_perception
source install/setup.bash
```

### 2. Launching the Pipeline

```bash
# Launch the complete perception pipeline
ros2 launch your_robot_perception perception_pipeline_launch.py
```

### 3. Testing with Isaac Sim

```bash
# Terminal 1: Launch Isaac Sim with your robot
cd /path/to/isaac-sim
./isaac-sim.sh

# Terminal 2: Launch the perception pipeline
cd ~/isaac_ros_ws
source install/setup.bash
ros2 launch your_robot_perception perception_pipeline_launch.py

# Terminal 3: Visualize results in RViz
ros2 run rviz2 rviz2
```

## Performance Optimization

### GPU Utilization
- Monitor GPU usage with `nvidia-smi`
- Optimize batch sizes for your hardware
- Use appropriate precision (FP16 vs FP32) based on accuracy requirements

### Memory Management
- Pre-allocate buffers where possible
- Use efficient data structures
- Monitor memory usage and optimize accordingly

### Real-time Performance
- Profile each pipeline stage
- Identify bottlenecks
- Optimize the slowest components first

## Troubleshooting

### Common Issues and Solutions

1. **Pipeline runs slowly**:
   - Check GPU utilization
   - Reduce input resolution
   - Optimize batch processing

2. **Memory errors**:
   - Reduce batch sizes
   - Check for memory leaks
   - Increase available GPU memory

3. **Sensor synchronization issues**:
   - Adjust sync tolerance parameters
   - Verify sensor publishing rates
   - Check for dropped messages

4. **Detection accuracy issues**:
   - Verify sensor calibration
   - Check lighting conditions
   - Validate model inputs

## Validation and Testing

### Performance Metrics
- **Frame Rate**: Target 30+ FPS for real-time operation
- **Latency**: Keep end-to-end latency under 100ms
- **Accuracy**: Validate detection accuracy against ground truth
- **Robustness**: Test in various environmental conditions

### Testing Scenarios
- Static scene testing
- Moving robot scenarios
- Dynamic object detection
- Low-light conditions
- Occluded objects

## Resources

- [Isaac ROS Perception Tutorials](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_perception/index.html)
- [ROS 2 Message Filters](http://wiki.ros.org/message_filters)
- [TensorRT Optimization Guide](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html)

## Next Steps

After implementing this basic perception pipeline:
1. Optimize for your specific robot and use case
2. Add more sophisticated fusion algorithms
3. Implement tracking and prediction
4. Test with real hardware
5. Deploy to your target platform