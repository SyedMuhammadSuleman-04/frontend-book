---
sidebar_position: 3
---

# Perception Pipelines with Isaac ROS

This section covers Isaac ROS perception packages that enable hardware-accelerated object detection, segmentation, and sensor fusion. These pipelines are essential for humanoid robots to understand their environment and make informed navigation decisions.

## Isaac ROS Perception Stack

Isaac ROS provides a comprehensive set of perception packages optimized for NVIDIA GPUs:

### Isaac ROS Detection2D
- **Package**: `isaac_ros_detection2d`
- **Function**: 2D object detection from camera images
- **GPU Acceleration**: TensorRT for inference acceleration
- **Output**: Bounding boxes, class labels, and confidence scores

### Isaac ROS Detection3D
- **Package**: `isaac_ros_detection3d`
- **Function**: 3D object detection from point clouds
- **GPU Acceleration**: CUDA-accelerated clustering and geometric operations
- **Output**: 3D bounding boxes and object properties

### Isaac ROS Segmentation
- **Package**: `isaac_ros_segmentation`
- **Function**: Semantic and instance segmentation
- **GPU Acceleration**: TensorRT for pixel-wise classification
- **Output**: Segmentation masks and class probabilities

### Isaac ROS Image Format Converter
- **Package**: `isaac_ros_image_format_converter`
- **Function**: Convert between different image formats
- **GPU Acceleration**: CUDA-accelerated format conversion
- **Output**: Optimized image formats for perception algorithms

## Setting Up Perception Pipelines

### Object Detection Pipeline

```yaml
# detection_params.yaml
detection_node:
  ros__parameters:
    # Model parameters
    engine_file_path: "/path/to/tensorrt/engine.plan"
    input_tensor_names: ["input_tensor"]
    input_binding_names: ["input"]
    output_tensor_names: ["output_tensor"]
    output_binding_names: ["output"]

    # Input/Output topics
    input_image_topic: "/camera/rgb/image_rect_color"
    output_detections_topic: "/detections"

    # Performance parameters
    tensorrt_precision: "FP16"  # or "FP32"
    max_batch_size: 1
    input_layer_width: 640
    input_layer_height: 480
    input_type: "image"
```

### Segmentation Pipeline

```python
# segmentation_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from isaac_ros_tensor_rt.tensor_rt_engine import TensorRTEngine
from cv_bridge import CvBridge
import numpy as np

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        self.bridge = CvBridge()

        # Initialize TensorRT engine
        self.engine = TensorRTEngine(
            engine_path='/path/to/segmentation.engine',
            input_binding_names=['input'],
            output_binding_names=['output']
        )

        # Create publisher and subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_rect_color',
            self.image_callback,
            10
        )
        self.result_pub = self.create_publisher(
            Image,
            '/segmentation/mask',
            10
        )

    def image_callback(self, msg):
        # Convert ROS image to numpy array
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Preprocess image for TensorRT
        input_tensor = self.preprocess(cv_image)

        # Run inference
        output_tensor = self.engine.infer({'input': input_tensor})

        # Postprocess segmentation mask
        mask = self.postprocess(output_tensor['output'])

        # Publish result
        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        self.result_pub.publish(mask_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## GPU Acceleration with TensorRT

### TensorRT Optimization Process
1. **Model Conversion**: Convert trained models to TensorRT format
2. **Precision Optimization**: Optimize for FP16 or INT8 precision
3. **Layer Fusion**: Combine operations for better performance
4. **Memory Optimization**: Optimize GPU memory usage

### Performance Benefits
- **Speed**: 2-5x faster inference compared to CPU
- **Power Efficiency**: Better performance per watt
- **Real-time Capability**: Consistent frame rates for robotics applications

## Sensor Fusion

### Multi-Sensor Integration
Isaac ROS enables fusion of multiple sensor modalities:

```yaml
# fusion_params.yaml
sensor_fusion_node:
  ros__parameters:
    # Camera parameters
    camera_topic: "/camera/rgb/image_rect_color"
    camera_info_topic: "/camera/rgb/camera_info"

    # LiDAR parameters
    lidar_topic: "/lidar/points"

    # IMU parameters
    imu_topic: "/imu/data"

    # Fusion parameters
    fusion_rate: 10.0  # Hz
    sync_tolerance: 0.01  # seconds
```

### Data Association
- **Feature Matching**: Associate visual features with 3D points
- **Temporal Fusion**: Combine data across time for better estimates
- **Multi-view Fusion**: Combine data from multiple camera views

## Real-time Performance Optimization

### Pipeline Optimization Techniques
1. **Threading**: Separate threads for different pipeline stages
2. **Memory Management**: Pre-allocate buffers to avoid dynamic allocation
3. **Batch Processing**: Process multiple frames when possible
4. **GPU Scheduling**: Optimize GPU task scheduling

### Performance Monitoring
```python
# performance_monitor.py
import rclpy
from rclpy.node import Node
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        self.frame_count = 0
        self.start_time = time.time()

    def update_performance(self):
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.start_time

        if elapsed >= 1.0:  # Print every second
            fps = self.frame_count / elapsed
            self.get_logger().info(f'Current FPS: {fps:.2f}')
            self.frame_count = 0
            self.start_time = current_time
```

## Quality Metrics for Perception

### Detection Quality
- **Precision**: True positives / (True positives + False positives)
- **Recall**: True positives / (True positives + False negatives)
- **mAP**: Mean Average Precision across all classes

### Segmentation Quality
- **IoU**: Intersection over Union for segmentation masks
- **Pixel Accuracy**: Percentage of correctly classified pixels
- **Mean IoU**: Average IoU across all classes

### Real-time Performance
- **Frame Rate**: Frames per second processed
- **Latency**: Time from input to output
- **Jitter**: Variation in processing time

## Practical Exercise: Multi-Object Detection

### Objective
Implement a perception pipeline that detects multiple objects and publishes results.

### Steps
1. Set up object detection node with TensorRT model
2. Configure camera input from Isaac Sim
3. Implement detection visualization
4. Test with various objects in simulation
5. Evaluate detection accuracy and performance

### Expected Output
- Real-time object detection with bounding boxes
- Class labels and confidence scores
- Performance metrics (FPS, accuracy)

## Integration with Navigation

### Perception for Navigation
- **Obstacle Detection**: Identify navigable vs non-navigable areas
- **Path Planning**: Use object positions for path planning
- **Dynamic Objects**: Track moving obstacles for navigation safety

### Feedback Loop
- Perception → Navigation → Action → Perception
- Continuous refinement of environment understanding
- Adaptive behavior based on perception results

## Troubleshooting Common Issues

### Low Detection Accuracy
- Check model calibration for simulation environment
- Verify image quality and lighting conditions
- Adjust confidence thresholds appropriately
- Validate camera calibration parameters

### Performance Issues
- Monitor GPU utilization and memory
- Reduce input resolution if needed
- Optimize batch sizes for your hardware
- Check for pipeline bottlenecks

### Sensor Synchronization
- Ensure proper timing between sensors
- Use message filters for temporal alignment
- Validate coordinate frame transformations
- Check for dropped messages

## Next Steps

Now that you understand perception pipelines, let's practice with exercises to solidify your understanding in the next section. In the following chapter, we'll explore how to use these perception capabilities for humanoid navigation with Nav2.