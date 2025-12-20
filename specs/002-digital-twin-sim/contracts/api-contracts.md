# API Contracts: Digital Twin Simulation for Humanoid Robots

**Feature**: 002-digital-twin-sim
**Created**: 2025-12-20
**Status**: Draft

## Overview

This document defines the API contracts for the Digital Twin Simulation educational module. These contracts represent the interfaces between different components of the simulation system and the expected interactions for educational purposes.

## ROS 2 Message Contracts

### Sensor Data Contracts

**sensor_msgs/LaserScan for LiDAR Simulation**
```
# Time of scan
Header header
# Start angle of the scan [rad]
float32 angle_min
# Final angle of the scan [rad]
float32 angle_max
# Angular distance between measurements [rad]
float32 angle_increment
# Time between measurements [seconds]
float32 time_increment
# Time between scans [seconds]
float32 scan_time
# Minimum range value [m]
float32 range_min
# Maximum range value [m]
float32 range_max
# Range data [m]
float32[] ranges
# Intensity data [device dependent]
float32[] intensities
```

**sensor_msgs/Image for Camera Simulation**
```
Header header
# Image encoding format
string encoding
# Is this data bigendian?
uint8 is_bigendian
# Image data size in bytes
uint32 step
# Image data
uint8[] data
```

**sensor_msgs/Imu for IMU Simulation**
```
Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

### Physics Simulation Contracts

**gazebo_msgs/ModelState for Robot Positioning**
```
string model_name
geometry_msgs/Pose pose
geometry_msgs/Twist twist
string reference_frame
```

**gazebo_msgs/WorldState for Environment State**
```
Header header
string[] name
geometry_msgs/Pose[] pose
geometry_msgs/Twist[] twist
geometry_msgs/Wrench[] wrench
```

## Service Contracts

### Environment Configuration Service
**Service Type**: `gazebo_msgs/SetPhysicsProperties`
```
# Gravity vector
geometry_msgs/Vector3 gravity
# Type of physics engine to use
string physics_engine
# Max step size for the integrator
float64 max_step_size
# Min step size for the integrator
float64 min_step_size
# Number of threads to use for physics updates
int32 real_time_update_rate
# Max number of contacts
int32 max_contacts
```

### Robot Control Service
**Service Type**: `std_srvs/SetBool`
```
bool data  # Enable/disable robot
---
bool success
string message
```

## Action Contracts

### Simulation Control Actions
**Action Type**: `control_msgs/FollowJointTrajectory`
```
# Goal
trajectory_msgs/JointTrajectory trajectory
control_msgs/JointTrajectoryControllerState path_tolerance
control_msgs/JointTrajectoryControllerState goal_tolerance
duration goal_time_tolerance

# Result
int32 error_code
string error_string

# Feedback
control_msgs/JointTrajectoryControllerState desired
control_msgs/JointTrajectoryControllerState actual
control_msgs/JointTrajectoryControllerState error
```

## Topic Contracts

### Published Topics
- `/robot/sensors/lidar` - `sensor_msgs/LaserScan` - LiDAR sensor data
- `/robot/sensors/camera/image_raw` - `sensor_msgs/Image` - Camera sensor data
- `/robot/sensors/imu` - `sensor_msgs/Imu` - IMU sensor data
- `/robot/state` - `gazebo_msgs/LinkStates` - Robot state information
- `/gazebo/model_states` - `gazebo_msgs/ModelStates` - Model state information

### Subscribed Topics
- `/robot/joint_commands` - `std_msgs/Float64MultiArray` - Joint position commands
- `/robot/velocity_commands` - `geometry_msgs/Twist` - Velocity commands
- `/gazebo/set_model_state` - `gazebo_msgs/ModelState` - Model state updates

## Validation Rules

### For LiDAR Data:
- Range values must be within [range_min, range_max]
- Array sizes of ranges and intensities must match
- Angle parameters must form a valid scan range

### For Camera Data:
- Image data size must match width * height * pixel_depth
- Encoding must be one of supported formats (rgb8, bgr8, rgba8, etc.)
- Step value must match width * bytes_per_pixel

### For IMU Data:
- Orientation quaternion must be normalized
- Covariance matrices must be positive semi-definite
- Acceleration values should be within realistic ranges

## Error Handling

### Common Error Codes:
- `0`: SUCCESS - Operation completed successfully
- `1`: INVALID_PARAMETERS - Provided parameters are invalid
- `2`: SIMULATION_ERROR - Error occurred during simulation
- `3`: TIMEOUT - Operation timed out
- `4`: NOT_READY - Required components not ready

### Error Response Format:
```
{
  "success": false,
  "error_code": int,
  "message": string,
  "timestamp": datetime
}
```