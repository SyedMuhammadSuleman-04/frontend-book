# Troubleshooting Guide: Isaac AI Brain Module

This guide provides solutions to common issues encountered when working with Isaac Sim, Isaac ROS, and Nav2 for humanoid robot navigation.

## Isaac Sim Troubleshooting

### Installation and Launch Issues

#### Isaac Sim Won't Launch
**Symptoms**: Isaac Sim fails to start or crashes immediately

**Solutions**:
1. **Check GPU drivers**:
   ```bash
   nvidia-smi
   ```
   - Ensure drivers are up to date (version 495 or higher)
   - If outdated, install latest drivers and reboot

2. **Verify CUDA installation**:
   ```bash
   nvcc --version
   ```
   - Ensure CUDA version is compatible with your GPU driver
   - Check that CUDA libraries are properly linked

3. **Check system requirements**:
   - Ensure sufficient VRAM (8GB+ recommended)
   - Verify adequate system RAM (16GB+ recommended)
   - Check available disk space (100GB+ recommended)

4. **Run with verbose output**:
   ```bash
   ./isaac-sim.sh --verbose
   ```
   - Check output for specific error messages
   - Look for missing dependencies or configuration issues

#### Poor Performance or Low Frame Rates
**Symptoms**: Isaac Sim runs slowly, low FPS, stuttering

**Solutions**:
1. **Reduce scene complexity**:
   - Simplify meshes or use lower-poly versions
   - Reduce the number of objects in the scene
   - Use simpler materials and textures

2. **Adjust rendering settings**:
   - Lower rendering quality in Isaac Sim settings
   - Reduce resolution if possible
   - Disable advanced rendering features (global illumination, etc.)

3. **Close other GPU-intensive applications**:
   - Close other applications using the GPU
   - Check for background processes consuming GPU resources

4. **Monitor GPU usage**:
   ```bash
   watch -n 0.1 nvidia-smi
   ```
   - Check if GPU is being fully utilized
   - Look for memory limitations

### Sensor Issues in Isaac Sim

#### Sensors Not Publishing Data
**Symptoms**: Isaac Sim sensors are not publishing ROS topics

**Solutions**:
1. **Check Isaac Sim ROS bridge extension**:
   - Ensure ROS2 Bridge extension is enabled in Isaac Sim
   - Verify ROS domain ID is set correctly
   - Check that Isaac Sim is properly connected to ROS

2. **Verify sensor configuration**:
   - Check that sensors are properly attached to robot
   - Verify sensor parameters (resolution, frequency, etc.)
   - Ensure sensor topics are configured correctly

3. **Check ROS topic connectivity**:
   ```bash
   ros2 topic list
   ros2 topic echo /sensor_topic_name
   ```

#### Camera Images Look Incorrect
**Symptoms**: Distorted images, wrong colors, or incorrect field of view

**Solutions**:
1. **Check camera parameters**:
   - Verify focal length and field of view settings
   - Check camera resolution settings
   - Verify camera position and orientation

2. **Calibrate camera**:
   - Use ROS camera calibration tools
   - Verify intrinsic and extrinsic parameters
   - Check for distortion parameters

3. **Adjust rendering settings**:
   - Check that color space is set correctly
   - Verify gamma settings
   - Adjust exposure and lighting parameters

## Isaac ROS Troubleshooting

### Installation Issues

#### Isaac ROS Packages Not Found
**Symptoms**: ROS cannot find Isaac ROS packages

**Solutions**:
1. **Check package installation**:
   ```bash
   dpkg -l | grep isaac
   ```
   - Verify packages are installed correctly
   - If missing, reinstall using apt

2. **Source ROS environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ```
   - Ensure Isaac ROS workspace is sourced
   - Check that workspace was built successfully

3. **Verify workspace build**:
   ```bash
   cd ~/isaac_ros_ws
   colcon build --packages-select isaac_ros_visual_slam
   ```

### Perception Pipeline Issues

#### VSLAM Nodes Not Running
**Symptoms**: Visual SLAM nodes fail to start or crash

**Solutions**:
1. **Check GPU compatibility**:
   - Verify GPU supports required CUDA operations
   - Check that GPU has sufficient VRAM for VSLAM operations
   - Ensure CUDA drivers are properly installed

2. **Verify sensor data**:
   ```bash
   ros2 topic echo /camera/rgb/image_raw
   ros2 topic echo /imu/data
   ```
   - Ensure stereo cameras or RGB-D data is available
   - Verify IMU data is being published

3. **Check parameter configuration**:
   - Verify camera info topics are correctly configured
   - Check that intrinsic and extrinsic parameters are correct
   - Ensure frame IDs match between sensors

#### Object Detection Not Working
**Symptoms**: Detection nodes run but produce no results or poor results

**Solutions**:
1. **Check TensorRT engine**:
   - Verify TensorRT engine file exists and is accessible
   - Ensure engine is compatible with current TensorRT version
   - Check that model was properly converted to TensorRT format

2. **Verify input data**:
   - Check that input image topics are publishing correctly
   - Verify image format and resolution match model requirements
   - Ensure images are properly rectified if required

3. **Adjust detection parameters**:
   - Modify confidence thresholds
   - Check that input tensor names match model expectations
   - Verify preprocessing parameters

### Sensor Bridge Issues

#### Isaac Sim to ROS Bridge Not Working
**Symptoms**: No data flowing from Isaac Sim to ROS

**Solutions**:
1. **Check Isaac Sim extension**:
   - Ensure ROS2 Bridge extension is enabled
   - Verify extension is properly configured
   - Check for any extension errors in Isaac Sim console

2. **Verify network connectivity**:
   - Check that Isaac Sim and ROS are on same network
   - Verify ROS domain ID matches between Isaac Sim and ROS
   - Check firewall settings if applicable

3. **Check topic mapping**:
   - Verify topic names match between Isaac Sim and ROS
   - Check that frame IDs are consistent
   - Ensure message types are compatible

## Nav2 Troubleshooting

### Navigation Stack Issues

#### Nav2 Nodes Not Starting
**Symptoms**: Navigation nodes fail to start or lifecycle nodes stay in unconfigured state

**Solutions**:
1. **Check parameter files**:
   - Verify YAML parameter files are properly formatted
   - Check for missing or incorrect parameter values
   - Ensure file paths are correct and accessible

2. **Verify lifecycle management**:
   ```bash
   ros2 lifecycle list controller_server
   ```
   - Check if nodes are in correct lifecycle state
   - Use lifecycle manager to activate nodes if needed

3. **Check costmap configuration**:
   - Verify costmap parameters are properly set
   - Check that sensor topics are correctly configured
   - Ensure TF tree is complete and consistent

#### Navigation Goals Not Working
**Symptoms**: Navigation goals are rejected or robot doesn't move

**Solutions**:
1. **Check TF tree**:
   ```bash
   ros2 run tf2_tools view_frames
   ```
   - Verify complete TF tree exists
   - Check that all required frames are published
   - Ensure transforms are being updated regularly

2. **Verify localization**:
   - Check that robot pose is being published
   - Verify AMCL or other localization node is running
   - Ensure initial pose is set correctly

3. **Check map server**:
   - Verify map is loaded and being published
   - Check that map topic is accessible
   - Ensure map resolution and metadata are correct

### Humanoid-Specific Navigation Issues

#### Balance Loss During Navigation
**Symptoms**: Robot falls over or becomes unstable during navigation

**Solutions**:
1. **Reduce navigation speeds**:
   - Lower commanded linear and angular velocities
   - Increase safety margins in controller parameters
   - Reduce acceleration and deceleration rates

2. **Check controller parameters**:
   - Verify step length and height parameters are appropriate
   - Check balance margin settings
   - Ensure CoM (Center of Mass) constraints are realistic

3. **Improve sensor integration**:
   - Ensure IMU data is properly integrated
   - Check that balance feedback is working
   - Verify sensor fusion is providing stable data

#### Footstep Planning Issues
**Symptoms**: Robot doesn't take proper steps or steps collide with obstacles

**Solutions**:
1. **Check footstep planner configuration**:
   - Verify step size parameters are appropriate
   - Check that footstep planning is enabled
   - Ensure obstacle avoidance is properly configured

2. **Verify sensor data quality**:
   - Check that LiDAR provides sufficient data for step planning
   - Verify camera data is properly processed
   - Ensure sensor fusion provides accurate environment representation

3. **Adjust planning parameters**:
   - Modify step frequency and timing
   - Adjust support polygon calculations
   - Tune gait parameters for stability

## General Troubleshooting Techniques

### System-Level Issues

#### High CPU or GPU Usage
**Symptoms**: System resources are maxed out, poor performance

**Solutions**:
1. **Monitor resource usage**:
   ```bash
   # GPU usage
   watch -n 0.1 nvidia-smi

   # CPU usage
   htop

   # Memory usage
   free -h
   ```

2. **Optimize node performance**:
   - Reduce node update frequencies
   - Lower image resolutions where possible
   - Use more efficient algorithms or data structures

3. **Check for memory leaks**:
   - Monitor memory usage over time
   - Restart nodes if memory usage grows continuously
   - Use memory profiling tools if needed

#### Communication Issues
**Symptoms**: Nodes can't communicate, topics not available, TF issues

**Solutions**:
1. **Check ROS 2 configuration**:
   - Verify ROS domain ID is consistent
   - Check ROS environment variables
   - Ensure nodes are on same network if using multi-machine setup

2. **Monitor topic connectivity**:
   ```bash
   ros2 topic list
   ros2 topic info /topic_name
   ros2 node info /node_name
   ```

3. **Check for message rate issues**:
   - Verify publishers are publishing at expected rates
   - Check for dropped messages
   - Adjust queue sizes if needed

### Debugging Strategies

#### Enable Detailed Logging
```bash
# Launch with detailed logging
ros2 run package_name node_name --ros-args --log-level debug

# Or set in launch file
import logging
logging.getLogger('rclpy').setLevel(logging.DEBUG)
```

#### Use RViz for Visualization
- Add displays for sensor data (images, point clouds, laserscan)
- Visualize TF frames and transforms
- Display costmaps and path planning results
- Monitor robot state and navigation status

#### Use Command Line Tools
```bash
# Check system status
ros2 lifecycle list
ros2 param list
ros2 action list

# Monitor specific topics
ros2 topic hz /topic_name
ros2 topic echo /topic_name

# Check service availability
ros2 service list
ros2 service call /service_name
```

## Common Error Messages and Solutions

### Isaac Sim Errors
- **"Failed to initialize GPU"**: Check GPU drivers and CUDA installation
- **"Insufficient VRAM"**: Reduce scene complexity or use higher-end GPU
- **"Extension failed to load"**: Check Isaac Sim logs for specific errors

### Isaac ROS Errors
- **"Could not load TensorRT engine"**: Verify engine file exists and is compatible
- **"No valid input data"**: Check sensor data availability and format
- **"CUDA error"**: Verify GPU compatibility and driver installation

### Nav2 Errors
- **"No valid plan found"**: Check map, localization, and costmap configuration
- **"Controller failed to follow path"**: Adjust controller parameters and tolerances
- **"Lifecycle node not active"**: Use lifecycle manager to activate nodes

## Performance Optimization Tips

### Isaac Sim Performance
- Reduce scene complexity for better performance
- Use lower resolution textures when possible
- Optimize lighting calculations
- Adjust physics simulation parameters

### Isaac ROS Performance
- Use appropriate image resolutions for processing
- Optimize TensorRT engine parameters
- Use efficient data structures and algorithms
- Consider multi-threading for parallel processing

### Nav2 Performance
- Optimize costmap update frequencies
- Use appropriate map resolutions
- Adjust planning and control frequencies
- Consider hierarchical planning approaches

## Getting Help

### Documentation Resources
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages.html)
- [Navigation2 Documentation](https://navigation.ros.org/)

### Community Resources
- [NVIDIA Developer Forums](https://forums.developer.nvidia.com/)
- [ROS Answers](https://answers.ros.org/)
- [Isaac ROS GitHub Issues](https://github.com/NVIDIA-ISAAC-ROS)

### Debugging Checklist
1. Check all hardware requirements are met
2. Verify software installations are complete
3. Ensure ROS environment is properly sourced
4. Confirm all required nodes are running
5. Validate parameter configurations
6. Check TF tree completeness
7. Monitor resource usage
8. Review error logs for specific issues

This troubleshooting guide helps students resolve common issues they may encounter while working with the Isaac AI Brain module, ensuring they can continue learning without getting stuck on technical problems.