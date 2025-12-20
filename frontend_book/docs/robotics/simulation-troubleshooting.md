# Simulation Troubleshooting Guide

This guide provides solutions to common issues encountered when working with Gazebo, Unity, and ROS 2 simulation environments.

## Common Gazebo Issues

### Gazebo Won't Start
**Symptoms**: Gazebo fails to launch or crashes immediately
**Solutions**:
1. Check GPU drivers are properly installed:
   ```bash
   # For NVIDIA
   nvidia-smi

   # For AMD
   sudo apt install mesa-vulkan-drivers
   ```
2. Try running with software rendering:
   ```bash
   gazebo --verbose --render-engine=ogre
   ```
3. Ensure sufficient system resources are available

### Simulation Runs Slowly
**Symptoms**: Low frame rate, poor real-time factor (< 0.5)
**Solutions**:
1. Reduce visual quality settings in Gazebo
2. Close unnecessary applications to free up system resources
3. Use simpler geometries (boxes instead of complex meshes) for testing
4. Limit the number of objects in the simulation environment
5. Adjust physics update rates based on simulation requirements

### Robot Falls Through Ground
**Symptoms**: Robot model falls through the ground plane
**Solutions**:
1. Verify the robot model has proper collision geometry defined
2. Check that the ground plane has sufficient thickness
3. Ensure proper mass and inertial properties are set in URDF/SDF

### Collision Detection Not Working
**Symptoms**: Objects pass through each other
**Solutions**:
1. Verify collision elements are properly defined in URDF/SDF
2. Check that collision detection is enabled in Gazebo
3. Ensure proper material properties are set

## Common Unity Issues

### Unity Hub Not Installing
**Symptoms**: Installation fails or Unity Hub doesn't launch
**Solutions**:
1. Check system requirements are met
2. Ensure sufficient disk space is available
3. Run installer as administrator (Windows) or with proper permissions (Linux)

### Unity Editor Performance Issues
**Symptoms**: Slow scene loading, laggy interface
**Solutions**:
1. Update graphics drivers
2. Close other resource-intensive applications
3. Adjust Unity Quality Settings to lower levels
4. Verify adequate RAM and VRAM availability

## Common ROS 2 Issues

### ROS 2 Nodes Can't Communicate
**Symptoms**: Nodes can't see each other, topics not connecting
**Solutions**:
1. Ensure all terminals have sourced ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Check if RMW implementation is consistent across terminals
3. Verify network configuration if using multi-machine setup

### Topic/Service Communication Failures
**Symptoms**: Messages not being received, services timing out
**Solutions**:
1. Use `ros2 topic list` and `ros2 service list` to verify availability
2. Check message/service definitions match between publisher and subscriber
3. Verify QoS settings are compatible

## Performance Optimization

### Gazebo Performance Tips
1. **Reduce visual complexity**: Use simple shapes for testing
2. **Adjust physics parameters**:
   - Lower max step size for faster simulation
   - Reduce real-time update rate if real-time performance isn't critical
   - Limit max contacts to reasonable values
3. **Optimize URDF models**: Simplify collision meshes, reduce joint count where possible

### Unity Performance Tips
1. **Use Level of Detail (LOD)**: Implement LOD groups for complex objects
2. **Optimize lighting**: Use baked lighting where possible instead of real-time
3. **Reduce draw calls**: Combine meshes and use atlasing for textures

### System-wide Optimization
1. **Monitor resource usage**: Use `htop` and `nvidia-smi` to monitor CPU/GPU usage
2. **Close unnecessary applications**: Free up RAM and CPU resources
3. **Use SSD storage**: For faster asset loading and simulation performance

## Debugging Simulation Issues

### Verifying Physics Parameters
```bash
# Check physics properties in Gazebo
ros2 service call /gazebo/set_physics_properties gazebo_msgs/srv/SetPhysicsProperties
```

### Monitoring Sensor Data
```bash
# Monitor LiDAR data
ros2 topic echo /robot/sensors/lidar sensor_msgs/msg/LaserScan

# Monitor camera data
ros2 topic echo /robot/sensors/camera/image_raw sensor_msgs/msg/Image

# Monitor IMU data
ros2 topic echo /robot/sensors/imu sensor_msgs/msg/Imu
```

### Checking Simulation State
```bash
# Check model states
ros2 topic echo /gazebo/model_states gazebo_msgs/msg/ModelStates

# Check robot state
ros2 topic echo /robot/state gazebo_msgs/msg/LinkStates
```

## Environment-Specific Issues

### Virtual Machine Issues
If running in a VM:
1. Enable hardware virtualization in BIOS/UEFI
2. Allocate sufficient resources (4+ cores, 8GB+ RAM)
3. Enable 3D acceleration in VM settings
4. Consider using native installation for better performance

### Container Issues
If using containers:
1. Ensure proper GPU passthrough for graphics acceleration
2. Verify network configuration for ROS 2 communication
3. Check file permissions for mounted volumes

## Validation Steps

### Physics Simulation Validation
1. Create a simple box model and verify it falls with gravity
2. Place objects on a surface and verify they don't fall through
3. Check that collision responses look physically realistic

### Sensor Data Validation
1. Verify sensor data topics are publishing
2. Check data formats match expected message types
3. Validate data ranges are within expected bounds

### Integration Validation
1. Confirm ROS 2 nodes can communicate with Gazebo
2. Verify Unity can connect to ROS 2 network (if applicable)
3. Test end-to-end data flow from simulation to visualization

## Getting Help

### Useful Commands
```bash
# Check ROS 2 environment
printenv | grep ROS

# List available packages
ros2 pkg list

# Check Gazebo version
gazebo --version

# Monitor system resources
htop
```

### Community Resources
- [ROS Answers](https://answers.ros.org/questions/)
- [Gazebo Answers](https://answers.gazebosim.org/questions/)
- [Unity Documentation](https://docs.unity3d.com/)
- [ROS Discourse](https://discourse.ros.org/)

## When to Seek Additional Help

Contact instructors or community if you encounter:
- Persistent crashes that don't match known issues
- Performance problems despite optimization attempts
- Integration issues between different simulation components
- Unexpected behavior that doesn't match documentation