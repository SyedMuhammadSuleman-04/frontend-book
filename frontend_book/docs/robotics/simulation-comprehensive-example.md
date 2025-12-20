# Comprehensive Simulation Example: Humanoid Robot in Digital Twin Environment

This comprehensive example demonstrates how to create a complete digital twin simulation environment combining Gazebo physics simulation, Unity virtual environments, and ROS 2 integration with sensor simulation.

## Overview

This example will guide you through creating a complete simulation scenario featuring:

- A humanoid robot model in Gazebo with physics properties
- A virtual environment in Unity with interactive elements
- Simulated sensors (LiDAR, camera, IMU) providing realistic data
- ROS 2 communication between all components
- Validation of simulation behavior against real-world expectations

## Prerequisites

Before starting this comprehensive example, ensure you have:

1. Completed all previous chapters in Module 2
2. ROS 2 Humble Hawksbill installed and configured
3. Gazebo Garden installed and tested
4. Unity 2022.3 LTS installed with Robotics package
5. Basic understanding of URDF/SDF and ROS 2 concepts

## Architecture Overview

The complete simulation system consists of:

```
Unity Environment ←→ ROS 2 Middleware ←→ Gazebo Physics Engine
       ↑                       ↑                    ↑
  Visual Feedback        Message Passing      Physics Simulation
       ↑                       ↑                    ↑
  Sensor Visualization  Sensor Data Flow    Robot Dynamics
```

## Step 1: Create the Humanoid Robot Model

### URDF Definition

First, create a simple humanoid robot model in URDF format:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_arm"/>
    <origin xyz="0.2 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_arm"/>
    <origin xyz="-0.2 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Sensors -->
  <!-- LiDAR sensor -->
  <link name="lidar_sensor">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_sensor"/>
    <origin xyz="0 0 0.05"/>
  </joint>

  <!-- Camera sensor -->
  <link name="camera_sensor">
    <visual>
      <geometry>
        <box size="0.04 0.04 0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.04 0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_sensor"/>
    <origin xyz="0.05 0 0"/>
  </joint>

  <!-- IMU sensor -->
  <link name="imu_sensor">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_sensor"/>
    <origin xyz="0 0 0.1"/>
  </joint>
</robot>
```

Save this as `simple_humanoid.urdf` in your project directory.

## Step 2: Configure Gazebo Simulation

### Gazebo World File

Create a simple world file to contain your simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include the default sun and ground plane -->
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add some obstacles for the robot to navigate around -->
    <model name="box1">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="box2">
      <pose>-2 1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixy>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Configure physics parameters -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

Save this as `simple_world.sdf`.

## Step 3: Add Sensor Plugins to URDF

Update your URDF to include Gazebo sensor plugins:

```xml
<!-- Add this inside the lidar_sensor link -->
<gazebo reference="lidar_sensor">
  <sensor name="lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=/robot/sensors/lidar</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>

<!-- Add this inside the camera_sensor link -->
<gazebo reference="camera_sensor">
  <sensor name="camera" type="camera">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=/robot/sensors/camera/image_raw</remapping>
        <remapping>camera_info:=/robot/sensors/camera/camera_info</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>

<!-- Add this inside the imu_sensor link -->
<gazebo reference="imu_sensor">
  <sensor name="imu" type="imu">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>100</update_rate>
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=/robot/sensors/imu</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Step 4: Launch the Simulation

Create a launch file to start the complete simulation:

```xml
<launch>
  <!-- Start Gazebo with the world -->
  <include file="$(find gazebo_ros)/launch/gazebo.launch.py">
    <arg name="world" value="$(find your_package)/worlds/simple_world.sdf"/>
  </include>

  <!-- Spawn the robot model -->
  <node name="spawn_urdf" pkg="gazebo_ros" exec="spawn_entity.py"
        args="-entity simple_humanoid -file $(find your_package)/urdf/simple_humanoid.urdf -z 1.0"/>

  <!-- Start robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" exec="robot_state_publisher"
        args="$(find your_package)/urdf/simple_humanoid.urdf"/>

  <!-- Start joint state publisher for the revolute joints -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" exec="joint_state_publisher">
    <param name="use_gui" value="true"/>
  </node>
</launch>
```

## Step 5: Unity Environment Setup

### Creating the Unity Scene

1. Open Unity Hub and create a new 3D project
2. Create a new scene and set up the basic environment:
   - Add a plane as the ground
   - Add cubes to represent the obstacles from the Gazebo world
   - Position them similarly to the Gazebo world (box1 at (2, 0, 0), box2 at (-2, 1, 0))

### Unity Robotics Integration

1. Install the Unity Robotics Hub package via Package Manager
2. Add the ROS TCP Connector to your scene
3. Create a script to receive robot pose data from ROS 2 and update the robot visualization in Unity

Example Unity C# script for ROS communication:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;

public class RobotVisualizer : MonoBehaviour
{
    ROSConnection ros;
    string robotPoseTopic = "/robot/pose";

    // Start is called before the first frame update
    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.instance;

        // Subscribe to the robot pose topic
        ros.Subscribe<OdometryMsg>(robotPoseTopic, UpdateRobotPose);
    }

    void UpdateRobotPose(OdometryMsg poseMsg)
    {
        // Update the robot's position and rotation in Unity
        transform.position = new Vector3(
            (float)poseMsg.pose.pose.position.x,
            (float)poseMsg.pose.pose.position.z, // Switch Y and Z for Unity coordinate system
            (float)poseMsg.pose.pose.position.y
        );

        transform.rotation = new Quaternion(
            (float)poseMsg.pose.pose.orientation.x,
            (float)poseMsg.pose.pose.orientation.z, // Switch Y and Z for Unity coordinate system
            (float)poseMsg.pose.pose.orientation.y,
            (float)poseMsg.pose.pose.orientation.w
        );
    }
}
```

## Step 6: Sensor Data Visualization

### LiDAR Data Visualization in Unity

Create a Unity script to visualize LiDAR data:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LidarVisualizer : MonoBehaviour
{
    ROSConnection ros;
    string lidarTopic = "/robot/sensors/lidar";
    GameObject[] lidarPoints;
    LaserScanMsg lidarData;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<LaserScanMsg>(lidarTopic, UpdateLidarData);

        // Create visualization points (limit to reasonable number for performance)
        lidarPoints = new GameObject[100]; // Visualize every 4th point from 360
        for (int i = 0; i < lidarPoints.Length; i++)
        {
            lidarPoints[i] = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            lidarPoints[i].transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            lidarPoints[i].GetComponent<Renderer>().material.color = Color.red;
            lidarPoints[i].SetActive(false);
        }
    }

    void UpdateLidarData(LaserScanMsg data)
    {
        lidarData = data;
        VisualizeLidar();
    }

    void VisualizeLidar()
    {
        if (lidarData == null) return;

        // Visualize every nth point to avoid too many objects
        int step = Mathf.Max(1, lidarData.ranges.Count / lidarPoints.Length);

        for (int i = 0; i < lidarPoints.Length; i++)
        {
            int idx = i * step;
            if (idx < lidarData.ranges.Count && lidarData.ranges[idx] < lidarData.range_max)
            {
                float angle = lidarData.angle_min + idx * lidarData.angle_increment;
                float distance = lidarData.ranges[idx];

                float x = distance * Mathf.Cos(angle);
                float y = distance * Mathf.Sin(angle);

                lidarPoints[i].transform.position = transform.position + new Vector3(x, 0.1f, y);
                lidarPoints[i].SetActive(true);
            }
            else
            {
                lidarPoints[i].SetActive(false);
            }
        }
    }
}
```

## Step 7: Validation and Testing

### Testing the Complete System

1. **Physics Validation**:
   - Launch Gazebo with the world and robot
   - Verify the robot falls to the ground due to gravity
   - Check that the robot doesn't pass through obstacles
   - Test joint movement by publishing to joint topics

2. **Sensor Validation**:
   - Monitor sensor topics to ensure they're publishing data:
     ```bash
     ros2 topic echo /robot/sensors/lidar
     ros2 topic echo /robot/sensors/camera/image_raw
     ros2 topic echo /robot/sensors/imu
     ```
   - Verify data formats match expected ROS 2 message types
   - Check that data values are within realistic ranges

3. **Unity Integration Validation**:
   - Launch Unity scene with ROS TCP Connector
   - Verify robot pose updates in Unity match Gazebo
   - Check that LiDAR visualization shows obstacles correctly

### Example Validation Commands

```bash
# Check if all required topics are available
ros2 topic list | grep robot

# Monitor LiDAR data
ros2 topic echo /robot/sensors/lidar --field ranges | head -n 5

# Monitor IMU data
ros2 topic echo /robot/sensors/imu --field orientation

# Check TF tree
ros2 run tf2_tools view_frames
```

## Step 8: Performance Optimization

### Gazebo Optimization
- Reduce physics update rate if real-time performance isn't critical
- Use simpler collision geometries for distant objects
- Limit the number of active sensors during development

### Unity Optimization
- Use Level of Detail (LOD) for complex objects
- Implement object pooling for LiDAR visualization points
- Use occlusion culling for better rendering performance

### ROS 2 Optimization
- Use appropriate QoS settings for different data types
- Implement message throttling for high-frequency sensors
- Use compression for image data if needed

## Troubleshooting Common Issues

### Robot Not Spawning in Gazebo
- Verify URDF file syntax and paths
- Check that all referenced meshes and materials exist
- Ensure Gazebo can parse the URDF correctly

### Sensor Data Not Publishing
- Verify sensor plugins are correctly configured in URDF
- Check that required Gazebo ROS packages are installed
- Ensure proper namespace remapping

### Unity-ROS Connection Issues
- Verify ROS TCP Connector IP address and port settings
- Check firewall settings if connecting across machines
- Ensure ROS 2 and Unity are on the same network

## Next Steps

After completing this comprehensive example, you should be able to:

1. Create complex simulation environments with multiple components
2. Integrate physics simulation with visualization
3. Implement sensor simulation with realistic data
4. Connect different simulation tools through ROS 2
5. Validate simulation behavior against real-world expectations

This example serves as a foundation for more complex digital twin implementations in your robotics projects.