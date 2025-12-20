---
sidebar_label: 'Robot Modeling'
title: 'Robot Modeling'
---

# Robot Modeling

This section covers the practical aspects of creating complete humanoid robot models, including proper kinematic structures, realistic physical properties, and validation techniques. You'll learn to create models suitable for simulation and control applications.

## Learning Objectives

After completing this section, you will be able to:
- Design complete humanoid robot models with proper kinematic chains
- Define realistic physical properties for robot components
- Create modular robot models using xacro for complex structures
- Validate robot models for simulation and control
- Understand the relationship between robot models and control systems

## Complete Humanoid Robot Structure

A complete humanoid robot model typically includes these major components:

```
Base (usually pelvis or torso)
├── Head (with neck joint)
├── Left Arm Chain
│   ├── Left Shoulder
│   ├── Left Upper Arm
│   ├── Left Elbow
│   ├── Left Forearm
│   └── Left Hand
├── Right Arm Chain
│   ├── Right Shoulder
│   ├── Right Upper Arm
│   ├── Right Elbow
│   ├── Right Forearm
│   └── Right Hand
├── Left Leg Chain
│   ├── Left Hip
│   ├── Left Thigh
│   ├── Left Knee
│   ├── Left Shin
│   └── Left Foot
└── Right Leg Chain
    ├── Right Hip
    ├── Right Thigh
    ├── Right Knee
    ├── Right Shin
    └── Right Foot
```

## Building a Complete Humanoid Model

Here's an example of a more complete humanoid robot using URDF:

```xml
<?xml version="1.0"?>
<robot name="complete_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base/Pelvis Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.2"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.116666666667" ixy="0.0" ixz="0.0" iyy="0.154166666667" iyz="0.0" izz="0.0541666666667"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.5"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <mass value="15.0"/>
      <inertia ixx="0.395833333333" ixy="0.0" ixz="0.0" iyy="0.4375" iyz="0.0" izz="0.0541666666667"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="brown"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <visual>
      <origin xyz="0.05 0.15 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0.05 0.15 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.05 0.15 0.3" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.001666666667" ixy="0.0" ixz="0.0" iyy="0.001666666667" iyz="0.0" izz="0.001666666667"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.05 0.15 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <origin xyz="0.05 0.15 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0.05 0.15 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.05 0.15 0.1" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.00625" ixy="0.0" ixz="0.0" iyy="0.00625" iyz="0.0" izz="0.001875"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <!-- Additional links and joints for complete arm would continue similarly -->

  <!-- Right Arm (similar structure to left) -->
  <link name="right_shoulder">
    <visual>
      <origin xyz="0.05 -0.15 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0.05 -0.15 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.05 -0.15 0.3" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.001666666667" ixy="0.0" ixz="0.0" iyy="0.001666666667" iyz="0.0" izz="0.001666666667"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="0.05 -0.15 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="0.05 -0.15 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0.05 -0.15 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.05 -0.15 0.1" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.00625" ixy="0.0" ixz="0.0" iyy="0.00625" iyz="0.0" izz="0.001875"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <!-- Similar structures would be created for legs -->
</robot>
```

## Using Xacro for Complex Models

Xacro (XML Macros) simplifies the creation of complex URDF models by allowing macros, properties, and mathematical expressions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_humanoid">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_mass" value="10.0" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="arm_radius" value="0.05" />

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="side parent *origin">
    <!-- Shoulder link -->
    <link name="${side}_shoulder">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.1 0.1 0.1"/>
        </geometry>
        <material name="orange"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.1 0.1 0.1"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.001666666667" ixy="0.0" ixz="0.0" iyy="0.001666666667" iyz="0.0" izz="0.001666666667"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_shoulder"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
    </joint>

    <!-- Upper arm link -->
    <link name="${side}_upper_arm">
      <visual>
        <origin xyz="0 0 -${arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${arm_length}" radius="${arm_radius}"/>
        </geometry>
        <material name="orange"/>
      </visual>
      <collision>
        <origin xyz="0 0 -${arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${arm_length}" radius="${arm_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 -${arm_length/2}" rpy="0 0 0"/>
        <mass value="1.5"/>
        <inertia ixx="0.00625" ixy="0.0" ixz="0.0" iyy="0.00625" iyz="0.0" izz="0.001875"/>
      </inertial>
    </link>

    <joint name="${side}_elbow_joint" type="revolute">
      <parent link="${side}_shoulder"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.2"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="${torso_mass}"/>
      <inertia ixx="0.116666666667" ixy="0.0" ixz="0.0" iyy="0.154166666667" iyz="0.0" izz="0.0541666666667"/>
    </inertial>
  </link>

  <!-- Create arms using the macro -->
  <xacro:arm side="left" parent="base_link">
    <origin xyz="0.05 0.15 0.3" rpy="0 0 0"/>
  </xacro:arm>

  <xacro:arm side="right" parent="base_link">
    <origin xyz="0.05 -0.15 0.3" rpy="0 0 0"/>
  </xacro:arm>

</robot>
```

## Physical Properties and Inertial Calculations

For realistic simulation, accurate inertial properties are crucial. Here are the formulas for basic shapes:

### Box (width x depth x height)
```
ixx = (1/12) * m * (depth² + height²)
iyy = (1/12) * m * (width² + height²)
izz = (1/12) * m * (width² + depth²)
```

### Cylinder (radius r, height h, mass m)
```
ixx = (1/12) * m * (3*r² + h²)
iyy = (1/12) * m * (3*r² + h²)
izz = (1/2) * m * r²
```

### Sphere (radius r, mass m)
```
ixx = iyy = izz = (2/5) * m * r²
```

## Validation Techniques

### 1. URDF Validation
```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Visualize the robot in RViz
ros2 run rviz2 rviz2
# Then add RobotModel display and set the URDF file
```

### 2. Kinematic Validation
```bash
# Use robot state publisher to visualize
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat /path/to/robot.urdf)'
```

### 3. Simulation Validation
```bash
# Load robot in Gazebo simulation
ros2 launch gazebo_ros empty_world.launch.py
ros2 run gazebo_ros spawn_entity.py -file /path/to/robot.urdf -entity my_robot
```

## Common Modeling Mistakes and Solutions

### 1. Floating Base Issue
**Problem**: Robot falls through the ground in simulation
**Solution**: Add a fixed joint to world or use a floating joint with proper controllers

### 2. Incorrect Inertial Properties
**Problem**: Robot behaves unrealistically in simulation
**Solution**: Calculate inertial properties correctly using proper formulas

### 3. Self-Collision
**Problem**: Robot links collide with each other
**Solution**: Add collision-avoidance joints or adjust collision meshes

### 4. Kinematic Loop Issues
**Problem**: Complex structures with closed loops
**Solution**: Use proper constraint handling or simplify the structure

## Model Optimization

### 1. Collision Mesh Simplification
Use simpler geometries (boxes, cylinders) instead of complex meshes for collision detection.

### 2. Level of Detail
Use high-detail meshes for visualization and simplified meshes for collision detection.

### 3. Joint Limit Optimization
Set appropriate joint limits based on real robot capabilities to prevent damage.

## Tools for Robot Modeling

1. **SolidWorks/Blender/Fusion 360**: For creating 3D models
2. **SW2URDF/Blender2URDF**: For converting CAD models to URDF
3. **RViz**: For visualization and debugging
4. **Gazebo**: For simulation and testing
5. **MoveIt**: For motion planning validation

## Relationship to Control Systems

Robot models in URDF are essential for:
- **Forward Kinematics**: Calculating end-effector position from joint angles
- **Inverse Kinematics**: Calculating joint angles for desired end-effector position
- **Dynamics Simulation**: Predicting robot motion under forces
- **Motion Planning**: Finding collision-free paths
- **Control Design**: Implementing controllers based on robot dynamics

## Best Practices for Humanoid Modeling

1. **Start Simple**: Begin with basic shapes, add detail gradually
2. **Realistic Limits**: Set joint limits based on human or robot capabilities
3. **Consistent Units**: Use meters for distance, kg for mass
4. **Proper Mass Distribution**: Assign realistic masses to different body parts
5. **Validation Early**: Test models in simulation early in the design process
6. **Documentation**: Comment your URDF files for maintainability

## Key Concepts Summary

- Complete humanoid models require complex kinematic chains
- Xacro simplifies complex model creation with macros and properties
- Accurate inertial properties are essential for realistic simulation
- Proper validation ensures models work correctly in simulation and control
- Models must match real robot capabilities for effective control

## Next Steps

The next section will provide hands-on exercises to practice creating and validating humanoid robot models in URDF format.