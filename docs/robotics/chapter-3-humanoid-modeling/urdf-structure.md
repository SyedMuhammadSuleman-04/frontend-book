---
sidebar_label: 'URDF Structure'
title: 'URDF Structure'
---

# URDF Structure

This section covers the Unified Robot Description Format (URDF), which is the standard XML-based format for representing robot models in ROS. You'll learn how to structure robot models for humanoid robots, including links, joints, and other essential components.

## Learning Objectives

After completing this section, you will be able to:
- Understand the structure and components of URDF files
- Create basic robot models with links and joints
- Define visual and collision properties for robot components
- Structure URDF files for humanoid robots
- Validate URDF files for correctness

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its kinematic structure, visual appearance, and collision properties. URDF is essential for robot simulation, visualization, and control.

## Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

## Links: The Building Blocks of Robots

Links represent rigid parts of the robot. Each link can have visual, collision, and inertial properties.

### Link Components

1. **Visual**: Defines how the link appears in visualization tools
2. **Collision**: Defines the collision geometry for physics simulation
3. **Inertial**: Defines the physical properties for dynamics simulation

```xml
<link name="link_name">
  <!-- Visual properties -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Geometry definition -->
    </geometry>
    <material name="color">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>

  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Geometry definition -->
    </geometry>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Geometry Types

URDF supports several geometry types:

- **Box**: `<box size="x y z"/>`
- **Cylinder**: `<cylinder radius="r" length="l"/>`
- **Sphere**: `<sphere radius="r"/>`
- **Mesh**: `<mesh filename="path_to_mesh_file"/>`

## Joints: Connecting Links

Joints define the connections between links and specify how they can move relative to each other.

### Joint Types

1. **Fixed**: No movement allowed (welds two links together)
2. **Revolute**: Rotational movement around an axis
3. **Continuous**: Like revolute but unlimited rotation
4. **Prismatic**: Linear sliding movement
5. **Floating**: 6DOF movement (rarely used)
6. **Planar**: Movement on a plane

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Creating a Simple Robot Model

Here's a complete example of a simple robot with a base and a single rotating joint:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Rotating arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## URDF for Humanoid Robots

Humanoid robots have complex kinematic structures that require careful organization. Here's an example of a simplified humanoid upper body:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left shoulder -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <box size="0.15 0.15 0.15"/>
      </geometry>
      <material name="shoulder_color">
        <color rgba="0.4 0.4 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.15 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>
</robot>
```

## Materials and Colors

Materials can be defined once and reused across multiple links:

```xml
<material name="blue">
  <color rgba="0 0 0.8 1"/>
</material>

<material name="red">
  <color rgba="0.8 0 0 1"/>
</material>

<material name="white">
  <color rgba="1 1 1 1"/>
</material>
```

## Transmissions

Transmissions define how joints connect to actuators (motors):

```xml
<transmission name="arm_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="arm_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="arm_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Gazebo-Specific Extensions

For simulation in Gazebo, you can add Gazebo-specific tags:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

## Best Practices for URDF

1. **Naming conventions**: Use consistent, descriptive names
2. **Origin placement**: Place origins at logical locations (joint centers, geometric centers)
3. **Units**: Use meters for distances, kilograms for mass
4. **Validation**: Always validate URDF files using `check_urdf` tool
5. **Organization**: Structure complex robots hierarchically
6. **Inertial properties**: Calculate or estimate realistic inertial properties

## Validating URDF Files

To validate a URDF file, you can use the `check_urdf` command:

```bash
# Install urdfdom tools if not already installed
sudo apt-get install liburdfdom-tools

# Check the URDF file
check_urdf /path/to/your/robot.urdf
```

## Key Concepts Summary

- URDF is XML-based robot description format
- Links represent rigid parts of the robot
- Joints connect links and define movement constraints
- Visual, collision, and inertial properties define link behavior
- Humanoid robots require complex kinematic chains
- Proper naming and organization are essential for complex models

## Next Steps

The next section will explore how to create complete humanoid robot models with proper kinematic structures and how to validate these models for simulation and control.