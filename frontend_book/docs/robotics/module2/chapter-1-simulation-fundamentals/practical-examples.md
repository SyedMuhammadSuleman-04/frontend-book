# Practical Examples: Realistic Physics-Based Robot Behaviors

This document provides practical examples demonstrating realistic physics-based robot behaviors in Gazebo simulation.

## Example 1: Simple Pendulum Robot

### Objective
Demonstrate basic physics concepts using a simple pendulum model.

### Setup
Create a pendulum with a fixed joint to demonstrate oscillation:

```xml
<robot name="simple_pendulum">
  <link name="world"/>

  <link name="pendulum_ball">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="pendulum_joint" type="revolute">
    <parent link="world"/>
    <child link="pendulum_ball"/>
    <origin xyz="0 0 2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="0" velocity="0"/>
  </joint>
</robot>
```

### Physics Concepts Demonstrated
- **Gravity**: Ball falls due to gravitational acceleration
- **Pendulum Motion**: Oscillatory motion around equilibrium point
- **Energy Conservation**: Conversion between potential and kinetic energy
- **Damping**: Gradual energy loss due to friction

### Expected Behavior
- The pendulum should oscillate with decreasing amplitude
- Period should be approximately T = 2π√(L/g) where L=2m, g=9.8
- Eventually comes to rest at the lowest point

## Example 2: Wheeled Robot with Physics

### Objective
Create a simple wheeled robot that demonstrates ground contact and friction.

### Robot Model
```xml
<robot name="wheeled_robot">
  <!-- Robot body -->
  <link name="chassis">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Wheels -->
  <link name="left_wheel">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
  </link>

  <link name="right_wheel">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="chassis"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0.05" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="chassis"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0.05" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### Physics Concepts Demonstrated
- **Ground Contact**: Wheels make contact with ground plane
- **Friction**: Wheels grip surface to enable locomotion
- **Torque Application**: Joint forces cause wheel rotation
- **Stability**: Robot remains upright due to proper mass distribution

### Expected Behavior
- Robot should rest stably on the ground
- Wheels should contact ground without penetration
- When joint torques are applied, robot should move forward/backward

## Example 3: Balancing Robot

### Objective
Demonstrate balance and stability concepts with a simple inverted pendulum.

### Robot Model
```xml
<robot name="balance_robot">
  <!-- Base platform -->
  <link name="base">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Pole (inverted pendulum) -->
  <link name="pole">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.6"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint between base and pole -->
  <joint name="pole_joint" type="revolute">
    <parent link="base"/>
    <child link="pole"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
    <limit lower="-0.2" upper="0.2" effort="10" velocity="1"/>
  </joint>

  <!-- Actuator for balance control -->
  <transmission name="pole_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="pole_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="pole_actuator">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</robot>
```

### Physics Concepts Demonstrated
- **Center of Mass**: Pole has high center of mass for instability
- **Balance Control**: Requires active control to maintain upright position
- **Torque Control**: Effort control to counteract gravitational forces
- **Stability**: Feedback control to maintain equilibrium

### Expected Behavior
- Without control, pole will fall over due to instability
- With proper control input, pole can be balanced upright
- Small disturbances require corrective action

## Example 4: Multi-Link Manipulator

### Objective
Demonstrate complex kinematics with multiple interconnected links.

### Robot Model
```xml
<robot name="simple_arm">
  <!-- Base -->
  <link name="base">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Upper arm -->
  <link name="upper_arm">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Lower arm -->
  <link name="lower_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- End effector -->
  <link name="end_effector">
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>
  </link>

  <!-- Joints -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="lower_arm"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <joint name="wrist_joint" type="revolute">
    <parent link="lower_arm"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>
</robot>
```

### Physics Concepts Demonstrated
- **Multi-body Dynamics**: Interconnected rigid bodies
- **Joint Limits**: Constraints on motion range
- **Inertial Coupling**: Motion of one joint affects others
- **Load Distribution**: Forces transmitted through links

### Expected Behavior
- Each joint moves independently within limits
- Links move in coordinated fashion
- Proper mass distribution prevents excessive forces
- Realistic motion considering all dynamic effects

## Example 5: Collision and Contact Scenarios

### Objective
Demonstrate various collision and contact physics scenarios.

### Test Environment
Create a world file with different surface properties:

```xml
<sdf version="1.7">
  <world name="physics_test_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Different friction surfaces -->
    <model name="high_friction_block">
      <pose>-1 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
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

    <model name="low_friction_block">
      <pose>1 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.1</mu>
                <mu2>0.1</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
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

    <!-- Bouncy sphere -->
    <model name="bouncy_sphere">
      <pose>0 2 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
          <surface>
            <bounce>
              <restitution_coefficient>0.8</restitution_coefficient>
            </bounce>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.02</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.02</iyy>
            <iyz>0</iyz>
            <izz>0.02</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Physics Concepts Demonstrated
- **Friction Effects**: Different sliding behaviors
- **Restitution**: Bouncing vs. non-bouncing collisions
- **Energy Loss**: Damping effects in collisions
- **Surface Properties**: Material-specific interactions

### Expected Behavior
- Objects slide differently on high vs. low friction surfaces
- Bouncy sphere exhibits elastic collision behavior
- Energy gradually dissipates in all scenarios
- Contact forces prevent object penetration

## Validation and Testing

### Physics Validation Checklist

For each example, verify:

1. **Gravity Response**:
   - Objects fall with appropriate acceleration
   - No objects float or move upward without cause
   - Stable equilibrium positions are maintained

2. **Collision Detection**:
   - Objects don't pass through each other
   - Contact forces prevent penetration
   - Appropriate collision responses occur

3. **Stability**:
   - No excessive jittering or oscillation
   - Systems reach stable equilibrium
   - Numerical instabilities are absent

4. **Energy Conservation**:
   - Appropriate energy loss in inelastic collisions
   - No energy gain without external input
   - Damping effects are realistic

### Quantitative Measurements

1. **Pendulum Period**: T = 2π√(L/g)
2. **Free Fall Distance**: d = ½gt²
3. **Collision Momentum**: Verify conservation where applicable
4. **Stability Time**: Time to reach equilibrium

## Implementation Tips

### Model Design
- Start with simple shapes, add complexity gradually
- Ensure proper mass distribution
- Use realistic inertia values
- Verify all joints have appropriate limits

### Physics Tuning
- Begin with default parameters
- Adjust only when necessary
- Document all changes
- Test thoroughly after modifications

### Performance Optimization
- Use simple collision geometries
- Minimize unnecessary contact points
- Balance accuracy with performance
- Monitor simulation stability

These practical examples demonstrate fundamental physics concepts that are essential for creating realistic robot simulations in Gazebo. Each example builds on basic principles to show how complex behaviors emerge from proper physics configuration.