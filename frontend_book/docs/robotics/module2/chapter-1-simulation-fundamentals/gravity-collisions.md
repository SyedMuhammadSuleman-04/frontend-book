# Gravity and Collisions in Gazebo

This document covers the implementation and understanding of gravity effects and collision handling in Gazebo simulation, which are fundamental for realistic robot behavior.

## Gravity in Gazebo

### Understanding Gravity

Gravity is a fundamental force that affects all objects with mass in the simulation. In Gazebo, gravity is applied globally to all objects in the world unless specifically overridden.

**Earth's Gravity**: The default gravity setting in Gazebo is -9.8 m/s² in the Z direction, representing Earth's gravitational acceleration.

### Configuring Gravity

Gravity is defined in the world file's physics section:

```xml
<world name="default">
  <physics name="default_physics" type="ode">
    <gravity>0 0 -9.8</gravity>
    <!-- Other physics parameters -->
  </physics>
</world>
```

**Gravity Vector Components**:
- `X`: Gravity component in the X direction (typically 0)
- `Y`: Gravity component in the Y direction (typically 0)
- `Z`: Gravity component in the Z direction (typically -9.8 for Earth)

### Custom Gravity Scenarios

**Different Celestial Bodies**:
```xml
<!-- Mars gravity (~3.71 m/s²) -->
<gravity>0 0 -3.71</gravity>

<!-- Moon gravity (~1.62 m/s²) -->
<gravity>0 0 -1.62</gravity>

<!-- Zero gravity (space simulation) -->
<gravity>0 0 0</gravity>

<!-- Custom gravity (e.g., experimental) -->
<gravity>0 -2.5 -7.3</gravity>
```

### Effects of Gravity on Robots

Gravity affects robots in several ways:

1. **Weight**: Determines the force pressing the robot against surfaces
2. **Stability**: Influences robot balance and center of mass considerations
3. **Locomotion**: Affects walking, climbing, and manipulation capabilities
4. **Energy Consumption**: Influences motor loads and power requirements

### Gravity and Robot Design

When designing robots for simulation:

**Mass Distribution**:
- Lower center of mass improves stability
- Proper weight distribution prevents tipping
- Consider gravity's effect on different robot parts

**Joint Torques**:
- Motors must overcome gravitational forces
- Vertical movements require more torque than horizontal
- Consider gravity compensation in control systems

## Collision Detection

### Collision Geometry Basics

Collision geometry defines the shape used for collision detection. It's separate from visual geometry to optimize performance.

**Types of Collision Shapes**:

1. **Primitive Shapes** (Most Efficient):
   - Box: `<box size="x y z"/>`
   - Sphere: `<sphere radius="r"/>`
   - Cylinder: `<cylinder radius="r" length="l"/>`
   - Capsule: Not directly supported, approximated with cylinders

2. **Mesh Shapes** (Most Accurate but Slower):
   - Complex shapes using triangle meshes
   - Use for detailed collision requirements

### Defining Collision Geometry in URDF

```xml
<link name="robot_link">
  <!-- Collision element -->
  <collision name="collision_name">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Choose one geometry type -->
      <box size="0.1 0.1 0.1"/>
      <!-- OR -->
      <sphere radius="0.05"/>
      <!-- OR -->
      <cylinder radius="0.05" length="0.1"/>
      <!-- OR -->
      <mesh filename="package://robot_description/meshes/link_collision.stl"/>
    </geometry>
  </collision>

  <!-- Visual element (separate from collision) -->
  <visual name="visual_name">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://robot_description/meshes/link_visual.dae"/>
    </geometry>
  </visual>
</link>
```

### Collision Properties

**Collision Layers and Groups**:
- Similar to filtering in other physics engines
- Control which objects can collide with each other
- Useful for ignoring self-collisions in robot arms

**Surface Properties**:
```xml
<collision>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>        <!-- Primary friction coefficient -->
        <mu2>1.0</mu2>      <!-- Secondary friction coefficient -->
        <slip1>0.0</slip1>  <!-- Primary slip coefficient -->
        <slip2>0.0</slip2>  <!-- Secondary slip coefficient -->
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>
      <threshold>100000</threshold>
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>1000000000000.0</kp>
        <kd>1.0</kd>
        <max_vel>100.0</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

## Collision Response

### Contact Detection

Gazebo uses several algorithms for collision detection:

**Broad Phase**: Quickly eliminates pairs of objects that are far apart
**Narrow Phase**: Precisely determines if and where objects collide
**Contact Generation**: Creates contact points and calculates forces

### Contact Forces

When objects collide, Gazebo calculates forces based on:

**Penetration Depth**: How much objects overlap
**Material Properties**: Density, elasticity, friction
**Relative Velocity**: Speed and direction of collision
**Time Step**: Duration of collision interaction

### Friction Models

**Static Friction**: Prevents objects from starting to slide
- Must overcome static friction to initiate motion
- Typically higher than dynamic friction

**Dynamic Friction**: Opposes motion when objects are sliding
- Acts during sliding motion
- Usually lower than static friction

**Friction Coefficient (μ)**: Dimensionless value representing friction strength
- 0 = no friction (perfectly slippery)
- 1 = strong friction (typical for rubber on concrete)
- >1 = very high friction (special materials)

### Bounce and Restitution

**Restitution Coefficient**: Determines "bounciness" of collisions
- 0 = perfectly inelastic (no bounce)
- 1 = perfectly elastic (maximum bounce)
- Values > 1 create energy gain (unrealistic)

## Practical Collision Scenarios

### Robot-Environment Collisions

**Ground Contact**:
- Essential for walking robots
- Requires proper foot collision geometry
- Friction enables walking and prevents sliding

**Obstacle Avoidance**:
- Collision detection triggers avoidance behaviors
- Proper collision shapes prevent "ghost" collisions
- Sensor integration with collision detection

**Wall Following**:
- Accurate wall collision geometry needed
- Friction properties affect contact quality
- Sensor fusion with collision data

### Robot Self-Collisions

**Joint Limits**: Prevent robot from colliding with itself
**Collision Filtering**: Ignore specific self-collision pairs
**Safety Controllers**: Stop motion when self-collision detected

### Multi-Robot Collisions

**Communication**: Robots coordinate to avoid collisions
**Path Planning**: Collision-aware trajectory generation
**Formation Control**: Maintain safe distances

## Collision Detection Algorithms

### Gazebo's Approach

Gazebo uses the **Open Dynamics Engine (ODE)** and **Bullet Physics** for collision detection:

**ODE Features**:
- QuickStep solver for real-time performance
- Hash table for broad-phase collision detection
- Gauss-Seidel iterative solver for contacts

**Bullet Features**:
- More modern collision detection algorithms
- Better handling of complex geometries
- Support for soft body dynamics

### Performance Considerations

**Collision Complexity**:
- Simple shapes: Fastest collision detection
- Convex meshes: Moderate performance
- Concave meshes: Slowest, requires decomposition

**Optimization Strategies**:
- Use bounding boxes for initial collision checks
- Simplify collision geometry where precision isn't critical
- Adjust collision margins for performance vs. accuracy

## Implementing Gravity and Collision in Practice

### Complete Robot Link Example

```xml
<link name="robot_base">
  <inertial>
    <mass value="5.0"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/>
  </inertial>

  <collision name="base_collision">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.5 0.2"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>
          <mu2>0.8</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient>
      </bounce>
    </surface>
  </collision>

  <visual name="base_visual">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://robot_description/meshes/base_visual.dae"/>
    </geometry>
  </visual>
</link>
```

### World Configuration Example

```xml
<world name="robot_world">
  <!-- Physics engine configuration -->
  <physics name="ode" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    <gravity>0 0 -9.8</gravity>
    <ode>
      <solver>
        <type>quick</type>
        <iters>10</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>0.000001</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>

  <!-- Include ground plane -->
  <include>
    <uri>model://ground_plane</uri>
  </include>

  <!-- Include sun for lighting -->
  <include>
    <uri>model://sun</uri>
  </include>
</world>
```

## Common Issues and Solutions

### Gravity-Related Issues

**Robot Falls Through Ground**:
- **Cause**: Missing collision geometry or incorrect orientation
- **Solution**: Verify collision elements exist and are properly positioned

**Unrealistic Falling Speed**:
- **Cause**: Incorrect gravity value or mass properties
- **Solution**: Check gravity setting and mass values in URDF

**Robot Floats Above Ground**:
- **Cause**: Collision geometry not touching ground plane
- **Solution**: Adjust collision geometry or ground plane position

### Collision-Related Issues

**Jittery Movement**:
- **Cause**: Time step too large or conflicting constraints
- **Solution**: Reduce time step or adjust solver parameters

**Objects Pass Through Each Other**:
- **Cause**: Time step too large or insufficient collision detection
- **Solution**: Decrease time step or improve collision geometry

**Excessive Bouncing**:
- **Cause**: High restitution coefficient
- **Solution**: Reduce restitution coefficient in surface properties

**Performance Degradation**:
- **Cause**: Complex collision geometry or too many objects
- **Solution**: Simplify collision shapes or reduce object count

## Validation Techniques

### Gravity Validation

**Free Fall Test**:
1. Create a simple sphere object
2. Place it above ground plane
3. Measure fall distance over time
4. Verify acceleration matches 9.8 m/s²

**Pendulum Test**:
1. Create a simple pendulum model
2. Measure oscillation period
3. Compare with theoretical period: T = 2π√(L/g)

### Collision Validation

**Conservation of Momentum**:
1. Create two objects with known masses and velocities
2. Set up a collision scenario
3. Measure velocities before and after collision
4. Verify momentum conservation: m₁v₁ + m₂v₂ = m₁v₁' + m₂v₂'

**Friction Validation**:
1. Create an inclined plane with known angle
2. Place object with known friction coefficient
3. Verify static friction threshold: tan(θ) ≤ μ

## Advanced Collision Topics

### Soft Contacts

For more realistic contact handling:
- Soft CFM (Constraint Force Mixing) values
- ERP (Error Reduction Parameter) tuning
- Contact surface layer adjustments

### Multi-Contact Scenarios

When objects have multiple contact points:
- Proper force distribution
- Stability considerations
- Computational complexity

## Best Practices

### Gravity Configuration
1. **Use Standard Values**: Start with Earth's gravity (9.8 m/s²)
2. **Document Changes**: If using custom gravity, document the reason
3. **Consider Units**: Ensure consistent units throughout the model

### Collision Design
1. **Start Simple**: Begin with basic shapes, add complexity as needed
2. **Conservative Approach**: Ensure collision geometry completely encloses visual geometry
3. **Performance vs. Accuracy**: Balance detailed collision with simulation performance
4. **Test Thoroughly**: Validate collision behavior with simple test cases

### Validation
1. **Incremental Testing**: Test gravity and collisions separately before integration
2. **Physical Plausibility**: Ensure simulated behavior matches real-world expectations
3. **Performance Monitoring**: Track real-time factor and CPU usage

## Summary

Gravity and collision handling are fundamental to realistic robot simulation in Gazebo. Proper implementation of these concepts ensures that robots behave predictably and realistically in their simulated environments. Understanding how to configure gravity, design collision geometry, and validate collision responses is essential for creating effective simulation environments that accurately represent real-world physics.

The next section will cover hands-on exercises to apply these gravity and collision concepts in practical scenarios.