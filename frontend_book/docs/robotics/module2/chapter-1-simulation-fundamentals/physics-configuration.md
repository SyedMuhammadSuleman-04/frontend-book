# Physics Configuration for Humanoid Robot Models in Gazebo

This document provides detailed guidance on configuring physics parameters for humanoid robot models to achieve realistic behavior in Gazebo simulation.

## Understanding Physics Configuration Components

### Mass Properties

Mass properties are fundamental to realistic physics simulation. Each link in your robot model must have appropriate mass values that reflect the real-world equivalent.

**Guidelines for Mass Assignment**:
- Base/Torso: 10-50 kg for humanoid robots (depending on size)
- Head: 1-3 kg (typically 2-5% of total body mass)
- Arms: 2-8 kg each (upper arm heavier than lower arm)
- Legs: 5-15 kg each (upper leg heavier than lower leg)
- Feet: 0.5-2 kg each

**Example mass configuration**:
```xml
<link name="base_link">
  <inertial>
    <mass value="15.0"/>  <!-- 15 kg for torso -->
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <!-- Inertia values calculated below -->
  </inertial>
</link>
```

### Inertia Tensor

The inertia tensor describes how mass is distributed in a 3D object. For humanoid robots, it's crucial to calculate realistic inertia values.

**Inertia Calculation for Common Shapes**:

**Box (rectangular prism)** with mass m and dimensions (x, y, z):
- Ixx = m*(y² + z²)/12
- Iyy = m*(x² + z²)/12
- Izz = m*(x² + y²)/12

**Cylinder** with mass m, radius r, and height h:
- Ixx = Iyy = m*(3*r² + h²)/12
- Izz = m*r²/2

**Sphere** with mass m and radius r:
- Ixx = Iyy = Izz = 2*m*r²/5

**Example for cylindrical arm**:
```xml
<link name="upper_arm">
  <inertial>
    <mass value="3.0"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <!-- For cylinder: r=0.05m, h=0.3m, m=3.0kg -->
    <inertia ixx="0.01875" ixy="0.0" ixz="0.0"
             iyy="0.01875" iyz="0.0" izz="0.00375"/>
  </inertial>
</link>
```

### Center of Mass

The center of mass (COM) affects robot stability and balance. For humanoid robots:
- Torso COM should be slightly above the hip joint
- Limb COM should be roughly at the geometric center
- Overall robot COM should be within the support polygon for stability

## Collision and Visual Geometry

### Collision Geometry Best Practices

**Shape Selection**:
- Use simple primitive shapes when possible (boxes, cylinders, spheres)
- Approximate complex shapes with multiple simple shapes
- Ensure collision geometry completely encloses visual geometry
- Avoid very thin or elongated collision shapes

**Example collision setup**:
```xml
<link name="base_link">
  <!-- Collision geometry (simplified for performance) -->
  <collision name="base_collision">
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.3 0.6"/>  <!-- Simplified box instead of complex mesh -->
    </geometry>
  </collision>

  <!-- Visual geometry (detailed for rendering) -->
  <visual name="base_visual">
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://robot_description/meshes/base.dae"/>
    </geometry>
  </visual>
</link>
```

### Surface Properties

Surface properties define how objects interact during contact:

**Friction Parameters**:
- `mu` and `mu2`: Primary and secondary friction coefficients
- Typical values: 0.3-0.8 for rubber on concrete, 0.1-0.2 for ice
- Higher values provide better grip but may cause simulation instability

**Restitution (Bounciness)**:
- `restitution_coefficient`: 0 for no bounce, 1 for perfect bounce
- Typical values: 0.05-0.2 for most robot parts
- Lower values for non-bouncing robot components

**Example surface configuration**:
```xml
<collision name="foot_collision">
  <geometry>
    <box size="0.15 0.1 0.05"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>    <!-- High friction for good grip -->
        <mu2>0.8</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>  <!-- Minimal bounce -->
    </bounce>
  </surface>
</collision>
```

## Gazebo Physics Engine Configuration

### World Physics Parameters

Configure the physics engine in your world file:

```xml
<world name="humanoid_world">
  <physics name="ode" type="ode">
    <!-- Time stepping -->
    <max_step_size>0.001</max_step_size>      <!-- 1ms time step -->
    <real_time_factor>1</real_time_factor>    <!-- Real-time simulation -->
    <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz update -->

    <!-- Gravity -->
    <gravity>0 0 -9.8</gravity>               <!-- Earth gravity -->

    <!-- ODE solver settings -->
    <ode>
      <solver>
        <type>quick</type>                    <!-- Fast iterative solver -->
        <iters>100</iters>                    <!-- Solver iterations -->
        <sor>1.3</sor>                        <!-- Successive over-relaxation -->
      </solver>
      <constraints>
        <cfm>0.000001</cfm>                   <!-- Constraint Force Mixing -->
        <erp>0.2</erp>                        <!-- Error Reduction Parameter -->
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>
</world>
```

### Robot-Specific Physics Parameters

For humanoid robots with complex kinematics:

```xml
<gazebo reference="robot_base">
  <self_collide>false</self_collide>          <!-- Disable self-collision for base -->
  <gravity>true</gravity>                     <!-- Enable gravity -->
  <mu1>0.8</mu1>                             <!-- Friction coefficients -->
  <mu2>0.8</mu2>
  <fdir1>0 0 0</fdir1>                       <!-- Friction direction -->
  <max_vel>100.0</max_vel>                   <!-- Maximum contact correction -->
  <min_depth>0.001</min_depth>               <!-- Penetration depth -->
</gazebo>
```

## Joint Configuration for Physics

### Joint Dynamics

Proper joint configuration is essential for realistic humanoid movement:

```xml
<joint name="hip_joint" type="revolute">
  <parent link="base_link"/>
  <child link="thigh"/>
  <origin xyz="0 0.1 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>                        <!-- Rotation axis -->
  <limit lower="-0.5" upper="0.5" effort="200" velocity="2"/>  <!-- Joint limits -->
  <dynamics damping="5.0" friction="1.0"/>   <!-- Joint dynamics -->
</joint>
```

**Damping and Friction**:
- Damping: Resists motion (like shock absorbers), typical 1-10 N*s/m
- Friction: Static resistance, typically 0.1-2.0 N*m

### Joint Safety Limits

```xml
<limit
  lower="-1.57"      <!-- Lower position limit (radians) -->
  upper="1.57"       <!-- Upper position limit (radians) -->
  effort="200"       <!-- Maximum effort (N or N*m) -->
  velocity="2"       <!-- Maximum velocity (m/s or rad/s) -->
/>
```

## Practical Configuration Examples

### Humanoid Torso Configuration

```xml
<link name="torso">
  <inertial>
    <mass value="20.0"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <!-- Approximate as box: 0.3x0.2x0.4m -->
    <inertia ixx="0.333" ixy="0.0" ixz="0.0"
             iyy="0.583" iyz="0.0" izz="0.250"/>
  </inertial>

  <collision name="torso_collision">
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.4"/>
    </geometry>
  </collision>

  <visual name="torso_visual">
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://humanoid_robot/meshes/torso.dae"/>
    </geometry>
  </visual>
</link>
```

### Humanoid Leg Configuration

```xml
<link name="upper_leg">
  <inertial>
    <mass value="8.0"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>  <!-- COM at geometric center -->
    <!-- Approximate as cylinder: r=0.08m, h=0.4m -->
    <inertia ixx="0.107" ixy="0.0" ixz="0.0"
             iyy="0.107" iyz="0.0" izz="0.026"/>
  </inertial>

  <collision name="upper_leg_collision">
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.08" length="0.4"/>
    </geometry>
  </collision>
</link>

<link name="lower_leg">
  <inertial>
    <mass value="5.0"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <!-- Approximate as cylinder: r=0.06m, h=0.4m -->
    <inertia ixx="0.067" ixy="0.0" ixz="0.0"
             iyy="0.067" iyz="0.0" izz="0.009"/>
  </inertial>

  <collision name="lower_leg_collision">
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.06" length="0.4"/>
    </geometry>
  </collision>
</link>

<joint name="knee_joint" type="revolute">
  <parent link="upper_leg"/>
  <child link="lower_leg"/>
  <origin xyz="0 0 -0.4"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.356" effort="300" velocity="3"/>  <!-- 0 to 135 degrees -->
  <dynamics damping="10.0" friction="2.0"/>
</joint>
```

## Validation and Testing

### Physics Validation Checklist

1. **Gravity Test**:
   - Spawn robot in Gazebo
   - Verify it falls to ground and stabilizes
   - Check for excessive jittering or oscillation

2. **Mass Distribution Test**:
   - Apply external forces to robot
   - Verify realistic acceleration and movement
   - Check that robot doesn't tip over easily

3. **Collision Test**:
   - Place robot near obstacles
   - Verify collision detection works properly
   - Check for appropriate contact responses

4. **Stability Test**:
   - Place robot in standing position
   - Apply small perturbations
   - Verify robot maintains balance

### Performance Monitoring

Monitor these metrics during physics configuration:

**Real-time Factor (RTF)**:
- Target: Close to 1.0 for real-time performance
- If RTF {'<<'} 1.0: Consider increasing time step or simplifying geometry
- If RTF >> 1.0: System has excess capacity

**CPU Usage**:
- Monitor during simulation
- High usage may indicate inefficient physics setup

**Simulation Stability**:
- Watch for jittering, unrealistic movements, or explosions
- Adjust solver parameters if instability occurs

## Common Configuration Issues and Solutions

### Issue: Robot Jitters or Vibrates
**Cause**: Time step too large or conflicting constraints
**Solution**:
- Reduce time step to 0.001s or smaller
- Increase solver iterations
- Adjust ERP and CFM values

### Issue: Robot Falls Through Ground
**Cause**: Missing collision geometry or incorrect orientation
**Solution**:
- Verify collision elements exist for all relevant links
- Check that collision geometry is properly positioned
- Ensure ground plane exists in world file

### Issue: Robot Tips Over Easily
**Cause**: High center of mass or low base mass
**Solution**:
- Lower the center of mass
- Increase mass of base/foot links
- Widen the stance if appropriate

### Issue: Joint Moves Too Fast or Uncontrollably
**Cause**: High effort limits or low damping
**Solution**:
- Reduce effort limits in joint limits
- Increase damping values in joint dynamics
- Check controller parameters

## Advanced Configuration Topics

### Custom Gravity Fields

For special applications, you can create custom gravity effects:

```xml
<world name="special_gravity">
  <gravity>0.1 0 -9.7</gravity>  <!-- Slight horizontal component -->
</world>
```

### Contact Sensors

Add contact sensors to detect collisions:

```xml
<gazebo reference="foot_link">
  <sensor name="foot_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <contact>
      <collision>foot_collision</collision>
    </contact>
  </sensor>
</gazebo>
```

### Custom Physics Materials

Define custom surface properties:

```xml
<gazebo reference="foot_link">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
        <fdir1>0 0 1</fdir1>
      </ode>
      <torsional>
        <coefficient>1.0</coefficient>
        <use_patch_radius>false</use_patch_radius>
        <surface_radius>0.01</surface_radius>
      </torsional>
    </friction>
  </surface>
</gazebo>
```

## Best Practices Summary

1. **Start Simple**: Begin with basic shapes and add complexity gradually
2. **Realistic Masses**: Use real-world mass estimates for each component
3. **Proper Inertias**: Calculate inertia tensors based on actual geometry
4. **Conservative Friction**: Start with moderate friction values
5. **Validate Incrementally**: Test physics properties one at a time
6. **Monitor Performance**: Balance accuracy with computational efficiency
7. **Document Values**: Keep records of successful configurations for reuse

## Troubleshooting Quick Reference

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| Excessive jittering | Time step too large | Reduce to 0.001s |
| Robot falls through objects | Missing collision geometry | Add collision elements |
| Robot unstable | High COM or low friction | Adjust mass distribution, increase friction |
| Poor performance | Complex geometry or small time step | Simplify collision shapes, optimize time step |
| Unnatural movement | Incorrect mass/inertia | Recalculate physics properties |

This configuration guide provides the foundation for creating realistic humanoid robot simulations in Gazebo. Proper physics configuration is essential for effective simulation-based robot development and testing.