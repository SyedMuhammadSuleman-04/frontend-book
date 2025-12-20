# Collision Detection and Response in Gazebo

This document explains the principles and implementation of collision detection and response in Gazebo, which are essential for realistic robot simulation.

## Understanding Collision Detection

### What is Collision Detection?

Collision detection is the computational problem of detecting the intersection of two or more geometric objects in a 3D environment. In robotics simulation, it's crucial for:
- Preventing objects from passing through each other
- Generating realistic contact forces
- Enabling robot navigation and obstacle avoidance
- Simulating physical interactions

### Collision Detection Pipeline

Gazebo uses a multi-stage collision detection pipeline:

1. **Broad Phase**: Quickly eliminate pairs of objects that are far apart
2. **Narrow Phase**: Precisely determine if and where objects collide
3. **Contact Generation**: Create contact points and calculate forces
4. **Response Calculation**: Apply forces to simulate realistic physics

## Broad Phase Collision Detection

### Spatial Partitioning

To efficiently check for potential collisions among many objects, Gazebo uses spatial partitioning techniques:

**Axis-Aligned Bounding Box (AABB) Trees**:
- Each object is enclosed in a bounding box
- Hierarchical tree structure for efficient collision queries
- Fast overlap tests between bounding boxes

**Grid-Based Partitioning**:
- Space divided into a grid of cells
- Objects placed in relevant grid cells
- Only check objects in the same or adjacent cells

### Activation/Deactivation

Gazebo optimizes performance by:
- Deactivating objects that haven't moved recently
- Reactivating objects when forces are applied
- Only checking active objects in broad phase

## Narrow Phase Collision Detection

### Primitive Shape Collisions

Gazebo efficiently handles collisions between primitive shapes:

**Box-Box**: Optimized algorithm for rectangular objects
**Sphere-Sphere**: Simple distance check
**Sphere-Box**: Efficient closest-point calculation
**Cylinder-Cylinder**: More complex but optimized

### Mesh Collisions

For complex shapes, Gazebo supports:
- **Convex Decomposition**: Breaking complex meshes into convex parts
- **Triangle Mesh Collisions**: Direct mesh-to-mesh collision detection
- **Performance Trade-offs**: More accurate but slower than primitives

### Continuous Collision Detection (CCD)

For fast-moving objects, CCD prevents:
- **Tunneling**: Objects passing through each other due to large time steps
- **Missed Collisions**: High-velocity impacts not detected
- **Implementation**: Swept-volume collision detection

## Contact Generation

### Contact Points

When objects collide, Gazebo generates contact points:

**Point Generation**:
- Multiple points for stable contact
- Penetration depth calculation
- Normal and tangential force directions

**Contact Properties**:
- Position in world coordinates
- Normal vector (direction of collision)
- Depth of penetration
- Contact force magnitude

### Contact Manifolds

For surface-to-surface contacts:
- Multiple contact points form a manifold
- Provides more stable contact forces
- Better simulation of flat surface contacts

## Collision Response Physics

### Contact Force Calculation

Gazebo calculates contact forces using:

**Normal Forces**:
- Prevent objects from penetrating
- Proportional to penetration depth
- Calculated using spring-damper model

**Friction Forces**:
- Oppose tangential motion
- Limited by friction coefficient
- Static vs. dynamic friction models

### Constraint Solving

Contact constraints are solved using iterative methods:

**Gauss-Seidel Method**:
- Iteratively solves contact constraints
- Handles multiple simultaneous contacts
- Balances accuracy and performance

**Projected Gauss-Seidel**:
- Improved version with better convergence
- Handles friction cones more accurately
- Standard in most physics engines

## Configuration Parameters

### Contact Properties in URDF/SDF

```xml
<collision name="collision_element">
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
  <surface>
    <!-- Contact parameters -->
    <contact>
      <ode>
        <soft_cfm>0.000001</soft_cfm>    <!-- Soft constraint force mixing -->
        <soft_erp>0.2</soft_erp>         <!-- Soft error reduction parameter -->
        <kp>1000000000000.0</kp>         <!-- Spring stiffness -->
        <kd>1.0</kd>                     <!-- Damping coefficient -->
        <max_vel>100.0</max_vel>         <!-- Maximum correction velocity -->
        <min_depth>0.001</min_depth>     <!-- Minimum penetration depth -->
      </ode>
    </contact>

    <!-- Friction parameters -->
    <friction>
      <ode>
        <mu>0.8</mu>                    <!-- Primary friction coefficient -->
        <mu2>0.8</mu2>                  <!-- Secondary friction coefficient -->
        <fdir1>0 0 1</fdir1>            <!-- Friction direction -->
      </ode>
    </friction>

    <!-- Bounce parameters -->
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>
      <threshold>100000</threshold>      <!-- Minimum velocity for bounce -->
    </bounce>
  </surface>
</collision>
```

### Parameter Explanations

**soft_cfm (Constraint Force Mixing)**:
- Adds softness to constraints
- Higher values = more compliant contacts
- Default: ~0.000001 (very stiff)

**soft_erp (Error Reduction Parameter)**:
- Controls error correction speed
- Higher values = faster error correction
- Default: 0.2 (20% error correction per step)

**kp (Spring Stiffness)**:
- How stiff the contact spring is
- Higher values = less penetration
- Very high values can cause instability

**kd (Damping Coefficient)**:
- Controls contact damping
- Helps prevent oscillation
- Usually much smaller than kp

**max_vel (Maximum Velocity)**:
- Limits contact correction velocity
- Prevents excessive force application
- Default: 100 m/s (very high)

**min_depth (Minimum Depth)**:
- Allows small penetration for stability
- Prevents jittery contacts
- Default: 0.001 m (1 mm)

## Practical Collision Scenarios

### Robot-Environment Collisions

**Ground Contact**:
```xml
<collision name="foot_collision">
  <geometry>
    <box size="0.15 0.1 0.05"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>      <!-- High friction for grip -->
        <mu2>0.8</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <soft_erp>0.1</soft_erp>    <!-- Moderate error correction -->
        <soft_cfm>0.00001</soft_cfm> <!-- Somewhat compliant -->
      </ode>
    </contact>
  </surface>
</collision>
```

**Wall Collisions**:
```xml
<collision name="body_collision">
  <geometry>
    <box size="0.3 0.2 0.5"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.3</mu>      <!-- Moderate friction -->
        <mu2>0.3</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.05</restitution_coefficient> <!-- Low bounce -->
    </bounce>
  </surface>
</collision>
```

### Robot Self-Collisions

For preventing robot parts from colliding with each other:

```xml
<gazebo reference="robot_base">
  <self_collide>false</self_collide>    <!-- Disable self-collision for base -->
</gazebo>

<!-- Or selectively enable/disable for specific joints -->
<gazebo>
  <joint name="elbow_joint">
    <disable_collisions_between_children>true</disable_collisions_between_children>
  </joint>
</gazebo>
```

## Collision Detection Algorithms

### Gazebo's Physics Engines

**ODE (Open Dynamics Engine)**:
- Default physics engine for Gazebo
- Fast, stable for most robotics applications
- Good for rigid body simulation

**Bullet Physics**:
- More modern collision detection
- Better handling of complex meshes
- Supports soft body dynamics

**DART (Dynamic Animation and Robotics Toolkit)**:
- Advanced contact handling
- Better for complex kinematic chains
- More accurate for certain scenarios

### Algorithm Selection

Choose based on your needs:
- **ODE**: Best for general robotics simulation
- **Bullet**: Better for complex geometries
- **DART**: Advanced contact scenarios

## Performance Considerations

### Optimization Strategies

**Collision Geometry Simplification**:
- Use primitive shapes instead of complex meshes
- Reduce polygon count for mesh collisions
- Approximate complex shapes with multiple simple shapes

**Spatial Optimization**:
- Group static objects into compound shapes
- Use appropriate world bounding volumes
- Deactivate distant objects when possible

**Parameter Tuning**:
- Balance accuracy vs. performance
- Adjust solver parameters for your scenario
- Use appropriate time steps

### Performance Metrics

Monitor these during collision-heavy simulations:

**Real-time Factor (RTF)**:
- Target: Close to 1.0 for real-time performance
- Low RTF: Consider simplifying collision geometry

**Contact Count**:
- High contact counts can slow simulation
- Optimize geometry to reduce unnecessary contacts

**CPU Usage**:
- Monitor physics engine CPU consumption
- Adjust parameters if physics dominates CPU usage

## Advanced Collision Features

### Custom Collision Filters

Control which objects can collide:

```xml
<gazebo reference="link1">
  <collision>
    <surface>
      <contact>
        <collide_without_contact>false</collide_without_contact>
      </contact>
    </surface>
  </collision>
</gazebo>
```

### Contact Sensors

Add sensors to detect collisions:

```xml
<gazebo reference="foot_link">
  <sensor name="foot_contact_sensor" type="contact">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <contact>
      <collision>foot_collision</collision>
    </contact>
    <plugin name="contact_plugin" filename="libgazebo_ros_contact.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/contacts:=/robot/foot_contacts</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### Custom Contact Materials

Define material-specific properties:

```xml
<gazebo reference="rubber_wheel">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>        <!-- High friction for grip -->
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.3</restitution_coefficient>  <!-- Some bounce -->
    </bounce>
  </surface>
</gazebo>
```

## Troubleshooting Common Issues

### Issue: Objects Pass Through Each Other

**Possible Causes**:
- Time step too large
- Collision geometry missing or incorrect
- Objects moving too fast (tunneling)

**Solutions**:
- Reduce time step (e.g., 0.001s)
- Verify collision geometry exists and is properly defined
- Enable continuous collision detection if needed
- Check that objects have appropriate mass and inertia

### Issue: Excessive Jittering

**Possible Causes**:
- Contact parameters too stiff
- Solver parameters inadequate
- Mass ratios too extreme

**Solutions**:
- Increase soft_erp (e.g., 0.2-0.5)
- Decrease soft_cfm (e.g., 0.000001)
- Adjust solver iterations (increase to 100-200)
- Ensure realistic mass ratios between objects

### Issue: Robot Falls Through Ground

**Possible Causes**:
- Ground plane missing
- Robot collision geometry incorrect
- Gravity not applied properly

**Solutions**:
- Verify ground plane exists in world file
- Check robot has proper collision geometry
- Ensure mass and inertia are defined
- Verify gravity is enabled

### Issue: Poor Performance with Many Contacts

**Possible Causes**:
- Too many simultaneous contacts
- Complex collision geometry
- High solver iteration count

**Solutions**:
- Simplify collision geometry
- Reduce unnecessary contact points
- Optimize solver parameters
- Consider using simpler physics approximations

## Validation Techniques

### Contact Validation

**Visual Inspection**:
- Use Gazebo's contact visualization
- Enable contact force display
- Check contact point locations

**Quantitative Validation**:
- Measure contact forces
- Verify conservation of momentum
- Test collision responses against theoretical values

### Performance Validation

**Real-time Factor Testing**:
- Monitor RTF under various contact scenarios
- Test with different numbers of simultaneous contacts
- Validate that performance meets requirements

## Best Practices

### Collision Design

1. **Start Simple**: Use primitive shapes initially, add complexity later
2. **Conservative Geometry**: Ensure collision geometry completely encloses visual geometry
3. **Appropriate Mass**: Balance collision accuracy with computational efficiency
4. **Material Properties**: Use realistic friction and restitution values

### Parameter Selection

1. **Soft_erp**: Start with 0.2, adjust based on stability needs
2. **Soft_cfm**: Start with 0.000001, increase if experiencing instability
3. **Friction**: Use realistic values based on actual materials
4. **Restitution**: Keep low (0.05-0.2) for most robot applications

### Testing Strategy

1. **Incremental Testing**: Test collisions one scenario at a time
2. **Edge Cases**: Test high-speed and glancing collisions
3. **Stress Testing**: Test with many simultaneous contacts
4. **Validation**: Compare simulation results with expected physical behavior

## Advanced Topics

### Multi-Contact Scenarios

For objects with multiple contact points:
- Proper force distribution
- Stability considerations
- Computational complexity management

### Soft Contact Modeling

For more realistic contact handling:
- Adaptive stiffness parameters
- Pressure-dependent friction
- Deformation modeling

Collision detection and response form the foundation of realistic physics simulation in Gazebo. Proper configuration ensures that robots interact naturally with their environment while maintaining simulation stability and performance. Understanding these concepts is essential for creating effective robotics simulations.