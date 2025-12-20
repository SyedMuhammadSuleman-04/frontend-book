# Testing Collision Responses with Environmental Objects

This document provides comprehensive guidance on testing collision responses between robots and environmental objects in Gazebo simulation.

## Understanding Collision Testing

### Why Test Collision Responses?

Collision testing is essential for:
- **Safety Validation**: Ensuring robots don't damage themselves or environments
- **Behavior Verification**: Confirming realistic physical interactions
- **Performance Assessment**: Validating computational efficiency
- **Robustness Testing**: Identifying failure modes and edge cases
- **Transfer Learning**: Ensuring simulation results apply to real-world scenarios

### Types of Collision Tests

1. **Static Collision Tests**: Robot interacts with fixed environmental objects
2. **Dynamic Collision Tests**: Robot interacts with moving environmental objects
3. **Multi-Contact Tests**: Robot experiences multiple simultaneous contacts
4. **High-Impact Tests**: Robot experiences high-velocity or high-force collisions
5. **Edge Case Tests**: Unusual or unexpected collision scenarios

## Test Environment Setup

### Creating Test Environments

**Basic Test Arena**:
```xml
<world name="collision_test_arena">
  <include>
    <uri>model://ground_plane</uri>
  </include>
  <include>
    <uri>model://sun</uri>
  </include>

  <!-- Fixed obstacles -->
  <model name="wall_1">
    <pose>2 0 1 0 0 0</pose>
    <link name="wall_link">
      <collision name="wall_collision">
        <geometry>
          <box size="0.1 4 2"/>
        </geometry>
      </collision>
      <visual name="wall_visual">
        <geometry>
          <box size="0.1 4 2"/>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>1000</mass>  <!-- Static object -->
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
      </inertial>
    </link>
  </model>

  <!-- Moveable obstacles -->
  <model name="box_1">
    <pose>1 1 0.5 0 0 0</pose>
    <link name="box_link">
      <collision name="box_collision">
        <geometry>
          <box size="0.5 0.5 0.5"/>
        </geometry>
      </collision>
      <visual name="box_visual">
        <geometry>
          <box size="0.5 0.5 0.5"/>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>5</mass>
        <inertia ixx="0.208" ixy="0" ixz="0" iyy="0.208" iyz="0" izz="0.208"/>
      </inertial>
    </link>
  </model>
</world>
```

### Test Object Categories

**Static Obstacles**:
- Walls and barriers
- Fixed furniture
- Architectural elements
- Ground surfaces with different properties

**Dynamic Obstacles**:
- Moveable boxes and objects
- Other robots or agents
- Moving platforms
- Pendulums and oscillating objects

**Special Surface Types**:
- High-friction surfaces (rubber mats)
- Low-friction surfaces (ice, polished floors)
- Bouncy surfaces (trampolines, springs)
- Deformable surfaces (sand, cushions)

## Test Procedures

### 1. Basic Contact Testing

**Objective**: Verify that basic collision detection and response work correctly.

**Procedure**:
1. Position robot at a known distance from a static obstacle
2. Command robot to move toward the obstacle
3. Observe collision detection and stopping behavior
4. Measure the distance at which collision occurs
5. Verify that appropriate contact forces are generated

**Expected Results**:
- Robot stops before penetrating the obstacle
- Contact forces prevent penetration
- Robot maintains stability after collision
- No excessive jittering or oscillation

### 2. Impact Testing

**Objective**: Test robot behavior during high-velocity impacts.

**Procedure**:
1. Position robot at a distance from an obstacle
2. Apply high-velocity commands to create impact
3. Measure impact forces and robot response
4. Document any bouncing, sliding, or tipping behavior
5. Verify that robot recovers to stable state

**Expected Results**:
- Robot experiences appropriate impact forces
- No penetration of obstacles
- Robot maintains structural integrity
- Recovery to stable state within reasonable time

### 3. Sliding Friction Testing

**Objective**: Test robot interaction with surfaces of different friction properties.

**Procedure**:
1. Create inclined planes with different friction coefficients (0.1, 0.5, 1.0)
2. Place robot on each surface
3. Apply forward motion commands
4. Measure actual vs. commanded motion
5. Document sliding vs. rolling behavior

**Expected Results**:
- Robot slides more on low-friction surfaces
- Robot maintains grip on high-friction surfaces
- Motion corresponds to friction coefficient
- No unexpected behaviors or instabilities

### 4. Multi-Contact Testing

**Objective**: Test robot behavior with multiple simultaneous contacts.

**Procedure**:
1. Position robot to contact multiple obstacles simultaneously
2. Apply various motion commands
3. Monitor all contact points and forces
4. Verify that robot maintains stability
5. Document force distribution across contacts

**Expected Results**:
- Multiple contacts are properly detected
- Forces are distributed appropriately
- Robot maintains balance with multiple contacts
- No numerical instabilities or errors

### 5. Corner and Edge Testing

**Objective**: Test robot behavior with complex geometric contacts.

**Procedure**:
1. Create corner configurations (L-shaped obstacles)
2. Position robot to contact edges and corners
3. Apply various approach angles
4. Monitor contact force generation
5. Document any unusual behaviors

**Expected Results**:
- Contacts are properly detected at edges/corners
- Appropriate forces prevent penetration
- No numerical artifacts or instabilities
- Realistic behavior at geometric discontinuities

## Quantitative Validation Methods

### Force Measurement

**Contact Force Monitoring**:
```bash
# Monitor contact forces in Gazebo
gz topic -e /gazebo/events/contact

# For ROS 2 integration
ros2 topic echo /robot/contact_sensors geometry_msgs/msg/WrenchStamped
```

**Force Validation Criteria**:
- Contact forces oppose penetration
- Force magnitude correlates with penetration depth
- Forces are applied at correct contact points
- No negative or unrealistic force values

### Position and Velocity Analysis

**Data Collection**:
1. Record robot position before, during, and after collision
2. Monitor robot velocity changes during impact
3. Measure time to stabilize after collision
4. Compare with theoretical physics predictions

**Validation Metrics**:
- Position accuracy: Robot stops at expected distance from obstacle
- Velocity changes: Consistent with momentum conservation
- Stabilization time: Reasonable recovery period
- Energy conservation: Appropriate energy loss during inelastic collisions

### Penetration Analysis

**Measurement Techniques**:
1. Monitor minimum distance between robot and obstacle
2. Check for negative distances (indicating penetration)
3. Measure penetration depth if it occurs
4. Document frequency and severity of penetrations

**Acceptable Penetration**:
- Should be minimal (typically < 1mm)
- Should resolve quickly after initial contact
- Should not cause simulation instabilities
- Should be consistent with soft contact parameters

## Test Scenarios

### Scenario 1: Wall Collision Test

**Setup**:
- Robot approaches a wall at 0.5 m/s
- Wall has high friction (μ=1.0) and low restitution (0.1)
- Robot has 4 contact points (feet)

**Procedure**:
1. Command robot to walk straight toward wall
2. Monitor approach velocity
3. Record collision time and forces
4. Measure stopping distance
5. Verify robot stability after collision

**Metrics**:
- Collision detection time: Within 1-2 time steps
- Stopping distance: < 2cm from wall
- Contact forces: Appropriate magnitude for robot mass and velocity
- Stability: Robot remains upright after collision

### Scenario 2: Box Pushing Test

**Setup**:
- Moveable box (5kg) on flat surface
- Robot approaches box to push it
- Various friction coefficients between robot feet and ground

**Procedure**:
1. Robot approaches box at controlled velocity
2. Robot contacts box and applies pushing force
3. Monitor box movement and robot stability
4. Measure force required to move box
5. Document robot gait changes during pushing

**Metrics**:
- Box starts moving at expected force threshold
- Robot maintains balance during pushing
- Force application is smooth and controlled
- Energy transfer is physically realistic

### Scenario 3: Stair Navigation Test

**Setup**:
- Series of steps with known height (0.1m each)
- Robot attempts to climb stairs
- Various approach angles and velocities

**Procedure**:
1. Robot approaches stairs at different angles
2. Monitor foot placement on step edges
3. Record forces during step climbing
4. Measure success rate of stair navigation
5. Document any failures or instabilities

**Metrics**:
- Successful step climbing rate: >90%
- Foot placement accuracy: {'<'}2cm from step edge
- Contact forces: Appropriate for step height and robot weight
- Stability: Robot maintains balance during climbing

### Scenario 4: Doorway Navigation Test

**Setup**:
- Narrow doorway (just wider than robot)
- Robot must navigate through without collision
- Various approach angles and velocities

**Procedure**:
1. Robot approaches doorway from different angles
2. Monitor proximity to door frame
3. Record any contact forces
4. Measure navigation success rate
5. Document collision avoidance effectiveness

**Metrics**:
- Navigation success rate: >95% for reasonable approaches
- Minimum clearance: >2cm from door frame
- Contact forces: Minimal (only from collision avoidance)
- Navigation time: Efficient path planning

## Automated Testing Framework

### Test Script Example

```python
#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64

class CollisionTester:
    def __init__(self):
        rospy.init_node('collision_tester')

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=1)

        # Subscribers
        self.contact_sub = rospy.Subscriber('/robot/contacts', ContactsState, self.contact_callback)
        self.force_pub = rospy.Publisher('/collision_force', Float64, queue_size=1)

        # Test parameters
        self.test_distance = 2.0  # meters from obstacle
        self.approach_velocity = 0.5  # m/s
        self.test_duration = 10.0  # seconds

        # Data collection
        self.contact_times = []
        self.contact_forces = []
        self.test_results = {}

    def contact_callback(self, msg):
        """Process contact sensor data"""
        if len(msg.states) > 0:
            # Calculate total contact force
            total_force = 0
            for contact_state in msg.states:
                for wrench in contact_state.wrenches:
                    force_magnitude = np.sqrt(
                        wrench.force.x**2 + wrench.force.y**2 + wrench.force.z**2
                    )
                    total_force += force_magnitude

            self.contact_forces.append(total_force)
            self.contact_times.append(rospy.get_time())

            # Publish force for monitoring
            force_msg = Float64()
            force_msg.data = total_force
            self.force_pub.publish(force_msg)

    def approach_test(self):
        """Run approach collision test"""
        # Move robot toward obstacle
        cmd_vel = Twist()
        cmd_vel.linear.x = self.approach_velocity

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < self.test_duration:
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.1)

        # Stop robot
        cmd_vel.linear.x = 0
        self.cmd_vel_pub.publish(cmd_vel)

        # Analyze results
        self.analyze_results()

    def analyze_results(self):
        """Analyze collected test data"""
        if self.contact_forces:
            max_force = max(self.contact_forces)
            avg_force = sum(self.contact_forces) / len(self.contact_forces)

            self.test_results = {
                'max_contact_force': max_force,
                'avg_contact_force': avg_force,
                'contact_count': len(self.contact_forces),
                'first_contact_time': self.contact_times[0] if self.contact_times else None
            }

            rospy.loginfo(f"Test Results: {self.test_results}")
        else:
            rospy.logwarn("No contacts detected during test")

if __name__ == '__main__':
    tester = CollisionTester()
    rospy.sleep(1)  # Wait for connections
    tester.approach_test()
```

### Test Result Documentation

**Test Report Template**:
```
Collision Test Report - [Date]

Test Configuration:
- Robot Model: [Model Name]
- Environment: [World File]
- Physics Parameters: [Time Step, Solver, etc.]
- Test Scenario: [Description]

Test Results:
- Success Rate: [Percentage]
- Average Contact Force: [Value]
- Maximum Penetration: [Value]
- Stabilization Time: [Value]
- Performance Metrics: [RTF, CPU usage]

Issues Identified:
- [List any problems encountered]
- [Suggested fixes or improvements]

Recommendations:
- [Any configuration changes needed]
- [Additional testing required]
```

## Performance Validation

### Real-time Factor (RTF) Testing

**Procedure**:
1. Run collision tests with varying complexity
2. Monitor RTF during different phases
3. Document performance impact of collisions
4. Identify performance bottlenecks

**Acceptable RTF Values**:
- >0.8: Good performance
- 0.5-0.8: Acceptable but may need optimization
- {'<'}0.5: Poor performance, requires optimization

### Computational Load Analysis

**Metrics to Monitor**:
- CPU usage during collision-heavy periods
- Memory consumption
- Physics engine update frequency
- Contact solver iteration count

## Common Issues and Solutions

### Issue: Penetration Through Objects

**Symptoms**: Robot passes through obstacles

**Causes**:
- Time step too large
- Collision geometry missing or incorrect
- Mass/inertia properties wrong

**Solutions**:
- Reduce time step (0.001s or smaller)
- Verify collision geometry exists for all links
- Check mass and inertia values

### Issue: Excessive Jittering

**Symptoms**: Robot vibrates or oscillates during contact

**Causes**:
- Contact parameters too stiff
- Solver parameters inadequate
- Mass ratio problems

**Solutions**:
- Increase soft_erp (0.2-0.5)
- Decrease soft_cfm (0.000001)
- Adjust solver iterations (100-200)

### Issue: Unstable Multi-Contact

**Symptoms**: Robot becomes unstable with multiple contacts

**Causes**:
- Poor constraint solving
- Inappropriate contact parameters
- Complex contact geometry

**Solutions**:
- Increase solver iterations
- Adjust contact parameters
- Simplify collision geometry where possible

### Issue: Performance Degradation

**Symptoms**: RTF drops significantly during collisions

**Causes**:
- Too many simultaneous contacts
- Complex collision geometry
- Inefficient contact algorithms

**Solutions**:
- Simplify collision geometry
- Reduce unnecessary contact points
- Optimize physics parameters

## Validation Criteria

### Pass/Fail Criteria

**Collision Detection**:
- ✅ Objects do not pass through each other
- ✅ Contact forces prevent penetration
- ✅ Appropriate contact points generated

**Stability**:
- ✅ Robot maintains balance after collisions
- ✅ No excessive oscillation or jittering
- ✅ Recovery to stable state within 2 seconds

**Performance**:
- ✅ RTF > 0.8 during normal operation
- ✅ RTF > 0.5 during collision events
- ✅ No simulation crashes or instabilities

**Accuracy**:
- ✅ Collision responses match physical expectations
- ✅ Force magnitudes are realistic
- ✅ Energy conservation is maintained appropriately

## Advanced Testing Techniques

### Statistical Validation

Run multiple tests with random parameters:
- Random approach angles
- Random velocities
- Random obstacle positions
- Statistical analysis of success rates

### Stress Testing

**High-Frequency Collisions**:
- Rapid oscillation between objects
- Continuous contact scenarios
- Maximum computational load testing

**Extreme Parameters**:
- Very high velocities
- Very high masses
- Extreme friction coefficients

### Regression Testing

**Version Control**:
- Track physics parameter changes
- Maintain test result history
- Identify performance degradation over time

## Documentation and Reporting

### Test Documentation Template

```
Test Case: [Name]
Objective: [What is being tested]
Preconditions: [Required setup state]
Test Steps: [Detailed procedure]
Expected Results: [What should happen]
Actual Results: [What actually happened]
Pass/Fail: [Result]
Notes: [Additional observations]
```

### Performance Tracking

Maintain a performance log:
- Date and Gazebo version
- Physics parameters used
- RTF achieved
- Collision-specific metrics
- Any issues encountered

## Best Practices

### Test Design

1. **Comprehensive Coverage**: Test all possible collision scenarios
2. **Realistic Parameters**: Use values that reflect real-world conditions
3. **Incremental Complexity**: Start simple, increase complexity gradually
4. **Automated Testing**: Use scripts for consistent, repeatable tests
5. **Documentation**: Keep detailed records of all tests

### Parameter Validation

1. **Start Conservative**: Begin with stable parameters
2. **Gradual Adjustment**: Make small changes to identify optimal values
3. **Cross-Validation**: Compare results with theoretical expectations
4. **Peer Review**: Have others review test procedures and results
5. **Continuous Monitoring**: Regularly retest as models evolve

Collision testing is critical for ensuring that robot simulations behave realistically and safely. Proper testing validates that collision detection and response systems work correctly, enabling reliable simulation-to-reality transfer and safe robot development.