---
sidebar_position: 3
---

# Bipedal Movement and Gait Planning

This section focuses on the specifics of bipedal locomotion for humanoid robots in navigation. Unlike wheeled robots, humanoid robots must maintain balance through coordinated leg movements and gait patterns that ensure stable locomotion.

## Understanding Bipedal Locomotion

### Gait Fundamentals
Bipedal locomotion involves alternating leg movements with a support phase and a swing phase:

- **Double Support Phase**: Both feet on ground (at beginning and end of step)
- **Single Support Phase**: One foot on ground while other swings forward
- **Flight Phase**: Brief moment when both feet are off ground (in running)

### Balance Control
Humanoid robots must maintain balance through:
- **Center of Mass (CoM) Control**: Keep CoM within support polygon
- **Zero Moment Point (ZMP)**: Ensure no net moment at contact points
- **Capture Point**: Point where robot can come to stop safely

## Gait Planning for Navigation

### Walking Patterns
Different gait patterns for various navigation scenarios:

#### Normal Walk
- **Step Length**: 0.2-0.4m for most humanoid robots
- **Step Height**: 0.02-0.05m clearance
- **Step Width**: 0.1-0.2m between feet
- **Cadence**: 0.5-1.5 steps per second

#### Careful Walk
- **Step Length**: Reduced (0.1-0.2m)
- **Step Height**: Slightly increased (0.05-0.1m)
- **Cadence**: Slower (0.3-0.8 steps per second)
- **Use Case**: Uneven terrain, near obstacles

#### Fast Walk
- **Step Length**: Extended (0.3-0.6m)
- **Step Height**: Maintained (0.02-0.05m)
- **Cadence**: Faster (1.0-2.0 steps per second)
- **Use Case**: Open, safe areas

### Gait Generation Approaches

#### Inverted Pendulum Model
The simplest model for bipedal walking:

```python
class InvertedPendulumGait:
    def __init__(self, robot_height, gravity=9.81):
        self.robot_height = robot_height  # CoM height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / robot_height)

    def compute_footstep(self, current_com, current_com_vel, step_time):
        """Compute next footstep position using inverted pendulum model"""
        # Calculate where to place foot to stop the robot
        com_x, com_y = current_com
        vel_x, vel_y = current_com_vel

        # Capture point: where to step to stop
        capture_point_x = com_x + vel_x / self.omega
        capture_point_y = com_y + vel_y / self.omega

        # Place foot near capture point with safety margin
        foot_x = capture_point_x - 0.1  # Small safety margin
        foot_y = capture_point_y

        return (foot_x, foot_y)
```

#### Preview Control
More sophisticated approach using future trajectory planning:

```python
class PreviewControlGait:
    def __init__(self, robot_height, preview_horizon=10):
        self.robot_height = robot_height
        self.preview_horizon = preview_horizon
        self.omega = np.sqrt(9.81 / robot_height)

    def plan_trajectory(self, start_pos, goal_pos, dt=0.1):
        """Plan CoM trajectory using preview control"""
        # Simplified preview control implementation
        trajectory = []
        current_pos = np.array(start_pos)
        goal_pos = np.array(goal_pos)

        # Calculate reference trajectory
        for i in range(self.preview_horizon):
            t = i * dt
            # Simple interpolation toward goal
            ref_pos = current_pos + (goal_pos - current_pos) * (t / (self.preview_horizon * dt))
            trajectory.append(ref_pos)

        return trajectory
```

## Implementing Gait Controllers

### ROS 2 Gait Controller Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class GaitController(Node):
    def __init__(self):
        super().__init__('gait_controller')

        # Parameters
        self.declare_parameter('step_length', 0.3)
        self.declare_parameter('step_height', 0.05)
        self.declare_parameter('step_duration', 1.0)
        self.declare_parameter('stance_width', 0.2)

        self.step_length = self.get_parameter('step_length').value
        self.step_height = self.get_parameter('step_height').value
        self.step_duration = self.get_parameter('step_duration').value
        self.stance_width = self.get_parameter('stance_width').value

        # State variables
        self.left_foot_pos = Point(x=0.0, y=self.stance_width/2, z=0.0)
        self.right_foot_pos = Point(x=0.0, y=-self.stance_width/2, z=0.0)
        self.com_pos = Point(x=0.0, y=0.0, z=self.robot_height)

        # Publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.joint_cmd_pub = self.create_publisher(
            JointState, 'joint_commands', 10)
        self.footstep_pub = self.create_publisher(
            Float64MultiArray, 'footsteps', 10)

        # Timer for gait control
        self.timer = self.create_timer(0.05, self.gait_control_loop)

    def cmd_vel_callback(self, msg):
        """Handle velocity commands and plan gait"""
        # Convert linear/angular velocity to step plan
        self.desired_velocity = (msg.linear.x, msg.linear.y, msg.angular.z)

    def gait_control_loop(self):
        """Main gait control loop"""
        # Plan next footsteps based on desired velocity
        next_left_foot, next_right_foot = self.plan_next_step()

        # Generate joint trajectories for the steps
        joint_commands = self.generate_joint_trajectories(
            next_left_foot, next_right_foot)

        # Publish joint commands
        self.joint_cmd_pub.publish(joint_commands)

    def plan_next_step(self):
        """Plan the next footstep positions"""
        # Simple gait planning based on desired velocity
        vx, vy, wz = self.desired_velocity

        # Calculate where to place next foot
        if self.is_left_supporting():
            # Right foot should move forward
            next_right = Point()
            next_right.x = self.right_foot_pos.x + vx * self.step_duration
            next_right.y = self.right_foot_pos.y + vy * self.step_duration - self.stance_width/2
            next_right.z = 0.0  # Ground level
            return self.left_foot_pos, next_right
        else:
            # Left foot should move forward
            next_left = Point()
            next_left.x = self.left_foot_pos.x + vx * self.step_duration
            next_left.y = self.left_foot_pos.y + vy * self.step_duration + self.stance_width/2
            next_left.z = 0.0  # Ground level
            return next_left, self.right_foot_pos

    def is_left_supporting(self):
        """Determine which foot is currently supporting"""
        # Simplified: alternate steps
        return self.get_clock().now().nanoseconds % (2 * int(self.step_duration * 1e9)) < int(self.step_duration * 1e9)

    def generate_joint_trajectories(self, next_left, next_right):
        """Generate joint trajectories for the next step"""
        # This would involve inverse kinematics to move feet
        # to desired positions while maintaining balance
        joint_state = JointState()
        joint_state.name = ['left_hip_joint', 'right_hip_joint', 'left_knee_joint', 'right_knee_joint']
        joint_state.position = [0.0, 0.0, 0.0, 0.0]  # Placeholder

        return joint_state
```

## Balance Control Strategies

### Center of Mass Control

```python
class BalanceController:
    def __init__(self, robot_height):
        self.robot_height = robot_height
        self.com_x_filtered = 0.0
        self.com_y_filtered = 0.0
        self.filter_const = 0.1  # Low-pass filter constant

    def update_balance(self, current_com, desired_com):
        """Update balance control based on CoM position"""
        # Low-pass filter CoM position
        self.com_x_filtered = (1 - self.filter_const) * self.com_x_filtered + \
                             self.filter_const * current_com.x
        self.com_y_filtered = (1 - self.filter_const) * self.com_y_filtered + \
                             self.filter_const * current_com.y

        # Calculate balance error
        error_x = desired_com.x - self.com_x_filtered
        error_y = desired_com.y - self.com_y_filtered

        # Apply corrective actions
        if abs(error_x) > 0.05:  # 5cm threshold
            self.adjust_com_x(error_x)
        if abs(error_y) > 0.05:  # 5cm threshold
            self.adjust_com_y(error_y)

    def adjust_com_x(self, error):
        """Adjust CoM position in x direction"""
        # This would involve adjusting foot positions or body lean
        pass

    def adjust_com_y(self, error):
        """Adjust CoM position in y direction"""
        # This would involve adjusting stance width
        pass
```

## Adaptive Gait for Navigation

### Terrain Adaptation
Different gaits for different terrain types:

```python
class AdaptiveGaitController:
    def __init__(self):
        self.terrain_types = {
            'flat': {'step_length': 0.3, 'step_height': 0.02, 'speed': 0.5},
            'uneven': {'step_length': 0.15, 'step_height': 0.05, 'speed': 0.2},
            'stairs': {'step_length': 0.2, 'step_height': 0.15, 'speed': 0.1},
            'narrow': {'step_length': 0.2, 'step_height': 0.03, 'speed': 0.3, 'stance_width': 0.1}
        }
        self.current_terrain = 'flat'

    def adapt_gait(self, terrain_type, obstacle_distance):
        """Adapt gait based on terrain and obstacles"""
        if terrain_type in self.terrain_types:
            params = self.terrain_types[terrain_type]
            self.update_gait_parameters(params)

        # Adjust for nearby obstacles
        if obstacle_distance < 0.5:  # Less than 50cm
            self.reduce_step_length(0.5)  # Reduce by 50%
            self.increase_caution()

    def update_gait_parameters(self, params):
        """Update gait parameters based on terrain"""
        for param, value in params.items():
            setattr(self, param, value)
```

## Practical Exercise: Implement Gait Controller

### Objective
Implement a basic gait controller for humanoid navigation.

### Steps
1. Create a ROS 2 node for gait control
2. Implement inverted pendulum model for balance
3. Add footstep planning based on velocity commands
4. Test with simple navigation commands
5. Evaluate stability and balance maintenance

### Expected Outcome
- Stable bipedal locomotion
- Proper footstep placement
- Balance maintenance during movement
- Responsive to navigation commands

## Integration with Navigation Stack

### Coordination with Nav2
The gait controller must work with the Nav2 stack:

1. **Path Following**: Convert path waypoints to velocity commands
2. **Obstacle Avoidance**: Adjust gait when obstacles detected
3. **Recovery Behaviors**: Handle falls or balance loss
4. **Safety Monitoring**: Stop if balance cannot be maintained

### Safety Features
- **Fall Detection**: Monitor for balance loss
- **Emergency Stop**: Halt motion if unsafe
- **Recovery Actions**: Attempt to regain balance
- **Safe Shutdown**: Controlled stop if needed

## Troubleshooting Gait Issues

### Common Problems
- **Instability**: Adjust CoM control parameters
- **Tripping**: Increase foot clearance or reduce step length
- **Oscillation**: Tune balance controller gains
- **Slow Movement**: Optimize gait parameters for speed

### Performance Tuning
- **Balance Gains**: Adjust PID parameters for CoM control
- **Step Timing**: Optimize step duration for stability
- **Foot Trajectory**: Improve swing foot trajectory planning
- **Weight Distribution**: Adjust for optimal balance

## Next Steps

Now that you understand bipedal movement and gait planning, let's practice with exercises to solidify your understanding in the next section. These exercises will help you implement and test these concepts in simulation.