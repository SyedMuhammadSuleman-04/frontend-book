---
sidebar_position: 2
---

# Nav2 Path Planning for Humanoid Robots

This section covers adapting the ROS 2 Navigation (Nav2) system for humanoid robot navigation. Unlike wheeled robots, humanoid robots have unique constraints related to balance, gait, and bipedal locomotion that require custom path planning approaches.

## Understanding Humanoid Navigation Constraints

Humanoid robots face several unique challenges in navigation:

### Balance Constraints
- **Center of Mass**: Must remain within support polygon during movement
- **Stability Margins**: Require larger safety margins than wheeled robots
- **Dynamic Balance**: Need continuous balance control during locomotion

### Gait Constraints
- **Foot Placement**: Must follow specific patterns for stable walking
- **Step Sequencing**: Each step must be carefully planned for balance
- **Timing Requirements**: Gait cycles must be synchronized with navigation

### Physical Constraints
- **Limited Turning**: Cannot turn in place like wheeled robots
- **Step Height**: Limited obstacle climbing capability
- **Ground Clearance**: Must maintain appropriate foot clearance

## Nav2 Architecture for Humanoids

### Custom Plugins Required
Nav2 for humanoid robots requires several custom plugins:

#### Global Planner
- **HumanoidAStar**: A* planner adapted for bipedal constraints
- **FootstepPlanner**: Generates feasible footstep sequences
- **BalanceAwarePlanner**: Considers balance constraints in path planning

#### Local Planner
- **HumanoidController**: Custom controller for bipedal movement
- **GaitController**: Manages gait patterns during navigation
- **BalanceController**: Maintains balance during movement

#### Costmap Layers
- **BalanceCostmap**: Adds balance-related costs to navigation
- **StepHeightLayer**: Considers step height limitations
- **StabilityLayer**: Evaluates stability of potential paths

## Setting Up Nav2 for Humanoids

### Configuration Files

```yaml
# humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: False
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    set_initial_pose: False
    tf_broadcast: True
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Note: Customize behavior tree for humanoid navigation
    default_nav_through_poses_bt_xml: "humanoid_nav_through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "humanoid_nav_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_consistent_localization_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    # Custom humanoid controller
    plugin_names: ["HumanoidController"]
    plugin_types: ["nav2_mppi_controller::MPPIController"]
    HumanoidController:
      # Humanoid-specific parameters
      frequency: 20.0
      time_steps: 25
      control_time_step: 0.05
      vx_std: 0.1
      vy_std: 0.05
      wz_std: 0.1
      vx_max: 0.5
      vx_min: -0.2
      vy_max: 0.3
      vy_min: -0.3
      wz_max: 0.5
      iteration_count: 100
      best_n: 50
      model_dt: 0.05
      aux_dt: 0.01
      transform_tolerance: 0.1
      penalty_velocity_tracking: 2.5
      penalty_collision: 10.0
      penalty_goal: 2.0
      penalty_goal_angle: 0.5
      penalty_path: 2.0
      penalty_footstep: 5.0
      penalty_balance: 8.0
      threshold_to_consider_goal_reached: 0.25
      threshold_to_consider_front_steer_angle: 0.25
      goal_check_tolerance: 0.25
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      debug_cost_function: False
      publish_cost_grid_pc: False
      transform_timeout: 0.1
      regularization_weight: 0.01
      command_topic: "/cmd_vel"
      velocity_scaling_factor: 1.0
      # Humanoid-specific parameters
      max_step_length: 0.3  # Maximum step length for bipedal locomotion
      min_step_length: 0.1  # Minimum step length
      step_height: 0.05     # Foot clearance height
      balance_margin: 0.1   # Safety margin for balance
```

### Custom Controller Implementation

```python
# humanoid_controller.py
import rclpy
from rclpy.node import Node
from nav2_core.controller import Controller
from nav2_util import lifecycle_node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration
import numpy as np

class HumanoidController(Controller):
    def __init__(self, name):
        super().__init__(name)
        self.logger = self.get_logger()
        self.max_step_length = 0.3  # meters
        self.step_height = 0.05     # meters
        self.balance_margin = 0.1   # meters
        self.gait_pattern = "walk"  # Current gait pattern

    def configure(self, plugin_name, node, tf, costmap_ros):
        """Configure the controller with parameters"""
        self.logger.info(f"Configuring {plugin_name}")
        self.node = node
        self.tf = tf
        self.costmap_ros = costmap_ros
        self.costmap = costmap_ros.get_costmap()

    def cleanup(self):
        """Clean up resources"""
        pass

    def activate(self):
        """Activate the controller"""
        self.logger.info("Activating HumanoidController")

    def deactivate(self):
        """Deactivate the controller"""
        pass

    def setPlan(self, path):
        """Set the global plan for the controller"""
        self.global_plan = path
        self.logger.info(f"Received plan with {len(path.poses)} poses")

    def computeVelocityCommands(self, pose, velocity):
        """Compute velocity commands for humanoid navigation"""
        # Calculate desired velocity based on path following
        cmd_vel = Twist()

        # Humanoid-specific constraints
        # Limit linear velocity for stable walking
        cmd_vel.linear.x = min(velocity.linear.x, 0.3)  # m/s
        cmd_vel.linear.y = min(velocity.linear.y, 0.1)  # m/s
        cmd_vel.angular.z = min(velocity.angular.z, 0.3)  # rad/s

        # Add balance constraints
        if self._isBalanceAtRisk(pose):
            cmd_vel.linear.x *= 0.5  # Reduce speed when balance is at risk
            cmd_vel.angular.z *= 0.3  # Reduce turning when unstable

        return cmd_vel, Duration()

    def _isBalanceAtRisk(self, pose):
        """Check if current pose puts balance at risk"""
        # Check if we're near obstacles that might affect balance
        robot_x = pose.pose.position.x
        robot_y = pose.pose.position.y

        # Get costmap coordinates
        map_x, map_y = self.costmap.worldToMap(robot_x, robot_y)
        cost = self.costmap.getCost(map_x, map_y)

        # If cost is high (near obstacles), balance might be at risk
        return cost > 100  # Adjust threshold as needed

    def isGoalReached(self, pose, velocity, goal):
        """Check if goal is reached with humanoid-specific tolerances"""
        # Calculate distance to goal
        dist_to_goal = np.sqrt(
            (pose.pose.position.x - goal.pose.position.x)**2 +
            (pose.pose.position.y - goal.pose.position.y)**2
        )

        # Check if within tolerance and velocity is low
        return dist_to_goal < 0.25 and abs(velocity.linear.x) < 0.05
```

## Footstep Planning

### Basic Footstep Planning Approach
For humanoid navigation, path planning must consider feasible footstep locations:

```python
class FootstepPlanner:
    def __init__(self):
        self.step_length = 0.3  # meters
        self.step_width = 0.2   # meters (lateral distance between feet)
        self.max_step_height = 0.1  # meters

    def planFootsteps(self, path, robot_pose):
        """Plan a sequence of footsteps from a path"""
        footsteps = []

        # Start with current foot positions
        left_foot = self._getLeftFootPosition(robot_pose)
        right_foot = self._getRightFootPosition(robot_pose)

        # Alternate footsteps along the path
        for i, waypoint in enumerate(path.poses):
            if i % 2 == 0:
                # Move left foot
                next_left = self._getNextFootPosition(waypoint, left_foot, right_foot)
                footsteps.append(('left', next_left))
                left_foot = next_left
            else:
                # Move right foot
                next_right = self._getNextFootPosition(waypoint, right_foot, left_foot)
                footsteps.append(('right', next_right))
                right_foot = next_right

        return footsteps

    def _getNextFootPosition(self, waypoint, current_foot, other_foot):
        """Calculate next foot position based on waypoint and current stance"""
        # Calculate direction toward waypoint
        dx = waypoint.pose.position.x - current_foot.x
        dy = waypoint.pose.position.y - current_foot.y
        distance = np.sqrt(dx*dx + dy*dy)

        # Normalize and scale to step length
        if distance > 0:
            scale = min(self.step_length, distance) / distance
            new_x = current_foot.x + dx * scale
            new_y = current_foot.y + dy * scale
        else:
            new_x, new_y = current_foot.x, current_foot.y

        return {'x': new_x, 'y': new_y, 'z': 0.0}
```

## Balance-Aware Path Planning

### Costmap Customization
Create custom costmap layers that consider balance requirements:

```yaml
# costmap_common_params.yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 10.0
  transform_tolerance: 0.5
  width: 40
  height: 40
  resolution: 0.05
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
    - {name: balance_layer, type: "nav2_humanoid_costmap::BalanceLayer"}
    - {name: step_height_layer, type: "nav2_humanoid_costmap::StepHeightLayer"}

local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 10.0
  transform_tolerance: 0.5
  width: 5.0
  height: 5.0
  resolution: 0.05
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
    - {name: balance_layer, type: "nav2_humanoid_costmap::BalanceLayer"}
    - {name: step_height_layer, type: "nav2_humanoid_costmap::StepHeightLayer"}
```

## Practical Exercise: Basic Humanoid Navigation

### Objective
Set up and test basic navigation for a humanoid robot in Isaac Sim.

### Steps
1. Configure Nav2 with humanoid-specific parameters
2. Launch Isaac Sim with humanoid robot model
3. Set up costmaps with balance constraints
4. Plan and execute a simple navigation task
5. Evaluate navigation performance and balance maintenance

### Expected Outcome
- Navigation system planning paths for humanoid robot
- Robot maintaining balance during navigation
- Successful goal reaching with stable locomotion

## Integration with Perception

### Using Perception for Navigation Safety
- **Obstacle Detection**: Integrate object detection for safe navigation
- **Terrain Analysis**: Use segmentation to identify traversable terrain
- **Dynamic Obstacle Avoidance**: Track moving objects for safe navigation
- **Costmap Updates**: Update costmaps based on perception data

## Troubleshooting Common Issues

### Navigation Instability
- Check balance constraints are properly configured
- Verify footstep planning is working correctly
- Adjust velocity limits for stable walking
- Validate gait parameters

### Path Planning Problems
- Ensure costmap layers are properly configured
- Check that balance margins are appropriate
- Verify step height constraints are realistic
- Validate that the global planner considers humanoid constraints

### Performance Issues
- Monitor computational requirements for real-time performance
- Optimize footstep planning algorithms
- Consider simplified balance models if needed
- Validate that perception pipeline doesn't create bottlenecks

## Next Steps

In the next section, we'll explore the specifics of bipedal movement and gait planning in detail, which is crucial for successful humanoid navigation.