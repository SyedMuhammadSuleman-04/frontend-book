# Nav2 Setup for Humanoid Robots

This guide provides instructions for setting up ROS 2 Navigation (Nav2) specifically for humanoid robot navigation, including custom plugins for bipedal locomotion.

## Understanding Humanoid Navigation Requirements

### Key Differences from Wheeled Robots
- **Balance Constraints**: Must maintain center of mass within support polygon
- **Gait Patterns**: Requires specific footstep sequences for stable walking
- **Turning Limitations**: Cannot turn in place like wheeled robots
- **Step Height**: Limited ability to step over obstacles
- **Stability Requirements**: Need larger safety margins for stability

### Nav2 Architecture for Humanoids
- **Custom Global Planner**: Humanoid-aware path planning
- **Custom Local Planner**: Bipedal movement controllers
- **Specialized Costmap Layers**: Balance and step height considerations
- **Footstep Planner**: Generates feasible footstep sequences

## Prerequisites

### System Requirements
- **ROS 2**: Humble Hawksbill distribution
- **Nav2**: Latest version compatible with Humble
- **Isaac Sim**: For simulation testing
- **Isaac ROS**: For perception integration
- **Humanoid Robot Model**: With proper URDF and controllers

### Installation Prerequisites
```bash
# Install Nav2 packages
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-nav2-gui-plugins
sudo apt install -y ros-humble-nav2-lifecycle-manager
sudo apt install -y ros-humble-dwb-core ros-humble-dwb-msgs
sudo apt install -y ros-humble-dwb-plugins ros-humble-nav-2d-utils
sudo apt install -y ros-humble-nav-cpp-tools ros-humble-nav-rotation-recovery
sudo apt install -y ros-humble-nav-smoother-lifecycle

# Install additional dependencies
sudo apt install -y ros-humble-angles ros-humble-costmap-queue
sudo apt install -y ros-humble-geometry2 ros-humble-geometry-msgs
sudo apt install -y ros-humble-nav-msgs ros-humble-tf2-geometry-msgs
```

## Installation and Setup

### Installing Nav2 for Humanoid Robots

1. **Create a workspace for Nav2 customization**:
   ```bash
   mkdir -p ~/humanoid_nav2_ws/src
   cd ~/humanoid_nav2_ws
   ```

2. **Clone necessary repositories**:
   ```bash
   # Clone Nav2 source for customization (optional)
   git clone -b humble https://github.com/ros-planning/navigation2.git src/navigation2

   # Or use pre-built packages (recommended for beginners)
   # The apt packages installed above are sufficient for basic use
   ```

3. **Build the workspace**:
   ```bash
   cd ~/humanoid_nav2_ws
   colcon build --packages-select nav2_bringup
   source install/setup.bash
   ```

### Custom Humanoid Navigation Packages

For advanced humanoid-specific features, create custom packages:

```bash
cd ~/humanoid_nav2_ws/src
# Create custom humanoid navigation package
ros2 pkg create --build-type ament_python humanoid_nav2 --dependencies rclpy nav2_common
```

## Configuration Files

### Basic Nav2 Configuration for Humanoids

#### Main Parameters File (humanoid_nav2_params.yaml)

```yaml
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

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

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

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

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

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3  # Adjust based on your humanoid robot
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "balance_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      balance_layer:
        plugin: "nav2_humanoid_costmap::BalanceLayer"
        enabled: True
        balance_margin: 0.2  # Additional margin for balance
        robot_width: 0.4     # Width of the humanoid robot
        robot_depth: 0.3     # Depth of the humanoid robot
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      robot_radius: 0.3  # Adjust based on your humanoid robot
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "balance_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      balance_layer:
        plugin: "nav2_humanoid_costmap::BalanceLayer"
        enabled: True
        balance_margin: 0.3  # Larger margin for global planning
        robot_width: 0.4
        robot_depth: 0.3
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific parameters
      step_size: 0.5  # Adjust for humanoid step capabilities
      min_distance_to_obstacles: 0.4  # Balance safety with navigation capability

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

recoveries_server:
  ros__parameters:
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/published_footprint"
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
      # Humanoid-specific recovery parameters
      min_duration: 5.0
      max_duration: 15.0
      velocity_scaling_factor: 0.5  # Slower for humanoid stability
    backup:
      plugin: "nav2_recoveries::BackUp"
      # Humanoid-specific backup parameters
      min_distance: 0.15
      max_distance: 0.3
      velocity: 0.05  # Very slow backup for stability
    wait:
      plugin: "nav2_recoveries::Wait"
      # Humanoid-specific wait parameters
      max_duration: 5.0

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

### Launch File for Humanoid Navigation

```python
# humanoid_nav2_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Input arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Launch configuration variables
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('humanoid_nav2'),
                                   'config', 'humanoid_nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    default_bt_xml_filename_arg = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(get_package_share_directory('nav2_bt_navigator'),
                                   'behavior_trees', 'humanoid_nav_to_pose_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    map_subscribe_transient_local_arg = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='false',
        description='Whether to set the map subscriber QoS to transient local')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Nodes launching commands
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server',
                                    'planner_server',
                                    'controller_server',
                                    'bt_navigator',
                                    'amcl',
                                    'velocity_smoother',
                                    'slam_toolbox']},  # Remove slam_toolbox if not using SLAM
                    ])

    # Map server is not needed if using SLAM
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # Planner server
    planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # Controller server
    controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # BT navigator
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # AMCL for localization
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # Velocity smoother
    velocity_smoother_cmd = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # Lifecycle manager for the costmap servers
    lifecycle_manager_costmap_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['local_costmap', 'global_costmap']}])

    # Local costmap
    local_costmap_cmd = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # Global costmap
    global_costmap_cmd = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # Recovery server (if needed)
    recovery_server_cmd = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # Waypoint follower (if needed)
    waypoint_follower_cmd = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace=namespace,
        output='screen',
        parameters=[configured_params])

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(autostart_arg)
    ld.add_action(params_file_arg)
    ld.add_action(default_bt_xml_filename_arg)
    ld.add_action(map_subscribe_transient_local_arg)

    # Launch Nav2 nodes
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(planner_server_cmd)
    ld.add_action(controller_server_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(velocity_smoother_cmd)
    ld.add_action(lifecycle_manager_costmap_cmd)
    ld.add_action(local_costmap_cmd)
    ld.add_action(global_costmap_cmd)
    ld.add_action(recovery_server_cmd)
    ld.add_action(waypoint_follower_cmd)

    return ld
```

## Custom Humanoid Plugins

### Creating a Basic Humanoid Controller

```python
#!/usr/bin/env python3
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

        # Return command with time horizon
        return cmd_vel, Duration()

    def _isBalanceAtRisk(self, pose):
        """Check if current pose puts balance at risk"""
        # Check if we're near obstacles that might affect balance
        robot_x = pose.pose.position.x
        robot_y = pose.pose.position.y

        # Get costmap coordinates
        try:
            map_x, map_y = self.costmap.worldToMap(robot_x, robot_y)
            cost = self.costmap.getCost(map_x, map_y)

            # If cost is high (near obstacles), balance might be at risk
            return cost > 100  # Adjust threshold as needed
        except Exception:
            # If conversion fails, assume safe
            return False

    def isGoalReached(self, pose, velocity, goal):
        """Check if goal is reached with humanoid-specific tolerances"""
        # Calculate distance to goal
        dist_to_goal = np.sqrt(
            (pose.pose.position.x - goal.pose.position.x)**2 +
            (pose.pose.position.y - goal.pose.position.y)**2
        )

        # Check if within tolerance and velocity is low
        return dist_to_goal < 0.25 and abs(velocity.linear.x) < 0.05

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidController("HumanoidController")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Isaac Sim

### Setting up Isaac Sim for Navigation Testing

1. **Prepare Isaac Sim Environment**:
   - Create or load a navigation testing environment
   - Ensure proper lighting and textures
   - Add navigation-relevant landmarks

2. **Configure Robot for Navigation**:
   - Ensure robot has proper sensors (LiDAR, IMU, cameras)
   - Verify coordinate frames are correct
   - Test basic movement capabilities

3. **Connect Isaac Sim to Nav2**:
   - Use Isaac ROS bridge to connect sensors
   - Ensure proper topic mapping
   - Verify TF tree is complete

## Testing and Validation

### Basic Navigation Test

1. **Launch Isaac Sim** with your humanoid robot
2. **Launch Nav2 stack**:
   ```bash
   ros2 launch humanoid_nav2 humanoid_nav2_launch.py use_sim_time:=true
   ```
3. **Send navigation goal** using RViz or command line:
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"
   ```

### Performance Metrics
- **Success Rate**: Percentage of goals reached successfully
- **Path Efficiency**: Actual path length vs optimal path
- **Time to Goal**: Time taken to reach destination
- **Safety**: Number of collisions or near-misses
- **Balance Maintenance**: Ability to maintain stable locomotion

## Troubleshooting Common Issues

### Navigation Instability
- **Issue**: Robot falls over during navigation
- **Solution**: Reduce velocity commands, improve balance controller

### Poor Path Planning
- **Issue**: Planner creates infeasible paths for humanoid
- **Solution**: Adjust costmap inflation, add humanoid-specific constraints

### Localization Issues
- **Issue**: Robot loses track of position
- **Solution**: Verify sensor configuration, tune AMCL parameters

### Performance Issues
- **Issue**: Slow navigation or high CPU usage
- **Solution**: Optimize parameters, reduce map resolution if needed

## Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)
- [Isaac Sim Navigation Integration](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_ros.html)

## Next Steps

After setting up Nav2 for humanoid robots:
1. Test navigation in various environments
2. Fine-tune parameters for your specific robot
3. Integrate with perception systems
4. Implement advanced features like footstep planning
5. Test on real hardware if available