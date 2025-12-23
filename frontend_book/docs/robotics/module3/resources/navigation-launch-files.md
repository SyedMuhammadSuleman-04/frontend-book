# Navigation Launch Files for Isaac Sim Humanoid Navigation

This document provides comprehensive launch files for setting up navigation with humanoid robots in Isaac Sim using Nav2.

## Understanding the Navigation Stack Components

### Core Navigation Components
- **AMCL**: Adaptive Monte Carlo Localization
- **Global Planner**: Path planning for navigation
- **Local Planner**: Local path following and obstacle avoidance
- **Controller**: Velocity command generation
- **Costmaps**: Global and local costmap management
- **Behavior Tree Navigator**: Task execution and recovery

### Isaac Sim Integration
- **Sensor Bridge**: Connects Isaac Sim sensors to ROS 2
- **TF Publisher**: Maintains coordinate frame relationships
- **Robot State Publisher**: Publishes robot joint states

## Basic Navigation Launch File

### Complete Navigation Launch File

```python
# isaac_humanoid_navigation_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Input arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    lifecycle_nodes = LaunchConfiguration('lifecycle_nodes')

    # Isaac Sim specific parameters
    isaac_sim = LaunchConfiguration('isaac_sim')
    robot_description = LaunchConfiguration('robot_description')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Set to true for Isaac Sim
        description='Use simulation (Isaac Sim) clock if true')

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_humanoid_navigation'),
            'config',
            'isaac_humanoid_nav2_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    default_bt_xml_filename_arg = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_humanoid_navigation'),
            'behavior_trees',
            'humanoid_nav_to_pose_w_replanning_and_recovery.xml'
        ]),
        description='Full path to the behavior tree xml file to use')

    map_subscribe_transient_local_arg = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='false',
        description='Whether to set the map subscriber QoS to transient local')

    isaac_sim_arg = DeclareLaunchArgument(
        'isaac_sim',
        default_value='true',
        description='Whether running in Isaac Sim environment')

    # Isaac Sim robot description (if using URDF)
    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=PathJoinSubstitution([
            FindPackageShare('your_robot_description'),
            'urdf',
            'humanoid_robot.urdf'
        ]),
        description='Full path to robot description file to use')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(autostart_arg)
    ld.add_action(params_file_arg)
    ld.add_action(default_bt_xml_filename_arg)
    ld.add_action(map_subscribe_transient_local_arg)
    ld.add_action(isaac_sim_arg)
    ld.add_action(robot_description_arg)

    # Set environment variables
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    # Launch navigation container
    navigation_group = GroupAction(
        actions=[
            # Lifecycle manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': [
                                'map_server',
                                'planner_server',
                                'controller_server',
                                'bt_navigator',
                                'amcl',
                                'velocity_smoother',
                                'slam_toolbox']}]),

            # Map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # Planner server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # Controller server
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # BT navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # AMCL for localization
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # Velocity smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # Lifecycle manager for costmaps
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': ['local_costmap', 'global_costmap']}]),

            # Local costmap
            Node(
                package='nav2_costmap_2d',
                executable='nav2_costmap_2d',
                name='local_costmap',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # Global costmap
            Node(
                package='nav2_costmap_2d',
                executable='nav2_costmap_2d',
                name='global_costmap',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # Recovery server
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # Waypoint follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                namespace=namespace,
                output='screen',
                parameters=[params_file]),

            # Isaac Sim specific nodes
            # Robot State Publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'robot_description': robot_description}])
        ]
    )

    ld.add_action(navigation_group)

    return ld
```

## Isaac Sim Specific Launch Configuration

### Isaac Sim Integration Launch File

```python
# isaac_sim_navigation_integration_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Input arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    isaac_sim = LaunchConfiguration('isaac_sim')
    isaac_sim_launch = LaunchConfiguration('isaac_sim_launch')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_humanoid_navigation'),
            'config',
            'isaac_humanoid_nav2_params.yaml'
        ]),
        description='Full path to navigation parameters file')

    isaac_sim_arg = DeclareLaunchArgument(
        'isaac_sim',
        default_value='true',
        description='Whether running in Isaac Sim')

    isaac_sim_launch_arg = DeclareLaunchArgument(
        'isaac_sim_launch',
        default_value='false',
        description='Whether to launch Isaac Sim from ROS')

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(params_file_arg)
    ld.add_action(isaac_sim_arg)
    ld.add_action(isaac_sim_launch_arg)

    # Isaac Sim Sensor Bridge
    isaac_ros_bridge = Node(
        package='isaac_ros_bridges',
        executable='ros_bridge_node',
        name='isaac_ros_bridge',
        namespace=namespace,
        parameters=[params_file],
        condition=IfCondition(isaac_sim),
        remappings=[
            ('/camera/rgb/image_raw', '/camera/rgb/image_rect_color'),
            ('/camera/depth/image_raw', '/camera/depth/image_rect_raw'),
            ('/lidar/points', '/scan'),
            ('/imu/data', '/imu/data_raw'),
            ('/odom', '/odometry/filtered')
        ]
    )

    # Isaac Sim Perception Pipeline
    perception_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('your_robot_perception'),
                'launch',
                'perception_pipeline_launch.py'
            ])
        ]),
        condition=IfCondition(isaac_sim)
    )

    # Add nodes to launch description
    ld.add_action(isaac_ros_bridge)
    ld.add_action(perception_pipeline)

    # Timer to delay Isaac Sim integration after main navigation
    delayed_isaac_integration = TimerAction(
        period=5.0,
        actions=[
            # Isaac Sim specific nodes that need to start after navigation
        ]
    )

    ld.add_action(delayed_isaac_integration)

    return ld
```

## Parameter Configuration Files

### Navigation Parameters for Isaac Sim

```yaml
# config/isaac_humanoid_nav2_params.yaml
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
    # Isaac Sim specific behavior tree
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
      # Humanoid-specific parameters for Isaac Sim
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
      # Humanoid-specific parameters for Isaac Sim
      step_size: 0.5  # Adjust for humanoid step capabilities
      min_distance_to_obstacles: 0.4  # Balance safety with navigation capability

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
```

## Isaac Sim Behavior Trees

### Custom Behavior Tree for Humanoid Navigation

```xml
<!-- behavior_trees/humanoid_nav_to_pose_w_replanning_and_recovery.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <RecoveryNode number_of_retries="2" name="ComputePathToPoseWithFallback">
              <Sequence>
                <GoalNode name="GoalUpdated"/>
                <ComputePathToPose goal="goal" path="path" planner_id="GridBased"/>
              </Sequence>
              <ReactiveFallback name="ComputePathToPoseWithFallbackFallback">
                <GoalUpdated/>
                <ComputePathToPose goal="goal" path="path" planner_id="GridBased"/>
              </ReactiveFallback>
            </RecoveryNode>
            <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="2" name="FollowPath">
          <FollowPath path="path" controller_id="HumanoidController"/>
          <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RecoveryNode number_of_retries="2" name="PlanThroughFailure">
          <PlanFollowRecovery path="path" planner_id="GridBased" controller_id="HumanoidController"/>
          <ClearEntireCostmap name="ClearLocalCostmap-2" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

## Simplified Launch for Testing

### Minimal Navigation Launch

```python
# minimal_navigation_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Input arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)

    # Simple AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Simple controller server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Simple planner server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Add nodes to launch description
    ld.add_action(amcl_node)
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)

    return ld
```

## Isaac Sim Navigation Integration Launch

### Complete Isaac Sim Navigation Launch

```python
# isaac_sim_complete_navigation_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Input arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('isaac_humanoid_navigation'),
            'config',
            'isaac_humanoid_nav2_params.yaml'
        ]),
        description='Full path to navigation parameters file')

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(params_file_arg)

    # Include main navigation launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_humanoid_navigation'),
                'launch',
                'isaac_humanoid_navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # Include Isaac Sim integration launch
    isaac_sim_integration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_humanoid_navigation'),
                'launch',
                'isaac_sim_navigation_integration_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # Add launches to description
    ld.add_action(nav2_launch)
    ld.add_action(isaac_sim_integration_launch)

    return ld
```

## Running the Navigation Stack

### Launch Commands

```bash
# Terminal 1: Launch Isaac Sim
cd /path/to/isaac-sim
./isaac-sim.sh

# Terminal 2: Launch Navigation Stack
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash
ros2 launch isaac_humanoid_navigation isaac_sim_complete_navigation_launch.py

# Terminal 3: Send Navigation Goals
ros2 run rviz2 rviz2  # Use RViz to send goals
# Or send directly:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"
```

## Resources

- [Nav2 Launch Documentation](https://navigation.ros.org/launch_configurations/index.html)
- [ROS 2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)
- [Isaac Sim ROS Integration](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_ros.html)

## Next Steps

After setting up the navigation launch files:
1. Test the basic navigation setup in Isaac Sim
2. Fine-tune parameters for your specific robot
3. Add custom behavior trees for humanoid-specific navigation
4. Implement footstep planning integration
5. Test recovery behaviors for humanoid robots