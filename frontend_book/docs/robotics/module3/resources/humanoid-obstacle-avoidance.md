# Humanoid Navigation: Obstacle Avoidance Implementation

This document provides a comprehensive guide to implementing obstacle avoidance specifically for humanoid robots in navigation scenarios, addressing the unique challenges of bipedal locomotion.

## Understanding Humanoid-Specific Obstacle Avoidance

### Key Challenges
- **Balance Constraints**: Avoiding obstacles while maintaining balance
- **Limited Maneuverability**: Cannot move in all directions equally well
- **Step Constraints**: Must take discrete steps rather than continuous motion
- **Turning Limitations**: Cannot turn in place like wheeled robots
- **Stability Requirements**: Need to maintain stable foot placement during avoidance

### Differences from Wheeled Robot Avoidance
- **Discrete Motion**: Must plan around obstacles in step-sized increments
- **Balance Priority**: Balance may take precedence over optimal path
- **Gait Considerations**: Avoidance must work with current gait pattern
- **Support Polygon**: Must maintain center of mass within support polygon during avoidance

## Humanoid Obstacle Avoidance Strategies

### 1. Local Path Adjustment
- **Modify existing path** to go around obstacles
- **Maintain balance** during path adjustments
- **Preserve gait rhythm** while avoiding obstacles

### 2. Footstep Planning
- **Plan safe footsteps** around obstacles
- **Consider step size limitations**
- **Ensure stable support polygon**

### 3. Dynamic Window Approach (DWA) Adaptation
- **Humanoid-specific velocity space**
- **Balance-constrained velocity selection**
- **Step-aware trajectory evaluation**

## Implementation Approaches

### Approach 1: Costmap-Based Avoidance with Humanoid Constraints

```python
#!/usr/bin/env python3
# humanoid_obstacle_avoider.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, Path
from builtin_interfaces.msg import Duration
import numpy as np
import math

class HumanoidObstacleAvoider(Node):
    def __init__(self):
        super().__init__('humanoid_obstacle_avoider')

        # Parameters for humanoid-specific avoidance
        self.declare_parameter('robot_width', 0.4)
        self.declare_parameter('robot_depth', 0.3)
        self.declare_parameter('balance_margin', 0.2)
        self.declare_parameter('step_length', 0.3)
        self.declare_parameter('max_avoidance_turn', 0.2)  # radians
        self.declare_parameter('avoidance_speed', 0.15)    # m/s

        self.robot_width = self.get_parameter('robot_width').value
        self.robot_depth = self.get_parameter('robot_depth').value
        self.balance_margin = self.get_parameter('balance_margin').value
        self.step_length = self.get_parameter('step_length').value
        self.max_avoidance_turn = self.get_parameter('max_avoidance_turn').value
        self.avoidance_speed = self.get_parameter('avoidance_speed').value

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.local_plan_sub = self.create_subscription(
            Path, '/local_plan', self.local_plan_callback, 10
        )

        # Publisher for avoidance commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_avoidance', 10)

        # State variables
        self.current_scan = None
        self.local_plan = None
        self.obstacle_detected = False
        self.avoidance_active = False

        self.get_logger().info('Humanoid Obstacle Avoider initialized')

    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        self.current_scan = msg
        self.check_for_obstacles()

    def local_plan_callback(self, msg):
        """Process local plan for context-aware avoidance"""
        self.local_plan = msg

    def check_for_obstacles(self):
        """Check scan data for obstacles in the path"""
        if self.current_scan is None:
            return

        # Check if there are obstacles in the forward direction
        # (typically in the front 90 degrees)
        ranges = np.array(self.current_scan.ranges)

        # Filter out invalid ranges
        valid_ranges = ranges[(ranges > self.current_scan.range_min) &
                             (ranges < self.current_scan.range_max)]

        if len(valid_ranges) == 0:
            self.obstacle_detected = False
            return

        # Get the front sector (90 degrees in front)
        angle_increment = self.current_scan.angle_increment
        front_start = int(len(ranges) / 2 - 45 * math.pi / 180 / angle_increment)
        front_end = int(len(ranges) / 2 + 45 * math.pi / 180 / angle_increment)

        if 0 <= front_start < len(ranges) and 0 <= front_end < len(ranges):
            front_ranges = ranges[front_start:front_end]
            front_ranges = front_ranges[(front_ranges > self.current_scan.range_min) &
                                       (front_ranges < self.current_scan.range_max)]

            if len(front_ranges) > 0:
                min_distance = np.min(front_ranges)
                # Consider obstacle if closer than safety distance
                safety_distance = self.robot_width + 0.3  # Add safety margin
                self.obstacle_detected = min_distance < safety_distance

    def compute_avoidance_velocity(self, current_pose, current_velocity):
        """Compute avoidance velocity based on obstacles"""
        if not self.obstacle_detected or self.current_scan is None:
            # No obstacle, return None to indicate normal navigation
            return None

        cmd_vel = Twist()

        # Get obstacle information
        ranges = np.array(self.current_scan.ranges)
        angles = np.linspace(
            self.current_scan.angle_min,
            self.current_scan.angle_max,
            len(ranges)
        )

        # Find obstacles in the immediate path
        obstacle_angles = []
        obstacle_distances = []

        for i, (r, a) in enumerate(zip(ranges, angles)):
            if self.current_scan.range_min < r < self.current_scan.range_max:
                # Check if this obstacle is in the path (within robot width)
                obstacle_x = r * math.cos(a)
                obstacle_y = r * math.sin(a)

                # If obstacle is within robot's width range
                if abs(obstacle_y) < self.robot_width / 2 + 0.2:  # Add buffer
                    obstacle_angles.append(a)
                    obstacle_distances.append(r)

        if not obstacle_angles:
            # No obstacles in immediate path, continue normally
            return None

        # Determine avoidance direction based on obstacle distribution
        left_obstacles = [d for a, d in zip(obstacle_angles, obstacle_distances) if a < 0]
        right_obstacles = [d for a, d in zip(obstacle_angles, obstacle_distances) if a > 0]

        avg_left_dist = np.mean(left_obstacles) if left_obstacles else float('inf')
        avg_right_dist = np.mean(right_obstacles) if right_obstacles else float('inf')

        # Choose direction with more clearance
        if avg_left_dist > avg_right_dist:
            # Turn left to avoid obstacle
            cmd_vel.angular.z = min(self.max_avoidance_turn, 0.5 / avg_right_dist)
        else:
            # Turn right to avoid obstacle
            cmd_vel.angular.z = max(-self.max_avoidance_turn, -0.5 / avg_left_dist)

        # Reduce forward speed during avoidance
        cmd_vel.linear.x = min(self.avoidance_speed, current_velocity.linear.x * 0.5)

        # Ensure balance during avoidance
        cmd_vel.angular.z = self.limit_turn_for_balance(cmd_vel.angular.z)

        return cmd_vel

    def limit_turn_for_balance(self, angular_velocity):
        """Limit turning rate to maintain balance"""
        # For humanoid robots, sharp turns can compromise balance
        max_angular_for_balance = 0.3  # Adjust based on robot capabilities
        return max(-max_angular_for_balance, min(max_angular_for_balance, angular_velocity))

    def is_path_clear(self, start_pose, target_pose):
        """Check if path to target is clear of obstacles"""
        # This would typically check the costmap
        # For this example, we'll use scan data
        if self.current_scan is None:
            return True

        # Calculate path from start to target
        dx = target_pose.position.x - start_pose.position.x
        dy = target_pose.position.y - start_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.1:  # Very close, assume clear
            return True

        # Check scan points along the path
        # This is a simplified version - in practice, you'd check the costmap
        return True

def main(args=None):
    rclpy.init(args=args)
    avoider = HumanoidObstacleAvoider()

    try:
        rclpy.spin(avoider)
    except KeyboardInterrupt:
        pass
    finally:
        avoider.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Approach 2: Footstep-Aware Obstacle Avoidance

```python
#!/usr/bin/env python3
# footstep_aware_avoider.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class FootstepAwareAvoider(Node):
    def __init__(self):
        super().__init__('footstep_aware_avoider')

        # Humanoid-specific parameters
        self.declare_parameter('step_length', 0.3)
        self.declare_parameter('step_width', 0.2)
        self.declare_parameter('foot_size', 0.15)  # Size of foot in meters
        self.declare_parameter('balance_polygon_radius', 0.25)

        self.step_length = self.get_parameter('step_length').value
        self.step_width = self.get_parameter('step_width').value
        self.foot_size = self.get_parameter('foot_size').value
        self.balance_polygon_radius = self.get_parameter('balance_polygon_radius').value

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.current_scan = None
        self.current_path = None
        self.left_foot_pos = None
        self.right_foot_pos = None

        self.get_logger().info('Footstep-Aware Obstacle Avoider initialized')

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.current_scan = msg

    def path_callback(self, msg):
        """Process navigation path"""
        self.current_path = msg

    def plan_avoidance_footsteps(self, current_pose, obstacles):
        """Plan safe footsteps around obstacles"""
        safe_footsteps = []

        # Get current foot positions
        current_left = self.get_left_foot_position(current_pose)
        current_right = self.get_right_foot_position(current_pose)

        # Plan footsteps that avoid obstacles
        for obstacle in obstacles:
            # Calculate safe position to step around obstacle
            safe_pos = self.calculate_safe_step_position(obstacle, current_pose)
            if safe_pos:
                safe_footsteps.append(safe_pos)

        return safe_footsteps

    def get_left_foot_position(self, robot_pose):
        """Calculate left foot position based on robot pose"""
        # Calculate left foot position relative to robot center
        yaw = self.get_yaw_from_quaternion(robot_pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Offset left foot to the left of robot center
        offset_x = -self.step_width/2 * sin_yaw
        offset_y = self.step_width/2 * cos_yaw

        return Point(
            x=robot_pose.position.x + offset_x,
            y=robot_pose.position.y + offset_y,
            z=0.0
        )

    def get_right_foot_position(self, robot_pose):
        """Calculate right foot position based on robot pose"""
        # Calculate right foot position relative to robot center
        yaw = self.get_yaw_from_quaternion(robot_pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Offset right foot to the right of robot center
        offset_x = self.step_width/2 * sin_yaw
        offset_y = -self.step_width/2 * cos_yaw

        return Point(
            x=robot_pose.position.x + offset_x,
            y=robot_pose.position.y + offset_y,
            z=0.0
        )

    def calculate_safe_step_position(self, obstacle, current_pose):
        """Calculate safe step position to avoid an obstacle"""
        # Calculate vector from robot to obstacle
        to_obstacle_x = obstacle.x - current_pose.position.x
        to_obstacle_y = obstacle.y - current_pose.position.y
        distance_to_obstacle = math.sqrt(to_obstacle_x**2 + to_obstacle_y**2)

        if distance_to_obstacle < 0.5:  # Obstacle is close
            # Calculate perpendicular direction to move around obstacle
            robot_yaw = self.get_yaw_from_quaternion(current_pose.orientation)

            # Calculate perpendicular vector
            perp_x = -to_obstacle_y
            perp_y = to_obstacle_x
            perp_length = math.sqrt(perp_x**2 + perp_y**2)

            if perp_length > 0:
                perp_x /= perp_length
                perp_y /= perp_length

                # Choose direction that's furthest from other obstacles
                step1_x = current_pose.position.x + perp_x * self.step_length
                step1_y = current_pose.position.y + perp_y * self.step_length
                step2_x = current_pose.position.x - perp_x * self.step_length
                step2_y = current_pose.position.y - perp_y * self.step_length

                # Check which side is clearer
                if self.is_area_clear(step1_x, step1_y):
                    return Point(x=step1_x, y=step1_y, z=0.0)
                else:
                    return Point(x=step2_x, y=step2_y, z=0.0)

        return None

    def is_area_clear(self, x, y):
        """Check if area around (x,y) is clear of obstacles"""
        if self.current_scan is None:
            return True

        # This is a simplified check - in practice, you'd use the costmap
        # For now, we'll check if any scan points are close to this position
        ranges = np.array(self.current_scan.ranges)
        angles = np.linspace(
            self.current_scan.angle_min,
            self.current_scan.angle_max,
            len(ranges)
        )

        for r, a in zip(ranges, angles):
            if self.current_scan.range_min < r < self.current_scan.range_max:
                scan_x = r * math.cos(a)
                scan_y = r * math.sin(a)

                # Check distance to our test point
                dist = math.sqrt((scan_x - x)**2 + (scan_y - y)**2)
                if dist < self.foot_size + 0.1:  # Add small buffer
                    return False

        return True

    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    avoider = FootstepAwareAvoider()

    try:
        rclpy.spin(avoider)
    except KeyboardInterrupt:
        pass
    finally:
        avoider.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Navigation Stack

### Behavior Tree Integration

```xml
<!-- behavior_trees/humanoid_with_obstacle_avoidance.xml -->
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
        <RecoveryNode number_of_retries="2" name="FollowPathWithAvoidance">
          <Sequence>
            <Fallback>
              <!-- Try normal path following -->
              <FollowPath path="path" controller_id="HumanoidController"/>
              <!-- If blocked, try obstacle avoidance -->
              <HumanoidObstacleAvoidance/>
            </Fallback>
          </Sequence>
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

## Costmap Layer for Humanoid-Specific Obstacles

### Balance-Aware Costmap Layer

```cpp
// balance_aware_layer.cpp (conceptual C++ code)
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

class BalanceAwareLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  BalanceAwareLayer() = default;

  virtual void onInitialize();
  virtual void updateBounds(
    double origin_x, double origin_y, double origin_yaw,
    double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i, int max_j);

private:
  double balance_margin_;
  double robot_width_;
  double robot_depth_;
};

void BalanceAwareLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  nh.param("balance_margin", balance_margin_, 0.2);
  nh.param("robot_width", robot_width_, 0.4);
  nh.param("robot_depth", robot_depth_, 0.3);
}

void BalanceAwareLayer::updateBounds(
  double origin_x, double origin_y, double origin_yaw,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  // Update bounds considering balance constraints
  *min_x = std::min(*min_x, origin_x - robot_width_/2 - balance_margin_);
  *min_y = std::min(*min_y, origin_y - robot_depth_/2 - balance_margin_);
  *max_x = std::max(*max_x, origin_x + robot_width_/2 + balance_margin_);
  *max_y = std::max(*max_y, origin_y + robot_depth_/2 + balance_margin_);
}

void BalanceAwareLayer::updateCosts(
  nav2_costmap_2d::Costmap2D& master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  // Apply costs based on balance constraints
  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      unsigned int mx, my;
      if (master_grid.indexToCells(i, j, mx, my)) {
        double wx, wy;
        master_grid.mapToWorld(mx, my, wx, wy);

        // Check if this cell is within the balance-critical area
        // and apply appropriate costs
        unsigned char old_cost = master_grid.getCost(mx, my);
        if (old_cost != nav2_costmap_2d::NO_INFORMATION) {
          // Apply balance-aware cost inflation
          master_grid.setCost(mx, my, old_cost + 20); // Increase cost near obstacles
        }
      }
    }
  }
}
```

## Parameter Configuration

### Humanoid Obstacle Avoidance Parameters

```yaml
# config/humanoid_obstacle_avoidance_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "balance_layer"]
      balance_layer:
        plugin: "nav2_costmap_2d::BalanceAwareLayer"
        enabled: True
        balance_margin: 0.25
        robot_width: 0.4
        robot_depth: 0.3
        cost_multiplier: 1.5

controller_server:
  ros__parameters:
    HumanoidController:
      # Humanoid-specific avoidance parameters
      obstacle_check_distance: 0.8
      avoidance_speed_factor: 0.5
      max_avoidance_turn_rate: 0.2
      balance_check_frequency: 10.0
      step_adjustment_threshold: 0.15

behavior_tree:
  ros__parameters:
    # Obstacle avoidance behavior parameters
    avoidance_timeout: 5.0
    recovery_min_distance: 0.3
    path_clearance_threshold: 0.4
```

## Testing and Validation

### Obstacle Avoidance Test Scenarios

```python
#!/usr/bin/env python3
# test_obstacle_avoidance.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoidanceTester(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_tester')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Timer to run tests
        self.test_timer = self.create_timer(10.0, self.run_test_scenario)

        self.test_stage = 0
        self.get_logger().info('Obstacle Avoidance Tester initialized')

    def run_test_scenario(self):
        """Run different obstacle avoidance test scenarios"""
        if self.test_stage == 0:
            # Test 1: Forward obstacle
            self.create_forward_obstacle_scenario()
        elif self.test_stage == 1:
            # Test 2: Narrow passage
            self.create_narrow_passage_scenario()
        elif self.test_stage == 2:
            # Test 3: Dynamic obstacle
            self.create_dynamic_obstacle_scenario()

        self.test_stage = (self.test_stage + 1) % 3

    def create_forward_obstacle_scenario(self):
        """Create a scenario with an obstacle directly in front"""
        self.get_logger().info('Running forward obstacle test')

        # Publish goal that requires obstacle avoidance
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 3.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

    def create_narrow_passage_scenario(self):
        """Create a scenario with a narrow passage to navigate through"""
        self.get_logger().info('Running narrow passage test')

        # Publish goal for narrow passage
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 4.0
        goal.pose.position.y = 0.5  # Offset to test lateral movement
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

    def create_dynamic_obstacle_scenario(self):
        """Create a scenario with a moving obstacle"""
        self.get_logger().info('Running dynamic obstacle test')

        # Publish goal that will require dynamic avoidance
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.5
        goal.pose.position.y = -1.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    tester = ObstacleAvoidanceTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### Efficient Obstacle Detection

```python
def efficient_obstacle_detection(self, scan_msg):
    """Optimized obstacle detection for humanoid navigation"""
    # Use only relevant sectors of the scan
    ranges = np.array(scan_msg.ranges)
    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

    # Define sectors relevant to humanoid navigation
    forward_sector = self.get_sector_indices(angles, -45, 45)  # Forward
    left_sector = self.get_sector_indices(angles, 45, 135)     # Left
    right_sector = self.get_sector_indices(angles, -135, -45)  # Right

    # Process each sector separately
    forward_obstacles = self.process_sector(ranges, forward_sector)
    left_obstacles = self.process_sector(ranges, left_sector)
    right_obstacles = self.process_sector(ranges, right_sector)

    return forward_obstacles, left_obstacles, right_obstacles

def process_sector(self, ranges, sector_indices):
    """Process a specific sector for obstacle information"""
    sector_ranges = ranges[sector_indices]
    valid_ranges = sector_ranges[(sector_ranges > 0.1) & (sector_ranges < 3.0)]  # Filter valid ranges

    if len(valid_ranges) == 0:
        return None

    min_distance = np.min(valid_ranges)
    avg_distance = np.mean(valid_ranges)

    return {
        'min_distance': min_distance,
        'avg_distance': avg_distance,
        'obstacle_count': len(valid_ranges)
    }
```

## Troubleshooting Common Issues

### Balance Loss During Avoidance
- **Issue**: Robot becomes unstable when avoiding obstacles
- **Solution**: Reduce turning rates, increase safety margins, implement balance feedback

### Ineffective Avoidance
- **Issue**: Robot doesn't properly avoid obstacles
- **Solution**: Adjust costmap inflation, increase sensor range, improve obstacle detection

### Oscillation
- **Issue**: Robot oscillates when trying to avoid obstacles
- **Solution**: Implement hysteresis, increase decision thresholds, add smoothing

### Performance Issues
- **Issue**: Avoidance computation is too slow
- **Solution**: Optimize algorithms, reduce sensor data processing, use efficient data structures

## Resources

- [Nav2 Obstacle Avoidance](https://navigation.ros.org/behavior_trees/index.html)
- [Humanoid Robotics Research](https://ieeexplore.ieee.org/document/8460821)
- [ROS 2 Navigation Tuning Guide](https://navigation.ros.org/tuning/index.html)

## Next Steps

After implementing obstacle avoidance:
1. Test in Isaac Sim with various obstacle scenarios
2. Fine-tune parameters for your specific robot
3. Integrate with footstep planning for stable avoidance
4. Test dynamic obstacle scenarios
5. Validate performance in complex environments