---
sidebar_label: 'AI Agent Control'
title: 'AI Agent Control'
---

# AI Agent Control

This section explores how to integrate AI algorithms with ROS 2 systems to create intelligent robot controllers. You'll learn to connect AI decision-making with robotic actuators through ROS 2 communication patterns.

## Learning Objectives

After completing this section, you will be able to:
- Connect AI algorithms to ROS 2 robot control systems
- Implement perception-action loops using ROS 2
- Design AI agents that respond to sensor data
- Create intelligent robot behaviors using Python AI libraries
- Implement decision-making algorithms for robotic tasks

## Introduction to AI-Robot Integration

Integrating AI with robotics involves connecting intelligent decision-making algorithms to physical or simulated robot systems. ROS 2 provides the communication infrastructure to bridge these domains, allowing AI agents to perceive their environment through sensors and act through robot actuators.

## Perception-Action Loop

The perception-action loop is fundamental to intelligent robotic systems:

```
Sensors → Perception → Decision Making → Action → Actuators
    ↑                                        ↓
    └──────────────── Environment ────────────┘
```

In ROS 2, this loop is implemented using:
- **Sensors**: Publishers sending sensor data (topics)
- **Perception**: Nodes processing sensor data
- **Decision Making**: AI agents making choices
- **Action**: Commands sent to robot actuators (topics, services, or actions)

## Basic AI Agent Structure

Here's a template for an AI agent that controls a robot:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AIAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')

        # Subscribers for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publisher for robot commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Initialize AI components
        self.sensor_data = None
        self.command_timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        # Store sensor data for AI processing
        self.sensor_data = msg.ranges  # Laser scan ranges

    def control_loop(self):
        if self.sensor_data is not None:
            # Process sensor data with AI algorithm
            command = self.make_decision(self.sensor_data)

            # Publish command to robot
            self.cmd_pub.publish(command)

    def make_decision(self, sensor_data):
        # AI decision-making logic here
        cmd = Twist()

        # Example: Simple obstacle avoidance
        min_distance = min(sensor_data) if sensor_data else float('inf')

        if min_distance < 1.0:  # Obstacle too close
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn
        else:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0

        return cmd

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgent()
    rclpy.spin(ai_agent)
    ai_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Machine Learning Libraries

### Using scikit-learn for Classification

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from sklearn.cluster import KMeans
import numpy as np

class MLSensorNode(Node):
    def __init__(self):
        super().__init__('ml_sensor_node')

        # Subscribers and publishers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pc_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize ML model
        self.model = KMeans(n_clusters=3)  # Example: clustering

    def pc_callback(self, msg):
        # Convert PointCloud2 to numpy array (simplified)
        # In practice, you'd use pcl or similar libraries
        point_data = self.pointcloud_to_array(msg)

        if point_data is not None and len(point_data) > 0:
            # Apply ML algorithm
            clusters = self.model.fit_predict(point_data)

            # Make decisions based on clustering results
            command = self.decide_based_on_clusters(clusters)
            self.cmd_pub.publish(command)

    def pointcloud_to_array(self, pc_msg):
        # Implementation to convert PointCloud2 to numpy array
        # This is simplified - real implementation would use libraries like sensor_msgs_py
        pass

    def decide_based_on_clusters(self, clusters):
        # Make robot decisions based on ML results
        cmd = Twist()
        # Example logic based on cluster analysis
        return cmd
```

### Using TensorFlow/PyTorch for Deep Learning

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
# import tensorflow as tf or torch

class DeepLearningController(Node):
    def __init__(self):
        super().__init__('dl_controller')

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Load trained model (example with placeholder)
        # self.model = tf.keras.models.load_model('path/to/model')

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Preprocess image for model
            processed_image = self.preprocess_image(cv_image)

            # Run inference
            # prediction = self.model.predict(processed_image)

            # Convert prediction to robot command
            command = self.prediction_to_command(None)  # Placeholder
            self.cmd_pub.publish(command)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def preprocess_image(self, image):
        # Preprocess image for your model
        # Resize, normalize, etc.
        return image

    def prediction_to_command(self, prediction):
        # Convert model prediction to robot command
        cmd = Twist()
        # Example: map prediction values to linear/angular velocities
        return cmd
```

## State Machines for AI Agents

For more complex behaviors, state machines can organize AI decision-making:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    EXPLORING = 2
    AVOIDING_OBSTACLE = 3
    REACHED_GOAL = 4

class StateMachineAIAgent(Node):
    def __init__(self):
        super().__init__('state_machine_agent')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_state = RobotState.IDLE
        self.state_timer = self.create_timer(0.1, self.state_machine)

        self.sensor_data = None

    def scan_callback(self, msg):
        self.sensor_data = msg.ranges

    def state_machine(self):
        if self.sensor_data is None:
            return

        # State transition logic
        self.transition_logic()

        # Execute current state behavior
        command = self.execute_state()
        self.cmd_pub.publish(command)

    def transition_logic(self):
        if self.current_state == RobotState.IDLE:
            # Transition to exploring if sensor data is available
            if self.sensor_data:
                self.current_state = RobotState.EXPLORING

        elif self.current_state == RobotState.EXPLORING:
            # Check if obstacle is detected
            if min(self.sensor_data) < 0.5:
                self.current_state = RobotState.AVOIDING_OBSTACLE

        elif self.current_state == RobotState.AVOIDING_OBSTACLE:
            # Check if path is clear
            if min(self.sensor_data) > 1.0:
                self.current_state = RobotState.EXPLORING

    def execute_state(self):
        cmd = Twist()

        if self.current_state == RobotState.IDLE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self.current_state == RobotState.EXPLORING:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0

        elif self.current_state == RobotState.AVOIDING_OBSTACLE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn to avoid

        return cmd
```

## Reinforcement Learning Integration

For adaptive behavior, reinforcement learning can be integrated with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class RLLearningAgent(Node):
    def __init__(self):
        super().__init__('rl_agent')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # RL components (simplified example)
        self.q_table = np.zeros((10, 10, 4))  # State, Action space
        self.learning_rate = 0.1
        self.discount_factor = 0.95
        self.epsilon = 0.1  # Exploration rate

        self.previous_state = None
        self.previous_action = None
        self.previous_reward = None

    def scan_callback(self, msg):
        # Convert sensor data to state representation
        state = self.sensors_to_state(msg.ranges)

        # Choose action using epsilon-greedy policy
        action = self.choose_action(state)

        # Execute action
        command = self.action_to_command(action)
        self.cmd_pub.publish(command)

        # Calculate reward based on new state
        reward = self.calculate_reward(state)

        # Update Q-table if we have previous experience
        if self.previous_state is not None:
            self.update_q_table(self.previous_state,
                              self.previous_action,
                              self.previous_reward,
                              state)

        # Store current experience for next iteration
        self.previous_state = state
        self.previous_action = action
        self.previous_reward = reward

    def sensors_to_state(self, ranges):
        # Convert sensor readings to discrete state
        # This is a simplified example
        front_avg = sum(ranges[:len(ranges)//4]) / (len(ranges)//4)
        left_avg = sum(ranges[len(ranges)//4:len(ranges)//2]) / (len(ranges)//4)
        right_avg = sum(ranges[-len(ranges)//4:]) / (len(ranges)//4)

        # Discretize into state indices
        front_idx = min(int(front_avg * 5), 9)  # 0-9
        left_idx = min(int(left_avg * 5), 9)
        right_idx = min(int(right_avg * 5), 9)

        return (front_idx, left_idx, right_idx)

    def choose_action(self, state):
        # Epsilon-greedy action selection
        if np.random.random() < self.epsilon:
            return np.random.choice(4)  # Explore
        else:
            return np.argmax(self.q_table[state])  # Exploit

    def action_to_command(self, action):
        cmd = Twist()

        # Define actions: 0=forward, 1=turn_left, 2=turn_right, 3=stop
        if action == 0:  # Forward
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        elif action == 1:  # Turn left
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        elif action == 2:  # Turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
        elif action == 3:  # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd

    def calculate_reward(self, state):
        # Calculate reward based on state
        # Positive for moving forward in clear space, negative for obstacles
        front_dist = state[0] / 5.0  # Convert back to distance

        if front_dist < 0.5:  # Too close to obstacle
            return -10
        elif front_dist > 2.0:  # Good distance
            return 1
        else:  # In between
            return 0.1

    def update_q_table(self, state, action, reward, next_state):
        # Q-learning update rule
        current_q = self.q_table[state][action]
        max_next_q = np.max(self.q_table[next_state])

        new_q = current_q + self.learning_rate * (
            reward + self.discount_factor * max_next_q - current_q
        )

        self.q_table[state][action] = new_q
```

## Best Practices for AI-Robot Integration

1. **Modularity**: Keep AI algorithms separate from ROS 2 interface code
2. **Safety**: Always implement safety checks and limits
3. **Testing**: Test AI algorithms separately before integration
4. **Logging**: Log AI decisions and robot responses for debugging
5. **Real-time constraints**: Be mindful of computational requirements

## Key Concepts Summary

- AI agents connect perception to action through ROS 2
- Perception-action loops form the basis of intelligent behavior
- Machine learning libraries integrate with ROS 2 through Python
- State machines help organize complex AI behaviors
- Reinforcement learning enables adaptive robot behavior

## Next Steps

The next section will provide hands-on exercises to practice integrating AI algorithms with ROS 2 systems.