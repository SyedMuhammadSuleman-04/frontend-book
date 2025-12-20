---
sidebar_label: 'Nodes and Topics'
title: 'Nodes and Topics'
---

# Nodes and Topics

This section covers the fundamental ROS 2 concepts of nodes and topics, which form the basis of the publish-subscribe communication pattern in ROS 2.

## Learning Objectives

After completing this section, you will be able to:
- Understand what ROS 2 nodes are and their role in robotic systems
- Explain the publish-subscribe communication pattern
- Create simple publisher and subscriber nodes
- Understand the concept of topics and messages

## What are Nodes?

In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 program. Multiple nodes are usually combined together to form a complete robot application.

Nodes in ROS 2 are designed to be modular, meaning a single application might have many nodes that each perform a specific task. For example, one node might handle sensor data, another might process that data, and a third might send commands to actuators.

## What are Topics and Messages?

Topics are named buses over which nodes exchange messages. The topic name is a unique identifier that allows nodes to find each other to send and receive data. Messages are the actual data being passed between nodes.

The publish-subscribe communication pattern works as follows:
- Publisher nodes send messages to a topic
- Subscriber nodes receive messages from a topic
- Multiple publishers and subscribers can use the same topic

## Creating a Simple Publisher Node

Here's an example of a simple publisher node in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Simple Subscriber Node

Here's an example of a simple subscriber node in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Example

To run these nodes, you would typically:
1. Source your ROS 2 installation
2. Navigate to your workspace
3. Run the publisher: `ros2 run <package_name> publisher`
4. In another terminal, run the subscriber: `ros2 run <package_name> subscriber`

## Key Concepts Summary

- Nodes are the fundamental building blocks of ROS 2 programs
- Topics provide named buses for communication between nodes
- The publish-subscribe pattern allows for decoupled communication
- Messages are the data passed between nodes
- Multiple publishers and subscribers can use the same topic

## Next Steps

In the next section, we'll explore services and actions, which provide different communication patterns for synchronous and goal-oriented interactions.