---
sidebar_label: 'rclpy Integration'
title: 'rclpy Integration'
---

# rclpy Integration

This section covers the Python client library for ROS 2 (rclpy) and how to effectively integrate Python-based applications with ROS 2 systems.

## Learning Objectives

After completing this section, you will be able to:
- Use rclpy to create ROS 2 nodes in Python
- Implement publishers and subscribers in Python
- Create service clients and servers using Python
- Work with ROS 2 parameters in Python nodes
- Handle asynchronous operations in rclpy

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides a Python API to ROS 2 concepts such as nodes, publishers, subscribers, services, actions, parameters, and more. rclpy is built on top of rcl (ROS Client Library) and provides a more Pythonic interface to ROS 2 functionality.

## Setting Up rclpy

Before using rclpy, you need to initialize the ROS 2 client library:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 client library
    # Create and use nodes here
    rclpy.shutdown()  # Shutdown the ROS 2 client library
```

## Creating a Node with rclpy

Here's a basic example of creating a node in Python:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers and Subscribers in Python

### Publisher Example

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

### Subscriber Example

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

## Working with Parameters

Parameters allow nodes to be configured at runtime. Here's how to use them with rclpy:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_param', 'default_value')
        self.declare_parameter('count_threshold', 10)

        # Get parameter values
        self.my_param = self.get_parameter('my_param').get_parameter_value().string_value
        self.threshold = self.get_parameter('count_threshold').get_parameter_value().integer_value

        self.get_logger().info(f'Parameter values: {self.my_param}, {self.threshold}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services in Python

### Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info('Result of add_two_ints: %d' % response.sum)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Asynchronous Operations

rclpy provides several ways to handle asynchronous operations:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Create a timer for periodic tasks
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Create a client for service calls
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        # Process message asynchronously

    def timer_callback(self):
        self.get_logger().info('Timer callback executed')

def main(args=None):
    rclpy.init(args=args)
    node = AsyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices with rclpy

1. **Error Handling**: Always implement proper error handling for ROS 2 operations
2. **Node Lifecycle**: Properly initialize and clean up nodes
3. **Logging**: Use the built-in logger for debugging and monitoring
4. **Parameters**: Use parameters for configuration instead of hardcoded values
5. **Message Types**: Use appropriate message types for your data

## Integration with Python AI Libraries

rclpy integrates seamlessly with popular Python AI libraries:

```python
import rclpy
from rclpy.node import Node
import numpy as np
# You can import other AI libraries like tensorflow, pytorch, etc.

class AINode(Node):
    def __init__(self):
        super().__init__('ai_node')
        # Initialize AI model here
        self.model = self.initialize_model()

        # Create subscribers for sensor data
        self.sensor_sub = self.create_subscription(
            # Appropriate message type for sensor data
            # Process sensor data with AI model
        )

    def initialize_model(self):
        # Initialize your AI model
        pass
```

## Key Concepts Summary

- rclpy provides the Python interface to ROS 2
- Nodes in Python inherit from the Node class
- Publishers, subscribers, services, and actions work similarly to other ROS 2 clients
- Parameters provide runtime configuration
- Asynchronous operations are handled through callbacks and futures

## Next Steps

Now that you understand how to integrate Python with ROS 2 using rclpy, the next section will explore how to use this integration to create AI agents that control robotic systems.