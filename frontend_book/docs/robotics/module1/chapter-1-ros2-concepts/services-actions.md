---
sidebar_label: 'Services and Actions'
title: 'Services and Actions'
---

# Services and Actions

This section covers two other important ROS 2 communication patterns: services for synchronous request-response interactions, and actions for goal-oriented, long-running tasks with feedback.

## Learning Objectives

After completing this section, you will be able to:
- Understand the service-server communication pattern
- Implement service clients and servers
- Understand when to use actions vs. topics or services
- Create action clients and servers
- Design appropriate communication patterns for different robotic tasks

## Services: Request-Response Communication

Services provide a request-response communication pattern in ROS 2. Unlike topics which are asynchronous, services are synchronous - the client sends a request and waits for a response from the server.

### Service Definition

Services are defined using .srv files that specify the request and response message types:

```
# Request message
string name
int32 age
---
# Response message
bool success
string message
```

### Creating a Service Server

Here's an example of a simple service server in Python:

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

### Creating a Service Client

Here's an example of a service client in Python:

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

## Actions: Goal-Oriented Communication

Actions are used for long-running tasks that provide feedback and can be canceled. They're ideal for tasks like navigation, manipulation, or any process that takes time and needs to report progress.

### Action Definition

Actions are defined using .action files with three parts:

```
# Goal: the goal of the action
int32 order
---
# Result: the result of the action
int32 sequence
---
# Feedback: the feedback of the action
int32 sequence
float32 percent_complete
```

### Creating an Action Server

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class MinimalActionServer(Node):
    def __init__(self):
        super().__init__('minimal_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Returning result: {0}'.format(result.sequence))

        return result
```

## When to Use Each Communication Pattern

### Use Topics when:
- You need asynchronous communication
- Data is continuously flowing (sensor data, state updates)
- Multiple publishers/subscribers need to interact
- The sender doesn't need to know if anyone receives the data

### Use Services when:
- You need synchronous request-response communication
- The task is relatively quick
- You need to know the result immediately
- The operation is idempotent (same request gives same result)

### Use Actions when:
- The task takes a long time to complete
- You need to provide feedback during execution
- The task can be canceled
- You need to track progress toward a goal

## Key Concepts Summary

- Services provide synchronous request-response communication
- Actions are designed for long-running, goal-oriented tasks with feedback
- Each communication pattern has appropriate use cases
- Choosing the right pattern is crucial for effective robotic system design

## Next Steps

Now that you understand all three core communication patterns in ROS 2, you can design more sophisticated robotic systems. The next chapter will show you how to integrate these concepts with Python-based AI agents.