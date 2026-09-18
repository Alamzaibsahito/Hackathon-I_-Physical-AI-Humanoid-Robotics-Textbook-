---
title: rclpy Basics
---

## Python Client Library for ROS 2

rclpy is the Python client library for ROS 2, providing a clean and intuitive API for interacting with the ROS 2 ecosystem. It allows developers to write ROS 2 nodes, publishers, subscribers, service clients, and service servers using Python, making it an excellent choice for rapid prototyping, educational purposes, and developing high-level robot behaviors. rclpy wraps the underlying C++ client library (rcl), providing Pythonic access to ROS 2 functionalities.

Developing with rclpy typically involves creating a `Node` class, which serves as the main entry point for your robot's components. Within this node, you can define publishers to send data, subscribers to receive data, and implement service clients and servers for synchronous communication. rclpy handles the intricate details of inter-process communication, message serialization, and quality of service settings, allowing developers to focus on the application logic.

### Key rclpy Concepts
*   **Node:** The fundamental unit of computation, inheriting from `rclpy.node.Node`.
*   **Publisher:** Sends messages to a topic using `create_publisher()`.
*   **Subscriber:** Receives messages from a topic using `create_subscription()`.
*   **Service Client:** Makes requests to a service using `create_client()`.
*   **Service Server:** Responds to service requests using `create_service()`.
*   **rclpy.spin():** Blocks until the node is shut down, processing callbacks.

### Code Example: Simple Subscriber (Python - rclpy)

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
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Summary
rclpy is the Python client library for ROS 2, enabling easy development of robot applications using Python. It provides classes and functions for creating nodes and managing ROS 2 communication patterns like topics and services.
