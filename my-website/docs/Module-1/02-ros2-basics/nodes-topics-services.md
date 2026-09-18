---
title: Nodes, Topics, and Services
---

## ROS 2 Communication Primitives

In ROS 2, the fundamental building blocks for communication between different components of a robot system are **Nodes**, **Topics**, and **Services**. Understanding these primitives is crucial for designing and implementing robust robotic applications. Nodes are essentially processes that perform computation, such as reading sensor data, processing images, or controlling motors. Each node should be designed to do one logical task well.

**Topics** provide a flexible publish/subscribe mechanism for asynchronous data streaming. A node can publish messages to a topic, and any other node that subscribes to that topic will receive those messages. This is ideal for continuous data flows like sensor readings, joint states, or odometry. The data sent over topics are typed messages, ensuring data consistency across the system.

**Services**, on the other hand, offer a synchronous request/response pattern. When a node requires a specific action or piece of information from another node, it can make a service call. The serving node processes the request and sends back a response. This is suitable for actions that have a clear start and end, such as triggering a robot arm to move to a specific position or requesting a map of the environment.

### Key Differences
*   **Topics:** Asynchronous, one-to-many, streaming data (e.g., camera feed).
*   **Services:** Synchronous, one-to-one, request-response (e.g., take a picture).

### Diagram: ROS 2 Communication

```
+----------+   Publishes   +---------+   Subscribes   +----------+
| Camera   |--------------->| /image  |<---------------| Image    |
| Node     |   (Image msg)  +---------+   (Image msg)  | Processor|
+----------+                 (Topic)                 +----------+

+----------+   Requests    +---------+   Responds     +----------+
| Control  |--------------->| /move_arm |<---------------| Arm      |
| Node     | (MoveArm req)  +---------+ (MoveArm resp) | Controller|
+----------+                (Service)                +----------+
```

### Code Example: Simple Publisher (Python - rclpy)

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
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
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

### Summary
Nodes, Topics, and Services are the foundational communication mechanisms in ROS 2. Topics facilitate continuous data streams, while services enable synchronous request-response interactions, providing a robust framework for complex robotic systems.
