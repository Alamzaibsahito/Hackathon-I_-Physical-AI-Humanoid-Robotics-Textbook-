---
sidebar_position: 3
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services

## Learning Objectives
- Master the Publisher/Subscriber pattern (Topics).
- Understand the Service/Client pattern.
- Implement simple ROS 2 nodes in Python.

## Topics (Publisher/Subscriber)
Topics are buses for data exchange. Publishers send messages, and Subscribers receive them.

```python
# Simple Publisher snippet
import rclpy
from std_msgs.msg import String

node = rclpy.create_node('my_publisher')
pub = node.create_publisher(String, 'chatter', 10)
pub.publish(String(data='Hello, ROS 2!'))
```

## Services (Service/Client)
Services are request/reply interactions. A client sends a request, and the server returns a response.

## Takeaways
- Topics are for continuous data streams.
- Services are for discrete request/response calls.

## Review Questions
1. When should you use a Topic instead of a Service?
