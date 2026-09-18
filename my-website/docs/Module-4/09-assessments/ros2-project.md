---
title: ROS 2 Project
---

## Assessment: Building a ROS 2 Humanoid Controller

This assessment challenges you to apply your knowledge of ROS 2 fundamentals to develop a basic controller for a simulated humanoid robot. The goal is to create a set of ROS 2 nodes that can publish commands to control the robot's joints and subscribe to sensor feedback, demonstrating your understanding of topics, messages, and node architecture. You will be provided with a simplified URDF model of a humanoid arm and a Gazebo simulation environment.

Your task will involve creating a Python-based ROS 2 package using `rclpy`. You need to implement a publisher node that sends joint commands (e.g., using `sensor_msgs/JointState` messages) to make the humanoid arm move to a series of predefined poses. Additionally, you will create a subscriber node to monitor the arm's actual joint states, providing feedback on the control performance. This project will reinforce your understanding of ROS 2 communication, URDF integration with Gazebo, and basic robot control.

### Project Requirements
*   Create a new ROS 2 Python package.
*   Implement a **`JointCommandPublisher`** node that:
    *   Publishes `sensor_msgs/JointState` messages to a `/joint_commands` topic.
    *   Cycles through at least three distinct arm poses with a delay between each.
    *   Utilizes `rclpy.timer` for periodic command publishing.
*   Implement a **`JointStateSubscriber`** node that:
    *   Subscribes to the `/joint_states` topic.
    *   Logs the received joint positions and velocities.
    *   Identifies when the robot has reached a target pose (within a small tolerance).
*   Write a ROS 2 launch file to start both your nodes and the provided Gazebo simulation.
*   Document your code with comments and a `README.md` explaining the package structure and how to run it.

### Diagram: ROS 2 Controller Architecture

```
+------------------+     /joint_commands    +------------------+
| Joint Command    |------------------------>| Gazebo           |
| Publisher Node   | (sensor_msgs/JointState)| Simulator        |
+------------------+                        | (Humanoid Arm)   |
                                            +--------^--------+
                                                     |
                                                     |
                                /joint_states        |
                               (sensor_msgs/JointState)
                                                     |
                                                     v
+------------------+                                 +
| Joint State      |<---------------------------------+
| Subscriber Node  |
+------------------+
```

### Summary
This ROS 2 project assesses your ability to build a basic humanoid arm controller. You will create publisher and subscriber nodes, integrate with Gazebo, and demonstrate fundamental ROS 2 communication and control concepts.
