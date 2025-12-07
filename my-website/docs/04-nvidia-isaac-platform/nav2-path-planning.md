---
title: Nav2 Path Planning
---

## Autonomous Navigation with ROS 2 Nav2

Nav2 is the revamped navigation stack for ROS 2, providing a complete framework for autonomous mobile robot navigation. It enables robots to safely and intelligently navigate from a starting point to a goal location, avoiding obstacles and adhering to specific operational constraints. For humanoid robots, Nav2 can be adapted to handle complex locomotion, allowing them to plan paths in human-centric environments, traverse stairs, and navigate through cluttered spaces. It builds upon years of research and development from the original ROS Navigation Stack, optimized for ROS 2's distributed and real-time capabilities.

The Nav2 stack consists of several modular components, including localization, global planning, local planning, and obstacle avoidance. The global planner computes a high-level path to the goal, while the local planner continuously adjusts the robot's trajectory to follow the global path and avoid dynamic obstacles in real-time. Nav2 also incorporates behavior trees for flexible and robust decision-making, allowing for complex navigation behaviors and recovery strategies.

### Key Nav2 Components
*   **amcl (Adaptive Monte Carlo Localization):** Estimates the robot's pose within a known map.
*   **map_server:** Provides the environment map to other navigation components.
*   **Global Planner (e.g., A*, Dijkstra):** Computes a collision-free path from start to goal.
*   **Local Planner (e.g., DWB, TEB):** Generates short-term trajectories for obstacle avoidance.
*   **BT Navigator (Behavior Tree):** Orchestrates the various navigation behaviors and recovery actions.

### Diagram: Simplified Nav2 Architecture

```
+---------+     Goal       +--------------+
| User/   |---------------->| BT Navigator |
| High-Lvl|                | (Orchestrates)|
| Control |                +------^-------+
+---------+                       |
                                  v
+-----------+    Path Request   +-----------+
| Global    |<------------------| Local     |
| Planner   |                   | Planner   |
+-----------+------------------>+-----------+
     ^        (Global Path)           |
     |                              (Vel Cmds)
     |                              v
+-----------+                   +---------+
| Map Server|<------------------| Robot   |
| (Map)     |                   | (Actuators)|
+-----------+
     ^                         ^
     |                         |
     +-------------------------+
          (Robot Pose / Odometry)
```

### Code Example: Sending a Nav2 Goal (ROS 2 - rclpy)

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # Set orientation (simplified for example)
        goal_msg.pose.pose.orientation.w = 1.0 # Facing forward

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: ({x}, {y}, {yaw})')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.outcome}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalSender()
    node.send_goal(5.0, 2.0, 0.0) # Example goal: (x=5.0, y=2.0, yaw=0.0)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### Summary
Nav2 provides a comprehensive and modular framework for autonomous navigation in ROS 2, crucial for humanoid robots to plan paths, avoid obstacles, and reach goals in complex environments. Its flexible architecture supports various planning and control strategies.
