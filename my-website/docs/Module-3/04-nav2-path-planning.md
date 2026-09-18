---
sidebar_position: 4
title: Nav2 and Path Planning for Humanoids
---

# Nav2 and Path Planning for Humanoids

## Learning Objectives
- Understand the basics of Nav2.
- Contrast path planning for bipedal robots vs. wheeled robots.
- Learn about simple goal-based navigation.

## What is Nav2?
Nav2 (Navigation 2) is the standard ROS 2 navigation stack. It provides a modular pipeline for moving a robot from A to B while avoiding obstacles.

## Path Planning: Bipedal vs. Wheeled
- **Wheeled Robots:** Generally follow 2D or 2.5D paths on flat surfaces. Simple, holonomic constraints.
- **Humanoids (Bipedal):** Must handle complex terrain, balance, and center-of-mass constraints. Planning for humanoids often involves full-body motion planning, not just planar pathfinding.

## Navigation Goal Example
```python
# Simple Python snippet to send a goal pose to Nav2
# (Requires Nav2 Action Clients)
goal_pose = PoseStamped()
goal_pose.pose.position.x = 2.0
# ...
client.send_goal(goal_pose)
```

## Takeaways
- Nav2 is powerful, but humanoid planning requires accounting for balance and complex kinematics.
- Navigation in robotics is always a multi-layered problem (global planning vs. local control).

## Review Questions
1. How does planning for a humanoid differ from planning for a wheeled robot?
