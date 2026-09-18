---
sidebar_position: 6
title: Understanding URDF for Humanoids
---

# Understanding URDF for Humanoids

## Learning Objectives
- Define URDF (Unified Robot Description Format).
- Understand its XML structure for representing links and joints.
- Learn how to describe a simple humanoid component.

## What is URDF?
URDF is an XML-based format used to describe the kinematic and physical structure of a robot (links, joints, materials, collision properties).

## URDF XML Structure
- **Link:** Represents a physical part of the robot.
- **Joint:** Describes the connection between two links and how they move (e.g., revolute, prismatic).

## URDF Example Snippet
```xml
<link name="leg_link" />
<joint name="knee_joint" type="revolute">
  <parent link="thigh" />
  <child link="leg_link" />
  <axis xyz="0 1 0" />
</joint>
```

## Takeaways
- URDF is the standard way to model robot geometry in ROS 2.
- Links and joints form the tree structure of a robot.

## Review Questions
1. What is the difference between a `link` and a `joint` in URDF?
