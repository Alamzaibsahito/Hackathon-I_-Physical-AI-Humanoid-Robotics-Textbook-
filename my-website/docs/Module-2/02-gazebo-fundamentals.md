---
sidebar_position: 2
title: Gazebo Fundamentals
---

# Gazebo Fundamentals

## Learning Objectives
- Set up the Gazebo environment.
- Understand how Gazebo communicates with ROS 2.
- Configure basic physics (gravity, collisions).

## Gazebo and ROS 2
Gazebo simulates the environment and robot sensors, while ROS 2 handles the control logic. The `ros_gz` packages provide the bridge between the two, allowing ROS 2 to send movement commands to Gazebo's motors and receive sensor data back.

## Physics Configuration
You can adjust world properties in the `.sdf` or `.world` file:

```xml
<world name="default">
  <physics type="ode">
    <gravity>0 0 -9.81</gravity>
  </physics>
</world>
```

## Takeaways
- Gazebo provides the physical world; ROS 2 provides the "brain".
- World files define the environment parameters.

## Review Questions
1. How does ROS 2 receive data from Gazebo?
2. Why configure gravity in the simulation world?
