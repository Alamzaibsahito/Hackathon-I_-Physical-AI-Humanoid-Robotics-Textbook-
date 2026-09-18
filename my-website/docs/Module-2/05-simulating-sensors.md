---
sidebar_position: 5
title: Simulating Sensors
---

# Simulating Sensors: LiDAR, Depth Cameras, and IMUs

## Learning Objectives
- Learn how to simulate common robot sensors in Gazebo.
- Understand the importance of sensor noise for sim-to-real transfer.

## Sensor Plugins
Gazebo uses sensor plugins (configured in SDF) to generate realistic data.

```xml
<sensor name="lidar" type="gpu_lidar">
  <plugin name="lidar_plugin" filename="libgz_sim_sensors_lidar.so"/>
</sensor>
```

## Sim-to-Real Transfer
Simulations are often "too perfect". To make AI agents work on real robots, you must add noise to simulated sensor data, forcing the agent to learn robustness against real-world sensor inaccuracies.

## Takeaways
- Sensor plugins are required to generate data in Gazebo.
- Adding noise is essential for successful sim-to-real transfer.

## Review Questions
1. Why is sensor noise important in simulation?
