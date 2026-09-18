---
sidebar_position: 3
title: URDF and SDF for Simulation
---

# URDF and SDF for Simulation

## Learning Objectives
- Differentiate between URDF and SDF.
- Understand how robot models are loaded into Gazebo.

## URDF vs. SDF
- **URDF (Unified Robot Description Format):** Simple, primarily for kinematic modeling, widely supported in ROS.
- **SDF (Simulation Description Format):** Robust, designed specifically for simulation, supports features like world settings, multiple robots, and sensor plugins.

## Loading Models
Gazebo uses SDF. When you pass a URDF to Gazebo, ROS 2 conversion tools typically convert it to SDF on-the-fly.

## Takeaways
- Use URDF for design/robotics logic.
- Use SDF for full environment simulation configuration.

## Review Questions
1. Why does Gazebo prefer SDF over URDF?
