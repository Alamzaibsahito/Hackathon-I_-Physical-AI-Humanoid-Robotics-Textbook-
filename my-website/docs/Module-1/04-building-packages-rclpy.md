---
sidebar_position: 4
title: Building ROS 2 Packages with rclpy
---

# Building ROS 2 Packages with rclpy

## Learning Objectives
- Learn how to structure a ROS 2 Python package.
- Bridge Python AI agents to ROS controllers.
- Create a full, working Python node.

## Package Structure
A standard ROS 2 package includes:
- `package.xml`: Package metadata.
- `setup.py`: Python package installation instructions.
- `src/`: Source code directory.

## Bridging AI to ROS
Use `rclpy` to wrap your AI agent (e.g., an LLM planning logic) in a ROS node that publishes commands to robot hardware topics.

## Takeaways
- `setup.py` is essential for ROS 2 Python package installation.
- Nodes act as the bridge between software logic and hardware actions.

## Review Questions
1. What is the purpose of `package.xml`?
