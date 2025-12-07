---
title: ROS 2 Overview
---

## Introduction to ROS 2 (Robot Operating System 2)

ROS 2 is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors across a wide variety of robotic platforms. Unlike its predecessor, ROS 1, ROS 2 was redesigned with improvements in mind, focusing on real-time capabilities, multi-robot systems, and support for various operating systems beyond Linux, including Windows and macOS.

At its core, ROS 2 provides a structured communication layer that allows different components of a robot system to interact seamlessly. These components, known as nodes, can be developed independently in various programming languages (e.g., Python, C++) and communicate through a publish/subscribe model, service calls, and action-based interactions. This modularity is crucial for managing the complexity of modern robotic applications, enabling easier development, debugging, and deployment.

### Key Features of ROS 2
*   **Distributed Architecture:** Built for multi-robot and distributed computing environments.
*   **Real-time Support:** Enhanced capabilities for applications requiring precise timing.
*   **Quality of Service (QoS):** Configurable policies for reliability, latency, and throughput.
*   **Security:** Authentication, authorization, and encryption for communication.
*   **Tooling:** A rich ecosystem of development tools for visualization, debugging, and data logging.

### ROS 2 Communication Concepts
*   **Nodes:** Executable processes that perform computations (e.g., sensor driver, path planner).
*   **Topics:** Anonymous publish/subscribe message buses for data streaming.
*   **Services:** Request/response communication for synchronous interactions.
*   **Actions:** Long-running, goal-oriented tasks with feedback and preemption capabilities.

```
+---------+     Publish     +---------+
| Node A  |-------------->| Topic X |<--------------| Node B  |
| (Camera)|               +---------+               | (Vision)|
+---------+                     ^
                                |
                                v
+---------+     Request     +---------+     Response     +---------+
| Node C  |-------------->| Service Y |<--------------| Node D  |
| (Client)|               +---------+               | (Server)|
+---------+
```

### Summary
ROS 2 is an essential framework for developing advanced robotic systems, offering a modular, distributed, and real-time capable architecture. Its communication primitives and extensive tooling make it a powerful platform for humanoid robotics.
