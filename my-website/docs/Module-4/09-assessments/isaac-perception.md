---
title: Isaac Perception
---

## Assessment: Real-time Perception with Isaac ROS

This assessment challenges you to implement a real-time perception pipeline for a humanoid robot using NVIDIA Isaac ROS. Your goal is to leverage Isaac ROS packages to perform a common perception task, such as stereo depth estimation or object detection, on simulated sensor data from NVIDIA Isaac Sim. This project will demonstrate your ability to integrate and utilize hardware-accelerated AI components within a ROS 2 framework.

You will be provided with a simulated humanoid robot in Isaac Sim that is equipped with appropriate cameras (e.g., stereo cameras for depth, or an RGB camera for object detection). Your task is to select an Isaac ROS perception module (e.g., `isaac_ros_stereo_image_proc` for depth, or `isaac_ros_detectnet` for object detection) and integrate it into your ROS 2 workspace. You will then configure the Isaac ROS node to process the simulated camera streams and publish its perception output (e.g., depth maps or detected bounding boxes) to new ROS 2 topics. The assessment emphasizes correct setup, configuration, and visualization of the real-time AI perception results.

### Project Requirements
*   Set up a ROS 2 workspace that integrates with NVIDIA Isaac Sim.
*   Launch a simulated humanoid robot in Isaac Sim with appropriate camera sensors.
*   Choose and integrate **one** Isaac ROS perception package (e.g., `isaac_ros_stereo_image_proc` or `isaac_ros_detectnet`).
*   Write a ROS 2 launch file to:
    *   Start Isaac Sim with your humanoid.
    *   Launch the Isaac ROS perception node, ensuring it subscribes to the correct camera topics from Isaac Sim.
*   Verify the output of the Isaac ROS node by:
    *   Subscribing to its output topic (e.g., `/stereo_camera/depth` for depth map, or `/detections` for object detection).
    *   Using RViz or a simple `rclpy` subscriber to visualize or log the processed perception data in real-time.
*   Provide a `README.md` detailing your setup, chosen Isaac ROS module, and demonstration of the perception output.

### Diagram: Isaac ROS Perception Pipeline

```
+------------------+     Camera Streams     +------------------+
| Isaac Sim        |------------------------>| Isaac ROS        |
| (Humanoid Robot  | (Image, Camera Info)   | Perception Node  |
|  with Cameras)   |                        | (Hardware-Acc. AI)|
+------------------+                        +--------^--------+
                                                     |
                                                     |
                                  Perception Output  |
                                (Depth Map / Detections)
                                                     |
                                                     v
+------------------+                                 +
| RViz /           |<---------------------------------+
| ROS 2 Subscriber |
| (Visualization)  |
+------------------+
```

### Summary
This assessment focuses on real-time perception with Isaac ROS, requiring you to integrate and configure an Isaac ROS perception module with a simulated humanoid robot in Isaac Sim. You will demonstrate the processing and visualization of AI-powered sensor data.
