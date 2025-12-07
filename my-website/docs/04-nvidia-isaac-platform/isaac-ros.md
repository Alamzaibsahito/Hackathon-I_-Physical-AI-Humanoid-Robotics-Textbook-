---
title: Isaac ROS
---

## Accelerating ROS 2 with NVIDIA Isaac ROS

NVIDIA Isaac ROS is a collection of hardware-accelerated packages that bring NVIDIA's advanced AI capabilities to the ROS 2 ecosystem. It provides optimized components for various robotics tasks, including perception, navigation, and manipulation, leveraging the power of NVIDIA GPUs and Jetson platforms. Isaac ROS modules are designed to be high-performance, enabling real-time AI inference directly on edge devices, which is critical for autonomous humanoid robots operating in dynamic environments.

Isaac ROS offers pre-trained models and highly optimized algorithms for common robotics functions like stereo depth estimation, object detection, pose estimation, and visual SLAM. By using these optimized components, developers can significantly reduce the development time and improve the performance of their ROS 2 applications. This allows humanoid robots to process large amounts of sensor data quickly, make informed decisions, and execute complex actions with low latency.

### Key Isaac ROS Modules
*   ** percepción:** Stereo matching, object detection, image segmentation.
*   ** navegación:** Visual SLAM (vSLAM), odometry, path planning.
*   ** Manipulación:** Grasping, inverse kinematics, motion control.
*   ** isaac_ros_common:** Core utilities and frameworks.

### Benefits for Humanoid Robotics
*   **Real-time Performance:** Enables complex AI tasks to run at high frame rates on edge devices.
*   **Accelerated Development:** Pre-built and optimized components reduce implementation effort.
*   **Synthetic Data Training:** Leverages Isaac Sim for data generation to train robust models.
*   **Ecosystem Integration:** Seamlessly integrates with ROS 2 and NVIDIA Jetson hardware.

### Diagram: Isaac ROS in the Robotics Stack

```
+-------------+
| AI Models   |
| (Trained)   |
+------^------+
       |
       v
+------------------+
| Isaac ROS Modules|
| (Hardware-Acc.)  |
+--------^---------+
         |
         v
+------------------+
| ROS 2 Nodes      |
| (Perception,     |
|  Navigation)     |
+--------^---------+
         |
         v
+------------------+
| Humanoid Robot   |
| (Physical System)|
+------------------+
```

### Code Example: Using an Isaac ROS Node (Conceptual ROS 2 Launch)

```xml
<!-- This is a conceptual ROS 2 launch file for an Isaac ROS node -->

<launch>
  <node pkg="isaac_ros_stereo_image_proc" exec="stereo_image_proc_node" name="stereo_image_proc">
    <param name="use_sim_time" value="true"/>
    <remap from="left/image_rect" to="/stereo_camera/left/image_rect"/>
    <remap from="right/image_rect" to="/stereo_camera/right/image_rect"/>
    <remap from="left/camer-info" to="/stereo_camera/left/camer-info"/>
    <remap from="right/camer-info" to="/stereo_camera/right/camer-info"/>
    <remap from="stereo_image_proc/depth" to="/stereo_camera/depth"/>
    <remap from="stereo_image_proc/points" to="/stereo_camera/points"/>
  </node>
</launch>
```

### Summary
NVIDIA Isaac ROS provides a suite of GPU-accelerated packages that significantly enhance ROS 2 applications for humanoid robots. It enables real-time AI perception and navigation on edge devices, streamlining development and boosting performance.
