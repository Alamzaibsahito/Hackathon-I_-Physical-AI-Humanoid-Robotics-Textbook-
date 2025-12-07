---
title: "Sensors: Lidar & Depth"
---

## Essential Sensors for Robot Perception

For humanoid robots to interact intelligently with the physical world, accurate perception of their surroundings is paramount. Two of the most critical sensor technologies for environmental understanding are **Lidar (Light Detection and Ranging)** and **Depth Cameras**. These sensors provide robots with the ability to measure distances to objects, build 3D maps of their environment, detect obstacles, and even recognize objects, forming the basis for navigation, manipulation, and safe human-robot interaction.

**Lidar** sensors emit pulsed laser light and measure the time it takes for the light to return, calculating distances to surfaces. They typically generate a 2D or 3D point cloud, providing highly accurate geometric information about the environment, largely unaffected by ambient light conditions. Lidar is extensively used for mapping (SLAM), localization, and obstacle avoidance in autonomous systems.

**Depth Cameras** (e.g., Intel RealSense, Microsoft Azure Kinect) capture not only color images but also per-pixel depth information. They achieve this using technologies like structured light, time-of-flight (ToF), or stereo vision. Depth cameras are excellent for short-range 3D perception, particularly useful for tasks like object recognition, gesture tracking, and close-quarters manipulation where dense depth information is beneficial.

### Sensor Characteristics
*   **Lidar:**
    *   **Pros:** Long range, high accuracy, robust to lighting, good for mapping.
    *   **Cons:** Can be expensive, sparse data, sometimes struggles with transparent surfaces.
*   **Depth Camera:**
    *   **Pros:** Dense depth data, compact, often combined with RGB, good for object recognition.
    *   **Cons:** Limited range, sensitive to ambient light, can suffer from occlusion.

### Diagram: Lidar and Depth Camera Concepts

```
+---------+        Laser Pulses        +---------+
| Lidar   |--------------------------->| Scene   |
| Sensor  | (Time-of-Flight/Reflection)| (Objects)|
+---------+<---------------------------+---------+
                 (Distance Data)

+------------+    IR Pattern / Stereo    +---------+
| Depth      |------------------------->| Scene   |
| Camera     | (Structured Light/ToF)   | (Objects)|
+------------+<-------------------------+---------+
               (Depth Map + RGB)
```

### Code Example: Reading Lidar Scan (ROS 2 - rclpy)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Lidar Scan with {len(msg.ranges)} ranges.')
        # Access range data: msg.ranges[i] for angle_min + i * angle_increment
        # Example: Find the minimum range
        min_range = min(msg.ranges)
        self.get_logger().info(f'Minimum detected range: {min_range:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Summary
Lidar and depth cameras are fundamental sensor technologies for humanoid robots, providing crucial 3D environmental perception. Lidar excels at accurate long-range mapping, while depth cameras offer dense, short-range depth information vital for object interaction.
