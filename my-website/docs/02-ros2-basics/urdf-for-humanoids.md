---
title: URDF for Humanoids
---

## Unified Robot Description Format (URDF)

URDF (Unified Robot Description Format) is an XML format used in ROS to describe all aspects of a robot. It allows you to model the robot's kinematics (joints and links), collision properties, visual representation, and even its inertial characteristics. For humanoid robots, URDF is crucial for accurately representing their complex articulated structure, which is essential for simulation, motion planning, and control.

Each URDF file defines a robot as a tree of **links** (rigid bodies) connected by **joints** (revolute, prismatic, fixed, etc.). Links have geometric and inertial properties, while joints define the degrees of freedom and movement constraints between links. A well-designed URDF is not only human-readable but also easily parsed by various robotics software tools, enabling seamless integration with simulators like Gazebo, visualization tools like RViz, and motion planning libraries.

### Key URDF Elements
*   **`<link>`:** Defines a rigid body segment of the robot. Contains visual, collision, and inertial properties.
*   **`<joint>`:** Defines the kinematic and dynamic properties of the connection between two links. Specifies type, axis, limits, and origin.
*   **`<robot>`:** The root element that encapsulates all links and joints.
*   **`<material>`:** Specifies visual properties like color.

### Diagram: Simple Link and Joint

```
+-----------+    Joint (Revolute)    +-----------+
| Base Link |------------------------| End Link  |
+-----------+                        +-----------+
    (Fixed)                                (Rotates)
```

### Code Example: Basic URDF Snippet (Simplified)

```xml
<?xml version="1.0" ?>
<robot name="simple_humanoid_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <link name="forearm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </visual>
  </link>

  <joint name="shoulder_pitch_joint" type="revolute">
    <parent link="base_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
  </joint>
</robot>
```

### Summary
URDF is a critical format for describing the physical and kinematic properties of robots in ROS 2, especially for complex humanoid structures. It enables accurate simulation, visualization, and control, forming the basis for many advanced robotics applications.
