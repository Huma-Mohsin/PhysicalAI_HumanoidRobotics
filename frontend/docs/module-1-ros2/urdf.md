---
sidebar_position: 4
title: URDF - Robot Description
---

# URDF: Describing Physical Robots for AI Planning

URDF (Unified Robot Description Format) defines the physical geometry and kinematics of robots, enabling AI to plan motions.

## AI + Physical Integration

- **AI uses URDF for path planning**: Avoid self-collisions, plan feasible trajectories
- **Physical robot matches URDF model**: Accurate simulation-to-reality transfer

### Example: Robot Arm URDF with AI Planner

```xml
<robot name="ai_arm">
  <link name="base_link">
    <visual>
      <geometry><cylinder length="0.1" radius="0.05"/></geometry>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <limit effort="30" velocity="1.0" lower="-3.14" upper="3.14"/>
  </joint>
</robot>
```

**AI Integration**: MoveIt motion planner reads URDF → Plans collision-free path → Sends joint commands to physical robot

## Key Concepts
- Links and joints
- Collision geometry
- URDF + AI motion planning (MoveIt, OMPL)

See [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html).
