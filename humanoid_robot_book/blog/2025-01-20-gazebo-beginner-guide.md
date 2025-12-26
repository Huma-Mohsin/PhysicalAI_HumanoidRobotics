---
slug: gazebo-beginner-guide
title: "Gazebo Simulation: Your First Robot in 10 Minutes"
authors: [robotics_team]
tags: [gazebo, tutorials, beginner]
---

Want to test robots without spending thousands on hardware? Gazebo lets you simulate robots for free on any platform. Here's your complete beginner's guide.

<!-- truncate -->

## Why Gazebo?

Gazebo is the **most popular robot simulator** for good reasons:

✅ **Free & Open Source** - No licensing costs
✅ **Cross-Platform** - Works on Linux, Mac, Windows (via Docker)
✅ **ROS 2 Integration** - Native support
✅ **Rich Ecosystem** - Thousands of robot models available
✅ **Physics Accuracy** - Multiple physics engines (ODE, Bullet, DART)

## Gazebo vs Other Simulators

| Feature | Gazebo | Isaac Sim | Webots | CoppeliaSim |
|---------|--------|-----------|---------|-------------|
| **Cost** | Free | Free | Free | Free/Paid |
| **Platform** | All | NVIDIA GPU only | All | All |
| **Graphics** | Basic | Photorealistic | Good | Good |
| **ROS 2** | ✅ Native | ✅ Via bridge | ✅ Native | ✅ Native |
| **Learning Curve** | Easy | Moderate | Easy | Moderate |
| **Best For** | General robotics | AI training | Education | Research |

**Verdict:** Start with Gazebo, move to Isaac Sim for advanced AI tasks.

## Installation (3 Methods)

### Method 1: Docker (Easiest - Works on Mac/Windows)

```bash
# Pull official ROS 2 + Gazebo image
docker pull osrf/ros:humble-desktop-full

# Run container
docker run -it --name ros2-gazebo \
  -v ~/robot_workspace:/workspace \
  osrf/ros:humble-desktop-full

# Inside container
gazebo
```

**Pros:** Works on any platform, no installation hassles
**Cons:** Slower graphics performance

### Method 2: Native Ubuntu (Best Performance)

```bash
# Ubuntu 22.04 only
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-gazebo-ros-pkgs

# Launch
gazebo
```

**Pros:** Best performance, full GPU acceleration
**Cons:** Linux only

### Method 3: Binary Install (Standalone)

```bash
# Install Gazebo Harmonic
curl -sSL http://get.gazebosim.org | sh

# Launch
gz sim
```

**Pros:** Works without ROS 2
**Cons:** No ROS 2 integration

## Your First Simulation (10 Minutes)

### Step 1: Launch Gazebo

```bash
# Terminal 1: Start Gazebo
gazebo
```

You'll see an empty world with a ground plane.

### Step 2: Insert a Robot

**Method A: GUI (Easiest)**
1. Click "Insert" tab (left panel)
2. Expand "https://fuel.gazebosim.org/..."
3. Search for "TurtleBot3"
4. Click to spawn in world

**Method B: SDF File (Programmatic)**

```xml
<!-- my_robot.sdf -->
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="my_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box><size>0.5 0.3 0.2</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.5 0.3 0.2</size></box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

Load it:
```bash
gazebo my_robot.sdf
```

### Step 3: Add Sensors

```xml
<!-- Add camera to robot -->
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.57</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  <update_rate>30</update_rate>
</sensor>
```

### Step 4: Control the Robot

**Via ROS 2 Topics:**

```bash
# Terminal 2: Publish velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

Your robot moves forward! 🚀

### Step 5: View Sensor Data

```bash
# View camera feed
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# View laser scan
ros2 run rviz2 rviz2
```

## Building a Complete Robot Simulation

### Project: Autonomous Delivery Robot

**Features:**
- 4-wheeled differential drive
- 2D LiDAR for obstacle detection
- RGB camera for object recognition
- GPS for localization

**Complete SDF:**

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="delivery_robot">
    <!-- Chassis -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.15</ixx><iyy>0.25</iyy><izz>0.35</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.6 0.4 0.2</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.6 0.4 0.2</size></box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
        </material>
      </visual>

      <!-- 2D LiDAR Sensor -->
      <sensor name="lidar" type="gpu_lidar">
        <pose>0 0 0.15 0 0 0</pose>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14</min_angle>
              <max_angle>3.14</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>10.0</max>
          </range>
        </lidar>
        <update_rate>10</update_rate>
      </sensor>

      <!-- RGB Camera -->
      <sensor name="camera" type="camera">
        <pose>0.3 0 0.1 0 0 0</pose>
        <camera>
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
          </image>
        </camera>
        <update_rate>30</update_rate>
      </sensor>
    </link>

    <!-- Left Wheel -->
    <link name="left_wheel">
      <pose>0 0.2 0.05 -1.57 0 0</pose>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.05</radius><length>0.05</length></cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.05</radius><length>0.05</length></cylinder>
        </geometry>
      </visual>
    </link>

    <!-- Right Wheel -->
    <link name="right_wheel">
      <pose>0 -0.2 0.05 -1.57 0 0</pose>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.05</radius><length>0.05</length></cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.05</radius><length>0.05</length></cylinder>
        </geometry>
      </visual>
    </link>

    <!-- Wheel Joints -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>left_wheel</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>

    <joint name="right_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>right_wheel</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>

    <!-- Differential Drive Plugin -->
    <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <command_topic>/cmd_vel</command_topic>
      <odometry_topic>/odom</odometry_topic>
    </plugin>
  </model>
</sdf>
```

### Launch & Test

```bash
# Launch simulation
gazebo delivery_robot.sdf

# Control robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# View LiDAR data
ros2 run rviz2 rviz2

# View camera
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

## Common Mistakes & Solutions

### Problem 1: Robot Falls Through Ground
**Cause:** Missing collision geometry
**Fix:** Add `<collision>` tag to all links

### Problem 2: Wheels Don't Move
**Cause:** Joint type wrong or no plugin
**Fix:** Use `type="revolute"` + differential drive plugin

### Problem 3: Simulation Runs Slow
**Cause:** Complex meshes or too many contacts
**Fix:** Use simple collision shapes (boxes, cylinders)

### Problem 4: Sensors Don't Publish Data
**Cause:** Missing ROS 2 plugins
**Fix:** Install `ros-humble-gazebo-ros-pkgs`

## Next Steps

1. ✅ **Complete Module 2** - Full Gazebo tutorial
2. 🛠️ **Build 3 Projects**:
   - Line-following robot
   - Maze solver
   - Object manipulation
3. 🚀 **Advanced Topics**:
   - Custom physics plugins
   - Sensor noise models
   - Multi-robot simulation

## Free Resources

- [Module 2: Gazebo & Unity](/docs/module-2-gazebo-unity)
- [Gazebo Tutorials](https://gazebosim.org/docs)
- [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- [Awesome Gazebo](https://github.com/fkromer/awesome-gazebo)

**Ready to build your first robot?** Start simulating now - it's free! 🤖
