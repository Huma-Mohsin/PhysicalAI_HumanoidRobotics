---
sidebar_position: 1
title: ROS 2 Nodes - AI Agents as Robot Controllers
---

# ROS 2 Nodes: AI Agents as Robot Controllers

## Introduction

A **ROS 2 node** is a process that performs computation. In Physical AI, nodes serve as the bridge between **AI reasoning** (LLMs, planners) and **physical robot execution** (motors, sensors).

## AI + Physical Integration: LLM-Driven Robot Node

### Architecture

```
[LLM Agent Node] ---(publishes)--> /cmd_vel topic ---> [Robot Base]
     ^
     |
(subscribes from) /sensor_data topic <--- [Robot Sensors]
```

The AI agent node:
1. **Receives** sensor data (camera images, LiDAR scans) via ROS 2 topics
2. **Reasons** using an LLM to decide next action ("move forward", "turn left")
3. **Publishes** velocity commands to physical robot actuators

### Python Implementation

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import openai

class LLMRobotController(Node):
    def __init__(self):
        super().__init__('llm_controller')
        
        # Publisher: Send commands to robot base
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber: Receive LiDAR data
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        # OpenAI client for LLM reasoning
        self.llm_client = openai.OpenAI(api_key="YOUR_API_KEY")
        
    def scan_callback(self, msg):
        """Process sensor data and use LLM to decide action."""
        # Extract obstacle distances
        front_distance = min(msg.ranges[0:10])  # Front sector
        
        # LLM reasoning: Decide action based on obstacle distance
        prompt = f"Robot sensor detects obstacle {front_distance:.2f}m ahead. Should I: (A) move forward, (B) stop, or (C) turn? Respond with one letter."
        
        response = self.llm_client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )
        
        action = response.choices[0].message.content.strip()
        
        # Physical execution: Translate LLM decision to robot command
        cmd = Twist()
        if action == "A":
            cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        elif action == "B":
            cmd.linear.x = 0.0  # Stop
        elif action == "C":
            cmd.angular.z = 0.5  # Turn left
            
        self.cmd_pub.publish(cmd)  # Send to robot!
        self.get_logger().info(f"LLM decided: {action}, sent command: {cmd}")

def main():
    rclpy.init()
    node = LLMRobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the AI Robot Controller

1. **Start ROS 2 and Gazebo simulation**:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. **Run the LLM controller node**:
```bash
python3 llm_controller.py
```

3. **Observe**: The robot uses GPT-4 to decide navigation actions based on real-time LiDAR sensor data!

## Key Concepts

### Node Lifecycle
- **Creation**: `Node.__init__()` initializes publishers and subscribers
- **Execution**: `rclpy.spin()` processes callbacks (sensor data → LLM reasoning → motor commands)
- **Shutdown**: `rclpy.shutdown()` cleans up resources

### Publishers vs. Subscribers
- **Publisher**: AI node sends commands to physical actuators (`/cmd_vel` topic)
- **Subscriber**: AI node receives data from physical sensors (`/scan` topic)

## AI + Physical Workflow

1. **Physical Sensing**: Robot's LiDAR scans environment → publishes to `/scan` topic
2. **AI Reasoning**: LLM node subscribes to `/scan` → processes data → decides action
3. **Physical Execution**: LLM node publishes to `/cmd_vel` → robot motors execute motion

This completes the **sense-reason-act** loop fundamental to Physical AI!

## Exercises

1. Modify the LLM prompt to handle different scenarios (narrow corridors, open spaces)
2. Add a camera subscriber and use GPT-4 Vision to recognize objects
3. Implement a multi-step task: "Navigate to the red object and stop 1m away"

## Next Chapter

Learn how to stream continuous sensor data and commands using [ROS 2 Topics](./topics.md).
