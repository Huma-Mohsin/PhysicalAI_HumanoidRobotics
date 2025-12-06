---
sidebar_position: 2
title: ROS 2 Topics
---

# ROS 2 Topics: Streaming AI-Physical Data

Topics enable continuous data streams between AI components and physical sensors/actuators.

## AI + Physical Integration

- **AI subscribes to sensor topics**: Process camera images, LiDAR scans in real-time
- **AI publishes to actuator topics**: Send velocity commands, joint positions continuously

### Example: Vision AI Processing Camera Stream

```python
# AI node subscribes to camera, processes with CV model, publishes detections
self.camera_sub = self.create_subscription(Image, '/camera/rgb', self.vision_callback, 10)
self.detection_pub = self.create_publisher(DetectionArray, '/detections', 10)
```

**Physical Execution**: Camera hardware → ROS 2 topic → AI vision model → Detection topic → Robot navigation

## Key Concepts
- Publisher/Subscriber pattern
- Message types (sensor_msgs, geometry_msgs)
- Topic QoS (Quality of Service) settings

See [ROS 2 documentation](https://docs.ros.org/en/humble/Tutorials/Topics.html) for detailed tutorials.
