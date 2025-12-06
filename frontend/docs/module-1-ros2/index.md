---
sidebar_position: 2
title: Module 1 - ROS 2 Fundamentals
---

# Module 1: ROS 2 Fundamentals

**Duration**: Weeks 3-5  
**Prerequisites**: Basic Python programming, command-line familiarity

## Overview

ROS 2 (Robot Operating System 2) is the industry-standard middleware for robot software development. This module teaches you how to build distributed robot systems where **AI reasoning components** communicate with **physical robot hardware** through a standardized message-passing framework.

## Learning Objectives

By the end of this module, you will:

- Understand ROS 2 architecture: nodes, topics, services, actions
- Build AI-driven robot controllers using rclpy (ROS 2 Python client)
- Integrate LLM-based decision-making with physical robot commands
- Simulate and control robots in Gazebo using ROS 2 interfaces

## AI + Physical Integration

Every chapter in this module demonstrates how AI reasoning connects to physical robot execution:

- **Nodes**: LLM agents as ROS 2 nodes that publish control commands
- **Topics**: Streaming sensor data to AI models, streaming commands to actuators
- **Services**: Request/response AI inference integrated with robot state queries
- **URDF**: Describing physical robot geometry for AI path planning

## Hardware Context

### Workstation Setup
- **OS**: Ubuntu 22.04 LTS (mandatory for ROS 2 Humble)
- **ROS 2 Distribution**: Humble Hawksbill (LTS)
- **Simulation**: Gazebo Classic or Gazebo Fortress

### Cloud/Mac Alternative
- **Cloud**: Use Docker containers with ROS 2 on AWS/GCP
- **Mac**: Limited support; use Docker for ROS 2 environment

## Chapters

1. **ROS 2 Nodes**: Build AI agent nodes that control robot behavior
2. **Topics & Publishers/Subscribers**: Stream sensor data to AI, stream commands to motors
3. **Services**: Synchronous AI inference for robot state queries
4. **URDF & Robot Description**: Define physical robot models for AI planning

## Next Steps

Start with [ROS 2 Nodes](./nodes.md) to learn how to create AI-driven robot controllers.
