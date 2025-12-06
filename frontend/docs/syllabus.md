---
sidebar_position: 6
title: 13-Week Syllabus
---

# 13-Week Syllabus: Physical AI & Humanoid Robotics

This syllabus structures your learning journey from ROS 2 fundamentals to building conversational humanoid robots.

## Course Structure

| Week | Module | Topics | AI + Physical Integration | Hardware |
|------|--------|--------|---------------------------|----------|
| **1-2** | **Introduction** | Physical AI concepts, Setup | LLM agents vs. traditional control | Ubuntu 22.04 setup |
| **3** | **Module 1: ROS 2** | Nodes, Topics | LLM nodes controlling simulated robots | ROS 2 Humble |
| **4** | **Module 1: ROS 2** | Services, Actions | Synchronous AI inference for robot state | Gazebo |
| **5** | **Module 1: ROS 2** | URDF, Robot Description | AI motion planning with MoveIt | TurtleBot3 sim |
| **6** | **Module 2: Digital Twin** | Gazebo Physics, Sensors | RL agents trained in simulation | Gazebo Fortress |
| **7** | **Module 2: Digital Twin** | Unity Rendering, Sim-to-Real | Transfer learned policies to real robots | Unity + ROS 2 |
| **8** | **Module 3: Isaac Sim** | USD Workflows, PhysX | Synthetic data generation for AI vision | RTX 4070 Ti+ |
| **9** | **Module 3: Isaac Sim** | Isaac ROS, Perception | Accelerated AI perception pipelines | Isaac Sim 2023.1 |
| **10** | **Module 4: VLA** | Voice-to-Action | Whisper + GPT-4 → Robot commands | Jetson Orin (edge) |
| **11** | **Module 4: VLA** | Cognitive Planning | LLM task decomposition → Execution | ROS 2 + OpenAI |
| **12** | **Module 4: VLA** | Multimodal Perception | Vision + Language → Physical actions | Camera + Microphone |
| **13** | **Capstone Project** | Build Conversational Robot | Voice → AI → Physical execution | Full stack integration |

## Week-by-Week Breakdown

### Weeks 1-2: Introduction to Physical AI

**Learning Objectives**:
- Understand the difference between software-only AI and Physical AI
- Set up development environment (Ubuntu 22.04, ROS 2, Python)
- Learn the sense-reason-act loop

**AI + Physical Integration**: Explore examples of LLMs controlling simulated robots vs. rule-based control systems.

**Hardware Setup**:
- Install Ubuntu 22.04 LTS (dual boot or VM)
- Install ROS 2 Humble
- Set up Python 3.10+ with OpenAI SDK

---

### Weeks 3-5: Module 1 - ROS 2 Fundamentals

**Week 3: Nodes and Topics**
- **AI**: LLM agents as ROS 2 nodes
- **Physical**: Publishing velocity commands to robot base
- **Integration**: LLM subscribes to sensor topic → reasons → publishes to actuator topic

**Week 4: Services and Actions**
- **AI**: Synchronous AI inference services
- **Physical**: Request/response with robot state queries
- **Integration**: Robot calls LLM planning service → receives trajectory → executes motion

**Week 5: URDF and Motion Planning**
- **AI**: MoveIt motion planner with collision avoidance
- **Physical**: Robot arm geometry defined in URDF
- **Integration**: AI reads URDF → plans path → sends joint commands to physical actuators

**Deliverable**: LLM-controlled TurtleBot3 navigating a Gazebo environment

---

### Weeks 6-7: Module 2 - Digital Twin

**Week 6: Gazebo Physics Simulation**
- **AI**: Reinforcement learning agent (PPO, SAC)
- **Physical**: Simulated sensors (LiDAR, camera, IMU) and actuators
- **Integration**: Train RL policy in Gazebo → Deploy to physical robot

**Week 7: Unity Rendering and Sim-to-Real**
- **AI**: Domain randomization for robust transfer
- **Physical**: Match simulation physics to real robot dynamics
- **Integration**: Train vision model in Unity → Test on real robot camera

**Deliverable**: RL-trained robot policy transferring from Gazebo to physical hardware

---

### Weeks 8-9: Module 3 - NVIDIA Isaac Sim

**Week 8: Isaac Sim Setup and USD Workflows**
- **AI**: Synthetic data generation for object detection
- **Physical**: Photorealistic rendering of robot environments
- **Integration**: Generate 10,000 synthetic RGB-D images → Train YOLO model → Deploy to real robot

**Week 9: Isaac ROS Perception Pipelines**
- **AI**: Accelerated AI inference with TensorRT
- **Physical**: Real-time sensor processing (30 FPS camera stream)
- **Integration**: Isaac ROS DNN node → GPU-accelerated inference → Robot navigation

**Hardware Requirement**: RTX 4070 Ti+ or Omniverse Cloud

**Deliverable**: Vision-based robot grasping trained entirely in Isaac Sim, deployed to real arm

---

### Weeks 10-13: Module 4 - Vision-Language-Action Models

**Week 10: Voice-to-Action Pipeline**
- **AI**: Whisper (speech recognition) + GPT-4 (task planning)
- **Physical**: ROS 2 action server executing robot commands
- **Integration**: Voice command → Whisper transcription → GPT-4 plans → ROS 2 executes

**Week 11: Cognitive Planning with LLMs**
- **AI**: GPT-4 task decomposition (high-level goals → low-level actions)
- **Physical**: Robot executes multi-step tasks (pick, place, navigate)
- **Integration**: "Clean the table" → LLM breaks down → Robot executes subtasks

**Week 12: Multimodal Perception**
- **AI**: GPT-4 Vision processes camera images
- **Physical**: Robot uses visual feedback to adjust grasp
- **Integration**: Camera stream → GPT-4V describes scene → Robot adapts behavior

**Week 13: Capstone Project**
Build a fully conversational robot:
- **Input**: Natural language voice commands
- **AI Reasoning**: LLM plans and adapts based on visual feedback
- **Physical Execution**: Robot arm + mobile base execute tasks in real environment

**Deliverable**: Demonstration video of robot responding to conversational commands

**Edge Deployment**: Deploy final system on Jetson Orin Nano for real-time inference

---

## Learning Outcomes

By the end of this 13-week course, you will be able to:

1. ✅ **Design** Physical AI systems that integrate LLMs with robot hardware
2. ✅ **Implement** ROS 2 nodes that bridge AI reasoning and physical actuators
3. ✅ **Simulate** robots in Gazebo and Isaac Sim before deploying to real hardware
4. ✅ **Train** AI models (RL, vision, language) using synthetic data
5. ✅ **Deploy** conversational robot systems on edge devices (Jetson Orin Nano)
6. ✅ **Transfer** learned policies from simulation to physical robots

---

## Assessment & Projects

- **Weekly Labs**: Hands-on coding exercises (AI + Physical integration)
- **Module Projects**: End-of-module demonstrations
- **Capstone**: Conversational robot demonstration with full voice → AI → physical execution pipeline

---

## Prerequisites

- Python programming (intermediate level)
- Basic robotics concepts (optional but helpful)
- Ubuntu 22.04 LTS (required for ROS 2 Humble)
- RTX 4070 Ti+ GPU (recommended for Isaac Sim) OR Cloud access

---

**Ready to start?** Begin with [Module 1: ROS 2 Fundamentals](./module-1-ros2/index.md).
