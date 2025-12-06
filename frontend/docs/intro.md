---
sidebar_position: 1
title: Welcome
---

import { EnvironmentSwitcher } from '@site/src/components/EnvironmentSwitcher';

# Welcome to Physical AI & Humanoid Robotics

Welcome to the comprehensive learning platform for **Physical AI** - the intersection of artificial intelligence and physical robotics. This course will guide you through a 13-week journey from ROS 2 fundamentals to building intelligent, conversational humanoid robots.

<EnvironmentSwitcher />

## What is Physical AI?

Physical AI represents the next frontier in artificial intelligence: **embodied intelligence** that can sense, reason, and act in the physical world. Unlike traditional AI that exists purely in software, Physical AI:

- **Perceives** the environment through sensors (cameras, LiDAR, IMUs)
- **Reasons** using AI models (LLMs, vision-language models, planning algorithms)
- **Acts** through physical actuators (motors, grippers, mobile bases)

## Course Overview

This platform covers four essential modules aligned with a **13-week syllabus**:

### Module 1: ROS 2 Fundamentals (Weeks 3-5)
Learn the Robot Operating System 2 (ROS 2) - the industry-standard framework for robot software development. Master nodes, topics, services, and how to integrate Python-based AI controllers with physical robot communication.

**AI + Physical Integration Example**: Control a simulated robot arm using LLM-generated motion commands sent via ROS 2 topics.

### Module 2: Digital Twin (Weeks 6-7)
Build virtual replicas of physical robots using Gazebo and Unity. Learn how to simulate sensors, physics, and environment interactions before deploying to real hardware.

**AI + Physical Integration Example**: Train a reinforcement learning agent in Gazebo simulation, then deploy the learned policy to a physical robot.

### Module 3: NVIDIA Isaac Sim (Weeks 8-9)
Explore photorealistic, physically-accurate simulation using NVIDIA Isaac Sim. Learn USD workflows, PhysX engine, and sim-to-real transfer techniques for vision-based AI.

**AI + Physical Integration Example**: Use Isaac Sim to generate synthetic training data for a vision-language model that guides robot manipulation tasks.

### Module 4: Vision-Language-Action (VLA) Models (Weeks 10-13)
Integrate large language models with robot perception and control. Build conversational robots that can understand natural language commands and execute them through physical actions.

**AI + Physical Integration Example**: Deploy a Whisper + GPT-4 + ROS 2 pipeline that listens to voice commands, reasons about task execution, and controls a robot arm to perform pick-and-place operations.

## Learning Path

Our 13-week syllabus is designed for progressive learning:

1. **Weeks 1-2**: Introduction to Physical AI concepts and setup
2. **Weeks 3-5**: ROS 2 fundamentals and robot communication
3. **Weeks 6-7**: Gazebo simulation and digital twin workflows
4. **Weeks 8-9**: NVIDIA Isaac Sim and photorealistic simulation
5. **Weeks 10-13**: Vision-Language-Action models and conversational robotics

## Platform Features

### 🤖 Interactive RAG Chatbot
Ask questions about any content in this book. Our AI-powered chatbot uses retrieval-augmented generation (RAG) to provide accurate, context-aware answers with citations.

**Try it**: Click the chat icon and ask "How do ROS 2 nodes communicate?"

### 🎯 Context-Scoped Queries
Select any text in the book and ask questions specifically about that passage. Perfect for deep-diving into technical concepts.

**Try it**: Highlight a paragraph and click the floating "Ask AI" button.

### 🔧 Personalized Content
Tell us about your hardware (GPU model, OS) during signup, and we'll show you content tailored to your environment:

- **Workstation Mode**: RTX 4070 Ti+ users get local Isaac Sim instructions
- **Cloud/Mac Mode**: Cloud instance setup or Mac-compatible alternatives

### 🌐 Urdu Localization
Full translation support with technical term preservation. Toggle any chapter to Urdu at the start.

## Hardware Requirements

### Workstation Setup (Recommended for Module 3)
- **OS**: Ubuntu 22.04 LTS
- **GPU**: NVIDIA RTX 4070 Ti or higher (RTX 4080, 4090, 5000 series)
- **RAM**: 16 GB minimum, 32 GB recommended
- **Storage**: 50 GB free space

### Edge Device Setup (Module 4 Deployment)
- **Device**: NVIDIA Jetson Orin Nano
- **OS**: Ubuntu 22.04 on Jetson
- **Use Case**: Deploy VLA models on edge for real-time robot control

### Cloud/Mac Alternative
- **Cloud**: AWS g5 instances for remote Isaac Sim access
- **Mac**: Gazebo-only simulation (Isaac Sim not supported)

## Prerequisites

Before starting, you should have:

- Basic programming knowledge (Python preferred)
- Familiarity with command-line interfaces
- Understanding of basic robotics concepts (optional but helpful)

## Let's Begin!

Ready to start your Physical AI journey? Navigate to **Module 1: ROS 2 Fundamentals** or explore the **13-Week Syllabus** to plan your learning path.

---

**Questions?** Use the chatbot to get instant answers, or explore the modules at your own pace. Every chapter includes hands-on examples that bridge AI reasoning with physical robot control.
