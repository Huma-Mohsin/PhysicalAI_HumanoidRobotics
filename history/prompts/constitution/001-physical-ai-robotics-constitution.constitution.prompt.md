---
id: 001
title: Physical AI Robotics Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Huma Aftab
command: /sp.constitution
labels: ["constitution", "physical-ai", "humanoid-robotics", "gamification", "personalization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A
---

## Prompt

/sp.constitution

Project: Physical AI & Humanoid Robotics: Capstone Book & RAG Platform

Core Principles:

- Embodied Intelligence: Content must bridge digital AI (LLMs) with physical robotics (ROS 2, Isaac Sim).

- Spec-Driven Architecture: Strict adherence to Spec-Kit Plus for structure and Claude Code for generation.

- Interactive Personalization: The reading experience must adapt to the user's specific hardware/software context.

- Gamified Completeness: All bonus objectives (Subagents, Auth, Personalization, Translation) are treated as mandatory requirements.

Key Standards:

- Tech Stack (Book):
  - Framework: Docusaurus.
  - Deployment: GitHub Pages or Vercel.

- Tech Stack (RAG Chatbot):
  - AI Engine: OpenAI Agents/ChatKit SDKs.
  - Backend: FastAPI.
  - Database: Neon Serverless Postgres.
  - Vector Store: Qdrant Cloud Free Tier.

- Chatbot Functionality:
  - Must answer general questions about the book.
  - Must support "Text Selection" queries (answering based specifically on user-highlighted text).

- Authentication & User Profile:
  - Provider: Better-Auth (https://www.better-auth.com/).
  - Onboarding: Must capture User Software & Hardware background at signup to enable personalization.

Gamification & Bonus Requirements (Target: 250 Total Points):

1. Reusable Intelligence (50 pts): Implementation of Claude Code Subagents and Agent Skills.
2. Auth & Survey (50 pts): Implementation of Better-Auth with hardware/software background survey.
3. Dynamic Content (50 pts): "Personalize" button at chapter start that toggles content based on user's hardware (e.g., RTX 4090 vs. Cloud/Mac).
4. Localization (50 pts): "Translate to Urdu" button at chapter start.

Content Domain Constraints:

- Syllabus Alignment: Must cover Weeks 1-13 (Intro, ROS 2, Gazebo, Isaac Sim, VLA, Conversational Robotics).
- Hardware Specifications: Accurately distinguish between "Digital Twin Workstations" (RTX 4070 Ti+) and "Edge Kits" (Jetson Orin Nano).
- Software Environment: Ubuntu 22.04 LTS, ROS 2 Humble/Iron, NVIDIA Isaac Sim.

Success Criteria:

- Public GitHub Repo and Live Book URL submitted.
- RAG Chatbot is functional and connected to the specific vector store logic.
- Users can sign up, input their hardware specs, and see the book content change to match their rig (Personalization).
- Users can toggle the entire chapter text into Urdu on demand.

## Response snapshot

Created initial constitution for Physical AI & Humanoid Robotics platform (v1.0.0).

Constitution establishes 4 core principles:
1. Embodied Intelligence - Bridge digital AI with physical robotics
2. Spec-Driven Architecture - Strict adherence to SpecKit Plus workflows
3. Interactive Personalization - Content adapts to user hardware/software context
4. Gamified Completeness - All 250 bonus points are mandatory requirements

Tech stack defined:
- Book: Docusaurus + GitHub Pages/Vercel
- RAG Chatbot: FastAPI + OpenAI Agents + Qdrant + Neon + Better-Auth

Content domain constraints include 13-week syllabus alignment and hardware-specific personalization (RTX GPU workstations, Jetson edge devices, Cloud/Mac fallback).

Quality gates and compliance checklists established for merge, book deployment, and chatbot deployment.

File updated: .specify/memory/constitution.md

## Outcome

- ✅ Impact: Established foundational governance for Physical AI platform. All future features and implementation must comply with these 4 principles and tech stack standards.
- 🧪 Tests: N/A (constitution document, no code)
- 📁 Files: Created/updated .specify/memory/constitution.md
- 🔁 Next prompts: User can now run `/sp.specify` to create feature specs that will be validated against constitution principles
- 🧠 Reflection: Constitution successfully bridges educational content requirements (13-week syllabus, hardware personalization) with engineering discipline (Spec-Kit Plus, PHR/ADR tracking). Gamification requirements elevated to mandatory status ensures complete delivery.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
