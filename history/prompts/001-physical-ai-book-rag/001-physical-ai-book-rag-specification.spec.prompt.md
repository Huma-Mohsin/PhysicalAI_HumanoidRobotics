---
id: 001
title: Physical AI Book RAG Platform Specification
stage: spec
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-book-rag
branch: 001-physical-ai-book-rag
user: Huma-Mohsin
command: /sp.specify
labels: ["robotics", "education", "rag", "docusaurus", "chatbot", "localization"]
links:
  spec: specs/001-physical-ai-book-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-physical-ai-book-rag/spec.md
 - specs/001-physical-ai-book-rag/checklists/requirements.md
tests:
 - Specification validation checklist (all items passing)
---

## Prompt

User requested specification creation via `/sp.specify` command with the following detailed requirements:

## 📝 /sp.specify

Project: Physical AI & Humanoid Robotics Capstone Book & RAG Platform

### Target Deliverables & Functionality

| Deliverable | Description | Base Points |
| :--- | :--- | :--- |
| **Book Creation** | A complete Docusaurus site covering **Modules 1-4** and the 13-week syllabus. | 25 |
| **RAG Chatbot** | Embedded chat interface that queries the book content using the **OpenAI Agent** and **FastAPI** backend. | 25 |
| **Contextual Query** | Chatbot must successfully handle and answer queries based *only* on **user-selected text**. | 50 |
| **Total Base Points** | | **100** |

---

### Technical Specifications (The "How")

| Component | Required Implementation Detail | Source |
| :--- | :--- | :--- |
| **Frontend Framework** | Docusaurus (v2 or v3) for static site generation. | Req 1 |
| **Deployment Target** | GitHub Pages or Vercel. | Submission |
| **Backend API** | FastAPI running the RAG logic, serving the frontend chat interface. | Req 2 |
| **Vector Database** | Qdrant Cloud Free Tier for indexing book content embeddings. | Req 2 |
| **Relational Database** | Neon Serverless Postgres (likely for user management/auth data). | Req 2 |
| **Agent Integration** | OpenAI Agents/ChatKit SDKs used for RAG orchestration. | Req 2 |
| **Authentication** | Full Signup/Signin flow using Better-Auth. | Req 5 |
| **User Data Schema** | Must store user's **Software and Hardware background** (e.g., GPU model, OS) upon signup. | Req 5 |

---

### Content and Scope Focus

| Module/Theme | Key Concepts to Cover | Hardware Context |
| :--- | :--- | :--- |
| **ROS 2 Fundamentals** | Nodes, Topics, Services, `rclpy` integration, URDF. | **Ubuntu 22.04 LTS** (Mandatory OS). |
| **Digital Twin** | Gazebo physics (gravity, collision), Unity for rendering, Sensor simulation (LiDAR, IMUs). | CPU-intensive physics calculations. |
| **NVIDIA Isaac** | Isaac Sim (Photorealistic simulation), Isaac ROS (VSLAM, Nav2), Sim-to-Real techniques. | **RTX 4070 Ti+** (VRAM requirement). |
| **VLA (LLMs & Robotics)** | Voice-to-Action (OpenAI Whisper), Cognitive Planning (LLM sequence generation). | Deployment to **Jetson Orin Nano** (Edge Kit). |

---

### Bonus Functionality (The "What Extra")

| Bonus Requirement | Specific User Interaction | Bonus Points |
| :--- | :--- | :--- |
| **Reusable Intelligence** | Implement **Claude Code Subagents/Skills** to automate content generation or RAG preparation tasks. | 50 |
| **Dynamic Content** | Button at chapter start toggles content between **Workstation** guide and **Cloud/Mac** alternative. | 50 |
| **Localization** | Button at chapter start toggles the entire chapter content to **Urdu**. | 50 |
| **Auth/Survey** | Better-Auth implementation includes a survey for hardware background. | 50 |

---

### Not Building

* Comprehensive comparison of specific robot vendors (e.g., Unitree vs. Robotis).
* Detailed cloud cost optimization outside of the provided AWS g5 instance example.
* Full real-time control (latency is a constraint, focus is on model transfer: train in cloud, deploy to Jetson).
* Any non-required features outside the 4 core modules.

---

**Follow-up clarifications provided:**

Q1: Urdu Translation Scope → **Option A**: Pre-translated content will be provided by a professional translation team
Q2: Hardware Profile Usage → **Option B**: Auto-select environment mode (Workstation vs. Cloud) based on hardware profile
Q3: Contextual Query Design → **Option B**: Automatically detect text selection and show a floating "Ask" button near the selection

## Response snapshot

Created comprehensive feature specification including:

1. **Branch & Directory Setup**
   - Created feature branch: `001-physical-ai-book-rag`
   - Initialized spec directory: `specs/001-physical-ai-book-rag/`

2. **Specification Document** (`spec.md`)
   - 6 prioritized user stories with independent test criteria (P1-P6)
   - 31 functional requirements organized by category:
     - Content Management (FR-001 to FR-005)
     - RAG Chatbot (FR-006 to FR-012)
     - User Management & Authentication (FR-013 to FR-019)
     - Content Personalization (FR-020 to FR-028)
     - Deployment & Access (FR-029 to FR-031)
   - 17 measurable, technology-agnostic success criteria
   - 8 edge cases identified with expected handling
   - Clear scope boundaries and out-of-scope items
   - Comprehensive assumptions and dependencies
   - 6 identified risks with detailed mitigation strategies
   - Design Decisions section documenting 3 clarified choices

3. **Quality Validation**
   - Created requirements checklist: `specs/001-physical-ai-book-rag/checklists/requirements.md`
   - All validation items passing
   - No [NEEDS CLARIFICATION] markers remaining
   - Specification ready for planning phase

4. **Key Design Decisions Documented**
   - Urdu localization via pre-translated professional content (dual file structure)
   - Hardware-based auto-selection of environment mode with user override capability
   - Floating "Ask AI" button for contextual text-selection queries

## Outcome

- ✅ Impact: Complete, validated specification ready for architectural planning
- 🧪 Tests: Specification validation checklist completed with all items passing
- 📁 Files: 2 files created (spec.md, requirements.md)
- 🔁 Next prompts: User can proceed with `/sp.clarify` for additional refinement or `/sp.plan` for architectural design
- 🧠 Reflection: Successfully captured complex multi-component educational platform requirements with minimal clarifications (3 questions, all answered). Specification balances base requirements (100 points) with bonus features (200 points) and clearly documents technical constraints.

## Evaluation notes (flywheel)

- Failure modes observed: None. Initial branch creation script had argument parsing issue but branch was successfully created.
- Graders run and results (PASS/FAIL): Specification validation checklist - PASS (all 12 criteria met)
- Prompt variant (if applicable): Standard /sp.specify workflow with clarification resolution
- Next experiment (smallest change to try): Specification complete; next step is `/sp.plan` to create architectural design document
