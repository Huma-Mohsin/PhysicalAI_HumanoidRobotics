<!--
SYNC IMPACT REPORT
==================
Version Change: [INITIAL] → 1.0.0
Action: Initial constitution creation

New Sections Added:
- Core Principles (4 principles defined)
- Tech Stack Standards (2 sections: Book platform + RAG chatbot)
- Gamification Requirements (mandatory bonus objectives)
- Content Domain Constraints
- Quality Gates & Compliance
- Governance

Templates Requiring Updates:
✅ plan-template.md - Constitution Check section compatible
✅ spec-template.md - User story format compatible with interactive/personalization requirements
✅ tasks-template.md - Task structure supports multi-component delivery

Rationale for v1.0.0 (MAJOR):
- Initial constitution establishing core governance for Physical AI & Humanoid Robotics platform
- Defines foundational principles and architectural decisions
- Sets mandatory requirements including gamification objectives

Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Embodied Intelligence
Content MUST bridge digital AI (LLMs, conversational agents) with physical robotics (ROS 2, Gazebo, Isaac Sim). Every chapter, code example, and RAG response must demonstrate the connection between software intelligence and physical embodiment.

**Rationale**: This project exists to teach Physical AI—the integration of AI reasoning with robotic action. Pure software examples or pure robotics without AI integration violate the core mission.

**Non-Negotiable Rules**:
- All tutorials include both AI reasoning component (LLM/agent) and physical simulation/execution (ROS 2, Isaac Sim)
- RAG chatbot responses must contextualize answers with embodiment considerations (e.g., "For RTX 4090 users, you can run this in Isaac Sim locally...")
- No abstract AI theory without physical grounding; no pure robotics without intelligence layer

### II. Spec-Driven Architecture
Strict adherence to Spec-Kit Plus structure and Claude Code workflows for all development artifacts. All features begin with specification, proceed through planning with architectural artifacts, then task decomposition before implementation.

**Rationale**: The platform must be exemplary in its own engineering practices to credibly teach AI-native development.

**Non-Negotiable Rules**:
- Every feature starts with `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` workflow
- All architectural decisions during planning trigger ADR suggestions (user consent required before creation)
- Prompt History Records (PHRs) created for every user interaction after completion
- Constitution supersedes all other guidance when conflicts arise

### III. Interactive Personalization
Reading experience MUST adapt dynamically to user's specific hardware/software context. Content is not one-size-fits-all; it must respond to whether the user has local GPU (RTX 4070 Ti+), edge device (Jetson Orin Nano), or cloud/Mac environment.

**Rationale**: Physical AI development varies dramatically based on hardware capabilities. Generic tutorials frustrate users or waste resources.

**Non-Negotiable Rules**:
- User onboarding captures: (a) Hardware profile (Local GPU / Edge Device / Cloud-Mac), (b) Software environment (Ubuntu version, ROS 2 distro, Isaac Sim access)
- Every chapter includes "Personalize" button that toggles content variants based on user profile
- RAG chatbot queries automatically inject user hardware context into retrieval and generation
- Default content shown before personalization must work on lowest common denominator (cloud/Mac)

### IV. Gamified Completeness
All bonus objectives are treated as mandatory requirements, not optional enhancements. The 250-point scoring system defines project scope, and full points are success criteria.

**Rationale**: The gamification rubric defines the learning outcomes and feature completeness. Skipping "bonus" items produces an incomplete educational product.

**Non-Negotiable Rules**:
- **Reusable Intelligence (50 pts)**: Claude Code Subagents and Agent Skills must be implemented and documented
- **Auth & Survey (50 pts)**: Better-Auth with hardware/software background survey fully functional
- **Dynamic Content (50 pts)**: "Personalize" button operational at chapter start
- **Localization (50 pts)**: "Translate to Urdu" button functional at chapter start
- Each requirement validated independently before final delivery

## Tech Stack Standards

### Book Platform (Docusaurus)
- **Framework**: Docusaurus (static site generator)
- **Deployment**: GitHub Pages OR Vercel (user choice, both supported)
- **Styling**: Responsive design, mobile-first, accessibility (WCAG 2.1 AA minimum)
- **Internationalization**: Docusaurus i18n plugin for Urdu translation
- **Personalization**: Client-side JavaScript toggles content variants based on user profile (stored in Better-Auth session)

**Constraints**:
- No server-side rendering complexity unless required by Docusaurus defaults
- Keep build times under 5 minutes for full site rebuild
- All content must be version-controlled in Markdown

### RAG Chatbot Platform
- **AI Engine**: OpenAI Agents SDK and/or ChatKit SDKs (API-based)
- **Backend**: FastAPI (Python 3.11+)
- **Database**: Neon Serverless Postgres (Free Tier)
- **Vector Store**: Qdrant Cloud (Free Tier)
- **Authentication**: Better-Auth (https://www.better-auth.com/)
- **Embedding Model**: OpenAI text-embedding-3-small (or compatible alternative documented in ADR if changed)

**Functional Requirements**:
- **General Q&A**: Answer questions about book content using vector similarity search + LLM generation
- **Text Selection Queries**: Support user-highlighted text as query context (frontend sends selection, backend retrieves relevant context + generates answer)
- **User Profile Injection**: All RAG queries include user's hardware/software profile as additional context for personalized responses

**Constraints**:
- API latency p95 < 3 seconds for RAG queries (excluding LLM generation time)
- Free tier limits must be monitored (Qdrant: 1GB storage, Neon: 500MB DB, OpenAI: budget alerts required)
- All secrets in environment variables, never committed to repo

## Content Domain Constraints

### Syllabus Alignment (Weeks 1-13)
Content structure MUST follow this progression:

1. **Week 1-2**: Introduction to Physical AI & Humanoid Robotics
2. **Week 3-5**: ROS 2 fundamentals (nodes, topics, services, actions)
3. **Week 6-7**: Gazebo simulation basics
4. **Week 8-9**: NVIDIA Isaac Sim (USD, PhysX, workflows)
5. **Week 10-11**: Vision-Language-Action (VLA) models for robot control
6. **Week 12-13**: Conversational robotics (LLM-driven interaction)

Each week includes tutorials, code examples, and hands-on labs. Content gaps in this progression block delivery.

### Hardware Specifications
Accurately distinguish between deployment targets and optimize instructions accordingly:

- **Digital Twin Workstations** (Local GPU): RTX 4070 Ti, RTX 4080, RTX 4090, RTX 5000 series
  - Can run Isaac Sim locally, full simulation fidelity
  - Ubuntu 22.04 LTS required
  - ROS 2 Humble or Iron

- **Edge Kits** (Embedded): Jetson Orin Nano, Jetson AGX Orin
  - Limited simulation (Gazebo only, no Isaac Sim)
  - Ubuntu 22.04 LTS on Jetson
  - ROS 2 Humble

- **Cloud/Mac** (Remote/Fallback): No local NVIDIA GPU
  - Cloud-based Isaac Sim (Omniverse Cloud), or Gazebo only
  - Tutorials emphasize Gazebo + ROS 2, with optional cloud Isaac Sim links

**Constraints**:
- Never recommend installing Isaac Sim on Mac or non-NVIDIA GPUs
- Provide Gazebo alternatives for every Isaac Sim tutorial
- RAG chatbot must recognize hardware context and adjust recommendations (e.g., "Your profile indicates Jetson Orin Nano—this tutorial uses Gazebo instead of Isaac Sim.")

### Software Environment Standards
- **OS**: Ubuntu 22.04 LTS (exceptions documented with alternatives)
- **ROS Distribution**: ROS 2 Humble (LTS) OR ROS 2 Iron (latest stable at time of writing)
- **Simulation**: Gazebo (all users), Isaac Sim (RTX GPU users + cloud users)
- **Python**: 3.10+ for all backend/AI code
- **Docker**: Optional but recommended for reproducible environments

## Quality Gates & Compliance

### Before Merge to Main
- [ ] PHR created for the work session
- [ ] If architectural decisions made during planning, ADR created (after user consent)
- [ ] Constitution check passed (all principles honored)
- [ ] Code references included for all modified files
- [ ] No hardcoded secrets or credentials

### Before Deployment (Book)
- [ ] All 13 weeks of content present
- [ ] "Personalize" button functional on at least 3 chapters (covering GPU/Edge/Cloud variants)
- [ ] "Translate to Urdu" button functional on at least 3 chapters
- [ ] Public GitHub repo accessible
- [ ] Live book URL accessible (GitHub Pages or Vercel)

### Before Deployment (RAG Chatbot)
- [ ] Better-Auth signup flow captures hardware/software background
- [ ] General Q&A queries return relevant book content
- [ ] Text selection queries work (user highlights text, gets contextual answer)
- [ ] User profile correctly injects into RAG queries (test with dummy GPU/Edge/Cloud profiles)
- [ ] Qdrant vector store populated with book embeddings
- [ ] Neon database schema created and seeded
- [ ] API endpoint accessible and documented

### Success Criteria (Final Delivery)
- [ ] Public GitHub Repo submitted
- [ ] Live Book URL submitted and accessible
- [ ] RAG Chatbot API URL submitted and functional
- [ ] Users can sign up, input hardware specs, and see personalized content
- [ ] Users can toggle chapter text to Urdu on demand
- [ ] All 250 gamification points achieved (Subagents, Auth, Personalization, Translation)

## Governance

**Amendment Procedure**:
1. Proposed changes documented with rationale
2. Version bump determined (MAJOR for principle removal/redefinition, MINOR for additions, PATCH for clarifications)
3. Sync Impact Report prepended to constitution file
4. Dependent templates updated (plan, spec, tasks) before amendment ratified
5. Commit message: `docs: amend constitution to vX.Y.Z (brief summary)`

**Versioning Policy**:
- MAJOR: Backward incompatible changes (e.g., removing a principle, changing tech stack)
- MINOR: New principles or sections added
- PATCH: Clarifications, typo fixes, non-semantic refinements

**Compliance Review**:
- All PRs and planning artifacts MUST include Constitution Check section
- Violations require explicit justification in "Complexity Tracking" table
- PHRs and ADRs are auditable records of adherence

**Runtime Guidance**:
- Claude Code agents follow CLAUDE.md for execution workflows
- Constitution supersedes CLAUDE.md when conflicts arise
- Use `/sp.constitution` command to update this file

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
