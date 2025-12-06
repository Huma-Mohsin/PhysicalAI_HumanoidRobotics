---
id: 003
title: Plan Physical AI RAG Platform
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-book-rag
branch: 001-physical-ai-book-rag
user: user
command: /sp.plan
labels: ["planning", "architecture", "rag", "docusaurus", "fastapi", "physical-ai"]
links:
  spec: specs/001-physical-ai-book-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-book-rag/plan.md
  - specs/001-physical-ai-book-rag/research.md
  - specs/001-physical-ai-book-rag/data-model.md
  - specs/001-physical-ai-book-rag/contracts/openapi.yaml
  - specs/001-physical-ai-book-rag/quickstart.md
  - CLAUDE.md
tests:
  - none (planning phase, no tests)
---

## Prompt

User provided three-phase implementation approach and invoked `/sp.plan` with the following guidance:

```
following The plan successfully sequences the work: Infrastructure first, then Core Content/RAG, and finally Frontend/Bonus integration, which is the most efficient method for a full-stack project as :I. Infrastructure Setup (Phase 1: Foundation)Goal: Establish all external services, the main Git repository, and the base authentication/data schema.Task IDComponentAction / DescriptionDependency1.1Repository & DeploymentInitialize Docusaurus project structure. Set up GitHub Repository and configure GitHub Pages/Vercel deployment workflow.None1.2Vector DB SetupCreate account and cluster on Qdrant Cloud Free Tier. Retrieve API key and cluster URL.None1.3Relational DB SetupCreate instance on Neon Serverless Postgres. Define tables for users and hardware_profiles.None1.4Auth Service (Better-Auth)Integrate Better-Auth SDK into the Docusaurus frontend. Configure Signup/Signin routes.1.11.5User Data CaptureModify the Signup flow (1.4) to include the hardware/software background survey. Store results in the Neon DB (1.3).1.4, 1.3II. Content & Architecture Development (Phase 2: Core Build)Goal: Generate all course content and build the core RAG system backend. This runs concurrently with Frontend development.Task IDComponentAction / DescriptionDependency2.1Content Generation (Claude Code)Use Claude Code (via Spec-Kit Plus) to generate content for all 4 Modules/13 Weeks, ensuring accuracy on ROS 2, Isaac Sim, and Hardware Specs (RTX 4070 Ti, Jetson Orin).None2.2RAG Backend (FastAPI)Set up a FastAPI service with endpoints for: /chat and /index_content.1.2, 1.32.3Embedding & IndexingCreate a script to parse the generated Docusaurus Markdown files (2.1), chunk the text, generate embeddings, and load them into the Qdrant Vector DB.2.1, 1.22.4Agent Integration LogicImplement the OpenAI Agents/ChatKit SDK within the FastAPI /chat endpoint. This agent must orchestrate the vector retrieval from Qdrant.2.2, 2.32.5Contextual Query Logic (50 pts)Enhance the RAG logic (2.4) to prioritize and integrate the user-selected text as primary context for the query, meeting the base functionality requirement.2.4III. Frontend Integration & Bonus Features (Phase 3: Completion)Goal: Embed the RAG chatbot, implement the personalization features, and secure all bonus points.Task IDComponentAction / DescriptionDependency3.1Chatbot UI EmbedEmbed the Chatbot UI component (React/Docusaurus) and connect it to the FastAPI /chat endpoint.1.1, 2.43.2Personalization Component (50 pts)Create a custom Docusaurus component with the "Personalize" button. Logic retrieves user's hardware profile (1.5) and renders content variants (Workstation vs. Cloud).1.5, 2.13.3Localization Component (50 pts)Create a custom Docusaurus component with the "Translate to Urdu" button. Integrate a translation service (or pre-generate content) to toggle chapter text.1.1, 2.13.4Subagent Intelligence (50 pts)Implement Claude Code Subagents/Skills. Example: An agent skill for automatically generating new URDF snippets based on user input, and embed the results in a chapter.2.13.5Final Review & SubmissionVerify all 250 points of functionality. Submit Public GitHub Repo Link and Published Book Link.All previous tasks
```

## Response snapshot

Created comprehensive implementation plan with the following structure:

**Main Plan File (plan.md)**:
- Technical Context: Defined all technologies, dependencies, performance goals, and constraints
- Constitution Check: Validated compliance with all 4 constitution principles (Embodied Intelligence, Spec-Driven Architecture, Interactive Personalization, Gamified Completeness) - ALL PASSED
- Project Structure: Designed monorepo architecture with separate frontend/backend directories
- Implementation Phases: Documented user's 3-phase approach with task dependencies
- Identified 4 architectural decisions requiring ADRs (OpenAI Agents, content chunking, dual-language embeddings, deployment strategy)

**Phase 0: Research Artifacts (research.md)**:
- Decision 1: OpenAI Agents SDK over LangChain (official support, simpler architecture)
- Decision 2: Semantic chunking (~500 tokens, 50-token overlap) over fixed-size chunks
- Decision 3: Separate Qdrant collections for EN/UR (50% smaller search space, better performance)
- Decision 4: Client-side Better-Auth SDK with backend session verification
- Decision 5: Vercel (frontend) + Render (backend) for deployment (free tier, excellent DX)
- Decisions 6-10: Contextual query strategy, feedback mechanism, hardware thresholds, Urdu translation, subagent intelligence
- Summary table with 10 key decisions and rationales

**Phase 1: Design Artifacts**:
- **data-model.md**: Complete database schema for Neon Postgres (users, hardware_profiles, chat_messages, feedback) and Qdrant collections (content_embeddings_en, content_embeddings_ur) with ERD, validation rules, state transitions, and migration scripts
- **contracts/openapi.yaml**: Full OpenAPI 3.0 specification with 8 endpoints (/chat, /auth/signup, /auth/signin, /auth/profile, /feedback, /admin/index_content, /health) including request/response schemas, authentication, and error handling
- **quickstart.md**: Developer onboarding guide covering prerequisites, backend setup (Python venv, .env configuration, database initialization), frontend setup (npm install, Docusaurus dev server), content indexing, Docker Compose workflow, testing, troubleshooting, and deployment instructions

**Agent Context Update**:
- Updated CLAUDE.md with feature-specific technology stack information

## Outcome

- ✅ Impact: Completed comprehensive planning phase for Physical AI RAG Platform, generating 1 plan file + 4 design artifacts (research.md, data-model.md, contracts/openapi.yaml, quickstart.md). All constitution checks passed. Ready for task decomposition (`/sp.tasks`).
- 🧪 Tests: No code tests (planning phase); validation through constitution compliance checks
- 📁 Files: 6 files created/modified (plan.md, research.md, data-model.md, openapi.yaml, quickstart.md, CLAUDE.md)
- 🔁 Next prompts: `/sp.tasks` to generate task decomposition; optionally `/sp.adr <decision-title>` to document the 4 identified architectural decisions (requires user consent per constitution)
- 🧠 Reflection: User's three-phase approach (Infrastructure → Core Content/RAG → Frontend/Bonus) is architecturally sound and aligns well with dependency sequencing. Research phase proactively resolved all technical unknowns, enabling smooth transition to implementation. Constitution check early validation prevented downstream rework.

## Evaluation notes (flywheel)

- Failure modes observed: None; all planning artifacts generated successfully
- Graders run and results (PASS/FAIL): PASS - Constitution check validated all principles, no violations detected
- Prompt variant (if applicable): Standard `/sp.plan` workflow with user-provided phased approach integrated into plan structure
- Next experiment (smallest change to try): Consider whether ADR suggestions should be made interactively during planning (blocking workflow) vs. documented for post-planning consent (current approach - less disruptive)
