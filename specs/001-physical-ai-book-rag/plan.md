# Implementation Plan: Physical AI & Humanoid Robotics Capstone Book & RAG Platform

**Branch**: `001-physical-ai-book-rag` | **Date**: 2025-12-05 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/001-physical-ai-book-rag/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a complete Docusaurus-based educational platform covering Physical AI and Humanoid Robotics across 4 modules and a 13-week syllabus, with an embedded RAG chatbot for contextual querying. The platform features user authentication with hardware profiling, content personalization (Workstation vs. Cloud/Mac modes), Urdu localization, and context-scoped chatbot queries based on user-selected text. Implementation follows a three-phase approach: Infrastructure Setup, Core Content & RAG Development, and Frontend Integration with Bonus Features.

## Technical Context

**Language/Version**:
- Frontend: JavaScript/TypeScript (Node.js 18+, React 18+ via Docusaurus 3.x)
- Backend: Python 3.11+ (FastAPI)
- Build/Deployment: Node.js for Docusaurus build, Python for API server

**Primary Dependencies**:
- **Frontend**: Docusaurus 3.x, Better-Auth SDK (authentication), React components for chatbot UI, i18n plugin for Urdu
- **Backend**: FastAPI, OpenAI Python SDK (Agents/ChatKit), Qdrant Client, asyncpg/SQLAlchemy (Neon Postgres), Pydantic
- **Infrastructure**: Qdrant Cloud (vector DB), Neon Serverless Postgres (relational DB), OpenAI API (embeddings + chat)

**Storage**:
- **Vector Database**: Qdrant Cloud Free Tier (content embeddings for RAG, dual-language indexed)
- **Relational Database**: Neon Serverless Postgres Free Tier (users, hardware_profiles, chat_messages, feedback)
- **Static Content**: Markdown files in Git (Docusaurus source), deployed to CDN (GitHub Pages or Vercel)

**Testing**:
- **Frontend**: Jest + React Testing Library (component tests), Playwright (E2E for auth flows, toggle behaviors)
- **Backend**: pytest (unit tests for RAG logic, API endpoints), pytest-asyncio (async FastAPI tests)
- **Integration**: Contract tests between frontend and FastAPI endpoints

**Target Platform**:
- **Frontend**: Modern web browsers (Chrome, Firefox, Safari, Edge) - desktop and mobile responsive
- **Backend**: Linux server (containerized FastAPI, deployable to Render/Fly.io/Railway)
- **Deployment**: Static site (GitHub Pages/Vercel) + API server (separate deployment)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Content page load: <2 seconds (p95, 25 Mbps broadband)
- Chatbot response: <5 seconds (p95, including vector search + LLM generation)
- Environment toggle: <3 seconds
- Language toggle: <2 seconds
- Concurrent users: 100 during peak usage

**Constraints**:
- Free tier limits: Qdrant (1GB storage), Neon (500MB DB, limited connections), OpenAI API (budget alerts required)
- Session duration: 7 days before re-authentication
- Minimum text selection for contextual query: 50 characters
- Build time: <5 minutes for full Docusaurus rebuild
- No server-side rendering complexity (static generation preferred)

**Scale/Scope**:
- Content: 4 modules, 13-week syllabus, estimated 50-100 chapter pages
- Users: Designed for 100-500 concurrent learners during evaluation period
- Vector embeddings: ~1000-2000 chunks (dual-language: English + Urdu)
- Database records: User profiles, chat history, feedback (estimated 1000-5000 records within free tier)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Embodied Intelligence ✅ PASS

**Requirement**: Content MUST bridge digital AI with physical robotics. All tutorials include both AI reasoning and physical simulation/execution.

**How This Plan Complies**:
- Module 1 (ROS 2): Covers physical robot communication (nodes, topics, services) with rclpy integration for AI control
- Module 2 (Digital Twin): Gazebo physics + Unity rendering for simulated physical environments
- Module 3 (NVIDIA Isaac): Isaac Sim for physically accurate simulation, Isaac ROS for sensor processing
- Module 4 (VLA): Vision-Language-Action models bridge LLM reasoning to physical robot commands
- RAG chatbot responses will include hardware context (e.g., "For RTX 4070 Ti+ users, run Isaac Sim locally...")

**Verification**: Content generation (Phase 2, Task 2.1) will enforce this via constitution-aware prompts requiring AI + physical integration in every chapter.

### II. Spec-Driven Architecture ✅ PASS

**Requirement**: Strict adherence to Spec-Kit Plus workflows (`/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`). ADR suggestions for architectural decisions. PHRs for all user interactions.

**How This Plan Complies**:
- Currently executing `/sp.plan` after completing `/sp.specify` and `/sp.clarify`
- This plan will generate research.md, data-model.md, contracts/, and quickstart.md artifacts
- Architectural decisions documented in research.md Phase 0
- ADR suggestions will be made for significant decisions (see "Architectural Decisions Requiring ADRs" section below)
- PHR will be created upon plan completion

**Verification**: Workflow execution logged in PHRs; ADRs created with user consent before implementation.

### III. Interactive Personalization ✅ PASS

**Requirement**: Content adapts to user hardware (Local GPU / Edge Device / Cloud-Mac). "Personalize" button toggles content variants. RAG queries inject user hardware context.

**How This Plan Complies**:
- Phase 1, Task 1.5: Signup flow captures hardware profile (OS, GPU model)
- Phase 3, Task 3.2: "Personalize" button component renders Workstation vs. Cloud content variants
- Data model includes hardware_profile table with GPU capability thresholds (RTX 4070 Ti+ → Workstation default)
- RAG backend (Phase 2, Task 2.4) injects user profile into every query as system context
- Content structure (Phase 2, Task 2.1) will include dual variants for GPU-dependent chapters

**Verification**: Acceptance tests confirm personalization logic works for all three hardware profiles.

### IV. Gamified Completeness ✅ PASS

**Requirement**: All 250 bonus points are mandatory. Reusable Intelligence (50), Auth & Survey (50), Dynamic Content (50), Localization (50) must all be implemented.

**How This Plan Complies**:
- **Reusable Intelligence (50 pts)**: Phase 3, Task 3.4 implements Claude Code Subagents/Skills for URDF generation and content enhancement
- **Auth & Survey (50 pts)**: Phase 1, Tasks 1.4-1.5 implement Better-Auth with hardware/software background survey
- **Dynamic Content (50 pts)**: Phase 3, Task 3.2 implements "Personalize" button with hardware-aware content variants
- **Localization (50 pts)**: Phase 3, Task 3.3 implements "Translate to Urdu" button with i18n plugin integration

**Verification**: Each feature independently testable and validated before final submission (Task 3.5).

### Tech Stack Standards ✅ PASS

**Book Platform**:
- ✅ Docusaurus 3.x (static site generator)
- ✅ GitHub Pages or Vercel deployment (user choice)
- ✅ Responsive design, WCAG 2.1 AA accessibility
- ✅ Docusaurus i18n plugin for Urdu translation
- ✅ Client-side JavaScript for personalization toggles

**RAG Chatbot Platform**:
- ✅ OpenAI Agents SDK / ChatKit (API-based)
- ✅ FastAPI (Python 3.11+)
- ✅ Neon Serverless Postgres (Free Tier)
- ✅ Qdrant Cloud (Free Tier)
- ✅ Better-Auth (authentication)
- ✅ OpenAI text-embedding-3-small (embeddings)

**Verification**: All dependencies specified in Technical Context align with constitution requirements.

### Content Domain Constraints ✅ PASS

**Syllabus Alignment**: Content follows Weeks 1-13 progression (Intro → ROS 2 → Gazebo → Isaac → VLA → Conversational Robotics).

**How This Plan Complies**:
- Content generation (Phase 2, Task 2.1) structured around 13-week syllabus with weekly learning objectives
- Module mapping: Module 1 (Weeks 3-5), Module 2 (Weeks 6-7), Module 3 (Weeks 8-9), Module 4 (Weeks 10-13)

**Hardware Specifications**:
- Workstation (RTX 4070 Ti+): Ubuntu 22.04, ROS 2 Humble, local Isaac Sim
- Edge (Jetson Orin Nano): Ubuntu 22.04, ROS 2 Humble, Gazebo only
- Cloud/Mac: Gazebo + optional Omniverse Cloud for Isaac Sim

**How This Plan Complies**:
- Hardware profiling (Task 1.5) captures GPU model with threshold detection
- Content variants (Task 3.2) provide Gazebo alternatives for every Isaac Sim tutorial
- RAG responses (Task 2.4) inject hardware context to adjust recommendations

**Verification**: Content review confirms accurate hardware specifications and no invalid recommendations (e.g., no Isaac Sim on Mac).

### Quality Gates ✅ PASS (Deferred to Implementation)

**Before Merge to Main**:
- PHR creation: Covered (Task will create PHR upon completion)
- ADR creation: Covered (see Architectural Decisions section)
- Constitution check: This section
- Code references: Implementation phase responsibility
- No hardcoded secrets: Implementation phase responsibility (will use .env, environment variables)

**Before Deployment**: All deployment gates are acceptance criteria in spec.md and will be validated in Task 3.5.

**Gate Status**: PASS - All planning-phase requirements met. Implementation-phase gates deferred to `/sp.tasks` and `/sp.implement`.

---

### Architectural Decisions Requiring ADRs

The following significant decisions will require ADR documentation (user consent required):

1. **ADR-001: RAG Architecture - OpenAI Agents vs. LangChain** (Decision: OpenAI Agents SDK)
2. **ADR-002: Content Chunking Strategy for Dual-Language Embeddings** (Decision: Separate vector collections for EN/UR vs. metadata filtering)
3. **ADR-003: Personalization Storage - Client-side vs. Server-side State** (Decision: Server-side in Neon DB with client-side caching)
4. **ADR-004: Deployment Strategy - Monorepo vs. Separate Repos** (Decision: Monorepo with separate deployment pipelines)

These will be suggested to the user during research.md generation (Phase 0) and created only with explicit consent.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Monorepo structure for web application (Docusaurus frontend + FastAPI backend)

# Backend (RAG API Server)
backend/
├── src/
│   ├── api/
│   │   ├── __init__.py
│   │   ├── main.py                 # FastAPI app entry point
│   │   ├── routes/
│   │   │   ├── chat.py             # /chat endpoint for RAG queries
│   │   │   ├── auth.py             # Better-Auth integration endpoints
│   │   │   └── admin.py            # /index_content endpoint
│   │   └── middleware/
│   │       └── cors.py             # CORS configuration for frontend
│   ├── models/
│   │   ├── user.py                 # User, HardwareProfile SQLAlchemy models
│   │   ├── chat.py                 # ChatMessage, Feedback models
│   │   └── embedding.py            # ContentEmbedding metadata models
│   ├── services/
│   │   ├── rag/
│   │   │   ├── embedder.py         # OpenAI embedding generation
│   │   │   ├── retriever.py        # Qdrant vector search
│   │   │   └── generator.py        # OpenAI Agents/ChatKit orchestration
│   │   ├── auth_service.py         # Better-Auth session validation
│   │   └── profile_service.py      # Hardware profile logic
│   ├── db/
│   │   ├── neon.py                 # Neon Postgres connection
│   │   └── qdrant_client.py        # Qdrant Cloud client
│   ├── schemas/
│   │   ├── chat_request.py         # Pydantic request/response schemas
│   │   └── user_profile.py
│   └── utils/
│       ├── config.py               # Environment variable loading
│       └── chunking.py             # Content chunking for embeddings
├── tests/
│   ├── unit/
│   │   ├── test_embedder.py
│   │   ├── test_retriever.py
│   │   └── test_generator.py
│   ├── integration/
│   │   ├── test_chat_endpoint.py
│   │   └── test_auth_flow.py
│   └── fixtures/
│       └── sample_content.md       # Test data
├── scripts/
│   └── index_content.py            # Script to populate Qdrant from Docusaurus MD
├── requirements.txt
├── Dockerfile
└── .env.example

# Frontend (Docusaurus Book Platform)
frontend/
├── docs/                           # Markdown content for modules
│   ├── intro.md
│   ├── module-1-ros2/
│   │   ├── index.md
│   │   ├── nodes.md
│   │   └── topics.md
│   ├── module-2-digital-twin/
│   ├── module-3-isaac/
│   └── module-4-vla/
├── i18n/
│   └── ur/                         # Urdu translations
│       └── docusaurus-plugin-content-docs/
│           └── current/
│               ├── intro.md
│               └── module-1-ros2/
├── src/
│   ├── components/
│   │   ├── ChatWidget/            # RAG chatbot UI
│   │   │   ├── ChatWidget.tsx
│   │   │   ├── MessageList.tsx
│   │   │   └── FeedbackButtons.tsx
│   │   ├── PersonalizeButton/     # Hardware-aware content toggle
│   │   │   ├── PersonalizeButton.tsx
│   │   │   └── ContentVariants.tsx
│   │   ├── LanguageToggle/        # Urdu translation toggle
│   │   │   └── LanguageToggle.tsx
│   │   └── TextSelection/         # Floating "Ask AI" button
│   │       └── SelectionButton.tsx
│   ├── pages/                     # Custom React pages
│   │   ├── index.tsx              # Homepage
│   │   └── auth/
│   │       ├── signup.tsx
│   │       └── signin.tsx
│   ├── css/
│   │   └── custom.css
│   └── hooks/
│       ├── useAuth.ts             # Better-Auth React hooks
│       └── useChat.ts             # Chat state management
├── static/
│   └── img/                       # Images, diagrams
├── docusaurus.config.js           # Docusaurus configuration
├── sidebars.js                    # Navigation structure
├── package.json
└── .env.example

# Shared configuration
.github/
├── workflows/
│   ├── deploy-frontend.yml        # GitHub Actions for Docusaurus build
│   └── deploy-backend.yml         # Backend deployment (Render/Fly.io)
└── CODEOWNERS

# Root-level files
README.md                          # Project overview, setup instructions
.gitignore
docker-compose.yml                 # Local development environment
```

**Structure Decision**: **Monorepo with separate frontend/backend directories**

**Rationale**:
- **Monorepo**: Single repository simplifies version control, issue tracking, and maintains cohesion between content and RAG system
- **Separate Deployment Pipelines**: Frontend (static site) deploys to GitHub Pages/Vercel, Backend (FastAPI) deploys to Render/Fly.io/Railway
- **Clear Separation of Concerns**: Frontend handles presentation and user interaction, Backend handles RAG logic and data persistence
- **Content Co-location**: Markdown content lives in `frontend/docs/` adjacent to the Docusaurus configuration, making content authoring and indexing straightforward
- **Shared Git History**: PHRs, ADRs, and specs in `.specify/` and `specs/` are accessible to both frontend and backend contexts

**Alternative Considered**: Separate repositories for frontend and backend were rejected because they would fragment the content generation workflow (content needs to be indexed by backend) and complicate the Spec-Kit Plus artifact management.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No constitution violations detected. All principles and requirements are satisfied by the planned architecture.

---

## Implementation Phases

This plan follows a three-phase approach that sequences work efficiently: Infrastructure first, then Core Content & RAG development, and finally Frontend Integration with Bonus Features.

### Phase 1: Infrastructure Setup (Foundation)

**Goal**: Establish all external services, the main Git repository, and the base authentication/data schema before any content or feature development begins.

| Task ID | Component | Action / Description | Dependency |
|---------|-----------|---------------------|------------|
| **1.1** | Repository & Deployment | Initialize Docusaurus project structure. Set up GitHub Repository and configure GitHub Pages/Vercel deployment workflow. | None |
| **1.2** | Vector DB Setup | Create account and cluster on Qdrant Cloud Free Tier. Retrieve API key and cluster URL. Configure collections for English and Urdu embeddings. | None |
| **1.3** | Relational DB Setup | Create instance on Neon Serverless Postgres. Define tables for `users`, `hardware_profiles`, `chat_messages`, and `feedback`. | None |
| **1.4** | Auth Service (Better-Auth) | Integrate Better-Auth SDK into the Docusaurus frontend. Configure Signup/Signin routes with session management (7-day expiration). | 1.1 |
| **1.5** | User Data Capture | Modify the Signup flow (1.4) to include the hardware/software background survey (OS, GPU model). Store results in the Neon DB (1.3). Implement hardware profile thresholds (RTX 4070 Ti+ → Workstation default). | 1.4, 1.3 |

**Outputs**:
- Docusaurus site skeleton with navigation structure
- GitHub repository with deployment workflows
- Qdrant Cloud cluster with dual-language collections configured
- Neon Postgres database with schema populated
- Better-Auth integrated with functional signup/signin flows
- Hardware profiling logic implemented and tested

**Acceptance Criteria**:
- Users can sign up, provide hardware profile, and authenticate successfully
- Session persistence validated across page navigation
- Database connections tested and functional
- Deployment pipeline runs successfully (even with placeholder content)

---

### Phase 2: Content & Architecture Development (Core Build)

**Goal**: Generate all course content and build the core RAG system backend. This runs concurrently with any remaining Frontend component work.

| Task ID | Component | Action / Description | Dependency |
|---------|-----------|---------------------|------------|
| **2.1** | Content Generation (Claude Code) | Use Claude Code (via Spec-Kit Plus) to generate content for all 4 Modules / 13 Weeks, ensuring accuracy on ROS 2, Isaac Sim, and Hardware Specs (RTX 4070 Ti, Jetson Orin). Create dual content variants (Workstation vs. Cloud/Mac) for GPU-dependent chapters. Structure follows 13-week syllabus. | None |
| **2.2** | RAG Backend (FastAPI) | Set up a FastAPI service with endpoints for: `/chat` (RAG queries), `/index_content` (admin), and `/health`. Implement CORS middleware for frontend integration. | 1.2, 1.3 |
| **2.3** | Embedding & Indexing | Create a script (`backend/scripts/index_content.py`) to parse the generated Docusaurus Markdown files (2.1), chunk the text (semantic chunking strategy), generate embeddings using OpenAI text-embedding-3-small, and load them into the Qdrant Vector DB. Index both English and Urdu content separately with language metadata. | 2.1, 1.2 |
| **2.4** | Agent Integration Logic | Implement the OpenAI Agents/ChatKit SDK within the FastAPI `/chat` endpoint. This agent must orchestrate: (1) vector retrieval from Qdrant based on query, (2) user hardware profile injection as system context, (3) cross-module synthesis with source citations, (4) language-aware response generation (Urdu if content mode is Urdu). | 2.2, 2.3 |
| **2.5** | Contextual Query Logic (50 pts) | Enhance the RAG logic (2.4) to prioritize and integrate the user-selected text as primary context for the query. When frontend sends `selected_text` parameter, constrain retrieval to semantically similar chunks and generate answers scoped to the selection. This meets the base 50-point functionality requirement. | 2.4 |

**Outputs**:
- Complete educational content for Modules 1-4 (ROS 2, Digital Twin, NVIDIA Isaac, VLA) in Markdown format
- Dual content variants for Workstation and Cloud/Mac modes
- FastAPI backend with functional RAG endpoints
- Qdrant database populated with ~1000-2000 dual-language embeddings
- OpenAI Agents integrated with cross-module synthesis and citation logic
- Contextual query feature functional and tested

**Acceptance Criteria**:
- All 13 weeks of content present with accurate technical details
- RAG chatbot answers in-scope queries with relevant book content citations
- Cross-module queries synthesize information from multiple modules
- Contextual queries constrain answers to user-selected text
- Language-aware responses (Urdu when content is in Urdu mode)
- Chatbot responds within 5 seconds (p95)

---

### Phase 3: Frontend Integration & Bonus Features (Completion)

**Goal**: Embed the RAG chatbot UI, implement the personalization features, and secure all bonus points (250 total).

| Task ID | Component | Action / Description | Dependency |
|---------|-----------|---------------------|------------|
| **3.1** | Chatbot UI Embed | Embed the Chatbot UI component (React/Docusaurus custom component) and connect it to the FastAPI `/chat` endpoint. Implement message list, input field, floating "Ask AI" button on text selection (50-character minimum), and thumbs up/down feedback buttons with optional text feedback. | 1.1, 2.4 |
| **3.2** | Personalization Component (50 pts) | Create a custom Docusaurus component with the "Personalize" button. Logic retrieves user's hardware profile (1.5) and renders content variants (Workstation vs. Cloud). Toggle switches content within 3 seconds. Preference persists for authenticated users via Neon DB. | 1.5, 2.1 |
| **3.3** | Localization Component (50 pts) | Create a custom Docusaurus component with the "Translate to Urdu" button. Integrate Docusaurus i18n plugin to toggle chapter text between English and Urdu. Pre-translated Urdu content loaded from `i18n/ur/` directory. Toggle switches content within 2 seconds. Preference persists across navigation. | 1.1, 2.1 |
| **3.4** | Subagent Intelligence (50 pts) | Implement Claude Code Subagents/Skills. Example: An agent skill for automatically generating new URDF snippets based on user input (e.g., `/generate-urdf <robot-description>`), and embed the results in a chapter or provide as downloadable artifact. Document the subagent workflow in a dedicated chapter showcasing reusable AI intelligence. | 2.1 |
| **3.5** | Final Review & Submission | Verify all 250 points of functionality: (1) Contextual Query (50), (2) Subagent Intelligence (50), (3) Auth & Survey (50), (4) Personalization (50), (5) Localization (50), (6) Base Platform (50). Run E2E tests for all user stories. Submit Public GitHub Repo Link and Published Book Link. | All previous tasks |

**Outputs**:
- Fully functional chatbot UI embedded in Docusaurus
- "Personalize" button operational with hardware-aware content variants
- "Translate to Urdu" button operational with full localization
- Claude Code Subagent/Skill implemented and documented
- All acceptance criteria validated
- Public GitHub repository and live book URL submitted

**Acceptance Criteria**:
- Chatbot UI displays on all content pages
- Text selection triggers floating "Ask AI" button (>50 chars)
- Feedback buttons record thumbs up/down with optional text
- "Personalize" button toggles Workstation/Cloud content (<3 sec)
- "Translate to Urdu" button toggles English/Urdu content (<2 sec)
- Subagent skill generates valid URDF snippets on demand
- All 6 user stories pass acceptance scenarios
- Platform accessible via public URL with 99% uptime
- GitHub repository public and well-documented

---

## Phase 0: Research & Design Artifacts

Before implementation begins, the following artifacts will be generated to resolve technical unknowns and establish architectural patterns:

### research.md

Will document decisions on:
1. **RAG Architecture**: OpenAI Agents SDK vs. LangChain (Decision: OpenAI Agents for ChatKit integration and official support)
2. **Content Chunking Strategy**: Semantic chunking vs. fixed-size chunks (Decision: Semantic with ~500-token chunks, overlap 50 tokens)
3. **Dual-Language Embeddings**: Separate collections vs. metadata filtering (Decision: Separate collections for performance)
4. **Better-Auth Integration**: Client-side SDK vs. server-side validation (Decision: Client-side SDK with backend session verification)
5. **Deployment Hosting**: GitHub Pages vs. Vercel for frontend, Render vs. Fly.io for backend (Decision: TBD based on free tier limits and DX)

### data-model.md

Will define:
- `User` entity (id, email, password_hash, created_at)
- `HardwareProfile` entity (user_id FK, os_type, gpu_model, environment_preference)
- `ChatMessage` entity (id, user_id FK, query, response, selected_text, language, timestamp)
- `Feedback` entity (message_id FK, rating, optional_text, timestamp)
- `ContentEmbedding` metadata (chunk_id, source_module, source_chapter, language, vector_id)

### contracts/

API contracts in OpenAPI format:
- `POST /chat` (ChatRequest → ChatResponse)
- `POST /auth/signup` (SignupRequest → AuthResponse)
- `POST /auth/signin` (SigninRequest → AuthResponse)
- `POST /feedback` (FeedbackRequest → FeedbackResponse)
- `POST /admin/index_content` (IndexRequest → IndexResponse)

### quickstart.md

Developer setup instructions:
1. Clone repository
2. Backend setup (Python venv, install requirements, configure .env)
3. Frontend setup (npm install, configure Docusaurus)
4. Database setup (Neon Postgres schema migration, Qdrant collection creation)
5. Local development (docker-compose up for full stack)
6. Content indexing (run indexing script)
7. Testing (pytest for backend, Jest for frontend)

---

## Next Steps

1. **Generate research.md**: Document all architectural decisions and technology choices
2. **Generate data-model.md**: Define complete database schema with relationships and constraints
3. **Generate API contracts**: Create OpenAPI specs for all endpoints
4. **Generate quickstart.md**: Write developer onboarding guide
5. **Update agent context**: Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`
6. **Proceed to `/sp.tasks`**: Generate task decomposition based on this plan
