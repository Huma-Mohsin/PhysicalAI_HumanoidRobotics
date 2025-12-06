# Implementation Tasks: Physical AI & Humanoid Robotics Capstone Book & RAG Platform

**Feature**: 001-physical-ai-book-rag
**Branch**: `001-physical-ai-book-rag`
**Date**: 2025-12-05
**Plan**: [plan.md](./plan.md) | **Spec**: [spec.md](./spec.md)

## Overview

This task list implements the Physical AI & Humanoid Robotics platform following a user-story-driven approach. Each phase corresponds to a user story from the spec (P1-P6), plus setup and polish phases. Tasks are organized to enable independent story implementation and testing.

**Total Tasks**: 85
**User Stories**: 6 (P1-P6 priorities from spec.md)
**Parallelizable Tasks**: 42 (marked with [P])
**Testing Approach**: Integration tests per user story (no unit tests unless explicitly needed)

---

## Phase 1: Setup & Infrastructure

**Goal**: Initialize project structure, configure external services, and establish deployment pipelines before any feature development.

**Independent Test**: Repository structure matches plan.md, deployment workflows execute successfully (even with placeholder content), all external services (Qdrant, Neon, OpenAI) are accessible.

### Tasks

- [ ] T001 Initialize Git repository and create branch structure (main, 001-physical-ai-book-rag)
- [ ] T002 [P] Create frontend/ directory and initialize Docusaurus 3.x project with package.json
- [ ] T003 [P] Create backend/ directory structure (src/api/, src/models/, src/services/, src/db/, tests/)
- [ ] T004 [P] Configure frontend/docusaurus.config.js with site metadata, theme, and navigation
- [ ] T005 [P] Create backend/requirements.txt with FastAPI, OpenAI SDK, Qdrant client, SQLAlchemy, Pydantic
- [ ] T006 [P] Set up .gitignore for Python (venv/, __pycache__/, .env) and Node.js (node_modules/, build/, .env)
- [ ] T007 Create backend/.env.example with placeholders for OPENAI_API_KEY, QDRANT_URL, DATABASE_URL, AUTH_SECRET
- [ ] T008 Create frontend/.env.example with REACT_APP_API_URL placeholder
- [ ] T009 [P] Create Qdrant Cloud account, cluster, and retrieve API key + URL
- [ ] T010 [P] Create Neon Serverless Postgres instance and retrieve connection string
- [ ] T011 [P] Create OpenAI API account and retrieve API key
- [ ] T012 Create .github/workflows/deploy-frontend.yml for Vercel deployment (build command: cd frontend && npm run build)
- [ ] T013 Create .github/workflows/deploy-backend.yml for Render deployment (start command: cd backend && uvicorn src.api.main:app)
- [ ] T014 Create docker-compose.yml for local development (frontend + backend services)
- [ ] T015 Create README.md with project overview, setup instructions, and links to specs/
- [ ] T015a [P] Configure PHR workflow environment: create history/prompts/ directory structure (constitution/, general/, 001-physical-ai-book-rag/) and verify .specify/scripts/bash/create-phr.sh is executable

**Parallel Execution Example**: T002-T006 can run simultaneously (different directories, no dependencies).

---

## Phase 2: Foundational Infrastructure

**Goal**: Implement shared infrastructure required by all user stories (database schema, API framework, authentication foundation).

**Independent Test**: Database schema created successfully, API server starts without errors, health check endpoint returns 200, Better-Auth SDK initializes in frontend.

### Tasks

- [ ] T016 Run backend/alembic init and create initial migration for Neon Postgres schema (users, hardware_profiles, chat_messages, feedback tables from data-model.md)
- [ ] T017 Create backend/src/db/neon.py with SQLAlchemy engine and session management for Neon Postgres
- [ ] T018 [P] Create backend/src/db/qdrant_client.py with Qdrant client initialization and connection validation
- [ ] T019 [P] Create backend/src/models/user.py with User SQLAlchemy model (id, email, password_hash, created_at, updated_at, is_active)
- [ ] T020 [P] Create backend/src/models/hardware_profile.py with HardwareProfile model (user_id FK, os_type, gpu_model, environment_preference ENUM, language_preference ENUM)
- [ ] T021 [P] Create backend/src/models/chat.py with ChatMessage and Feedback models (message_id FK, rating ENUM, optional_text)
- [ ] T022 Create backend/src/api/main.py with FastAPI app initialization, CORS middleware for frontend origin
- [ ] T023 Create backend/src/api/routes/__init__.py to register all route blueprints
- [ ] T024 Create backend/src/api/middleware/cors.py with CORS configuration allowing http://localhost:3000
- [ ] T025 Create backend/src/api/routes/health.py with /health GET endpoint returning {status: "healthy", database: "connected", vector_db: "connected"}
- [ ] T026 [P] Create backend/scripts/init_qdrant.py to create content_embeddings_en and content_embeddings_ur collections (vector_size=1536, distance=Cosine)
- [ ] T027 Run backend/scripts/init_qdrant.py to initialize Qdrant collections
- [ ] T028 Install Better-Auth SDK in frontend (npm install @better-auth/react)
- [ ] T029 Create frontend/src/hooks/useAuth.ts with Better-Auth React hooks (useSession, useSignup, useSignin, useSignout)
- [ ] T030 Configure Better-Auth provider in frontend/src/pages/_app.tsx (or equivalent Docusaurus wrapper) with Neon Postgres connection
- [ ] T031 Test Phase 2: Start backend server, verify /health endpoint returns 200, verify Qdrant collections exist, verify Better-Auth initializes in frontend

**Parallel Execution Example**: T019-T021 (models) can run in parallel, T026 (Qdrant init script) can run independently.

---

## Phase 3: User Story 1 - Browse Educational Content (P1)

**Goal**: Implement complete browsable educational content for 4 modules and 13-week syllabus with navigation.

**Independent Test**: All 4 modules are accessible via navigation, each module contains chapters with accurate content, weekly syllabus structure is visible, pages load within 2 seconds.

**Story Label**: [US1]

### Tasks

- [ ] T032 [P] [US1] Create frontend/docs/intro.md with platform overview and learning objectives
- [ ] T033 [P] [US1] Create frontend/docs/module-1-ros2/index.md with Module 1 overview (ROS 2 Fundamentals, Weeks 3-5)
- [ ] T034 [P] [US1] Create frontend/docs/module-1-ros2/nodes.md with ROS 2 nodes tutorial (rclpy integration, AI control examples)
- [ ] T035 [P] [US1] Create frontend/docs/module-1-ros2/topics.md with ROS 2 topics tutorial (pub/sub patterns, message types)
- [ ] T036 [P] [US1] Create frontend/docs/module-1-ros2/services.md with ROS 2 services tutorial (request/response, synchronous communication)
- [ ] T037 [P] [US1] Create frontend/docs/module-1-ros2/urdf.md with URDF tutorial (robot description formats, link/joint definitions)
- [ ] T038 [P] [US1] Create frontend/docs/module-2-digital-twin/index.md with Module 2 overview (Digital Twin, Weeks 6-7)
- [ ] T039 [P] [US1] Create frontend/docs/module-2-digital-twin/gazebo.md with Gazebo physics tutorial (world files, sensor simulation)
- [ ] T040 [P] [US1] Create frontend/docs/module-2-digital-twin/unity.md with Unity rendering tutorial (visualization, real-time rendering)
- [ ] T041 [P] [US1] Create frontend/docs/module-3-isaac/index.md with Module 3 overview (NVIDIA Isaac, Weeks 8-9, RTX 4070 Ti+ requirements)
- [ ] T042 [P] [US1] Create frontend/docs/module-3-isaac/isaac-sim.md with Isaac Sim tutorial (USD workflows, PhysX, GPU requirements)
- [ ] T043 [P] [US1] Create frontend/docs/module-3-isaac/isaac-ros.md with Isaac ROS tutorial (ROS 2 integration, accelerated perception)
- [ ] T044 [P] [US1] Create frontend/docs/module-4-vla/index.md with Module 4 overview (Vision-Language-Action, Weeks 10-13)
- [ ] T045 [P] [US1] Create frontend/docs/module-4-vla/voice-to-action.md with VLA tutorial (Whisper integration, command parsing, physical execution)
- [ ] T046 [P] [US1] Create frontend/docs/module-4-vla/cognitive-planning.md with cognitive planning tutorial (LLM-driven task decomposition, deployment on Jetson Orin Nano)
- [ ] T047 [US1] Create frontend/docs/syllabus.md with 13-week syllabus structure and weekly learning objectives
- [ ] T047a [US1] Validate Module 1-4 technical accuracy by cross-referencing content against official documentation (ROS 2 Humble docs at docs.ros.org, NVIDIA Isaac Sim docs at docs.omniverse.nvidia.com/isaacsim, Gazebo docs, VLA model papers). Create validation checklist with at least 5 key technical facts per module verified against authoritative sources.
- [ ] T047b [US1] Audit all chapters (T032-T047 outputs) to ensure compliance with Constitution Principle I: verify every chapter includes BOTH (a) AI reasoning component (LLM/agent integration, conversational interface, or intelligent decision-making) AND (b) physical simulation/execution example (ROS 2 commands, Gazebo/Isaac Sim workflows, or robot hardware interaction). Document audit results in specs/001-physical-ai-book-rag/constitution-audit.md with pass/fail status per chapter.
- [ ] T048 [US1] Configure frontend/sidebars.js with hierarchical navigation for all 4 modules and syllabus
- [ ] T049 [US1] Test US1: Navigate to all modules and chapters, verify content accuracy (ROS 2 concepts, hardware specifications), measure page load times (<2 sec)

**Parallel Execution Example**: T032-T047 (content files) can all be created in parallel as they are independent Markdown files.

---

## Phase 4: User Story 2 - Query Content Using AI Chatbot (P2)

**Goal**: Implement RAG chatbot that answers questions about book content with contextually relevant responses and source citations.

**Independent Test**: Chat interface is visible on all pages, queries about module content return relevant answers with citations, chatbot responds within 5 seconds, out-of-scope queries are handled gracefully.

**Story Label**: [US2]

### Tasks

- [ ] T050 [US2] Create backend/src/utils/config.py to load environment variables (OPENAI_API_KEY, QDRANT_URL, DATABASE_URL)
- [ ] T051 [P] [US2] Create backend/src/utils/chunking.py with semantic chunking function (~500 tokens, 50-token overlap, Markdown-aware separators)
- [ ] T052 [P] [US2] Create backend/src/services/rag/embedder.py with OpenAI embedding generation function (text-embedding-3-small)
- [ ] T053 [US2] Create backend/scripts/index_content.py to parse frontend/docs/, chunk content, generate embeddings, upload to Qdrant (language: en)
- [ ] T054 [US2] Run backend/scripts/index_content.py --language en to populate Qdrant content_embeddings_en collection
- [ ] T055 [P] [US2] Create backend/src/services/rag/retriever.py with Qdrant vector search function (top-5 chunks, cosine similarity)
- [ ] T056 [P] [US2] Create backend/src/services/rag/generator.py with OpenAI Agents SDK integration for response generation with citations
- [ ] T057 [US2] Create backend/src/api/routes/chat.py with POST /chat endpoint (accepts query, language, session_id; returns message_id, response, citations)
- [ ] T058 [US2] Implement cross-module synthesis logic in generator.py (retrieve from multiple modules, cite each source module in response)
- [ ] T059 [US2] Implement language-aware response logic in generator.py (respond in Urdu when language=ur, English when language=en)
- [ ] T060 [P] [US2] Create backend/src/schemas/chat_request.py with Pydantic schemas (ChatRequest, ChatResponse, Citation)
- [ ] T061 [P] [US2] Create frontend/src/components/ChatWidget/ChatWidget.tsx with chat UI (message list, input field, send button)
- [ ] T062 [P] [US2] Create frontend/src/components/ChatWidget/MessageList.tsx to display chat messages with citations as links
- [ ] T063 [US2] Create frontend/src/hooks/useChat.ts with state management for chat messages and API calls to POST /chat
- [ ] T064 [US2] Integrate ChatWidget component into Docusaurus theme wrapper to display on all content pages
- [ ] T065 [US2] Test US2: Ask "What is a ROS 2 node?" and verify response includes Module 1 citation, ask cross-module question and verify synthesis, measure response time (<5 sec), ask out-of-scope question and verify graceful handling

**Parallel Execution Example**: T051-T052 (chunking + embedder) and T061-T062 (frontend components) can run in parallel.

---

## Phase 5: User Story 3 - Contextual Query on Selected Text (P3)

**Goal**: Implement text selection-based contextual queries where answers are constrained to the highlighted section.

**Independent Test**: Selecting text >50 chars displays "Ask AI" button, clicking button triggers contextual query, response is scoped to selection, deselecting text reverts to full-book mode.

**Story Label**: [US3]

### Tasks

- [ ] T066 [P] [US3] Create frontend/src/components/TextSelection/SelectionButton.tsx with floating "Ask AI" button (appears on text selection >50 chars)
- [ ] T067 [US3] Implement text selection detection logic in SelectionButton.tsx using window.getSelection() API
- [ ] T068 [US3] Update backend/src/api/routes/chat.py to accept optional selected_text parameter (min 50 chars validation)
- [ ] T069 [US3] Implement selection-scoped retrieval logic in backend/src/services/rag/retriever.py (embed selected_text, search for similar chunks, merge with query results)
- [ ] T070 [US3] Update backend/src/services/rag/generator.py to include selection context in system prompt ("Answer based primarily on this selected text: {selected_text}")
- [ ] T071 [US3] Update ChatWidget.tsx to display "contextual mode" indicator when selected_text is present
- [ ] T072 [US3] Implement deselection handler in SelectionButton.tsx to hide button and revert to full-book mode
- [ ] T073 [US3] Test US3: Select URDF paragraph, click "Ask AI", ask "What file format is this?", verify answer is constrained to selection; deselect text and verify reversion to full-book mode; select <50 chars and verify no button appears

**Parallel Execution Example**: T066-T067 (frontend selection UI) can run in parallel with T068-T070 (backend contextual logic).

---

## Phase 6: User Story 4 - User Registration and Hardware Profile (P4)

**Goal**: Implement authentication with hardware profiling to enable personalized content guidance.

**Independent Test**: Users can sign up with email, password, OS, GPU model; hardware profile is saved; logged-in users see hardware-specific guidance; session persists across navigation.

**Story Label**: [US4]

### Tasks

- [ ] T074 [P] [US4] Create frontend/src/pages/auth/signup.tsx with signup form (email, password, os_type, gpu_model fields)
- [ ] T075 [P] [US4] Create frontend/src/pages/auth/signin.tsx with signin form (email, password fields)
- [ ] T076 [P] [US4] Create backend/src/services/auth_service.py with Better-Auth session validation middleware
- [ ] T077 [P] [US4] Create backend/src/services/profile_service.py with is_workstation_capable(gpu_model) function (checks for RTX 4070 Ti, 4080, 4090, 50 series)
- [ ] T078 [US4] Create backend/src/api/routes/auth.py with POST /auth/signup endpoint (creates user + hardware_profile, auto-selects environment_preference based on GPU)
- [ ] T079 [US4] Implement POST /auth/signin endpoint in backend/src/api/routes/auth.py (validates credentials, returns JWT token with 7-day expiration)
- [ ] T080 [US4] Implement GET /auth/profile endpoint in backend/src/api/routes/auth.py (returns user profile with hardware_profile data)
- [ ] T081 [US4] Implement PATCH /auth/profile endpoint to allow users to update hardware profile (os_type, gpu_model, environment_preference, language_preference)
- [ ] T082 [US4] Update backend/src/api/routes/chat.py to inject user hardware profile into RAG system context (retrieve from session token)
- [ ] T083 [US4] Integrate useAuth hook in frontend to manage signup/signin flows and session state
- [ ] T084 [US4] Test US4: Sign up with "Ubuntu 22.04" and "RTX 4070 Ti", verify hardware profile saved with environment_preference="workstation"; sign in and verify session persists across page navigation; verify RAG responses include hardware context

**Parallel Execution Example**: T074-T075 (frontend auth pages) and T076-T077 (backend services) can run in parallel.

---

## Phase 7: User Story 5 - Toggle Content for Different Environments (P5)

**Goal**: Implement environment toggle (Workstation vs. Cloud/Mac) at chapter start to match learner's development environment.

**Independent Test**: Environment toggle is visible at chapter start, clicking toggle switches between Workstation and Cloud/Mac content within 3 seconds, preference persists across chapters for logged-in users.

**Story Label**: [US5]

### Tasks

- [ ] T085 [P] [US5] Create dual content variants for GPU-dependent chapters: frontend/docs/module-3-isaac/isaac-sim.md (Workstation variant) and frontend/docs/module-3-isaac/isaac-sim-cloud.md (Cloud/Mac variant)
- [ ] T086 [P] [US5] Create frontend/src/components/PersonalizeButton/PersonalizeButton.tsx with environment toggle buttons (Workstation / Cloud-Mac)
- [ ] T087 [P] [US5] Create frontend/src/components/PersonalizeButton/ContentVariants.tsx to render content based on selected environment mode
- [ ] T088 [US5] Integrate PersonalizeButton component at the start of chapters (Module 3, Module 4 primarily)
- [ ] T089 [US5] Implement client-side state management in PersonalizeButton to track selected environment (default: from user hardware_profile)
- [ ] T090 [US5] Update PATCH /auth/profile endpoint to persist environment_preference changes to Neon DB
- [ ] T091 [US5] Implement local storage fallback for anonymous users (persist environment preference in browser localStorage)
- [ ] T092 [US5] Display hardware warning banner when user selects Workstation mode but is_workstation_capable() returns false
- [ ] T093 [US5] Test US5: Click environment toggle on Isaac Sim chapter, verify content switches within 3 seconds; navigate to another chapter and verify preference persists; test with logged-in and anonymous users

**Parallel Execution Example**: T085 (content creation) and T086-T087 (components) can run in parallel.

---

## Phase 8: User Story 6 - View Content in Urdu (P6)

**Goal**: Implement Urdu localization toggle at chapter start to access material in preferred language.

**Independent Test**: Language toggle is visible at chapter start, clicking toggle switches entire chapter to Urdu within 2 seconds, technical terms remain in English, preference persists across navigation, chatbot responds in Urdu when content is in Urdu mode.

**Story Label**: [US6]

### Tasks

- [ ] T094 [US6] Configure Docusaurus i18n plugin in frontend/docusaurus.config.js (locales: ['en', 'ur'], defaultLocale: 'en')
- [ ] T095 [P] [US6] Create frontend/i18n/ur/docusaurus-plugin-content-docs/current/intro.md with Urdu translation of intro (technical terms in English)
- [ ] T096 [P] [US6] Create frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-1-ros2/index.md with Urdu translation
- [ ] T097 [P] [US6] Create frontend/i18n/ur/docusaurus-plugin-content-docs/current/module-1-ros2/nodes.md with Urdu translation (preserve "ROS 2", "node", "rclpy" in English)
- [ ] T098 [P] [US6] Create Urdu translations for remaining Module 1 chapters (topics.md, services.md, urdf.md)
- [ ] T099 [P] [US6] Create Urdu translations for Module 2 chapters (index.md, gazebo.md, unity.md)
- [ ] T100 [P] [US6] Create Urdu translations for Module 3 chapters (index.md, isaac-sim.md, isaac-ros.md)
- [ ] T101 [P] [US6] Create Urdu translations for Module 4 chapters (index.md, voice-to-action.md, cognitive-planning.md)
- [ ] T102 [P] [US6] Create frontend/src/components/LanguageToggle/LanguageToggle.tsx with language toggle buttons (English / Urdu)
- [ ] T103 [US6] Integrate LanguageToggle component at the start of all chapters (uses Docusaurus useLocation hook to switch locale)
- [ ] T104 [US6] Run backend/scripts/index_content.py --language ur to populate Qdrant content_embeddings_ur collection with Urdu embeddings
- [ ] T105 [US6] Update backend/src/api/routes/chat.py to select appropriate Qdrant collection based on language parameter (en → content_embeddings_en, ur → content_embeddings_ur)
- [ ] T106 [US6] Test language-aware chatbot response in generator.py: when language=ur, respond in Urdu with English technical terms
- [ ] T107 [US6] Test US6: Click Urdu toggle on Module 1 nodes chapter, verify content switches to Urdu within 2 seconds and technical terms remain in English; ask chatbot question and verify response is in Urdu; navigate to another chapter and verify language preference persists

**Parallel Execution Example**: T095-T101 (Urdu content creation) can all run in parallel.

---

## Phase 9: Bonus Features - Subagent Intelligence & Feedback (Remaining Gamification Points)

**Goal**: Implement Claude Code URDF generation skill (50 pts) and thumbs up/down feedback mechanism to complete all 250 gamification points.

**Independent Test**: /generate-urdf skill generates valid URDF snippets, skill is documented in Module 1 chapter, feedback buttons appear below chatbot responses, thumbs down reveals text field, feedback is recorded in database.

### Tasks

- [ ] T108 [P] Create .claude/skills/generate-urdf.md with Claude Code skill definition for URDF generation (takes natural language robot description, returns URDF XML)
- [ ] T109 [P] Document URDF generation skill usage in frontend/docs/module-1-ros2/urdf.md with example invocation (/generate-urdf "two-wheeled robot with lidar sensor")
- [ ] T110 [P] Create frontend/src/components/ChatWidget/FeedbackButtons.tsx with thumbs up/down buttons below each chatbot response
- [ ] T111 [P] Create backend/src/api/routes/feedback.py with POST /feedback endpoint (accepts message_id, rating, optional_text)
- [ ] T112 [P] Create backend/src/schemas/feedback_request.py with Pydantic schemas (FeedbackRequest, FeedbackResponse)
- [ ] T113 Implement thumbs down click handler in FeedbackButtons.tsx to reveal optional text field
- [ ] T114 Integrate FeedbackButtons into MessageList.tsx to display below each assistant message
- [ ] T115 Test Bonus Features: Invoke /generate-urdf skill and verify valid URDF output; click thumbs down on a chatbot response, provide text feedback, verify feedback is recorded in Neon DB feedback table

**Parallel Execution Example**: T108-T109 (skill) and T110-T114 (feedback UI/API) can run in parallel.

---

## Phase 10: Polish & Cross-Cutting Concerns

**Goal**: Add final touches, deployment configuration, testing, and documentation to prepare for submission.

**Independent Test**: Platform accessible via public URL (GitHub Pages/Vercel + Render), all 6 user stories pass acceptance scenarios, README is complete, repository is public.

### Tasks

- [ ] T116 [P] Configure Vercel deployment in .github/workflows/deploy-frontend.yml (build command, output directory, environment variables)
- [ ] T117 [P] Configure Render deployment in .github/workflows/deploy-backend.yml (start command, environment variables)
- [ ] T118 [P] Create Playwright E2E tests in frontend/tests/e2e/ for signup flow, signin flow, environment toggle, language toggle
- [ ] T119 [P] Create pytest integration tests in backend/tests/integration/ for /chat endpoint (general query, contextual query, cross-module synthesis)
- [ ] T120 [P] Update README.md with project overview, architecture diagram, setup instructions from quickstart.md, deployment URLs, submission checklist
- [ ] T121 Verify all 250 gamification points: Contextual Query (50) ✓, Subagent Intelligence (50) ✓, Auth & Survey (50) ✓, Personalization (50) ✓, Localization (50) ✓, Base Platform (50) ✓
- [ ] T122 Run E2E tests and verify all 6 user stories pass acceptance scenarios from spec.md
- [ ] T123 Deploy frontend to Vercel and verify public URL is accessible
- [ ] T124 Deploy backend to Render and verify API is accessible and functional
- [ ] T125 Make GitHub repository public and verify README, specs/, and code are accessible
- [ ] T126 Submit Public GitHub Repo URL and Live Book URL

**Parallel Execution Example**: T116-T119 (deployment configs + tests) can run in parallel.

---

## Dependencies & Execution Order

### Story Dependency Graph

```
Setup (Phase 1) ─────┐
                     ▼
Foundational (Phase 2) ──┬──► US1: Browse Content (P1) ──────┐
                         │                                    │
                         ├──► US2: RAG Chatbot (P2) ─────────┼──► US4: Auth & Hardware Profile (P4)
                         │                      │             │                │
                         │                      ▼             │                ▼
                         └──► US3: Contextual Query (P3) ────┘    ┌─────► US5: Environment Toggle (P5)
                                                                   │
                                                                   └─────► US6: Urdu Localization (P6)
                                                                               │
                                                                               ▼
                                                                   Bonus Features (Phase 9)
                                                                               │
                                                                               ▼
                                                                   Polish & Deploy (Phase 10)
```

**Critical Path**: Setup → Foundational → US1 → US2 → US3 → US4 → US5 → US6 → Bonus → Polish

**Parallelizable Stories**:
- After Foundational: US1 and US2 can start simultaneously (independent content + RAG development)
- After US4: US5 and US6 can start simultaneously (both depend on auth for preference persistence but are otherwise independent)

### Recommended MVP Scope

**Minimum Viable Product (MVP)**: Complete **US1 (Browse Content)** and **US2 (RAG Chatbot)** only.

**Rationale**:
- US1 provides core educational value (browsable book content)
- US2 adds AI-powered querying (RAG chatbot)
- Together: 100 points (Base Platform 50 + Contextual Query potential 50 if US3 is added)
- All other stories (US3-US6, Bonus) are enhancements

**Full Delivery Scope**: All 10 phases for 250 points.

---

## Parallel Execution Examples (Per Phase)

### Phase 3 (US1): Browse Educational Content
**Parallel Tasks**: T032-T047 (all content files can be created simultaneously)
```bash
# Example: Create Module 1 chapters in parallel
parallel task {
  T034: Create nodes.md
  T035: Create topics.md
  T036: Create services.md
  T037: Create urdf.md
}
```

### Phase 4 (US2): RAG Chatbot
**Parallel Tasks**: T051-T052 (backend utilities), T061-T062 (frontend components)
```bash
# Example: Backend and Frontend development in parallel
parallel task {
  Backend: T051 (chunking), T052 (embedder), T055 (retriever), T056 (generator)
  Frontend: T061 (ChatWidget), T062 (MessageList), T063 (useChat hook)
}
```

### Phase 8 (US6): Urdu Localization
**Parallel Tasks**: T095-T101 (all Urdu translation files)
```bash
# Example: Translate all modules simultaneously
parallel task {
  T095: intro.md (Urdu)
  T096-T098: Module 1 chapters (Urdu)
  T099: Module 2 chapters (Urdu)
  T100: Module 3 chapters (Urdu)
  T101: Module 4 chapters (Urdu)
}
```

---

## Implementation Strategy

1. **Setup First**: Complete Phase 1-2 (T001-T031) before any feature development
2. **Story-by-Story**: Implement one complete user story at a time (P1 → P2 → P3 → P4 → P5 → P6)
3. **Test After Each Story**: Run independent tests after completing each phase
4. **Leverage Parallelism**: Within each phase, execute parallelizable tasks ([P] marked) simultaneously
5. **Incremental Delivery**: After each story, platform should be deployable and testable
6. **MVP Decision Point**: After US1 + US2, evaluate whether to continue to full 250-point implementation or stop at MVP

---

## Task Format Validation

✅ **All tasks follow required checklist format**:
- Checkbox: `- [ ]`
- Task ID: `T###`
- Parallel marker: `[P]` (if applicable)
- Story label: `[US#]` (for user story phases only)
- Description with file path

**Example Validation**:
- ✅ T032: `- [ ] T032 [P] [US1] Create frontend/docs/intro.md with platform overview and learning objectives`
- ✅ T057: `- [ ] T057 [US2] Create backend/src/api/routes/chat.py with POST /chat endpoint`
- ✅ T116: `- [ ] T116 [P] Configure Vercel deployment in .github/workflows/deploy-frontend.yml`

---

## Next Steps

1. **Review Tasks**: Verify task list aligns with spec.md user stories and plan.md implementation phases
2. **Begin Implementation**: Start with Phase 1 (Setup & Infrastructure)
3. **Track Progress**: Mark tasks as complete (`- [x]`) as you finish them
4. **Test Incrementally**: Run independent tests after each phase
5. **Adjust as Needed**: If blockers arise, add new tasks or reorder as necessary

**Command to Begin**: `/sp.implement` (or manually execute tasks T001-T126 in order)
