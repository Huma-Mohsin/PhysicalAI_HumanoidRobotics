# Tasks: RAG Chatbot

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are NOT explicitly requested in the spec, so they are omitted per template guidance. Focus is on implementation and manual validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a **web application** with:
- Backend: `backend/src/` (FastAPI, Python)
- Frontend: `humanoid_robot_book/src/` (React, TypeScript)
- Migrations: `backend/migrations/`
- Scripts: `backend/scripts/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backend directory structure: backend/src/{models,services,api,utils}, backend/{migrations,scripts,tests}
- [ ] T002 [P] Initialize Python project with pyproject.toml or requirements.txt (FastAPI, openai, qdrant-client, asyncpg, pydantic)
- [ ] T003 [P] Create backend/.env.example with placeholders for OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL
- [ ] T004 [P] Create backend/src/utils/config.py to load environment variables using pydantic Settings
- [ ] T005 [P] Create backend/src/utils/logger.py for structured logging (using Python logging module)
- [ ] T006 [P] Create backend/src/main.py with FastAPI application initialization and CORS middleware

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database & Vector Store Setup

- [ ] T007 Create migration 001_create_tables.sql in backend/migrations/ with user_sessions, conversations, messages tables and trigger
- [ ] T008 Create migration 001_rollback.sql in backend/migrations/ for rollback strategy
- [ ] T009 Create backend/scripts/run_migrations.py to execute SQL migrations against Neon Postgres
- [ ] T010 Create backend/scripts/setup_qdrant.py to create humanoid_robotics_book collection with 1536-dim vectors

### Data Models (Shared across all user stories)

- [ ] T011 [P] Create backend/src/models/user_session.py with UserSession Pydantic model (session_id, user_id, hardware_profile, created_at, last_active)
- [ ] T012 [P] Create backend/src/models/conversation.py with Conversation Pydantic model (conv_id, session_id, title, summary, created_at, updated_at)
- [ ] T013 [P] Create backend/src/models/message.py with Message Pydantic model (message_id, conv_id, role, content, text_selection, metadata, created_at)

### Database Service Layer (Shared)

- [ ] T014 Create backend/src/services/database_service.py with asyncpg connection pool and CRUD operations for user_sessions, conversations, messages

### Embedding Pipeline (Foundational for RAG)

- [ ] T015 Create backend/src/services/embedding_service.py with OpenAI text-embedding-3-small integration
- [ ] T016 Create backend/scripts/embed_book_content.py to parse docs/*.mdx, chunk content (600 tokens), generate embeddings, upload to Qdrant
- [ ] T017 Create backend/scripts/validate_embeddings.py to verify Qdrant collection contains expected number of chunks with correct metadata

### Qdrant Service Layer

- [ ] T018 Create backend/src/services/qdrant_service.py with semantic search methods (query_similar_chunks, filter by chapter_id/hardware_profile)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - General Q&A Chat (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about book content and receive AI-generated answers based on semantic search

**Independent Test**: Open any chapter, type "What is ROS 2?" in chat interface, verify response includes relevant Module 1 content about ROS 2 components

### Backend Implementation for User Story 1

- [ ] T019 [P] [US1] Create backend/src/services/rag_service.py with query_book method (embed question, search Qdrant, assemble context, call OpenAI GPT-4o-mini)
- [ ] T020 [P] [US1] Create backend/src/api/chat.py with POST /api/chat/query endpoint implementing contract from contracts/chat-query.md
- [ ] T021 [US1] Implement session resolution logic in chat.py (create anonymous session if session_id not provided, fetch authenticated user profile if Bearer token present)
- [ ] T022 [US1] Implement conversation resolution logic in chat.py (create new conversation if conversation_id not provided, fetch existing conversation with last 5 messages)
- [ ] T023 [US1] Implement RAG pipeline orchestration in chat.py (validate input, embed question, search Qdrant, generate response, save messages)
- [ ] T024 [US1] Add error handling for OpenAI API failures (timeout, rate limit, invalid API key) with retry logic (3 retries, exponential backoff)
- [ ] T025 [US1] Add error handling for Qdrant failures (connection error, collection not found) with graceful degradation message
- [ ] T026 [US1] Add input validation and sanitization (strip HTML/JS from question, validate UUIDs, enforce 1-2000 char limit)
- [ ] T027 [US1] Add rate limiting middleware (10 requests/minute per session_id) using in-memory cache or Redis
- [ ] T028 [US1] Add response metadata tracking (latency_ms, tokens_used, qdrant_query_ms, retrieved_count)

### Frontend Implementation for User Story 1

- [ ] T029 [P] [US1] Create humanoid_robot_book/src/components/Chatbot/Chatbot.tsx with floating chat widget UI (minimized bubble, expandable panel)
- [ ] T030 [P] [US1] Create humanoid_robot_book/src/components/Chatbot/ChatMessage.tsx for individual message rendering (user/assistant, markdown support)
- [ ] T031 [P] [US1] Create humanoid_robot_book/src/components/Chatbot/Chatbot.module.css for floating widget styles (bottom-right corner, z-index, responsive)
- [ ] T032 [US1] Create humanoid_robot_book/src/services/chatApi.ts with sendQuery method calling POST /api/chat/query
- [ ] T033 [US1] Implement chat state management in Chatbot.tsx (conversation_id, session_id, messages array, loading state)
- [ ] T034 [US1] Implement message submission in Chatbot.tsx (send question to API, display loading indicator, append assistant response)
- [ ] T035 [US1] Add markdown rendering for assistant responses in ChatMessage.tsx (code blocks, lists, bold, links using react-markdown)
- [ ] T036 [US1] Add conversation history persistence in Chatbot.tsx (save session_id and conversation_id to localStorage for anonymous users)
- [ ] T037 [US1] Add error handling UI (display error messages for API failures, rate limits, network errors)
- [ ] T038 [US1] Integrate Chatbot widget into Docusaurus layout (render on all book pages, exclude from landing page)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Text Selection Queries (Priority: P2)

**Goal**: Enable users to highlight text passages and ask questions specifically about that selection

**Independent Test**: Highlight a paragraph about "bipedal locomotion" in Module 4, click "Ask about selection", type "Explain this in simpler terms", verify response focuses on highlighted text

### Backend Implementation for User Story 2

- [ ] T039 [P] [US2] Create backend/src/api/text_selection.py with POST /api/chat/text-selection endpoint implementing contract from contracts/text-selection.md
- [ ] T040 [US2] Implement selection verification logic in text_selection.py (read docs/{chapter_id}.mdx, verify selection.text matches content at start_offset:end_offset)
- [ ] T041 [US2] Implement text selection RAG pipeline in rag_service.py (embed selection+question, search with chapter_id filter, prioritize same chapter, retrieve top-k=3)
- [ ] T042 [US2] Update message storage to include text_selection JSONB field when saving user message
- [ ] T043 [US2] Add validation for text selection (max 5000 chars, valid chapter_id, valid offsets, selection matches chapter content)
- [ ] T044 [US2] Add error handling for selection verification failures (422 Unprocessable Entity if tampering detected, 404 if chapter not found)

### Frontend Implementation for User Story 2

- [ ] T045 [P] [US2] Create humanoid_robot_book/src/components/Chatbot/TextSelectionHandler.tsx to capture browser text selection events
- [ ] T046 [US2] Implement text selection capture in TextSelectionHandler.tsx (window.getSelection API, calculate offsets, extract chapter_id from URL)
- [ ] T047 [US2] Add "Ask about this selection" button in TextSelectionHandler.tsx (show button near selection, hide on deselect)
- [ ] T048 [US2] Update chatApi.ts with sendTextSelectionQuery method calling POST /api/chat/text-selection
- [ ] T049 [US2] Integrate TextSelectionHandler with Chatbot component (pass selection to chat interface, pre-fill with selection context)
- [ ] T050 [US2] Add visual indicator in ChatMessage.tsx when message has text_selection (show highlighted text context, link to original chapter location)
- [ ] T051 [US2] Add context extraction in TextSelectionHandler.tsx (capture 50 chars before/after selection for context_before/context_after)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Profile-Aware Responses (Priority: P3)

**Goal**: Personalize chatbot responses based on user's hardware/software profile (GPU/Edge/Cloud)

**Independent Test**: Log in as GPU Workstation user, ask "How do I run Isaac Sim?", verify response includes local RTX GPU instructions. Log in as Cloud/Mac user, verify response mentions Omniverse Cloud.

### Backend Implementation for User Story 3

- [ ] T052 [US3] Update backend/src/services/auth_service.py to integrate with Better-Auth API (fetch user profile with hardware_profile field)
- [ ] T053 [US3] Update session resolution in chat.py to fetch hardware_profile from Better-Auth when Bearer token present, populate user_sessions.hardware_profile
- [ ] T054 [US3] Update RAG pipeline in rag_service.py to include hardware_profile in system prompt ("User has GPU Workstation: recommend local Isaac Sim...")
- [ ] T055 [US3] Update Qdrant search in qdrant_service.py to apply hardware_relevance soft filter (boost chunks tagged with user's hardware type)
- [ ] T056 [US3] Add profile context injection logic in rag_service.py (modify system prompt based on hardware_profile: GPU/Edge/Cloud/None)
- [ ] T057 [US3] Update anonymous user handling to provide generic responses (no hardware-specific recommendations when user_id is NULL)

### Frontend Implementation for User Story 3

- [ ] T058 [US3] Update chatApi.ts to include Authorization: Bearer header when user is authenticated (fetch token from Better-Auth session)
- [ ] T059 [US3] Add profile indicator in Chatbot UI (show hardware badge: "Responses optimized for GPU Workstation" when logged in)
- [ ] T060 [US3] Add anonymous user notice in Chatbot UI ("Log in to get personalized hardware recommendations")

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Conversation History (Bonus Feature)

- [ ] T061 [P] Create backend/src/api/conversations.py with GET /api/chat/conversations endpoint implementing contract from contracts/conversations.md
- [ ] T062 Create conversation history UI in humanoid_robot_book/src/components/Chatbot/ConversationHistory.tsx (sidebar with past conversations, load on click)
- [ ] T063 Implement conversation pagination in conversations.py (limit=20, offset, include_messages query param)
- [ ] T064 Add conversation delete functionality (DELETE /api/chat/conversations/:conv_id endpoint and UI button)

### Deployment & Documentation

- [ ] T065 [P] Create backend/README.md with setup instructions, environment variables, migration steps, embedding generation
- [ ] T066 [P] Update humanoid_robot_book/README.md with chatbot integration documentation
- [ ] T067 Create backend/Dockerfile for FastAPI deployment (multi-stage build, Python 3.11-slim)
- [ ] T068 Create docker-compose.yml for local development (FastAPI backend + Postgres mock + Qdrant mock)
- [ ] T069 [P] Add deployment guide in specs/002-rag-chatbot/deployment.md (Vercel backend deployment, environment variable setup, embedding pipeline trigger)
- [ ] T070 Run embedding generation script (backend/scripts/embed_book_content.py) to populate Qdrant with book content
- [ ] T071 Validate quickstart.md instructions by following them in a fresh environment

### Performance & Monitoring

- [ ] T072 [P] Add performance monitoring to rag_service.py (log p95 latency, token usage, Qdrant query time)
- [ ] T073 [P] Add health check endpoint (GET /api/health) to verify OpenAI, Qdrant, Neon connectivity
- [ ] T074 Optimize Qdrant queries (batch embedding generation, connection pooling, index tuning for faster similarity search)
- [ ] T075 Add conversation summarization cron job (backend/scripts/summarize_conversations.py to compress messages when count > 20)

### Security Hardening

- [ ] T076 [P] Add CORS configuration in main.py (whitelist Vercel deployment domain)
- [ ] T077 [P] Add request validation middleware (enforce max payload size 10MB, timeout 10s)
- [ ] T078 Add secrets management validation (ensure no API keys exposed in logs or error messages)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 but independently testable
- **User Story 3 (P3)**: Depends on Better-Auth integration (Feature 2) for profile data - Can implement backend logic now, requires Feature 2 for full functionality

### Within Each User Story

- Backend models before services
- Services before API endpoints
- API endpoints before frontend integration
- Core implementation before error handling
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Backend and Frontend tasks within a story can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# After Foundational phase complete, launch backend tasks in parallel:
Task T019: "Create backend/src/services/rag_service.py"
Task T020: "Create backend/src/api/chat.py"

# Launch frontend tasks in parallel:
Task T029: "Create Chatbot.tsx"
Task T030: "Create ChatMessage.tsx"
Task T031: "Create Chatbot.module.css"
Task T032: "Create chatApi.ts"

# Sequential dependencies:
T021-T028 depend on T020 (API endpoint exists)
T033-T038 depend on T029, T032 (Chatbot + API client exist)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T018) - CRITICAL - blocks all stories
3. Complete Phase 3: User Story 1 (T019-T038)
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Open book chapter, ask "What is ROS 2?", verify response
   - Test error handling (empty question, OpenAI API timeout)
   - Check p95 latency < 3 seconds
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational (T001-T018) → Foundation ready
2. Add User Story 1 (T019-T038) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (T039-T051) → Test independently → Deploy/Demo
4. Add User Story 3 (T052-T060) → Test independently → Deploy/Demo (requires Feature 2: Better-Auth)
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T018)
2. Once Foundational is done:
   - **Developer A**: User Story 1 Backend (T019-T028)
   - **Developer B**: User Story 1 Frontend (T029-T038)
   - **Developer C**: User Story 2 Backend (T039-T044)
   - **Developer D**: User Story 2 Frontend (T045-T051)
3. Stories complete and integrate independently

---

## Critical Path & Blockers

### Critical Path (Minimum for MVP)
T001 → T002 → T007-T010 (Database/Qdrant setup) → T011-T014 (Models + DB service) → T015-T018 (Embedding pipeline + Qdrant service) → T019-T020 (RAG service + API) → T021-T028 (Backend logic) → T029-T038 (Frontend) → T070 (Embed book content)

**Estimated MVP Completion**: 35 tasks (T001-T006, T007-T018, T019-T038, T070)

### Blockers

- **T016 (Embedding generation script)**: BLOCKS T017, T070, all user stories (no embeddings = no RAG)
- **T007-T010 (Database/Qdrant setup)**: BLOCKS all user stories (no storage = no chatbot)
- **T014 (Database service)**: BLOCKS T021, T022 (session/conversation resolution)
- **T019 (RAG service)**: BLOCKS T023, T041 (core RAG pipeline)
- **T052 (Better-Auth integration)**: BLOCKS User Story 3 fully (profile-aware responses require auth)

### Dependencies on Other Features

- **User Story 3 (Profile-Aware)**: Requires **Feature 2 (Better-Auth)** to be implemented first
  - Can implement backend logic (T052-T057) assuming Better-Auth API contract
  - Frontend integration (T058-T060) requires active Better-Auth session

---

## Task Summary

- **Total Tasks**: 78
- **Setup Phase**: 6 tasks
- **Foundational Phase**: 12 tasks (CRITICAL - blocks all user stories)
- **User Story 1 (MVP)**: 20 tasks
- **User Story 2**: 13 tasks
- **User Story 3**: 9 tasks
- **Polish**: 18 tasks

### Task Breakdown by User Story

- **US1 (General Q&A)**: 20 tasks (Backend: 10, Frontend: 10)
- **US2 (Text Selection)**: 13 tasks (Backend: 6, Frontend: 7)
- **US3 (Profile-Aware)**: 9 tasks (Backend: 6, Frontend: 3)

### Parallel Opportunities

- **Phase 1**: 5 parallel tasks (T002-T006)
- **Phase 2**: 7 parallel tasks (T011-T013, T015-T018)
- **Phase 3**: 10 parallel tasks (T019-T020, T029-T032 within US1)
- **Phase 4**: 6 parallel tasks (T039, T045 within US2)
- **Phase 5**: 2 parallel tasks (T052, T058 within US3)
- **Phase 6**: 11 parallel tasks (T061, T065-T066, T069, T072-T073, T076-T078)

---

## Suggested MVP Scope (User Story 1 Only)

**Goal**: Deliver a working RAG chatbot with general Q&A functionality in minimal time

**Scope**:
- ✅ General Q&A queries about book content
- ✅ Conversation history persistence
- ✅ Anonymous user support
- ✅ Floating chat widget UI
- ✅ Markdown response formatting
- ✅ Error handling and rate limiting

**Out of Scope** (defer to incremental delivery):
- ❌ Text selection queries (User Story 2)
- ❌ Profile-aware responses (User Story 3)
- ❌ Conversation history UI (Polish phase)
- ❌ Performance monitoring (Polish phase)

**MVP Task Count**: 35 tasks
**MVP Task IDs**: T001-T006 (Setup), T007-T018 (Foundational), T019-T038 (US1), T070 (Embed book)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- **Tests NOT included**: Spec does not explicitly request TDD approach, so test tasks are omitted per template guidance
- **Better-Auth Dependency**: User Story 3 assumes Better-Auth (Feature 2) is implemented; can stub profile data for development
