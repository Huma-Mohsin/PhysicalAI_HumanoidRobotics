# Feature Specification: Backend RAG Chatbot Fix

**Feature ID:** 004-backend-rag-fix
**Status:** Draft
**Priority:** P0 (Critical - Blocks Core Functionality)
**Created:** 2025-12-18
**Version:** 1.0.0

---

## Executive Summary

Fix the non-functional RAG chatbot backend by implementing the missing critical components: embedding generation pipeline, database migrations, and proper deployment configuration. This feature does NOT modify any frontend code - the chatbot UI is already perfect and will remain untouched. The goal is to make the existing backend API functional so that the chatbot can respond to user queries about the Physical AI & Humanoid Robotics book.

**Why This Exists:**
The chatbot UI exists and looks perfect, but clicking "Send" results in "Failed to Fetch" errors because:
1. Book content embeddings are missing (Qdrant collection is empty)
2. Database tables don't exist (migrations never ran)
3. Backend deployment configuration incomplete
4. Environment variables not properly configured

**Technology Stack:**
- **Backend:** FastAPI (already implemented) + Cohere API (already implemented)
- **Vector DB:** Qdrant Cloud (configured but empty)
- **Database:** Neon Serverless Postgres (configured but no schema)
- **Deployment:** Vercel Functions

**Constitutional Alignment:**
- **Principle I (Embodied Intelligence):** Enables AI teaching assistant for Physical AI book ✅
- **Principle II (Spec-Driven Architecture):** Following proper SpecKit Plus workflow ✅
- **Principle IV (Gamified Completeness):** Delivers "Reusable Intelligence" (50 pts) ✅

---

## User Scenarios & Testing

### User Story 1 - Embedding Generation from Book Content (Priority: P1)

As a developer deploying the RAG chatbot, I want the system to automatically generate and upload embeddings for all book chapters, so that the chatbot can retrieve relevant content when users ask questions.

**Why this priority**: This is the foundational requirement - without embeddings, the RAG system cannot function. The chatbot will have no knowledge to retrieve, making it completely non-functional. This blocks all other user stories.

**Independent Test**: Can be fully tested by running the embedding script, verifying Qdrant collection is populated with ~37 chunks (6 chapters × ~6 chunks each), and confirming semantic search returns relevant results for test queries like "What is ROS 2?". Delivers standalone value even without frontend integration - backend API will start working.

**Acceptance Scenarios**:

1. **Given** I have all 6 book chapters in `docs/*.mdx` format, **When** I run the embedding generation script, **Then** the script parses all chapters, chunks content into 600-token segments, generates Cohere embeddings, and uploads them to Qdrant
2. **Given** embeddings are uploaded to Qdrant, **When** I query the collection with "ROS 2 fundamentals", **Then** I receive chunks from Module 1 (03-module-1-ros2.mdx) with similarity scores > 0.7
3. **Given** the book content is updated (new chapter added or existing content modified), **When** I re-run the embedding script, **Then** old embeddings are cleared and new embeddings are generated reflecting the latest content
4. **Given** the embedding script fails mid-execution (network error, API timeout), **When** I re-run the script, **Then** it resumes or restarts cleanly without leaving partial/corrupted data in Qdrant

---

### User Story 2 - Database Schema Setup (Priority: P2)

As a developer deploying the RAG chatbot, I want database migrations to create all required tables automatically, so that the backend can store conversations, messages, and user sessions without manual SQL execution.

**Why this priority**: This enables conversation persistence and session management. Without database tables, the backend will crash when trying to save messages. This is essential infrastructure but doesn't block basic chatbot functionality if we use in-memory storage temporarily (which we won't - proper solution required).

**Independent Test**: Can be tested by running migrations against a fresh Neon Postgres database, verifying all tables exist (`user_sessions`, `conversations`, `messages`), inserting test data, and confirming backend API can read/write without errors. Can be demonstrated independently of US1 (embeddings) by stubbing RAG responses.

**Acceptance Scenarios**:

1. **Given** I have a fresh Neon Postgres database, **When** I run the migration script, **Then** tables `user_sessions`, `conversations`, and `messages` are created with correct schemas, indexes, and foreign key constraints
2. **Given** migrations have already run once, **When** I run the migration script again, **Then** it detects existing tables and skips creation (idempotent behavior)
3. **Given** the backend API receives a chat query, **When** it attempts to save a message, **Then** data is inserted successfully into `messages` table with all required fields (message_id, conversation_id, role, content, metadata, created_at)
4. **Given** a user has multiple conversations, **When** the backend fetches conversation history, **Then** it retrieves messages ordered by created_at with correct joins to `conversations` and `user_sessions` tables

---

### User Story 3 - Backend Deployment Configuration (Priority: P3)

As a developer deploying the RAG chatbot, I want the backend to be properly configured for Vercel deployment with all environment variables set, so that the frontend can successfully call backend API endpoints in production.

**Why this priority**: This makes the chatbot accessible to end users. Without proper deployment, the system only works locally. This is the final step to deliver value but requires US1 and US2 to be functional first.

**Independent Test**: Can be tested by deploying backend to Vercel, setting all environment variables (COHERE_API_KEY, QDRANT_URL, NEON_DATABASE_URL, etc.), calling the `/health` endpoint, and verifying successful response. Then test `/api/chat/query` endpoint with a real question. Can be demonstrated independently by checking deployment logs and API responses.

**Acceptance Scenarios**:

1. **Given** I have backend code ready with embeddings and migrations complete, **When** I deploy to Vercel, **Then** the deployment succeeds, API is accessible at the production URL, and `/health` endpoint returns 200 OK
2. **Given** the backend is deployed to Vercel, **When** the frontend (running on the same domain) sends a POST request to `/api/chat/query`, **Then** CORS headers are correctly configured and the request is not blocked
3. **Given** all environment variables are set in Vercel, **When** the backend starts up, **Then** it logs successful connections to Cohere API, Qdrant, and Neon Postgres without errors
4. **Given** a user asks a question via the chatbot UI, **When** the request reaches the deployed backend, **Then** the full RAG pipeline executes (embed → search → generate) and returns a response within 3 seconds (p95 latency)

---

### Edge Cases

- What happens when the embedding script is run multiple times consecutively?
  - **Expected**: Script should be idempotent - clear existing collection and re-upload fresh embeddings without duplicates or stale data
- How does the system handle missing environment variables during deployment?
  - **Expected**: Backend startup should fail gracefully with clear error messages indicating which variables are missing (e.g., "COHERE_API_KEY not found in environment")
- What if Qdrant free tier storage limit is exceeded (1GB)?
  - **Expected**: Monitor embedding count (currently ~37 chunks = ~37KB). If approaching limit, implement chunk pruning or alert developer to upgrade tier
- How does the backend behave when Neon Postgres connection pool is exhausted?
  - **Expected**: Implement connection pooling with retry logic. Queue requests or return 503 Service Unavailable if all connections busy
- What happens if a user sends a chat query before embeddings are uploaded?
  - **Expected**: Qdrant search returns 0 results → chatbot responds with "I don't have information about that yet. The knowledge base is being initialized."

---

## Requirements

### Functional Requirements

- **FR-001**: System MUST provide a script (`backend/scripts/embed_book_content.py`) that parses all MDX files in `docs/` directory, chunks content into 600-token segments, and generates Cohere embeddings
- **FR-002**: System MUST upload embeddings to Qdrant Cloud collection `humanoid_robotics_book` with metadata (chapter_id, chapter_title, section_title, chunk_index)
- **FR-003**: System MUST provide SQL migration files (`backend/migrations/001_create_tables.sql`) to create `user_sessions`, `conversations`, and `messages` tables in Neon Postgres
- **FR-004**: System MUST include rollback migration (`backend/migrations/001_rollback.sql`) to safely drop tables if needed
- **FR-005**: System MUST validate all required environment variables on backend startup and fail fast with clear error messages if any are missing
- **FR-006**: System MUST configure CORS middleware to allow requests from frontend domain (both localhost for dev and Vercel production URL)
- **FR-007**: System MUST deploy backend to Vercel as serverless functions with cold start time < 5 seconds
- **FR-008**: System MUST expose `/health` endpoint that verifies connectivity to Cohere API, Qdrant, and Neon Postgres
- **FR-009**: System MUST NOT modify any frontend code (chatbot UI, book pages, navigation, or components)
- **FR-010**: System MUST support local development with `uvicorn` for testing before deployment

### Key Entities

- **DocumentChunk**: Represents a chunk of book content with embedding
  - Attributes: chunk_id (UUID), chapter_id (string), chapter_title (string), section_title (string), chunk_text (string), embedding_vector (1024-dim float array for Cohere), chunk_index (int), metadata (JSONB)

- **Migration**: Represents a database schema version
  - Attributes: migration_id (int), name (string), executed_at (timestamp), sql_script (text)

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: Embedding generation script successfully processes all 6 chapters and uploads ~37 chunks to Qdrant with 100% success rate
- **SC-002**: Qdrant collection contains embeddings with correct dimensionality (1024-dim for Cohere embed-english-v3.0) and metadata
- **SC-003**: Database migrations create all tables without errors in a fresh Neon Postgres database
- **SC-004**: Backend `/health` endpoint returns 200 OK with connectivity status for all services (Cohere, Qdrant, Neon)
- **SC-005**: Frontend chatbot UI can send a query and receive a response from deployed backend within 3 seconds
- **SC-006**: Backend deployment to Vercel completes successfully with build time < 2 minutes
- **SC-007**: Zero frontend code changes - `git diff humanoid_robot_book/` shows no modifications
- **SC-008**: Local development environment (`uvicorn src.main:app`) runs without errors after migrations and embeddings are complete

---

## Feature Description

### What We're Building

A **complete backend infrastructure setup** for the existing RAG chatbot system. This is NOT a new feature - it's fixing the missing pieces of the already-designed backend.

**Core Components:**

1. **Embedding Generation Pipeline**
   - **Script:** `backend/scripts/embed_book_content.py`
   - **Input:** MDX files from `humanoid_robot_book/docs/` (6 chapters)
   - **Process:**
     - Parse MDX → Extract text content → Remove frontmatter
     - Chunk text (600 tokens per chunk, 100-token overlap)
     - Generate embeddings using Cohere `embed-english-v3.0` (1024-dim)
     - Upload to Qdrant with metadata (chapter_id, title, section)
   - **Output:** Populated Qdrant collection ready for semantic search

2. **Database Migrations**
   - **Script:** `backend/migrations/001_create_tables.sql`
   - **Tables:**
     - `user_sessions`: session_id (PK), user_id (FK to Better-Auth), hardware_profile, created_at, last_active
     - `conversations`: conversation_id (PK), session_id (FK), title, created_at, updated_at
     - `messages`: message_id (PK), conversation_id (FK), role (user/assistant), content, metadata (JSONB), created_at
   - **Indexes:** conversation_id, session_id, created_at for fast queries
   - **Triggers:** Auto-update `conversations.updated_at` on new message insert

3. **Deployment Configuration**
   - **Platform:** Vercel Serverless Functions
   - **Configuration:** `backend/vercel.json` with build settings
   - **Environment Variables:**
     - COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY
     - NEON_DATABASE_URL
     - CORS_ORIGINS (comma-separated list)
     - APP_ENV=production
   - **Cold Start Optimization:** Lazy-load heavy dependencies (Cohere client initialization)

4. **Health Check System**
   - **Endpoint:** `GET /health`
   - **Checks:**
     - Cohere API: Test embedding generation
     - Qdrant: Verify collection exists and is accessible
     - Neon Postgres: Test database connectivity with simple query
   - **Response:** JSON with status for each service + overall health

### Technology Stack

**Unchanged (Already Implemented):**
- **Backend Framework:** FastAPI 0.109.0 ✅
- **LLM Provider:** Cohere (command-r-08-2024 for chat, embed-english-v3.0 for embeddings) ✅
- **Vector DB:** Qdrant Cloud (free tier: 1GB storage) ✅
- **Database:** Neon Serverless Postgres (free tier: 500MB) ✅
- **Deployment:** Vercel ✅

**New Components (This Feature):**
- **MDX Parser:** `frontmatter` + `markdown` libraries for parsing book content
- **Chunking:** `tiktoken` for token counting (600-token chunks)
- **Migration Tool:** Custom Python script using `asyncpg` to execute SQL

---

## Scope

### In Scope

✅ **Embedding Pipeline:**
- Python script to parse all 6 MDX chapters
- Chunk text into 600-token segments with 100-token overlap
- Generate Cohere embeddings (1024-dim)
- Upload to Qdrant with metadata
- Idempotent execution (can re-run safely)

✅ **Database Setup:**
- SQL migration files for table creation
- Rollback migration for safe schema changes
- Python script to execute migrations against Neon
- Connection pooling configuration

✅ **Deployment Infrastructure:**
- Vercel configuration (`vercel.json`)
- Environment variable documentation
- CORS configuration for production domain
- Health check endpoint

✅ **Documentation:**
- README.md with setup instructions
- Environment variable template (.env.example)
- Deployment guide

### Out of Scope

❌ **Frontend Changes:**
- NO modifications to chatbot UI components
- NO changes to book pages or navigation
- NO updates to Docusaurus configuration

❌ **Backend Logic Changes:**
- NO modifications to existing RAG service code
- NO changes to API endpoints (already implemented)
- NO updates to authentication logic

❌ **Advanced Features:**
- Incremental embedding updates (full re-upload only)
- Embedding versioning or rollback
- Automatic re-embedding on content changes
- Advanced chunking strategies (semantic chunking, etc.)

### Dependencies

**Prerequisites:**
- Existing backend code (`backend/src/`) ✅
- Book content in MDX format (`humanoid_robot_book/docs/`) ✅
- Cohere API account with credits ✅
- Qdrant Cloud account (free tier) ✅
- Neon Postgres account (free tier) ✅
- Vercel account ✅

**External Dependencies:**
- Cohere API availability and rate limits
- Qdrant Cloud uptime (99.9% SLA)
- Neon Postgres uptime (99.95% SLA)
- Vercel deployment platform

---

## Non-Functional Requirements

### Performance
- **Embedding Generation:** Complete for all 6 chapters in < 5 minutes
- **Migration Execution:** Complete in < 10 seconds
- **Backend Cold Start:** < 5 seconds on Vercel
- **Health Check:** Respond in < 1 second

### Reliability
- **Embedding Script:** Retry failed embeddings up to 3 times with exponential backoff
- **Database Migrations:** Atomic transactions (all-or-nothing)
- **Deployment:** Automatic rollback on failed deployment
- **Health Check:** 100% accurate status reporting

### Security
- **API Keys:** Never commit to repository, use environment variables only
- **Database:** Use parameterized queries to prevent SQL injection
- **CORS:** Whitelist only known frontend domains
- **Secrets:** Mask in logs and error messages

### Maintainability
- **Code Quality:** Type hints, docstrings, PEP 8 compliant
- **Logging:** Structured logs with context (chunk_id, chapter_id, etc.)
- **Error Messages:** Clear, actionable error messages for common failures
- **Documentation:** Step-by-step guides for common tasks

---

## Risks and Mitigation

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Cohere API rate limits during embedding generation | High | Medium | Implement rate limiting (1 req/sec), batch processing, retry logic |
| Qdrant free tier insufficient for embeddings | Medium | Low | Monitor usage (currently ~37 chunks = ~37KB well within 1GB), implement chunk pruning if needed |
| Neon Postgres free tier database size exceeded | Low | Low | Implement message retention policy (30 days), monitor DB size |
| Vercel cold start timeout (10s limit) | Medium | Low | Optimize imports (lazy-load Cohere client), reduce bundle size |
| Migration script fails mid-execution | Medium | Low | Use transactions, implement rollback mechanism, test on staging database first |
| Frontend still shows "Failed to Fetch" after backend fix | High | Medium | Verify CORS configuration, check frontend API_BASE_URL, test with curl first |

---

## Future Enhancements

### Phase 2 (After MVP)
- [ ] **Incremental Embedding Updates:** Only re-embed changed chapters
- [ ] **Embedding Versioning:** Track schema versions, support rollback
- [ ] **Automated Re-embedding:** Trigger on MDX file changes (GitHub Actions)
- [ ] **Advanced Chunking:** Semantic chunking with sentence boundaries
- [ ] **Monitoring Dashboard:** Qdrant storage usage, API call counts, latency metrics

### Phase 3 (Advanced)
- [ ] **Multi-language Embeddings:** Support Urdu translations
- [ ] **Hybrid Search:** Combine semantic + keyword search
- [ ] **Embedding Cache:** CDN for faster cold starts
- [ ] **A/B Testing:** Experiment with chunking strategies

---

## Acceptance Tests

### Test 1: Embedding Generation Success
**Given** I have all 6 book chapters in `docs/` directory
**When** I run `python backend/scripts/embed_book_content.py`
**Then** the script completes without errors, logs show "Uploaded 37 chunks to Qdrant", and querying Qdrant returns embeddings

### Test 2: Qdrant Semantic Search
**Given** embeddings are uploaded to Qdrant
**When** I search with query "ROS 2 nodes and topics"
**Then** top result is from Module 1 (03-module-1-ros2.mdx) with similarity score > 0.7

### Test 3: Database Migration Success
**Given** I have a fresh Neon Postgres database
**When** I run the migration script
**Then** tables `user_sessions`, `conversations`, and `messages` are created with correct schemas

### Test 4: Backend Health Check
**Given** backend is deployed to Vercel with all environment variables set
**When** I call `GET /health`
**Then** response is 200 OK with JSON showing all services (Cohere, Qdrant, Neon) as "healthy"

### Test 5: End-to-End Chat Query
**Given** embeddings are in Qdrant, database tables exist, and backend is deployed
**When** I send POST request to `/api/chat/query` with question "What is ROS 2?"
**Then** response includes relevant content from Module 1, latency < 3 seconds, and message is saved to database

### Test 6: Frontend Integration (No Code Changes)
**Given** backend is fully functional
**When** I run `git diff humanoid_robot_book/`
**Then** output shows 0 files changed (frontend untouched)

### Test 7: CORS Configuration
**Given** frontend is running on Vercel domain
**When** chatbot sends a query to backend API
**Then** request is NOT blocked by CORS policy and response is received successfully

---

## Metrics for Success

### Usage Metrics
- **Target:** Backend API responds to 100% of valid chat queries without errors
- **Embedding Coverage:** 100% of book chapters have embeddings in Qdrant
- **Database Health:** 100% of message save operations succeed

### Technical Metrics
- **Availability:** 99.9% uptime (Vercel SLA)
- **Performance:** p95 latency < 3 seconds for chat queries
- **Cold Start:** < 5 seconds on Vercel
- **Embedding Quality:** Semantic search returns correct chapter for 90% of test queries

### Developer Experience
- **Setup Time:** New developer can set up backend in < 30 minutes following README
- **Deployment Time:** < 5 minutes from push to production
- **Error Clarity:** 100% of errors have actionable messages

---

## Appendix

### File Structure (New Files Only)
```
backend/
├── scripts/
│   ├── embed_book_content.py          # NEW: Generate and upload embeddings
│   ├── run_migrations.py              # NEW: Execute SQL migrations
│   └── validate_embeddings.py         # NEW: Verify Qdrant collection
├── migrations/
│   ├── 001_create_tables.sql          # NEW: Create database schema
│   └── 001_rollback.sql               # NEW: Rollback script
├── .env.example                       # UPDATED: Add COHERE_API_KEY
├── vercel.json                        # UPDATED: Deployment configuration
└── README.md                          # UPDATED: Setup instructions
```

### Related Documents
- `specs/004-backend-rag-fix/plan.md` - Architectural decisions (to be created)
- `specs/004-backend-rag-fix/tasks.md` - Implementation tasks (to be created)
- `specs/002-rag-chatbot/spec.md` - Original RAG chatbot specification
- `.specify/memory/constitution.md` - Project principles

### Environment Variables (Complete List)
```env
# LLM Provider
LLM_PROVIDER=cohere

# Cohere Configuration
COHERE_API_KEY=your-cohere-api-key-here
COHERE_EMBEDDING_MODEL=embed-english-v3.0
COHERE_CHAT_MODEL=command-r-08-2024

# Qdrant Vector Store
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION=humanoid_robotics_book

# Neon Serverless Postgres
NEON_DATABASE_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require

# CORS (Comma-separated)
CORS_ORIGINS=http://localhost:3000,http://localhost:3001,https://your-app.vercel.app

# Application
APP_ENV=production
LOG_LEVEL=INFO
```

---

**Specification Version:** 1.0.0
**Last Updated:** 2025-12-18
**Status:** Draft - Ready for Planning Phase ✅
