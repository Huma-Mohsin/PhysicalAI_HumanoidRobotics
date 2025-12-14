# Implementation Plan: RAG Chatbot

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/002-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a Retrieval-Augmented Generation (RAG) chatbot that enables readers to ask questions about the Physical AI & Humanoid Robotics book content. The system supports two query modes: (1) General Q&A using semantic search across embedded book chapters, and (2) Text selection queries where users highlight specific passages for targeted explanations. All responses are personalized based on the user's hardware profile (GPU Workstation, Edge Device, or Cloud/Mac) when authenticated.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/React 19 (frontend)
**Primary Dependencies**: FastAPI, OpenAI SDK (text-embedding-3-small + GPT-4), Qdrant Python client, asyncpg (Neon Postgres), Better-Auth integration SDK
**Storage**: Neon Serverless Postgres (conversations, messages, user profiles) + Qdrant Cloud (vector embeddings)
**Testing**: pytest (backend), Jest + React Testing Library (frontend)
**Target Platform**: Web server (Linux/cloud), deployed alongside Docusaurus frontend
**Project Type**: Web application (backend + frontend integrated)
**Performance Goals**: <3 seconds p95 latency for Q&A queries, <1 second for text selection annotation
**Constraints**: OpenAI API budget monitoring, Qdrant free tier (1GB), Neon free tier (500MB), no local state (stateless backend for scalability)
**Scale/Scope**: 50+ concurrent users, 6 book chapters (~3000 lines total), conversation history retention (30 days default)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Embodied Intelligence ✅ PASS

**Requirement**: Content MUST bridge digital AI with physical robotics. RAG responses must contextualize answers with embodiment considerations.

**How this feature satisfies**:
- RAG chatbot responds to questions about Physical AI topics (ROS 2, Isaac Sim, bipedal locomotion)
- Responses automatically inject hardware context (e.g., "For RTX 4090 users, you can run this in Isaac Sim locally...")
- Text selection queries allow users to ask about code examples that control robots
- Profile-aware responses (US3) ensure embodiment guidance is hardware-appropriate

**Evidence**: FR-008 (inject user profile context), SC-004 (100% profile-specific responses)

### Principle II: Spec-Driven Architecture ✅ PASS

**Requirement**: Follow `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` workflow. ADR suggestions required, PHRs mandatory.

**How this feature satisfies**:
- Currently executing `/sp.plan` after completing `/sp.specify`
- ADR candidates identified (see research phase below)
- PHR created for specification work (003-rag-chatbot-specification.spec.prompt.md)
- Will create PHR for this planning phase

**Evidence**: Following SDD workflow strictly

### Principle III: Interactive Personalization ✅ PASS

**Requirement**: Reading experience MUST adapt to user's hardware/software context. RAG queries automatically inject user hardware context.

**How this feature satisfies**:
- User Story 3 (Profile-Aware Responses) implements hardware profile injection
- RAG service retrieves user profile from Better-Auth and includes in prompt context
- Responses differentiate between GPU/Edge/Cloud recommendations
- FR-007 and FR-008 mandate profile integration

**Evidence**: SC-004 (100% profile-specific responses when logged in)

### Principle IV: Gamified Completeness ✅ PASS

**Requirement**: All bonus objectives mandatory. RAG Chatbot delivers 50 base points toward 100-point goal.

**How this feature satisfies**:
- RAG Chatbot is base functionality (50 points)
- Integrates with Better-Auth (Feature 2) for profile injection
- Enables personalization (Feature 3) by providing context-aware answers
- Prepares infrastructure for Subagents/Skills (Feature 5) extraction

**Evidence**: This feature is P1 (highest priority) and unlocks downstream features

**GATE STATUS**: ✅ **PASSED** - All 4 constitution principles satisfied

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (architectural decisions & patterns)
├── data-model.md        # Phase 1 output (database schemas & entities)
├── quickstart.md        # Phase 1 output (local development setup)
├── contracts/           # Phase 1 output (API endpoint specifications)
│   ├── chat-query.md    # POST /api/chat/query
│   ├── text-selection.md # POST /api/chat/text-selection
│   └── conversations.md  # GET /api/chat/conversations
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

**Selected Structure**: Web application (backend + frontend)

```text
backend/
├── src/
│   ├── models/              # Database models
│   │   ├── conversation.py  # Conversation entity
│   │   ├── message.py       # Message entity
│   │   └── user.py          # User profile integration
│   ├── services/            # Business logic
│   │   ├── rag_service.py   # RAG pipeline (embedding + retrieval + generation)
│   │   ├── qdrant_service.py # Vector store operations
│   │   ├── embedding_service.py # Document chunking & embedding
│   │   └── auth_service.py  # Better-Auth integration
│   ├── api/                 # FastAPI endpoints
│   │   ├── chat.py          # Chat endpoints (query, text-selection, conversations)
│   │   └── health.py        # Health check endpoint
│   ├── utils/               # Utilities
│   │   ├── config.py        # Environment configuration
│   │   └── logger.py        # Logging setup
│   └── main.py              # FastAPI application entry point
├── tests/
│   ├── unit/                # Unit tests
│   │   ├── test_rag_service.py
│   │   └── test_embedding_service.py
│   ├── integration/         # Integration tests
│   │   └── test_chat_api.py
│   └── fixtures/            # Test fixtures
│       └── sample_conversations.json
├── requirements.txt         # Python dependencies
├── .env.example             # Environment variables template
└── README.md                # Backend setup instructions

humanoid_robot_book/
└── src/
    ├── components/          # React components
    │   ├── Chatbot/         # Chatbot UI component
    │   │   ├── Chatbot.tsx  # Main chatbot interface
    │   │   ├── ChatMessage.tsx # Individual message component
    │   │   ├── TextSelectionHandler.tsx # Text selection capture
    │   │   └── Chatbot.module.css # Chatbot styles
    │   └── Auth/            # Authentication components (Feature 2)
    └── services/            # Frontend services
        └── chatApi.ts       # API client for chat endpoints
```

**Structure Decision**:

Selected **Option 2 (Web application)** because:
1. RAG chatbot requires a separate backend service (FastAPI) for API orchestration
2. Docusaurus frontend (React-based) needs chat UI components
3. Backend handles RAG pipeline, database operations, and OpenAI SDK integration
4. Frontend handles user interaction, text selection, and API communication

Existing directories (`backend/`, `humanoid_robot_book/`) will be extended with RAG-specific modules.

## Complexity Tracking

**No violations** - Constitution check passed without exceptions.

---

## Key Architectural Decisions

### Decision 1: Cohere vs. OpenAI for RAG

**Context**: Need to choose an LLM provider for embeddings and response generation.

**Options Considered:**

1. **OpenAI (GPT-4 + text-embedding-3-small)**
   - ✅ Widely used, proven RAG performance
   - ✅ 8192-dim embeddings (higher quality)
   - ❌ Higher cost ($0.13/1M tokens for embeddings, $5/1M for GPT-4o-mini)
   - ❌ Stricter rate limits

2. **Cohere (command-r + embed-english-v3.0)** ⭐ **SELECTED**
   - ✅ RAG-optimized models (128k context window for command-r)
   - ✅ Lower cost ($0.10/1M tokens for embeddings, $0.15/1M for command-r)
   - ✅ 1024-dim embeddings (efficient, good quality)
   - ✅ Built-in RAG citation support
   - ❌ Smaller community compared to OpenAI

3. **Anthropic Claude**
   - ✅ Best-in-class reasoning
   - ❌ No embedding model (requires third-party embeddings)
   - ❌ Higher cost

**Decision**: **Cohere (command-r-08-2024 + embed-english-v3.0)**

**Rationale:**
- Cohere is purpose-built for RAG workflows with 128k context window
- Lower cost per query (~30% cheaper than OpenAI for equivalent quality)
- 1024-dim embeddings are efficient for Qdrant free tier (1GB storage)
- Future: Easy to add citation support for source attribution

**Trade-offs Accepted:**
- Smaller community means fewer examples and tutorials
- Cohere's ecosystem is less mature than OpenAI's

**Implementation Notes:**
- Use `cohere.embed(model="embed-english-v3.0", texts=[...])` for embeddings
- Use `cohere.chat(model="command-r-08-2024", messages=[...])` for generation
- Store API key in `.env` file, never commit to git

---

### Decision 2: Qdrant Cloud vs. Alternatives for Vector Search

**Context**: Need a vector database for semantic search over book embeddings.

**Options Considered:**

1. **Pinecone**
   - ✅ Managed service, easy to use
   - ✅ Good documentation
   - ❌ Free tier too small (1 index, 100k vectors)
   - ❌ Cold start latency on free tier

2. **Weaviate**
   - ✅ Open source, self-hostable
   - ✅ Built-in hybrid search (vector + keyword)
   - ❌ Requires Docker for local dev
   - ❌ No generous free tier

3. **Qdrant Cloud** ⭐ **SELECTED**
   - ✅ Free tier: 1GB storage (~1M vectors with 1024-dim)
   - ✅ Fast similarity search (<50ms for small collections)
   - ✅ Python client with async support
   - ✅ Payload filtering (chapter_id, hardware_profile)
   - ❌ Less popular than Pinecone

4. **PostgreSQL pgvector**
   - ✅ Already using Neon Postgres
   - ✅ No additional service needed
   - ❌ Slower for large-scale similarity search
   - ❌ Limited to inner product/L2 distance (no HNSW indexing)

**Decision**: **Qdrant Cloud**

**Rationale:**
- Free tier (1GB) is sufficient for MVP (6 chapters ≈ 37 chunks with 1024-dim embeddings)
- Sub-50ms query latency for small collections (<10k vectors)
- Native support for metadata filtering (filter by `chapter_id`, `hardware_profile`)
- Async Python client integrates well with FastAPI

**Trade-offs Accepted:**
- Vendor lock-in to Qdrant Cloud (mitigated by open-source Qdrant for self-hosting)
- Need to upgrade to paid tier if exceeding 1GB (~1M vectors)

**Implementation Notes:**
- Create collection with 1024-dim vectors: `client.create_collection("humanoid_robotics_book", vectors_config=VectorParams(size=1024, distance=Distance.COSINE))`
- Use `client.search(collection_name="humanoid_robotics_book", query_vector=..., limit=5)` for semantic search
- Store Qdrant URL and API key in `.env`

---

### Decision 3: Neon Serverless Postgres vs. Alternatives

**Context**: Need a relational database for conversations, messages, and user sessions.

**Options Considered:**

1. **Supabase**
   - ✅ Free tier: 500MB, 1GB bandwidth
   - ✅ Built-in auth (but we're using Better-Auth)
   - ❌ Realtime features we don't need (overhead)

2. **PlanetScale (MySQL)**
   - ✅ Generous free tier (5GB storage)
   - ❌ MySQL not ideal for JSONB storage (text_selection, metadata)
   - ❌ No native UUID support

3. **Neon Serverless Postgres** ⭐ **SELECTED**
   - ✅ Free tier: 500MB storage, autosuspend (cost-effective)
   - ✅ PostgreSQL 15+ with JSONB support
   - ✅ Native UUID type
   - ✅ Connection pooling built-in
   - ❌ 500MB limit (mitigated by message retention policy)

4. **SQLite (local file)**
   - ✅ No setup, no cost
   - ❌ Not suitable for multi-user web app
   - ❌ No concurrent write support

**Decision**: **Neon Serverless Postgres**

**Rationale:**
- PostgreSQL's JSONB type is perfect for `text_selection` and `metadata` fields
- Native UUID support for `session_id`, `conv_id`, `message_id`
- Autosuspend feature keeps costs low (free tier doesn't charge for idle time)
- Connection pooling handles concurrent requests efficiently

**Trade-offs Accepted:**
- 500MB storage limit (mitigated by 30-day message retention policy)
- Need to implement cleanup job for old conversations

**Implementation Notes:**
- Use `asyncpg` for async database operations (integrates with FastAPI)
- Schema: `user_sessions`, `conversations`, `messages` tables
- Implement retention policy: DELETE messages older than 30 days

---

### Decision 4: Anonymous Users vs. Auth-Required

**Context**: Should the chatbot require authentication or support anonymous users?

**Options Considered:**

1. **Auth Required**
   - ✅ User tracking and analytics
   - ✅ Conversation history persistence
   - ❌ Barrier to entry (users must sign up first)
   - ❌ Delays MVP (depends on Feature 003: Better-Auth)

2. **Anonymous Allowed** ⭐ **SELECTED**
   - ✅ Lower barrier to entry (try before signup)
   - ✅ MVP can launch without auth integration
   - ✅ Freemium model (anonymous = basic, auth = premium)
   - ❌ No conversation history for anonymous users
   - ❌ No personalization without profile

**Decision**: **Anonymous Allowed, Authentication Optional**

**Rationale:**
- Allows users to try the chatbot immediately without signup friction
- MVP can launch independently of Feature 003 (Better-Auth)
- Freemium model: anonymous users get basic Q&A, authenticated users get history + personalization

**Implementation:**
- **Anonymous Users**:
  - Generate `session_id` in browser (localStorage)
  - Create session in database with `user_id = NULL`
  - No conversation history after page refresh
  - Generic responses (no hardware profile)
- **Authenticated Users**:
  - Use Bearer token from Better-Auth
  - Fetch hardware profile from auth service
  - Persist conversation history across sessions
  - Personalized responses based on profile

**Trade-offs Accepted:**
- Need to handle two user flows (anonymous vs. authenticated)
- Anonymous sessions accumulate in database (need cleanup)

---

### Decision 5: Text Selection Implementation

**Context**: How to capture and verify text selections from book chapters?

**Options Considered:**

1. **Browser Selection API + Server Verification**
   - ✅ Secure (server verifies selection matches chapter content)
   - ✅ Prevents tampering (users can't inject fake selections)
   - ❌ Higher latency (round-trip to server)
   - ❌ Requires reading MDX files on server

2. **Browser Selection API Only** ⭐ **SELECTED**
   - ✅ Fast (no server verification)
   - ✅ Simple implementation
   - ❌ Users can manipulate selection (low risk)
   - ❌ No guarantee selection is from actual chapter

3. **No Text Selection Support**
   - ✅ Simplest (defer to Phase 2)
   - ❌ Misses key User Story 2 requirement

**Decision**: **Browser Selection API with Client-Side Capture**

**Rationale:**
- User Story 2 (Text Selection Queries) is P2 priority and adds significant value
- Risk of selection manipulation is low (only affects user's own experience)
- Can add server-side verification later if abuse detected
- Simpler implementation allows faster delivery

**Implementation:**
- Frontend: Use `window.getSelection()` to capture highlighted text
- Extract `chapter_id` from URL path (e.g., `/docs/03-module-1-ros2`)
- Send selection to backend: `POST /api/chat/text-selection`
  ```json
  {
    "question": "What does this mean?",
    "text_selection": {
      "text": "ROS 2 is a...",
      "chapter_id": "03-module-1-ros2",
      "context_before": "...",
      "context_after": "..."
    }
  }
  ```
- Backend: Prioritize chunks from same chapter in Qdrant search

**Trade-offs Accepted:**
- No server-side verification (trust client selection)
- Users could inject arbitrary text (mitigated by rate limiting)

---

### Decision 6: Conversation Context Management

**Context**: How much conversation history should we include in LLM context?

**Options Considered:**

1. **Full Conversation History**
   - ✅ Maximum context for LLM
   - ❌ Token costs increase linearly with conversation length
   - ❌ Exceeds context window for long conversations (>50 messages)

2. **Last N Messages (N=5)** ⭐ **SELECTED**
   - ✅ Balances context and cost
   - ✅ Handles 95% of conversations (most are <10 messages)
   - ❌ Loses context for very long conversations

3. **Sliding Window + Summarization**
   - ✅ Best context preservation
   - ❌ Complex implementation (need summarization job)
   - ❌ Higher latency (summary generation)

**Decision**: **Last 5 Messages**

**Rationale:**
- Most conversations are short (<10 messages based on similar chatbots)
- Last 5 messages provide sufficient context for follow-up questions
- Keeps token costs predictable (~500 tokens per conversation context)
- Can add summarization later if needed

**Implementation:**
- Query: `SELECT * FROM messages WHERE conv_id = $1 ORDER BY created_at DESC LIMIT 5`
- Reverse list to chronological order
- Build LLM context: `[{role: "user", content: "..."}, {role: "assistant", content: "..."}]`

**Trade-offs Accepted:**
- Long conversations (>20 messages) may lose early context
- Can implement summarization in Phase 6 (Polish) if needed

---

## Interface & API Contracts

### POST /api/chat/query

**Purpose**: General Q&A endpoint for book content queries

**Request**:
```json
{
  "question": "What is ROS 2?",
  "session_id": "uuid-v4",  // optional (generated if not provided)
  "conversation_id": "uuid-v4"  // optional (new conversation if not provided)
}
```

**Headers**:
```
Authorization: Bearer <better-auth-token>  // optional (for profile-aware responses)
Content-Type: application/json
```

**Response (200 OK)**:
```json
{
  "conversation_id": "uuid-v4",
  "message_id": "uuid-v4",
  "response": "ROS 2 is a middleware framework for robotics...",
  "timestamp": "2025-12-13T10:30:00Z",
  "metadata": {
    "latency_ms": 1250,
    "tokens_used": 450,
    "qdrant_query_ms": 45,
    "retrieved_count": 5
  }
}
```

**Error Responses**:
- `400 Bad Request`: Invalid question (empty, too long, HTML injection)
- `404 Not Found`: Session ID not found (should create session instead)
- `429 Too Many Requests`: Rate limit exceeded (10 req/min per session)
- `500 Internal Server Error`: Cohere/Qdrant/Neon failure
- `503 Service Unavailable`: Cohere API timeout

**Idempotency**: Not idempotent (creates new message each time)

**SLA**: p95 latency < 3 seconds

---

### POST /api/chat/text-selection

**Purpose**: Query endpoint for text selection queries

**Request**:
```json
{
  "question": "What does this mean?",
  "session_id": "uuid-v4",
  "conversation_id": "uuid-v4",  // optional
  "text_selection": {
    "text": "ROS 2 uses a DDS middleware...",
    "chapter_id": "03-module-1-ros2",
    "context_before": "...previous sentence...",
    "context_after": "...next sentence..."
  }
}
```

**Response**: Same as `/api/chat/query`

**Error Responses**: Same as `/api/chat/query` plus:
- `422 Unprocessable Entity`: Text selection too long (>5000 chars)

---

### GET /api/chat/conversations

**Purpose**: Retrieve conversation history for authenticated user

**Query Parameters**:
- `limit` (default: 20, max: 100)
- `offset` (default: 0)

**Headers**:
```
Authorization: Bearer <better-auth-token>  // required
```

**Response (200 OK)**:
```json
{
  "conversations": [
    {
      "conv_id": "uuid",
      "title": "ROS 2 Setup Questions",
      "summary": "Discussed ROS 2 installation...",
      "created_at": "2025-12-13T10:00:00Z",
      "updated_at": "2025-12-13T10:30:00Z"
    }
  ],
  "total": 45,
  "limit": 20,
  "offset": 0
}
```

**Error Responses**:
- `401 Unauthorized`: No Bearer token provided
- `403 Forbidden`: Invalid token

---

## Non-Functional Requirements

### Performance
- **Response Latency**: p95 < 3 seconds for general Q&A
- **Database Queries**: < 100ms per query (Neon connection pooling)
- **Vector Search**: < 50ms per Qdrant query (1024-dim cosine similarity)
- **Concurrent Users**: 50+ simultaneous conversations without degradation

### Reliability
- **Availability**: 99.9% uptime (Vercel SLA + retry logic)
- **Error Budget**: 0.1% downtime allowed (43 minutes/month)
- **Degradation Strategy**: Graceful fallback (show error message, retry button)
- **Retry Logic**: 3 retries with exponential backoff (1s, 2s, 4s)

### Security
- **Authentication**: Optional Bearer token (Better-Auth integration)
- **Authorization**: Users can only access own conversations
- **Input Validation**: Strip HTML/JS, validate UUIDs, enforce length limits
- **Rate Limiting**: 10 requests/minute per session_id (in-memory or Redis)
- **Secrets Management**: Store API keys in `.env`, never log sensitive data

### Cost
- **Unit Economics**: < $0.01 per question (including embedding + completion)
- **Monthly Budget**: ~$10/month for 1000 queries
- **Free Tiers**: Qdrant (1GB), Neon (500MB), Vercel (hobby tier)
- **Monitoring**: Track token usage per session, alert if exceeding budget

---

## Operational Readiness

### Observability

**Logs**:
- Structured JSON logs (FastAPI + Python logging)
- Fields: `timestamp`, `level`, `session_id`, `conv_id`, `message_id`, `latency_ms`, `error`
- Example: `{"timestamp": "2025-12-13T10:30:00Z", "level": "INFO", "session_id": "uuid", "latency_ms": 1250}`

**Metrics**:
- Request count (total, per endpoint)
- Latency (p50, p95, p99)
- Error rate (Cohere failures, Qdrant failures, Neon timeouts)
- Token usage (per session, per day)
- Retrieved chunk count (average per query)

**Traces**:
- Not implemented in MVP (add in Phase 6 with OpenTelemetry)

### Alerting

**Thresholds**:
- **High Error Rate**: > 5% of requests failing (alert: email to team)
- **High Latency**: p95 > 5 seconds (warning: Slack notification)
- **Budget Exceeded**: Token usage > $15/month (alert: pause service)
- **Qdrant Full**: > 900MB used (warning: implement pruning)

**On-Call Owners**:
- Backend: Primary contact for API failures
- DevOps: Primary contact for deployment issues
- Escalation: Product owner for budget/scope decisions

### Runbooks

**Common Tasks**:
1. **Re-generate Embeddings**: Run `python backend/scripts/embed_book_content.py` after book updates
2. **Clear Old Conversations**: Run `DELETE FROM messages WHERE created_at < NOW() - INTERVAL '30 days'`
3. **Check Qdrant Health**: Run `python backend/scripts/validate_embeddings.py`
4. **Monitor Token Usage**: Query Cohere dashboard for daily usage

**Incident Response**:
- **Cohere API Down**: Display "Chat temporarily unavailable" message, retry every 30 seconds
- **Qdrant Down**: Fallback to "I don't have access to book content right now" response
- **Neon Down**: Queue messages in memory (temporary), retry database insert every 10 seconds

### Deployment & Rollback

**Deployment Strategy**:
- **Backend**: Vercel serverless functions (auto-deploy on git push to `main`)
- **Frontend**: Vercel static site (Docusaurus build)
- **Migrations**: Run manually via `python backend/scripts/run_migrations.py` before deployment
- **Embeddings**: Auto-generate during deployment (GitHub Actions hook)

**Rollback Strategy**:
- **Frontend**: Revert Vercel deployment to previous version (instant rollback)
- **Backend**: Revert git commit, re-deploy (< 2 minutes)
- **Database Migrations**: Run rollback SQL script (`001_rollback.sql`)
- **Qdrant**: No rollback needed (embeddings are versioned by timestamp)

**Feature Flags**:
- Not implemented in MVP (use environment variables for feature toggles)
- Future: LaunchDarkly or Unleash for gradual rollouts

### Compatibility

**Backend Compatibility**:
- Python 3.11+ (FastAPI, asyncpg)
- Cohere API v2 (stable, no breaking changes expected)
- Qdrant v1.7+ (stable)

**Frontend Compatibility**:
- Browsers: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+
- Mobile: iOS 14+, Android 10+
- Screen Readers: VoiceOver, NVDA, JAWS

**Database Compatibility**:
- PostgreSQL 15+ (Neon Serverless)
- JSONB type for metadata fields

---

## Risk Analysis and Mitigation

### Top 3 Risks

**Risk 1: Embedding Quality (Irrelevant Results)**
- **Impact**: Users receive off-topic answers, lose trust in chatbot
- **Blast Radius**: All users affected (degraded user experience)
- **Probability**: Medium (similarity threshold may be too high)
- **Mitigation**:
  - Lower similarity threshold from 0.7 to 0.5 (Cohere-optimized)
  - Increase top-k from 5 to 10 chunks
  - Test with 20+ sample queries from real students
  - Add "Was this helpful?" feedback button (track accuracy)
- **Kill Switch**: Disable chatbot widget if accuracy < 70%

**Risk 2: API Cost Overrun**
- **Impact**: Monthly budget exceeded, service suspended
- **Blast Radius**: All users unable to use chatbot
- **Probability**: Medium (depends on usage patterns)
- **Mitigation**:
  - Set budget alert at $12/month (80% of $15 limit)
  - Implement request caching (same question = cached response)
  - Rate limiting (10 req/min per session)
  - Monitor token usage daily via Cohere dashboard
- **Guardrails**: Auto-pause chatbot if budget exceeds $15/month

**Risk 3: Better-Auth Integration Delays**
- **Impact**: User Story 3 (Profile-Aware Responses) blocked
- **Blast Radius**: Personalization feature delayed (not MVP-blocking)
- **Probability**: High (Feature 003 depends on external auth service)
- **Mitigation**:
  - Stub profile data for development (`user_id = "test-user-1"`, `hardware_profile = "GPU Workstation"`)
  - Implement backend logic independently (auth integration can be added later)
  - Deliver User Story 1 & 2 first (MVP doesn't require auth)
- **Guardrails**: Skip User Story 3 if Better-Auth not ready by MVP deadline

---

## Evaluation and Validation

### Definition of Done

**Code Quality**:
- [ ] All Python code has type hints (mypy --strict passes)
- [ ] All TypeScript code passes linting (ESLint zero warnings)
- [ ] No secrets hardcoded in source code (git-secrets scan passes)

**Testing**:
- [ ] Manual testing: 10+ sample queries return relevant answers
- [ ] Mobile testing: Chatbot UI works on iOS Safari and Android Chrome
- [ ] Error testing: Cohere API failure handled gracefully
- [ ] Rate limit testing: 11th request in 1 minute returns 429

**Documentation**:
- [ ] `backend/README.md` has setup instructions
- [ ] API contracts documented in `specs/002-rag-chatbot/contracts/`
- [ ] Environment variables template (`.env.example`) complete

**Deployment**:
- [ ] Backend deployed to Vercel (health check passes)
- [ ] Frontend integrated with Docusaurus (chatbot visible on all book pages)
- [ ] Embeddings generated and uploaded to Qdrant (37 chunks verified)
- [ ] Database migrations applied to Neon (tables created)

### Output Validation

**Format Validation**:
- API responses match JSON schema (conversation_id, message_id, response, metadata)
- Markdown rendering works (code blocks, lists, links)

**Requirement Validation**:
- User Story 1 acceptance scenarios pass (4/4 tests)
- User Story 2 acceptance scenarios pass (4/4 tests)
- User Story 3 acceptance scenarios pass (4/4 tests)

**Safety Validation**:
- No prompt injection vulnerabilities (test with malicious inputs)
- No PII leakage in logs (session_id hashed, no user_id in logs)
- Rate limiting prevents abuse (10 req/min enforced)

---

## Architectural Decision Records (ADRs)

📋 **ADR Candidates** (suggested for documentation):

1. **ADR-001: Cohere over OpenAI for RAG** (Decision 1 above)
2. **ADR-002: Qdrant Cloud for Vector Storage** (Decision 2 above)
3. **ADR-003: Anonymous User Support Strategy** (Decision 4 above)
4. **ADR-004: Conversation Context Management (Last 5 Messages)** (Decision 6 above)

These decisions meet the significance test:
- ✅ **Impact**: Long-term consequences (provider lock-in, cost structure)
- ✅ **Alternatives**: Multiple viable options considered
- ✅ **Scope**: Cross-cutting (affects backend, cost, scalability)

**Note**: User should run `/sp.adr <decision-title>` to create formal ADR documents in `history/adr/` directory.

---

**Plan Version:** 1.0.0
**Last Updated:** 2025-12-13
**Status:** Approved ✅
