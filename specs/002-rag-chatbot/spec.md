# Feature Specification: RAG Chatbot

**Feature ID:** 002-rag-chatbot
**Status:** Completed (Retroactive Documentation)
**Priority:** P1 (Core Feature)
**Created:** 2025-12-10
**Updated:** 2025-12-13
**Version:** 1.0.0

---

## Executive Summary

Build an intelligent RAG (Retrieval-Augmented Generation) chatbot that enables readers to ask questions about the Physical AI & Humanoid Robotics book content and receive accurate, context-aware answers. The chatbot supports two interaction modes: (1) General Q&A using semantic search across all book chapters, and (2) Text selection queries where users highlight specific passages for targeted explanations. All responses are personalized based on the user's hardware profile (GPU Workstation, Edge Device, or Cloud/Mac) when authenticated.

**Why This Exists:**
Students learning Physical AI need instant access to information without manually searching through 6 chapters of technical content. The RAG chatbot acts as an AI teaching assistant that understands the book's context and provides hardware-specific guidance tailored to each student's setup.

**Technology Stack:**
- **Backend:** FastAPI (Python 3.11+) + Cohere LLM (command-r-08-2024) + Qdrant Vector DB
- **Frontend:** React 19.0 + TypeScript + Docusaurus integration
- **Storage:** Neon Serverless Postgres (conversations) + Qdrant Cloud (embeddings)
- **Embedding Model:** Cohere embed-english-v3.0 (1024-dim vectors)

---

## User Scenarios & Testing

### User Story 1 - General Q&A Chat (Priority: P1)

As a reader of the Physical AI & Humanoid Robotics book, I want to ask questions about any topic covered in the book and receive accurate, contextual answers based on the book's content, so that I can quickly clarify concepts without manually searching through chapters.

**Why this priority**: This is the core value proposition of the RAG chatbot - enabling readers to get instant answers about book content. Without this, the feature has no foundational utility.

**Independent Test**: Can be fully tested by opening any chapter, typing a question like "What is ROS 2?" in the chat interface, and verifying that the response includes relevant information from Module 1 content about ROS 2. Delivers standalone value even without text selection or personalization features.

**Acceptance Scenarios**:

1. **Given** I am reading Module 1 about ROS 2, **When** I ask "What are the main components of ROS 2?", **Then** the chatbot responds with information about nodes, topics, and services extracted from the book content
2. **Given** I ask a general question like "How do I set up NVIDIA Isaac Sim?", **When** the chatbot searches the book, **Then** it returns setup instructions from Module 3 with accurate steps
3. **Given** I ask a question about a topic not covered in the book, **When** the chatbot searches for relevant content, **Then** it politely indicates the topic isn't covered and suggests related content that exists
4. **Given** I ask a follow-up question in an ongoing conversation, **When** the chatbot processes my question, **Then** it maintains context from previous messages and provides a coherent response

---

### User Story 2 - Text Selection Queries (Priority: P2)

As a reader, I want to highlight specific text passages in any chapter and ask targeted questions about that selection, so that I can get detailed explanations about confusing sections without losing context.

**Why this priority**: This enhances the basic Q&A by allowing contextual queries tied to specific content the user is reading. It's an incremental improvement over general Q&A but requires the foundation from US1.

**Independent Test**: Can be tested by highlighting a paragraph about "bipedal locomotion" in Module 4, clicking "Ask about selection", typing "Explain this in simpler terms", and verifying the response focuses specifically on the highlighted text. Can be demonstrated independently of US3 (personalization).

**Acceptance Scenarios**:

1. **Given** I highlight a paragraph about Isaac Sim in Module 3, **When** I click "Ask about this selection" and type "What does this mean?", **Then** the chatbot responds with an explanation focused specifically on the highlighted text
2. **Given** I select a code snippet from Module 1, **When** I ask "How does this code work?", **Then** the chatbot breaks down the code step-by-step based on the selection
3. **Given** I highlight text and then ask a general question, **When** the chatbot processes both inputs, **Then** it prioritizes the selected text context but can reference broader book knowledge if needed
4. **Given** I deselect text and return to general chat, **When** I ask a new question, **Then** the chatbot no longer treats the previous selection as active context

---

### User Story 3 - Profile-Aware Responses (Priority: P3)

As a logged-in user with a hardware profile (GPU Workstation, Edge Device, or Cloud/Mac), I want the chatbot to tailor its responses to my specific setup, so that I receive relevant guidance without needing to filter through advice for other platforms.

**Why this priority**: This adds personalization to enhance user experience but depends on authentication (Feature 2) being implemented. It's valuable but not required for the chatbot to function.

**Independent Test**: Can be tested by logging in as a user with "GPU Workstation" profile, asking "How do I run Isaac Sim?", and verifying the response includes local RTX GPU instructions. Then log in as a "Cloud/Mac" user, ask the same question, and verify the response mentions Omniverse Cloud instead. Requires auth integration but is independently verifiable.

**Acceptance Scenarios**:

1. **Given** I am logged in with a "GPU Workstation" hardware profile, **When** I ask "What's the best way to run simulations?", **Then** the chatbot recommends local Isaac Sim with RTX GPU setup
2. **Given** I am logged in with an "Edge Device" profile, **When** I ask the same question, **Then** the chatbot suggests Jetson-based Gazebo workflows
3. **Given** I am logged in with a "Cloud/Mac" profile, **When** I ask the same question, **Then** the chatbot recommends cloud-based Isaac Sim via Omniverse Cloud
4. **Given** I am not logged in (anonymous user), **When** I ask hardware-specific questions, **Then** the chatbot provides generic advice covering all hardware options

---

### Edge Cases

- What happens when a user asks the same question multiple times in a conversation?
  - **Expected**: Chatbot recognizes repetition and either provides a different angle or confirms previous answer
- How does the system handle very long text selections (e.g., entire chapter)?
  - **Expected**: System limits selection context to a reasonable token count (e.g., 2000 tokens) and informs user if truncation occurs
- What if a user highlights text and asks an unrelated question?
  - **Expected**: Chatbot acknowledges the selection but responds to the question as asked, potentially referencing the selection if relevant
- How does the chatbot behave when the book content is updated after embeddings are generated?
  - **Expected**: System includes a re-embedding trigger mechanism (manual or automated) to keep vector store synchronized with book updates
- What happens if OpenAI API is unavailable or rate-limited?
  - **Expected**: User sees a graceful error message like "Chat is temporarily unavailable. Please try again in a moment."
- How does the system handle concurrent questions from multiple users?
  - **Expected**: Each user session is independent; conversations don't interfere with each other

## Requirements

### Functional Requirements

- **FR-001**: System MUST allow users to type questions in a chat interface visible on every book page
- **FR-002**: System MUST retrieve relevant content from book chapters using semantic search against embedded book content
- **FR-003**: System MUST generate conversational responses using retrieved book content as context
- **FR-004**: System MUST persist conversation history for each user session so users can reference previous questions and answers
- **FR-005**: System MUST support text selection queries where users can highlight passage and ask questions specifically about that selection
- **FR-006**: System MUST differentiate between general Q&A mode and text selection mode based on user interaction
- **FR-007**: System MUST integrate with user authentication to retrieve hardware/software profile when user is logged in
- **FR-008**: System MUST inject user profile context (hardware type: GPU/Edge/Cloud, software experience) into response generation prompts when available
- **FR-009**: System MUST provide responses within 3 seconds (p95 latency) for general Q&A queries
- **FR-010**: System MUST store conversation history in persistent storage so users can access previous conversations across sessions
- **FR-011**: System MUST handle gracefully when book content doesn't contain answer to user's question
- **FR-012**: System MUST display chat interface as a floating widget or sidebar that doesn't obstruct book content
- **FR-013**: System MUST allow users to clear current conversation and start a new one
- **FR-014**: System MUST show typing indicators when generating responses to provide feedback
- **FR-015**: System MUST support markdown formatting in responses (code blocks, lists, bold, links)

### Key Entities

- **User**: Represents a reader interacting with the chatbot (anonymous or authenticated)
  - Attributes: session_id (required), user_id (optional, if logged in), hardware_profile (optional, from auth)

- **Conversation**: Represents a chat thread between a user and the chatbot
  - Attributes: conversation_id, user_session_id, created_at, updated_at, title (optional, auto-generated from first question)

- **Message**: Represents a single message in a conversation
  - Attributes: message_id, conversation_id, role (user | assistant), content, text_selection (optional), created_at

- **TextSelection**: Represents highlighted text from a chapter that provides additional context
  - Attributes: selection_id, message_id, selected_text, chapter_id, start_position, end_position

- **DocumentChunk**: Represents embedded chunks of book content for semantic search
  - Attributes: chunk_id, chapter_id, chunk_text, embedding_vector, metadata (chapter title, section, page number)

## Success Criteria

### Measurable Outcomes

- **SC-001**: 90% of user queries about book content receive relevant answers containing information from the correct chapter
- **SC-002**: Chatbot responds to queries within 3 seconds for 95% of requests (p95 latency)
- **SC-003**: Text selection queries return responses that reference the highlighted text in at least 80% of cases
- **SC-004**: Users with hardware profiles receive profile-specific responses (e.g., GPU users get local Isaac Sim guidance) in 100% of relevant queries when logged in
- **SC-005**: Conversation history persists across sessions with 100% reliability (no lost conversations)
- **SC-006**: System handles at least 50 concurrent users without degradation in response time
- **SC-007**: Users can complete a question-answer cycle (ask → receive → read) in under 30 seconds
- **SC-008**: Less than 5% of queries result in "I don't have information about that" responses when the content exists in the book
- **SC-009**: Chatbot interface remains accessible and doesn't obstruct reading experience on mobile devices (viewport width > 320px)
- **SC-010**: Follow-up questions maintain context from previous messages in the same conversation with 90% accuracy

## Implementation Clarifications

**Status**: ✅ Specification Complete (2025-12-10)

The following clarifications were confirmed to finalize the specification:

### SDK Strategy
**Decision**: Combination (Agents + Direct SDK)
- **Primary Implementation**: Direct OpenAI Python SDK for RAG pipeline (embeddings + completions)
- **Architecture**: Design with extensibility for future OpenAI Agents SDK integration
- **Rationale**: Stable foundation with room for multi-agent features (Subagents/Skills extraction per Constitution Principle IV)

### UI Embedding
**Decision**: Floating widget (bottom-right)
- **Pattern**: Expandable chat bubble that floats over book content
- **Behavior**: Minimized by default, expands to chat interface on click
- **Positioning**: Fixed bottom-right corner, z-index above content, responsive design
- **Rationale**: Doesn't obstruct reading, common UX pattern, mobile-friendly

### Authentication Model
**Decision**: Anonymous allowed, authentication optional
- **Anonymous Users**: Can access basic Q&A functionality
  - No conversation history persistence across sessions
  - Generic responses (no hardware profile personalization)
  - Session-only state (cleared on page refresh)
- **Authenticated Users** (Better-Auth integration):
  - Conversation history persists across sessions
  - Hardware profile-aware responses (US3)
  - Personalized recommendations based on user setup
- **Rationale**: Lowers barrier to entry while providing premium features for registered users

### Content Ingestion Strategy
**Decision**: Automatic on deployment
- **Trigger**: Embedding generation pipeline runs during deployment (GitHub Actions / Vercel build)
- **Process**: Parse `docs/*.mdx` → chunk content → generate embeddings → upload to Qdrant
- **Synchronization**: Book content and vector store always in sync
- **Rationale**: Zero manual intervention, content updates automatically reflected in chatbot

### Updated Functional Requirements

- **FR-016**: System MUST support anonymous users accessing basic Q&A functionality without authentication
- **FR-017**: System MUST persist conversation history only for authenticated users (Better-Auth sessions)
- **FR-018**: System MUST render chatbot as a floating widget (bottom-right corner) that doesn't obstruct book content
- **FR-019**: System MUST automatically generate and upload embeddings during deployment pipeline
- **FR-020**: System MUST be designed with extensibility for future OpenAI Agents SDK integration (multi-agent orchestration)

---

## Feature Description

### What We're Building

A **RAG-powered conversational AI assistant** integrated directly into the Physical AI & Humanoid Robotics book. The chatbot appears as a floating widget on every book page, providing instant access to book knowledge through natural language queries.

**Core Components:**

1. **Backend RAG Pipeline (FastAPI)**
   - **Embedding Service**: Chunks book content (600 tokens) and generates embeddings using Cohere embed-english-v3.0 (1024-dim vectors)
   - **Vector Search**: Semantic search using Qdrant Cloud with similarity threshold (0.5 for Cohere embeddings)
   - **LLM Generation**: Cohere command-r-08-2024 for response generation with retrieved context
   - **Database Layer**: Neon Serverless Postgres for conversations, messages, and user sessions

2. **Frontend Chat Interface (React + TypeScript)**
   - **Floating Widget**: Bottom-right chat bubble (minimized by default, expands on click)
   - **Message Display**: User/assistant messages with markdown rendering (code blocks, lists, links)
   - **Text Selection Handler**: Captures highlighted text for targeted queries
   - **Session Management**: localStorage for anonymous users, Better-Auth for authenticated users

3. **Data Flow**
   ```
   User Question → Embed Query (Cohere) → Search Qdrant (top-k=5) → Retrieve Chunks →
   Build Context → Call Cohere LLM → Generate Response → Save to Postgres → Display to User
   ```

4. **Personalization Engine**
   - Retrieves hardware profile from Better-Auth (GPU/Edge/Cloud)
   - Injects profile into LLM system prompt
   - Tailors responses to user's specific setup (e.g., "For your RTX 4090 setup, run Isaac Sim locally...")

### Technology Stack

**Backend:**
- **Framework:** FastAPI 0.104+ (async Python web framework)
- **LLM:** Cohere command-r-08-2024 (RAG-optimized, 128k context window)
- **Embedding Model:** Cohere embed-english-v3.0 (1024-dim, multilingual)
- **Vector DB:** Qdrant Cloud (free tier: 1GB storage, ~37 embeddings for 6 chapters)
- **Database:** Neon Serverless Postgres (conversations, messages, sessions)
- **Dependencies:** `cohere`, `qdrant-client`, `asyncpg`, `pydantic`

**Frontend:**
- **Framework:** React 19.0 + TypeScript
- **Integration:** Docusaurus 3.9.2 (book platform)
- **Markdown Rendering:** `react-markdown` for assistant responses
- **State Management:** React hooks (useState, useEffect, useRef)
- **API Client:** Custom `chatApi.ts` with fetch API

**Infrastructure:**
- **Deployment:** Vercel (backend + frontend)
- **CI/CD:** GitHub Actions (embedding generation on deployment)
- **Monitoring:** Custom logging (latency, token usage, errors)

---

## Scope

### In Scope

✅ **Core Functionality:**
- General Q&A about any book topic (Module 1-6 content)
- Text selection queries for targeted explanations
- Conversation history persistence (authenticated users)
- Anonymous user support (session-based, no persistence)
- Hardware profile-aware responses (GPU/Edge/Cloud)

✅ **User Experience:**
- Floating chat widget (non-intrusive, bottom-right)
- Markdown formatting in responses (code blocks, links, lists)
- Typing indicators and loading states
- Error handling and graceful degradation
- Mobile-responsive design

✅ **Data Management:**
- Automatic embedding generation from book content
- Semantic search with Qdrant vector similarity
- Conversation context (last 5 messages)
- Message metadata (latency, tokens used, retrieved chunks)

✅ **Integration:**
- Better-Auth integration for user profiles
- Docusaurus layout integration (all book pages)
- localStorage for anonymous session persistence

### Out of Scope

❌ **Advanced Features:**
- Voice input/output (future enhancement)
- Multi-language responses (except Urdu in bonus)
- Image/diagram understanding (future: GPT-4 Vision)
- Code execution or simulation (handled separately)

❌ **Authentication Features:**
- User signup/signin (handled by Feature 003: Better-Auth)
- OAuth integration (handled by Feature 003)
- User profile management (handled by Feature 003)

❌ **Performance Optimizations:**
- Caching layer (Redis) - future enhancement
- Batch embedding generation - future enhancement
- Conversation summarization - future enhancement

❌ **Advanced RAG Techniques:**
- Multi-hop reasoning (requires multi-agent orchestration)
- Source citation with page numbers (future enhancement)
- Query rewriting for better retrieval (future enhancement)

### Dependencies

**Prerequisites:**
- Docusaurus book (Feature 001) deployed and accessible
- Book content in MDX format (`docs/*.mdx`)
- Node.js 20+ and Python 3.11+ installed

**External Dependencies:**
- Cohere API (paid, ~$0.10 per 1M tokens)
- Qdrant Cloud (free tier: 1GB)
- Neon Serverless Postgres (free tier: 500MB)
- Vercel deployment (free tier for MVP)

**Feature Dependencies:**
- **Feature 003 (Better-Auth)**: Required for User Story 3 (profile-aware responses)
- **Feature 001 (Book)**: Required for content to embed and search

---

## Non-Functional Requirements

### Performance
- **Response Latency:** p95 < 3 seconds for general Q&A queries
- **Text Selection:** < 1 second to capture and display selection UI
- **Concurrent Users:** Support 50+ simultaneous conversations without degradation
- **Embedding Generation:** < 5 minutes for full book re-embedding (6 chapters)
- **Database Queries:** < 100ms for conversation history retrieval

### Scalability
- **Stateless Backend:** No in-memory session storage (horizontal scaling ready)
- **Connection Pooling:** Postgres pool (min: 2, max: 10) for efficient DB usage
- **Vector Search:** Qdrant handles 1M+ vectors (free tier: 37 chunks, room for growth)
- **Message Retention:** 30-day default (configurable per user tier)

### Reliability
- **Availability:** 99.9% uptime (Vercel SLA)
- **Error Handling:** Graceful degradation for API failures (Cohere, Qdrant, Neon)
- **Retry Logic:** 3 retries with exponential backoff for transient failures
- **Rate Limiting:** 10 requests/minute per session_id (prevent abuse)
- **Data Persistence:** 100% conversation history reliability for authenticated users

### Security
- **API Keys:** Stored in environment variables (`.env`), never exposed in client
- **Input Validation:** Sanitize user input (strip HTML/JS, validate UUIDs)
- **CORS:** Whitelist Vercel deployment domain
- **SQL Injection:** Use parameterized queries (asyncpg)
- **Rate Limiting:** Prevent DoS attacks (10 req/min per session)

### Maintainability
- **Code Quality:** Type hints (Python), TypeScript (frontend), linting (Ruff, ESLint)
- **Logging:** Structured logs with contextual info (session_id, conv_id, latency)
- **Error Tracking:** Cohere API errors, Qdrant failures, database timeouts
- **Documentation:** README.md with setup instructions, API contracts
- **Versioning:** Semantic versioning for breaking changes

---

## Risks and Mitigation

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Cohere API rate limits exceed budget | High | Medium | Implement request caching, monitor token usage, set budget alerts |
| Qdrant free tier insufficient (>1GB) | Medium | Low | Optimize chunk size (600 tokens), implement chunk pruning, upgrade to paid tier if needed |
| Embedding quality poor (irrelevant results) | High | Medium | Lower similarity threshold (0.5 → 0.4), increase top-k (5 → 10), test with sample queries |
| Neon Postgres free tier exceeded | Medium | Low | Implement message retention (30 days), archive old conversations, upgrade if needed |
| Better-Auth integration delays User Story 3 | Medium | High | Stub profile data for development, implement backend logic independently |
| User highlights entire chapter (token overflow) | Low | Medium | Limit selection length (5000 chars), truncate with warning |
| Conversation context too long (context window) | Medium | Medium | Implement conversation summarization after 20 messages |
| Deployment pipeline fails (embedding generation) | High | Low | Manual fallback script, GitHub Actions retry logic |

---

## Future Enhancements

### Phase 2 (After MVP)
- [ ] **Voice Input/Output**: Integrate OpenAI Whisper (speech-to-text) and ElevenLabs (text-to-speech)
- [ ] **Source Citations**: Link responses to specific book sections with page numbers
- [ ] **Query Suggestions**: Auto-suggest related questions based on current conversation
- [ ] **Conversation Export**: Download chat history as PDF or Markdown
- [ ] **Advanced Search**: Filter responses by chapter, module, or hardware profile

### Phase 3 (Advanced)
- [ ] **Multi-Agent Orchestration**: Use OpenAI Agents SDK for sub-task delegation
- [ ] **Code Execution**: Sandbox for running ROS 2 or Python examples directly
- [ ] **Diagram Understanding**: GPT-4 Vision for analyzing book diagrams
- [ ] **Urdu Translation**: Bilingual responses (English + Urdu)
- [ ] **Community Q&A**: Integrate with forum/Discord for peer-to-peer learning

### Phase 4 (Enterprise)
- [ ] **Admin Dashboard**: Analytics (popular questions, response quality, token usage)
- [ ] **A/B Testing**: Experiment with different prompts, models, or retrieval strategies
- [ ] **Fine-Tuned Model**: Custom Cohere model trained on Physical AI domain
- [ ] **Caching Layer**: Redis for repeated queries (reduce API costs)
- [ ] **Batch Processing**: Pre-generate answers for common questions

---

## Acceptance Tests

### Test 1: General Q&A Query
**Given** a user is reading Module 1 about ROS 2
**When** they type "What are the main components of ROS 2?" in the chatbot
**Then** the response should include information about nodes, topics, and services extracted from Module 1 content
**And** the response latency should be < 3 seconds (p95)

### Test 2: Text Selection Query
**Given** a user highlights a paragraph about Isaac Sim in Module 3
**When** they click "Ask about this selection" and type "What does this mean?"
**Then** the response should focus specifically on the highlighted text
**And** the message metadata should include `text_selection` field with the highlighted content

### Test 3: Profile-Aware Response (GPU Workstation)
**Given** a user is logged in with "GPU Workstation" hardware profile
**When** they ask "How do I run simulations?"
**Then** the response should recommend local Isaac Sim with RTX GPU setup
**And** the response should NOT mention cloud-based alternatives

### Test 4: Profile-Aware Response (Cloud/Mac)
**Given** a user is logged in with "Cloud/Mac" hardware profile
**When** they ask "How do I run simulations?"
**Then** the response should recommend Omniverse Cloud
**And** the response should NOT assume local RTX GPU availability

### Test 5: Anonymous User Experience
**Given** an anonymous user (not logged in)
**When** they ask a hardware-specific question
**Then** the response should provide generic advice covering all hardware options (GPU/Edge/Cloud)
**And** conversation history should NOT persist after page refresh

### Test 6: Conversation Context Continuity
**Given** a user asks "What is ROS 2?" and receives an answer
**When** they follow up with "How do I install it?"
**Then** the response should maintain context and provide ROS 2 installation instructions
**And** the response should NOT require re-explaining what ROS 2 is

### Test 7: Error Handling (API Failure)
**Given** the Cohere API is temporarily unavailable
**When** a user submits a question
**Then** the chatbot should display "Chat is temporarily unavailable. Please try again in a moment."
**And** the error should be logged with contextual information (session_id, timestamp)

### Test 8: Rate Limiting
**Given** a user sends 11 questions within 1 minute
**When** they attempt the 11th question
**Then** the chatbot should return a 429 Too Many Requests error
**And** display "You're asking questions too quickly. Please wait a moment."

### Test 9: Mobile Responsiveness
**Given** a user visits the book on a mobile device (viewport width 375px)
**When** they open the chatbot widget
**Then** the chat interface should be readable without horizontal scrolling
**And** the floating button should NOT obstruct book content

### Test 10: Markdown Rendering
**Given** the chatbot response contains code blocks and lists
**When** the response is displayed
**Then** code blocks should have syntax highlighting
**And** lists should render with proper indentation and bullet points

---

## Metrics for Success

### Usage Metrics
- **Target:** 80% of book readers engage with chatbot at least once per session
- **Engagement:** Average 3+ questions per conversation
- **Completion:** 90% of questions receive answers (not "I don't have information")

### Technical Metrics
- **Availability:** 99.9% uptime (measured monthly)
- **Performance:** p95 latency < 3 seconds for 95% of queries
- **Accuracy:** 90% of responses contain information from correct chapter
- **Token Efficiency:** Average < 2000 tokens per question (embedding + completion)

### User Satisfaction
- **Relevance:** 80% of users find answers helpful (post-chat survey)
- **Profile Personalization:** 100% of logged-in users receive profile-specific responses
- **Error Rate:** < 5% of queries result in API errors or degraded experience

### Cost Metrics
- **API Costs:** < $10/month for 1000 queries (Cohere API)
- **Infrastructure:** Remain within free tiers (Qdrant 1GB, Neon 500MB)
- **Cost Per Query:** < $0.01 per question (including embedding + completion)

---

## Appendix

### File Structure
```
backend/
├── src/
│   ├── models/
│   │   ├── user_session.py       # User session data model
│   │   ├── conversation.py       # Conversation entity
│   │   └── message.py            # Message entity with text selection
│   ├── services/
│   │   ├── rag_service.py        # Core RAG pipeline
│   │   ├── qdrant_service.py     # Vector search operations
│   │   ├── database_service.py   # Postgres CRUD operations
│   │   └── embedding_service.py  # Cohere embedding generation
│   ├── api/
│   │   ├── chat.py               # Chat endpoints (query, conversations)
│   │   └── health.py             # Health check endpoint
│   ├── utils/
│   │   ├── config.py             # Environment configuration
│   │   └── logger.py             # Structured logging
│   └── main.py                   # FastAPI application entry
├── migrations/
│   └── 001_create_tables.sql    # Database schema
├── scripts/
│   ├── embed_book_content.py    # Generate embeddings from MDX
│   └── setup_qdrant.py          # Initialize Qdrant collection
├── requirements.txt             # Python dependencies
├── .env.example                 # Environment variables template
└── README.md                    # Setup instructions

humanoid_robot_book/
└── src/
    ├── components/
    │   └── Chatbot/
    │       ├── Chatbot.tsx              # Main chat interface
    │       ├── ChatMessage.tsx          # Message component
    │       ├── TextSelectionHandler.tsx # Text selection capture
    │       └── Chatbot.module.css       # Widget styles
    └── services/
        └── chatApi.ts           # API client for chat endpoints
```

### Related Documents
- `specs/002-rag-chatbot/plan.md` - Architectural decisions and ADRs
- `specs/002-rag-chatbot/tasks.md` - Implementation tasks (78 tasks)
- `backend/README.md` - Backend setup instructions
- `.specify/memory/constitution.md` - Project principles

### API Endpoints
- `POST /api/chat/query` - General Q&A endpoint
- `POST /api/chat/text-selection` - Text selection query endpoint
- `GET /api/chat/conversations` - Conversation history endpoint
- `GET /api/health` - Health check endpoint

### Database Schema
- `user_sessions` - User sessions with hardware profiles
- `conversations` - Chat conversations with titles
- `messages` - Individual messages with text selections

### Environment Variables
```env
# Cohere API
COHERE_API_KEY=your_cohere_api_key
COHERE_EMBEDDING_MODEL=embed-english-v3.0
COHERE_CHAT_MODEL=command-r-08-2024

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=humanoid_robotics_book

# Neon Postgres
NEON_DATABASE_URL=postgresql://user:pass@host/db

# CORS
CORS_ORIGINS=http://localhost:3001,https://your-vercel-app.vercel.app

# RAG Configuration
SIMILARITY_THRESHOLD=0.5
TOP_K_CHUNKS=5
MAX_CONVERSATION_CONTEXT=5
```

---

**Specification Version:** 1.0.0
**Last Updated:** 2025-12-13
**Status:** Approved ✅
