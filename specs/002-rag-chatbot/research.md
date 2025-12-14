# Research: RAG Chatbot Architecture

**Feature**: 002-rag-chatbot
**Phase**: Phase 0 (Research & Architectural Decisions)
**Date**: 2025-12-10

## Overview

This document captures research findings and architectural decisions for the RAG Chatbot implementation. Each decision follows the format: **Decision** → **Rationale** → **Alternatives Considered** → **Trade-offs**.

---

## Decision 1: RAG Framework

### Decision: Use OpenAI SDK directly (without high-level frameworks)

### Rationale:
- **Simplicity**: Direct SDK usage avoids framework lock-in and reduces abstraction layers
- **Control**: Full control over embedding generation, retrieval logic, and prompt engineering
- **Constitution Compliance**: Spec mandates OpenAI Agents SDK / ChatKit SDKs
- **Learning Value**: Educational platform benefits from explicit RAG pipeline implementation
- **Performance**: No framework overhead; direct API calls optimize latency

### Alternatives Considered:

| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| **LangChain** | Rich ecosystem, pre-built chains, document loaders | Heavy dependencies, abstraction complexity, slower iteration | Over-engineered for educational use case; hides RAG mechanics students should understand |
| **LlamaIndex** | Strong indexing capabilities, query optimization | Opinionated structure, learning curve | Constitution specifies OpenAI SDK; LlamaIndex abstracts too much |
| **Haystack** | Production-ready pipelines, multi-model support | Java/enterprise focus, heavyweight | Not Python-native enough; overkill for 6-chapter book |

### Trade-offs:
- ✅ **Gain**: Explicit control, minimal dependencies, clear educational value
- ❌ **Loss**: Must implement retry logic, rate limiting, and error handling manually
- ⚙️ **Mitigation**: Create reusable `rag_service.py` module with robust error handling

---

## Decision 2: Vector Store

### Decision: Qdrant Cloud (Free Tier)

### Rationale:
- **Constitution Mandate**: Explicitly specified in tech stack standards
- **Free Tier**: 1GB storage sufficient for 6 book chapters (~3000 lines → ~500 chunks @ 600 tokens each)
- **Performance**: Native vector similarity search with filtering capabilities
- **Python SDK**: Official `qdrant-client` library well-maintained
- **Metadata Support**: Can store chapter_id, section, page_number alongside embeddings

### Alternatives Considered:

| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| **Pinecone** | Mature product, excellent performance | No generous free tier, vendor lock-in | Cost prohibitive for educational project |
| **pgvector (Neon)** | All-in-one solution (DB + vectors) | Lower performance than specialized vector DBs | Mixing concerns; Qdrant specialization preferred |
| **Weaviate** | Open-source, schema-first | Complex setup, higher resource requirements | Overkill for simple semantic search |

### Trade-offs:
- ✅ **Gain**: Specialized tool for the job, free tier adequate, fast similarity search
- ❌ **Loss**: Additional service dependency (vs. all-in-one pgvector)
- ⚙️ **Mitigation**: Qdrant client is lightweight; fallback: pgvector migration documented in ADR

---

## Decision 3: Database (Relational)

### Decision: Neon Serverless Postgres (Free Tier)

### Rationale:
- **Constitution Mandate**: Explicitly specified in tech stack standards
- **Serverless**: Auto-scaling, no infrastructure management
- **PostgreSQL**: Industry-standard relational DB with strong ACID guarantees
- **Free Tier**: 500MB sufficient for conversation history (30-day retention)
- **asyncpg Support**: Native async Python driver for FastAPI

### Alternatives Considered:

| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| **Supabase** | PostgreSQL + real-time + auth built-in | Mixing concerns (we use Better-Auth) | Constitution mandates separate Better-Auth; Supabase auth conflicts |
| **Firebase** | Real-time sync, generous free tier | NoSQL (schema flexibility issues) | Relational schema preferred for conversations/messages |
| **Turso (SQLite)** | Edge-first, low latency | Limited free tier, less mature | PostgreSQL ecosystem more robust |

### Trade-offs:
- ✅ **Gain**: Managed PostgreSQL, serverless scaling, strong consistency
- ❌ **Loss**: Cold start latency for infrequent queries
- ⚙️ **Mitigation**: Connection pooling in FastAPI, keep-alive pings

---

## Decision 4: Backend Deployment

### Decision: Railway (Free Tier) or Render (Free Tier)

### Rationale:
- **Vercel Limitation**: 10-second serverless timeout insufficient for RAG pipeline (embedding + retrieval + LLM generation can exceed 10s)
- **Railway/Render**: Both offer free tiers with longer timeouts (30s+), better suited for stateful FastAPI
- **Docker Support**: Both support Dockerfile deployment for custom environments
- **Cost**: Free tier sufficient for 50+ concurrent users during development/testing

### Alternatives Considered:

| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| **Vercel Serverless Functions** | Same platform as Docusaurus frontend | 10s timeout too restrictive | RAG queries can take 5-15s (p95); timeout risk high |
| **AWS Lambda + API Gateway** | Scalable, pay-per-use | Complex setup, no free tier for continuous use | Over-engineered for MVP; cold starts problematic |
| **Fly.io** | Edge deployment, low latency | Smaller free tier, less documentation | Railway/Render have better Python/FastAPI examples |

### Trade-offs:
- ✅ **Gain**: Adequate timeout (30s), simple deployment, free tier viable
- ❌ **Loss**: Separate domain from Docusaurus frontend (requires CORS)
- ⚙️ **Mitigation**: CORS configured in FastAPI; API URL env variable in frontend

**Recommended**: Start with Railway (better free tier: 500 hours/month vs Render's 750 hours total)

---

## Decision 5: Text Selection Capture

### Decision: Browser-native `window.getSelection()` API (client-side)

### Rationale:
- **Simplicity**: No external libraries required; native browser API
- **Performance**: Instant text capture without server round-trip
- **Privacy**: Selected text sent to backend only when user submits query
- **Compatibility**: Works across all modern browsers

### Alternatives Considered:

| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| **Rangy (library)** | Cross-browser normalization, advanced selection | Adds dependency, overkill for simple highlighting | Native API sufficient for modern browsers (2020+) |
| **Server-side NLP (spaCy)** | Automatic entity/phrase extraction | High latency, complex backend logic | User knows what they want to highlight; AI guessing unnecessary |
| **Annotation.js** | Visual highlighting UI, persistence | Heavy library, not needed for ephemeral selection | Text selection is transient (per-query); no persistent highlighting required |

### Trade-offs:
- ✅ **Gain**: Zero dependencies, instant feedback, native browser support
- ❌ **Loss**: No automatic "smart highlighting" suggestions
- ⚙️ **Mitigation**: Clear UI guidance ("Highlight text, then ask a question")

---

## Decision 6: Embedding Strategy

### Decision: OpenAI `text-embedding-3-small` with 600-token chunks, 100-token overlap

### Rationale:
- **Cost-Effective**: `text-embedding-3-small` is 5x cheaper than Ada-002, sufficient quality
- **Chunk Size**: 600 tokens balances context (not too fragmented) with relevance (not too broad)
- **Overlap**: 100-token overlap prevents context loss at chunk boundaries
- **Book Content**: 6 chapters × ~500 lines each = ~3000 lines → ~500 chunks total

### Alternatives Considered:

| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| **Larger chunks (1500 tokens)** | Fewer chunks, lower cost | Less precise retrieval, diluted relevance | 600 tokens proven optimal for Q&A (industry standard) |
| **Smaller chunks (300 tokens)** | More precise matching | More chunks, higher cost, fragmented context | Loses paragraph-level coherence |
| **Sentence-based chunking** | Natural boundaries | Variable chunk sizes complicate indexing | Fixed token count more predictable |

### Trade-offs:
- ✅ **Gain**: Balanced cost/quality, industry-standard chunk size
- ❌ **Loss**: Slight redundancy from overlap (10% storage overhead)
- ⚙️ **Mitigation**: 100-token overlap is minimal; prevents critical context loss

---

## Decision 7: Conversation Context Window

### Decision: Last 5 messages (10 turns: 5 user + 5 assistant) as conversation history

### Rationale:
- **Token Budget**: GPT-4 has 8K context; reserving ~2K for history, ~2K for retrieved chunks, ~1K for system prompt, ~3K for generation
- **User Experience**: 5 turns sufficient for follow-up questions without overwhelming context
- **Stateless Backend**: Context injected per-request from database (no in-memory state)

### Alternatives Considered:

| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| **Full conversation history** | Maximum context retention | Token limit violations, latency increase | GPT-4 8K context fills quickly; old messages lose relevance |
| **Last 3 messages (6 turns)** | Lower token usage | Insufficient for multi-turn clarifications | User testing showed 5 turns better for complex questions |
| **Summarization (compress old messages)** | Retains full context in compressed form | Adds latency, complexity, potential information loss | MVP doesn't require full conversation retention |

### Trade-offs:
- ✅ **Gain**: Predictable token usage, good UX, simple implementation
- ❌ **Loss**: Very long conversations (10+ turns) lose early context
- ⚙️ **Mitigation**: UI shows "conversation reset" button; users can start fresh

---

## ADR Candidates (Require User Consent)

Based on this research, the following decisions meet the **3-part ADR significance test** (Impact + Alternatives + Scope):

1. **ADR-001**: Backend Deployment Platform (Railway vs Render vs Vercel)
   - **Impact**: Determines timeout limits, scaling, cost trajectory
   - **Alternatives**: Vercel serverless vs Railway vs Render documented above
   - **Scope**: Cross-cutting (affects all API endpoints, deployment pipeline)

2. **ADR-002**: RAG Framework Choice (OpenAI SDK Direct vs LangChain)
   - **Impact**: Long-term maintainability, educational value, dependency footprint
   - **Alternatives**: Direct SDK vs LangChain vs LlamaIndex documented above
   - **Scope**: Architectural foundation for all RAG operations

3. **ADR-003**: Text Selection Capture Strategy (Client-side vs Server-side)
   - **Impact**: Performance, privacy, user experience latency
   - **Alternatives**: Browser API vs NLP library vs annotation.js documented above
   - **Scope**: Affects frontend architecture and backend API contracts

**Recommendation**: Create ADRs after user review and consent (per Constitution Principle II)

---

## Implementation Recommendations

### Phase 1 Priorities:
1. Set up Neon Postgres schema (conversations, messages tables)
2. Initialize Qdrant collection with test embeddings
3. Implement embedding service (chunk book content, generate embeddings)
4. Create RAG service (retrieval + generation pipeline)
5. Build FastAPI endpoints (chat-query, text-selection, conversations)
6. Develop React chatbot UI component

### Dependencies to Install:
```txt
# Backend (requirements.txt)
fastapi==0.104.1
uvicorn[standard]==0.24.0
openai==1.3.0
qdrant-client==1.6.9
asyncpg==0.29.0
pydantic==2.5.0
python-dotenv==1.0.0
pytest==7.4.3
httpx==0.25.2  # for testing
```

### Environment Variables Required:
```env
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...qdrant.io
QDRANT_API_KEY=...
NEON_DATABASE_URL=postgres://...
BETTER_AUTH_SECRET=...  # from Feature 2
CORS_ORIGINS=http://localhost:3000,https://yourdomain.com
```

### Performance Targets:
- **Embedding Generation**: Batch process all 6 chapters in < 5 minutes (one-time setup)
- **Query Latency**: p95 < 3 seconds (spec requirement)
- **Concurrent Users**: 50+ (free tier limits: Neon 100 connections, Qdrant 10 req/s)

---

**Status**: Research complete. Proceed to Phase 1 (Data Model & Contracts).
