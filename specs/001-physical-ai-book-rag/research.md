# Research & Architectural Decisions

**Feature**: Physical AI & Humanoid Robotics Capstone Book & RAG Platform
**Date**: 2025-12-05
**Phase**: Phase 0 - Technology Research and Decision Documentation

This document captures all technical research, architectural decisions, and their rationales for the platform implementation.

---

## 1. RAG Architecture: OpenAI Agents SDK vs. LangChain

### Decision
**Use OpenAI Agents SDK with ChatKit integration**

### Rationale
- **Official Support**: OpenAI Agents SDK is the official framework for building agent-based applications with OpenAI models, ensuring first-class support and compatibility with future model releases
- **ChatKit Integration**: Provides pre-built UI components and state management patterns specifically designed for conversational interfaces, reducing frontend complexity
- **Simplified Architecture**: Agents SDK abstracts away lower-level orchestration logic (prompt management, context windowing, tool calling) compared to LangChain's more granular control
- **Performance**: Direct integration with OpenAI API reduces latency by eliminating intermediate abstraction layers
- **Constitution Alignment**: Supports Embodied Intelligence principle by allowing hardware context injection into system prompts without complex middleware

### Alternatives Considered
1. **LangChain**:
   - **Pros**: Framework-agnostic, supports multiple LLM providers, extensive community ecosystem
   - **Cons**: Higher complexity for simple RAG use cases, additional abstraction overhead, steeper learning curve
   - **Rejected Because**: Over-engineered for this project's needs; OpenAI-specific features (like Agents SDK) are sufficient and simpler

2. **Custom RAG Implementation** (retriever + OpenAI Chat Completions):
   - **Pros**: Full control over every component, minimal dependencies
   - **Cons**: Requires manual implementation of context management, conversation history, and agent orchestration
   - **Rejected Because**: Reinventing the wheel; Agents SDK provides these features out-of-the-box with better reliability

### Implementation Notes
- Use `openai` Python package (v1.x) with Agents SDK
- Configure system prompts to inject user hardware profile context
- Implement retrieval function as a tool callable by the agent
- Store conversation history in Neon Postgres for session continuity

---

## 2. Content Chunking Strategy for Embeddings

### Decision
**Semantic chunking with ~500-token chunks, 50-token overlap**

### Rationale
- **Semantic Coherence**: Chunking by semantic units (paragraphs, sections) preserves meaning better than arbitrary fixed-size splits
- **Optimal Retrieval Granularity**: 500 tokens (~375 words) is large enough to contain meaningful context but small enough to avoid diluting relevance in vector search
- **Overlap for Continuity**: 50-token overlap ensures that concepts spanning chunk boundaries are not lost during retrieval
- **Performance**: Balances index size (~1000-2000 chunks for 50-100 pages) with search precision

### Alternatives Considered
1. **Fixed-size chunks (512 tokens, no overlap)**:
   - **Pros**: Simpler to implement, predictable chunk count
   - **Cons**: May split mid-sentence or mid-concept, reducing retrieval quality
   - **Rejected Because**: Semantic integrity is critical for technical educational content

2. **Document-level embeddings (entire chapter as one chunk)**:
   - **Pros**: Maximum context preserved
   - **Cons**: Poor retrieval precision (entire chapter retrieved even for specific question), exceeds LLM context window limits
   - **Rejected Because**: Dilutes relevance; user queries target specific concepts, not entire chapters

3. **Sentence-level embeddings**:
   - **Pros**: Extremely fine-grained retrieval
   - **Cons**: Fragments context, requires many more embeddings (~5000+), increases storage and latency
   - **Rejected Because**: Too granular; technical concepts often span multiple sentences

### Implementation Notes
- Use `LangChain`'s `RecursiveCharacterTextSplitter` with custom separators for Markdown structure (headers, code blocks)
- Metadata for each chunk: `{module, chapter, section_heading, language, page_url}`
- Store chunk source in Neon Postgres for citation generation

---

## 3. Dual-Language Embeddings: Separate Collections vs. Metadata Filtering

### Decision
**Separate Qdrant collections for English and Urdu**

### Rationale
- **Performance**: Separate collections reduce search space by 50% (search only the language-specific collection), improving retrieval latency
- **Embedding Quality**: English and Urdu embeddings have different vector space characteristics; separating them prevents cross-language noise
- **Simpler Queries**: No need for metadata filtering on every search; collection selection is determined by user's current language mode
- **Scalability**: Easier to tune indexing parameters (HNSW graph) separately for each language

### Alternatives Considered
1. **Single Collection with Language Metadata Filter**:
   - **Pros**: Simpler setup (one collection), easier to manage
   - **Cons**: Every query requires metadata filter, larger search space (2x embeddings), potential cross-language contamination in vector space
   - **Rejected Because**: Performance penalty for filtering; language separation is a hard requirement per spec

2. **Multi-lingual Embeddings (single embedding per content chunk)**:
   - **Pros**: One embedding represents both languages
   - **Cons**: Requires specialized multi-lingual embedding model (not OpenAI text-embedding-3-small), may reduce quality for language-specific queries
   - **Rejected Because**: OpenAI embeddings are language-specific; would require different embedding provider

### Implementation Notes
- Create two Qdrant collections: `content_embeddings_en` and `content_embeddings_ur`
- Collection configuration: `vector_size=1536` (OpenAI text-embedding-3-small), `distance=Cosine`, `on_disk_payload=True` (free tier optimization)
- Frontend passes `language` parameter in `/chat` request; backend selects appropriate collection

---

## 4. Better-Auth Integration: Client-side SDK vs. Server-side Validation

### Decision
**Client-side Better-Auth SDK with backend session verification**

### Rationale
- **User Experience**: Client-side SDK provides seamless session management in the browser (automatic token refresh, persistent sessions)
- **Security**: Backend verifies session tokens on every RAG query, ensuring unauthorized users cannot access personalized features
- **Better-Auth Design**: Official recommendation is client-side SDK for frontend apps with backend session validation via middleware
- **Stateless API**: FastAPI remains stateless; session state is managed by Better-Auth's token mechanism, validated against Neon Postgres

### Alternatives Considered
1. **Server-side Only (session cookies managed by FastAPI)**:
   - **Pros**: Full control over session logic
   - **Cons**: Requires custom auth implementation, loses Better-Auth's built-in features (OAuth, MFA, session refresh)
   - **Rejected Because**: Reinventing auth is risky; Better-Auth's SDK is battle-tested

2. **JWT Tokens (no Better-Auth)**:
   - **Pros**: Simple, stateless, widely understood
   - **Cons**: Manual implementation of signup/signin flows, password hashing, token refresh, session expiration
   - **Rejected Because**: Better-Auth provides all of this out-of-the-box with better security

### Implementation Notes
- Install `@better-auth/react` in Docusaurus frontend
- Configure Better-Auth provider with Neon Postgres connection
- FastAPI middleware: validate `Authorization: Bearer <token>` header on protected endpoints (`/chat`, `/feedback`)
- Store session expiration (7 days) in Better-Auth configuration

---

## 5. Deployment Hosting: Frontend and Backend

### Decision
**Frontend: Vercel | Backend: Render (Free Tier)**

### Rationale

#### Frontend (Vercel)
- **Docusaurus Optimized**: Vercel has first-class support for static site frameworks (automatic builds from Git)
- **Performance**: Global CDN, automatic HTTPS, optimal caching headers
- **Free Tier**: Generous limits (100GB bandwidth/month, unlimited projects)
- **DX**: Seamless GitHub integration, preview deployments for PRs
- **Alternative (GitHub Pages)**: Viable but slower builds, no preview deployments, limited customization

#### Backend (Render)
- **Free Tier**: 750 hours/month (sufficient for evaluation period), automatic HTTPS, persistent disk
- **Python Support**: Native FastAPI deployment with automatic dependency installation from `requirements.txt`
- **Database Proximity**: Render's US regions have low latency to Neon Postgres (both US-based)
- **No Cold Starts**: Unlike serverless platforms (AWS Lambda), Render keeps services warm longer (15-minute timeout vs. instant cold start)
- **Alternative (Fly.io)**: Viable but more complex configuration; Render has simpler UX for Python apps

### Alternatives Considered

**Frontend**:
1. **GitHub Pages**:
   - **Pros**: Free, simple, GitHub-native
   - **Cons**: Slower builds (GitHub Actions), no preview deployments, CNAME limitations
   - **Rejected Because**: Vercel's DX and performance are superior with same cost (free)

**Backend**:
1. **Fly.io**:
   - **Pros**: Better free tier (3 shared VMs), edge deployment
   - **Cons**: More complex config (Dockerfile required), less Python-friendly
   - **Rejected Because**: Render's zero-config Python support is faster to set up

2. **Railway**:
   - **Pros**: Excellent DX, generous free tier ($5 credit/month)
   - **Cons**: Credit-based (can exhaust mid-month), newer platform (less battle-tested)
   - **Rejected Because**: Render's hour-based free tier is more predictable

### Implementation Notes
- **Vercel**: Connect GitHub repo, set build command to `cd frontend && npm run build`, output directory to `frontend/build`
- **Render**: Create Web Service, set start command to `cd backend && uvicorn src.api.main:app --host 0.0.0.0 --port $PORT`
- Environment variables configured in both platforms' dashboards (Qdrant API key, Neon connection string, OpenAI API key, Better-Auth secrets)

---

## 6. Contextual Query Implementation Strategy

### Decision
**Frontend sends `selected_text` parameter; backend augments retrieval with selection-scoped semantic search**

### Rationale
- **User Control**: User explicitly triggers contextual query by selecting text and clicking "Ask AI" button
- **Hybrid Retrieval**: Backend performs two searches: (1) vector search on `selected_text` to find semantically similar chunks, (2) vector search on user's question. Combine results with higher weight on selection-scoped chunks.
- **Answer Constraint**: LLM prompt instructs agent to prioritize information from selection-scoped chunks and explicitly note when answer extends beyond selected text
- **Spec Compliance**: Meets FR-009 requirement for context-aware queries constrained to user-selected text

### Alternatives Considered
1. **Client-side Filtering (no backend change)**:
   - **Pros**: Simple, no backend logic change
   - **Cons**: Cannot semantically match selection to chunks; relies on exact text match (fails if selection spans multiple chunks or includes non-indexed text)
   - **Rejected Because**: Semantic search is essential for accurate contextual retrieval

2. **Selection as Entire Context (no retrieval)**:
   - **Pros**: Extremely simple, no vector search needed
   - **Cons**: Selection may be incomplete (user highlights partial concept); misses related context from same chapter
   - **Rejected Because**: User queries often need context beyond exact selection

### Implementation Notes
- Frontend: On text selection >50 chars, display floating "Ask AI" button; clicking it sends `{query, selected_text, language}` to `/chat`
- Backend: If `selected_text` present, embed it and search for top-3 similar chunks; merge with top-5 chunks from query search
- LLM prompt: "Answer the question based primarily on this selected text: {selected_text}. Use these additional chunks for context: {retrieved_chunks}. Clearly indicate if your answer goes beyond the selection."

---

## 7. Feedback Mechanism Implementation

### Decision
**Thumbs up/down buttons with optional text field, stored in Neon Postgres**

### Rationale
- **User Friction**: Low-friction buttons encourage more feedback; optional text field allows detailed feedback when needed
- **Analytics**: Rating data enables quantitative analysis (% positive responses over time); text feedback provides qualitative insights
- **Spec Compliance**: Meets clarified requirement from `/sp.clarify` (simple thumbs up/down with optional elaboration)
- **Privacy**: Feedback is anonymous unless user is authenticated; no PII required

### Implementation Notes
- `Feedback` table schema: `{id, message_id FK, rating: enum(thumbs_up, thumbs_down), optional_text: string, timestamp}`
- Frontend: Display buttons below each chatbot response; clicking thumbs down reveals text area
- Backend: `POST /feedback` endpoint stores feedback; no real-time processing (deferred to future analytics pipeline)

---

## 8. Hardware Profile Thresholds

### Decision
**RTX 4070 Ti or higher → Workstation default | Otherwise → Cloud/Mac default**

### Rationale
- **Isaac Sim Requirements**: NVIDIA Isaac Sim officially requires RTX 4070 Ti (or equivalent) for acceptable performance
- **Clear Threshold**: Provides unambiguous decision boundary; avoids gray area (e.g., RTX 3080 might work but isn't officially supported)
- **User Override**: Users can manually toggle to Workstation mode even with lower-end GPU, but system warns about hardware insufficiency
- **Spec Compliance**: Aligns with constitution's hardware specifications (RTX 4070 Ti+ for local Isaac Sim)

### Implementation Notes
- `HardwareProfile` table includes `gpu_model` field (string, user-entered)
- Backend service: `is_workstation_capable(gpu_model)` function checks for "RTX 4070 Ti", "RTX 4080", "RTX 4090", "RTX 50" (series)
- Frontend: During signup, if `is_workstation_capable() == true`, set `environment_preference = "workstation"`, else `"cloud"`
- Content toggle: If user selects Workstation mode but `is_workstation_capable() == false`, display warning banner

---

## 9. Urdu Translation Approach

### Decision
**Pre-translated content files in `i18n/ur/` directory using Docusaurus i18n plugin**

### Rationale
- **Quality**: Professional translation ensures technical accuracy (robotics and AI terminology)
- **Performance**: No runtime translation overhead; Urdu content served as static files
- **Docusaurus Native**: i18n plugin is built-in, mature, and well-documented
- **Spec Compliance**: Aligns with spec assumption of pre-translated content (not dynamic translation)

### Implementation Notes
- Docusaurus config: Enable i18n plugin with `locales: ['en', 'ur']`, `defaultLocale: 'en'`
- Content structure: `docs/` (English), `i18n/ur/docusaurus-plugin-content-docs/current/` (Urdu)
- Language toggle: Custom React component sets locale via `useLocation` hook, triggers navigation to `/ur/` prefix
- Technical terms: Translation guide ensures terms like "ROS 2", "Isaac Sim", "URDF" remain in English within Urdu text

---

## 10. Subagent Intelligence: URDF Generation Skill

### Decision
**Implement `/generate-urdf <description>` skill using Claude Code SDK**

### Rationale
- **Reusability**: Demonstrates constitution principle IV (Gamified Completeness - 50 pts for Reusable Intelligence)
- **Educational Value**: Teaches learners how to use AI agents for code generation in robotics domain
- **Practical Utility**: URDF generation is a common pain point in ROS 2 development; automating it provides real value
- **Constitution Alignment**: Bridges AI (LLM code generation) with physical robotics (robot description files)

### Implementation Notes
- Create Claude Code skill file: `.claude/skills/generate-urdf.md`
- Skill logic: Takes natural language robot description → generates URDF XML with links, joints, visual/collision geometry
- Integration: Document skill in Module 1 chapter on URDF; provide example usage
- Output: Skill returns URDF code block that users can copy or download as `.urdf` file

---

## Summary of Decisions

| # | Decision Area | Choice | Rationale |
|---|---------------|--------|-----------|
| 1 | RAG Architecture | OpenAI Agents SDK | Official support, ChatKit integration, simpler than LangChain |
| 2 | Content Chunking | Semantic, 500 tokens, 50-token overlap | Balances coherence and retrieval precision |
| 3 | Dual-Language Embeddings | Separate Qdrant collections | Performance (50% smaller search space), quality |
| 4 | Better-Auth Integration | Client-side SDK + backend validation | UX, security, leverages Better-Auth features |
| 5 | Deployment Hosting | Vercel (frontend), Render (backend) | DX, free tier generosity, performance |
| 6 | Contextual Query | Selection-scoped semantic search | Semantic accuracy, spec compliance |
| 7 | Feedback Mechanism | Thumbs up/down + optional text | Low friction, quantitative + qualitative data |
| 8 | Hardware Thresholds | RTX 4070 Ti+ = Workstation | Isaac Sim official requirements |
| 9 | Urdu Translation | Pre-translated files via i18n plugin | Quality, performance, Docusaurus native |
| 10 | Subagent Intelligence | URDF generation skill | Reusability, educational value, AI+robotics bridge |

---

## Open Questions / Future Research

1. **Content Generation Automation**: Can Claude Code autonomously generate accurate ROS 2 / Isaac Sim code examples, or will manual review be required? (Answer: Manual review required for technical accuracy)

2. **Free Tier Monitoring**: What's the best approach for tracking Qdrant/Neon usage to avoid quota exhaustion? (Answer: Implement logging middleware, set up alerts at 80% capacity)

3. **Urdu Embedding Quality**: Does OpenAI text-embedding-3-small perform well for Urdu technical content? (Answer: Assume yes; validate during indexing phase; fallback to multilingual-e5-large if quality issues)

4. **Session Refresh UX**: How to handle 7-day session expiration gracefully if user is mid-conversation? (Answer: Better-Auth SDK auto-refreshes; if expired, show modal prompting re-login)

---

**Next Artifact**: data-model.md (database schema design)
