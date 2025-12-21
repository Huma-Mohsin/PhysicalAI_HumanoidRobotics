# ADR-002: Qdrant for Vector Database

**Date:** 2025-12-21
**Status:** Accepted

## Context

The RAG (Retrieval-Augmented Generation) chatbot requires a vector database to store and search embeddings of book content (226 chunks from 7 MDX chapters). Key requirements:

- Fast semantic similarity search (cosine similarity)
- Support for high-dimensional vectors (1024 dimensions for Cohere embeddings)
- Filter by metadata (chapter_id, section, hardware type)
- Serverless/cloud-hosted (no self-hosting complexity)
- Cost-effective for educational/hobby project

## Decision

We chose **Qdrant Cloud** as our vector database.

Qdrant is a high-performance vector search engine built in Rust. We use the managed cloud version with:
- Collection: `humanoid_robotics_book`
- Vector size: 1024 (Cohere embed-multilingual-v3.0)
- Distance metric: Cosine similarity
- Metadata: chapter_id, chapter_title, section_title, chunk_id

## Consequences

### Positive

- **Performance**: Sub-100ms search latency for 226 vectors, written in Rust for speed
- **Free Tier**: 1GB free cluster sufficient for our book content (< 50MB)
- **Rich Filtering**: Can filter by chapter, section, or hardware type before similarity search
- **Python SDK**: Excellent `qdrant-client` library with type hints
- **Payload Storage**: Stores full chunk text + metadata, no separate database needed
- **Scalable**: Can grow to millions of vectors if we expand content

### Negative

- **Vendor Lock-in**: Harder to migrate than SQL databases (no standard vector DB format)
- **Cold Start**: Occasional latency spikes on free tier after inactivity
- **Learning Curve**: Different mental model than traditional databases
- **Limited Queries**: Cannot do complex joins or transactions like PostgreSQL

### Neutral

- Cloud-only for our use case (self-hosted available but not needed)
- Requires separate HTTP API calls (not embedded in app like SQLite)

## Alternatives Considered

- **Pinecone**: Leading vector DB with great DX. However, free tier only supports 1 index and requires credit card. Qdrant's free tier is more generous and no credit card required.

- **Weaviate**: Open-source with GraphQL API. More complex setup for serverless deployment. Qdrant Cloud is simpler for our needs.

- **ChromaDB**: Excellent for local development and Jupyter notebooks. Less mature cloud offering compared to Qdrant. Better for prototyping than production.

- **PostgreSQL pgvector**: We already use Neon PostgreSQL for user data. pgvector extension was considered, but:
  - Slower than specialized vector DBs for large-scale search
  - Requires self-managed indexing (HNSW, IVFFlat)
  - Mixing concerns (user data + vectors in same DB)
  - Decided to keep concerns separated: Neon for structured data, Qdrant for vectors

## References

- Related specs: `specs/002-rag-chatbot/spec.md`, `specs/004-backend-rag-fix/spec.md`
- Qdrant docs: https://qdrant.tech/documentation/
- Implementation: `backend/src/services/qdrant_service.py`
- Embedding script: `backend/scripts/embed_book_content.py`
