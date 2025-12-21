# ADR-004: Cohere for Embeddings

**Date:** 2025-12-21
**Status:** Accepted

## Context

The RAG system requires embedding generation to convert book chunks into vector representations for semantic search. Key requirements:

- High-quality semantic embeddings (1024 dimensions)
- Multilingual support (English + future Urdu translation)
- Fast inference (< 200ms for query embedding)
- Cost-effective for hobby project
- Good retrieval accuracy for technical robotics content

We needed an embedding model that balances quality, speed, and cost while supporting future internationalization.

## Decision

We chose **Cohere embed-multilingual-v3.0** as our primary embedding model, with OpenAI `text-embedding-3-small` as fallback.

Configuration:
- Model: `embed-multilingual-v3.0`
- Dimensions: 1024
- Input type: `search_document` for chunks, `search_query` for user questions
- Fallback: OpenAI `text-embedding-3-small` (1536 dims)

## Consequences

### Positive

- **Multilingual**: Supports 100+ languages including Urdu (critical for future translation feature)
- **Quality**: State-of-the-art performance on MTEB benchmark for semantic search
- **Free Tier**: 100 API calls/minute, sufficient for our usage (~226 chunks + user queries)
- **Fast**: ~100ms latency for single embedding generation
- **Input Types**: Separate optimization for documents vs queries improves retrieval
- **Compression**: Can reduce to 256 dimensions later if needed for performance

### Negative

- **Vendor Lock-in**: Switching to different embedding model requires re-embedding all 226 chunks
- **API Dependency**: Requires internet connection, can't run offline
- **Rate Limits**: Free tier caps at 100 calls/min (usually sufficient, but could hit limits during bulk re-embedding)
- **Vector Size**: 1024 dims larger than some alternatives (e.g., MiniLM-L6 at 384 dims)

### Neutral

- Requires API key management (`.env` file)
- Cloud-based only (no local model option used)

## Alternatives Considered

- **OpenAI text-embedding-3-small**: Excellent quality and 1536 dimensions. Rejected as primary because:
  - More expensive ($0.02/1M tokens vs Cohere free tier)
  - Slightly worse multilingual performance than Cohere
  - We use it as fallback if Cohere API fails

- **Sentence Transformers (e.g., all-MiniLM-L6-v2)**: Open-source, runs locally. Rejected because:
  - Worse quality than Cohere/OpenAI on technical content
  - Would need GPU on Vercel (not available in free tier)
  - Cold start time for loading model (~2-5 seconds)
  - Smaller 384-dim vectors = lower retrieval accuracy

- **Voyage AI**: Specialized for RAG with good benchmarks. Rejected because:
  - Newer company with uncertain long-term support
  - More expensive than Cohere
  - Less mature Python SDK

- **Google PaLM Embeddings**: Part of Google Cloud ecosystem. Rejected because:
  - Requires Google Cloud setup (we're already using Gemini for LLM)
  - More expensive than Cohere
  - Worse multilingual support

## References

- Related specs: `specs/002-rag-chatbot/spec.md`, `specs/003-localization-urdu/spec.md`
- Cohere docs: https://docs.cohere.com/docs/embeddings
- Implementation: `backend/src/services/embedding_service.py`
- Embedding script: `backend/scripts/embed_book_content.py`
- Current embeddings: 226 chunks in Qdrant collection `humanoid_robotics_book`
