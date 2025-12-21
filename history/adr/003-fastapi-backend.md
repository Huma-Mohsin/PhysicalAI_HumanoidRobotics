# ADR-003: FastAPI for Backend

**Date:** 2025-12-21
**Status:** Accepted

## Context

The RAG chatbot requires a Python backend to:
- Process chat queries from frontend
- Generate embeddings using Cohere/OpenAI
- Search Qdrant vector database
- Call LLM APIs (Google Gemini)
- Manage user sessions and conversation history
- Deploy as serverless functions on Vercel

We needed a Python web framework that:
- Supports async/await for concurrent API calls
- Has excellent type hints and validation
- Deploys easily to Vercel serverless functions
- Provides automatic API documentation
- Has minimal overhead for fast cold starts

## Decision

We chose **FastAPI** as our backend framework.

FastAPI is a modern, high-performance Python web framework built on Starlette and Pydantic. Our backend structure:
- API routes: `backend/src/api/chat.py`, `backend/src/api/auth.py`
- Services: RAG, embeddings, Qdrant, database
- Deployment: Vercel serverless functions via `vercel.json`

## Consequences

### Positive

- **Performance**: Async/await support for concurrent Qdrant + LLM calls (2-3x faster than Flask)
- **Type Safety**: Pydantic models catch errors at development time, not runtime
- **Auto Docs**: `/docs` endpoint with Swagger UI for testing APIs
- **Validation**: Request/response validation built-in via Pydantic
- **Modern Python**: Uses latest Python 3.10+ features (type unions, pattern matching)
- **Vercel Compatible**: Works with Vercel serverless functions out of the box

### Negative

- **Startup Time**: Slightly slower cold starts than Express.js (300ms vs 150ms) due to Python runtime
- **Less Ecosystem**: Fewer plugins than Express.js or Django
- **ASGI Required**: Needs ASGI server (Uvicorn) instead of simpler WSGI
- **Dependency Size**: Larger deployment bundle than Node.js backends

### Neutral

- Python-only: Can't share code with frontend (unlike Next.js API routes)
- Requires separate deployment configuration for backend

## Alternatives Considered

- **Express.js (Node.js)**: Industry standard with huge ecosystem. Rejected because:
  - Our RAG logic was already prototyped in Python (Cohere, Qdrant SDKs)
  - Python has better ML/AI libraries for potential future features
  - Would require rewriting all embedding/RAG logic in JavaScript

- **Flask**: Simpler and more mature than FastAPI. Rejected because:
  - No native async support (Flask-Async is a workaround)
  - No built-in validation (requires Flask-Pydantic extension)
  - Slower performance for concurrent requests
  - FastAPI's auto-docs save development time

- **Django**: Full-featured framework with ORM, admin panel. Rejected because:
  - Overkill for a simple RAG API (we don't need ORM, forms, templates)
  - Larger cold start times on serverless (500-800ms)
  - More opinionated structure slows down rapid iteration

- **Next.js API Routes**: Could have kept everything in one codebase. Rejected because:
  - Python is superior for ML/AI workloads (embedding generation, future model fine-tuning)
  - Vercel serverless functions support Python well
  - Separation of concerns: frontend focuses on UI, backend on AI logic

## References

- Related specs: `specs/002-rag-chatbot/spec.md`, `specs/004-backend-rag-fix/spec.md`
- FastAPI docs: https://fastapi.tiangolo.com/
- Implementation: `backend/src/api/chat.py`
- Deployment: `backend/vercel.json`
