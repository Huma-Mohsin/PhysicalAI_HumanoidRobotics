# Architecture Decision Records (ADRs)

This directory contains records of architecturally significant decisions made during the development of the Physical AI & Humanoid Robotics interactive book project.

## What is an ADR?

An Architecture Decision Record (ADR) documents a significant decision made about the architecture, design, or technology stack of a software project. Each ADR captures:

- **Context**: The problem or requirement driving the decision
- **Decision**: What was chosen and why
- **Consequences**: Positive and negative outcomes of the decision
- **Alternatives**: Other options considered and why they were rejected

ADRs help future maintainers understand **why** certain choices were made, not just **what** was implemented.

---

## Index of ADRs

### [ADR-001: Better-Auth for Authentication](./001-better-auth-authentication.md)
**Status:** Accepted | **Date:** 2025-12-21

Chose Better-Auth over Supabase Auth, Clerk, and Auth.js for user authentication with custom hardware/software profile fields.

**Key Decision:** TypeScript-first authentication with native custom field support for personalization.

---

### [ADR-002: Qdrant for Vector Database](./002-qdrant-vector-database.md)
**Status:** Accepted | **Date:** 2025-12-21

Chose Qdrant Cloud over Pinecone, Weaviate, and PostgreSQL pgvector for RAG chatbot embeddings storage.

**Key Decision:** High-performance Rust-based vector search with generous free tier (1GB cluster).

---

### [ADR-003: FastAPI for Backend](./003-fastapi-backend.md)
**Status:** Accepted | **Date:** 2025-12-21

Chose FastAPI over Express.js, Flask, and Django for Python serverless backend on Vercel.

**Key Decision:** Modern async Python framework with auto-docs and Pydantic validation for RAG API.

---

### [ADR-004: Cohere for Embeddings](./004-cohere-embeddings.md)
**Status:** Accepted | **Date:** 2025-12-21

Chose Cohere embed-multilingual-v3.0 over OpenAI, Sentence Transformers, and Voyage AI for generating vector embeddings.

**Key Decision:** Multilingual support (English + Urdu) with state-of-the-art semantic search quality and free tier.

---

### [ADR-005: Vercel for Deployment](./005-vercel-deployment.md)
**Status:** Accepted | **Date:** 2025-12-21

Chose Vercel over Netlify, AWS Lambda, Railway, and self-hosted VPS for deploying frontend and backend.

**Key Decision:** Zero-config deployment with dual Node.js/Python runtime support and auto-deploy from GitHub.

---

## Creating New ADRs

When making a new architecturally significant decision:

1. Copy `template.md` to a new file: `NNN-short-title.md`
2. Fill in all sections (Context, Decision, Consequences, Alternatives)
3. Update this README with a link and brief description
4. Commit the ADR with the code changes it documents

### What qualifies as "architecturally significant"?

A decision is worth an ADR if it:
- Affects multiple parts of the system
- Is hard to reverse later (high switching cost)
- Has important trade-offs
- Will be questioned by future developers ("why did they choose X over Y?")

Examples: Framework choices, database selection, authentication approach, deployment platform.

**Not worth an ADR:** Library choices for single features, CSS frameworks, minor refactorings.

---

## Status Definitions

- **Proposed**: Decision is being considered but not yet implemented
- **Accepted**: Decision is implemented and active
- **Deprecated**: Decision is outdated but still in use (migration planned)
- **Superseded**: Decision has been replaced by a newer ADR

---

## Related Documentation

- **Specs**: `specs/` - Feature requirements and planning
- **Constitution**: `.specify/memory/constitution.md` - Project principles
- **Prompt History**: `history/prompts/` - Development session records

---

**Last Updated:** 2025-12-21
