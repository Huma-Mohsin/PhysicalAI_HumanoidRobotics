# Feature 011: Architecture Decision Records (ADRs)

**Priority**: P1 (Gamification requirement)
**Points**: +25 (ADR Documentation)
**Effort**: 20 minutes

## Problem
Constitution requires ADRs for architecturally significant decisions. Currently missing documentation for key decisions made during development:
- Better-Auth vs Supabase Auth
- Qdrant vs Pinecone for vector DB
- FastAPI vs Express for backend
- Cohere vs OpenAI embeddings
- Vercel serverless vs traditional hosting

## Goal
Document 5 key architectural decisions in ADR format to satisfy constitution requirements and gain +25 gamification points.

## Requirements

### Functional
1. Create ADR template if not exists
2. Document 5 architectural decisions:
   - ADR-001: Authentication framework (Better-Auth)
   - ADR-002: Vector database (Qdrant)
   - ADR-003: Backend framework (FastAPI)
   - ADR-004: Embedding model (Cohere)
   - ADR-005: Deployment platform (Vercel)

### Non-Functional
- Follow standard ADR format (Status, Context, Decision, Consequences)
- Store in `history/adr/` directory
- Reference related specs/PRs where applicable

## Success Metrics
- 5 ADRs created with complete information
- Gamification points: 200/250 → 225/250
- All ADRs follow consistent template

## Constraints
- Token budget: ~3% (minimal)
- Time: 20 minutes
- Must use existing decisions (no new architecture work)

## Out of Scope
- Creating new architectural decisions
- Refactoring existing code
- Implementation changes
