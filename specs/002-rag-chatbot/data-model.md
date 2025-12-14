# Data Model: RAG Chatbot

**Feature**: `002-rag-chatbot`
**Phase**: Phase 1 (Data Model & Contracts)
**Date**: 2025-12-10

This document defines the database schemas, entity relationships, and data validation rules for the RAG chatbot feature.

---

## Storage Architecture

### Primary Storage (Neon Serverless Postgres)
- **Purpose**: Relational data (conversations, messages, user sessions)
- **Technology**: PostgreSQL 15+ via Neon Serverless
- **Connection**: asyncpg (async Python driver)
- **Schema Management**: Raw SQL migrations (versioned in `backend/migrations/`)

### Vector Storage (Qdrant Cloud)
- **Purpose**: Document embeddings for semantic search
- **Technology**: Qdrant Cloud Free Tier (1GB)
- **Connection**: qdrant-client (official Python SDK)
- **Collection**: `humanoid_robotics_book`

---

## Entity Relationship Diagram

```
┌─────────────────┐
│  user_sessions  │  (Tracks anonymous + authenticated users)
│─────────────────│
│ session_id (PK) │
│ user_id         │◄────── (FK to Better-Auth users table, nullable)
│ hardware_profile│
│ created_at      │
│ last_active     │
└────────┬────────┘
         │
         │ 1:N
         ▼
┌─────────────────┐
│  conversations  │  (Chat threads)
│─────────────────│
│ conv_id (PK)    │
│ session_id (FK) │
│ title           │  (Auto-generated from first question)
│ summary         │  (For conversation compression, nullable)
│ created_at      │
│ updated_at      │
└────────┬────────┘
         │
         │ 1:N
         ▼
┌─────────────────┐
│    messages     │  (Individual Q&A exchanges)
│─────────────────│
│ message_id (PK) │
│ conv_id (FK)    │
│ role            │  (enum: 'user' | 'assistant')
│ content         │  (TEXT)
│ text_selection  │  (JSONB, nullable)
│ metadata        │  (JSONB, nullable - for token counts, latency, etc.)
│ created_at      │
└─────────────────┘
```

**External Reference**:
- `user_id` references Better-Auth `users` table (managed by Feature 2)

---

## Entity Definitions

### 1. user_sessions

**Purpose**: Track user sessions (anonymous or authenticated) for conversation context.

**Schema**:
```sql
CREATE TABLE user_sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NULL,  -- References Better-Auth users table
    hardware_profile JSONB NULL,  -- {type: "GPU"|"Edge"|"Cloud", details: {...}}
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_active TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Indexes
    INDEX idx_user_sessions_user_id (user_id),
    INDEX idx_user_sessions_last_active (last_active)
);

COMMENT ON TABLE user_sessions IS 'Tracks user sessions for conversation context (anonymous or authenticated)';
COMMENT ON COLUMN user_sessions.hardware_profile IS 'JSON object containing hardware type and configuration from Better-Auth profile';
```

**Validation Rules**:
- `session_id`: Must be valid UUID
- `hardware_profile`: If present, must contain `{"type": "GPU"|"Edge"|"Cloud"}`
- `last_active`: Updated on every message interaction
- **Retention**: Sessions inactive for 30+ days auto-deleted (cron job)

**State Transitions**:
1. **Anonymous User**: `session_id` created, `user_id` is NULL
2. **User Logs In**: `user_id` populated, `hardware_profile` fetched from Better-Auth
3. **Session Expires**: After 30 days of inactivity, record deleted

---

### 2. conversations

**Purpose**: Group related messages into conversation threads.

**Schema**:
```sql
CREATE TABLE conversations (
    conv_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES user_sessions(session_id) ON DELETE CASCADE,
    title VARCHAR(200) NULL,  -- Auto-generated from first user message (truncate to 200 chars)
    summary TEXT NULL,  -- LLM-generated summary after 20+ messages
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Indexes
    INDEX idx_conversations_session_id (session_id),
    INDEX idx_conversations_updated_at (updated_at DESC)
);

COMMENT ON TABLE conversations IS 'Chat threads grouping related Q&A exchanges';
COMMENT ON COLUMN conversations.summary IS 'Compressed summary of messages 1-10 when conversation exceeds 20 messages';
```

**Validation Rules**:
- `conv_id`: Must be valid UUID
- `session_id`: Must reference existing session (CASCADE delete when session deleted)
- `title`: Auto-generated from first user message (max 200 chars), nullable until first message
- `summary`: Only populated when conversation exceeds 20 messages
- `updated_at`: Automatically updated on new message via trigger

**State Transitions**:
1. **New Conversation**: Created when user sends first message in new session
2. **Active**: Updated with each new message
3. **Archived**: After 20 messages, older messages summarized and deleted, summary stored

**Trigger** (auto-update `updated_at`):
```sql
CREATE OR REPLACE FUNCTION update_conversation_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    UPDATE conversations SET updated_at = NOW() WHERE conv_id = NEW.conv_id;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER trg_update_conversation_timestamp
AFTER INSERT ON messages
FOR EACH ROW
EXECUTE FUNCTION update_conversation_timestamp();
```

---

### 3. messages

**Purpose**: Store individual user and assistant messages in conversations.

**Schema**:
```sql
CREATE TYPE message_role AS ENUM ('user', 'assistant');

CREATE TABLE messages (
    message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conv_id UUID NOT NULL REFERENCES conversations(conv_id) ON DELETE CASCADE,
    role message_role NOT NULL,
    content TEXT NOT NULL,  -- User question or assistant response
    text_selection JSONB NULL,  -- {text: string, chapter_id: string, start: int, end: int}
    metadata JSONB NULL,  -- {tokens: int, latency_ms: int, model: string, retrieved_chunks: int}
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),

    -- Indexes
    INDEX idx_messages_conv_id_created_at (conv_id, created_at DESC),
    INDEX idx_messages_role (role)
);

COMMENT ON TABLE messages IS 'Individual messages (user questions + assistant responses) in conversations';
COMMENT ON COLUMN messages.text_selection IS 'If user highlighted text, stores selection context for retrieval';
COMMENT ON COLUMN messages.metadata IS 'Performance and debugging metadata (token counts, latency, model used)';
```

**Validation Rules**:
- `message_id`: Must be valid UUID
- `conv_id`: Must reference existing conversation (CASCADE delete when conversation deleted)
- `role`: Must be 'user' or 'assistant'
- `content`: Cannot be empty (min 1 character)
- `text_selection`: If present, must contain `{text, chapter_id, start, end}`
- `metadata`: Optional JSON for analytics (token counts, latency, model version)

**JSONB Schema for `text_selection`**:
```json
{
  "text": "highlighted content from book chapter",
  "chapter_id": "03-module-1-ros2",
  "start_offset": 1234,  // Character position in chapter
  "end_offset": 1890,
  "context_before": "previous 50 chars...",  // For context window
  "context_after": "next 50 chars..."
}
```

**JSONB Schema for `metadata`**:
```json
{
  "tokens_used": 856,  // Total tokens (prompt + completion)
  "latency_ms": 2340,  // Response time in milliseconds
  "model": "gpt-4o-mini",  // OpenAI model used
  "retrieved_chunks": 5,  // Number of chunks retrieved from Qdrant
  "qdrant_query_ms": 120,  // Qdrant search latency
  "embedding_model": "text-embedding-3-small"
}
```

**State Transitions**:
1. **User Message**: Created when user submits question → role='user'
2. **Assistant Response**: Created after RAG pipeline completes → role='assistant'
3. **Retention**: Messages older than 20 per conversation are summarized and deleted (see `conversations.summary`)

---

## Qdrant Vector Store Schema

### Collection: `humanoid_robotics_book`

**Purpose**: Store embedded book chapters for semantic search.

**Configuration**:
```python
from qdrant_client.models import Distance, VectorParams, CollectionInfo

collection_config = {
    "vectors": VectorParams(
        size=1536,  # text-embedding-3-small dimensionality
        distance=Distance.COSINE,  # Cosine similarity for semantic search
        on_disk=True  # Required for free tier
    )
}
```

**Payload Schema** (metadata attached to each vector):
```json
{
  "chunk_id": "uuid-string",
  "chapter_id": "03-module-1-ros2",  // File name without extension
  "chapter_title": "Module 1: ROS 2 Fundamentals",
  "section_title": "Publisher-Subscriber Pattern",  // H2 heading
  "chunk_index": 3,  // Sequential index within chapter
  "content": "Full chunk text (600 tokens max)",
  "content_type": "text|code|mixed",  // Detected during chunking
  "hardware_relevance": ["GPU", "Edge", "Cloud", "all"],  // Applicable hardware
  "token_count": 567,
  "prev_context": "Previous 100 tokens for overlap",  // Overlap for context
  "embedding_model": "text-embedding-3-small",
  "embedded_at": "2025-12-10T12:34:56Z"  // ISO timestamp
}
```

**Indexes** (Qdrant automatically indexes):
- `chapter_id` (keyword match)
- `hardware_relevance` (array match)
- `content_type` (keyword match)

**Search Strategy**:
```python
# Example query with profile filtering
search_params = {
    "collection_name": "humanoid_robotics_book",
    "query_vector": embedding_vector,  # 1536-dim array
    "limit": 5,
    "score_threshold": 0.7,  # Minimum similarity score
    "query_filter": {
        "should": [  # Soft filter (boost, don't exclude)
            {"key": "hardware_relevance", "match": {"value": user_profile["hardware"]}},
            {"key": "hardware_relevance", "match": {"value": "all"}}  # General content
        ]
    }
}
```

---

## Data Validation Rules Summary

### user_sessions
- [x] `session_id` is valid UUID
- [x] `hardware_profile` is valid JSON (if present)
- [x] `last_active` updated on every message
- [x] Sessions inactive 30+ days are deleted

### conversations
- [x] `conv_id` is valid UUID
- [x] `session_id` references existing session
- [x] `title` max 200 characters
- [x] `summary` only populated after 20+ messages
- [x] `updated_at` auto-updated via trigger

### messages
- [x] `message_id` is valid UUID
- [x] `conv_id` references existing conversation
- [x] `role` is 'user' or 'assistant'
- [x] `content` is non-empty
- [x] `text_selection` has required fields (if present)
- [x] `metadata` is valid JSON (if present)
- [x] Messages older than 20 per conversation are summarized/deleted

### Qdrant Collection
- [x] Vector size is 1536
- [x] Distance metric is Cosine
- [x] Payload contains required fields (chunk_id, chapter_id, content)
- [x] `hardware_relevance` is array of valid values

---

## Migration Strategy

### Initial Setup (Migration 001)
```sql
-- 001_create_tables.sql
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

CREATE TABLE user_sessions (...);
CREATE TABLE conversations (...);
CREATE TYPE message_role AS ENUM (...);
CREATE TABLE messages (...);

CREATE TRIGGER trg_update_conversation_timestamp ...;
```

### Rollback Strategy
```sql
-- 001_rollback.sql
DROP TRIGGER IF EXISTS trg_update_conversation_timestamp ON messages;
DROP TABLE IF EXISTS messages CASCADE;
DROP TYPE IF EXISTS message_role;
DROP TABLE IF EXISTS conversations CASCADE;
DROP TABLE IF EXISTS user_sessions CASCADE;
```

### Qdrant Setup (Python Script)
```python
# scripts/setup_qdrant.py
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Create collection
client.create_collection(
    collection_name="humanoid_robotics_book",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE, on_disk=True)
)

# Create indexes
client.create_payload_index(
    collection_name="humanoid_robotics_book",
    field_name="chapter_id",
    field_schema="keyword"
)
```

---

## Performance Considerations

### Neon Postgres
- **Free Tier Limits**: 500MB storage, 100 concurrent connections
- **Estimated Capacity**: ~50K conversations (10KB each), ~500K messages (1KB each)
- **Query Optimization**: Indexes on `session_id`, `conv_id`, `updated_at`
- **Connection Pooling**: asyncpg pool with `min_size=2`, `max_size=10`

### Qdrant Cloud
- **Free Tier Limits**: 1GB storage, 10 requests/sec
- **Estimated Capacity**: ~100K chunks (10KB each: 600 tokens + metadata)
- **Query Performance**: <200ms for semantic search (p95)
- **Batch Operations**: Embed and upload chunks in batches of 100

---

**Status**: Data model complete. Proceed to API contracts.
