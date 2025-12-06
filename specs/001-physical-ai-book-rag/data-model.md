# Data Model

**Feature**: Physical AI & Humanoid Robotics Capstone Book & RAG Platform
**Date**: 2025-12-05
**Phase**: Phase 1 - Database Schema Design

This document defines the complete database schema for Neon Serverless Postgres and Qdrant vector database, including entities, relationships, constraints, and state transitions.

---

## Database Overview

### Relational Database (Neon Serverless Postgres)
- **Purpose**: Store user profiles, authentication sessions, chat history, and feedback
- **Schema**: `public` (default PostgreSQL schema)
- **Connection**: Managed via SQLAlchemy ORM in FastAPI backend

### Vector Database (Qdrant Cloud)
- **Purpose**: Store content embeddings for semantic search in RAG system
- **Collections**: `content_embeddings_en` (English), `content_embeddings_ur` (Urdu)
- **Connection**: Managed via Qdrant Python client

---

## Neon Postgres Schema

### Entity: User

**Purpose**: Represents a learner registered on the platform.

**Table Name**: `users`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `UUID` | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique user identifier |
| `email` | `VARCHAR(255)` | UNIQUE, NOT NULL | User's email address (used for authentication) |
| `password_hash` | `VARCHAR(255)` | NOT NULL | Bcrypt-hashed password (Better-Auth handles hashing) |
| `created_at` | `TIMESTAMP` | DEFAULT NOW(), NOT NULL | Account creation timestamp |
| `updated_at` | `TIMESTAMP` | DEFAULT NOW(), NOT NULL | Last profile update timestamp |
| `is_active` | `BOOLEAN` | DEFAULT TRUE, NOT NULL | Account status (soft delete capability) |

**Indexes**:
- `idx_users_email` on `email` (for fast login lookups)

**Validation Rules**:
- Email must be valid format (validated by Better-Auth before insertion)
- Password must be hashed before storage (never store plaintext)

**Relationships**:
- One-to-One with `hardware_profiles`
- One-to-Many with `chat_messages`

---

### Entity: HardwareProfile

**Purpose**: Stores user's hardware and software environment for personalized content delivery.

**Table Name**: `hardware_profiles`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `UUID` | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique profile identifier |
| `user_id` | `UUID` | FOREIGN KEY REFERENCES users(id) ON DELETE CASCADE, UNIQUE, NOT NULL | Associated user (one profile per user) |
| `os_type` | `VARCHAR(50)` | NOT NULL | Operating system (e.g., "Ubuntu 22.04", "macOS", "Windows") |
| `gpu_model` | `VARCHAR(100)` | NULLABLE | GPU model (e.g., "RTX 4070 Ti", "RTX 4090", "Integrated Graphics", NULL for none) |
| `environment_preference` | `ENUM('workstation', 'cloud', 'mac')` | DEFAULT 'cloud', NOT NULL | User's preferred content variant |
| `language_preference` | `ENUM('en', 'ur')` | DEFAULT 'en', NOT NULL | Preferred language for content |
| `created_at` | `TIMESTAMP` | DEFAULT NOW(), NOT NULL | Profile creation timestamp |
| `updated_at` | `TIMESTAMP` | DEFAULT NOW(), NOT NULL | Last profile update timestamp |

**Indexes**:
- `idx_hardware_profiles_user_id` on `user_id` (for fast profile lookups)

**Validation Rules**:
- `os_type`: Free text (user self-reported)
- `gpu_model`: Free text or NULL (user self-reported; backend validates for Workstation capability)
- `environment_preference`: Must be one of ('workstation', 'cloud', 'mac')
- `language_preference`: Must be one of ('en', 'ur')

**State Transitions**:
- **Initial State**: On signup, profile created with user-provided `os_type` and `gpu_model`
- **Auto-Selection**: If GPU model contains "RTX 4070 Ti", "RTX 4080", "RTX 4090", or "RTX 50", set `environment_preference = 'workstation'`; otherwise `'cloud'`
- **User Override**: User can manually change `environment_preference` via frontend toggle (persisted on next request)

**Relationships**:
- One-to-One with `users` (cascade delete: if user deleted, profile deleted)

---

### Entity: ChatMessage

**Purpose**: Represents a single query-response pair in the RAG chatbot interaction.

**Table Name**: `chat_messages`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `UUID` | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique message identifier |
| `user_id` | `UUID` | FOREIGN KEY REFERENCES users(id) ON DELETE CASCADE, NULLABLE | Associated user (NULL for anonymous users) |
| `session_id` | `VARCHAR(255)` | NOT NULL | Session identifier (groups messages in a conversation) |
| `query` | `TEXT` | NOT NULL | User's question or prompt |
| `response` | `TEXT` | NOT NULL | Chatbot's generated answer |
| `selected_text` | `TEXT` | NULLABLE | User-selected text for contextual queries (NULL for general queries) |
| `language` | `ENUM('en', 'ur')` | NOT NULL | Language of the query/response |
| `retrieved_chunks` | `JSONB` | NULLABLE | Metadata of retrieved chunks (for debugging/citation) |
| `created_at` | `TIMESTAMP` | DEFAULT NOW(), NOT NULL | Message creation timestamp |

**Indexes**:
- `idx_chat_messages_user_id` on `user_id` (for user chat history)
- `idx_chat_messages_session_id` on `session_id` (for session-based queries)
- `idx_chat_messages_created_at` on `created_at` (for time-based queries)

**Validation Rules**:
- `query`: Must be non-empty (validated in API layer)
- `response`: Must be non-empty
- `selected_text`: If present, must be ≥50 characters (per spec requirement FR-012)
- `language`: Must be one of ('en', 'ur')
- `retrieved_chunks`: JSON array of `{chunk_id, module, chapter, relevance_score}`

**Relationships**:
- Many-to-One with `users` (optional: anonymous users have `user_id = NULL`)
- One-to-Many with `feedback` (one message can have one feedback record)

---

### Entity: Feedback

**Purpose**: Stores user feedback (thumbs up/down with optional text) for chatbot responses.

**Table Name**: `feedback`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `UUID` | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique feedback identifier |
| `message_id` | `UUID` | FOREIGN KEY REFERENCES chat_messages(id) ON DELETE CASCADE, UNIQUE, NOT NULL | Associated chat message (one feedback per message) |
| `rating` | `ENUM('thumbs_up', 'thumbs_down')` | NOT NULL | User's rating |
| `optional_text` | `TEXT` | NULLABLE | User's elaboration (required for thumbs_down in UI, but NULLABLE in DB) |
| `created_at` | `TIMESTAMP` | DEFAULT NOW(), NOT NULL | Feedback submission timestamp |

**Indexes**:
- `idx_feedback_message_id` on `message_id` (for message-specific feedback lookups)
- `idx_feedback_rating` on `rating` (for aggregated analytics)

**Validation Rules**:
- `rating`: Must be one of ('thumbs_up', 'thumbs_down')
- `optional_text`: No length restriction (validated in API layer if needed)

**Relationships**:
- One-to-One with `chat_messages` (cascade delete: if message deleted, feedback deleted)

---

## Qdrant Vector Database Schema

### Collection: content_embeddings_en

**Purpose**: Store English content embeddings for semantic search.

**Configuration**:
```python
{
    "vector_size": 1536,  # OpenAI text-embedding-3-small
    "distance": "Cosine",
    "on_disk_payload": True,  # Free tier optimization
    "hnsw_config": {
        "m": 16,
        "ef_construct": 100
    }
}
```

**Payload Schema** (metadata stored with each vector):

| Field | Type | Description |
|-------|------|-------------|
| `chunk_id` | `string` | Unique identifier for this chunk (e.g., "module1_ch2_p3") |
| `module` | `string` | Module name (e.g., "module-1-ros2") |
| `chapter` | `string` | Chapter name (e.g., "nodes") |
| `section_heading` | `string` | Section heading within chapter (e.g., "Creating a Node") |
| `content` | `string` | Full text of the chunk (for citation and display) |
| `page_url` | `string` | URL path to the source page (e.g., "/docs/module-1-ros2/nodes") |
| `token_count` | `integer` | Number of tokens in chunk (for context window management) |
| `language` | `string` | Always "en" for this collection |

**Indexing Process**:
1. Parse Markdown files from `frontend/docs/`
2. Chunk content using semantic splitter (~500 tokens, 50-token overlap)
3. Generate embeddings via OpenAI API
4. Upsert vectors with payload to Qdrant

---

### Collection: content_embeddings_ur

**Purpose**: Store Urdu content embeddings for semantic search.

**Configuration**: Same as `content_embeddings_en`

**Payload Schema**: Same as `content_embeddings_en`, except `language` field is always "ur"

**Indexing Process**:
1. Parse Urdu Markdown files from `frontend/i18n/ur/docusaurus-plugin-content-docs/current/`
2. Apply same chunking strategy
3. Generate embeddings (OpenAI text-embedding-3-small supports Urdu)
4. Upsert vectors with payload to Qdrant

---

## Entity Relationship Diagram (ERD)

```
┌─────────────┐
│   users     │
├─────────────┤
│ id (PK)     │
│ email       │
│ password_   │
│  hash       │
│ created_at  │
│ updated_at  │
│ is_active   │
└──────┬──────┘
       │ 1:1
       │
┌──────▼───────────────┐
│ hardware_profiles    │
├──────────────────────┤
│ id (PK)              │
│ user_id (FK, UNIQUE) │
│ os_type              │
│ gpu_model            │
│ environment_         │
│  preference          │
│ language_preference  │
│ created_at           │
│ updated_at           │
└──────────────────────┘

┌─────────────┐
│   users     │
└──────┬──────┘
       │ 1:N
       │
┌──────▼────────────┐
│  chat_messages    │
├───────────────────┤
│ id (PK)           │
│ user_id (FK, NULL)│◄───── Anonymous users allowed
│ session_id        │
│ query             │
│ response          │
│ selected_text     │
│ language          │
│ retrieved_chunks  │
│ created_at        │
└──────┬────────────┘
       │ 1:1
       │
┌──────▼──────────┐
│   feedback      │
├─────────────────┤
│ id (PK)         │
│ message_id (FK, │
│   UNIQUE)       │
│ rating          │
│ optional_text   │
│ created_at      │
└─────────────────┘

[Qdrant Collections]
┌────────────────────────┐     ┌────────────────────────┐
│ content_embeddings_en  │     │ content_embeddings_ur  │
├────────────────────────┤     ├────────────────────────┤
│ Vector (1536 dims)     │     │ Vector (1536 dims)     │
│ Payload:               │     │ Payload:               │
│  - chunk_id            │     │  - chunk_id            │
│  - module              │     │  - module              │
│  - chapter             │     │  - chapter             │
│  - section_heading     │     │  - section_heading     │
│  - content             │     │  - content             │
│  - page_url            │     │  - page_url            │
│  - token_count         │     │  - token_count         │
│  - language ("en")     │     │  - language ("ur")     │
└────────────────────────┘     └────────────────────────┘
```

---

## State Transitions

### User Registration Flow
1. **Initial State**: No user record exists
2. **Signup**: User submits email, password, OS, GPU model → `users` record created with `is_active = TRUE`
3. **Profile Creation**: `hardware_profiles` record created with `environment_preference` auto-selected based on GPU model
4. **Active State**: User can authenticate and access personalized features

### Chat Interaction Flow
1. **Query Submitted**: User sends query → `chat_messages` record created with `user_id` (or NULL if anonymous), `query`, `language`
2. **Response Generated**: RAG system generates response → `response` and `retrieved_chunks` fields populated
3. **Feedback Submitted (Optional)**: User rates response → `feedback` record created with `message_id` FK and `rating`

### Environment Preference Update Flow
1. **Initial Preference**: Set during signup based on GPU model
2. **User Toggle**: User clicks "Personalize" button and selects different environment → `hardware_profiles.environment_preference` updated via API
3. **Persistence**: Preference persists across sessions (stored in Neon DB, loaded on page load)

---

## Migration Scripts

### Initial Schema (Neon Postgres)

```sql
-- Create ENUM types
CREATE TYPE environment_preference AS ENUM ('workstation', 'cloud', 'mac');
CREATE TYPE language_preference AS ENUM ('en', 'ur');
CREATE TYPE feedback_rating AS ENUM ('thumbs_up', 'thumbs_down');

-- Users table
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT NOW() NOT NULL,
    updated_at TIMESTAMP DEFAULT NOW() NOT NULL,
    is_active BOOLEAN DEFAULT TRUE NOT NULL
);

CREATE INDEX idx_users_email ON users(email);

-- Hardware profiles table
CREATE TABLE hardware_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID UNIQUE NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    os_type VARCHAR(50) NOT NULL,
    gpu_model VARCHAR(100),
    environment_preference environment_preference DEFAULT 'cloud' NOT NULL,
    language_preference language_preference DEFAULT 'en' NOT NULL,
    created_at TIMESTAMP DEFAULT NOW() NOT NULL,
    updated_at TIMESTAMP DEFAULT NOW() NOT NULL
);

CREATE INDEX idx_hardware_profiles_user_id ON hardware_profiles(user_id);

-- Chat messages table
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    session_id VARCHAR(255) NOT NULL,
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    selected_text TEXT,
    language language_preference NOT NULL,
    retrieved_chunks JSONB,
    created_at TIMESTAMP DEFAULT NOW() NOT NULL
);

CREATE INDEX idx_chat_messages_user_id ON chat_messages(user_id);
CREATE INDEX idx_chat_messages_session_id ON chat_messages(session_id);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at);

-- Feedback table
CREATE TABLE feedback (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID UNIQUE NOT NULL REFERENCES chat_messages(id) ON DELETE CASCADE,
    rating feedback_rating NOT NULL,
    optional_text TEXT,
    created_at TIMESTAMP DEFAULT NOW() NOT NULL
);

CREATE INDEX idx_feedback_message_id ON feedback(message_id);
CREATE INDEX idx_feedback_rating ON feedback(rating);
```

### Qdrant Collection Creation (Python)

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, OptimizersConfigDiff

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Create English collection
client.create_collection(
    collection_name="content_embeddings_en",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
    optimizers_config=OptimizersConfigDiff(default_segment_number=2),
    on_disk_payload=True  # Free tier optimization
)

# Create Urdu collection
client.create_collection(
    collection_name="content_embeddings_ur",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
    optimizers_config=OptimizersConfigDiff(default_segment_number=2),
    on_disk_payload=True
)
```

---

## Validation & Constraints Summary

| Entity | Constraint | Enforcement |
|--------|------------|-------------|
| User | Email uniqueness | Database UNIQUE constraint |
| User | Password strength | Better-Auth validation (client-side) |
| HardwareProfile | One per user | Database UNIQUE constraint on user_id |
| HardwareProfile | Valid environment preference | Database ENUM constraint |
| ChatMessage | Selected text ≥50 chars | API layer validation (FastAPI Pydantic schema) |
| ChatMessage | Valid language | Database ENUM constraint |
| Feedback | One per message | Database UNIQUE constraint on message_id |
| Feedback | Valid rating | Database ENUM constraint |

---

**Next Artifact**: contracts/ (API OpenAPI specifications)
