# API Contract: Chat Query

**Endpoint**: `POST /api/chat/query`
**Purpose**: Submit a general question about book content and receive an AI-generated response based on RAG retrieval.
**Authentication**: Optional (Bearer token for authenticated users)

---

## Request

### Headers
```http
Content-Type: application/json
Authorization: Bearer <session_token>  # Optional: for authenticated users
```

### Body Schema
```typescript
interface ChatQueryRequest {
  question: string;              // User's question (1-2000 characters)
  conversation_id?: string;      // Optional: UUID of existing conversation
  session_id?: string;           // Optional: UUID of user session (created if not provided)
}
```

### Validation Rules
- `question`:
  - **Required**: Yes
  - **Type**: string
  - **Min Length**: 1 character
  - **Max Length**: 2000 characters
  - **Pattern**: No script injection (sanitized on backend)

- `conversation_id`:
  - **Required**: No
  - **Type**: UUID string
  - **Validation**: Must exist in database if provided
  - **Behavior**: If omitted, creates new conversation

- `session_id`:
  - **Required**: No
  - **Type**: UUID string
  - **Validation**: Must exist in database if provided
  - **Behavior**: If omitted, creates new anonymous session

### Example Request
```json
{
  "question": "What are the main components of ROS 2?",
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7"
}
```

---

## Response

### Success Response (200 OK)

#### Body Schema
```typescript
interface ChatQueryResponse {
  message_id: string;            // UUID of assistant message
  conversation_id: string;       // UUID of conversation (new or existing)
  session_id: string;            // UUID of session
  response: string;              // AI-generated answer (markdown formatted)
  retrieved_chunks: ChunkMetadata[];  // Top-k chunks used for context
  metadata: ResponseMetadata;    // Performance and debugging info
  timestamp: string;             // ISO 8601 timestamp
}

interface ChunkMetadata {
  chunk_id: string;
  chapter_id: string;
  chapter_title: string;
  section_title: string;
  similarity_score: number;      // 0.0-1.0 cosine similarity
  excerpt: string;               // First 200 chars of chunk
}

interface ResponseMetadata {
  latency_ms: number;            // Total response time
  tokens_used: number;           // Prompt + completion tokens
  model: string;                 // OpenAI model used
  qdrant_query_ms: number;       // Vector search latency
  retrieved_count: number;       // Number of chunks retrieved
  embedding_model: string;       // Embedding model used
}
```

#### Example Response
```json
{
  "message_id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "response": "ROS 2 has three main components:\n\n1. **Nodes**: Independent processes that perform computation\n2. **Topics**: Named buses for asynchronous message passing\n3. **Services**: Synchronous request-reply communication\n\nFor your GPU workstation setup, you can run these examples locally in Isaac Sim...",
  "retrieved_chunks": [
    {
      "chunk_id": "c1d2e3f4-5678-90ab-cdef-1234567890ab",
      "chapter_id": "03-module-1-ros2",
      "chapter_title": "Module 1: ROS 2 Fundamentals",
      "section_title": "Core Concepts",
      "similarity_score": 0.89,
      "excerpt": "ROS 2 (Robot Operating System 2) is built on three foundational components: nodes, topics, and services..."
    }
  ],
  "metadata": {
    "latency_ms": 2340,
    "tokens_used": 856,
    "model": "gpt-4o-mini",
    "qdrant_query_ms": 120,
    "retrieved_count": 5,
    "embedding_model": "text-embedding-3-small"
  },
  "timestamp": "2025-12-10T14:23:45.678Z"
}
```

---

## Error Responses

### 400 Bad Request
**Reason**: Invalid input (empty question, invalid UUID format, etc.)

```json
{
  "error": "ValidationError",
  "message": "Question must be between 1 and 2000 characters",
  "field": "question",
  "timestamp": "2025-12-10T14:23:45.678Z"
}
```

### 401 Unauthorized
**Reason**: Invalid or expired authentication token (for protected features)

```json
{
  "error": "AuthenticationError",
  "message": "Invalid or expired session token",
  "timestamp": "2025-12-10T14:23:45.678Z"
}
```

### 404 Not Found
**Reason**: Provided `conversation_id` or `session_id` does not exist

```json
{
  "error": "NotFoundError",
  "message": "Conversation ID '550e8400-e29b-41d4-a716-446655440000' not found",
  "field": "conversation_id",
  "timestamp": "2025-12-10T14:23:45.678Z"
}
```

### 429 Too Many Requests
**Reason**: Rate limit exceeded (10 requests/minute per session)

```json
{
  "error": "RateLimitError",
  "message": "Rate limit exceeded. Please wait 30 seconds before retrying",
  "retry_after": 30,
  "timestamp": "2025-12-10T14:23:45.678Z"
}
```

### 500 Internal Server Error
**Reason**: OpenAI API failure, Qdrant unavailable, or database error

```json
{
  "error": "InternalServerError",
  "message": "Failed to generate response. Please try again later",
  "details": "OpenAI API timeout",  // Only in development mode
  "timestamp": "2025-12-10T14:23:45.678Z"
}
```

### 503 Service Unavailable
**Reason**: System under maintenance or dependent service (Qdrant, Neon) unavailable

```json
{
  "error": "ServiceUnavailableError",
  "message": "Chatbot is temporarily unavailable. Please try again in a few minutes",
  "retry_after": 120,
  "timestamp": "2025-12-10T14:23:45.678Z"
}
```

---

## Business Logic

### RAG Pipeline Flow
1. **Input Validation**: Sanitize question, validate UUIDs
2. **Session Resolution**:
   - If `session_id` provided → fetch existing session
   - If authenticated (Bearer token) → fetch user profile from Better-Auth
   - Else → create anonymous session
3. **Conversation Resolution**:
   - If `conversation_id` provided → fetch conversation + last 5 messages
   - Else → create new conversation
4. **Embedding Generation**: Embed question using OpenAI `text-embedding-3-small`
5. **Vector Search** (Qdrant):
   - Query `humanoid_robotics_book` collection
   - Apply hardware profile filter (if user authenticated)
   - Retrieve top-k=5 chunks with similarity score > 0.7
6. **Context Assembly**:
   - Conversation history (last 5 messages)
   - Retrieved chunks (concatenated, max 2000 tokens)
   - User profile (hardware type, if authenticated)
7. **LLM Generation**:
   - System prompt with profile context
   - User question + retrieved context
   - Generate response using `gpt-4o-mini` (or `gpt-4o` for premium users)
8. **Response Storage**:
   - Save user message to `messages` table (role='user')
   - Save assistant response to `messages` table (role='assistant')
   - Update conversation `updated_at` timestamp
9. **Return Response**: JSON response with answer + metadata

---

## Performance Requirements

- **Latency**: p95 < 3 seconds (per SC-002)
- **Throughput**: 50+ concurrent requests (per SC-006)
- **Timeout**: 10 seconds max (Vercel serverless limit)
- **Retry Logic**: 3 retries with exponential backoff for OpenAI API failures

---

## Security Considerations

- **Input Sanitization**: Strip HTML/JS from question to prevent XSS
- **Rate Limiting**: 10 requests/minute per session_id (per-IP fallback for anonymous)
- **CORS**: Whitelist frontend domain (Vercel deployment URL)
- **Secrets**: Never expose OpenAI API keys, Qdrant API keys in responses
- **Error Messages**: Generic errors in production, detailed only in development

---

## Testing Scenarios

### Happy Path
1. **Anonymous user, new conversation**
   - Request: `{"question": "What is ROS 2?"}`
   - Expected: 200 OK, new `session_id` and `conversation_id` created

2. **Authenticated user, existing conversation**
   - Request: `{"question": "How does this work on Jetson?", "conversation_id": "...", "session_id": "..."}`
   - Expected: 200 OK, response includes Edge-specific guidance

### Edge Cases
3. **Empty question**
   - Request: `{"question": ""}`
   - Expected: 400 Bad Request, validation error

4. **Very long question (2001 chars)**
   - Expected: 400 Bad Request, exceeds max length

5. **Invalid conversation_id**
   - Request: `{"question": "...", "conversation_id": "invalid-uuid"}`
   - Expected: 404 Not Found

6. **Qdrant returns 0 chunks (no relevant content)**
   - Expected: 200 OK, response indicates "I don't have information about that topic"

7. **OpenAI API timeout**
   - Expected: 500 Internal Server Error, retry with exponential backoff

---

**Status**: Contract complete. See `text-selection.md` for text selection queries and `conversations.md` for history retrieval.
