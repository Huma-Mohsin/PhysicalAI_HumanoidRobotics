# API Contract: Conversations History

**Endpoint**: `GET /api/chat/conversations`
**Purpose**: Retrieve conversation history for a user session (authenticated users only).
**Authentication**: Required (Bearer token)

---

## Request

### Headers
```http
Authorization: Bearer <session_token>  # Required
```

### Query Parameters
```typescript
interface ConversationsQueryParams {
  session_id?: string;           // Optional: UUID of user session (defaults to token's session)
  limit?: number;                // Optional: Max conversations to return (default: 20, max: 100)
  offset?: number;               // Optional: Pagination offset (default: 0)
  include_messages?: boolean;    // Optional: Include messages in response (default: false)
}
```

### Validation Rules

**session_id**:
- **Required**: No (defaults to session from Bearer token)
- **Type**: UUID string
- **Validation**: Must belong to authenticated user (prevents unauthorized access)

**limit**:
- **Required**: No
- **Type**: integer
- **Default**: 20
- **Min**: 1
- **Max**: 100

**offset**:
- **Required**: No
- **Type**: integer
- **Default**: 0
- **Min**: 0

**include_messages**:
- **Required**: No
- **Type**: boolean
- **Default**: false
- **Behavior**: If `true`, returns last 5 messages per conversation

### Example Request
```http
GET /api/chat/conversations?limit=10&offset=0&include_messages=true
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

## Response

### Success Response (200 OK)

#### Body Schema (without messages)
```typescript
interface ConversationsResponse {
  conversations: ConversationSummary[];
  pagination: PaginationMetadata;
  timestamp: string;             // ISO 8601 timestamp
}

interface ConversationSummary {
  conv_id: string;               // UUID
  session_id: string;            // UUID
  title: string;                 // Auto-generated from first question
  message_count: number;         // Total messages in conversation
  last_message_preview: string;  // First 100 chars of last assistant message
  created_at: string;            // ISO 8601 timestamp
  updated_at: string;            // ISO 8601 timestamp
}

interface PaginationMetadata {
  total_count: number;           // Total conversations for session
  limit: number;                 // Requested limit
  offset: number;                // Requested offset
  has_more: boolean;             // true if more conversations exist
}
```

#### Example Response (without messages)
```json
{
  "conversations": [
    {
      "conv_id": "550e8400-e29b-41d4-a716-446655440000",
      "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
      "title": "ROS 2 Components and Setup",
      "message_count": 8,
      "last_message_preview": "Yes, you can install ROS 2 Humble on Ubuntu 22.04 using the official APT repository. Here's how...",
      "created_at": "2025-12-09T10:15:30.000Z",
      "updated_at": "2025-12-09T10:42:18.000Z"
    },
    {
      "conv_id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
      "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
      "title": "Isaac Sim Installation on RTX 4090",
      "message_count": 12,
      "last_message_preview": "For RTX 4090, you have two options: (1) Install Isaac Sim locally using Omniverse Launcher, or (2) Use...",
      "created_at": "2025-12-08T14:22:15.000Z",
      "updated_at": "2025-12-08T15:10:45.000Z"
    }
  ],
  "pagination": {
    "total_count": 15,
    "limit": 10,
    "offset": 0,
    "has_more": true
  },
  "timestamp": "2025-12-10T16:20:33.456Z"
}
```

#### Body Schema (with messages)
```typescript
interface ConversationsResponseWithMessages {
  conversations: ConversationDetail[];
  pagination: PaginationMetadata;
  timestamp: string;
}

interface ConversationDetail extends ConversationSummary {
  messages: Message[];           // Last 5 messages (ordered by created_at DESC)
  summary?: string;              // Conversation summary (if > 20 messages)
}

interface Message {
  message_id: string;            // UUID
  role: "user" | "assistant";
  content: string;               // Full message content
  text_selection?: TextSelection;  // If message was a text selection query
  created_at: string;            // ISO 8601 timestamp
}

interface TextSelection {
  text: string;
  chapter_id: string;
  chapter_title: string;
}
```

#### Example Response (with messages)
```json
{
  "conversations": [
    {
      "conv_id": "550e8400-e29b-41d4-a716-446655440000",
      "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
      "title": "ROS 2 Components and Setup",
      "message_count": 8,
      "last_message_preview": "Yes, you can install ROS 2 Humble on Ubuntu 22.04...",
      "created_at": "2025-12-09T10:15:30.000Z",
      "updated_at": "2025-12-09T10:42:18.000Z",
      "messages": [
        {
          "message_id": "9e1a2b3c-4d5e-6f7a-8b9c-0d1e2f3a4b5c",
          "role": "assistant",
          "content": "Yes, you can install ROS 2 Humble on Ubuntu 22.04 using the official APT repository. Here's how:\n\n1. Set up sources...",
          "created_at": "2025-12-09T10:42:18.000Z"
        },
        {
          "message_id": "8d9c0b1a-2e3f-4a5b-6c7d-8e9f0a1b2c3d",
          "role": "user",
          "content": "Can I install ROS 2 Humble on my machine?",
          "created_at": "2025-12-09T10:42:10.000Z"
        },
        {
          "message_id": "7c8b9a0d-1e2f-3a4b-5c6d-7e8f9a0b1c2d",
          "role": "assistant",
          "content": "ROS 2 has three main components:\n\n1. **Nodes**: Independent processes...",
          "created_at": "2025-12-09T10:20:45.000Z"
        },
        {
          "message_id": "6b7c8a9d-0e1f-2a3b-4c5d-6e7f8a9b0c1d",
          "role": "user",
          "content": "What are the main components of ROS 2?",
          "created_at": "2025-12-09T10:20:30.000Z"
        }
      ]
    }
  ],
  "pagination": {
    "total_count": 15,
    "limit": 10,
    "offset": 0,
    "has_more": true
  },
  "timestamp": "2025-12-10T16:20:33.456Z"
}
```

---

## Error Responses

### 401 Unauthorized
**Reason**: Missing or invalid Bearer token

```json
{
  "error": "AuthenticationError",
  "message": "Authentication required. Please log in to view conversation history",
  "timestamp": "2025-12-10T16:20:33.456Z"
}
```

### 403 Forbidden
**Reason**: Authenticated user trying to access another user's sessions

```json
{
  "error": "AuthorizationError",
  "message": "You do not have permission to access this session's conversations",
  "field": "session_id",
  "timestamp": "2025-12-10T16:20:33.456Z"
}
```

### 400 Bad Request
**Reason**: Invalid query parameters (limit > 100, negative offset, etc.)

```json
{
  "error": "ValidationError",
  "message": "Limit must be between 1 and 100",
  "field": "limit",
  "timestamp": "2025-12-10T16:20:33.456Z"
}
```

### 404 Not Found
**Reason**: Provided `session_id` does not exist

```json
{
  "error": "NotFoundError",
  "message": "Session ID '7c9e6679-7425-40de-944b-e07fc1f90ae7' not found",
  "field": "session_id",
  "timestamp": "2025-12-10T16:20:33.456Z"
}
```

### 500 Internal Server Error
**Reason**: Database query failure

```json
{
  "error": "InternalServerError",
  "message": "Failed to retrieve conversations. Please try again later",
  "timestamp": "2025-12-10T16:20:33.456Z"
}
```

---

## Business Logic

### Conversation Retrieval Flow

1. **Authentication**: Verify Bearer token, extract `user_id`
2. **Session Resolution**:
   - If `session_id` provided → verify it belongs to authenticated user
   - Else → fetch default session for authenticated user
3. **Query Conversations**:
   - Fetch conversations for session, ordered by `updated_at DESC`
   - Apply `limit` and `offset` for pagination
   - Count total conversations for pagination metadata
4. **Include Messages** (if `include_messages=true`):
   - For each conversation, fetch last 5 messages ordered by `created_at DESC`
   - If conversation has `summary`, include it in response
5. **Return Response**: JSON with conversations + pagination metadata

---

## Behavioral Considerations

### Anonymous Users
- **Not Supported**: This endpoint requires authentication
- **Reason**: Anonymous users don't have persistent conversation history (session-only state)
- **Alternative**: Anonymous users can continue existing conversation via `conversation_id` in `/api/chat/query`, but cannot retrieve past conversations

### Pagination Best Practices
- **Default Limit**: 20 conversations (balance between UX and performance)
- **Max Limit**: 100 (prevents excessive database load)
- **Ordering**: Always `updated_at DESC` (most recent first)
- **Performance**: Index on `(session_id, updated_at DESC)` for fast retrieval

### Message Inclusion Trade-offs
- **Without Messages** (`include_messages=false`):
  - **Pros**: Lightweight response, fast query (~50ms)
  - **Use Case**: Show conversation list in sidebar
- **With Messages** (`include_messages=true`):
  - **Pros**: Full context, no additional requests needed
  - **Cons**: Larger payload (~10x size), slower query (~200ms)
  - **Use Case**: Resume conversation modal, show recent exchanges

---

## Performance Requirements

- **Latency**: p95 < 500ms (without messages), < 1s (with messages)
- **Throughput**: 100+ requests/sec (read-heavy workload)
- **Caching**: Consider Redis cache for frequently accessed sessions (future optimization)

---

## Security Considerations

- **Authorization**: Users can only access their own sessions (enforce `user_id` check)
- **Rate Limiting**: 60 requests/minute per user (prevent abuse)
- **Data Privacy**: Never expose other users' conversation data
- **Token Validation**: Verify Bearer token on every request (no caching)

---

## Testing Scenarios

### Happy Path
1. **Authenticated user requests conversation list**
   - Request: `GET /api/chat/conversations`
   - Expected: 200 OK, list of conversations for user's session

2. **Paginated request**
   - Request: `GET /api/chat/conversations?limit=5&offset=10`
   - Expected: 200 OK, conversations 11-15

3. **Request with messages**
   - Request: `GET /api/chat/conversations?include_messages=true`
   - Expected: 200 OK, conversations with last 5 messages each

### Edge Cases
4. **Unauthenticated request**
   - Request: No Bearer token
   - Expected: 401 Unauthorized

5. **User tries to access another user's session**
   - Request: `GET /api/chat/conversations?session_id=<other-user-session>`
   - Expected: 403 Forbidden

6. **Invalid limit (101)**
   - Request: `GET /api/chat/conversations?limit=101`
   - Expected: 400 Bad Request, validation error

7. **Session with 0 conversations (new user)**
   - Request: `GET /api/chat/conversations`
   - Expected: 200 OK, empty array, `total_count: 0`

8. **Conversation with >20 messages (summarized)**
   - Expected: Response includes `summary` field with compressed history

---

## Additional Endpoints (Future Enhancements)

### Delete Conversation
**Endpoint**: `DELETE /api/chat/conversations/:conv_id`
**Purpose**: Allow users to delete individual conversations

### Rename Conversation
**Endpoint**: `PATCH /api/chat/conversations/:conv_id`
**Purpose**: Allow users to customize conversation titles

### Export Conversation
**Endpoint**: `GET /api/chat/conversations/:conv_id/export`
**Purpose**: Export conversation as markdown or JSON

---

**Status**: Contract complete. See `chat-query.md` and `text-selection.md` for query endpoints.
