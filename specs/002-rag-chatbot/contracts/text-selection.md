# API Contract: Text Selection Query

**Endpoint**: `POST /api/chat/text-selection`
**Purpose**: Submit a question about specific highlighted text from a book chapter (context-aware query).
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
interface TextSelectionRequest {
  question: string;              // User's question about the selection (1-2000 characters)
  selection: TextSelection;      // Highlighted text metadata
  conversation_id?: string;      // Optional: UUID of existing conversation
  session_id?: string;           // Optional: UUID of user session
}

interface TextSelection {
  text: string;                  // Highlighted text (1-5000 characters)
  chapter_id: string;            // Chapter file name (e.g., "03-module-1-ros2")
  start_offset: number;          // Character position where selection starts
  end_offset: number;            // Character position where selection ends
  context_before?: string;       // Optional: 50 chars before selection
  context_after?: string;        // Optional: 50 chars after selection
}
```

### Validation Rules

**question**:
- **Required**: Yes
- **Type**: string
- **Min Length**: 1 character
- **Max Length**: 2000 characters
- **Pattern**: No script injection (sanitized on backend)

**selection.text**:
- **Required**: Yes
- **Type**: string
- **Min Length**: 1 character
- **Max Length**: 5000 characters (prevents entire chapter selection)
- **Pattern**: Must match content in specified chapter_id

**selection.chapter_id**:
- **Required**: Yes
- **Type**: string
- **Pattern**: Valid MDX file name from `docs/` directory
- **Example**: `01-introduction`, `03-module-1-ros2`

**selection.start_offset** / **selection.end_offset**:
- **Required**: Yes
- **Type**: integer
- **Constraint**: `0 <= start_offset < end_offset`
- **Constraint**: `end_offset - start_offset <= 5000` (max 5000 chars)

**conversation_id** / **session_id**:
- Same as `/api/chat/query` endpoint

### Example Request
```json
{
  "question": "Can you explain this in simpler terms?",
  "selection": {
    "text": "The Transform (TF) library in ROS 2 provides a standardized way to track coordinate frames over time. Each frame represents a 3D coordinate system with a translation and rotation relative to a parent frame.",
    "chapter_id": "03-module-1-ros2",
    "start_offset": 4567,
    "end_offset": 4823,
    "context_before": "...for robot localization. ",
    "context_after": " This allows for..."
  },
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7"
}
```

---

## Response

### Success Response (200 OK)

#### Body Schema
```typescript
interface TextSelectionResponse {
  message_id: string;            // UUID of assistant message
  conversation_id: string;       // UUID of conversation (new or existing)
  session_id: string;            // UUID of session
  response: string;              // AI-generated explanation (markdown formatted)
  selection_context: SelectionContext;  // Metadata about the highlighted text
  retrieved_chunks: ChunkMetadata[];  // Additional chunks for broader context
  metadata: ResponseMetadata;    // Performance and debugging info
  timestamp: string;             // ISO 8601 timestamp
}

interface SelectionContext {
  chapter_id: string;
  chapter_title: string;
  section_title: string;         // H2 heading containing the selection
  relevance_score: number;       // How relevant selection is to question (0.0-1.0)
}

// ChunkMetadata and ResponseMetadata same as /api/chat/query
```

#### Example Response
```json
{
  "message_id": "9e1a2b3c-4d5e-6f7a-8b9c-0d1e2f3a4b5c",
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "response": "Let me simplify the TF (Transform) library concept:\n\n**In everyday terms**: Imagine you're giving directions. You might say \"the coffee shop is 2 blocks north of the library.\" Here:\n- The **library** is your reference point (parent frame)\n- The **coffee shop** is what you're locating (child frame)\n- **2 blocks north** is the transformation (how to get there)\n\nIn ROS 2, the TF library does the same thing for robots:\n- It tracks where different parts of the robot are relative to each other\n- For example: \"the camera is 30cm forward and 15cm up from the robot's base\"\n\nThis is crucial because sensors (like cameras or LIDAR) see the world from their own perspective, but the robot needs to understand everything relative to its base to make decisions.",
  "selection_context": {
    "chapter_id": "03-module-1-ros2",
    "chapter_title": "Module 1: ROS 2 Fundamentals",
    "section_title": "Coordinate Frames and TF Library",
    "relevance_score": 0.95
  },
  "retrieved_chunks": [
    {
      "chunk_id": "d4e5f6a7-8b9c-0d1e-2f3a-4b5c6d7e8f9a",
      "chapter_id": "03-module-1-ros2",
      "chapter_title": "Module 1: ROS 2 Fundamentals",
      "section_title": "Coordinate Frames and TF Library",
      "similarity_score": 0.92,
      "excerpt": "TF library example: A robot arm has a base_link frame at its mount point, and an end_effector frame at the gripper..."
    }
  ],
  "metadata": {
    "latency_ms": 1890,
    "tokens_used": 742,
    "model": "gpt-4o-mini",
    "qdrant_query_ms": 95,
    "retrieved_count": 3,
    "embedding_model": "text-embedding-3-small"
  },
  "timestamp": "2025-12-10T14:35:22.123Z"
}
```

---

## Error Responses

### 400 Bad Request
**Reason**: Invalid input (selection too long, invalid chapter_id, etc.)

```json
{
  "error": "ValidationError",
  "message": "Selection text exceeds maximum length of 5000 characters",
  "field": "selection.text",
  "timestamp": "2025-12-10T14:35:22.123Z"
}
```

### 404 Not Found
**Reason**: Provided `chapter_id` does not exist in book content

```json
{
  "error": "NotFoundError",
  "message": "Chapter '99-nonexistent-chapter' not found in book",
  "field": "selection.chapter_id",
  "timestamp": "2025-12-10T14:35:22.123Z"
}
```

### 422 Unprocessable Entity
**Reason**: Selection text does not match content in specified chapter (possible tampering)

```json
{
  "error": "ValidationError",
  "message": "Selection text does not match content at specified offset in chapter",
  "field": "selection",
  "timestamp": "2025-12-10T14:35:22.123Z"
}
```

**Other Error Codes**: Same as `/api/chat/query` (401, 429, 500, 503)

---

## Business Logic

### Text Selection RAG Pipeline Flow

1. **Input Validation**: Validate question, selection text, chapter_id, offsets
2. **Session/Conversation Resolution**: Same as `/api/chat/query`
3. **Selection Verification**:
   - Fetch chapter content from `docs/{chapter_id}.mdx`
   - Verify `selection.text` matches content at `start_offset:end_offset`
   - If mismatch → return 422 error (prevents injection attacks)
4. **Contextual Embedding**:
   - Embed **selection text + question** (combined query)
   - This ensures retrieval prioritizes content related to highlighted text
5. **Scoped Vector Search** (Qdrant):
   - Query with **chapter_id filter** (prioritize same chapter)
   - Retrieve top-k=3 chunks (lower than general Q&A since selection provides primary context)
   - Minimum similarity score: 0.65 (lower threshold due to focused query)
6. **Context Assembly**:
   - **Primary Context**: Highlighted text + context_before/after (if provided)
   - **Secondary Context**: Retrieved chunks from same chapter
   - Conversation history (last 5 messages)
   - User profile (if authenticated)
7. **LLM Generation**:
   - System prompt emphasizes **explaining the selection first**
   - Inject selection text prominently in prompt
   - Generate response using `gpt-4o-mini`
8. **Response Storage**:
   - Save user message with `text_selection` JSON in `messages` table
   - Save assistant response
9. **Return Response**: JSON with explanation + selection context

---

## Behavioral Differences from General Q&A

| Aspect | General Q&A (`/api/chat/query`) | Text Selection (`/api/chat/text-selection`) |
|--------|----------------------------------|---------------------------------------------|
| **Primary Context** | Retrieved chunks from entire book | Highlighted text from selection |
| **Retrieval Scope** | Full book (all chapters) | Prioritize same chapter |
| **Top-k Chunks** | 5 chunks | 3 chunks (selection is primary source) |
| **Similarity Threshold** | 0.7 | 0.65 (more lenient for focused queries) |
| **LLM Instruction** | "Answer based on book content" | "Explain the highlighted text first, then expand" |
| **Storage** | No `text_selection` field | Stores `text_selection` JSON in message |

---

## Performance Requirements

- **Latency**: p95 < 2.5 seconds (faster than general Q&A due to scoped search)
- **Throughput**: 50+ concurrent requests (per SC-006)
- **Selection Verification**: <100ms (local file read + string match)
- **Timeout**: 10 seconds max (Vercel serverless limit)

---

## Security Considerations

- **Selection Verification**: Critical security feature
  - Prevents users from injecting arbitrary text disguised as book content
  - Verifies `selection.text` matches `docs/{chapter_id}.mdx` at specified offset
  - If mismatch → reject request (possible tampering/attack)
- **Rate Limiting**: Same as `/api/chat/query` (10 requests/minute per session)
- **Chapter_id Validation**: Whitelist only valid MDX files from `docs/` directory
- **Input Sanitization**: Strip HTML/JS from both question and selection text

---

## Testing Scenarios

### Happy Path
1. **User highlights paragraph, asks for explanation**
   - Request: Valid selection from Module 1, question: "Explain this"
   - Expected: 200 OK, response focuses on explaining highlighted text

2. **User highlights code snippet, asks how it works**
   - Request: Code block selection, question: "What does this code do?"
   - Expected: 200 OK, step-by-step code explanation

### Edge Cases
3. **Selection text doesn't match chapter content (tampering)**
   - Request: `selection.text` modified client-side
   - Expected: 422 Unprocessable Entity, validation error

4. **Very long selection (5000+ chars)**
   - Request: User selects entire chapter
   - Expected: 400 Bad Request, exceeds max length

5. **Invalid chapter_id**
   - Request: `chapter_id: "99-fake-chapter"`
   - Expected: 404 Not Found, chapter not found

6. **Selection offsets out of bounds**
   - Request: `start_offset: 999999` (exceeds chapter length)
   - Expected: 422 Unprocessable Entity, invalid offset

7. **Selection spans multiple chapters (JavaScript error)**
   - Expected: Frontend prevents this, but backend validates chapter_id
   - Fallback: 400 Bad Request if frontend validation bypassed

---

## Frontend Integration Notes

### Text Selection Capture (Browser API)
```javascript
// Frontend implementation (React/Docusaurus)
document.addEventListener('mouseup', () => {
  const selection = window.getSelection();
  if (selection.rangeCount > 0 && selection.toString().length > 0) {
    const range = selection.getRangeAt(0);
    const selectedText = selection.toString();

    // Get chapter_id from current page URL
    const chapterId = getCurrentChapterIdFromURL();

    // Calculate offsets relative to chapter content
    const startOffset = calculateAbsoluteOffset(range.startContainer, range.startOffset);
    const endOffset = calculateAbsoluteOffset(range.endContainer, range.endOffset);

    // Show "Ask about this selection" button
    showSelectionButton({
      text: selectedText,
      chapter_id: chapterId,
      start_offset: startOffset,
      end_offset: endOffset
    });
  }
});
```

### Context Capture (Optional Enhancement)
```javascript
// Extract 50 chars before/after selection for better context
function getSelectionWithContext(selection) {
  const range = selection.getRangeAt(0);
  const fullText = range.startContainer.textContent;
  const start = range.startOffset;
  const end = range.endOffset;

  return {
    text: selection.toString(),
    context_before: fullText.substring(Math.max(0, start - 50), start),
    context_after: fullText.substring(end, Math.min(fullText.length, end + 50))
  };
}
```

---

**Status**: Contract complete. See `chat-query.md` for general Q&A and `conversations.md` for history retrieval.
