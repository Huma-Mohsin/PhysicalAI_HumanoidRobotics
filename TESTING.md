# Testing Guide: Physical AI RAG Platform

This guide covers testing for Phase 4 (RAG Chatbot) functionality.

## ✅ Phase 4 Testing Checklist

### Backend RAG Services (T055)

#### Test 1: OpenAI Service - Embedding Generation

```bash
# From backend/ directory
python -c "
from src.services.openai_service import OpenAIService
service = OpenAIService()
embedding = service.generate_embedding('How do I create a ROS 2 node?')
print(f'✅ Embedding generated: {len(embedding)} dimensions')
assert len(embedding) == 1536, 'Expected 1536-dimension vector'
"
```

**Expected Output**: `✅ Embedding generated: 1536 dimensions`

#### Test 2: Qdrant Service - Vector Search

```bash
python -c "
from src.db.qdrant_client import QdrantService
service = QdrantService()
collections = service.list_collections()
print(f'✅ Qdrant collections: {collections}')
assert 'content_embeddings_en' in collections, 'Missing EN collection'
"
```

**Expected Output**: `✅ Qdrant collections: ['content_embeddings_en', 'content_embeddings_ur']`

#### Test 3: RAG Pipeline - End-to-End

```bash
python -c "
import asyncio
from src.services.rag_service import RAGService

async def test():
    rag = RAGService()
    result = await rag.process_query(
        query='How do I create a ROS 2 node with GPT-4?',
        language='en'
    )
    print(f'✅ Response length: {len(result[\"response\"])} characters')
    print(f'✅ Retrieved {result[\"num_chunks\"]} context chunks')
    print(f'Response: {result[\"response\"][:200]}...')

asyncio.run(test())
"
```

**Expected Output**: Response mentioning `LLMRobotController`, `rclpy`, or similar content from the docs.

### Backend API Endpoints (T064)

#### Test 4: Health Check

```bash
curl http://localhost:8000/health
```

**Expected Output**:
```json
{
  "status": "healthy",
  "database": "connected",
  "vector_db": "connected"
}
```

#### Test 5: Chat Health Check

```bash
curl http://localhost:8000/chat/health
```

**Expected Output**:
```json
{
  "status": "healthy",
  "rag_service": true,
  "timestamp": "2025-12-06T10:30:00.000Z"
}
```

#### Test 6: Chat Query (No Context)

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "language": "en"
  }'
```

**Expected Output**:
```json
{
  "response": "Physical AI refers to...",
  "retrieved_chunks": [...],
  "num_chunks": 3,
  "language": "en",
  "timestamp": "2025-12-06T10:30:00.000Z"
}
```

#### Test 7: Chat Query (With Selected Text)

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this code in detail",
    "language": "en",
    "selected_text": "class LLMRobotController(Node):\n    def __init__(self):\n        super().__init__('"'"'llm_controller'"'"')"
  }'
```

**Expected Output**: Response explaining the `LLMRobotController` class specifically.

#### Test 8: Chat Query (With History)

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What about actions?",
    "language": "en",
    "chat_history": [
      {"role": "user", "content": "Tell me about ROS 2 topics"},
      {"role": "assistant", "content": "ROS 2 topics enable publish-subscribe communication..."}
    ]
  }'
```

**Expected Output**: Response continuing the conversation about ROS 2 actions.

### Frontend Chat Interface (T065)

#### Test 9: Chat UI Loads

1. Navigate to http://localhost:3000
2. Verify **"💬 Ask AI"** button appears in bottom-right corner
3. Click button → Chat interface expands

**Expected Behavior**: Smooth animation, no console errors.

#### Test 10: Send Basic Query

1. Expand chat interface
2. Type: `"How do I create a ROS 2 node?"`
3. Press Enter or click **➤**

**Expected Behavior**:
- Loading indicator (3 animated dots) appears
- Response appears within 2-5 seconds
- Response mentions `rclpy`, `Node class`, or similar

#### Test 11: Selected Text Query

1. Navigate to any docs page (e.g., `/docs/module-1-ros2/nodes`)
2. Select a code block (e.g., the `LLMRobotController` class)
3. Chat interface shows **"📌 Asking about selected text"**
4. Ask: `"Explain this code"`

**Expected Behavior**: Response is scoped to the selected code block.

#### Test 12: Chat History Persistence

1. Ask: `"What is ROS 2?"`
2. Wait for response
3. Ask: `"What are its benefits?"`

**Expected Behavior**: Second response uses context from first question.

#### Test 13: Error Handling

1. Stop backend server (`Ctrl+C` in backend terminal)
2. Try sending a chat query

**Expected Behavior**: Error message appears: `"⚠️ Failed to get chat response"`

#### Test 14: Clear Chat

1. Send multiple queries
2. Click **"Clear"** button in chat header

**Expected Behavior**: All messages removed, chat history reset.

### Mobile Responsiveness (T061)

#### Test 15: Mobile Layout

1. Open Chrome DevTools (`F12`)
2. Toggle device toolbar (`Ctrl+Shift+M`)
3. Select iPhone 12 Pro viewport
4. Navigate to http://localhost:3000

**Expected Behavior**:
- Chat button is accessible
- Chat interface takes full width on mobile
- Input remains usable (no keyboard overlap)

---

## 🔧 Troubleshooting Tests

### Issue: "RAG service is not available"

**Diagnosis**:
```bash
# Check environment variables
cd backend
cat .env | grep OPENAI_API_KEY
```

**Fix**: Ensure `OPENAI_API_KEY` is set in `backend/.env`.

### Issue: "Collection not found"

**Diagnosis**:
```bash
python scripts/init_qdrant.py
```

**Fix**: Reinitialize Qdrant collections.

### Issue: "No relevant chunks retrieved"

**Diagnosis**:
```bash
python -c "
from src.db.qdrant_client import QdrantService
service = QdrantService()
count = service.client.count('content_embeddings_en')
print(f'Total embeddings: {count}')
"
```

**Fix**: If count is 0, run `python scripts/embed_content.py`.

### Issue: CORS errors in browser console

**Diagnosis**: Check browser console for messages like:
```
Access to fetch at 'http://localhost:8000/chat' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**Fix**: CORS should be configured in `backend/src/api/middleware/cors.py`. Verify `add_cors_middleware(app)` is called in `main.py`.

---

## 📊 Performance Benchmarks

### Expected Latencies

| Operation | Target | Acceptable |
|-----------|--------|------------|
| Embedding generation | < 500ms | < 1s |
| Vector search (Qdrant) | < 200ms | < 500ms |
| OpenAI completion | < 2s | < 5s |
| End-to-end chat query | < 3s | < 7s |

### Load Testing (Optional)

```bash
# Install Apache Bench
sudo apt-get install apache2-utils  # Ubuntu

# Test 100 requests, 10 concurrent
ab -n 100 -c 10 -p query.json -T application/json http://localhost:8000/chat
```

**query.json**:
```json
{
  "query": "What is ROS 2?",
  "language": "en"
}
```

---

## ✅ Phase 4 Acceptance Criteria

- [x] **T050**: OpenAI service generates 1536-dimension embeddings
- [x] **T051**: RAG service retrieves relevant chunks from Qdrant
- [x] **T052**: Chat API endpoint returns structured responses
- [x] **T053**: Pydantic schemas validate requests/responses
- [x] **T054**: Chat router registered in FastAPI
- [x] **T055**: RAG pipeline tested with sample queries
- [x] **T056**: Chat history persisted to database (optional for authenticated users)
- [x] **T057**: Error handling for OpenAI API failures
- [x] **T058**: ChatInterface component renders with animations
- [x] **T059**: Chat service communicates with backend API
- [x] **T060**: Chat integrated into Docusaurus Root theme
- [x] **T061**: Mobile-responsive CSS with smooth UX
- [x] **T062**: Loading states and typing indicator
- [x] **T063**: Error handling displays user-friendly messages
- [x] **T064**: Chat tested on all module pages
- [x] **T065**: RAG context verification (responses use retrieved chunks)

---

**Phase 4 Status**: ✅ **COMPLETE**

**Next Phase**: Phase 5 - Contextual Query on Selected Text (T066-T073) [Already implemented in ChatInterface]
