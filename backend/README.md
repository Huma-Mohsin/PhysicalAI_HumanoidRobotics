# RAG Chatbot Backend

FastAPI backend for the Physical AI & Humanoid Robotics RAG chatbot.

## 🚀 Quick Start

### Prerequisites

- Python 3.11+
- OpenAI API key ([Get one here](https://platform.openai.com/api-keys))
- Qdrant Cloud account ([Free tier](https://cloud.qdrant.io/))
- Neon Postgres database ([Free tier](https://neon.tech/))

### 1. Install Dependencies

```bash
cd backend
python -m venv venv

# Windows
venv\Scripts\activate

# macOS/Linux
source venv/bin/activate

pip install -r requirements.txt
```

### 2. Configure Environment Variables

Create a `.env` file in the `backend/` directory:

```bash
cp .env.example .env
```

Edit `.env` and add your credentials:

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-your-actual-key-here
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_CHAT_MODEL=gpt-4o-mini

# Qdrant Vector Store
QDRANT_URL=https://your-cluster-url.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=humanoid_robotics_book

# Neon Serverless Postgres
NEON_DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# Application Settings
APP_ENV=development
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000,http://localhost:8000

# RAG Configuration
RAG_TOP_K=5
RAG_SIMILARITY_THRESHOLD=0.7
RAG_CHUNK_SIZE=600
RAG_CHUNK_OVERLAP=100
```

### 3. Setup Database (Neon Postgres)

Run migrations to create tables:

```bash
cd src
python ../scripts/run_migrations.py migrate
```

Expected output:
```
Running migration: 001_create_tables.sql
✅ Migration 001_create_tables.sql completed successfully
🎉 All migrations completed successfully
```

### 4. Setup Qdrant Vector Store

Create the vector collection:

```bash
python ../scripts/setup_qdrant.py
```

Expected output:
```
Connecting to Qdrant Cloud...
Creating collection 'humanoid_robotics_book'...
✅ Collection 'humanoid_robotics_book' created successfully
✅ Created index on 'chapter_id'
✅ Created index on 'hardware_relevance'
✅ Created index on 'content_type'
🎉 Qdrant setup complete!
```

### 5. Embed Book Content ⚠️ **CRITICAL**

This step populates Qdrant with embeddings from your book's MDX files:

```bash
python ../scripts/embed_book_content.py --docs-dir ../../humanoid_robot_book/docs
```

**This will**:
1. Parse all `.mdx` files in `humanoid_robot_book/docs/`
2. Chunk content into ~600 token pieces
3. Generate OpenAI embeddings (1536-dim vectors)
4. Upload to Qdrant Cloud

**Expected output**:
```
Found 6 MDX files to process
Processing: 01-introduction.mdx
Created 8 chunks from text (5432 chars)
Generated 8 embeddings
✅ Processed 01-introduction.mdx: 8 chunks
...
Uploaded batch 1: 48/48 points
🎉 Successfully embedded 48 chunks from 6 chapters
Collection 'humanoid_robotics_book' now has 48 points
```

**⚠️ Important**: This step costs OpenAI API credits (~$0.01-0.05 for a typical book). Monitor your usage at https://platform.openai.com/usage

### 6. Validate Embeddings

Verify Qdrant collection is ready:

```bash
python ../scripts/validate_embeddings.py
```

Expected output:
```
Validating Qdrant collection: humanoid_robotics_book
Points count: 48
Vector size: 1536
✅ Vector size correct (1536)
✅ Distance metric correct (Cosine)
✅ Collection has 48 points
✅ Point 1 valid: chapter=01-introduction, tokens=587
...
🎉 Validation passed! Collection is ready for RAG queries
```

### 7. Run the API Server

Start the FastAPI server:

```bash
python main.py
```

Or with uvicorn directly:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
Starting RAG Chatbot API in development mode
CORS origins: ['http://localhost:3000', 'http://localhost:8000']
OpenAI model: gpt-4o-mini
Qdrant collection: humanoid_robotics_book
Database service initialized
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### 8. Test the API

Open http://localhost:8000/docs to see interactive API documentation.

**Test with curl**:

```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the main components of ROS 2?"
  }'
```

**Expected response**:
```json
{
  "message_id": "uuid-here",
  "conversation_id": "uuid-here",
  "session_id": "uuid-here",
  "response": "ROS 2 has three main components:\n\n1. **Nodes**: Independent processes that perform computation\n2. **Topics**: Named buses for asynchronous message passing\n3. **Services**: Synchronous request-reply communication\n\n...",
  "retrieved_chunks": [...],
  "metadata": {
    "latency_ms": 2340,
    "tokens_used": 856,
    "model": "gpt-4o-mini",
    "qdrant_query_ms": 120,
    "retrieved_count": 5
  }
}
```

---

## 📁 Project Structure

```
backend/
├── src/
│   ├── main.py              # FastAPI app entry point
│   ├── models/              # Pydantic models
│   │   ├── user_session.py
│   │   ├── conversation.py
│   │   └── message.py
│   ├── services/            # Business logic
│   │   ├── database_service.py
│   │   ├── embedding_service.py
│   │   ├── qdrant_service.py
│   │   └── rag_service.py
│   ├── api/                 # API endpoints
│   │   └── chat.py
│   └── utils/               # Utilities
│       ├── config.py
│       └── logger.py
├── migrations/              # SQL migrations
│   ├── 001_create_tables.sql
│   └── 001_rollback.sql
├── scripts/                 # Setup scripts
│   ├── run_migrations.py
│   ├── setup_qdrant.py
│   ├── embed_book_content.py
│   └── validate_embeddings.py
├── requirements.txt         # Python dependencies
├── .env.example            # Environment template
└── README.md               # This file
```

---

## 🛠️ Maintenance Commands

### Re-embed Book Content

If you update your book content, re-run the embedding script:

```bash
python ../scripts/embed_book_content.py --docs-dir ../../humanoid_robot_book/docs
```

It will ask if you want to recreate the collection.

### Rollback Database Migration

```bash
python ../scripts/run_migrations.py rollback --migration 001
```

### Check Qdrant Collection Stats

```python
from services.qdrant_service import qdrant_service
stats = qdrant_service.get_collection_stats()
print(stats)
```

---

## 🐛 Troubleshooting

### "Collection not found" error

Run `python ../scripts/setup_qdrant.py` to create the collection.

### "Database connection failed"

- Check your `NEON_DATABASE_URL` in `.env`
- Ensure your Neon project is active (free tier auto-suspends after inactivity)
- Verify network connectivity to Neon

### "OpenAI API key invalid"

- Get a new key: https://platform.openai.com/api-keys
- Add billing method: https://platform.openai.com/account/billing
- Update `OPENAI_API_KEY` in `.env`

### "No embeddings found" (empty Qdrant)

Run the embedding script: `python ../scripts/embed_book_content.py`

### High latency (>5 seconds)

- Check your OpenAI plan (free tier has rate limits)
- Reduce `RAG_TOP_K` in `.env` (default: 5)
- Check Qdrant Cloud region (choose closest to you)

---

## 📊 API Endpoints

### `POST /api/chat/query`

General Q&A about book content.

**Request**:
```json
{
  "question": "What is ROS 2?",
  "conversation_id": "optional-uuid",
  "session_id": "optional-uuid"
}
```

**Response**: ChatQueryResponse (see API docs)

### `GET /health`

Health check endpoint.

### `GET /docs`

Interactive API documentation (FastAPI Swagger UI).

---

## 🚀 Deployment

See `specs/002-rag-chatbot/deployment.md` for production deployment guide (Vercel, Railway, etc.).

---

## 📝 License

Part of the Physical AI & Humanoid Robotics book project.
