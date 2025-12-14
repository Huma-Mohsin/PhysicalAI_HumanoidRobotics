# RAG Chatbot Backend - Setup Checklist

Complete this checklist to set up your RAG chatbot backend.

## ✅ Pre-Setup (Get API Keys)

- [ ] **OpenAI API Key**
  - Sign up: https://platform.openai.com/signup
  - Get API key: https://platform.openai.com/api-keys
  - Add payment method: https://platform.openai.com/account/billing
  - Set budget alerts (recommended: $10/month)
  - Copy key (starts with `sk-proj-...`)

- [ ] **Qdrant Cloud**
  - Sign up: https://cloud.qdrant.io/signup
  - Create cluster (Free Tier: 1GB)
  - Copy Cluster URL (e.g., `https://abc-123.qdrant.io`)
  - Copy API Key (from cluster settings)

- [ ] **Neon Postgres**
  - Sign up: https://neon.tech/signup
  - Create project (Free Tier: 500MB)
  - Copy connection string (from project dashboard)
  - Format: `postgresql://user:password@host.neon.tech/dbname?sslmode=require`

## ✅ Installation

- [ ] **Install Python 3.11+**
  - Check: `python --version` (should show 3.11+)
  - Download: https://www.python.org/downloads/

- [ ] **Create virtual environment**
  ```bash
  cd backend
  python -m venv venv
  ```

- [ ] **Activate virtual environment**
  ```bash
  # Windows
  venv\Scripts\activate

  # macOS/Linux
  source venv/bin/activate
  ```

- [ ] **Install dependencies**
  ```bash
  pip install -r requirements.txt
  ```

## ✅ Configuration

- [ ] **Create .env file**
  ```bash
  cp .env.example .env
  ```

- [ ] **Edit .env and add your credentials**
  ```env
  OPENAI_API_KEY=sk-proj-your-actual-key-here
  QDRANT_URL=https://your-cluster.qdrant.io
  QDRANT_API_KEY=your-qdrant-api-key-here
  NEON_DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require
  ```

- [ ] **Verify configuration**
  ```bash
  cd src
  python -c "from utils.config import settings; print('✅ Config loaded:', settings.openai_chat_model)"
  ```

## ✅ Database Setup

- [ ] **Run database migrations**
  ```bash
  cd src
  python ../scripts/run_migrations.py migrate
  ```

- [ ] **Verify migration success**
  - Should see: `✅ Migration 001_create_tables.sql completed successfully`

## ✅ Vector Store Setup

- [ ] **Create Qdrant collection**
  ```bash
  python ../scripts/setup_qdrant.py
  ```

- [ ] **Verify collection created**
  - Should see: `✅ Collection 'humanoid_robotics_book' created successfully`

## ✅ Embedding Book Content ⚠️ **CRITICAL**

- [ ] **Check book content exists**
  ```bash
  ls ../../humanoid_robot_book/docs/*.mdx
  ```
  - Should list MDX files (01-introduction.mdx, 03-module-1-ros2.mdx, etc.)

- [ ] **Run embedding script**
  ```bash
  python ../scripts/embed_book_content.py --docs-dir ../../humanoid_robot_book/docs
  ```

- [ ] **Monitor embedding progress**
  - Should see: `Found X MDX files to process`
  - Should see: `✅ Processed [filename]: Y chunks` for each file
  - Should see: `🎉 Successfully embedded Z chunks from X chapters`

- [ ] **Check API usage** (optional)
  - Visit: https://platform.openai.com/usage
  - Should show embedding API calls (cost: ~$0.01-0.05)

- [ ] **Validate embeddings**
  ```bash
  python ../scripts/validate_embeddings.py
  ```
  - Should see: `🎉 Validation passed! Collection is ready for RAG queries`

## ✅ Start API Server

- [ ] **Run the server**
  ```bash
  python main.py
  ```

- [ ] **Verify server started**
  - Should see: `Starting RAG Chatbot API in development mode`
  - Should see: `Uvicorn running on http://0.0.0.0:8000`

- [ ] **Test health endpoint**
  - Open: http://localhost:8000/health
  - Should return: `{"status": "healthy", ...}`

- [ ] **Open API documentation**
  - Open: http://localhost:8000/docs
  - Should see FastAPI Swagger UI

## ✅ Test RAG Chatbot

- [ ] **Test with curl**
  ```bash
  curl -X POST http://localhost:8000/api/chat/query \
    -H "Content-Type: application/json" \
    -d '{"question": "What is ROS 2?"}'
  ```

- [ ] **Verify response contains**
  - `"response"`: AI-generated answer about ROS 2
  - `"retrieved_chunks"`: Array of relevant book chunks
  - `"metadata"`: latency_ms, tokens_used, model

- [ ] **Test in Swagger UI**
  - Open: http://localhost:8000/docs
  - Click `/api/chat/query` → "Try it out"
  - Enter question: "What are the main components of ROS 2?"
  - Click "Execute"
  - Verify 200 response with AI-generated answer

## ✅ Optional: Automated Setup

Instead of manual steps, use the automated setup script:

```bash
python scripts/setup_all.py
```

This will:
1. Check environment configuration
2. Run database migrations
3. Setup Qdrant collection
4. Embed book content (with confirmation)
5. Validate embeddings

## 🐛 Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'openai'"

**Fix**: Install dependencies
```bash
pip install -r requirements.txt
```

### Issue: "Connection to database failed"

**Fix**: Check Neon database URL
- Ensure URL is correct in `.env`
- Check if Neon project is active (free tier auto-suspends)
- Try pinging Neon from terminal

### Issue: "Collection not found" (Qdrant)

**Fix**: Run Qdrant setup
```bash
python scripts/setup_qdrant.py
```

### Issue: "OpenAI API key invalid"

**Fix**: Update API key
- Get new key: https://platform.openai.com/api-keys
- Update `OPENAI_API_KEY` in `.env`
- Ensure billing is set up

### Issue: "No MDX files found"

**Fix**: Check book content path
- Verify path: `humanoid_robot_book/docs/`
- Ensure MDX files exist (not .md)
- Use `--docs-dir` flag to specify custom path

### Issue: "Embeddings validation failed"

**Fix**: Re-run embedding
```bash
python scripts/embed_book_content.py --docs-dir ../../humanoid_robot_book/docs
```

## 📊 Success Criteria

After completing this checklist, you should have:

- ✅ Python virtual environment activated
- ✅ All dependencies installed
- ✅ Environment variables configured
- ✅ Database tables created (user_sessions, conversations, messages)
- ✅ Qdrant collection created (1536-dim vectors, cosine similarity)
- ✅ Book content embedded (X chunks from Y chapters)
- ✅ API server running on http://localhost:8000
- ✅ RAG chatbot responding to questions

**Test Query**:
```
Question: "What is ROS 2?"
Expected: AI-generated answer mentioning nodes, topics, services
```

---

## 🚀 Next Steps

1. **Build Frontend** (React chatbot UI)
2. **Deploy to Production** (Vercel/Railway)
3. **Add Features** (Better-Auth, personalization, Urdu translation)

See `specs/002-rag-chatbot/tasks.md` for full implementation roadmap.
