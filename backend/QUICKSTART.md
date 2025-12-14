# RAG Chatbot Backend - Quick Start Guide

Get your RAG chatbot backend running in 5 minutes.

## 📋 Prerequisites

Before you start, get these API keys:

1. **OpenAI API Key**: https://platform.openai.com/api-keys
2. **Qdrant Cloud**: https://cloud.qdrant.io/ (Free tier)
3. **Neon Postgres**: https://neon.tech/ (Free tier)

## 🚀 Quick Start (5 Steps)

### 1. Install Dependencies (2 minutes)

```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate it
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install packages
pip install -r requirements.txt
```

### 2. Configure Environment (1 minute)

```bash
# Copy template
cp .env.example .env

# Edit .env and add your API keys
# Use any text editor (VS Code, Notepad, nano, etc.)
```

Required values in `.env`:
- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: Your Qdrant cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `NEON_DATABASE_URL`: Your Neon connection string

### 3. Check Environment (30 seconds)

```bash
cd src
python ../scripts/check_env.py
```

This verifies all API keys are configured. You should see all ✅ checkmarks.

### 4. Run Automated Setup (2 minutes)

```bash
python ../scripts/setup_all.py
```

This will:
- ✅ Create database tables
- ✅ Setup Qdrant collection
- ✅ Embed your book content (requires confirmation)
- ✅ Validate everything is working

**Note**: Embedding uses OpenAI API credits (~$0.01-0.05 for a typical book).

### 5. Start the Server (instant)

```bash
python main.py
```

Server runs at: **http://localhost:8000**

## ✅ Test Your Setup

### Option A: Browser (Easiest)

1. Open http://localhost:8000/docs
2. Click `/api/chat/query` → "Try it out"
3. Enter: `{"question": "What is ROS 2?"}`
4. Click "Execute"
5. See AI response! 🎉

### Option B: Command Line

```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

### Option C: Test Script

```bash
python ../scripts/test_rag.py
```

## 📁 What You Built

```
✅ FastAPI server (http://localhost:8000)
✅ Database (3 tables: sessions, conversations, messages)
✅ Vector store (embedded book content)
✅ RAG pipeline (embed → search → generate)
✅ API endpoint: POST /api/chat/query
```

## 🔧 Troubleshooting

### "Environment check failed"

Run: `python scripts/check_env.py`

Fix any ❌ items shown.

### "No MDX files found"

Make sure your book content is in:
`humanoid_robot_book/docs/*.mdx`

### "OpenAI API error"

- Check API key is correct
- Verify billing is set up: https://platform.openai.com/account/billing

### "Qdrant connection failed"

- Verify cluster URL and API key
- Check cluster is active in Qdrant dashboard

### "Database migration failed"

- Check Neon project is active (free tier suspends after inactivity)
- Verify connection string format

## 📖 Next Steps

### Build Frontend (React UI)

The backend is ready! Now build the chat interface:

See: `humanoid_robot_book/src/components/Chatbot/` (coming next)

### Deploy to Production

See: `specs/002-rag-chatbot/deployment.md`

Options:
- Vercel (FastAPI serverless)
- Railway (container hosting)
- Render (free tier available)

### Add Features

- Better-Auth integration (user login)
- Profile-aware responses (GPU/Edge/Cloud)
- Text selection queries
- Conversation history UI
- Urdu translation

## 📚 Documentation

- **Full Setup**: `backend/SETUP_CHECKLIST.md`
- **API Docs**: http://localhost:8000/docs (when server running)
- **Implementation Tasks**: `specs/002-rag-chatbot/tasks.md`
- **Architecture**: `specs/002-rag-chatbot/plan.md`

## 🆘 Need Help?

1. Check `backend/SETUP_CHECKLIST.md` for detailed steps
2. Run `python scripts/check_env.py` to diagnose issues
3. Check `backend/README.md` for troubleshooting

## 🎯 Success Criteria

You know it's working when:

✅ Server starts without errors
✅ http://localhost:8000/health returns `"status": "healthy"`
✅ http://localhost:8000/docs shows API documentation
✅ POST /api/chat/query returns AI-generated answer
✅ Response includes retrieved chunks from book content

---

**Estimated time to working backend**: 5-10 minutes

**Cost**: ~$0.01-0.05 for embedding (one-time)
