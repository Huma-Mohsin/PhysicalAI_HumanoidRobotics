# Quick Start Guide: Physical AI RAG Platform

Get your Physical AI educational platform running in 5 minutes.

## Prerequisites

- **Python 3.11+** (for backend)
- **Node.js 18+** (for frontend)
- **Git** (for version control)
- **API Keys**:
  - OpenAI API key ([get here](https://platform.openai.com/api-keys))
  - Qdrant Cloud account ([free tier](https://qdrant.tech/cloud/))
  - Neon Postgres ([free tier](https://neon.tech/))

---

## 🚀 Setup (5 Minutes)

### 1. Clone and Install

```bash
# Clone repository
git clone <your-repo-url>
cd PhysicalAI_HumanoidRobotics

# Backend setup
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt

# Frontend setup
cd ../frontend
npm install
```

### 2. Configure Environment

**Backend (.env)**:
```bash
cd backend
cp .env.example .env
# Edit .env and add your API keys:
```

```env
# Database
DATABASE_URL=postgresql://user:password@your-neon-host/physicalai

# Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI
OPENAI_API_KEY=sk-your-openai-key
```

**Frontend (.env.local)**:
```bash
cd frontend
cp .env.example .env.local
# Default config should work for local development
```

### 3. Initialize Databases

```bash
cd backend

# Create database tables
alembic upgrade head

# Initialize Qdrant collections
python scripts/init_qdrant.py

# Embed educational content
python scripts/embed_content.py
```

Expected output:
```
✅ Successfully embedded 47 chunks to content_embeddings_en
```

### 4. Run the Application

**Terminal 1 - Backend**:
```bash
cd backend
source venv/bin/activate
python -m src.api.main
```

**Terminal 2 - Frontend**:
```bash
cd frontend
npm start
```

### 5. Test the Platform

Open your browser:
- **Frontend**: http://localhost:3000
- **API Docs**: http://localhost:8000/docs
- **Health Check**: http://localhost:8000/health

---

## ✅ Verify Installation

### Test RAG Chatbot

1. Navigate to http://localhost:3000
2. Click **"💬 Ask AI"** button in bottom-right
3. Ask: *"How do I create a ROS 2 node with GPT-4?"*
4. You should receive a contextual response with code examples

### Test Selected Text Query

1. On any docs page, select a paragraph of text
2. The chat interface will show "📌 Asking about selected text"
3. Ask a question about the selected content
4. Response will be scoped to that context

---

## 🐳 Docker Setup (Alternative)

```bash
# Build and run with Docker Compose
docker-compose up --build

# Backend: http://localhost:8000
# Frontend: http://localhost:3000
```

---

## 📖 What's Included?

### Educational Content (13 Chapters)
- **Introduction**: Physical AI concepts, platform overview
- **Module 1**: ROS 2 fundamentals with LLM integration
- **Module 2**: Digital Twin simulation (Gazebo, Unity)
- **Module 3**: NVIDIA Isaac Sim for synthetic data
- **Module 4**: Vision-Language-Action models

### Platform Features
- ✅ RAG-powered AI chatbot
- ✅ Context-scoped queries (selected text)
- ✅ 13-week structured syllabus
- ✅ Hardware-aware recommendations
- ✅ Constitutional compliance (100% AI+Physical integration)

---

## 🔧 Common Issues

### "Collection not found" error
```bash
# Reinitialize Qdrant
cd backend
python scripts/init_qdrant.py
```

### "No embeddings found" in chatbot
```bash
# Re-embed content
cd backend
python scripts/embed_content.py
```

### Frontend can't connect to backend
Check that:
- Backend is running on port 8000
- `REACT_APP_API_URL=http://localhost:8000` in `frontend/.env.local`
- CORS is enabled (already configured)

---

## 📚 Next Steps

1. **Explore Modules**: Navigate through ROS 2, Digital Twin, Isaac Sim, and VLA modules
2. **Customize Content**: Add your own chapters to `frontend/docs/`
3. **Re-embed**: Run `python backend/scripts/embed_content.py` after content changes
4. **Deploy**: Use Vercel (frontend) + Render (backend) via GitHub Actions

---

## 🆘 Need Help?

- **API Documentation**: http://localhost:8000/docs
- **Constitution**: See `.specify/memory/constitution.md` for platform principles
- **Specification**: See `specs/001-physical-ai-book-rag/spec.md`

---

**Platform Version**: 1.0.0
**Constitutional Compliance**: 100% (13/13 chapters)
**Last Updated**: 2025-12-06
