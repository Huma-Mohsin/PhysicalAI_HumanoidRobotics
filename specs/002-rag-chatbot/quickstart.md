# Quickstart: RAG Chatbot Local Development

**Feature**: `002-rag-chatbot`
**Phase**: Phase 1 (Development Setup)
**Date**: 2025-12-10

This guide helps you set up the RAG chatbot development environment on your local machine.

---

## Prerequisites

### System Requirements
- **OS**: Windows 10/11, macOS 11+, or Ubuntu 20.04+
- **Node.js**: v20.0+ (for Docusaurus frontend)
- **Python**: 3.11+ (for FastAPI backend)
- **Git**: Latest version
- **RAM**: 8GB minimum (16GB recommended for embedding generation)
- **Storage**: 5GB free space

### Account Setup (Required)
Before starting, create free accounts for the following services:

1. **OpenAI** (https://platform.openai.com/signup)
   - Sign up for an account
   - Add payment method (required, but free tier available)
   - Create API key: https://platform.openai.com/api-keys
   - Set budget alerts: https://platform.openai.com/account/limits

2. **Qdrant Cloud** (https://cloud.qdrant.io/signup)
   - Sign up with GitHub or email
   - Create a new cluster (Free Tier: 1GB)
   - Copy Cluster URL and API Key

3. **Neon** (https://neon.tech/signup)
   - Sign up with GitHub or email
   - Create a new project (Free Tier: 500MB)
   - Copy database connection string (postgres://...)

4. **Better-Auth** (Integrated with Feature 2)
   - If Feature 2 is not set up yet, use mock authentication for development

---

## Quick Setup (Automated)

### Option 1: One-Command Setup (Recommended)

```bash
# From project root
cd backend
./scripts/setup-dev.sh

# Follow prompts to enter API keys
```

This script will:
1. Install Python dependencies
2. Set up virtual environment
3. Create `.env` file with your API keys
4. Initialize Neon database schema
5. Create Qdrant collection
6. Generate embeddings from book content (first-time setup: ~5 minutes)

---

## Manual Setup (Step-by-Step)

### Step 1: Clone Repository

```bash
git clone <repository-url>
cd PhysicalAI_HumanoidRobotics
git checkout 002-rag-chatbot
```

### Step 2: Backend Setup

#### 2.1 Create Python Virtual Environment

```bash
cd backend
python3.11 -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate
```

#### 2.2 Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

**requirements.txt** (reference):
```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
openai==1.3.0
qdrant-client==1.6.9
asyncpg==0.29.0
pydantic==2.5.0
pydantic-settings==2.1.0
python-dotenv==1.0.0
pytest==7.4.3
pytest-asyncio==0.21.1
httpx==0.25.2
```

#### 2.3 Configure Environment Variables

Create `backend/.env`:

```bash
# Copy template
cp .env.example .env

# Edit .env with your API keys
nano .env  # or use any text editor
```

**Example `.env` file**:
```env
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-...

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=humanoid_robotics_book

# Neon Postgres Configuration
DATABASE_URL=postgresql+asyncpg://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb

# Backend Configuration
ENVIRONMENT=development  # development | production
CORS_ORIGINS=http://localhost:3000,http://localhost:3001
LOG_LEVEL=INFO

# Better-Auth Integration (if Feature 2 implemented)
BETTER_AUTH_URL=http://localhost:3000/api/auth
BETTER_AUTH_SECRET=your-auth-secret

# Rate Limiting
RATE_LIMIT_REQUESTS=10  # requests per minute per session
RATE_LIMIT_WINDOW=60    # window in seconds
```

### Step 3: Initialize Database

#### 3.1 Run Migrations

```bash
# From backend/ directory
python scripts/run_migrations.py
```

This creates:
- `user_sessions` table
- `conversations` table
- `messages` table
- Required indexes and triggers

#### 3.2 Verify Database Setup

```bash
# Test database connection
python scripts/test_db_connection.py

# Expected output:
# ✓ Database connection successful
# ✓ Tables created: user_sessions, conversations, messages
# ✓ Indexes created: 6 total
```

### Step 4: Initialize Qdrant Collection

```bash
# From backend/ directory
python scripts/setup_qdrant.py
```

This creates:
- `humanoid_robotics_book` collection
- Vector configuration (1536 dims, cosine distance)
- Payload indexes (chapter_id, hardware_relevance)

**Verify**:
```bash
python scripts/test_qdrant_connection.py

# Expected output:
# ✓ Qdrant connection successful
# ✓ Collection 'humanoid_robotics_book' exists
# ✓ Vector config: size=1536, distance=cosine
```

### Step 5: Generate Embeddings

#### 5.1 Parse Book Content

```bash
# From backend/ directory
python scripts/generate_embeddings.py --source ../humanoid_robot_book/docs
```

This script:
1. Parses all `.mdx` files from `docs/` directory
2. Chunks content using semantic chunking (600 tokens, 100 overlap)
3. Generates embeddings using OpenAI `text-embedding-3-small`
4. Uploads vectors + metadata to Qdrant

**Progress indicators**:
```
📖 Parsing book content...
  ✓ 01-introduction.mdx → 12 chunks
  ✓ 02-hardware-requirements.mdx → 18 chunks
  ✓ 03-module-1-ros2.mdx → 45 chunks
  ...
  Total: 142 chunks

🔢 Generating embeddings...
  Batch 1/2: 100 chunks [██████████] 100%
  Batch 2/2: 42 chunks  [██████████] 100%

☁️  Uploading to Qdrant...
  ✓ 142 vectors uploaded successfully

⏱️  Total time: 4m 23s
💰 Estimated cost: $0.012 (142 chunks × $0.00002/chunk)
```

#### 5.2 Verify Embeddings

```bash
python scripts/test_retrieval.py --query "What is ROS 2?"

# Expected output:
# 📊 Search Results (top 3):
# 1. [Score: 0.89] 03-module-1-ros2.mdx (Section: Core Concepts)
#    "ROS 2 (Robot Operating System 2) is built on three foundational..."
# 2. [Score: 0.85] 03-module-1-ros2.mdx (Section: Publisher-Subscriber Pattern)
#    "ROS 2 nodes communicate via topics using a pub-sub architecture..."
# 3. [Score: 0.81] 01-introduction.mdx (Section: What is Physical AI?)
#    "...ROS 2 provides the middleware for robot communication..."
```

### Step 6: Run Backend Server

```bash
# From backend/ directory (with venv activated)
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

**Verify backend**:
- Health check: http://localhost:8000/health
- API docs: http://localhost:8000/docs (Swagger UI)
- Redoc: http://localhost:8000/redoc

### Step 7: Frontend Setup

#### 7.1 Install Node Dependencies

```bash
# From project root
cd humanoid_robot_book
npm install
```

#### 7.2 Configure Frontend Environment

Create `humanoid_robot_book/.env.local`:

```env
# Backend API URL
NEXT_PUBLIC_API_URL=http://localhost:8000/api

# Better-Auth URL (if Feature 2 implemented)
NEXT_PUBLIC_AUTH_URL=http://localhost:3000/api/auth
```

#### 7.3 Run Docusaurus Development Server

```bash
# From humanoid_robot_book/ directory
npm start
```

**Expected output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

### Step 8: Test RAG Chatbot

1. Open http://localhost:3000
2. Click the **floating chat widget** (bottom-right corner)
3. Type a question: "What are the main components of ROS 2?"
4. Verify:
   - Response appears within 3 seconds
   - Answer includes information from Module 1
   - Citations reference correct chapters

**Test text selection**:
1. Navigate to any chapter (e.g., Module 1)
2. Highlight a paragraph about ROS 2 topics
3. Click "Ask about this selection" button
4. Type: "Explain this in simpler terms"
5. Verify response focuses on highlighted text

---

## Development Workflow

### Daily Development

```bash
# Terminal 1: Backend
cd backend
source venv/bin/activate  # or venv\Scripts\activate on Windows
uvicorn src.main:app --reload --port 8000

# Terminal 2: Frontend
cd humanoid_robot_book
npm start
```

### Re-generate Embeddings (After Content Changes)

```bash
# Only re-embed changed chapters
cd backend
python scripts/generate_embeddings.py --source ../humanoid_robot_book/docs --incremental

# Force full re-embedding
python scripts/generate_embeddings.py --source ../humanoid_robot_book/docs --force
```

### Run Tests

```bash
# Backend tests
cd backend
pytest tests/ -v

# Frontend tests (when implemented)
cd humanoid_robot_book
npm test
```

---

## Troubleshooting

### Common Issues

#### 1. **Import Error: No module named 'fastapi'**
**Solution**: Ensure virtual environment is activated
```bash
source venv/bin/activate  # macOS/Linux
venv\Scripts\activate     # Windows
```

#### 2. **Database Connection Error**
**Symptoms**: `asyncpg.exceptions.InvalidCatalogNameError`
**Solution**: Verify DATABASE_URL in `.env`, ensure Neon database is running

```bash
# Test connection manually
psql postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb
```

#### 3. **Qdrant Collection Not Found**
**Symptoms**: `qdrant_client.exceptions.UnexpectedResponse: 404`
**Solution**: Run setup script again
```bash
python scripts/setup_qdrant.py
```

#### 4. **OpenAI Rate Limit Exceeded**
**Symptoms**: `openai.error.RateLimitError: You exceeded your current quota`
**Solutions**:
- Check billing: https://platform.openai.com/account/billing
- Set up payment method
- Use smaller batch sizes for embedding generation:
  ```bash
  python scripts/generate_embeddings.py --batch-size 10
  ```

#### 5. **CORS Error in Browser**
**Symptoms**: `Access to XMLHttpRequest blocked by CORS policy`
**Solution**: Verify `CORS_ORIGINS` in backend `.env` includes frontend URL
```env
CORS_ORIGINS=http://localhost:3000
```

#### 6. **Frontend Can't Connect to Backend**
**Symptoms**: `ERR_CONNECTION_REFUSED`
**Solution**: Ensure backend is running on port 8000
```bash
curl http://localhost:8000/health
# Expected: {"status": "healthy"}
```

### Debug Mode

Enable verbose logging:

```bash
# Backend
LOG_LEVEL=DEBUG uvicorn src.main:app --reload

# Check logs
tail -f backend/logs/app.log
```

---

## Next Steps

After successful local setup:

1. **Test All User Stories**:
   - US1: General Q&A about book content
   - US2: Text selection queries
   - US3: Profile-aware responses (requires Feature 2)

2. **Run Integration Tests**:
   ```bash
   cd backend
   pytest tests/integration/ -v
   ```

3. **Review API Contracts**:
   - `specs/002-rag-chatbot/contracts/chat-query.md`
   - `specs/002-rag-chatbot/contracts/text-selection.md`
   - `specs/002-rag-chatbot/contracts/conversations.md`

4. **Proceed to Implementation**:
   - Run `/sp.tasks` to generate implementation tasks
   - Begin coding backend services (rag_service.py, embedding_service.py)

---

## Deployment Preview

### Deploy to Railway (Backend)

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login and deploy
railway login
railway link
railway up
```

### Deploy to Vercel (Frontend)

```bash
# Install Vercel CLI
npm install -g vercel

# Deploy from humanoid_robot_book/
cd humanoid_robot_book
vercel --prod
```

**Environment variables**: Set via Railway/Vercel dashboard (same as local `.env`)

---

##Resources

### Documentation Links
- FastAPI: https://fastapi.tiangolo.com/
- Docusaurus: https://docusaurus.io/docs
- OpenAI SDK: https://github.com/openai/openai-python
- Qdrant Client: https://qdrant.tech/documentation/quick-start/
- Neon Postgres: https://neon.tech/docs/introduction
- Better-Auth: https://www.better-auth.com/docs

### Project-Specific Docs
- Research: `specs/002-rag-chatbot/research.md`
- Data Model: `specs/002-rag-chatbot/data-model.md`
- API Contracts: `specs/002-rag-chatbot/contracts/`
- Constitution: `.specify/memory/constitution.md`

---

**Status**: Quickstart guide complete. Ready for development.
