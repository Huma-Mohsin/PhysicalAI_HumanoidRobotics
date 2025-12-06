# Quickstart Guide

**Feature**: Physical AI & Humanoid Robotics Capstone Book & RAG Platform
**Date**: 2025-12-05
**Phase**: Phase 1 - Developer Onboarding

This guide provides step-by-step instructions for setting up the development environment and running the platform locally.

---

## Prerequisites

Before you begin, ensure you have the following installed:

- **Node.js** 18+ (for Docusaurus frontend)
- **Python** 3.11+ (for FastAPI backend)
- **Git** (for version control)
- **Docker** (optional, for containerized development)
- **npm** or **yarn** (package manager for frontend)
- **pip** (package manager for Python)

### External Accounts Required

You'll need to create free-tier accounts for the following services:

1. **Qdrant Cloud**: [https://cloud.qdrant.io/](https://cloud.qdrant.io/)
   - Create a cluster and retrieve API key + cluster URL

2. **Neon Serverless Postgres**: [https://neon.tech/](https://neon.tech/)
   - Create a database instance and retrieve connection string

3. **OpenAI**: [https://platform.openai.com/](https://platform.openai.com/)
   - Obtain API key for embeddings and chat completions
   - Set up billing alerts to monitor usage

4. **Better-Auth**: Configuration handled locally (no external account needed for development)

---

## Setup Steps

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/physical-ai-rag-platform.git
cd physical-ai-rag-platform
```

---

### 2. Backend Setup (FastAPI)

#### 2.1 Navigate to Backend Directory

```bash
cd backend
```

#### 2.2 Create Python Virtual Environment

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate
```

#### 2.3 Install Python Dependencies

```bash
pip install -r requirements.txt
```

#### 2.4 Configure Environment Variables

Create a `.env` file in the `backend/` directory:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-your-openai-api-key-here

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster-url.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here

# Neon Postgres Configuration
DATABASE_URL=postgres://user:password@ep-your-endpoint.us-east-2.aws.neon.tech/dbname?sslmode=require

# Better-Auth Configuration
AUTH_SECRET=your-random-secret-key-here  # Generate with: openssl rand -hex 32

# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=http://localhost:3000,http://127.0.0.1:3000  # Docusaurus dev server

# Environment
ENV=development
```

#### 2.5 Initialize Database Schema

Run the Neon Postgres migration script:

```bash
python -m alembic upgrade head
# Or manually execute the SQL from specs/001-physical-ai-book-rag/data-model.md
```

#### 2.6 Initialize Qdrant Collections

```bash
python scripts/init_qdrant.py
```

This script creates the `content_embeddings_en` and `content_embeddings_ur` collections.

#### 2.7 Run the Backend Server

```bash
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

The API should now be running at `http://localhost:8000`.

**Verify Backend**:
- Open `http://localhost:8000/docs` in your browser (Swagger UI)
- Try the `/health` endpoint to confirm database connectivity

---

### 3. Frontend Setup (Docusaurus)

#### 3.1 Navigate to Frontend Directory

```bash
cd ../frontend  # From repository root: cd frontend
```

#### 3.2 Install Node.js Dependencies

```bash
npm install
# Or with yarn:
yarn install
```

#### 3.3 Configure Environment Variables

Create a `.env` file in the `frontend/` directory:

```bash
cp .env.example .env
```

Edit `.env`:

```env
# Backend API URL
REACT_APP_API_URL=http://localhost:8000

# Better-Auth Configuration
REACT_APP_AUTH_URL=http://localhost:8000/auth
```

#### 3.4 Run the Development Server

```bash
npm start
# Or with yarn:
yarn start
```

The Docusaurus site should now be running at `http://localhost:3000`.

**Verify Frontend**:
- Navigate to `http://localhost:3000`
- You should see the homepage (even with placeholder content)
- Check browser console for any errors

---

### 4. Content Indexing (Populate RAG Database)

Before the chatbot can answer questions, you need to index the book content into Qdrant.

#### 4.1 Ensure Content Exists

Verify that Markdown content files are present in `frontend/docs/` directory:

```bash
ls frontend/docs/
# Expected output: intro.md, module-1-ros2/, module-2-digital-twin/, etc.
```

#### 4.2 Run Indexing Script

From the `backend/` directory:

```bash
python scripts/index_content.py --language both
```

This script will:
1. Parse all Markdown files from `frontend/docs/` (English) and `frontend/i18n/ur/` (Urdu)
2. Chunk the content (~500 tokens per chunk, 50-token overlap)
3. Generate embeddings using OpenAI API
4. Upload embeddings to Qdrant Cloud

**Expected Output**:
```
Indexing English content...
Processed 1243 chunks from frontend/docs/
Indexing Urdu content...
Processed 1189 chunks from frontend/i18n/ur/
Indexing completed in 127.45 seconds.
```

#### 4.3 Verify Indexing

Test a query via the API:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is a ROS 2 node?",
    "language": "en",
    "session_id": "test_session"
  }'
```

You should receive a JSON response with a generated answer and citations.

---

### 5. Local Development with Docker (Optional)

For a unified development environment, use Docker Compose:

#### 5.1 Build and Run Containers

From the repository root:

```bash
docker-compose up --build
```

This will:
- Build the backend container (FastAPI)
- Build the frontend container (Docusaurus)
- Start both services with hot-reloading enabled

**Services**:
- Frontend: `http://localhost:3000`
- Backend: `http://localhost:8000`
- Backend API Docs: `http://localhost:8000/docs`

#### 5.2 Stop Containers

```bash
docker-compose down
```

---

## Testing

### Backend Tests

From the `backend/` directory:

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=html

# Run specific test file
pytest tests/unit/test_embedder.py
```

### Frontend Tests

From the `frontend/` directory:

```bash
# Run Jest tests
npm test

# Run E2E tests (Playwright)
npx playwright test
```

---

## Common Issues & Troubleshooting

### Issue: "OPENAI_API_KEY not set"
**Solution**: Ensure `.env` file exists in `backend/` and contains valid OpenAI API key.

### Issue: "Connection to Qdrant failed"
**Solution**:
- Verify Qdrant cluster is active in Qdrant Cloud dashboard
- Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Test connectivity: `curl -H "api-key: YOUR_KEY" https://your-cluster.qdrant.io/collections`

### Issue: "Database connection error"
**Solution**:
- Verify Neon Postgres instance is running
- Check `DATABASE_URL` format: `postgres://user:password@host/dbname?sslmode=require`
- Ensure IP whitelisting allows your local machine (Neon settings)

### Issue: "CORS error when calling API from frontend"
**Solution**:
- Verify `CORS_ORIGINS` in backend `.env` includes `http://localhost:3000`
- Check that backend is running before starting frontend

### Issue: "Indexing script fails with 'RateLimitError'"
**Solution**:
- OpenAI API rate limits may be hit during bulk embedding generation
- Add delays in indexing script or use batching
- Check OpenAI dashboard for usage limits

---

## Development Workflow

1. **Start Backend**: `cd backend && uvicorn src.api.main:app --reload`
2. **Start Frontend**: `cd frontend && npm start`
3. **Make Changes**: Edit code in `backend/src/` or `frontend/src/`
4. **Test Changes**: Use hot-reload (backend auto-restarts, frontend auto-refreshes)
5. **Run Tests**: `pytest` (backend) and `npm test` (frontend)
6. **Commit**: Follow conventional commits (e.g., `feat: add contextual query endpoint`)

---

## Deployment

### Frontend Deployment (Vercel)

1. **Connect GitHub Repository** to Vercel
2. **Configure Build Settings**:
   - Build Command: `cd frontend && npm run build`
   - Output Directory: `frontend/build`
   - Environment Variables: Add `REACT_APP_API_URL` (production backend URL)
3. **Deploy**: Vercel auto-deploys on push to `main` branch

### Backend Deployment (Render)

1. **Create Web Service** on Render
2. **Configure Service**:
   - Build Command: `cd backend && pip install -r requirements.txt`
   - Start Command: `cd backend && uvicorn src.api.main:app --host 0.0.0.0 --port $PORT`
   - Environment Variables: Add all variables from `.env` (use Render dashboard)
3. **Deploy**: Render auto-deploys on push to `main` branch

---

## Next Steps

After completing this quickstart:

1. **Content Generation**: Use Claude Code to generate Module 1-4 content (see `specs/001-physical-ai-book-rag/plan.md`, Phase 2, Task 2.1)
2. **Implement Chatbot UI**: Build React components for chat widget (Task 3.1)
3. **Add Personalization**: Implement hardware-aware content toggles (Task 3.2)
4. **Enable Urdu Localization**: Configure Docusaurus i18n plugin (Task 3.3)
5. **Create Subagent Skills**: Implement URDF generation skill (Task 3.4)

---

## Additional Resources

- **Docusaurus Documentation**: [https://docusaurus.io/docs](https://docusaurus.io/docs)
- **FastAPI Documentation**: [https://fastapi.tiangolo.com/](https://fastapi.tiangolo.com/)
- **OpenAI Agents SDK**: [https://platform.openai.com/docs/agents](https://platform.openai.com/docs/agents)
- **Qdrant Documentation**: [https://qdrant.tech/documentation/](https://qdrant.tech/documentation/)
- **Better-Auth Documentation**: [https://www.better-auth.com/docs](https://www.better-auth.com/docs)

---

## Support

For issues or questions:
- **GitHub Issues**: [https://github.com/your-org/physical-ai-rag-platform/issues](https://github.com/your-org/physical-ai-rag-platform/issues)
- **Project Documentation**: See `specs/001-physical-ai-book-rag/` for detailed planning artifacts
