# 🚀 Backend Deployment Guide

Complete guide to deploy the RAG Chatbot backend to Vercel.

---

## Prerequisites

Before deploying, ensure you have:

1. ✅ **Vercel Account** - [Sign up free](https://vercel.com/signup)
2. ✅ **Neon Postgres Database** - [Get free tier](https://neon.tech/)
3. ✅ **Qdrant Cloud Account** - [Get free tier](https://cloud.qdrant.io/)
4. ✅ **Cohere API Key** - [Get API key](https://dashboard.cohere.com/api-keys)
5. ✅ **Git Repository** - Push your code to GitHub

---

## Step 1: Prepare External Services

### 1.1 Create Neon Postgres Database

1. Go to [Neon Console](https://console.neon.tech/)
2. Click **"New Project"**
3. Name: `humanoid-robotics-rag`
4. Region: Select closest to your users
5. Copy **Connection String** (looks like: `postgresql://user:pass@ep-xyz.neon.tech/neondb`)

### 1.2 Create Qdrant Cloud Cluster

1. Go to [Qdrant Cloud](https://cloud.qdrant.io/)
2. Click **"Create Cluster"**
3. Name: `humanoid-robotics-book`
4. Tier: **Free (1GB)**
5. Copy:
   - **Cluster URL**: `https://xyz-abc.qdrant.io`
   - **API Key**: Click "Generate API Key"

### 1.3 Get Cohere API Key

1. Go to [Cohere Dashboard](https://dashboard.cohere.com/)
2. Click **"API Keys"** → **"Create API Key"**
3. Name: `humanoid-robotics-chatbot`
4. Copy the key (starts with `co_...`)

---

## Step 2: Deploy Backend to Vercel

### 2.1 Install Vercel CLI (Optional but Recommended)

```bash
npm install -g vercel
```

### 2.2 Deploy Using Vercel CLI

```bash
# Navigate to backend directory
cd backend

# Login to Vercel
vercel login

# Deploy (follow prompts)
vercel

# When prompted:
# - Set up and deploy? → Yes
# - Which scope? → Your personal account
# - Link to existing project? → No
# - Project name? → humanoid-robotics-backend
# - Directory? → ./
# - Override settings? → No
```

### 2.3 Deploy Using Vercel Dashboard (Alternative)

1. Go to [Vercel Dashboard](https://vercel.com/dashboard)
2. Click **"Add New" → "Project"**
3. Import your GitHub repository
4. **Root Directory**: Set to `backend/`
5. Click **"Deploy"**

---

## Step 3: Configure Environment Variables

After deployment, add environment variables in Vercel:

### 3.1 Via Vercel Dashboard

1. Go to your project in [Vercel Dashboard](https://vercel.com/dashboard)
2. Click **"Settings" → "Environment Variables"**
3. Add the following variables:

```env
# LLM Provider
LLM_PROVIDER=cohere

# Cohere Configuration
COHERE_API_KEY=co_your_actual_api_key_here
COHERE_EMBEDDING_MODEL=embed-english-v3.0
COHERE_CHAT_MODEL=command-r-08-2024

# Qdrant Vector Store
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION=humanoid_robotics_book

# Neon Serverless Postgres
NEON_DATABASE_URL=postgresql://user:pass@ep-xyz.neon.tech/neondb

# Application Settings
APP_ENV=production
LOG_LEVEL=INFO
CORS_ORIGINS=https://your-frontend.vercel.app,http://localhost:3000

# Rate Limiting
RATE_LIMIT_REQUESTS=10
RATE_LIMIT_WINDOW=60

# RAG Configuration
RAG_TOP_K=5
RAG_SIMILARITY_THRESHOLD=0.5
RAG_MAX_CONTEXT_TOKENS=2000
RAG_CHUNK_SIZE=600
RAG_CHUNK_OVERLAP=100
```

4. Click **"Save"** for each variable

### 3.2 Via Vercel CLI (Faster)

```bash
# Set environment variables
vercel env add COHERE_API_KEY
# Paste your API key when prompted
# Select: Production

# Repeat for all variables above
```

---

## Step 4: Run Database Migrations

### 4.1 Connect to Neon Database

```bash
# Install psql (if not already installed)
# Windows: Download from https://www.postgresql.org/download/windows/
# Mac: brew install postgresql

# Connect to Neon
psql "postgresql://user:pass@ep-xyz.neon.tech/neondb"
```

### 4.2 Run Migration Script

```sql
-- Copy contents from backend/migrations/001_create_tables.sql
-- Paste and execute in psql
```

**OR** run migration script from your local machine:

```bash
cd backend

# Install dependencies
pip install -r requirements.txt

# Run migration
python scripts/run_migrations.py
```

---

## Step 5: Generate and Upload Embeddings

### 5.1 Prepare Book Content

Ensure your book content is in `humanoid_robot_book/docs/` as MDX files.

### 5.2 Run Embedding Generation Script

```bash
cd backend

# Ensure .env is configured with production credentials
python scripts/embed_book_content.py
```

**Expected Output:**
```
Processing: 01-introduction.mdx
Processing: 02-hardware-requirements.mdx
Processing: 03-module-1-ros2.mdx
Processing: 04-module-2-gazebo-unity.mdx
Processing: 05-module-3-nvidia-isaac.mdx
Processing: 06-module-4-vla.mdx
✅ Successfully uploaded 37 embeddings to Qdrant
```

### 5.3 Verify Embeddings in Qdrant

```bash
# Run validation script
python scripts/validate_embeddings.py
```

---

## Step 6: Test Backend API

### 6.1 Get Your Backend URL

Your Vercel deployment URL will be:
```
https://humanoid-robotics-backend.vercel.app
```

### 6.2 Test Health Endpoint

```bash
curl https://your-backend-url.vercel.app/health
```

**Expected Response:**
```json
{
  "status": "healthy",
  "environment": "production"
}
```

### 6.3 Test Chat Endpoint

```bash
curl -X POST https://your-backend-url.vercel.app/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "session_id": "test-session-123"
  }'
```

**Expected Response:**
```json
{
  "message_id": "uuid",
  "conversation_id": "uuid",
  "response": "ROS 2 (Robot Operating System 2) is...",
  "retrieved_chunks": [...],
  "metadata": {
    "latency_ms": 1250,
    "tokens_used": 450
  }
}
```

---

## Step 7: Update Frontend Configuration

Update your frontend `chatApi.ts` with production backend URL:

```typescript
// src/services/chatApi.ts
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-backend-url.vercel.app'  // ← Update this
  : 'http://localhost:8000';
```

---

## Step 8: Redeploy with Updated CORS

Update CORS_ORIGINS in Vercel to include your frontend URL:

```env
CORS_ORIGINS=https://your-frontend.vercel.app,http://localhost:3000
```

Then redeploy:

```bash
vercel --prod
```

---

## Troubleshooting

### Issue: "Module not found: cohere"

**Solution**: Ensure `cohere==5.11.0` is in `requirements.txt`

### Issue: "Database connection failed"

**Solution**:
1. Verify `NEON_DATABASE_URL` is correct
2. Check Neon database is active (not suspended)
3. Ensure SSL mode is enabled: `?sslmode=require`

### Issue: "Qdrant collection not found"

**Solution**:
1. Run `python scripts/setup_qdrant.py` to create collection
2. Run `python scripts/embed_book_content.py` to upload embeddings

### Issue: "CORS errors in frontend"

**Solution**:
1. Update `CORS_ORIGINS` in Vercel environment variables
2. Include both `http://localhost:3000` and production URL
3. Redeploy backend

### Issue: "API returns 500 errors"

**Solution**:
1. Check Vercel logs: `vercel logs --follow`
2. Verify all environment variables are set correctly
3. Check Cohere API key is valid

---

## Monitoring & Logs

### View Logs

```bash
# Real-time logs
vercel logs --follow

# Recent logs
vercel logs
```

### Check Cohere Usage

Go to [Cohere Dashboard](https://dashboard.cohere.com/) → **Usage**

### Check Qdrant Storage

Go to [Qdrant Cloud](https://cloud.qdrant.io/) → Your cluster → **Storage**

---

## Next Steps

After backend is deployed and tested:

1. ✅ Update frontend API URL
2. ✅ Test chatbot end-to-end from frontend
3. ✅ Monitor API latency (should be <3s p95)
4. ✅ Implement Better-Auth integration
5. ✅ Add Personalize button feature

---

## Production Checklist

- [ ] Backend deployed to Vercel
- [ ] All environment variables configured
- [ ] Database migrations executed
- [ ] Embeddings uploaded to Qdrant (37 chunks)
- [ ] Health endpoint returns 200 OK
- [ ] Chat endpoint returns valid responses
- [ ] Frontend updated with production backend URL
- [ ] CORS configured correctly
- [ ] Logs show no errors
- [ ] Cohere API usage within budget

---

**Deployment Complete!** 🎉

Your backend URL: `https://your-backend-url.vercel.app`
