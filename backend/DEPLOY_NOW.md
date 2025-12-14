# 🚀 Deploy Backend NOW - Simple Guide

Follow these exact steps to deploy your backend in 15-20 minutes.

---

## STEP 1: Open Terminal in Backend Folder

```bash
# Navigate to backend directory
cd C:\Users\Huma Aftab\OneDrive\Documents\Gemini_cli_practice_q4\AINative-HackathonProjects\project_1\PhysicalAI_HumanoidRobotics\backend
```

---

## STEP 2: Login to Vercel

```bash
vercel login
```

**What happens:**
- Browser will open
- Click "Continue with GitHub" (or Email)
- Authorize Vercel CLI
- Come back to terminal

---

## STEP 3: Deploy Backend

```bash
vercel
```

**You'll see prompts - Answer like this:**

```
? Set up and deploy "backend"? → Press Y (Yes)
? Which scope? → Select your account (use arrow keys, press Enter)
? Link to existing project? → Press N (No)
? What's your project's name? → Type: humanoid-robotics-backend
? In which directory is your code located? → Press Enter (./)
? Want to override settings? → Press N (No)
```

**Wait 1-2 minutes... Backend will deploy!**

**You'll get a URL like:**
```
✅ Deployed to production: https://humanoid-robotics-backend.vercel.app
```

**COPY THIS URL!** You'll need it later.

---

## STEP 4: Get External Service Keys

Now you need 3 services. Open these links in browser:

### 4.1 Neon Postgres (FREE - Database)

1. Go to: https://console.neon.tech/signup
2. Sign up with GitHub
3. Click "Create Project"
4. Name: `humanoid-robotics-db`
5. Click "Create Project"
6. **Copy the Connection String** (looks like):
   ```
   postgresql://alex:AbC123...@ep-cool-darkness-123456.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```
7. Save this somewhere safe!

---

### 4.2 Qdrant Cloud (FREE - Vector Database)

1. Go to: https://cloud.qdrant.io/login
2. Sign up with Google/GitHub
3. Click "Create Cluster"
4. Settings:
   - Name: `humanoid-book`
   - Cloud: AWS
   - Region: us-east-1
   - Configuration: Free (1 GB)
5. Click "Create"
6. Wait 1 minute for cluster to start
7. Click on your cluster name
8. Click "Data Access Control" → "Generate API Key"
9. **Copy these two things:**
   - Cluster URL: `https://abc-xyz-123.qdrant.io` (from cluster dashboard)
   - API Key: `qdr_abc123...` (from API Keys section)
10. Save both!

---

### 4.3 Cohere API Key (PAID ~$0.10 per 1M tokens)

1. Go to: https://dashboard.cohere.com/api-keys
2. Sign up with Google/GitHub
3. Add billing info (you'll get $5-10 free credits usually)
4. Click "Create API Key"
5. Name: `humanoid-robotics`
6. Click "Create"
7. **Copy the API Key** (starts with `co_...`)
8. Save it!

---

## STEP 5: Add Environment Variables to Vercel

Now go to Vercel dashboard:

1. Open: https://vercel.com/dashboard
2. Click on your project: `humanoid-robotics-backend`
3. Click "Settings" tab (top)
4. Click "Environment Variables" (left sidebar)

**Add these variables one by one:**

Click "Add New" button, then:

### Variable 1: LLM_PROVIDER
- **Key:** `LLM_PROVIDER`
- **Value:** `cohere`
- **Environment:** Production, Preview, Development (select all)
- Click "Save"

### Variable 2: COHERE_API_KEY
- **Key:** `COHERE_API_KEY`
- **Value:** Paste your Cohere API key (starts with `co_...`)
- **Environment:** Production, Preview, Development
- Click "Save"

### Variable 3: COHERE_EMBEDDING_MODEL
- **Key:** `COHERE_EMBEDDING_MODEL`
- **Value:** `embed-english-v3.0`
- **Environment:** Production, Preview, Development
- Click "Save"

### Variable 4: COHERE_CHAT_MODEL
- **Key:** `COHERE_CHAT_MODEL`
- **Value:** `command-r-08-2024`
- **Environment:** Production, Preview, Development
- Click "Save"

### Variable 5: QDRANT_URL
- **Key:** `QDRANT_URL`
- **Value:** Paste your Qdrant cluster URL (e.g., `https://abc-xyz.qdrant.io`)
- **Environment:** Production, Preview, Development
- Click "Save"

### Variable 6: QDRANT_API_KEY
- **Key:** `QDRANT_API_KEY`
- **Value:** Paste your Qdrant API key
- **Environment:** Production, Preview, Development
- Click "Save"

### Variable 7: QDRANT_COLLECTION
- **Key:** `QDRANT_COLLECTION`
- **Value:** `humanoid_robotics_book`
- **Environment:** Production, Preview, Development
- Click "Save"

### Variable 8: NEON_DATABASE_URL
- **Key:** `NEON_DATABASE_URL`
- **Value:** Paste your Neon connection string (starts with `postgresql://`)
- **Environment:** Production, Preview, Development
- Click "Save"

### Variable 9: APP_ENV
- **Key:** `APP_ENV`
- **Value:** `production`
- **Environment:** Production
- Click "Save"

### Variable 10: CORS_ORIGINS
- **Key:** `CORS_ORIGINS`
- **Value:** `https://your-frontend.vercel.app,http://localhost:3000`
  (Replace `your-frontend.vercel.app` with your actual frontend URL)
- **Environment:** Production, Preview, Development
- Click "Save"

---

## STEP 6: Redeploy with Environment Variables

Environment variables add karne ke baad, redeploy karna zaroori hai:

**Option A: Via Dashboard**
1. Go to "Deployments" tab
2. Click "..." on latest deployment
3. Click "Redeploy"
4. Wait 1-2 minutes

**Option B: Via Terminal**
```bash
vercel --prod
```

---

## STEP 7: Test Your Backend

Open browser and go to:

```
https://your-backend-url.vercel.app/health
```

**Expected Response:**
```json
{
  "status": "healthy",
  "environment": "production"
}
```

✅ **If you see this, your backend is LIVE!**

---

## STEP 8: Run Database Migrations

Backend is live, but database is empty. Let's create tables:

```bash
# In your terminal (backend directory)
cd C:\Users\Huma Aftab\OneDrive\Documents\Gemini_cli_practice_q4\AINative-HackathonProjects\project_1\PhysicalAI_HumanoidRobotics\backend

# Install dependencies if not installed
pip install -r requirements.txt

# Set local .env with production credentials
# Copy your Neon database URL to backend/.env
# Add: NEON_DATABASE_URL=your_neon_url_here

# Run migrations
python scripts/run_migrations.py
```

**Expected Output:**
```
✅ Database tables created successfully
✅ Trigger created for conversation updates
```

---

## STEP 9: Generate Book Embeddings

Now upload book content to Qdrant:

```bash
# Make sure your .env has Qdrant and Cohere credentials
# Add these to backend/.env:
# QDRANT_URL=your_qdrant_url
# QDRANT_API_KEY=your_qdrant_key
# COHERE_API_KEY=your_cohere_key
# LLM_PROVIDER=cohere
# COHERE_EMBEDDING_MODEL=embed-english-v3.0

# Run embedding generation
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

---

## STEP 10: Test Chat API

Test if chatbot can answer questions:

```bash
curl -X POST https://your-backend-url.vercel.app/api/chat/query \
  -H "Content-Type: application/json" \
  -d "{\"question\": \"What is ROS 2?\", \"session_id\": \"test-123\"}"
```

**Expected Response:**
```json
{
  "message_id": "uuid-here",
  "conversation_id": "uuid-here",
  "response": "ROS 2 (Robot Operating System 2) is the industry-standard middleware...",
  "retrieved_chunks": [...],
  "metadata": {
    "latency_ms": 1200,
    "tokens_used": 450
  }
}
```

✅ **If you see a response about ROS 2, everything is working!**

---

## STEP 11: Update Frontend API URL

Edit this file:
`humanoid_robot_book/src/services/chatApi.ts`

**Change line 6-8 to:**
```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-actual-backend-url.vercel.app'  // ← Your backend URL here
  : 'http://localhost:8000';
```

**Then redeploy frontend:**
```bash
cd ../humanoid_robot_book
git add .
git commit -m "feat: connect chatbot to production backend"
git push
```

Vercel will auto-deploy your frontend.

---

## STEP 12: Test on Live Website

1. Go to your live book: `https://your-book.vercel.app`
2. Click the chatbot button (bottom-right corner)
3. Type: "What is ROS 2?"
4. Wait 2-3 seconds
5. You should see an answer!

---

## ✅ DEPLOYMENT COMPLETE!

Your RAG chatbot is now LIVE! 🎉

**What you achieved:**
- ✅ Backend deployed to Vercel
- ✅ Database created and migrated
- ✅ Book content embedded in vector database
- ✅ Chatbot working end-to-end
- ✅ Users can ask questions and get answers

**Current Score: 100/250 points**

---

## 🔍 Troubleshooting

### Issue: "Module not found: cohere"
**Fix:** Make sure `cohere==5.11.0` is in `requirements.txt`, then redeploy

### Issue: Backend returns 500 errors
**Fix:**
1. Check Vercel logs: https://vercel.com/dashboard → Your project → Deployments → Latest → Logs
2. Verify all environment variables are set correctly
3. Check Cohere API key is valid

### Issue: "Collection not found" in Qdrant
**Fix:** Run `python scripts/setup_qdrant.py` first, then `embed_book_content.py`

### Issue: Embeddings script fails
**Fix:**
1. Make sure `.env` has all credentials
2. Check Qdrant cluster is running (not suspended)
3. Verify Cohere API key is valid

---

## 📞 Need Help?

If stuck, ask Claude Code or check:
- Vercel logs: https://vercel.com/dashboard
- Cohere usage: https://dashboard.cohere.com/usage
- Qdrant dashboard: https://cloud.qdrant.io/

---

**Next:** After chatbot works, implement Better-Auth, Personalize, Urdu Translation, and Reusable Intelligence features!
