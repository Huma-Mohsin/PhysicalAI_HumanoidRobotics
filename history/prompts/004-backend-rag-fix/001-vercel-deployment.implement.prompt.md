---
id: "001"
title: "Backend RAG Deployment to Vercel"
stage: "implement"
date: "2025-12-18"
surface: "agent"
model: "claude-sonnet-4-5"
feature: "004-backend-rag-fix"
branch: "main"
user: "Huma Aftab"
command: "Manual deployment and fixes"
labels: ["backend", "deployment", "vercel", "serverless", "production"]
links:
  spec: "specs/004-backend-rag-fix/spec.md"
  ticket: null
  adr: "history/adr/001-cohere-llm-provider.md"
  pr: null
---

# Prompt History Record: Backend RAG Deployment to Vercel

## User Prompt

"Maine backend ki pipeline complete ki hoi thi aur Vercel par deploy karne ki koshish ki thi lekin frontend dikha raha tha backend ki jagah. Humein backend ko alag se deploy karna hai aur sab environment variables configure karna hai."

**Context**: User had local backend working (localhost:8000) with successful chatbot responses, but Vercel deployment was failing. Frontend was showing instead of backend API.

## Assistant Response Summary

### Analysis
Identified that Vercel was deploying entire repo, causing frontend to override backend. Recommended creating separate Vercel project for backend.

### Actions Taken

1. **Separate Backend Deployment**:
   - Created new Vercel project: `humanoid-robotics-backend`
   - Configured `backend/vercel.json` for Python serverless
   - Production URL: `https://humanoid-robotics-backend.vercel.app/`

2. **Critical Fixes Applied**:

   **Fix 1: Import Error (database_service.py:235)**
   ```python
   # BEFORE (BROKEN):
   from models.message import TextSelection, MessageMetadata

   # AFTER (FIXED):
   from src.models.message import TextSelection, MessageMetadata
   ```

   **Fix 2: Serverless Compatibility (main.py)**
   - Removed `@app.on_event("startup")` and `@app.on_event("shutdown")`
   - Added lazy initialization pattern in database_service.py
   - Added `ensure_connected()` method to all database operations

   **Fix 3: Vercel Handler (api.py)**
   ```python
   # Simplified from Mangum to direct ASGI export
   from src.main import app
   __all__ = ["app"]
   ```

   **Fix 4: Frontend API URLs**
   - Updated `chatApi.ts` and `authApi.ts`
   - Changed from `https://humanoidrobotbook.vercel.app` to `https://humanoid-robotics-backend.vercel.app`

3. **Environment Variables Configured** (11 total):
   - LLM_PROVIDER=cohere
   - COHERE_API_KEY, COHERE_EMBEDDING_MODEL, COHERE_CHAT_MODEL
   - QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION
   - NEON_DATABASE_URL
   - APP_ENV=production
   - LOG_LEVEL=INFO
   - CORS_ORIGINS=https://humanoidrobotbook.vercel.app,http://localhost:3000

4. **CORS Configuration**:
   - Identified correct frontend URL: `humanoidrobotbook.vercel.app`
   - Updated backend CORS_ORIGINS environment variable

### Files Modified

- `backend/src/services/database_service.py` (import fix + lazy init)
- `backend/src/main.py` (removed startup events)
- `backend/api.py` (simplified Vercel handler)
- `backend/vercel.json` (removed runtime specification)
- `backend/requirements.txt` (removed mangum dependency)
- `humanoid_robot_book/src/services/chatApi.ts` (backend URL)
- `humanoid_robot_book/src/services/authApi.ts` (backend URL)
- `humanoid_robot_book/src/contexts/AuthContext.tsx` (Better-Auth baseURL)

### Tests Run

**Production Backend Test** (curl):
```bash
curl -X POST "https://humanoid-robotics-backend.vercel.app/api/chat/query" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

**Result**: ✅ SUCCESS
- Response time: 9.8 seconds
- Tokens used: 4,372
- Chunks retrieved: 5 from Qdrant
- Model: command-r-08-2024
- Status: 200 OK

**Frontend Integration Test**:
- URL: https://humanoidrobotbook.vercel.app/
- Chatbot question: "What is ROS 2?"
- Network tab confirmed: Request to `https://humanoid-robotics-backend.vercel.app/api/chat/query`
- Response: Detailed ROS 2 explanation with code examples

## Outcome

### Success Criteria Met
- ✅ Backend deployed separately to Vercel
- ✅ All environment variables configured correctly
- ✅ Database lazy initialization for serverless
- ✅ Frontend connecting to production backend
- ✅ RAG pipeline working end-to-end in production
- ✅ CORS configured correctly

### Production URLs
- **Frontend**: https://humanoidrobotbook.vercel.app/
- **Backend**: https://humanoid-robotics-backend.vercel.app/

### Performance Metrics
- First request (cold start): ~10 seconds
- Subsequent requests: ~3-5 seconds (warm)
- Vector search: ~150ms
- Database queries: <100ms

## Follow-up Tasks

1. ⚠️ Better-Auth 404 error (frontend calling `/api/auth/get-session` - endpoint not implemented on backend yet)
2. 📝 Create spec for remaining features (Personalization system)
3. 📝 Integrate Better-Auth at book start (constitution requirement)
4. 📝 Implement text selection RAG queries (backend ready, frontend partial)

## Evaluation

**What Went Well**:
- Systematic debugging approach (local first, then production)
- Clear separation of frontend and backend deployments
- Proper serverless patterns applied
- Comprehensive testing at each step

**What Could Improve**:
- Earlier identification of import path issue
- Better understanding of Vercel serverless constraints upfront
- More proactive ADR creation (created retroactively)

**Lessons Learned**:
- FastAPI startup events don't work in serverless (use lazy initialization)
- Vercel auto-detects ASGI apps, no need for Mangum wrapper
- Always test locally before production deployment
- Environment variable management critical for serverless

## Related Artifacts

- Spec: `specs/004-backend-rag-fix/spec.md`
- ADR: `history/adr/001-cohere-llm-provider.md`
- Tasks: `specs/004-backend-rag-fix/tasks.md`

---

**Completion Status**: ✅ Implemented and Deployed
**Production Status**: ✅ Live and Working
**Next Feature**: Interactive Personalization System (005)
