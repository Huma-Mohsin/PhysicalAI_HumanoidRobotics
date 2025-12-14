# Vercel Environment Variables - Complete Checklist

## Critical Required Variables (Must Have - No Defaults)

Add these **EXACTLY** as shown with UPPERCASE names in Vercel Dashboard:

### 1. QDRANT_URL
```
QDRANT_URL=https://80eea1b7-b6bd-46ad-b0d9-2331fabe8794.us-east4-0.gcp.cloud.qdrant.io
```

### 2. QDRANT_API_KEY ⚠️ CRITICAL
```
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.BZeuO046GC1jVx6wVEitrcU1gCh2D7D4QnaWn8yBTSI
```

### 3. NEON_DATABASE_URL ⚠️ CRITICAL
```
NEON_DATABASE_URL=postgresql://neondb_owner:T3EdLwVxC8Xg@ep-restless-water-a1uvgzo2-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require
```

## Essential for Production (Has defaults but won't work without proper values)

### 4. COHERE_API_KEY
```
COHERE_API_KEY=BRqwL7jH3WGuSy1oSoulLvjEI64pcYJtgsBLkIf7
```

### 5. LLM_PROVIDER
```
LLM_PROVIDER=cohere
```

### 6. COHERE_EMBEDDING_MODEL
```
COHERE_EMBEDDING_MODEL=embed-english-v3.0
```

### 7. COHERE_CHAT_MODEL
```
COHERE_CHAT_MODEL=command-r-08-2024
```

### 8. QDRANT_COLLECTION
```
QDRANT_COLLECTION=humanoid_robotics_book
```

### 9. APP_ENV
```
APP_ENV=production
```

### 10. CORS_ORIGINS
```
CORS_ORIGINS=https://humanoid-robot-book.vercel.app,http://localhost:3000
```

---

## How to Add in Vercel Dashboard

1. **Go to:** https://vercel.com/huma-mohsins-projects/humanoidbackend/settings/environment-variables

2. **For EACH variable above:**
   - Click "Add New"
   - **Key:** Copy EXACTLY from above (e.g., `QDRANT_API_KEY`)
   - **Value:** Copy the value after `=`
   - **Environments:** Select "Production" (or keep "All" selected)
   - Click "Save"

3. **Verify UPPERCASE:**
   - ❌ WRONG: `qdrant_api_key` (lowercase)
   - ✅ CORRECT: `QDRANT_API_KEY` (UPPERCASE)

4. **After adding all 10 variables, redeploy:**
   ```bash
   cd backend
   npx vercel --prod
   ```

---

## Verification Checklist

After adding, you should see ALL 10 variables in the dashboard:
- [ ] QDRANT_URL
- [ ] QDRANT_API_KEY ⚠️
- [ ] NEON_DATABASE_URL ⚠️
- [ ] COHERE_API_KEY
- [ ] LLM_PROVIDER
- [ ] COHERE_EMBEDDING_MODEL
- [ ] COHERE_CHAT_MODEL
- [ ] QDRANT_COLLECTION
- [ ] APP_ENV
- [ ] CORS_ORIGINS

---

## Current Issue

**Error:** `qdrant_api_key Field required, neon_database_url Field required`

**Cause:** Variables #2 and #3 above are NOT in Vercel (or have wrong names)

**Fix:** Add QDRANT_API_KEY and NEON_DATABASE_URL with EXACT uppercase names shown above
