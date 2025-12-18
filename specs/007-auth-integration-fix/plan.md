# Implementation Plan: Better-Auth Integration at Book Start

**Feature**: 007-auth-integration-fix
**Created**: 2025-12-18
**Status**: Planning
**Priority**: P1 (50 bonus points)

---

## Current State Analysis

### What Exists ✅
1. **Backend Auth Files** (ALREADY IMPLEMENTED!):
   - `backend/src/api/auth.py` (182 lines) - Auth endpoints defined
   - `backend/src/services/auth_service.py` (272 lines) - Auth service logic
   - Router registered in `main.py` (line 34: `app.include_router(auth_router)`)

2. **Frontend Auth Pages**:
   - Auth pages exist but not tested yet
   - AuthContext.tsx configured with backend URL

3. **Database**:
   - `user_sessions` table exists
   - May need `user_profiles` table (to verify)

### What's Missing ❌
1. **Critical Missing Endpoint**: `/api/auth/get-session`
   - Frontend calls this (404 error)
   - Not defined in `backend/src/api/auth.py`
   - Spec requires it (FR-005)

2. **Auth Service Initialization**:
   - `init_auth_service()` not called in main.py
   - Will cause "Auth service not initialized" error

3. **Database Schema**:
   - `user_profiles` table may not exist in Neon
   - Need migration script

4. **Frontend Integration**:
   - Auth gate not at book start
   - No redirect to signup/login

---

## Architecture Decisions

### Decision 1: Use Existing Backend Auth Code ✅
**Rationale**: Backend auth endpoints already 90% complete. Just need to:
- Add missing `/api/auth/get-session` endpoint
- Initialize auth_service in main.py
- Create user_profiles table

**Trade-offs**:
- ✅ Fast implementation (1-2 hours vs 4-5 hours)
- ✅ Code already reviewed by previous work
- ⚠️ Need to verify it works with Better-Auth

**Alternatives Considered**:
- Rewrite from scratch: Too slow
- Use Better-Auth SDK on backend: Adds dependency

### Decision 2: Simplified Authentication (No JWT, No bcrypt)
**Rationale**: Hackathon context - focus on functionality over security.

**Implementation**:
- Store passwords in plaintext (TEMPORARY - document as tech debt)
- Use simple session tokens (UUID in cookies)
- Better-Auth frontend handles client-side logic

**Trade-offs**:
- ✅ Fast implementation
- ✅ Enough for hackathon demo
- ❌ NOT production-ready (document this clearly)
- ❌ Passwords not hashed (security risk)

**Migration Path** (post-hackathon):
```python
# Add bcrypt hashing:
import bcrypt
hashed = bcrypt.hashpw(password.encode(), bcrypt.gensalt())

# Add JWT tokens:
import jwt
token = jwt.encode({"user_id": user_id}, SECRET_KEY)
```

### Decision 3: Session Management Strategy
**Options**:
A. Store sessions in database (user_sessions table)
B. Store sessions in memory (fast but lost on restart)
C. Use JWT tokens (stateless)

**Selected**: A - Database sessions

**Rationale**:
- Already have user_sessions table
- Persistent across restarts
- Can track session activity

**Implementation**:
```python
# Create session on login:
session_id = uuid4()
await db.create_session(session_id, user_id)

# Verify session on get-session:
session = await db.get_session_by_id(session_id)
if session: return user_profile
```

### Decision 4: Skip "Auth Gate at Book Start" for Now
**Rationale**: Frontend implementation is larger effort. Focus on backend first.

**Phase 1** (This PR):
- ✅ Backend endpoints working
- ✅ Frontend can call endpoints
- ❌ No redirect to auth pages

**Phase 2** (Future):
- Add AuthGate component
- Redirect unauthenticated users

---

## Data Flow

### Signup Flow
```
Frontend: POST /api/auth/signup
  Body: { email, password, name, hardware_type }
  ↓
Backend: auth.py:signup()
  → Validate input
  → Create user_id (UUID)
  → Store in user_profiles table
  → Create session token
  → Return { user, session_token }
  ↓
Frontend: Store session_token in cookie
  → Redirect to book homepage
```

### Login Flow
```
Frontend: POST /api/auth/login
  Body: { email, password }
  ↓
Backend: auth.py:login()
  → Query user_profiles by email
  → Validate password (plaintext match)
  → Create session token
  → Return { user, session_token }
  ↓
Frontend: Store session_token in cookie
  → Redirect to book homepage
```

### Get Session Flow (MISSING - KEY IMPLEMENTATION)
```
Frontend: GET /api/auth/get-session
  Headers: { Cookie: session_token=xxx }
  ↓
Backend: auth.py:get_session() ← NEW ENDPOINT
  → Extract session_token from cookie
  → Query user_sessions by session_id
  → Get user_id from session
  → Query user_profiles by user_id
  → Return { user, hardware_profile }
  ↓
Frontend: Update AuthContext state
```

### Logout Flow
```
Frontend: POST /api/auth/logout
  Headers: { Cookie: session_token=xxx }
  ↓
Backend: auth.py:logout()
  → Extract session_token from cookie
  → Delete session from user_sessions
  → Return { success: true }
  ↓
Frontend: Clear cookie
  → Redirect to /auth/login
```

---

## Database Schema Changes

### 1. Create `user_profiles` Table

```sql
CREATE TABLE IF NOT EXISTS user_profiles (
  user_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password VARCHAR(255) NOT NULL,  -- PLAINTEXT (temporary)
  name VARCHAR(255),
  hardware_type VARCHAR(50),  -- 'gpu_workstation' | 'edge_device' | 'cloud_mac'
  hardware_details JSONB,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_user_profiles_email ON user_profiles(email);
```

### 2. Update `user_sessions` Table

```sql
-- Add user_id foreign key if not exists
ALTER TABLE user_sessions
ADD COLUMN IF NOT EXISTS user_id UUID REFERENCES user_profiles(user_id);

CREATE INDEX IF NOT EXISTS idx_user_sessions_user_id ON user_sessions(user_id);
```

---

## API Endpoints Implementation

### NEW: GET /api/auth/get-session
**Status**: MISSING (404 error currently)

**Implementation**:
```python
@router.get("/get-session")
async def get_session(request: Request):
    """
    Get current user session from cookie.
    Returns user profile + hardware details.
    """
    # Extract session_token from cookie
    session_token = request.cookies.get("session_token")

    if not session_token:
        raise HTTPException(401, "Not authenticated")

    # Get session from database
    session = await db_service.get_session(UUID(session_token))

    if not session or not session.user_id:
        raise HTTPException(401, "Invalid session")

    # Get user profile
    auth_service = get_auth_service()
    user_profile = await auth_service.get_user_profile(session.user_id)

    if not user_profile:
        raise HTTPException(404, "User not found")

    return {
        "user": {
            "id": user_profile.user_id,
            "email": user_profile.email,
            "name": user_profile.name
        },
        "hardware_profile": {
            "type": user_profile.hardware_type,
            "details": user_profile.hardware_details
        }
    }
```

### UPDATE: POST /api/auth/signup
**Current**: Creates user_profile but doesn't create session

**Fix Needed**:
```python
# After creating user_profile:
user_profile = await auth_service.create_user_profile(profile_data)

# ✅ ADD: Create session
session = await db_service.create_session(
    UserSessionCreate(user_id=user_profile.user_id)
)

# ✅ ADD: Set session cookie
response = JSONResponse(...)
response.set_cookie(
    key="session_token",
    value=str(session.session_id),
    httponly=True,
    secure=True,  # HTTPS only in production
    samesite="lax"
)
return response
```

### UPDATE: POST /api/auth/login
**Current**: Stub implementation (doesn't validate)

**Fix Needed**:
```python
@router.post("/login")
async def login(request: Request, email: str, password: str):
    # ✅ Query user by email
    auth_service = get_auth_service()
    user_profile = await auth_service.get_user_by_email(email)

    if not user_profile:
        raise HTTPException(401, "Invalid credentials")

    # ✅ Validate password (plaintext match)
    if user_profile.password != password:
        raise HTTPException(401, "Invalid credentials")

    # ✅ Create session
    session = await db_service.create_session(
        UserSessionCreate(user_id=user_profile.user_id)
    )

    # ✅ Return with session cookie
    response = JSONResponse({
        "success": True,
        "user": {
            "id": user_profile.user_id,
            "email": user_profile.email,
            "name": user_profile.name
        }
    })
    response.set_cookie(
        key="session_token",
        value=str(session.session_id),
        httponly=True
    )
    return response
```

---

## File Changes Required

### 1. Backend Database Migration
**New File**: `backend/migrations/003_create_user_profiles.sql`
```sql
-- Create user_profiles table
-- Add user_id to user_sessions
```

### 2. Backend API Endpoint
**File**: `backend/src/api/auth.py`

**Changes**:
- ✅ Line 15-90: signup() - Add session creation + cookie
- ✅ Line 92-120: login() - Implement validation + session
- ✅ NEW: get_session() - Add new endpoint (~40 lines)
- ✅ Line 167-182: logout() - Add session deletion

### 3. Backend Auth Service
**File**: `backend/src/services/auth_service.py`

**Changes**:
- ✅ NEW: get_user_by_email() method (~30 lines)
- ✅ Line 56-127: create_user_profile() - Add password field

### 4. Backend Main
**File**: `backend/src/main.py`

**Changes**:
- ✅ Add startup event to initialize auth_service:
```python
@app.on_event("startup")
async def startup_event():
    await db_service.connect()
    init_auth_service(db_service.pool)  # ← ADD THIS
```

Wait, serverless doesn't support startup events! Use lazy init instead:
```python
# In auth.py endpoints, call:
if not auth_service:
    init_auth_service(db_service.pool)
```

### 5. Backend Models
**File**: `backend/src/models/user_profile.py` (may need to create)

---

## Testing Strategy

### Phase 1: Local Testing (Unit)
```bash
# Test database migration
python backend/migrations/run_migrations.py

# Test signup endpoint
curl -X POST http://localhost:8000/api/auth/signup \
  -d '{"email":"test@example.com", "password":"test123", "name":"Test User", "hardware_type":"gpu_workstation"}'

# Expected: 200 OK, user created, session cookie set

# Test login endpoint
curl -X POST http://localhost:8000/api/auth/login \
  -d '{"email":"test@example.com", "password":"test123"}'

# Expected: 200 OK, session cookie set

# Test get-session endpoint
curl -X GET http://localhost:8000/api/auth/get-session \
  -H "Cookie: session_token=xxx"

# Expected: 200 OK, user profile + hardware
```

### Phase 2: Integration Testing
- Signup → Login → Get Session → Logout (full flow)
- Invalid credentials → 401 error
- Expired session → 401 error
- Guest mode → No session cookie

### Phase 3: Production Testing
- Deploy to Vercel
- Test with live frontend
- Verify 404 error on /api/auth/get-session is fixed

---

## Risks & Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Plaintext passwords | HIGH | CERTAIN | Document as tech debt, add TODO for bcrypt |
| Session hijacking | MEDIUM | MEDIUM | Use httpOnly cookies, HTTPS only |
| user_profiles table doesn't exist | HIGH | HIGH | Create migration script first |
| Better-Auth incompatibility | MEDIUM | LOW | Test with frontend early |
| Serverless auth_service init fails | MEDIUM | MEDIUM | Use lazy initialization pattern |

---

## Out of Scope (This Phase)

- ❌ Auth gate at book start (frontend redirect)
- ❌ Password hashing (bcrypt)
- ❌ JWT tokens
- ❌ OAuth providers (Google, GitHub)
- ❌ Email verification
- ❌ Password reset
- ❌ MFA (Multi-factor authentication)
- ❌ Rate limiting
- ❌ CAPTCHA

---

## Success Criteria

### Must Have (P0)
- ✅ `/api/auth/get-session` endpoint returns 200 (fixes 404 error)
- ✅ Signup creates user in database + session cookie
- ✅ Login validates credentials + creates session cookie
- ✅ Logout deletes session
- ✅ All endpoints tested locally with curl

### Should Have (P1)
- ✅ user_profiles table created in Neon
- ✅ Production deployment successful
- ✅ Frontend can call all auth endpoints

### Nice to Have (P2)
- ⚠️ Auth gate at book start (deferred)
- ⚠️ Guest mode button (deferred)
- ⚠️ Password hashing (deferred)

---

## Implementation Estimate

**Total Time**: 2-3 hours (revised from 4-5 hours)

**Breakdown**:
- Database migration: 0.5 hours
- Add `/api/auth/get-session` endpoint: 0.5 hours
- Fix signup (add session creation): 0.25 hours
- Fix login (add validation): 0.5 hours
- Test locally: 0.5 hours
- Deploy to production: 0.25 hours
- Test with frontend: 0.25 hours
- Create PHR: 0.25 hours

**Why Faster**: Backend auth code already 90% complete!

---

## Next Steps After Completion

1. **Immediate**:
   - Test text selection feature (Feature 006)
   - Verify both features work together

2. **Phase 2** (Future):
   - Implement auth gate at book start
   - Add password hashing (bcrypt)
   - Add JWT tokens for stateless auth

3. **Bonus Points**:
   - Personalization System (Feature 005) - 50 points
   - Requires auth working first!

---

**Approved**: Pending user confirmation
**Blocked By**: None (can start immediately)
**Blocks**: Personalization System (Feature 005)
**Points**: 50 (Constitution compliance)
