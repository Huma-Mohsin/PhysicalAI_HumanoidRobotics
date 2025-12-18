# Implementation Tasks: Better-Auth Integration

**Feature**: 007-auth-integration-fix
**Created**: 2025-12-18
**Status**: Ready for Implementation
**Est. Time**: 2-3 hours

---

## Task Breakdown

### T1: Database Schema - Create user_profiles Table ⏱️ 30min

**Priority**: P0 (Blocking)
**Dependencies**: None
**Files**: `backend/migrations/003_create_user_profiles.sql` (NEW)

**Description**: Create user_profiles table in Neon database to store user credentials and hardware profiles.

**Implementation Steps**:
1. Create migration file `003_create_user_profiles.sql`
2. Add SQL to create `user_profiles` table
3. Add index on email for fast lookups
4. Add `user_id` column to `user_sessions` table (if not exists)
5. Run migration script locally
6. Verify table exists in Neon dashboard

**SQL to Write**:
```sql
CREATE TABLE IF NOT EXISTS user_profiles (
  user_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password VARCHAR(255) NOT NULL,
  name VARCHAR(255),
  hardware_type VARCHAR(50),
  hardware_details JSONB,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_user_profiles_email ON user_profiles(email);

ALTER TABLE user_sessions
ADD COLUMN IF NOT EXISTS user_id UUID REFERENCES user_profiles(user_id);

CREATE INDEX IF NOT EXISTS idx_user_sessions_user_id ON user_sessions(user_id);
```

**Acceptance Criteria**:
- [ ] Migration file exists at `backend/migrations/003_create_user_profiles.sql`
- [ ] Running migration locally succeeds (no errors)
- [ ] Table `user_profiles` visible in Neon dashboard
- [ ] Can insert test row: `INSERT INTO user_profiles (email, password, name) VALUES ('test@test.com', 'test123', 'Test User')`
- [ ] Email uniqueness enforced (duplicate email → error)

**Test Command**:
```bash
cd backend
python migrations/run_migrations.py migrate
# Expected: "Migration 003_create_user_profiles.sql applied successfully"
```

---

### T2: Auth Service - Add get_user_by_email() Method ⏱️ 20min

**Priority**: P0 (Blocking for login)
**Dependencies**: T1 (user_profiles table must exist)
**Files**: `backend/src/services/auth_service.py`

**Description**: Add method to query user by email for login validation.

**Implementation**:
```python
async def get_user_by_email(self, email: str) -> Optional[UserProfile]:
    """Retrieve a user profile by email address."""
    try:
        async with self.db_pool.acquire() as conn:
            query = """
                SELECT user_id, email, password, name, hardware_type,
                       hardware_details, created_at, updated_at
                FROM user_profiles
                WHERE email = $1
            """
            result = await conn.fetchrow(query, email)

            if not result:
                return None

            # Create HardwareDetails object
            hardware_details = None
            if result['hardware_details']:
                hardware_details = HardwareDetails(
                    gpu_model=result['hardware_details'].get('gpu_model'),
                    cpu_model=result['hardware_details'].get('cpu_model'),
                    ram_size=result['hardware_details'].get('ram_size'),
                    os_type=result['hardware_details'].get('os_type'),
                    additional_notes=result['hardware_details'].get('additional_notes')
                )

            return UserProfile(
                user_id=result['user_id'],
                email=result['email'],
                password=result['password'],  # INCLUDE password for login validation
                name=result['name'],
                hardware_type=result['hardware_type'],
                hardware_details=hardware_details,
                created_at=result['created_at'],
                updated_at=result['updated_at']
            )
    except Exception as e:
        logger.error(f"Error retrieving user by email: {str(e)}")
        raise
```

**Acceptance Criteria**:
- [ ] Method `get_user_by_email()` added to AuthService class (line ~270)
- [ ] Returns UserProfile if email exists
- [ ] Returns None if email not found
- [ ] Password field included in returned UserProfile
- [ ] No errors when user doesn't exist

**Test Command**:
```python
# In Python shell:
from backend.src.services.auth_service import get_auth_service
auth = get_auth_service()
user = await auth.get_user_by_email("test@test.com")
print(user.email, user.password)  # Should print email and password
```

---

### T3: Auth Service - Update UserProfile Model with Password ⏱️ 10min

**Priority**: P0
**Dependencies**: None
**Files**: `backend/src/services/auth_service.py`

**Description**: Add password field to UserProfile model so it can be used for login validation.

**Implementation**:
```python
class UserProfile(BaseModel):
    """User profile with hardware information."""
    user_id: str
    email: str
    password: str  # ← ADD THIS FIELD
    name: Optional[str] = None
    hardware_type: Optional[str] = None
    hardware_details: Optional[HardwareDetails] = None
    created_at: datetime
    updated_at: datetime
```

**Acceptance Criteria**:
- [ ] `password: str` field added to UserProfile (line ~19)
- [ ] UserProfileCreate also has password field
- [ ] No Pydantic validation errors

---

### T4: API Endpoint - Add GET /api/auth/get-session ⏱️ 30min

**Priority**: P0 (Fixes 404 error)
**Dependencies**: T1, T2
**Files**: `backend/src/api/auth.py`

**Description**: Add missing `/api/auth/get-session` endpoint to return current user from session cookie.

**Implementation**:
```python
@router.get("/get-session")
async def get_session(request: Request):
    """
    Get current user session from cookie.
    Returns user profile + hardware details.
    """
    try:
        # Extract session_token from cookie
        session_token = request.cookies.get("session_token")

        if not session_token:
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "No session token provided"}
            )

        # Parse UUID
        try:
            session_id = UUID(session_token)
        except ValueError:
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "Invalid session token"}
            )

        # Get session from database
        from src.services.database_service import db_service
        await db_service.ensure_connected()
        session = await db_service.get_session(session_id)

        if not session or not session.user_id:
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "Session not found or not authenticated"}
            )

        # Get user profile
        auth_service = get_auth_service()
        user_profile = await auth_service.get_user_profile(session.user_id)

        if not user_profile:
            raise HTTPException(
                status_code=404,
                detail={"error": "NotFound", "message": "User profile not found"}
            )

        # Return user + hardware profile (do NOT include password)
        return JSONResponse(
            status_code=200,
            content={
                "success": True,
                "user": {
                    "id": user_profile.user_id,
                    "email": user_profile.email,
                    "name": user_profile.name
                },
                "hardware_profile": {
                    "type": user_profile.hardware_type,
                    "details": user_profile.hardware_details.dict() if user_profile.hardware_details else None
                }
            }
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get session error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={"error": "InternalServerError", "message": "Failed to get session"}
        )
```

**Acceptance Criteria**:
- [ ] Endpoint `GET /api/auth/get-session` added to router (line ~183)
- [ ] Returns 401 if no session_token cookie
- [ ] Returns 401 if invalid session_token
- [ ] Returns 404 if user not found
- [ ] Returns 200 with user + hardware_profile if valid
- [ ] Password NOT included in response

**Test Command**:
```bash
# Without cookie (should fail):
curl -X GET http://localhost:8000/api/auth/get-session
# Expected: 401 {"error": "Unauthorized"}

# With valid cookie:
curl -X GET http://localhost:8000/api/auth/get-session \
  -H "Cookie: session_token=xxx-valid-session-xxx"
# Expected: 200 {"user": {...}, "hardware_profile": {...}}
```

---

### T5: API Endpoint - Fix POST /api/auth/signup (Add Session Creation) ⏱️ 15min

**Priority**: P0
**Dependencies**: T1
**Files**: `backend/src/api/auth.py`

**Description**: Update signup endpoint to create session and set cookie after user creation.

**Changes to Make**:
```python
@router.post("/signup")
async def signup(
    request: Request,
    email: str,
    password: str,
    name: str,
    hardware_type: Optional[str] = None,
    hardware_details: Optional[dict] = None
):
    try:
        auth_service = get_auth_service()

        # Generate user_id
        import uuid
        user_id = str(uuid.uuid4())

        # Create user profile
        profile_data = UserProfileCreate(
            user_id=user_id,
            email=email,
            password=password,  # ← PLAINTEXT (document as tech debt)
            name=name,
            hardware_type=hardware_type,
            hardware_details=hardware_details
        )

        user_profile = await auth_service.create_user_profile(profile_data)

        # ✅ ADD: Create session
        from src.services.database_service import db_service
        from src.models.user_session import UserSessionCreate
        await db_service.ensure_connected()

        session = await db_service.create_session(
            UserSessionCreate(user_id=user_profile.user_id)
        )

        # ✅ ADD: Return response with session cookie
        response = JSONResponse(
            status_code=201,
            content={
                "success": True,
                "message": "User created successfully",
                "user": {
                    "id": user_profile.user_id,
                    "email": user_profile.email,
                    "name": user_profile.name
                }
            }
        )

        # Set httpOnly cookie
        response.set_cookie(
            key="session_token",
            value=str(session.session_id),
            httponly=True,
            secure=False,  # Set to True in production (HTTPS)
            samesite="lax",
            max_age=30 * 24 * 60 * 60  # 30 days
        )

        return response
    except Exception as e:
        logger.error(f"Signup error: {str(e)}")
        raise HTTPException(status_code=400, detail=f"Signup failed: {str(e)}")
```

**Acceptance Criteria**:
- [ ] Signup creates user in user_profiles table
- [ ] Signup creates session in user_sessions table
- [ ] Response includes Set-Cookie header with session_token
- [ ] Cookie has httponly=True, samesite=lax
- [ ] Returns 201 status (not 200)

**Test Command**:
```bash
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"newuser@test.com", "password":"pass123", "name":"New User", "hardware_type":"gpu_workstation"}' \
  -v

# Expected:
# - Status: 201 Created
# - Header: Set-Cookie: session_token=xxx; HttpOnly; SameSite=Lax
# - Body: {"success": true, "user": {...}}
```

---

### T6: API Endpoint - Fix POST /api/auth/login (Add Validation) ⏱️ 20min

**Priority**: P0
**Dependencies**: T2 (get_user_by_email)
**Files**: `backend/src/api/auth.py`

**Description**: Implement real login validation instead of stub.

**Changes to Make**:
```python
@router.post("/login")
async def login(
    request: Request,
    email: str,
    password: str
):
    try:
        auth_service = get_auth_service()

        # ✅ Query user by email
        user_profile = await auth_service.get_user_by_email(email)

        if not user_profile:
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "Invalid email or password"}
            )

        # ✅ Validate password (plaintext comparison)
        if user_profile.password != password:
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "Invalid email or password"}
            )

        # ✅ Create session
        from src.services.database_service import db_service
        from src.models.user_session import UserSessionCreate
        await db_service.ensure_connected()

        session = await db_service.create_session(
            UserSessionCreate(user_id=user_profile.user_id)
        )

        # ✅ Return with session cookie
        response = JSONResponse(
            status_code=200,
            content={
                "success": True,
                "message": "Login successful",
                "user": {
                    "id": user_profile.user_id,
                    "email": user_profile.email,
                    "name": user_profile.name
                }
            }
        )

        response.set_cookie(
            key="session_token",
            value=str(session.session_id),
            httponly=True,
            secure=False,  # True in production
            samesite="lax",
            max_age=30 * 24 * 60 * 60  # 30 days
        )

        return response
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Login error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={"error": "InternalServerError", "message": "Login failed"}
        )
```

**Acceptance Criteria**:
- [ ] Login validates email exists (401 if not)
- [ ] Login validates password matches (401 if wrong)
- [ ] Creates session on successful login
- [ ] Sets session_token cookie
- [ ] Returns user profile without password

**Test Command**:
```bash
# Valid login:
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@test.com", "password":"test123"}' \
  -v
# Expected: 200 OK, Set-Cookie header

# Invalid password:
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@test.com", "password":"wrong"}' \
  -v
# Expected: 401 Unauthorized
```

---

### T7: API Endpoint - Fix POST /api/auth/logout (Delete Session) ⏱️ 15min

**Priority**: P1
**Dependencies**: T4 (get-session logic)
**Files**: `backend/src/api/auth.py`

**Description**: Delete session from database on logout.

**Changes to Make**:
```python
@router.post("/logout")
async def logout(request: Request):
    try:
        # Extract session_token from cookie
        session_token = request.cookies.get("session_token")

        if session_token:
            try:
                session_id = UUID(session_token)

                # Delete session from database
                from src.services.database_service import db_service
                await db_service.ensure_connected()
                await db_service.delete_session(session_id)

                logger.info(f"Session {session_id} deleted successfully")
            except ValueError:
                # Invalid UUID, just clear cookie
                pass
            except Exception as e:
                logger.error(f"Error deleting session: {str(e)}")
                # Continue to clear cookie even if delete fails

        # Return response with cleared cookie
        response = JSONResponse(
            status_code=200,
            content={
                "success": True,
                "message": "Logout successful"
            }
        )

        # Clear session cookie
        response.delete_cookie(key="session_token")

        return response
    except Exception as e:
        logger.error(f"Logout error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={"error": "InternalServerError", "message": "Logout failed"}
        )
```

**Acceptance Criteria**:
- [ ] Logout deletes session from user_sessions table
- [ ] Clears session_token cookie
- [ ] Returns 200 even if no session exists
- [ ] Subsequent get-session calls return 401

**Test Command**:
```bash
# Logout:
curl -X POST http://localhost:8000/api/auth/logout \
  -H "Cookie: session_token=xxx" \
  -v
# Expected: 200 OK, Set-Cookie: session_token=; Max-Age=0

# Try get-session after logout:
curl -X GET http://localhost:8000/api/auth/get-session \
  -H "Cookie: session_token=xxx"
# Expected: 401 Unauthorized
```

---

### T8: Database Service - Add delete_session() Method ⏱️ 10min

**Priority**: P1 (For logout)
**Dependencies**: None
**Files**: `backend/src/services/database_service.py`

**Description**: Add method to delete session from user_sessions table.

**Implementation**:
```python
async def delete_session(self, session_id: UUID) -> bool:
    """Delete a session from the database."""
    await self.ensure_connected()

    try:
        query = """
            DELETE FROM user_sessions
            WHERE session_id = $1
            RETURNING session_id
        """

        result = await self.pool.fetchrow(query, session_id)
        return result is not None
    except Exception as e:
        logger.error(f"Error deleting session {session_id}: {str(e)}")
        raise
```

**Acceptance Criteria**:
- [ ] Method `delete_session()` added to DatabaseService
- [ ] Returns True if session deleted
- [ ] Returns False if session not found
- [ ] No errors on non-existent session

---

### T9: Integration Test - Full Auth Flow ⏱️ 20min

**Priority**: P0
**Dependencies**: T1-T8 (all previous tasks)
**Files**: None (manual testing)

**Description**: Test complete auth flow: signup → login → get-session → logout.

**Test Scenarios**:

**Scenario 1: New User Signup Flow**
```bash
# Step 1: Signup
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"alice@test.com", "password":"alice123", "name":"Alice", "hardware_type":"edge_device"}' \
  -c cookies.txt -v

# Expected: 201, Set-Cookie, user created

# Step 2: Get session (using saved cookie)
curl -X GET http://localhost:8000/api/auth/get-session \
  -b cookies.txt

# Expected: 200, user profile + hardware_type: "edge_device"

# Step 3: Logout
curl -X POST http://localhost:8000/api/auth/logout \
  -b cookies.txt

# Expected: 200, cookie cleared

# Step 4: Try get-session after logout
curl -X GET http://localhost:8000/api/auth/get-session \
  -b cookies.txt

# Expected: 401 Unauthorized
```

**Scenario 2: Returning User Login Flow**
```bash
# Step 1: Login
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"alice@test.com", "password":"alice123"}' \
  -c cookies2.txt -v

# Expected: 200, Set-Cookie, login successful

# Step 2: Get session
curl -X GET http://localhost:8000/api/auth/get-session \
  -b cookies2.txt

# Expected: 200, same user profile as before
```

**Scenario 3: Invalid Credentials**
```bash
# Wrong password:
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"alice@test.com", "password":"wrong"}'

# Expected: 401 Unauthorized

# Non-existent user:
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"nobody@test.com", "password":"anything"}'

# Expected: 401 Unauthorized
```

**Acceptance Criteria**:
- [ ] Signup → Get-session works (user logged in immediately)
- [ ] Login → Get-session works (returning user)
- [ ] Logout → Get-session fails (session invalidated)
- [ ] Invalid credentials → 401 errors
- [ ] Duplicate email signup → Error (unique constraint)

---

### T10: Production Deployment ⏱️ 15min

**Priority**: P0
**Dependencies**: T9 (local testing passes)
**Files**: None (Vercel deployment)

**Description**: Deploy backend to Vercel and verify auth endpoints work in production.

**Steps**:
1. Commit changes to git:
   ```bash
   git add backend/
   git commit -m "feat: Implement Better-Auth backend endpoints"
   ```

2. Deploy to Vercel:
   ```bash
   cd backend
   vercel --prod --yes
   ```

3. Verify production URLs:
   ```bash
   curl https://humanoid-robotics-backend.vercel.app/api/auth/get-session
   # Expected: 401 (no session - correct!)
   ```

**Acceptance Criteria**:
- [ ] Backend deployed successfully to Vercel
- [ ] All auth endpoints accessible at production URL
- [ ] `/api/auth/get-session` returns 401 (not 404!)
- [ ] Signup creates user in Neon production database
- [ ] Login works with production backend

---

### T11: Frontend Verification ⏱️ 10min

**Priority**: P1
**Dependencies**: T10 (production deployment)
**Files**: None (manual testing)

**Description**: Verify frontend can call production auth endpoints without 404 errors.

**Test Steps**:
1. Open browser DevTools (F12)
2. Visit https://humanoidrobotbook.vercel.app/
3. Check Console for errors
4. Look for `/api/auth/get-session` request in Network tab

**Acceptance Criteria**:
- [ ] No 404 errors in Console
- [ ] `/api/auth/get-session` returns 401 (not 404)
- [ ] Frontend AuthContext receives response

---

## Summary

**Total Tasks**: 11
**Estimated Time**: 2-3 hours
**P0 Tasks**: 9 (must complete)
**P1 Tasks**: 2 (should complete)

**Critical Path**:
T1 (DB) → T2 (Auth Service) → T4 (get-session) → T5 (signup fix) → T6 (login fix) → T9 (integration test) → T10 (deploy)

**Completion Criteria**:
- ✅ All P0 tasks completed
- ✅ Integration test passes (T9)
- ✅ Production deployment successful (T10)
- ✅ Frontend 404 error fixed (T11)

**Next Steps After Completion**:
1. Create PHR documenting implementation
2. Update constitution compliance tracker
3. Test Feature 006 (Text Selection) + Feature 007 together
4. Optional: Implement auth gate at book start (Phase 2)
