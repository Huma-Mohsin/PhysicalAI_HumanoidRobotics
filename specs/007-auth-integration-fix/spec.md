# Feature Specification: Better-Auth Integration at Book Start

**Feature Branch**: `007-auth-integration-fix`
**Created**: 2025-12-18
**Status**: Draft
**Priority**: P1 (Constitution Principle IV - 50 pts, currently misplaced)
**Effort**: Medium (Auth pages exist, need relocation + backend endpoints)

## User Scenarios & Testing

### User Story 1 - Optional Signup via Navbar Menu (Priority: P1) 🎯 MVP

As a visitor to the Physical AI book, I want to freely read the content without forced signup, with an OPTIONAL "Login" button in the navbar menu for those who want personalization features.

**Why this priority**: Lower barrier to entry - users can explore content first, then decide to create account for personalization. Constitution requires "Auth & Survey (50 pts)" but doesn't mandate forced signup.

**Independent Test**: Visit https://humanoidrobotbook.vercel.app/ → Can read book immediately → Click "Login" in navbar → Sign up/login → Navbar now shows user email + logout.

**Acceptance Scenarios**:

1. **Given** I am a new visitor (not logged in), **When** I visit the book homepage, **Then** I can read content freely AND see "Login" + "Sign Up" buttons in navbar
2. **Given** I want to personalize my experience, **When** I click "Sign Up" in navbar, **Then** I am taken to signup page with hardware survey questions
3. **Given** I completed signup, **When** I return to the book, **Then** navbar shows my email (e.g., "alice@test.com") with dropdown for logout

---

### User Story 2 - Hardware Survey Integration (Priority: P1) 🎯 MVP

As a user signing up, I want to provide my hardware details during registration, so that the book can personalize content for my setup without asking again.

**Why this priority**: This connects Better-Auth with personalization system (future feature 005). Captures hardware profile at signup.

**Independent Test**: Go to signup page → Fill form → Select "RTX 4090 Workstation" from hardware dropdown → Submit → Profile saved to backend → Can query via API.

**Acceptance Scenarios**:

1. **Given** I am filling out the signup form, **When** I see hardware options, **Then** I can select from: GPU Workstation (RTX 4070 Ti+), Edge Device (Jetson Orin), Cloud/Mac
2. **Given** I selected "Edge Device" and submitted signup, **When** my account is created, **Then** my hardware profile is saved to Neon database (user_sessions table)
3. **Given** I am logged in, **When** the chatbot queries my profile, **Then** it retrieves my hardware type and uses it for personalized responses

---

### User Story 3 - Backend Auth Endpoints (Priority: P1) 🎯 MVP

As the frontend auth system, I need backend API endpoints for signup, login, logout, and session management, so that user authentication actually works.

**Why this priority**: Currently frontend auth pages exist but backend endpoints are missing (404 errors). This unblocks the entire auth flow.

**Independent Test**: Call `POST /api/auth/signup` with valid data → Receive 201 Created → User stored in database → Can login with same credentials.

**Acceptance Scenarios**:

1. **Given** frontend sends signup request to `/api/auth/signup`, **When** backend processes it, **Then** user is created in Neon database and session token returned
2. **Given** frontend sends login request to `/api/auth/login`, **When** credentials are correct, **Then** backend returns session token and user profile
3. **Given** frontend calls `/api/auth/get-session`, **When** valid session token provided, **Then** backend returns current user's profile and hardware details

---

### User Story 4 - Dynamic Navbar User Menu (Priority: P1) 🎯 MVP

As a user, I want to see my login status in the navbar, with my email displayed when logged in and a logout option, so I know which account I'm using.

**Why this priority**: Core UX requirement - users need visibility into their auth state and easy logout access.

**Independent Test**: Log in as alice@test.com → Navbar shows "alice@test.com" with dropdown icon → Click dropdown → See "Logout" option → Click logout → Navbar reverts to "Login" + "Sign Up" buttons.

**Acceptance Scenarios**:

1. **Given** I am NOT logged in, **When** I visit the book, **Then** navbar shows "Login" and "Sign Up" buttons on the right
2. **Given** I am logged in as "alice@test.com", **When** I view the navbar, **Then** it displays my email "alice@test.com" with a dropdown arrow
3. **Given** I click on my email dropdown, **When** the menu opens, **Then** I see "Logout" option
4. **Given** I click "Logout", **When** session is cleared, **Then** navbar reverts to showing "Login" + "Sign Up" buttons

---

### Edge Cases

- What if user is already logged in but visits /auth/signup?
  - **Expected**: Redirect to book homepage (already authenticated)

- What if backend auth API is down during signup?
  - **Expected**: Show error message "Authentication service unavailable. Please try again later."

- What if user provides invalid hardware profile data?
  - **Expected**: Form validation shows error "Please select a valid hardware type"

- What if user logs out mid-session?
  - **Expected**: Redirect to /auth/login, preserve current page for redirect after login

---

## Requirements

### Functional Requirements

- **FR-001**: Navbar MUST show "Login" + "Sign Up" buttons for unauthenticated users
- **FR-002**: Signup form MUST capture: email, password, name, hardware_type (dropdown with: GPU Workstation, Edge Device, Cloud/Mac)
- **FR-003**: Backend `/api/auth/signup` MUST create user in Neon database with hardware profile
- **FR-004**: Backend `/api/auth/login` MUST validate credentials and return session token
- **FR-005**: Backend `/api/auth/get-session` MUST return current user's profile for authenticated requests
- **FR-006**: Backend `/api/auth/logout` MUST invalidate session token
- **FR-007**: System MUST store session token in httpOnly cookie (secure)
- **FR-008**: Navbar MUST show user email with dropdown menu when authenticated
- **FR-009**: User dropdown menu MUST contain "Logout" option
- **FR-010**: Book content MUST be freely accessible without authentication (no auth wall)
- **FR-011**: Personalization features (future) MAY prompt user to sign up if not authenticated

### Key Entities

**User** (Better-Auth schema, already partially defined):
```typescript
{
  id: string;                // UUID
  email: string;
  name: string;
  password_hash: string;     // bcrypt
  created_at: Date;
  updated_at: Date;
}
```

**UserSession** (existing in backend, connects to User):
```typescript
{
  session_id: UUID;
  user_id?: string;          // Links to Better-Auth user
  hardware_profile: HardwareProfile;
  created_at: Date;
  last_active: Date;
}
```

**HardwareProfile** (existing):
```typescript
{
  type: 'gpu_workstation' | 'edge_device' | 'cloud_mac';
  gpu_model?: string;
  cpu_model?: string;
  ram_size?: number;
  os_type?: string;
}
```

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: 100% of unauthenticated visitors are redirected to auth pages before reading
- **SC-002**: Signup flow completes successfully with hardware profile saved to database
- **SC-003**: Backend auth endpoints (`/api/auth/*`) return 200/201 responses (not 404)
- **SC-004**: Authenticated users' hardware profiles are queryable via RAG chatbot
- **SC-005**: "Continue as Guest" option available with <3 second load time

---

## Technical Implementation (High-Level)

### Frontend Changes

**Files to Modify**:
1. `humanoid_robot_book/docusaurus.config.ts` - Replace static navbar items with dynamic UserMenu
2. `humanoid_robot_book/src/pages/signup.tsx` - Add hardware dropdown to form
3. `humanoid_robot_book/src/pages/login.tsx` - Connect to backend (already done!)
4. `humanoid_robot_book/src/contexts/AuthContext.tsx` - Fix baseURL (already done!)

**New Components**:
- `src/theme/Navbar/Content/index.tsx` - Swizzled Docusaurus navbar with dynamic auth menu
- `src/components/Auth/UserMenu.tsx` - User dropdown component (email + logout)

### Backend Changes (NEW)

**Files to Create**:
1. `backend/src/api/auth.py` - Auth router (partially exists, needs endpoints)
2. `backend/src/services/auth_service.py` - Business logic (exists, needs implementation)
3. `backend/src/models/user.py` - User model (if not using Better-Auth schema)

**Endpoints to Implement**:
```python
POST /api/auth/signup
  Body: { email, password, name, hardware_type }
  Returns: { user, session_token }

POST /api/auth/login
  Body: { email, password }
  Returns: { user, session_token }

GET /api/auth/get-session
  Headers: { Cookie: session_token }
  Returns: { user, hardware_profile }

POST /api/auth/logout
  Headers: { Cookie: session_token }
  Returns: { success: true }

PUT /api/auth/update-profile
  Body: { hardware_type, ... }
  Returns: { updated_profile }
```

### Database Schema (Already Exists in Neon)

**Tables**:
- ✅ `user_sessions` - Already exists
- ⚠️ `users` - May need to create (Better-Auth table)

**Migration Needed**:
```sql
-- Create users table if not exists
CREATE TABLE IF NOT EXISTS users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  name VARCHAR(255) NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

-- Add user_id to user_sessions (may already exist)
ALTER TABLE user_sessions
ADD COLUMN user_id UUID REFERENCES users(id);
```

### Data Flow

```
User visits book → AuthGate checks localStorage/session
  → No auth? Redirect to /auth/signup

User fills signup form:
  - Email: user@example.com
  - Password: ********
  - Name: John Doe
  - Hardware: GPU Workstation (RTX 4090)

Frontend → POST /api/auth/signup
Backend:
  → Hash password with bcrypt
  → Create user in users table
  → Create user_session with hardware_profile
  → Return session token (httpOnly cookie)

Frontend stores session, redirects to book
User clicks "Personalize" → RAG chatbot queries user_id
Backend retrieves hardware_profile from user_sessions
Chatbot generates hardware-aware response
```

---

## Constitution Compliance

**Gamification Line 71-72**: ✅ "Auth & Survey (50 pts): Better-Auth with hardware/software background survey fully functional"
- This feature unlocks 50 points when complete

**Principle III Line 59**: ✅ "User onboarding captures: (a) Hardware profile, (b) Software environment"
- Signup form captures hardware type

---

## Dependencies

- ✅ Better-Auth library installed
- ✅ Frontend auth pages exist (src/pages/auth/)
- ✅ Backend user_sessions table exists
- ⚠️ Backend auth endpoints NOT implemented (main gap)
- ⚠️ Users table may not exist in Neon

---

## Out of Scope (This Feature)

- OAuth providers (Google, GitHub)
- Email verification
- Password reset flow
- Multi-factor authentication (MFA)
- User profile pages (basic settings only)

---

## Risks & Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Better-Auth complexity | HIGH | MEDIUM | Use simple bcrypt + sessions, skip Advanced Better-Auth features |
| Database migration fails | HIGH | LOW | Test migrations locally before production |
| Session security issues | MEDIUM | MEDIUM | Use httpOnly cookies, secure flags in production |
| Guest mode conflicts with personalization | LOW | MEDIUM | Clear messaging: "Sign up to unlock personalization" |

---

## Implementation Effort

**Estimated Time**: 4-5 hours

**Breakdown**:
- Backend auth endpoints: 2 hours
  - POST /api/auth/signup
  - POST /api/auth/login
  - GET /api/auth/get-session
  - POST /api/auth/logout
- Database migration: 0.5 hours
- Frontend auth gate: 1 hour
  - AuthGate component
  - Redirect logic
- Frontend form updates: 1 hour
  - Hardware dropdown
  - Error handling
- Testing: 0.5 hours

---

## Next Steps

1. **Planning**: Create plan in `specs/007-auth-integration-fix/plan.md`
2. **Tasks**: Break down into tasks in `specs/007-auth-integration-fix/tasks.md`
3. **Implementation**: Backend endpoints first, then frontend integration
4. **Testing**: Manual testing of full signup/login flow
5. **PHR**: Document in `history/prompts/007-auth-integration-fix/`

---

**Approved**: Pending user confirmation
**Blocked By**: None (can start immediately)
**Blocks**: Personalization system (feature 005) - needs user profiles
**Effort**: Medium (backend endpoints main work)
