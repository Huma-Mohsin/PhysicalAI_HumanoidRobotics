# Feature 008: Auth + Personalization Integration - Tasks

**Feature**: 008-auth-personalization-integration
**Status**: In Progress
**Total Estimated Time**: 6 hours

---

## Phase 1: Database Migration (1 hour)

### T1: Create Database Migration Script ⏱️ 30 min
**Priority**: P0 (Critical)
**Dependencies**: None

**Implementation**:
```sql
-- backend/database/migrations/004_add_background_fields.sql

-- Add software background fields
ALTER TABLE user_profiles ADD COLUMN IF NOT EXISTS software_experience VARCHAR(20);
ALTER TABLE user_profiles ADD COLUMN IF NOT EXISTS programming_languages JSONB;

-- Add hardware background fields
ALTER TABLE user_profiles ADD COLUMN IF NOT EXISTS hardware_type VARCHAR(30);
ALTER TABLE user_profiles ADD COLUMN IF NOT EXISTS hardware_experience BOOLEAN DEFAULT false;

-- Add indexes for performance
CREATE INDEX IF NOT EXISTS idx_user_profiles_hardware_type ON user_profiles(hardware_type);
CREATE INDEX IF NOT EXISTS idx_user_profiles_software_experience ON user_profiles(software_experience);

-- Set default values for existing users (null = see all content)
-- No UPDATE needed, nullable columns handle this
```

**Acceptance Criteria**:
- [ ] Migration script created
- [ ] Runs without errors on local database
- [ ] Existing user data unchanged
- [ ] New columns accept NULL values

**Test Cases**:
```bash
# Test migration
psql -U postgres -d humanoid_robotics -f backend/database/migrations/004_add_background_fields.sql

# Verify columns added
psql -U postgres -d humanoid_robotics -c "\d user_profiles"

# Check existing users still exist
psql -U postgres -d humanoid_robotics -c "SELECT email, software_experience FROM user_profiles;"
```

---

### T2: Update Database Models ⏱️ 30 min
**Priority**: P0 (Critical)
**Dependencies**: T1

**Implementation**:
```python
# backend/src/models/user.py

from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime

class UserProfile(BaseModel):
    user_id: str
    email: str
    name: Optional[str] = None
    password: str  # TECH DEBT: plaintext

    # NEW: Software background
    software_experience: Optional[str] = None  # 'beginner', 'intermediate', 'expert'
    programming_languages: Optional[List[str]] = None  # ['Python', 'JavaScript', etc.]

    # NEW: Hardware background
    hardware_type: Optional[str] = None  # 'gpu_workstation', 'edge_device', 'cloud_mac'
    hardware_experience: Optional[bool] = False

    created_at: datetime
    updated_at: datetime
```

**Acceptance Criteria**:
- [ ] UserProfile model updated
- [ ] All new fields optional (don't break existing code)
- [ ] Type hints correct
- [ ] Default values set

---

## Phase 2: Backend API Update (2 hours)

### T3: Update Signup Request Model ⏱️ 20 min
**Priority**: P0 (Critical)
**Dependencies**: T2

**Implementation**:
```python
# backend/src/api/auth.py

class SignupRequest(BaseModel):
    email: str
    password: str

    # NEW: Optional background fields
    software_experience: Optional[str] = None
    programming_languages: Optional[List[str]] = None
    hardware_type: Optional[str] = None
    hardware_experience: Optional[bool] = False

    @validator('software_experience')
    def validate_software_experience(cls, v):
        if v and v not in ['beginner', 'intermediate', 'expert']:
            raise ValueError('Invalid software experience level')
        return v

    @validator('hardware_type')
    def validate_hardware_type(cls, v):
        if v and v not in ['gpu_workstation', 'edge_device', 'cloud_mac']:
            raise ValueError('Invalid hardware type')
        return v
```

**Acceptance Criteria**:
- [ ] SignupRequest accepts new fields
- [ ] Validation works for invalid values
- [ ] Fields are optional (can be None)

---

### T4: Update Signup Endpoint Logic ⏱️ 40 min
**Priority**: P0 (Critical)
**Dependencies**: T3

**Implementation**:
```python
# backend/src/api/auth.py

@router.post("/api/auth/signup")
async def signup(request: Request, data: SignupRequest):
    """Create new user with optional background information."""

    # Check if user exists
    existing_user = await db_service.get_user_by_email(data.email)
    if existing_user:
        raise HTTPException(status_code=400, detail="Email already registered")

    # Create user with background data
    user_id = str(uuid.uuid4())

    await db_service.create_user(
        user_id=user_id,
        email=data.email,
        password=data.password,  # TECH DEBT: should hash
        software_experience=data.software_experience,
        programming_languages=data.programming_languages,
        hardware_type=data.hardware_type,
        hardware_experience=data.hardware_experience
    )

    # Create session
    session_id = await auth_service.create_session(user_id)

    # Set cookie
    response = JSONResponse({
        "success": True,
        "user": {
            "id": user_id,
            "email": data.email,
            "softwareExperience": data.software_experience,
            "programmingLanguages": data.programming_languages,
            "hardwareType": data.hardware_type,
            "hardwareExperience": data.hardware_experience
        }
    })

    response.set_cookie(
        key="session_token",
        value=str(session_id),
        httponly=True,
        max_age=7 * 24 * 60 * 60,  # 7 days
        samesite="lax"
    )

    return response
```

**Acceptance Criteria**:
- [ ] Signup creates user with background fields
- [ ] Session created after signup
- [ ] Cookie set correctly
- [ ] Response includes background data

**Test Cases**:
```bash
# Test full signup
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "pass123",
    "softwareExperience": "intermediate",
    "programmingLanguages": ["Python", "JavaScript"],
    "hardwareType": "gpu_workstation",
    "hardwareExperience": true
  }'

# Test partial signup (only email/password)
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "minimal@example.com",
    "password": "pass123"
  }'
```

---

### T5: Update Database Service ⏱️ 30 min
**Priority**: P0 (Critical)
**Dependencies**: T2

**Implementation**:
```python
# backend/src/services/database_service.py

async def create_user(
    self,
    user_id: str,
    email: str,
    password: str,
    software_experience: Optional[str] = None,
    programming_languages: Optional[List[str]] = None,
    hardware_type: Optional[str] = None,
    hardware_experience: Optional[bool] = False
):
    """Create new user with background information."""

    query = """
        INSERT INTO user_profiles (
            user_id, email, password,
            software_experience, programming_languages,
            hardware_type, hardware_experience,
            created_at, updated_at
        )
        VALUES ($1, $2, $3, $4, $5, $6, $7, NOW(), NOW())
        RETURNING *
    """

    # Convert list to JSONB
    languages_json = json.dumps(programming_languages) if programming_languages else None

    result = await self.pool.fetchrow(
        query,
        user_id, email, password,
        software_experience, languages_json,
        hardware_type, hardware_experience
    )

    return result
```

**Acceptance Criteria**:
- [ ] create_user() accepts new parameters
- [ ] JSONB conversion works for programming_languages
- [ ] Existing calls still work (default values)

---

### T6: Update Get Session Endpoint ⏱️ 30 min
**Priority**: P0 (Critical)
**Dependencies**: T2

**Implementation**:
```python
# backend/src/api/auth.py

@router.get("/api/auth/get-session")
async def get_session(request: Request):
    """Get current user session with background data."""

    session_token = request.cookies.get("session_token")
    if not session_token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    try:
        session = await db_service.get_session(UUID(session_token))
        if not session:
            raise HTTPException(status_code=401, detail="Invalid session")

        user = await db_service.get_user_by_id(session.user_id)

        return {
            "user": {
                "id": user.user_id,
                "email": user.email,
                "name": user.name,
                # NEW: Include background data
                "softwareExperience": user.software_experience,
                "programmingLanguages": json.loads(user.programming_languages) if user.programming_languages else [],
                "hardwareType": user.hardware_type,
                "hardwareExperience": user.hardware_experience
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

**Acceptance Criteria**:
- [ ] Returns background data in session response
- [ ] JSONB properly deserialized to array
- [ ] Works for users without background data (returns null/empty)

---

## Phase 3: Frontend Update (2.5 hours)

### T7: Update Signup Page UI ⏱️ 1 hour
**Priority**: P0 (Critical)
**Dependencies**: None

**Implementation**:
```tsx
// humanoid_robot_book/src/pages/signup.tsx

import React, { useState } from 'react';

export default function SignupPage() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  // NEW: Background state
  const [softwareExperience, setSoftwareExperience] = useState('');
  const [programmingLanguages, setProgrammingLanguages] = useState<string[]>([]);
  const [hardwareType, setHardwareType] = useState('');
  const [hardwareExperience, setHardwareExperience] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    const response = await fetch('/api/auth/signup', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        email,
        password,
        softwareExperience,
        programmingLanguages,
        hardwareType,
        hardwareExperience
      })
    });

    if (response.ok) {
      window.location.href = '/';
    }
  };

  return (
    <form onSubmit={handleSubmit}>
      <h1>Create Account</h1>

      {/* Section 1: Account Details */}
      <section>
        <h2>Account Details</h2>
        <input
          type="email"
          placeholder="Email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
        />
        <input
          type="password"
          placeholder="Password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
        />
      </section>

      {/* Section 2: Software Background (Optional) */}
      <section>
        <h2>Software Background (Optional)</h2>
        <label>Experience Level:</label>
        <select
          value={softwareExperience}
          onChange={(e) => setSoftwareExperience(e.target.value)}
        >
          <option value="">Select...</option>
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="expert">Expert</option>
        </select>

        <label>Programming Languages:</label>
        {['Python', 'JavaScript', 'C++', 'ROS 2', 'None'].map((lang) => (
          <label key={lang}>
            <input
              type="checkbox"
              checked={programmingLanguages.includes(lang)}
              onChange={(e) => {
                if (e.target.checked) {
                  setProgrammingLanguages([...programmingLanguages, lang]);
                } else {
                  setProgrammingLanguages(programmingLanguages.filter(l => l !== lang));
                }
              }}
            />
            {lang}
          </label>
        ))}
      </section>

      {/* Section 3: Hardware Background (Optional) */}
      <section>
        <h2>Hardware Background (Optional)</h2>
        <label>Hardware Type:</label>
        {[
          { value: 'gpu_workstation', label: 'GPU Workstation (RTX 4090+)' },
          { value: 'edge_device', label: 'Jetson Orin Nano' },
          { value: 'cloud_mac', label: 'Cloud / Mac' }
        ].map((hw) => (
          <label key={hw.value}>
            <input
              type="radio"
              name="hardwareType"
              value={hw.value}
              checked={hardwareType === hw.value}
              onChange={(e) => setHardwareType(e.target.value)}
            />
            {hw.label}
          </label>
        ))}

        <label>
          <input
            type="checkbox"
            checked={hardwareExperience}
            onChange={(e) => setHardwareExperience(e.target.checked)}
          />
          I have experience with robotics hardware
        </label>
      </section>

      <button type="submit">Create Account</button>
    </form>
  );
}
```

**Acceptance Criteria**:
- [ ] Form has 3 sections: Account, Software, Hardware
- [ ] All background fields optional
- [ ] Multi-select checkboxes work for languages
- [ ] Radio buttons work for hardware type
- [ ] Form submits all data to backend

---

### T8: Update AuthContext ⏱️ 30 min
**Priority**: P0 (Critical)
**Dependencies**: T6

**Implementation**:
```tsx
// humanoid_robot_book/src/contexts/AuthContext.tsx

export interface User {
  id: string;
  email: string;
  name?: string;
  // NEW: Background fields
  softwareExperience?: string;
  programmingLanguages?: string[];
  hardwareType?: string;
  hardwareExperience?: boolean;
}

// In useEffect, update session fetch to include background data
useEffect(() => {
  const fetchSession = async () => {
    const response = await fetch('/api/auth/get-session', {
      credentials: 'include'
    });

    if (response.ok) {
      const data = await response.json();
      setUser(data.user);  // Now includes background fields
    }
  };

  fetchSession();
}, []);
```

**Acceptance Criteria**:
- [ ] User interface includes background fields
- [ ] AuthContext fetches background data
- [ ] user object available via useAuth() hook

---

### T9: Update HardwareSurveyModal Logic ⏱️ 30 min
**Priority**: P0 (Critical)
**Dependencies**: T8

**Implementation**:
```tsx
// humanoid_robot_book/src/components/Personalization/HardwareSurveyModal.tsx

export default function HardwareSurveyModal() {
  const { user } = useAuth();  // NEW: Check if user logged in
  const { profile, setProfile } = useHardwareProfile();
  const [showModal, setShowModal] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => {
      // NEW LOGIC: Only show if no hardware data exists
      const hasHardwareData = user?.hardwareType || profile?.type;

      if (!hasHardwareData) {
        console.log('[HardwareSurveyModal] No hardware data, showing modal');
        setShowModal(true);
      } else {
        console.log('[HardwareSurveyModal] User has hardware data, skipping modal');
      }
    }, 1000);

    return () => clearTimeout(timer);
  }, [user, profile]);

  // Rest of component unchanged...
}
```

**Acceptance Criteria**:
- [ ] Modal skipped if user has `hardwareType` from signup
- [ ] Modal still shows for anonymous users
- [ ] Modal shows for logged-in users WITHOUT hardware data

---

### T10: Update ContentVariant Component ⏱️ 30 min
**Priority**: P0 (Critical)
**Dependencies**: T8

**Implementation**:
```tsx
// humanoid_robot_book/src/components/Personalization/ContentVariant.tsx

export default function ContentVariant({ hardwareType, children }: ContentVariantProps) {
  const { user } = useAuth();  // NEW: Use auth data first
  const { profile, isPersonalized } = useHardwareProfile();

  // Priority: user.hardwareType (from signup) > profile.type (from modal)
  const userHardwareType = user?.hardwareType || profile?.type;

  const isUserHardware = userHardwareType === hardwareType;

  // Rest of logic unchanged...
}
```

**Acceptance Criteria**:
- [ ] Uses `user.hardwareType` from auth first
- [ ] Falls back to `profile.type` from localStorage
- [ ] Personalization works for signed-up users

---

## Phase 4: Testing (30 min)

### T11: End-to-End Testing ⏱️ 30 min
**Priority**: P0 (Critical)
**Dependencies**: T1-T10

**Test Scenarios**:

**Scenario 1: Full Signup with Background**
1. Visit `/signup`
2. Fill email, password, software (intermediate, Python), hardware (GPU)
3. Submit form
4. Verify redirected to dashboard
5. Verify personalized content shows GPU-specific guides
6. Verify HardwareSurveyModal does NOT appear

**Scenario 2: Minimal Signup (No Background)**
1. Visit `/signup`
2. Fill only email + password
3. Submit form
4. Verify redirected to dashboard
5. Verify HardwareSurveyModal DOES appear after 1 second
6. Fill hardware modal → verify personalization works

**Scenario 3: Existing User Backward Compatibility**
1. Login as existing user (no background data)
2. Verify no errors
3. Verify all content visible (no filtering)
4. Verify HardwareSurveyModal appears

**Scenario 4: Update Profile**
1. Login as user with background
2. Go to Settings page
3. Update hardware type
4. Verify personalized content updates

**Acceptance Criteria**:
- [ ] All 4 scenarios pass
- [ ] No console errors
- [ ] No 404/500 API errors
- [ ] Page load < 2 seconds

---

## Summary

| Phase | Tasks | Time | Priority |
|-------|-------|------|----------|
| **Phase 1: Database** | T1-T2 | 1 hour | P0 |
| **Phase 2: Backend** | T3-T6 | 2 hours | P0 |
| **Phase 3: Frontend** | T7-T10 | 2.5 hours | P0 |
| **Phase 4: Testing** | T11 | 30 min | P0 |
| **TOTAL** | 11 tasks | **6 hours** | P0 |

---

**Status**: Ready to implement
**Next Step**: Start with T1 (Database Migration)
