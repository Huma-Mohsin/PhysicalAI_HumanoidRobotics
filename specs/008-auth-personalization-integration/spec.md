# Feature 008: Auth + Personalization Integration

**Status**: In Progress
**Priority**: P0 (Critical Fix)
**Created**: 2025-12-19
**Dependencies**: Feature 007 (Better-Auth), Feature 005 (Personalization)

---

## Problem Statement

**Current Issue**: Authentication (Feature 007) and Personalization (Feature 005) are implemented separately:
- Feature 007: Basic signup/login with only email + password
- Feature 005: Hardware survey modal appears AFTER visiting the book
- **Missing**: Software background collection
- **Missing**: Integration of background questions into signup flow

**User's Original Requirement** (from initial prompt):
> "Signup ke waqt user se unka software aur hardware background puchna hai taake dashboard personalized ho."

This was NOT implemented correctly. Background data should be collected **during signup**, not after.

---

## User Stories

### US-1: Comprehensive Signup Flow (P0 - Critical)
**As a** new user
**I want to** provide my software and hardware background during signup
**So that** I get personalized content from the first login

**Acceptance Criteria**:
- [ ] Signup form includes Software Background section
  - Experience level dropdown (Beginner, Intermediate, Expert)
  - Programming languages multi-select (Python, JavaScript, C++, ROS, etc.)
- [ ] Signup form includes Hardware Background section
  - Hardware type selection (GPU Workstation, Jetson Orin Nano, Cloud/Mac)
  - Hardware experience checkbox (Yes/No)
- [ ] All background data saves to database during signup
- [ ] User can skip background questions (optional fields)
- [ ] After signup, user is logged in and redirected to personalized dashboard

### US-2: Personalized Dashboard (P0 - Critical)
**As a** logged-in user
**I want to** see content tailored to my background
**So that** I learn efficiently without seeing irrelevant content

**Acceptance Criteria**:
- [ ] Software beginners see beginner-friendly tutorials first
- [ ] Users with Python experience see Python-based examples
- [ ] GPU workstation users see Isaac Sim installation guides
- [ ] Jetson users see edge deployment guides (no Isaac Sim)
- [ ] Cloud/Mac users see Docker/cloud setup guides

### US-3: Profile Update (P1)
**As a** logged-in user
**I want to** update my background information
**So that** I can get updated personalized content

**Acceptance Criteria**:
- [ ] Settings page shows current background info
- [ ] User can edit software/hardware background
- [ ] Changes save to database and reflect immediately

---

## Functional Requirements

### FR-001: Signup Form Enhancement
**Priority**: P0 (Critical)

**Current State**: Signup form has:
```tsx
- Email input
- Password input
- Submit button
```

**Required State**: Signup form should have:
```tsx
1. Account Details
   - Email (required)
   - Password (required)

2. Software Background (optional)
   - Experience Level: Dropdown
     Options: ["Beginner", "Intermediate", "Expert"]
   - Programming Languages: Multi-select checkboxes
     Options: ["Python", "JavaScript", "C++", "ROS 2", "None"]

3. Hardware Background (optional)
   - Hardware Type: Radio buttons
     Options: ["GPU Workstation (RTX 4090+)", "Jetson Orin Nano", "Cloud/Mac"]
   - Hardware Experience: Checkbox
     "I have experience with robotics hardware"
```

### FR-002: Database Schema Update
**Priority**: P0 (Critical)

**Existing Schema** (`user_profiles` table):
```sql
CREATE TABLE user_profiles (
  user_id VARCHAR PRIMARY KEY,
  email VARCHAR UNIQUE,
  name VARCHAR,
  password VARCHAR,  -- TECH DEBT: plaintext
  created_at TIMESTAMP,
  updated_at TIMESTAMP
);
```

**Required Schema** (add columns):
```sql
ALTER TABLE user_profiles ADD COLUMN software_experience VARCHAR;  -- 'beginner', 'intermediate', 'expert'
ALTER TABLE user_profiles ADD COLUMN programming_languages JSONB; -- ['Python', 'JavaScript', 'C++', 'ROS 2']
ALTER TABLE user_profiles ADD COLUMN hardware_type VARCHAR;       -- 'gpu_workstation', 'edge_device', 'cloud_mac'
ALTER TABLE user_profiles ADD COLUMN hardware_experience BOOLEAN; -- true/false
```

### FR-003: Backend API Update
**Priority**: P0 (Critical)

**Update** `POST /api/auth/signup`:

**Current Request**:
```json
{
  "email": "user@example.com",
  "password": "securepass123"
}
```

**Required Request**:
```json
{
  "email": "user@example.com",
  "password": "securepass123",
  "softwareExperience": "intermediate",
  "programmingLanguages": ["Python", "JavaScript"],
  "hardwareType": "gpu_workstation",
  "hardwareExperience": true
}
```

**Response** (unchanged):
```json
{
  "success": true,
  "user": {
    "id": "...",
    "email": "user@example.com",
    "softwareExperience": "intermediate",
    "programmingLanguages": ["Python", "JavaScript"],
    "hardwareType": "gpu_workstation",
    "hardwareExperience": true
  }
}
```

### FR-004: Remove HardwareSurveyModal
**Priority**: P0 (Critical)

**Current Behavior**: Modal appears 1 second after visiting the book (even for logged-in users).

**Required Behavior**:
- If user is logged in AND has `hardwareType` in profile: **NO MODAL**
- If user is logged in but has NO `hardwareType`: Show modal with message "Please update your profile in Settings"
- If user is NOT logged in: Show modal (for anonymous users)

### FR-005: Personalized Content Rendering
**Priority**: P0 (Critical)

**Logic**:
```tsx
// In ContentVariant component
const { user } = useAuth();  // Get logged-in user

// Check hardware type
const userHardwareType = user?.hardwareType || profile?.type || 'cloud_mac';

// Check software experience
const userSoftwareLevel = user?.softwareExperience || 'beginner';

// Render appropriate content
if (userSoftwareLevel === 'beginner') {
  // Show beginner-friendly explanations
}

if (userHardwareType === 'gpu_workstation') {
  // Show Isaac Sim local installation
} else if (userHardwareType === 'edge_device') {
  // Show Gazebo + Jetson deployment
}
```

---

## Non-Functional Requirements

### NFR-001: Data Privacy
- Background information is non-sensitive (skill level, hardware type)
- No PII beyond email
- User can skip all background questions during signup

### NFR-002: Performance
- Signup API response < 1 second
- Database query for user profile < 100ms
- No noticeable delay when rendering personalized content

### NFR-003: Compatibility
- Works for both authenticated and anonymous users
- Backward compatible: existing users without background data see default content

---

## Technical Architecture

### Data Flow: Signup to Personalization

```
┌─────────────────────────────────────────────────────────┐
│ 1. User Visits /signup                                   │
│    - Sees comprehensive signup form                      │
│    - Fills email, password, software/hardware background │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│ 2. POST /api/auth/signup                                │
│    - Validates email/password                            │
│    - Creates user in database with background fields     │
│    - Creates session (httpOnly cookie)                   │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│ 3. Redirect to Dashboard                                │
│    - AuthContext fetches user session                    │
│    - User object includes: softwareExperience,           │
│      programmingLanguages, hardwareType                  │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│ 4. Personalized Content Rendering                       │
│    - Check user.softwareExperience → show beginner/adv  │
│    - Check user.hardwareType → show GPU/Jetson/Cloud    │
│    - ContentVariant components filter based on profile   │
└─────────────────────────────────────────────────────────┘
```

---

## Migration Strategy

### Phase 1: Database Migration
1. Create migration script `004_add_background_fields.sql`
2. Add columns to `user_profiles` table
3. Set default values for existing users (`null` - shows all content)

### Phase 2: Backend Update
1. Update `SignupRequest` Pydantic model
2. Modify `/api/auth/signup` endpoint to accept new fields
3. Update `user_profiles` INSERT query

### Phase 3: Frontend Update
1. Update Signup page with background questions
2. Update AuthContext to include background fields in user object
3. Modify HardwareSurveyModal logic (skip for users with data)

### Phase 4: Testing
1. Test signup with full background data
2. Test signup with partial data (some fields empty)
3. Test signup with no background data (skip all)
4. Test personalized content rendering
5. Test backward compatibility (existing users)

---

## Rollback Plan

If integration breaks:
1. **Quick Fix**: Make all background fields nullable and optional
2. **Fallback**: Revert to separate modal (Feature 005 behavior)
3. **Database**: Background columns are nullable, no data loss

---

## Success Metrics

- [ ] 80%+ of new users provide at least one background field
- [ ] Users see personalized content on first login
- [ ] No 404 errors or auth failures
- [ ] Page load time < 2 seconds

---

## References

- Feature 007: Better-Auth Integration (Basic signup/login)
- Feature 005: Personalization System (Content variants)
- User's Original Requirement: Urdu prompt with software/hardware background collection

---

**Status**: Ready for implementation
**Next Step**: Create `plan.md` and `tasks.md`
