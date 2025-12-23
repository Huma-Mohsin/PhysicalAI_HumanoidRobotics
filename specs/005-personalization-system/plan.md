# Architecture Plan: Interactive Personalization System

**Feature**: 005-personalization-system
**Created**: 2025-12-19
**Tech Stack**: Docusaurus (React), FastAPI (Python), Better-Auth, PostgreSQL/Neon
**Status**: Planning Complete

---

## 1. Scope and Dependencies

### In Scope
- **Hardware Profile Capture**: Modal-based survey on first visit (localStorage + Better-Auth session)
- **Personalize Button**: React component at chapter start (toggles content variants)
- **Content Variant System**: MDX wrapper components that show/hide based on hardware type
- **RAG Chatbot Integration**: Inject user hardware profile into chatbot prompt context
- **Profile Management**: Simple UI to update hardware profile after initial selection

### Out of Scope
- Writing all hardware-specific content variants (separate content authoring task)
- Advanced multi-device profiles (future enhancement)
- Profile import/export functionality
- Better-Auth deep integration (already done in Feature 007)

### External Dependencies
- ✅ Better-Auth integrated (Feature 007 - COMPLETE)
- ✅ Frontend AuthContext exists
- ✅ Backend `user_profiles` table with `hardware_type` column
- ✅ Backend RAG chatbot functional
- ⚠️ Content authors need to write hardware-specific MDX variants (3 chapters minimum)

---

## 2. Key Architectural Decisions

### Decision 1: localStorage + Better-Auth Dual Storage

**Options Considered**:
1. **Option A**: Only Better-Auth session (requires login before personalization)
2. **Option B**: Only localStorage (no persistence across devices)
3. **Option C**: Dual storage - localStorage for immediate persistence, Better-Auth for logged-in sync ✅ **SELECTED**

**Trade-offs**:
- **A Pros**: Centralized, cross-device. **Cons**: Blocks anonymous users (violates constitution)
- **B Pros**: Fast, no auth needed. **Cons**: Lost on browser clear, no cross-device
- **C Pros**: Best UX for both anonymous and logged-in users. **Cons**: Sync logic complexity

**Rationale**: Constitution requires personalization WITHOUT forced authentication. Dual storage allows:
- Anonymous users get immediate personalization via localStorage
- Logged-in users get cross-device sync via Better-Auth session
- Graceful fallback if one storage fails

**Implementation**:
```typescript
// HardwareProfileContext.tsx
const saveProfile = async (profile: HardwareProfile) => {
  // 1. Save to localStorage (immediate)
  localStorage.setItem('hardwareProfile', JSON.stringify(profile));

  // 2. If logged in, sync to Better-Auth session
  if (user?.id) {
    await fetch('/api/profile/update', {
      method: 'POST',
      body: JSON.stringify({ userId: user.id, hardware: profile })
    });
  }
};
```

---

### Decision 2: MDX Component-Based Content Variants

**Options Considered**:
1. **Option A**: Separate MDX files per hardware type (e.g., `module-3-gpu.mdx`, `module-3-jetson.mdx`)
2. **Option B**: Conditional rendering within single MDX file using `<ContentVariant>` wrapper ✅ **SELECTED**
3. **Option C**: Dynamic markdown parsing with custom tags

**Trade-offs**:
- **A Pros**: Simple file structure. **Cons**: Massive duplication, hard to maintain common sections
- **B Pros**: Single source of truth, shared sections automatic. **Cons**: More complex MDX syntax
- **C Pros**: Powerful. **Cons**: Custom parser needed, breaks standard MDX tools

**Rationale**: Single MDX files with variant wrappers reduce duplication and keep related content together.

**Implementation**:
```mdx
<!-- module-3.mdx -->
import { ContentVariant } from '@site/src/components/ContentVariant';
import { PersonalizeButton } from '@site/src/components/PersonalizeButton';

# Module 3: NVIDIA Isaac Sim

<PersonalizeButton />

<ContentVariant hardwareType="gpu">
## Installation for RTX 4090 Users
Download Isaac Sim locally...
</ContentVariant>

<ContentVariant hardwareType="edge">
## Jetson Orin Nano Users
Isaac Sim is not supported. Use Gazebo instead...
</ContentVariant>

<ContentVariant hardwareType="cloud">
## Cloud/Mac Users
Use Omniverse Cloud: https://omniverse.nvidia.com...
</ContentVariant>
```

---

### Decision 3: Toggle-Based Personalization (Not Filter)

**Options Considered**:
1. **Option A**: Filter out irrelevant hardware variants (show only user's hardware)
2. **Option B**: Toggle visibility - show all variants with user's hardware highlighted ✅ **SELECTED**

**Trade-offs**:
- **A Pros**: Clean, focused view. **Cons**: Users can't compare alternatives, hidden content feels like loss
- **B Pros**: Full transparency, users can explore other options. **Cons**: Slightly more visual clutter

**Rationale**: Users might have multiple devices or want to see what's different for other hardware. Toggle allows:
- Default state: All variants visible with labels ("For GPU:", "For Jetson:", etc.)
- Personalized state: User's variant highlighted/expanded, others collapsed
- Users can manually expand collapsed variants if curious

**Implementation**:
```typescript
// PersonalizeButton.tsx
const [personalized, setPersonalized] = useState(false);

// ContentVariant.tsx
const isActive = personalized && variant.hardwareType === userProfile.hardwareType;

return (
  <div className={`variant ${isActive ? 'active' : personalized ? 'dimmed' : ''}`}>
    <h3>{variant.label}</h3>
    {children}
  </div>
);
```

---

### Decision 4: RAG Chatbot Profile Injection at Prompt Level

**Options Considered**:
1. **Option A**: Modify embedding/retrieval to filter hardware-specific chunks
2. **Option B**: Inject hardware profile into system prompt ✅ **SELECTED**
3. **Option C**: Fine-tune LLM on hardware-specific data

**Trade-offs**:
- **A Pros**: Results already filtered. **Cons**: Complex RAG pipeline changes, might miss relevant generic content
- **B Pros**: Simple, works with existing RAG. **Cons**: LLM might ignore context
- **C Pros**: Best performance. **Cons**: Requires training data, cost, complexity

**Rationale**: Prompt injection is fastest to implement and doesn't break existing RAG pipeline.

**Implementation**:
```python
# backend/src/services/rag_service.py
async def generate_response(query: str, user_id: str = None):
    # Look up user hardware profile
    hardware_profile = None
    if user_id:
        session = await db_service.get_session_by_user_id(user_id)
        hardware_profile = session.hardware_profile if session else None

    # Inject into system prompt
    system_prompt = "You are a helpful robotics tutor."
    if hardware_profile:
        system_prompt += f"\n\nIMPORTANT: The user has a {hardware_profile.type} setup. "
        if hardware_profile.type == "edge_device":
            system_prompt += "They are using Jetson Orin Nano. Avoid suggesting NVIDIA Isaac Sim (not supported). Recommend Gazebo instead."
        elif hardware_profile.type == "gpu_workstation":
            system_prompt += f"They have {hardware_profile.details.get('gpu_model', 'RTX GPU')}. You can recommend local Isaac Sim setup."

    # Continue with existing RAG logic...
```

---

## 3. Component Architecture

### Frontend Components (Docusaurus/React)

```
humanoid_robot_book/src/
├── components/
│   └── Personalization/
│       ├── HardwareSurveyModal.tsx       # First-visit modal
│       ├── HardwareSurveyModal.module.css
│       ├── PersonalizeButton.tsx         # Chapter-level toggle
│       ├── PersonalizeButton.module.css
│       ├── ContentVariant.tsx            # MDX wrapper for variants
│       ├── ContentVariant.module.css
│       └── ProfileSettings.tsx           # Update profile UI
├── contexts/
│   └── HardwareProfileContext.tsx        # Global state provider
└── hooks/
    └── useHardwareProfile.ts             # Hook for profile access
```

**Component Hierarchy**:
```
<Root>                                  (Existing, from Root.tsx)
  <AuthProvider>                        (Existing, from AuthContext)
    <HardwareProfileProvider>           (NEW - wraps entire app)
      <HardwareSurveyModal />           (NEW - shown on first visit)
      <DocPage>                         (Existing Docusaurus)
        <MDXContent>
          <PersonalizeButton />         (NEW - user adds to MDX)
          <ContentVariant>              (NEW - user wraps variants)
            Hardware-specific content
          </ContentVariant>
        </MDXContent>
      </DocPage>
    </HardwareProfileProvider>
  </AuthProvider>
</Root>
```

---

### Backend Integration Points

**Existing Endpoints** (no changes needed):
- `GET /api/auth/get-session` - Already returns hardware_profile from user_sessions

**New Endpoints** (minimal):
```python
# backend/src/api/profile.py
@router.post("/api/profile/update")
async def update_hardware_profile(
    request: Request,
    data: HardwareProfileUpdate
):
    """Update user's hardware profile in database."""
    session_token = request.cookies.get("session_token")
    session = await db_service.get_session(UUID(session_token))

    # Update hardware_profile in user_sessions table
    await db_service.update_session_hardware(
        session.session_id,
        data.hardware_type,
        data.hardware_details
    )

    return {"success": True}
```

**RAG Chatbot Modification**:
```python
# backend/src/services/rag_service.py
# Add hardware profile lookup before LLM call (shown in Decision 4)
```

---

## 4. Data Flow Diagrams

### Flow 1: First Visit (Anonymous User)
```
User visits book
  ↓
Check localStorage for 'hardwareProfile'
  ↓ (Not found)
Show HardwareSurveyModal
  ↓
User selects "GPU Workstation (RTX 4090)"
  ↓
Save to localStorage: {type: 'gpu_workstation', gpu_model: 'RTX 4090'}
  ↓
HardwareProfileContext updates
  ↓
User can read book with personalization available
```

### Flow 2: First Visit (Logged-In User)
```
User visits book (already logged in from Feature 007)
  ↓
Check localStorage for 'hardwareProfile'
  ↓ (Not found)
Show HardwareSurveyModal
  ↓
User selects "Edge Device (Jetson Orin)"
  ↓
Save to localStorage: {type: 'edge_device'}
  ↓
POST /api/profile/update (because user logged in)
  ↓
Backend updates user_sessions.hardware_profile
  ↓
Profile synced across devices
```

### Flow 3: Click "Personalize" Button
```
User clicks "Personalize" on Module 3 page
  ↓
PersonalizeButton component reads HardwareProfileContext
  ↓
Profile = {type: 'gpu_workstation'}
  ↓
Set personalized state = true
  ↓
ContentVariant components check:
  - hardwareType="gpu" → SHOW (expanded)
  - hardwareType="edge" → DIM (collapsed)
  - hardwareType="cloud" → DIM (collapsed)
  ↓
User sees GPU-specific content highlighted
```

### Flow 4: RAG Chatbot with Profile
```
User asks: "How do I install Isaac Sim?"
  ↓
Frontend sends: {query, userId} to /api/chat/query
  ↓
Backend looks up user_id in user_sessions table
  ↓
Retrieves hardware_profile = {type: 'edge_device'}
  ↓
Inject into system prompt:
  "User has Jetson Orin. Isaac Sim not supported. Recommend Gazebo."
  ↓
LLM generates response:
  "I see you're using Jetson Orin Nano. Unfortunately, Isaac Sim requires
   RTX GPU and is not supported on Jetson. I recommend using Gazebo instead..."
```

---

## 5. Non-Functional Requirements

### Performance Budgets
- **Hardware Survey Modal**: Must appear within 2 seconds of page load
- **Profile Save**: localStorage write < 50ms, API sync < 500ms
- **Personalize Toggle**: Content re-render < 300ms
- **RAG Profile Lookup**: Database query < 100ms (indexed user_id)

### Reliability
- **localStorage Persistence**: 99%+ reliability (browser-dependent)
- **Sync Success Rate**: 95%+ for logged-in users (network-dependent)
- **Fallback Behavior**: If profile missing, show ALL variants (no personalization)

### Security
- **Profile Data**: Non-sensitive (hardware type only), no PII
- **API Authorization**: Session token validation (existing from Feature 007)
- **XSS Prevention**: All user data sanitized (hardware type is enum, safe)

---

## 6. Testing Strategy

### Unit Tests (Jest/React Testing Library)
- `HardwareSurveyModal.test.tsx`: Modal visibility, selection save
- `PersonalizeButton.test.tsx`: Toggle state management
- `ContentVariant.test.tsx`: Conditional rendering based on profile
- `useHardwareProfile.test.ts`: Hook state and localStorage sync

### Integration Tests
- **Scenario 1**: First visit → survey → profile saved → personalize works
- **Scenario 2**: Logged-in user → survey → profile syncs to backend
- **Scenario 3**: Return visit → no survey shown → profile loaded from localStorage
- **Scenario 4**: RAG chatbot → hardware profile injected → hardware-aware response

### Manual Testing Checklist
- [ ] Visit book without profile → survey modal appears
- [ ] Select GPU Workstation → modal closes → profile saved
- [ ] Navigate to Module 3 → click "Personalize" → GPU content highlighted
- [ ] Clear localStorage → reload → survey appears again
- [ ] Login → set profile → logout → login on different browser → profile restored
- [ ] Ask chatbot about Isaac Sim with Jetson profile → warns not supported

---

## 7. Migration and Rollout

### Phase 1: Foundation (MVP - 3 hours)
- Create HardwareProfileContext
- Create HardwareSurveyModal (localStorage only)
- Add to Root.tsx wrapper
- Test: Survey appears, saves to localStorage

### Phase 2: Personalize Button (2 hours)
- Create PersonalizeButton component
- Create ContentVariant wrapper
- Add to 1 test chapter (Module 3)
- Test: Toggle works, content shows/hides

### Phase 3: Content Variants (Content Team - 4 hours)
- Write hardware-specific variants for Module 2, 3, 4
- Wrap with ContentVariant components
- Test: All variants render correctly

### Phase 4: Backend Sync (1.5 hours)
- Add POST /api/profile/update endpoint
- Modify HardwareProfileContext to sync when logged in
- Test: Profile persists across devices for logged-in users

### Phase 5: RAG Integration (1 hour)
- Modify rag_service.py to inject hardware profile
- Test: Chatbot gives hardware-aware responses

### Phase 6: Settings UI (1 hour)
- Create ProfileSettings page
- Add link in footer
- Test: Users can update profile

**Total Estimated Time**: 12.5 hours (excluding content authoring)

---

## 8. Rollback Plan

### If Personalization Breaks:
1. **Quick Fix**: Hide PersonalizeButton via CSS (emergency)
   ```css
   .personalize-button { display: none !important; }
   ```

2. **Disable Feature**: Comment out HardwareProfileProvider in Root.tsx
   - All variants will show (unpersonalized state)
   - No data loss (localStorage + backend persist)

3. **Rollback Deployment**: Revert to previous Git commit
   - Previous commit: Feature 007 (auth working, no personalization)

### Data Recovery:
- localStorage data: Browser-specific, user can re-set via survey
- Database data: No schema changes, only new hardware_profile column (nullable)

---

## 9. Future Enhancements (Out of Scope)

- **Multi-Device Profiles**: User has both GPU workstation AND Jetson
- **Profile Sharing**: Export/import hardware profiles
- **A/B Testing**: Track which hardware profile has better engagement
- **Advanced Recommendations**: ML-based content suggestions based on profile
- **Profile Verification**: Upload screenshots to verify hardware claims

---

**Approved By**: Pending user confirmation
**Ready to Implement**: Yes (all decisions documented)
**Next Step**: Create tasks.md
