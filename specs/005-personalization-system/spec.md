# Feature Specification: Interactive Personalization System

**Feature Branch**: `005-personalization-system`
**Created**: 2025-12-18
**Status**: Draft
**Priority**: P0 (Critical - Constitution Principle III violation)
**Points**: 50 (Gamification requirement)

## User Scenarios & Testing

### User Story 1 - Hardware Profile Capture (Priority: P1) 🎯 MVP

As a new book reader, I want to provide my hardware details when I first visit the site, so that I get personalized content recommendations tailored to my setup.

**Why this priority**: Without hardware profile, personalization cannot work. This is the foundation.

**Independent Test**: Visit book for first time → Hardware survey modal appears → Select "RTX 4090 Workstation" → Modal closes → Profile saved → Can proceed to reading.

**Acceptance Scenarios**:

1. **Given** I visit the book for the first time (no profile saved), **When** the page loads, **Then** I see a hardware survey modal with 3 options: GPU Workstation, Edge Device, Cloud/Mac
2. **Given** I select "GPU Workstation (RTX 4070 Ti+)" and click Continue, **When** the modal closes, **Then** my selection is saved to localStorage and Better-Auth session (if logged in)
3. **Given** I have already provided my hardware profile, **When** I visit the book again, **Then** the survey modal does not appear

---

### User Story 2 - Personalize Button at Chapter Start (Priority: P1) 🎯 MVP

As a reader with a specific hardware setup, I want to see a "Personalize" button at the start of each chapter, so that I can view content optimized for my hardware.

**Why this priority**: This is the core UI element mandated by constitution. Must be visible and functional.

**Independent Test**: Navigate to Module 3 (Isaac Sim chapter) → Click "Personalize" button → Content dynamically updates to show RTX 4090-specific instructions → GPU-specific tips highlighted.

**Acceptance Scenarios**:

1. **Given** I have selected "GPU Workstation" profile and I'm reading Module 3 (Isaac Sim), **When** I click the "Personalize" button, **Then** the content shows RTX GPU-specific installation steps and local Isaac Sim setup
2. **Given** I have selected "Edge Device (Jetson)" profile and I'm reading Module 3, **When** I click "Personalize", **Then** the content shows Gazebo alternatives and warns that Isaac Sim is not supported on Jetson
3. **Given** I have selected "Cloud/Mac" profile and I'm reading Module 3, **When** I click "Personalize", **Then** the content shows Omniverse Cloud links and Gazebo fallback options

---

### User Story 3 - Content Variant Rendering (Priority: P1) 🎯 MVP

As a reader who clicked "Personalize", I want to see hardware-specific content blocks that replace generic instructions, so that I don't waste time on irrelevant setup steps.

**Why this priority**: The actual value delivery - showing relevant content based on hardware.

**Independent Test**: Set profile to "Jetson Orin Nano" → Navigate to any chapter with hardware-specific sections → Click "Personalize" → Verify Jetson-specific commands appear, GPU installation hidden.

**Acceptance Scenarios**:

1. **Given** I have "GPU Workstation" profile and personalized content is active, **When** I read installation instructions, **Then** I see CUDA toolkit installation steps and Isaac Sim local setup
2. **Given** I have "Edge Device" profile and personalized content is active, **When** I read installation instructions, **Then** I see Jetson-specific apt packages and Gazebo setup (no Isaac Sim)
3. **Given** I have "Cloud/Mac" profile and personalized content is active, **When** I read installation instructions, **Then** I see Docker-based alternatives and cloud simulation links

---

### User Story 4 - RAG Chatbot Profile Integration (Priority: P2)

As a user with a hardware profile, I want the chatbot to automatically know my setup, so that it gives me hardware-appropriate answers without me repeating my configuration.

**Why this priority**: Extends personalization to the RAG chatbot, increasing value.

**Independent Test**: Set profile to "Jetson Orin Nano" → Ask chatbot "How do I run Isaac Sim?" → Verify response includes "Your profile indicates Jetson Orin Nano—Isaac Sim is not supported. Use Gazebo instead."

**Acceptance Scenarios**:

1. **Given** I have "GPU Workstation" profile, **When** I ask the chatbot "What GPU do I need?", **Then** the response confirms I already have suitable hardware and suggests optimizations
2. **Given** I have "Edge Device" profile, **When** I ask about simulation, **Then** the chatbot recommends Gazebo and warns about Isaac Sim limitations
3. **Given** I have "Cloud/Mac" profile, **When** I ask about local setup, **Then** the chatbot suggests cloud-based or Docker alternatives

---

### Edge Cases

- What if user skips hardware survey modal?
  - **Expected**: Show default "Cloud/Mac" content (lowest common denominator per constitution)
  - Provide "Set Hardware Profile" link in footer to re-open modal

- What if user wants to change hardware profile later?
  - **Expected**: Settings page with "Update Hardware Profile" button
  - Re-opens modal, updates localStorage and Better-Auth session

- What if content doesn't have hardware-specific variants?
  - **Expected**: "Personalize" button disabled or hidden for generic chapters
  - Show message: "This chapter has universal content"

- What if user has GPU profile but content shows Cloud/Mac instructions?
  - **Expected**: Bug - personalization logic failed
  - Fallback: Show all variants with clear labels ("For GPU:", "For Jetson:", "For Cloud:")

---

## Requirements

### Functional Requirements

- **FR-001**: System MUST display hardware survey modal on first visit (before authentication)
- **FR-002**: Survey MUST offer exactly 3 options: GPU Workstation (RTX 4070 Ti+), Edge Device (Jetson Orin), Cloud/Mac
- **FR-003**: User selection MUST be stored in localStorage (immediate persistence) and Better-Auth session (if logged in)
- **FR-004**: "Personalize" button MUST appear at the start of chapters that have hardware-specific content
- **FR-005**: Clicking "Personalize" MUST toggle content variants based on saved hardware profile
- **FR-006**: Default (unpersonalized) content MUST work on Cloud/Mac (constitution requirement)
- **FR-007**: RAG chatbot queries MUST include user hardware profile as context (user_id lookup → profile)
- **FR-008**: System MUST provide UI to update hardware profile after initial selection
- **FR-009**: Personalized content MUST clearly indicate which hardware it targets (labels/badges)
- **FR-010**: System MUST gracefully handle missing hardware profile (show default Cloud/Mac content)

### Key Entities

**HardwareProfile** (already exists in backend):
```typescript
{
  userId?: string;           // Better-Auth user ID (optional for anonymous)
  hardwareType: 'gpu_workstation' | 'edge_device' | 'cloud_mac';
  gpuModel?: string;         // e.g., "RTX 4090"
  cpuModel?: string;
  ramSize?: number;          // GB
  osType?: string;           // e.g., "Ubuntu 22.04"
  additionalNotes?: string;
  createdAt: Date;
  updatedAt: Date;
}
```

**ContentVariant** (new frontend concept):
```typescript
{
  hardwareType: 'gpu' | 'edge' | 'cloud';
  content: string;           // MDX content block
  label: string;             // e.g., "For RTX 4090 Users"
}
```

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: 100% of first-time visitors see hardware survey modal within 2 seconds of page load
- **SC-002**: Hardware profile selection persists across browser sessions with 99%+ reliability
- **SC-003**: "Personalize" button appears on at least 3 chapters (Module 2, 3, 4 minimum)
- **SC-004**: Content variant switching completes within 500ms of button click
- **SC-005**: RAG chatbot includes hardware context in 100% of queries from profiled users
- **SC-006**: Users can update hardware profile with ≤3 clicks from any page

---

## Technical Architecture (High-Level)

### Frontend Components (Docusaurus)

1. **HardwareSurveyModal.tsx**
   - Shown on first visit (check localStorage)
   - 3 radio buttons: GPU / Edge / Cloud
   - "Continue" button saves selection

2. **PersonalizeButton.tsx**
   - Displays at chapter start (MDX import)
   - Reads hardware profile from context
   - Toggles content variant visibility

3. **HardwareProfileContext.tsx**
   - Provides global state for hardware profile
   - Syncs localStorage ↔ Better-Auth session
   - Exposes `updateProfile()` method

4. **ContentVariant.tsx**
   - Wrapper component for hardware-specific MDX blocks
   - Shows/hides based on active profile
   - Props: `hardwareType`, `children`

### Backend Integration

- Existing `HardwareProfile` model in `backend/src/models/user_session.py`
- Better-Auth session stores `hardwareType` (small string, not full object)
- RAG chatbot reads profile from `user_id` → `user_sessions` table → injects into prompt

### Data Flow

```
User visits → Check localStorage → No profile?
  → Show HardwareSurveyModal
  → User selects "GPU Workstation"
  → Save to localStorage
  → If logged in: Save to Better-Auth session via API

User clicks "Personalize"
  → Read profile from HardwareProfileContext
  → Toggle CSS classes on ContentVariant components
  → Show only matching hardware variants

User asks chatbot
  → Frontend sends user_id (if logged in)
  → Backend looks up hardware_profile from user_sessions
  → Injects into RAG prompt: "User has RTX 4090 workstation..."
  → LLM generates hardware-aware response
```

---

## Constitution Compliance

### Principle III: Interactive Personalization

**Line 54**: ✅ "User onboarding captures hardware profile" - Satisfied by HardwareSurveyModal
**Line 60**: ✅ "'Personalize' button at chapter start" - Satisfied by PersonalizeButton component
**Line 61**: ✅ "RAG chatbot injects hardware context" - Satisfied by profile lookup in RAG service
**Line 62**: ✅ "Default content works on Cloud/Mac" - Satisfied by showing Cloud/Mac variant when unpersonalized

### Gamification (50 points)

**Dynamic Content (50 pts)**: "Personalize" button operational at chapter start
- This feature unlocks all 50 points

---

## Dependencies

- ✅ Better-Auth pages exist (not integrated yet - will integrate in Phase 3)
- ✅ Backend `HardwareProfile` model exists
- ✅ Frontend auth context exists (needs minor updates)
- ⚠️ Content needs hardware-specific MDX variants (manual content work)

---

## Out of Scope (This Feature)

- **Content Creation**: Writing hardware-specific variants for all chapters (done separately)
- **Better-Auth Integration**: Moving auth to book start (separate feature 006)
- **Advanced Profiles**: Detailed GPU models, multiple devices per user
- **Profile Sharing**: Exporting/importing hardware profiles

---

## Risks & Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Content variants not ready | HIGH | MEDIUM | Start with 3 chapters (Module 2, 3, 4), expand later |
| localStorage cleared by user | MEDIUM | LOW | Re-show survey modal, easy re-selection |
| Better-Auth not integrated | MEDIUM | HIGH | Use localStorage-only mode first, add auth sync later |
| Complexity overwhelming users | MEDIUM | MEDIUM | Simple 3-option modal, clear labels |

---

## Next Steps

1. **Planning**: Create architectural plan in `specs/005-personalization-system/plan.md`
2. **Tasks**: Break down into tasks in `specs/005-personalization-system/tasks.md`
3. **Implementation**: Build components following SpecKit Plus workflow
4. **Content**: Add hardware-specific variants to at least 3 chapters
5. **Testing**: Manual testing on all 3 hardware profiles
6. **PHR**: Document implementation in `history/prompts/005-personalization-system/`

---

**Approved**: Pending user confirmation
**Blocked By**: None (can start immediately)
**Blocks**: Better-Auth integration (feature 006), Advanced personalization features
