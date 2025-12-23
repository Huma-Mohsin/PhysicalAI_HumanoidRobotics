# Implementation Tasks: Interactive Personalization System

**Feature**: 005-personalization-system
**Created**: 2025-12-19
**Total Estimated Time**: 12.5 hours (excluding content authoring)

---

## Phase 1: Foundation - Hardware Profile Context ⏱️ 3 hours

### T1: Create HardwareProfile TypeScript Types ⏱️ 15 min

**Priority**: P0 (Foundation)
**Dependencies**: None

**Implementation**:
```typescript
// humanoid_robot_book/src/types/hardware.ts
export type HardwareType = 'gpu_workstation' | 'edge_device' | 'cloud_mac';

export interface HardwareProfile {
  type: HardwareType;
  gpuModel?: string;
  cpuModel?: string;
  ramSize?: number;
  osType?: string;
  selectedAt: Date;
}

export interface HardwareProfileContextValue {
  profile: HardwareProfile | null;
  isPersonalized: boolean;
  setProfile: (profile: HardwareProfile) => Promise<void>;
  clearProfile: () => void;
  togglePersonalization: () => void;
}
```

**Acceptance Criteria**:
- [ ] Types compile without errors
- [ ] Exported from `src/types/hardware.ts`
- [ ] HardwareType enum matches backend values

**Testing**:
```bash
cd humanoid_robot_book && npm run typecheck
```

---

### T2: Create HardwareProfileContext Provider ⏱️ 1 hour

**Priority**: P0
**Dependencies**: T1

**Implementation**:
```typescript
// humanoid_robot_book/src/contexts/HardwareProfileContext.tsx
import React, { createContext, useState, useEffect, useContext } from 'react';
import { useAuth } from './AuthContext';
import { HardwareProfile, HardwareProfileContextValue } from '../types/hardware';

const HardwareProfileContext = createContext<HardwareProfileContextValue | null>(null);

const STORAGE_KEY = 'hardwareProfile';

export function HardwareProfileProvider({ children }) {
  const { user } = useAuth();
  const [profile, setProfileState] = useState<HardwareProfile | null>(null);
  const [isPersonalized, setIsPersonalized] = useState(false);

  // Load from localStorage on mount
  useEffect(() => {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (stored) {
      try {
        const parsed = JSON.parse(stored);
        setProfileState(parsed);
      } catch (e) {
        console.error('Failed to parse hardware profile:', e);
      }
    }
  }, []);

  const setProfile = async (newProfile: HardwareProfile) => {
    // 1. Save to state
    setProfileState(newProfile);

    // 2. Save to localStorage
    localStorage.setItem(STORAGE_KEY, JSON.stringify(newProfile));

    // 3. If logged in, sync to backend
    if (user?.id) {
      try {
        await fetch('/api/profile/update', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          credentials: 'include', // Send cookies
          body: JSON.stringify({
            userId: user.id,
            hardwareType: newProfile.type,
            hardwareDetails: {
              gpuModel: newProfile.gpuModel,
              cpuModel: newProfile.cpuModel,
              ramSize: newProfile.ramSize,
              osType: newProfile.osType,
            },
          }),
        });
      } catch (error) {
        console.error('Failed to sync profile to backend:', error);
        // Non-critical: localStorage save succeeded
      }
    }
  };

  const clearProfile = () => {
    setProfileState(null);
    localStorage.removeItem(STORAGE_KEY);
  };

  const togglePersonalization = () => {
    setIsPersonalized(prev => !prev);
  };

  return (
    <HardwareProfileContext.Provider
      value={{ profile, isPersonalized, setProfile, clearProfile, togglePersonalization }}
    >
      {children}
    </HardwareProfileContext.Provider>
  );
}

export function useHardwareProfile() {
  const context = useContext(HardwareProfileContext);
  if (!context) {
    throw new Error('useHardwareProfile must be used within HardwareProfileProvider');
  }
  return context;
}
```

**Acceptance Criteria**:
- [ ] Context provider wraps app without errors
- [ ] Profile loads from localStorage on mount
- [ ] setProfile saves to localStorage
- [ ] setProfile syncs to backend if user logged in
- [ ] useHardwareProfile hook works in components

**Testing**:
```typescript
// Test in browser console
localStorage.setItem('hardwareProfile', JSON.stringify({type: 'gpu_workstation'}));
// Reload page → context should load profile
```

---

### T3: Integrate HardwareProfileProvider in Root.tsx ⏱️ 15 min

**Priority**: P0
**Dependencies**: T2

**Implementation**:
```typescript
// humanoid_robot_book/src/theme/Root.tsx
import { HardwareProfileProvider } from '../contexts/HardwareProfileContext';

export default function Root({ children }): JSX.Element {
  return (
    <AuthProvider>
      <HardwareProfileProvider>  {/* Add this */}
        <TranslationProvider>
          <>
            {children}
            <TextSelection ... />
            <Chatbot ... />
          </>
        </TranslationProvider>
      </HardwareProfileProvider>  {/* Close here */}
    </AuthProvider>
  );
}
```

**Acceptance Criteria**:
- [ ] App compiles and runs without errors
- [ ] HardwareProfileContext available throughout app
- [ ] No performance regression (check React DevTools)

**Testing**:
```bash
npm run start
# Open http://localhost:3000
# Check React DevTools → Components → HardwareProfileProvider should exist
```

---

### T4: Create HardwareSurveyModal Component ⏱️ 1.5 hours

**Priority**: P1 (MVP)
**Dependencies**: T3

**Implementation**:
```typescript
// humanoid_robot_book/src/components/Personalization/HardwareSurveyModal.tsx
import React, { useState, useEffect } from 'react';
import { useHardwareProfile } from '../../contexts/HardwareProfileContext';
import { HardwareType } from '../../types/hardware';
import './HardwareSurveyModal.module.css';

export default function HardwareSurveyModal() {
  const { profile, setProfile } = useHardwareProfile();
  const [isOpen, setIsOpen] = useState(false);
  const [selected, setSelected] = useState<HardwareType | null>(null);

  // Show modal on first visit (no profile)
  useEffect(() => {
    if (!profile) {
      const timer = setTimeout(() => setIsOpen(true), 1000); // 1s delay for UX
      return () => clearTimeout(timer);
    }
  }, [profile]);

  const handleSubmit = async () => {
    if (!selected) return;

    const newProfile = {
      type: selected,
      selectedAt: new Date(),
    };

    await setProfile(newProfile);
    setIsOpen(false);
  };

  const handleSkip = () => {
    // Default to cloud_mac (lowest common denominator)
    setProfile({ type: 'cloud_mac', selectedAt: new Date() });
    setIsOpen(false);
  };

  if (!isOpen) return null;

  return (
    <div className="hardware-survey-overlay">
      <div className="hardware-survey-modal">
        <h2>Welcome to Physical AI & Humanoid Robotics</h2>
        <p>
          To personalize your learning experience, please select your hardware setup:
        </p>

        <div className="hardware-options">
          <label className={selected === 'gpu_workstation' ? 'selected' : ''}>
            <input
              type="radio"
              name="hardware"
              value="gpu_workstation"
              checked={selected === 'gpu_workstation'}
              onChange={() => setSelected('gpu_workstation')}
            />
            <div className="option-content">
              <h3>GPU Workstation</h3>
              <p>RTX 4070 Ti or higher, local Isaac Sim</p>
            </div>
          </label>

          <label className={selected === 'edge_device' ? 'selected' : ''}>
            <input
              type="radio"
              name="hardware"
              value="edge_device"
              checked={selected === 'edge_device'}
              onChange={() => setSelected('edge_device')}
            />
            <div className="option-content">
              <h3>Edge Device</h3>
              <p>Jetson Orin Nano, Gazebo simulation</p>
            </div>
          </label>

          <label className={selected === 'cloud_mac' ? 'selected' : ''}>
            <input
              type="radio"
              name="hardware"
              value="cloud_mac"
              checked={selected === 'cloud_mac'}
              onChange={() => setSelected('cloud_mac')}
            />
            <div className="option-content">
              <h3>Cloud / Mac</h3>
              <p>No local GPU, cloud-based or Docker setup</p>
            </div>
          </label>
        </div>

        <div className="modal-actions">
          <button onClick={handleSkip} className="btn-skip">
            Skip for now
          </button>
          <button
            onClick={handleSubmit}
            disabled={!selected}
            className="btn-continue"
          >
            Continue
          </button>
        </div>
      </div>
    </div>
  );
}
```

**CSS**:
```css
/* HardwareSurveyModal.module.css */
.hardware-survey-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.7);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 9999;
}

.hardware-survey-modal {
  background: var(--ifm-background-color);
  padding: 2rem;
  border-radius: 12px;
  max-width: 600px;
  width: 90%;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
}

.hardware-options {
  margin: 1.5rem 0;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.hardware-options label {
  border: 2px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  padding: 1rem;
  cursor: pointer;
  transition: all 0.2s;
}

.hardware-options label:hover {
  border-color: var(--ifm-color-primary);
}

.hardware-options label.selected {
  border-color: var(--ifm-color-primary);
  background: var(--ifm-color-primary-lightest);
}

.modal-actions {
  display: flex;
  justify-content: space-between;
  margin-top: 1.5rem;
}

.btn-skip {
  padding: 0.75rem 1.5rem;
  background: transparent;
  border: 1px solid var(--ifm-color-emphasis-400);
  border-radius: 6px;
  cursor: pointer;
}

.btn-continue {
  padding: 0.75rem 2rem;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 6px;
  cursor: pointer;
  font-weight: 600;
}

.btn-continue:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}
```

**Acceptance Criteria**:
- [ ] Modal appears 1 second after page load (if no profile)
- [ ] 3 radio options render correctly
- [ ] Selecting option updates UI (border highlight)
- [ ] "Continue" button saves profile and closes modal
- [ ] "Skip" button sets default cloud_mac profile
- [ ] Modal does NOT appear on second visit (profile exists)

**Testing**:
```bash
# Clear localStorage
localStorage.clear();
# Reload page → modal should appear after 1s
# Select "GPU Workstation" → click Continue → modal closes
# Reload page → modal should NOT appear
```

---

## Phase 2: Personalize Button & Content Variants ⏱️ 2 hours

### T5: Create PersonalizeButton Component ⏱️ 45 min

**Priority**: P1 (MVP)
**Dependencies**: T3

**Implementation**:
```typescript
// humanoid_robot_book/src/components/Personalization/PersonalizeButton.tsx
import React from 'react';
import { useHardwareProfile } from '../../contexts/HardwareProfileContext';
import './PersonalizeButton.module.css';

export default function PersonalizeButton() {
  const { profile, isPersonalized, togglePersonalization } = useHardwareProfile();

  if (!profile) {
    return (
      <div className="personalize-notice">
        <p>👋 Set your hardware profile to see personalized content.</p>
        <a href="#" onClick={() => {/* TODO: Re-open survey */}}>
          Set Hardware Profile
        </a>
      </div>
    );
  }

  const hardwareLabel = {
    gpu_workstation: 'GPU Workstation',
    edge_device: 'Jetson Orin',
    cloud_mac: 'Cloud/Mac',
  }[profile.type];

  return (
    <div className="personalize-container">
      <button
        onClick={togglePersonalization}
        className={`personalize-btn ${isPersonalized ? 'active' : ''}`}
      >
        {isPersonalized ? '✓ Personalized' : 'Personalize'} for {hardwareLabel}
      </button>
      {isPersonalized && (
        <p className="personalize-hint">
          Content optimized for your {hardwareLabel}. Click again to see all variants.
        </p>
      )}
    </div>
  );
}
```

**CSS**:
```css
/* PersonalizeButton.module.css */
.personalize-container {
  margin: 1.5rem 0;
  padding: 1rem;
  background: var(--ifm-color-emphasis-100);
  border-radius: 8px;
}

.personalize-btn {
  padding: 0.75rem 1.5rem;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 6px;
  cursor: pointer;
  font-weight: 600;
  font-size: 1rem;
  transition: all 0.3s;
}

.personalize-btn:hover {
  background: var(--ifm-color-primary-dark);
  transform: translateY(-2px);
}

.personalize-btn.active {
  background: var(--ifm-color-success);
}

.personalize-hint {
  margin-top: 0.5rem;
  font-size: 0.9rem;
  color: var(--ifm-color-success-dark);
}

.personalize-notice {
  padding: 1rem;
  background: var(--ifm-alert-background-color);
  border-left: 4px solid var(--ifm-color-warning);
  border-radius: 4px;
}
```

**Acceptance Criteria**:
- [ ] Button renders with hardware type label
- [ ] Clicking button toggles `isPersonalized` state
- [ ] Active state shows checkmark and green color
- [ ] If no profile, shows "Set Hardware Profile" notice

**Testing**:
```mdx
<!-- Test in any .mdx file -->
import PersonalizeButton from '@site/src/components/Personalization/PersonalizeButton';

<PersonalizeButton />
```

---

### T6: Create ContentVariant Component ⏱️ 1 hour

**Priority**: P1 (MVP)
**Dependencies**: T3

**Implementation**:
```typescript
// humanoid_robot_book/src/components/Personalization/ContentVariant.tsx
import React from 'react';
import { useHardwareProfile } from '../../contexts/HardwareProfileContext';
import { HardwareType } from '../../types/hardware';
import './ContentVariant.module.css';

interface ContentVariantProps {
  hardwareType: HardwareType;
  label?: string;
  children: React.ReactNode;
}

export default function ContentVariant({
  hardwareType,
  label,
  children
}: ContentVariantProps) {
  const { profile, isPersonalized } = useHardwareProfile();

  // If no profile, show all variants (default behavior)
  if (!profile) {
    return (
      <div className="content-variant">
        <h4 className="variant-label">{label || `For ${hardwareType}`}</h4>
        {children}
      </div>
    );
  }

  // If personalized, highlight matching variant
  const isActive = profile.type === hardwareType;
  const className = isPersonalized
    ? isActive
      ? 'content-variant active'
      : 'content-variant dimmed'
    : 'content-variant';

  return (
    <div className={className}>
      <h4 className="variant-label">
        {isActive && isPersonalized && '✓ '}
        {label || `For ${hardwareType}`}
      </h4>
      {children}
    </div>
  );
}
```

**CSS**:
```css
/* ContentVariant.module.css */
.content-variant {
  margin: 1.5rem 0;
  padding: 1rem;
  border: 2px solid var(--ifm-color-emphasis-200);
  border-radius: 8px;
  transition: all 0.3s;
}

.content-variant.active {
  border-color: var(--ifm-color-primary);
  background: var(--ifm-color-primary-lightest);
}

.content-variant.dimmed {
  opacity: 0.5;
  filter: blur(1px);
}

.content-variant.dimmed:hover {
  opacity: 1;
  filter: blur(0);
}

.variant-label {
  margin-top: 0;
  color: var(--ifm-color-primary);
  font-size: 1.1rem;
  font-weight: 600;
}

.content-variant.active .variant-label {
  color: var(--ifm-color-success);
}
```

**Acceptance Criteria**:
- [ ] Renders children content inside bordered div
- [ ] Shows label header (default or custom)
- [ ] When personalized + matching type: shows checkmark, green border
- [ ] When personalized + NOT matching: dimmed/blurred
- [ ] Hovering dimmed variant un-dims it
- [ ] When NOT personalized: all variants shown equally

**Testing**:
```mdx
<!-- Test in .mdx -->
import ContentVariant from '@site/src/components/Personalization/ContentVariant';
import PersonalizeButton from '@site/src/components/Personalization/PersonalizeButton';

<PersonalizeButton />

<ContentVariant hardwareType="gpu_workstation" label="For RTX 4090 Users">
Install Isaac Sim locally...
</ContentVariant>

<ContentVariant hardwareType="edge_device" label="For Jetson Users">
Use Gazebo instead...
</ContentVariant>
```

---

### T7: Add PersonalizeButton to Module 3 (Test Chapter) ⏱️ 15 min

**Priority**: P1 (MVP)
**Dependencies**: T5, T6

**Implementation**:
```mdx
<!-- humanoid_robot_book/docs/05-module-3-nvidia-isaac.mdx -->
---
sidebar_position: 5
title: "Module 3: NVIDIA Isaac Sim"
---

import PersonalizeButton from '@site/src/components/Personalization/PersonalizeButton';
import ContentVariant from '@site/src/components/Personalization/ContentVariant';

# Module 3: NVIDIA Isaac Sim & Simulation

<PersonalizeButton />

<ContentVariant hardwareType="gpu_workstation" label="For GPU Workstation (RTX 4090+)">

## Local Isaac Sim Installation

If you have an RTX 4090 or higher, you can run Isaac Sim locally:

```bash
# Install NVIDIA Omniverse
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# Install Isaac Sim from Launcher
# Go to Exchange → Isaac Sim → Install
```

**Performance**: Expect 60+ FPS at 1080p on RTX 4090.

</ContentVariant>

<ContentVariant hardwareType="edge_device" label="For Jetson Orin Nano">

## Gazebo Alternative for Jetson

**⚠️ IMPORTANT**: NVIDIA Isaac Sim requires RTX GPU and is **not supported** on Jetson Orin.

Use **Gazebo** instead:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
```

**Performance**: Gazebo runs well on Jetson at 30 FPS.

</ContentVariant>

<ContentVariant hardwareType="cloud_mac" label="For Cloud / Mac Users">

## Omniverse Cloud

No local GPU? Use **Omniverse Cloud**:

1. Sign up at [https://www.nvidia.com/en-us/omniverse/cloud/](https://www.nvidia.com/en-us/omniverse/cloud/)
2. Launch Isaac Sim in browser
3. Stream to your Mac/Cloud instance

**OR** use Docker with CPU-only Gazebo:

```bash
docker run -it ros:humble-desktop-full
```

</ContentVariant>
```

**Acceptance Criteria**:
- [ ] PersonalizeButton appears at top of chapter
- [ ] 3 ContentVariant blocks render
- [ ] Clicking "Personalize" highlights correct variant
- [ ] Non-matching variants get dimmed

**Testing**:
```bash
# Visit http://localhost:3000/docs/module-3-nvidia-isaac
# Set hardware profile to "GPU Workstation"
# Click "Personalize"
# → GPU variant should be highlighted, others dimmed
```

---

## Phase 3: Backend Integration ⏱️ 2.5 hours

### T8: Create Backend API - POST /api/profile/update ⏱️ 1 hour

**Priority**: P1
**Dependencies**: None (backend only)

**Implementation**:
```python
# backend/src/api/profile.py (NEW FILE)
from fastapi import APIRouter, Request, HTTPException
from pydantic import BaseModel
from typing import Optional
from uuid import UUID

from ..services.database_service import db_service
from ..utils.logger import logger

router = APIRouter()

class HardwareProfileUpdate(BaseModel):
    userId: str
    hardwareType: str  # 'gpu_workstation' | 'edge_device' | 'cloud_mac'
    hardwareDetails: Optional[dict] = None

@router.post("/api/profile/update")
async def update_hardware_profile(
    request: Request,
    data: HardwareProfileUpdate
):
    """Update user's hardware profile in user_sessions table."""
    try:
        # Get session from cookie
        session_token = request.cookies.get("session_token")
        if not session_token:
            raise HTTPException(status_code=401, detail="No session token")

        # Get session by user_id
        session = await db_service.get_session_by_user_id(data.userId)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        # Update hardware profile
        await db_service.update_session_hardware(
            session.session_id,
            data.hardwareType,
            data.hardwareDetails or {}
        )

        logger.info(f"Updated hardware profile for user {data.userId}: {data.hardwareType}")

        return {
            "success": True,
            "message": "Hardware profile updated"
        }

    except Exception as e:
        logger.error(f"Failed to update hardware profile: {e}")
        raise HTTPException(status_code=500, detail=str(e))
```

**Add to main FastAPI app**:
```python
# backend/src/main.py
from .api import profile  # Add this import

# Add router
app.include_router(profile.router)
```

**Add database method**:
```python
# backend/src/services/database_service.py
async def update_session_hardware(
    self,
    session_id: UUID,
    hardware_type: str,
    hardware_details: dict
) -> bool:
    """Update hardware profile for a session."""
    await self.ensure_connected()
    async with self.pool.acquire() as conn:
        await conn.execute(
            """
            UPDATE user_sessions
            SET hardware_profile = $1
            WHERE session_id = $2
            """,
            {
                "type": hardware_type,
                "details": hardware_details
            },
            session_id
        )
    return True

async def get_session_by_user_id(self, user_id: str):
    """Get session by user_id."""
    await self.ensure_connected()
    async with self.pool.acquire() as conn:
        result = await conn.fetchrow(
            "SELECT * FROM user_sessions WHERE user_id = $1 ORDER BY created_at DESC LIMIT 1",
            user_id
        )
        if not result:
            return None
        return UserSession(
            session_id=result['session_id'],
            user_id=result['user_id'],
            hardware_profile=result.get('hardware_profile'),
            created_at=result['created_at'],
            last_active=result.get('last_active', result['created_at'])
        )
```

**Acceptance Criteria**:
- [ ] Endpoint accepts POST with userId + hardwareType
- [ ] Updates user_sessions.hardware_profile in database
- [ ] Returns 200 with success message
- [ ] Returns 401 if no session token
- [ ] Returns 404 if user session not found

**Testing**:
```bash
curl -X POST http://localhost:8000/api/profile/update \
  -H "Content-Type: application/json" \
  -H "Cookie: session_token=YOUR_SESSION_TOKEN" \
  -d '{"userId":"test-user-id","hardwareType":"gpu_workstation","hardwareDetails":{"gpuModel":"RTX 4090"}}'

# Should return: {"success":true,"message":"Hardware profile updated"}
```

---

### T9: Modify RAG Service to Inject Hardware Profile ⏱️ 1 hour

**Priority**: P2
**Dependencies**: T8

**Implementation**:
```python
# backend/src/services/rag_service.py
async def generate_response(
    self,
    query: str,
    conversation_id: Optional[UUID] = None,
    user_id: Optional[str] = None  # ADD THIS PARAMETER
) -> str:
    """Generate RAG response with hardware profile awareness."""

    # NEW: Look up user hardware profile
    hardware_context = ""
    if user_id:
        try:
            session = await db_service.get_session_by_user_id(user_id)
            if session and session.hardware_profile:
                hw_type = session.hardware_profile.get('type', 'unknown')

                # Build hardware-aware context
                if hw_type == 'edge_device':
                    hardware_context = """
IMPORTANT: The user has a Jetson Orin Nano (edge device).
- Do NOT recommend NVIDIA Isaac Sim (requires RTX GPU, not supported on Jetson)
- DO recommend Gazebo for simulation
- DO recommend ROS 2 on Jetson
- Mention edge-specific optimizations when relevant
"""
                elif hw_type == 'gpu_workstation':
                    gpu_model = session.hardware_profile.get('details', {}).get('gpuModel', 'RTX GPU')
                    hardware_context = f"""
IMPORTANT: The user has a {gpu_model} workstation.
- You CAN recommend local NVIDIA Isaac Sim installation
- Suggest local simulation for best performance
- Recommend CUDA-accelerated libraries
"""
                elif hw_type == 'cloud_mac':
                    hardware_context = """
IMPORTANT: The user is on Cloud/Mac (no local GPU).
- Do NOT recommend local Isaac Sim
- DO recommend Omniverse Cloud or Docker-based Gazebo
- Suggest cloud-based alternatives when possible
"""
        except Exception as e:
            logger.warning(f"Failed to fetch hardware profile for RAG: {e}")

    # Build final prompt with hardware context
    system_prompt = f"""You are a helpful robotics and AI tutor for the book
"Physical AI & Humanoid Robotics". Answer questions based on the provided context.

{hardware_context}

Context from book:
{retrieved_chunks}

Question: {query}
"""

    # Continue with existing LLM call...
    response = await self.llm.generate(system_prompt)
    return response
```

**Update API to pass user_id**:
```python
# backend/src/api/chat.py
@router.post("/api/chat/query")
async def chat_query(request: Request, data: ChatQuery):
    # Get user_id from session (if logged in)
    session_token = request.cookies.get("session_token")
    user_id = None
    if session_token:
        try:
            session = await db_service.get_session(UUID(session_token))
            user_id = session.user_id
        except:
            pass  # Anonymous user

    # Pass to RAG service
    response = await rag_service.generate_response(
        query=data.question,
        user_id=user_id  # ADD THIS
    )
    return {"response": response}
```

**Acceptance Criteria**:
- [ ] RAG service accepts user_id parameter
- [ ] Looks up hardware profile from database
- [ ] Injects hardware-specific context into system prompt
- [ ] Chatbot gives hardware-aware responses
- [ ] Works for anonymous users (no hardware context)

**Testing**:
```bash
# Test with Jetson profile user
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -H "Cookie: session_token=JETSON_USER_TOKEN" \
  -d '{"question":"How do I run Isaac Sim?"}'

# Should respond: "I see you're using Jetson Orin Nano. Isaac Sim requires RTX GPU..."
```

---

### T10: Add Profile Settings Page ⏱️ 30 min

**Priority**: P2
**Dependencies**: T4

**Implementation**:
```typescript
// humanoid_robot_book/src/pages/profile-settings.tsx
import React from 'react';
import Layout from '@theme/Layout';
import { useHardwareProfile } from '../contexts/HardwareProfileContext';

export default function ProfileSettings() {
  const { profile, setProfile, clearProfile } = useHardwareProfile();
  const [showModal, setShowModal] = useState(false);

  return (
    <Layout title="Profile Settings">
      <div className="container margin-vert--lg">
        <h1>Hardware Profile Settings</h1>

        {profile ? (
          <div>
            <p>Current Hardware: <strong>{profile.type}</strong></p>
            <button onClick={() => setShowModal(true)}>
              Update Hardware Profile
            </button>
            <button onClick={clearProfile}>Clear Profile</button>
          </div>
        ) : (
          <div>
            <p>No hardware profile set.</p>
            <button onClick={() => setShowModal(true)}>
              Set Hardware Profile
            </button>
          </div>
        )}

        {showModal && <HardwareSurveyModal onClose={() => setShowModal(false)} />}
      </div>
    </Layout>
  );
}
```

**Acceptance Criteria**:
- [ ] Page accessible at /profile-settings
- [ ] Shows current hardware profile
- [ ] "Update" button re-opens survey modal
- [ ] "Clear" button removes profile (shows survey on next visit)

---

## Phase 4: Content Authoring ⏱️ 4 hours (separate)

### T11: Write Hardware-Specific Variants for Module 2

**Priority**: P2
**Dependencies**: T6, T7
**Owner**: Content Team

**Implementation**: Add ContentVariant blocks to `docs/04-module-2-gazebo-unity.mdx` for:
- GPU: Local Unity setup
- Jetson: Gazebo on Jetson
- Cloud/Mac: Unity Cloud or Docker Gazebo

---

### T12: Write Hardware-Specific Variants for Module 4

**Priority**: P2
**Dependencies**: T6, T7
**Owner**: Content Team

**Implementation**: Add ContentVariant blocks to `docs/06-module-4-vla.mdx` for:
- GPU: Local VLA training with PyTorch + CUDA
- Jetson: Edge VLA deployment (inference only)
- Cloud/Mac: Google Colab VLA training

---

## Phase 5: Testing & Polish ⏱️ 1 hour

### T13: Manual End-to-End Testing

**Priority**: P1
**Dependencies**: All previous tasks

**Test Scenarios**:
1. **First Visit (Anonymous)**:
   - [ ] Visit book → survey modal appears
   - [ ] Select GPU → modal closes → profile saved
   - [ ] Navigate to Module 3 → PersonalizeButton appears
   - [ ] Click button → GPU content highlighted
   - [ ] Reload page → no modal, profile persists

2. **Logged-In User**:
   - [ ] Login → set hardware profile
   - [ ] Logout → login on different browser
   - [ ] Profile restored from backend ✓

3. **Chatbot Integration**:
   - [ ] Set Jetson profile
   - [ ] Ask "How do I run Isaac Sim?"
   - [ ] Response warns Isaac Sim not supported ✓

4. **Profile Update**:
   - [ ] Go to /profile-settings
   - [ ] Change profile from GPU → Cloud
   - [ ] Return to Module 3 → click Personalize
   - [ ] Cloud variant now highlighted ✓

---

## Summary

**Total Tasks**: 13
**Estimated Time**: 12.5 hours (excluding content authoring)

**Critical Path**:
T1 → T2 → T3 → T4 → T5 → T6 → T7 → T13

**Parallel Tracks**:
- Backend (T8 → T9) can be done simultaneously with Frontend (T1-T7)
- Content (T11, T12) can be done by separate team while development progresses

**MVP Definition** (minimum viable):
- T1-T7: Hardware survey + Personalize button + Content variants (1 chapter)
- T13: Manual testing

**Full Release**:
- All tasks T1-T13 complete
- 3 chapters with hardware-specific variants
- Backend sync + RAG integration
- Settings page functional
