# Feature 009: Content Variants Expansion

**Priority**: P0 (Gamification requirement)
**Points**: +25 (Dynamic Content completion)
**Effort**: 1 hour

## Problem
Constitution requires "Personalize" button functional on **3+ chapters**. Currently only Module 3 has ContentVariant components.

## Goal
Add hardware-specific ContentVariant blocks to Modules 2 and 4.

## Acceptance Criteria
- [ ] Module 2 (Gazebo) has GPU/Edge/Cloud variants for installation
- [ ] Module 4 (VLA) has GPU/Edge/Cloud variants for model training
- [ ] PersonalizeButton already exists (no changes needed)
- [ ] Total: 3 chapters with working personalization ✅

## Implementation
Add `<ContentVariant>` blocks with hardware-specific instructions for:
- GPU Workstation: Local installation, full features
- Edge Device: Jetson-specific commands, limited features
- Cloud/Mac: Docker/cloud alternatives

## Success Metrics
- Gamification points: 25/50 → 50/50 (Dynamic Content) ✅
