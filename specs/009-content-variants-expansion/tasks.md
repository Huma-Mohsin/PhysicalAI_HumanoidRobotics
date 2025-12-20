# Tasks: Content Variants Expansion

## T001: Add ContentVariant to Module 2 (Gazebo)
**File**: `humanoid_robot_book/docs/04-module-2-gazebo-unity.mdx`

**Steps**:
1. Import PersonalizeButton and ContentVariant
2. Add PersonalizeButton after Learning Objectives
3. Add ContentVariant blocks for Gazebo installation:
   - GPU Workstation: Native installation
   - Edge Device: Jetson-optimized Gazebo
   - Cloud/Mac: Docker setup

**Acceptance**: Module 2 shows personalized Gazebo setup instructions

---

## T002: Add ContentVariant to Module 4 (VLA)
**File**: `humanoid_robot_book/docs/06-module-4-vla.mdx`

**Steps**:
1. Import PersonalizeButton and ContentVariant
2. Add PersonalizeButton after Learning Objectives
3. Add ContentVariant blocks for VLA model training:
   - GPU Workstation: Local training with PyTorch
   - Edge Device: Pre-trained model deployment only
   - Cloud/Mac: Colab/cloud training links

**Acceptance**: Module 4 shows personalized VLA training options

---

## T003: Test Personalization
**Steps**:
1. Set profile to GPU → verify RTX-specific content shows
2. Set profile to Edge → verify Jetson alternatives show
3. Set profile to Cloud → verify cloud options show

**Acceptance**: All 3 profiles work across 3 modules
