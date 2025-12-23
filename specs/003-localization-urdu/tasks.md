# Tasks: Urdu Localization for Docusaurus Book

**Input**: Design documents from `/specs/003-localization-urdu/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT explicitly requested in the spec, so they are omitted per template guidance. Focus is on implementation and manual validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web App**: `humanoid_robot_book/src/` (Docusaurus React components)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 [P] Install required dependencies for translation functionality (i18n libraries, translation APIs if needed)
- [ ] T002 Configure Docusaurus i18n plugin in docusaurus.config.ts for multi-language support
- [ ] T003 [P] Create basic directory structure: humanoid_robot_book/src/components/Translation/, humanoid_robot_book/src/hooks/, humanoid_robot_book/src/utils/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Translation Infrastructure

- [ ] T004 Create translation utility functions in humanoid_robot_book/src/utils/translation.ts (API integration, caching, error handling)
- [ ] T005 Create custom translation hook in humanoid_robot_book/src/hooks/useTranslation.ts (state management, language toggle logic)
- [ ] T006 Create translation context provider in humanoid_robot_book/src/components/Translation/TranslationProvider.tsx (global state for language preference)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Urdu Translation Toggle (Priority: P1) 🎯 MVP

**Goal**: Enable users to toggle chapter content between English and Urdu with preserved formatting

**Independent Test**: Visit any book chapter, click "Translate to Urdu" button, verify content switches to Urdu while preserving all technical diagrams, code blocks, and navigation

### Implementation for User Story 1

- [ ] T007 [P] Create TranslationToggle component in humanoid_robot_book/src/components/Translation/TranslationToggle.tsx (button UI, RTL styling)
- [ ] T008 Implement translation API integration in utils/translation.ts (Google Translate or similar service)
- [ ] T009 Add RTL support styling in *.module.css files (right-to-left text rendering for Urdu)
- [ ] T010 Integrate TranslationToggle with Docusaurus layout (add to docs pages)
- [ ] T011 Implement content translation functionality (translate text while preserving code blocks)
- [ ] T012 Add language preference persistence using localStorage
- [ ] T013 Add loading indicators and error handling for translation operations
- [ ] T014 Test translation toggle functionality on sample chapter

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Translation Persistence (Priority: P2)

**Goal**: Remember user's language preference across sessions

**Independent Test**: Toggle to Urdu, refresh page or close browser, reopen book and verify content remains in Urdu

### Implementation for User Story 2

- [ ] T015 Enhance useTranslation hook with session persistence across browser tabs
- [ ] T016 Implement cross-tab language preference synchronization
- [ ] T017 Add cache invalidation for translated content after certain time period
- [ ] T018 Test persistence functionality across page refreshes and browser sessions
- [ ] T019 Add fallback mechanism if localStorage is unavailable

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Code Block Translation (Priority: P3)

**Goal**: Translate code comments while preserving syntax and functionality

**Independent Test**: View chapter with code examples, toggle to Urdu, verify code blocks have translated comments while preserving syntax highlighting

### Implementation for User Story 3

- [ ] T020 [P] Update translation utility to handle code block parsing (preserve syntax, translate comments only)
- [ ] T021 Implement code block translation logic (detect comments, preserve structure)
- [ ] T022 Update TranslationToggle to handle code block translation appropriately
- [ ] T023 Test code block translation functionality while maintaining syntax highlighting
- [ ] T024 Add special handling for technical terminology in code examples

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Quality Assurance
- [ ] T025 [P] Add comprehensive error handling for API failures (translation service unavailable)
- [ ] T026 Add graceful degradation when translation is not available
- [ ] T027 Test functionality across all major browsers (Chrome, Firefox, Safari, Edge)
- [ ] T028 Add accessibility support for translated content (ARIA labels, screen readers)

### Documentation & Deployment
- [ ] T029 Update docusaurus.config.ts with proper i18n configuration for production
- [ ] T030 Add documentation for translation feature in README.md
- [ ] T031 Run quickstart validation to ensure all functionality works as expected
- [ ] T032 Test deployment to verify translation features work in production environment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Depends on US1 foundation (translation hook and context)
- **User Story 3 (P3)**: Depends on US1 foundation (translation functionality)

### Within Each User Story

- Core implementation before UI integration
- Error handling and loading states implemented
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- T007, T008, T009 in User Story 1 can run in parallel

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test translation toggle functionality independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo (Complete feature)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Focus on core functionality first, polish later