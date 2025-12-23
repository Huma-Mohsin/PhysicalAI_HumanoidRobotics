# Implementation Plan: Reusable Intelligence Enhancement

**Branch**: `013-reusable-intelligence-enhancement` | **Date**: 2025-12-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/013-reusable-intelligence-enhancement/spec.md`

## Summary

Enhance the project's reusable intelligence infrastructure by creating additional Claude Code agents and skills with comprehensive documentation. This achieves the 50-point gamification requirement for Reusable Intelligence by delivering:
1. `.claude/skills/README.md` with full documentation of all automation skills
2. `hardware-advisor` agent for budget-based hardware recommendations
3. `create-chapter` skill for MDX content scaffolding
4. Enhanced documentation for existing `book-rag-helper` agent and `deploy-book` skill

**Technical Approach**: Static documentation and JSON configuration files (no executable code). Agents are declarative JSON configs; skills are Markdown documentation of manual workflows.

## Technical Context

**Language/Version**: Markdown + JSON (static configuration, no runtime code)
**Primary Dependencies**: None (Claude Code reads configurations natively)
**Storage**: N/A (version-controlled files in `.claude/` directory)
**Testing**: Manual verification - invoke agents/skills and validate outputs match documentation
**Target Platform**: Claude Code CLI (cross-platform: Windows/Mac/Linux)
**Project Type**: Documentation enhancement (no source code changes)
**Performance Goals**: Agent responses within 10 seconds; skill documentation readable in <2 minutes
**Constraints**:
- Agent configurations limited to 4 tools max (Read, Grep, Glob, Bash)
- Skills must be platform-agnostic (work on Windows + Mac + Linux)
- JSON must be valid and parseable by Claude Code
- Markdown must render correctly in GitHub/VS Code/Claude Code
**Scale/Scope**:
- 2 agents total (existing `book-rag-helper` + new `hardware-advisor`)
- 2 skills total (existing `deploy-book` + new `create-chapter`)
- 2 README files (~300-400 lines each with examples, troubleshooting, best practices)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Embodied Intelligence
**Status**: COMPLIANT

- **Rationale**: While this feature is documentation-focused, it supports embodied intelligence by:
  - `hardware-advisor` agent bridges digital AI recommendations with physical hardware selection for robotics
  - Agents contextualize book content that demonstrates AI+robotics integration
  - Skills automate Physical AI book deployment and content creation workflows

**Verification**: Agents reference physical hardware (GPU, Jetson, sensors) and robotics concepts (ROS 2, Isaac Sim).

---

### ✅ II. Spec-Driven Architecture
**Status**: COMPLIANT

- **Rationale**: This feature follows `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` workflow
- **Verification**:
  - Spec created: `specs/013-reusable-intelligence-enhancement/spec.md`
  - Plan created: `specs/013-reusable-intelligence-enhancement/plan.md` (this file)
  - Tasks will be generated via `/sp.tasks`
  - ADR suggestion will be evaluated (likely not needed for documentation-only feature)
  - PHR will be created after plan completion

---

### ✅ III. Interactive Personalization
**Status**: COMPLIANT

- **Rationale**: `hardware-advisor` agent enhances personalization by helping users select hardware based on their profile
- **Verification**: Agent considers user's existing hardware, budget, and role (student/educator) to provide tailored recommendations

---

### ✅ IV. Gamified Completeness
**Status**: TARGET FEATURE

- **Rationale**: This feature directly implements the "Reusable Intelligence (50 pts)" requirement
- **Verification**:
  - 2+ agents with diverse capabilities
  - 2+ skills automating real workflows
  - Comprehensive documentation with examples and troubleshooting
  - Target: 50/50 points achieved after implementation

---

**Constitution Violations**: NONE

**Re-check After Phase 1**: Will verify agent/skill documentation quality meets constitution's exemplary engineering standards.

## Project Structure

### Documentation (this feature)

```text
specs/013-reusable-intelligence-enhancement/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file (complete)
├── research.md          # Phase 0 output (to be created)
├── data-model.md        # N/A (no data entities)
├── quickstart.md        # Phase 1 output (usage guide for agents/skills)
├── contracts/           # N/A (no API contracts)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

**Structure Decision**: This feature modifies `.claude/` directory structure only. No application source code changes.

```text
.claude/
├── agents/
│   ├── README.md                  # ENHANCED (add usage examples, troubleshooting)
│   ├── book-rag-helper.json       # EXISTING (minor updates for additional examples)
│   └── hardware-advisor.json      # NEW (hardware recommendation agent)
│
├── skills/
│   ├── README.md                  # NEW (comprehensive skill documentation)
│   ├── deploy-book.md             # EXISTING (verify completeness)
│   └── create-chapter.md          # NEW (MDX scaffolding skill)
│
└── settings.local.json            # UNCHANGED (no configuration changes needed)
```

**No changes to**:
- `humanoid_robot_book/` (frontend)
- `backend/` (API)
- `specs/` (except this feature's directory)
- `.specify/` (templates)

## Complexity Tracking

**No Constitution Violations** - This section is not applicable.

---

## Phase 0: Research & Decision Making

### Research Topics

#### RT-001: Claude Code Agent Best Practices
**Question**: What are the best practices for creating effective Claude Code agents with JSON configuration?

**Research Tasks**:
1. Review Claude Code documentation on custom agents
2. Analyze existing `book-rag-helper.json` structure
3. Identify optimal `systemPrompt` length and structure
4. Determine ideal number of examples (target: 2-4 per agent)
5. Evaluate tool usage patterns (when to use Grep vs Read vs Glob)

**Decision Criteria**: Agent configurations must be:
- Concise (systemPrompt <500 words)
- Focused (single domain, not general-purpose)
- Well-exemplified (2+ realistic usage examples)
- Tool-efficient (use minimum tools needed)

---

#### RT-002: Hardware Recommendation Logic
**Question**: What hardware recommendation algorithm should `hardware-advisor` agent use?

**Research Tasks**:
1. Review existing hardware requirements (from `humanoid_robot_book/docs/02-hardware-requirements.mdx`)
2. Extract cost data for GPU workstations, Jetson kits, and cloud alternatives
3. Define budget ranges:
   - Low (<$500): Cloud/Mac + Docker
   - Medium ($500-$2000): Jetson Orin Nano + sensors
   - High ($2000+): GPU workstation (RTX 4070 Ti+)
4. Consider user roles:
   - Student: Budget-optimized, prioritize learning
   - Educator: Bulk purchasing, lab setup
   - Hobbyist: Balancing capability and cost

**Decision Criteria**: Recommendations must be:
- Cost-accurate (within $50 of actual prices)
- Role-appropriate (students get different advice than educators)
- Hardware-aware (consider what user already owns)
- Actionable (include component lists and purchase links)

---

#### RT-003: MDX Chapter Template Structure
**Question**: What MDX structure should `create-chapter` skill generate?

**Research Tasks**:
1. Analyze existing chapter files:
   - `humanoid_robot_book/docs/01-introduction.mdx`
   - `humanoid_robot_book/docs/03-module-1-ros2.mdx`
   - `humanoid_robot_book/docs/04-module-2-gazebo-unity.mdx`
2. Extract common frontmatter fields (id, title, sidebar_label, sidebar_position)
3. Identify required imports (TranslationToggle, PersonalizeButton, ContentVariant)
4. Determine ContentVariant block structure for 3 hardware types
5. Define placeholder locations for content authors

**Decision Criteria**: Template must:
- Build without errors in Docusaurus
- Include all required imports
- Have ContentVariant placeholders for GPU/Edge/Cloud
- Include TODO comments guiding content authors
- Match existing chapter style and structure

---

#### RT-004: Documentation Structure & Style Guide
**Question**: What documentation format best supports agent/skill discovery and usage?

**Research Tasks**:
1. Review industry-standard README formats (GitHub best practices)
2. Analyze successful open-source project documentation
3. Define sections: Overview, Quick Start, Reference, Troubleshooting, Examples
4. Determine appropriate detail level (balance brevity with completeness)
5. Evaluate table vs. list formats for feature comparisons

**Decision Criteria**: Documentation must:
- Enable discovery within 2 minutes (browsing README)
- Include copy-paste examples for immediate use
- Provide troubleshooting for common issues
- Explain when to use each agent/skill vs. alternatives
- Be maintainable (easy to update as agents/skills evolve)

---

### Research Outputs

**Deliverable**: `research.md` with decisions for:
- Agent JSON structure standards
- Hardware recommendation pricing table
- MDX chapter template with placeholders
- Documentation style guide and section templates

---

## Phase 1: Design & Contracts

### Data Model

**Not Applicable** - This feature has no data entities. Agents and skills are static configuration files.

### API Contracts

**Not Applicable** - No APIs. Agents are invoked via Claude Code CLI commands; skills are manual workflows documented in Markdown.

### Architecture Decisions

#### AD-001: Agent Scope Boundaries
**Decision**: `hardware-advisor` agent focused exclusively on hardware recommendations; does not answer general robotics questions

**Rationale**:
- Prevents overlap with `book-rag-helper` (which handles content queries)
- Keeps agent system prompts focused and effective
- Allows specialized pricing/component knowledge

**Alternatives Considered**:
- **Combined agent**: Single agent for all queries → Rejected (dilutes effectiveness, exceeds token limits)
- **Hardware + course planning**: Recommend courses too → Rejected (out of scope for this feature)

---

#### AD-002: Skill Execution Model
**Decision**: Skills are documented manual workflows, not executable scripts

**Rationale**:
- Claude Code skills are Markdown documentation, not runnable code
- Ensures cross-platform compatibility (Windows/Mac/Linux)
- User retains control over each step (can adapt to their environment)
- Aligns with Claude Code's design (assistive, not autonomous)

**Alternatives Considered**:
- **Executable Bash/PowerShell scripts**: → Rejected (platform-specific, security concerns, not Claude Code's model)
- **Fully automated workflows**: → Rejected (removes user agency, error-prone across environments)

---

#### AD-003: Documentation Versioning Strategy
**Decision**: Documentation includes creation date and version numbers in agent configs; no automated versioning system

**Rationale**:
- Simple, low-maintenance approach
- Version numbers in agent JSON help track when recommendations become stale
- Manual updates acceptable given low change frequency (agents/skills updated quarterly at most)

**Alternatives Considered**:
- **Automated doc generation**: → Rejected (overkill for 4 total agents/skills)
- **Changelog files**: → Rejected (adds maintenance burden without clear value)

---

### Quickstart Guide

**Deliverable**: `quickstart.md` with:

#### Using Agents

```markdown
## Quick Start: Hardware Advisor Agent

**Use Case**: "I have $1000 budget and a Mac. What hardware should I buy?"

**Steps**:
1. Ensure you're in the project directory
2. Invoke Claude Code
3. Type: `@hardware-advisor I have $1000 and a Mac, what should I buy for this course?`
4. Review recommendations (GPU vs Jetson vs Cloud)
5. Ask follow-up: `@hardware-advisor What sensors do I need for the Jetson setup?`

**Expected Output**: Cost breakdown table, component list, purchase links
```

#### Using Skills

```markdown
## Quick Start: Create Chapter Skill

**Use Case**: "I want to add Week 15: Swarm Robotics chapter"

**Steps**:
1. Navigate to `humanoid_robot_book/docs/`
2. Reference `create-chapter` skill documentation
3. Create new file: `08-week-15-swarm.mdx`
4. Copy template from skill docs
5. Fill in frontmatter (title, sidebar position)
6. Add content to ContentVariant blocks (GPU/Edge/Cloud)
7. Build and verify: `npm run build`

**Expected Output**: Valid MDX file that builds without errors
```

---

## Phase 2: Tasks

**Not created in `/sp.plan`**. Run `/sp.tasks` command next to generate task breakdown from this plan.

**Preview of Task Categories**:
1. **Documentation Tasks**: Write README files for agents/ and skills/
2. **Agent Creation**: Write `hardware-advisor.json` configuration
3. **Skill Creation**: Write `create-chapter.md` documentation
4. **Testing Tasks**: Verify all agents/skills work as documented
5. **Validation Tasks**: Confirm 50/50 gamification points achieved

---

## Critical Files to Modify/Create

| File | Action | Priority | Estimated LOC |
|------|--------|----------|---------------|
| `.claude/agents/README.md` | UPDATE (already exists, enhance) | P1 | ~400 lines |
| `.claude/skills/README.md` | CREATE | P1 | ~350 lines |
| `.claude/agents/hardware-advisor.json` | CREATE | P2 | ~80 lines |
| `.claude/skills/create-chapter.md` | CREATE | P3 | ~150 lines |
| `.claude/agents/book-rag-helper.json` | UPDATE (add more examples) | P3 | +20 lines |
| `specs/013-reusable-intelligence-enhancement/research.md` | CREATE (Phase 0) | P0 | ~200 lines |
| `specs/013-reusable-intelligence-enhancement/quickstart.md` | CREATE (Phase 1) | P1 | ~100 lines |

**Total New Content**: ~1300 lines of documentation and configuration

---

## Success Criteria

### Phase 0 Complete When:
- [ ] `research.md` exists with all 4 research topics addressed
- [ ] Hardware pricing table compiled with accurate costs
- [ ] MDX template structure defined with required imports
- [ ] Documentation style guide established

### Phase 1 Complete When:
- [ ] `quickstart.md` exists with usage examples for all agents/skills
- [ ] Architecture decisions documented (agent scope, skill execution model, versioning)
- [ ] No unresolved design questions

### Phase 2 Complete When (via `/sp.tasks`):
- [ ] All tasks generated and prioritized
- [ ] Acceptance criteria defined for each task
- [ ] Estimated effort assigned

### Implementation Complete When:
- [ ] `.claude/agents/README.md` enhanced with examples and troubleshooting
- [ ] `.claude/skills/README.md` created with full skill documentation
- [ ] `hardware-advisor` agent provides accurate recommendations (tested with 3 budget scenarios)
- [ ] `create-chapter` skill template generates valid MDX files
- [ ] All documentation reviewed for clarity and accuracy
- [ ] 50/50 Reusable Intelligence points verified

---

## Risks & Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Agent system prompts too long (exceed token limits) | Medium | High | Keep prompts <500 words; test with long queries |
| Hardware prices become outdated | High | Low | Include "last updated" date in agent config; document quarterly review process |
| Skills don't work across platforms (Windows/Mac/Linux) | Medium | Medium | Test skill documentation on all 3 platforms before finalizing |
| Documentation is too verbose (users skip reading) | Low | Medium | Use concise bullet points, clear headings, and "Quick Start" sections |

---

## Next Steps

1. **Run `/sp.tasks`** to generate task breakdown from this plan
2. **Create Phase 0 Research**: Write `research.md` with hardware pricing, template structure, and style guide
3. **Create Phase 1 Quickstart**: Write `quickstart.md` with usage examples
4. **Implement Tasks**: Follow task order (P1 → P2 → P3)
5. **Validate Gamification**: Verify 50/50 points achieved
6. **Create PHR**: Document this planning session

---

**Plan Status**: ✅ COMPLETE
**Ready for**: `/sp.tasks` command
**Branch**: `013-reusable-intelligence-enhancement`
**Constitution Check**: ✅ PASSING
