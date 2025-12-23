# Claude Code Skills

This directory contains reusable skills (workflows) for the Physical AI & Humanoid Robotics project.

## What Are Skills?

**Skills** are documented workflows that automate multi-step tasks. Unlike agents (which answer questions), skills guide you through executing complex procedures step-by-step.

**Key Characteristics**:
- **Manual execution** - You run the commands, not automated scripts
- **Platform-agnostic** - Work on Windows, Mac, and Linux
- **Documented workflows** - Clear steps with expected outputs
- **Reusable** - Can be invoked repeatedly for consistent results

---

## Quick Start

### How to Use Skills

1. **Find the skill** you need in the list below
2. **Open the skill file** (e.g., `deploy-book.md`)
3. **Read prerequisites** - Ensure you have required tools installed
4. **Follow steps sequentially** - Copy-paste commands as documented
5. **Verify success** - Check success criteria at the end

### When to Use Skills vs. Manual Work

| Use Skills When | Do Manual Work When |
|-----------------|---------------------|
| Repeating multi-step workflows (deployment, setup) | One-time tasks |
| Need consistency across team members | Exploratory/experimental work |
| Steps are well-defined and tested | Requirements are unclear |
| Workflow has dependencies (order matters) | Task is trivial (single command) |

---

## Available Skills

### 1. `deploy-book` - Automated Book Deployment

**Purpose**: Deploy the Physical AI & Humanoid Robotics book to production (Vercel)

**Use When**:
- Publishing new content chapters
- Updating RAG embeddings after content changes
- Deploying fixes or improvements to live site

**Quick Example**:
```bash
# From project root
cd humanoid_robot_book
npm run build        # Build Docusaurus site
cd ../backend
python scripts/embed_book_content.py  # Update embeddings
git add . && git commit -m "deploy: Update book content"
git push origin main  # Vercel auto-deploys
```

**Success Criteria**:
- ✅ Docusaurus builds without errors
- ✅ Embeddings updated in Qdrant
- ✅ Git push successful
- ✅ Vercel deployment live (check dashboard)
- ✅ RAG chatbot returns updated content

**Prerequisites**:
- Node.js 18+
- Python 3.10+
- Vercel CLI configured
- Environment variables set (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)

**Full Documentation**: [deploy-book.md](./deploy-book.md)

---

### 2. `create-chapter` - MDX Chapter Scaffolding

**Purpose**: Generate new chapter files with proper structure, imports, and personalization blocks

**Use When**:
- Adding new course modules (Week 14+)
- Creating supplementary content
- Ensuring consistency with existing chapter format

**Quick Example**:
```bash
# Navigate to docs directory
cd humanoid_robot_book/docs/

# Create new file (manual - no script)
touch 08-week-14-swarm-robotics.mdx

# Copy template from create-chapter.md skill
# Fill in:
# - Frontmatter (id, title, sidebar_position)
# - Content for ContentVariant blocks (GPU/Edge/Cloud)
# - Build and verify
npm run build
```

**Success Criteria**:
- ✅ Valid MDX file created
- ✅ Frontmatter correct (id, title, sidebar_label, sidebar_position)
- ✅ Imports present (TranslationToggle, PersonalizeButton, ContentVariant)
- ✅ ContentVariant blocks for all 3 hardware types
- ✅ Docusaurus builds without errors

**Prerequisites**:
- Docusaurus project set up
- Understanding of MDX format
- Content ready for all 3 hardware variants

**Full Documentation**: [create-chapter.md](./create-chapter.md)

---

## Skill Comparison Table

| Skill | Type | Duration | Automation Level | Prerequisites |
|-------|------|----------|------------------|---------------|
| **deploy-book** | Deployment | 5-10 min | Semi-automated | Node.js, Python, Vercel CLI |
| **create-chapter** | Content Creation | 15-30 min | Manual template | MDX knowledge, Docusaurus |

---

## Creating New Skills

Want to add a new skill for your team? Follow this template:

### Skill Template Structure

```markdown
# [Skill Name]

**Description**: [One-line description]

**Version**: 1.0.0

**Author**: [Your Name]

---

## What This Skill Does

[2-3 sentences explaining the workflow]

## Usage

[Example invocation]

## Steps Executed

### 1. [Step Name]
[Commands to run]

### 2. [Step Name]
[Commands to run]

## Prerequisites

- [Tool 1]
- [Tool 2]

## Success Criteria

- ✅ [Check 1]
- ✅ [Check 2]

## Troubleshooting

**Problem**: [Common issue]
**Solution**: [How to fix]

## Tags

`[tag1]`, `[tag2]`
```

**Best Practices**:
1. **Keep steps atomic** - One clear action per step
2. **Include expected output** - Show what success looks like
3. **Add troubleshooting** - Document common failures
4. **Test cross-platform** - Verify on Windows/Mac/Linux
5. **Version your skills** - Update version number when changing steps

---

## Troubleshooting

### General Issues

#### Skill Prerequisites Not Met
**Problem**: Commands fail with "command not found" or "module not found"
**Solution**:
1. Check Prerequisites section in skill documentation
2. Install missing tools (Node.js, Python packages, CLI tools)
3. Verify versions match requirements

#### Platform-Specific Commands Fail
**Problem**: Bash commands don't work on Windows (or vice versa)
**Solution**:
1. Use platform-agnostic commands where possible (e.g., `npm` works everywhere)
2. Check skill documentation for platform notes
3. Use Git Bash or WSL on Windows for Unix commands

#### Outdated Skill Documentation
**Problem**: Skill steps reference files/commands that no longer exist
**Solution**:
1. Check skill version number and last updated date
2. Review recent git commits for related changes
3. Update skill documentation with correct paths/commands
4. Submit PR with fixes

---

### Skill-Specific Issues

#### deploy-book: Vercel Deployment Fails
**Problem**: `git push` succeeds but Vercel shows build errors
**Solution**:
1. Check Vercel dashboard for specific error logs
2. Common cause: Missing environment variables
3. Verify all required env vars set in Vercel Settings → Environment Variables
4. Retry deployment after fixing

#### deploy-book: Embeddings Update Fails
**Problem**: `embed_book_content.py` script errors
**Solution**:
1. Check API keys: `echo $COHERE_API_KEY` or `echo $OPENAI_API_KEY`
2. Verify Qdrant connection: `curl $QDRANT_URL`
3. Check Python dependencies: `pip install -r requirements.txt`
4. Review script output for specific error (API rate limit, network timeout, etc.)

#### create-chapter: Docusaurus Build Fails
**Problem**: `npm run build` shows MDX syntax errors
**Solution**:
1. Validate frontmatter YAML syntax (no tabs, proper indentation)
2. Check imports use exact component names (case-sensitive)
3. Ensure ContentVariant `hardwareType` prop matches: `gpu_workstation`, `edge_device`, `cloud_or_mac`
4. Test with `npm start` (dev mode shows clearer errors)

---

## Best Practices

### When to Use Each Skill

**Use `deploy-book` when**:
- ✅ You've updated content in `humanoid_robot_book/docs/`
- ✅ You've fixed RAG chatbot responses
- ✅ You need to publish changes to production
- ❌ NOT for testing (use `npm start` for local preview)

**Use `create-chapter` when**:
- ✅ Adding new curriculum modules
- ✅ Creating supplementary guides
- ✅ Need consistent structure across chapters
- ❌ NOT for minor edits (just edit existing files directly)

### Workflow Integration

**Typical Development Flow**:
1. **Create chapter** using `create-chapter` skill
2. **Fill content** for all hardware variants
3. **Test locally** with `npm start`
4. **Deploy** using `deploy-book` skill
5. **Verify** live site and RAG chatbot

**Collaboration Tips**:
- Share skill documentation with team members
- Update skills when project structure changes
- Document custom variations for your environment

---

## Metrics & Success

### Skill Effectiveness

Track these metrics to measure skill value:
- **Time saved**: How much faster than manual process?
- **Error rate**: Fewer mistakes than manual execution?
- **Adoption**: How many team members use this skill?
- **Maintenance**: How often does skill need updates?

### Success Indicators

**Skills are working well when**:
- ✅ New team members can execute workflows independently
- ✅ Deployment errors decrease over time
- ✅ Less time spent on repetitive tasks
- ✅ Consistent results across team members

---

## Support & Contributions

**Questions?**
- Check skill-specific documentation in individual .md files
- Review troubleshooting section above
- Open an issue in the GitHub repo

**Want to contribute a new skill?**
1. Follow the Skill Template Structure above
2. Test on Windows, Mac, and Linux
3. Document prerequisites and troubleshooting
4. Submit PR with your skill .md file

---

## Related Resources

- **Agents**: See [.claude/agents/README.md](../agents/README.md) for AI assistants that answer questions
- **Main Documentation**: See [project README.md](../../README.md)
- **Deployment Guide**: See [backend/DEPLOYMENT.md](../../backend/DEPLOYMENT.md)

---

**Last Updated**: 2025-12-23
**Maintainer**: Physical AI & Humanoid Robotics Team
**Version**: 1.0.0
