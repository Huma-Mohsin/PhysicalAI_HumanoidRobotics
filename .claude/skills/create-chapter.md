# Create Chapter Skill

**Description**: Scaffold new MDX chapter files with proper structure, imports, and personalization variants for the Physical AI & Humanoid Robotics book

**Version**: 1.0.0

**Author**: Physical AI & Humanoid Robotics Team

---

## What This Skill Does

This skill provides a template and workflow for creating new book chapters that:
1. Include required frontmatter (id, title, sidebar configuration)
2. Import necessary components (TranslationToggle, PersonalizeButton, ContentVariant)
3. Include personalization blocks for all 3 hardware types (GPU/Edge/Cloud)
4. Follow the established chapter structure and styling
5. Build successfully in Docusaurus without errors

Use this skill when adding new course modules (Week 14+) or supplementary content to maintain consistency across all chapters.

---

## Usage

### Quick Start

```bash
# 1. Navigate to docs directory
cd humanoid_robot_book/docs/

# 2. Create new file (use sequential numbering)
touch 08-week-14-your-topic.mdx

# 3. Copy template below into the file
# 4. Fill in TODOs with your content
# 5. Build and verify
cd ..
npm run build
```

---

## MDX Chapter Template

Copy this entire template into your new `.mdx` file:

```mdx
---
id: your-chapter-id
title: "Your Chapter Title"
sidebar_label: "Short Label"
sidebar_position: 8
---

import TranslationToggle from '@site/src/components/Translation/TranslationToggle';
import PersonalizeButton from '@site/src/components/Personalization/PersonalizeButton';
import ContentVariant from '@site/src/components/Personalization/ContentVariant';

<div style={{display: 'flex', justifyContent: 'flex-end', marginBottom: '1rem'}}>
  <TranslationToggle />
</div>

# Your Chapter Title

## Overview

<!-- TODO: Add 2-3 sentence overview of this chapter's focus -->

**Focus**: [Main focus area, e.g., "Multi-robot coordination"]

[Brief description of what students will learn]

---

## Learning Objectives

By the end of this chapter, you will:

1. ✅ [Learning objective 1]
2. ✅ [Learning objective 2]
3. ✅ [Learning objective 3]
4. ✅ [Learning objective 4]

---

<PersonalizeButton />

---

## Section 1: [Section Title]

<!-- TODO: Add general content that applies to all users -->

[General introduction to the topic]

<ContentVariant hardwareType="gpu_workstation">

### For GPU Workstation Users (RTX 4070 Ti+)

<!-- TODO: Add GPU-specific content -->

**You can run this locally with full performance!**

✅ **What You Can Do:**
- [GPU-specific capability 1]
- [GPU-specific capability 2]
- [GPU-specific capability 3]

**Installation/Setup:**
```bash
# TODO: Add GPU-specific commands
sudo apt install [package-name]
```

**Your Advantage:** [Explain why GPU workstation excels at this]

</ContentVariant>

<ContentVariant hardwareType="edge_device">

### For Edge Device Users (Jetson Orin Nano)

<!-- TODO: Add Jetson-specific content -->

**Optimized for edge deployment!**

✅ **What You Can Do:**
- [Jetson-specific capability 1]
- [Jetson-specific capability 2]
- [Jetson-specific capability 3]

⚠️ **Limitations:**
- [Limitation 1, e.g., "Cannot run full Isaac Sim"]
- [Limitation 2]

**Installation/Setup:**
```bash
# TODO: Add Jetson-specific commands (ARM64)
sudo apt install [package-name]
```

**Your Advantage:** [Explain why Jetson excels at this]

</ContentVariant>

<ContentVariant hardwareType="cloud_or_mac">

### For Cloud/Mac Users

<!-- TODO: Add Cloud/Mac-specific content -->

**Use cloud-based or Docker alternatives!**

✅ **What You Can Do:**
- [Cloud/Mac capability 1]
- [Cloud/Mac capability 2]
- [Cloud/Mac capability 3]

⚠️ **Considerations:**
- [Consideration 1, e.g., "Cloud costs apply"]
- [Consideration 2, e.g., "Docker performance varies"]

**Setup via Docker:**
```bash
# TODO: Add Docker-based commands
docker run -it [image-name]
```

**Your Advantage:** [Explain platform independence benefit]

</ContentVariant>

---

## Section 2: [Another Section]

<!-- TODO: Add more sections as needed -->

[Content here]

---

## Hands-On Project

<!-- TODO: Add practical exercise -->

### Project Requirements

[List project requirements]

### Assessment Criteria

- [Criterion 1]
- [Criterion 2]

---

## Resources

- [Official Documentation Link]
- [Tutorial Link]
- [Community Resources]

---

## Next Steps

[Transition to next chapter]

👉 **Next**: [Link to next chapter]
```

---

## Step-by-Step Instructions

### Step 1: Determine Chapter Details

Before creating the file, decide:
- **Chapter Number**: Next sequential number (e.g., 08, 09, 10)
- **Topic**: Clear, descriptive topic name
- **Sidebar Position**: Determines order in navigation

**Naming Convention**: `{number}-{slug}.mdx`
- Example: `08-week-14-swarm-robotics.mdx`
- Use lowercase with hyphens (kebab-case)

### Step 2: Create File

```bash
cd humanoid_robot_book/docs/
touch 08-week-14-swarm-robotics.mdx
```

### Step 3: Fill Frontmatter

```yaml
---
id: week-14-swarm-robotics      # Matches filename (without number prefix)
title: "Week 14: Swarm Robotics"  # Full display title
sidebar_label: "Week 14: Swarm"   # Shorter label for sidebar
sidebar_position: 8                # Navigation order
---
```

**Tips**:
- `id`: Use kebab-case, no spaces
- `title`: Use quotes if it contains colons
- `sidebar_label`: Keep under 20 characters
- `sidebar_position`: Sequential numbering

### Step 4: Add Content for All Hardware Types

**IMPORTANT**: Every chapter MUST include ContentVariant blocks for all 3 types:
1. `gpu_workstation` - RTX 4070 Ti / 4080 / 4090 users
2. `edge_device` - Jetson Orin Nano / AGX Orin users
3. `cloud_or_mac` - Mac / Windows / Cloud users

**Content Guidelines**:
- **GPU**: Emphasize local performance, full feature access, Isaac Sim
- **Edge**: Focus on deployment, real sensors, edge AI optimization
- **Cloud/Mac**: Docker alternatives, cloud services, platform independence

### Step 5: Build and Verify

```bash
cd humanoid_robot_book/
npm run build
```

**Expected Output**: Build completes without errors

**Common Errors**:
- Frontmatter YAML syntax (no tabs, proper indentation)
- Missing imports (check line 8-10)
- Invalid `hardwareType` prop (must be exact: `gpu_workstation`, `edge_device`, `cloud_or_mac`)
- Unclosed JSX tags

### Step 6: Test Locally

```bash
npm start
```

Open `http://localhost:3000` and navigate to your new chapter:
- ✅ Sidebar shows your chapter
- ✅ TranslationToggle button appears (top-right)
- ✅ PersonalizeButton appears
- ✅ ContentVariant blocks render correctly
- ✅ All markdown formatting renders properly

---

## Prerequisites

- **Docusaurus**: Project already set up
- **Node.js**: 18+ installed
- **MDX Knowledge**: Basic understanding of Markdown + JSX
- **Content Ready**: Have content prepared for all 3 hardware variants

---

## Success Criteria

After completing this skill, verify:

- ✅ New `.mdx` file exists in `humanoid_robot_book/docs/`
- ✅ Frontmatter fields all filled correctly
- ✅ Required imports present (TranslationToggle, PersonalizeButton, ContentVariant)
- ✅ TranslationToggle button visible (top-right corner)
- ✅ PersonalizeButton placed after overview section
- ✅ ContentVariant blocks for **all 3 hardware types** included
- ✅ `npm run build` succeeds without errors
- ✅ `npm start` renders chapter correctly
- ✅ Chapter appears in sidebar navigation
- ✅ All markdown/code blocks render properly

---

## Example Output

**File**: `08-week-14-swarm-robotics.mdx`

**Frontmatter**:
```yaml
---
id: week-14-swarm-robotics
title: "Week 14: Swarm Robotics & Multi-Agent Systems"
sidebar_label: "Week 14: Swarm"
sidebar_position: 8
---
```

**Build Output**:
```
[SUCCESS] Generated static files in "build/".
✨  Done in 45.23s.
```

**Browser View**:
- Title: "Week 14: Swarm Robotics & Multi-Agent Systems"
- Sidebar: Shows "Week 14: Swarm" at position 8
- TranslationToggle: Visible top-right
- PersonalizeButton: Visible after overview
- ContentVariant: GPU section highlighted (if user has GPU profile)

---

## Troubleshooting

### Build Fails: "Error parsing frontmatter"
**Problem**: YAML syntax error in frontmatter
**Solution**:
1. Check no tabs used (use spaces only)
2. Ensure proper indentation (2 spaces)
3. Use quotes around values with special characters (colons, quotes)
4. Valid example:
```yaml
---
id: test-chapter
title: "Test: Chapter Title"
sidebar_label: "Test"
sidebar_position: 99
---
```

### Build Fails: "Cannot resolve component"
**Problem**: Missing or incorrect import statement
**Solution**:
1. Verify imports are exact (case-sensitive):
```javascript
import TranslationToggle from '@site/src/components/Translation/TranslationToggle';
import PersonalizeButton from '@site/src/components/Personalization/PersonalizeButton';
import ContentVariant from '@site/src/components/Personalization/ContentVariant';
```
2. Check `@site` prefix is included
3. Ensure component files exist at those paths

### ContentVariant Not Working
**Problem**: Hardware-specific content not toggling
**Solution**:
1. Verify `hardwareType` prop is **exactly** one of:
   - `gpu_workstation` (not `gpu-workstation` or `GPU_Workstation`)
   - `edge_device` (not `edge-device`)
   - `cloud_or_mac` (not `cloud-mac`)
2. Ensure user has hardware profile set (check via PersonalizeButton)
3. Clear browser cache and reload

### Chapter Not in Sidebar
**Problem**: New chapter doesn't appear in navigation
**Solution**:
1. Check `sidebar_position` is unique and sequential
2. Verify file is in `docs/` directory (not subdirectory)
3. Restart dev server (`npm start`)
4. Check Docusaurus config if custom sidebar defined

### Markdown Not Rendering
**Problem**: Code blocks or formatting broken
**Solution**:
1. Ensure code blocks use triple backticks (\`\`\`)
2. Close all JSX tags properly (`<ContentVariant>...</ContentVariant>`)
3. Use `{/*  */}` for comments inside JSX (not `<!-- -->`)
4. Test with minimal content first, add incrementally

---

## Advanced Tips

### Multi-Line Code Examples

Use language-specific syntax highlighting:

\`\`\`python
# Python code here
import rospy
\`\`\`

\`\`\`bash
# Bash commands
sudo apt install ros-humble-desktop
\`\`\`

### Admonitions (Info Boxes)

Docusaurus supports special callout boxes:

```markdown
:::tip Hardware Recommendation
GPU users should use local Isaac Sim for best performance.
:::

:::warning Important
Jetson users cannot run Isaac Sim locally.
:::

:::info Note
Mac users can use Omniverse Cloud as an alternative.
:::
```

### Including Images

```markdown
![Alt text](./assets/image-name.png)
```

Place images in `docs/assets/` directory.

---

## Tags

`content-creation`, `mdx`, `docusaurus`, `templates`, `chapters`

---

**Last Updated**: 2025-12-23
**Maintainer**: Physical AI & Humanoid Robotics Team
