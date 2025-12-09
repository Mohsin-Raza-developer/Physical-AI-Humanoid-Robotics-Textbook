# Quickstart Guide: Creating Content for the Textbook

**Audience**: Content authors, instructors, and contributors
**Last Updated**: 2025-12-10

## Overview

This guide explains how to create new lessons, assessments, and other content for the Physical AI & Humanoid Robotics textbook. It covers file naming conventions, frontmatter requirements, sidebar configuration, and common workflows.

## Prerequisites

- Basic knowledge of Markdown
- Git installed and repository cloned
- Node.js and npm installed
- Familiarity with the 13-week course structure

## Quick Reference

### File Naming Patterns

| Content Type | Pattern | Example |
|--------------|---------|---------|
| Module Intro | `intro.md` | `docs/module-1-ros2/intro.md` |
| Lesson | `week-X-lesson-Y-topic.md` | `docs/module-1-ros2/week-3-lesson-1-ros2-architecture.md` |
| Multi-week | `week-X-Y-topic.md` | `docs/module-3-isaac/week-8-10-isaac-rl-lab.md` |
| Assessment | `descriptive-name.md` | `docs/module-1-ros2/assessments/ros2-package-project.md` |
| Setup Guide | `descriptive-name.md` | `docs/setup/hardware-requirements.md` |
| Resource | `descriptive-name.md` | `docs/resources/glossary.md` |

### Frontmatter Template

```yaml
---
title: "Week X Lesson Y: Full Descriptive Title"
sidebar_label: "Lesson Y: Short Title"
sidebar_position: XY
description: "Brief lesson summary (1-2 sentences)"
tags: [module-name, week-X, topic-keywords]
---
```

### Directory Structure

```
docs/
├── intro/                     # Weeks 1-2 content
├── module-1-ros2/             # Weeks 3-5 content
├── module-2-gazebo-unity/     # Weeks 6-7 content
├── module-3-isaac/            # Weeks 8-10 content
├── module-4-vla/              # Weeks 11-13 content
├── setup/                     # Hardware/software setup
└── resources/                 # Glossary, references, etc.
```

## Creating a New Lesson

### Step 1: Determine Location

Identify which module and week your lesson belongs to:

- **Weeks 1-2**: `docs/intro/`
- **Weeks 3-5**: `docs/module-1-ros2/`
- **Weeks 6-7**: `docs/module-2-gazebo-unity/`
- **Weeks 8-10**: `docs/module-3-isaac/`
- **Weeks 11-13**: `docs/module-4-vla/`

### Step 2: Create the File

Use the naming pattern: `week-{WEEK}-lesson-{NUMBER}-{topic}.md`

Example:
```bash
touch docs/module-1-ros2/week-4-lesson-2-building-packages.md
```

### Step 3: Add Frontmatter

Start your file with YAML frontmatter:

```yaml
---
title: "Week 4 Lesson 2: Building ROS 2 Packages with Python"
sidebar_label: "Lesson 2: Building Packages"
sidebar_position: 42
description: "Learn to create, build, and manage ROS 2 packages using colcon and setuptools"
tags: [ros2, packages, week-4, python]
---
```

**Frontmatter Field Guide**:
- `title`: Full title shown at top of page (include week and lesson number)
- `sidebar_label`: Shorter title for sidebar (omit week number to save space)
- `sidebar_position`: Week number * 10 + lesson number (e.g., Week 4 Lesson 2 = 42)
- `description`: Brief summary for SEO and previews
- `tags`: Module identifier, week identifier, and topic keywords

### Step 4: Write Content

Structure your lesson with clear sections:

```markdown
# Week 4 Lesson 2: Building ROS 2 Packages with Python

## Learning Objectives

By the end of this lesson, you will be able to:
- Create a new ROS 2 Python package from scratch
- Understand package structure and configuration files
- Build packages using colcon

## Prerequisites

- Completed Week 3 lessons on ROS 2 architecture
- Python 3.8+ installed
- ROS 2 Humble installed

## Introduction

[Your content here...]

## Section 1: Package Structure

[Your content here...]

## Section 2: Hands-On Lab

:::tip Lab Exercise
Try creating your own package following the steps below...
:::

## Summary

[Key takeaways...]

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org)
- [Colcon Tutorial](https://colcon.readthedocs.io)

## Next Steps

In the next lesson, we'll explore [topic]...
```

### Step 5: Update Sidebar

Edit `sidebars.ts` to include your new lesson:

```typescript
{
  type: 'category',
  label: 'Week 4: Advanced Concepts',
  items: [
    'module-1-ros2/week-4-lesson-1-services-actions',
    'module-1-ros2/week-4-lesson-2-building-packages',  // ← Add this
  ],
},
```

### Step 6: Test Locally

```bash
npm run start
```

Navigate to your new lesson and verify:
- Title and sidebar label display correctly
- Lesson appears in correct position
- Next/previous navigation works
- All links resolve properly

### Step 7: Commit and Push

```bash
git add docs/module-1-ros2/week-4-lesson-2-building-packages.md sidebars.ts
git commit -m "feat(module-1): add Week 4 Lesson 2 on building packages"
git push
```

## Creating a New Assessment

### Step 1: Create Assessments Directory (if needed)

```bash
mkdir -p docs/module-1-ros2/assessments
```

### Step 2: Create Assessment File

```bash
touch docs/module-1-ros2/assessments/ros2-package-project.md
```

### Step 3: Use Assessment Template

```yaml
---
title: "ROS 2 Package Development Project"
sidebar_label: "Package Project"
sidebar_position: 99
description: "Build a complete ROS 2 package with nodes, topics, and services"
tags: [ros2, assessment, project, week-5]
---

# ROS 2 Package Development Project

## Overview

[Project description...]

## Learning Objectives

This project assesses your ability to:
- Design and implement ROS 2 nodes
- Create publishers and subscribers
- Define custom messages

## Requirements

### Functional Requirements
1. Create a ROS 2 package named `student_robot_controller`
2. Implement a publisher node that sends sensor data
3. Implement a subscriber node that processes commands

### Technical Requirements
- Python 3.8+
- ROS 2 Humble
- Must pass provided test suite

## Deliverables

- [ ] Source code in GitHub repository
- [ ] README with setup instructions
- [ ] Video demo (3-5 minutes)
- [ ] Written reflection (1-2 pages)

## Grading Rubric

| Criteria | Points | Description |
|----------|--------|-------------|
| Functionality | 40 | All requirements met, code works |
| Code Quality | 20 | Clean, documented, follows conventions |
| Testing | 20 | Comprehensive tests, all pass |
| Documentation | 20 | Clear README, inline comments |
| **Total** | **100** | |

## Resources

- [ROS 2 Package Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Python Coding Standards](https://peps.python.org/pep-0008/)

## Submission

Submit via [platform] by end of Week 5.
```

### Step 4: Add to Sidebar

```typescript
{
  type: 'category',
  label: 'Assessments',
  items: [
    'module-1-ros2/assessments/ros2-package-project',
  ],
},
```

## Creating a Setup Guide

### Step 1: Create File in Setup Directory

```bash
touch docs/setup/software-setup.md
```

### Step 2: Use Setup Template

```yaml
---
title: "Software Setup: ROS 2 and Development Environment"
sidebar_label: "Software Setup"
sidebar_position: 2
description: "Install Ubuntu, ROS 2 Humble, and required development tools"
tags: [setup, installation, ros2, ubuntu]
---

# Software Setup Guide

## Overview

This guide walks you through installing all required software for the course.

## Prerequisites

- Computer with Ubuntu 22.04 LTS or ability to dual-boot/VM
- 50GB free disk space
- Administrator/sudo access

## Step 1: Install Ubuntu 22.04

[Instructions...]

## Step 2: Install ROS 2 Humble

[Instructions...]

## Verification

Run the following to verify installation:

\`\`\`bash
ros2 --version
# Expected output: ros2 doctor 0.10.3
\`\`\`

## Troubleshooting

### Issue: "ros2 command not found"

**Solution**: Add ROS 2 to your PATH...

## Next Steps

Once setup is complete, proceed to [Hardware Requirements](./hardware-requirements.md).
```

## Content Writing Best Practices

### 1. Use Docusaurus Admonitions

```markdown
:::note
This is a note for additional context.
:::

:::tip
Helpful tips and best practices go here.
:::

:::warning
Important warnings about potential issues.
:::

:::danger
Critical information about breaking changes or hazards.
:::
```

### 2. Include Code Blocks with Language Tags

````markdown
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
```
````

### 3. Add Images

```markdown
![ROS 2 Architecture](./img/ros2-architecture.png)
```

Store images in module-specific directories: `static/img/module-1/`

### 4. Link to Other Lessons

```markdown
See [Week 3 Lesson 1: ROS 2 Architecture](../module-1-ros2/week-3-lesson-1-ros2-architecture.md) for background.
```

### 5. Use Tables for Comparisons

```markdown
| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Middleware | Custom | DDS |
| Python | 2.7 | 3.x |
```

## Sidebar Configuration Guide

### Basic Category Structure

```typescript
{
  type: 'category',
  label: 'Week 3: ROS 2 Fundamentals',
  items: [
    'module-1-ros2/week-3-lesson-1-ros2-architecture',
    'module-1-ros2/week-3-lesson-2-nodes-topics',
  ],
},
```

### Category with Link to Intro

```typescript
{
  type: 'category',
  label: 'Module 1: ROS 2 (Weeks 3-5)',
  collapsed: true,
  link: {
    type: 'doc',
    id: 'module-1-ros2/intro',
  },
  items: [
    // week categories here
  ],
},
```

### Collapsible vs. Expanded

- Use `collapsed: true` for modules (expanded when active)
- Use `collapsed: false` for Introduction (always visible)

## Testing Checklist

Before submitting new content, verify:

- [ ] File name follows pattern: `week-X-lesson-Y-topic.md`
- [ ] Frontmatter includes all required fields
- [ ] `sidebar_position` is unique within module
- [ ] Tags include module and week identifiers
- [ ] Lesson added to `sidebars.ts` in correct position
- [ ] Local build succeeds: `npm run build`
- [ ] No broken links (Docusaurus will warn during build)
- [ ] Images are optimized (< 200KB each)
- [ ] Code blocks have language tags for syntax highlighting
- [ ] Next/previous navigation works correctly

## Common Tasks

### Renaming a Lesson

1. Update filename: `git mv old-name.md new-name.md`
2. Update frontmatter `title` and `sidebar_label`
3. Update `sidebars.ts` with new document ID
4. Update any internal links to this lesson
5. Test build

### Moving a Lesson to Different Week

1. Update filename week number: `week-3-*` → `week-4-*`
2. Update frontmatter `sidebar_position`: 31 → 41
3. Update frontmatter `title`: "Week 3..." → "Week 4..."
4. Update tags: `week-3` → `week-4`
5. Move sidebar entry to new week category
6. Test build

### Adding a New Module (Future Expansion)

1. Create directory: `mkdir docs/module-5-new-topic`
2. Create intro: `touch docs/module-5-new-topic/intro.md`
3. Add module category to `sidebars.ts`
4. Update navbar in `docusaurus.config.ts` if needed
5. Create lessons following standard pattern

## Troubleshooting

### Build Error: "Duplicate sidebar_position"

**Cause**: Two lessons in same module have same `sidebar_position`

**Solution**: Ensure each lesson has unique position (week * 10 + lesson number)

### Warning: "Document not included in sidebar"

**Cause**: Created file but didn't add to `sidebars.ts`

**Solution**: Add document ID to appropriate category in `sidebars.ts`

### Broken Link Warning

**Cause**: Link to non-existent document or incorrect path

**Solution**: Use relative paths and verify file exists:
```markdown
<!-- Correct -->
[Link](../module-2-gazebo-unity/intro.md)

<!-- Incorrect -->
[Link](/docs/module-2-gazebo-unity/intro)
```

## Getting Help

- **Documentation**: https://docusaurus.io/docs
- **Project Issues**: [GitHub Issues](https://github.com/Mohsin-Raza-developer/Physical-AI-Humanoid-Robotics-Textbook/issues)
- **Community**: [Discord](#) (if available)

## Templates

### Lesson Template

See `docs/intro/intro.md` for a complete example of lesson structure.

### Assessment Template

See example in "Creating a New Assessment" section above.

### Setup Guide Template

See example in "Creating a Setup Guide" section above.

## Next Steps

Now that you understand the structure, you can:

1. Review existing lessons for examples
2. Create your first lesson following this guide
3. Test locally and iterate
4. Submit PR for review

Happy content creation! 🚀
