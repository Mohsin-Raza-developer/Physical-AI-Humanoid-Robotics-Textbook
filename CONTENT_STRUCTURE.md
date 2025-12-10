# Content Structure Documentation

**Physical AI & Humanoid Robotics Textbook**

This document describes the folder structure, file naming conventions, and organizational principles for the Physical AI Humanoid Robotics course content.

## Overview

The textbook is organized as a 13-week course covering ROS 2, simulation, NVIDIA Isaac, and Vision-Language-Action models. Content follows a hierarchical structure mapping to the weekly course progression.

## Directory Structure

```
docs/
├── intro/                              # Introduction (Weeks 1-2)
│   ├── index.md                       # Course overview and objectives
│   ├── week-1-2-physical-ai-foundations.md
│   └── week-1-2-sensors-overview.md
│
├── module-1-ros2/                     # Module 1: ROS 2 (Weeks 3-5)
│   ├── intro.md                       # Module overview and learning objectives
│   ├── week-3-lesson-1-ros2-architecture.md
│   ├── week-3-lesson-2-nodes-packages.md
│   ├── week-4-lesson-1-services-actions.md
│   ├── week-4-lesson-2-building-packages.md
│   ├── week-5-lesson-1-launch-files.md
│   ├── week-5-lesson-2-urdf-humanoids.md
│   └── assessments/
│       └── ros2-package-project.md
│
├── module-2-gazebo-unity/             # Module 2: Simulation (Weeks 6-7)
│   ├── intro.md
│   ├── week-6-lesson-1-gazebo-setup.md
│   ├── week-6-lesson-2-urdf-sdf.md
│   ├── week-7-lesson-1-physics-simulation.md
│   ├── week-7-lesson-2-unity-robotics.md
│   ├── week-7-lesson-3-sensor-simulation.md
│   └── assessments/
│       └── gazebo-simulation-project.md
│
├── module-3-isaac/                    # Module 3: NVIDIA Isaac (Weeks 8-10)
│   ├── intro.md
│   ├── week-8-lesson-1-isaac-sim-setup.md
│   ├── week-8-lesson-2-isaac-sdk-intro.md
│   ├── week-9-lesson-1-ai-perception.md
│   ├── week-9-lesson-2-manipulation.md
│   ├── week-10-lesson-1-reinforcement-learning.md
│   ├── week-10-lesson-2-sim-to-real.md
│   └── assessments/
│       └── isaac-perception-pipeline.md
│
├── module-4-vla/                      # Module 4: VLA & Capstone (Weeks 11-13)
│   ├── intro.md
│   ├── week-11-lesson-1-humanoid-kinematics.md
│   ├── week-11-lesson-2-bipedal-locomotion.md
│   ├── week-12-lesson-1-manipulation-grasping.md
│   ├── week-12-lesson-2-human-robot-interaction.md
│   ├── week-13-lesson-1-conversational-robotics.md
│   ├── week-13-lesson-2-gpt-integration.md
│   ├── week-13-lesson-3-capstone-project.md
│   └── assessments/
│       └── capstone-simulated-humanoid.md
│
├── setup/                             # Hardware & Software Setup Guides
│   ├── hardware-requirements.md
│   ├── software-setup.md
│   ├── lab-infrastructure.md
│   └── student-kit-guide.md
│
└── resources/                         # Supplementary Resources
    ├── glossary.md
    ├── references.md
    └── additional-reading.md
```

## File Naming Conventions

### General Principles

1. **Use kebab-case** (lowercase with hyphens): `week-3-lesson-1-ros2-architecture.md`
2. **Be descriptive**: Filenames should indicate content without reading the file
3. **Follow patterns consistently**: All lesson files use the same structure

### Naming Patterns

#### Module Intro Pages
```
intro.md
```
- Located in each module directory
- Contains module overview, learning objectives, weekly breakdown, prerequisites

#### Weekly Lesson Files
```
week-{week_number}-lesson-{lesson_number}-{topic}.md
```

**Examples:**
- `week-3-lesson-1-ros2-architecture.md`
- `week-7-lesson-2-unity-robotics.md`
- `week-13-lesson-3-capstone-project.md`

**Components:**
- `week-{number}`: Course week (1-13)
- `lesson-{number}`: Lesson within that week (1, 2, 3, etc.)
- `{topic}`: Descriptive topic name in kebab-case

#### Multi-Week Content
```
week-{start}-{end}-{topic}.md
```

**Examples:**
- `week-1-2-physical-ai-foundations.md`
- `week-1-2-sensors-overview.md`

#### Assessment Files
```
assessments/{project-name}.md
```

**Examples:**
- `assessments/ros2-package-project.md`
- `assessments/gazebo-simulation-project.md`
- `assessments/capstone-simulated-humanoid.md`

#### Setup and Resource Files
```
{descriptive-name}.md
```

**Examples:**
- `hardware-requirements.md`
- `software-setup.md`
- `glossary.md`

## Frontmatter Requirements

Every content file MUST include frontmatter with these fields:

```yaml
---
title: Page Title
sidebar_label: Sidebar Display Name
sidebar_position: 10
description: Brief description for SEO
tags: [tag1, tag2, tag3]
---
```

### Field Descriptions

- **title**: Full page title (displayed at top of page)
- **sidebar_label**: Shorter label for sidebar display
- **sidebar_position**: Numeric position for ordering (see Sidebar Position System below)
- **description**: SEO description (optional but recommended)
- **tags**: Array of relevant tags for categorization (optional)

### Sidebar Position System

Position numbers follow a hierarchical pattern:

```
Introduction:
  - index.md:               1
  - week-1-2 content:       2, 3

Module 1 (Weeks 3-5):
  - intro.md:               1
  - Week 3 lessons:         31, 32
  - Week 4 lessons:         41, 42
  - Week 5 lessons:         51, 52

Module 2 (Weeks 6-7):
  - intro.md:               1
  - Week 6 lessons:         61, 62
  - Week 7 lessons:         71, 72, 73

Module 3 (Weeks 8-10):
  - intro.md:               1
  - Week 8 lessons:         81, 82
  - Week 9 lessons:         91, 92
  - Week 10 lessons:        101, 102

Module 4 (Weeks 11-13):
  - intro.md:               1
  - Week 11 lessons:        111, 112
  - Week 12 lessons:        121, 122
  - Week 13 lessons:        131, 132, 133
```

**Pattern:** `{week_number}{lesson_number}`

**Examples:**
- Week 3, Lesson 1 → `sidebar_position: 31`
- Week 7, Lesson 3 → `sidebar_position: 73`
- Week 13, Lesson 2 → `sidebar_position: 132`

## Content Organization Principles

### 1. Hierarchical Module Structure

Content is organized in a clear hierarchy:
1. **Course Introduction** (Weeks 1-2)
2. **Module 1: ROS 2 Fundamentals** (Weeks 3-5)
3. **Module 2: Simulation** (Weeks 6-7)
4. **Module 3: NVIDIA Isaac** (Weeks 8-10)
5. **Module 4: VLA & Capstone** (Weeks 11-13)
6. **Setup & Resources** (Reference materials)

### 2. Weekly Progression

Each module maps directly to course weeks:
- **Weeks 1-2**: Introduction to Physical AI
- **Weeks 3-5**: ROS 2 Fundamentals
- **Weeks 6-7**: Gazebo and Unity Simulation
- **Weeks 8-10**: NVIDIA Isaac Platform
- **Weeks 11-13**: Vision-Language-Action & Capstone

### 3. Self-Contained Modules

Each module directory contains:
- **intro.md**: Module overview, objectives, weekly breakdown
- **Lesson files**: Weekly content organized by week and lesson
- **assessments/**: Module projects and evaluation materials

### 4. Assessments Integration

Each module has a dedicated `assessments/` subdirectory containing:
- Project requirements and specifications
- Grading rubrics
- Submission guidelines
- Testing checklists

### 5. Supplementary Content

**Setup Guides** (`setup/`):
- Hardware requirements and recommendations
- Software installation instructions
- Lab infrastructure options
- Student kit assembly guide

**Resources** (`resources/`):
- Glossary of technical terms
- Academic references and citations
- Additional reading materials and tutorials

## Sidebar Configuration

The sidebar structure is defined in `sidebars.ts` and follows this organization:

```typescript
{
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction (Weeks 1-2)',
      collapsed: false,
      items: [...],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 (Weeks 3-5)',
      collapsed: true,
      link: { type: 'doc', id: 'module-1-ros2/intro' },
      items: [
        {type: 'category', label: 'Week 3: ...', items: [...]},
        {type: 'category', label: 'Week 4: ...', items: [...]},
        {type: 'category', label: 'Assessments', items: [...]},
      ],
    },
    // ... other modules
    {
      type: 'category',
      label: 'Setup & Resources',
      items: [...],
    },
  ],
}
```

### Sidebar Features

- **Collapsible categories**: Modules start collapsed for better navigation
- **Linked categories**: Module titles link to intro pages
- **Weekly grouping**: Lessons grouped by week for clarity
- **Assessment sections**: Dedicated category in each module

## Content Creation Guidelines

### Adding a New Lesson

1. **Create file** with proper naming: `week-X-lesson-Y-topic.md`
2. **Add frontmatter** with required fields
3. **Set sidebar_position** following the pattern (XY)
4. **Update sidebars.ts** to include the new file
5. **Link from module intro** if applicable
6. **Run build** to validate: `npm run build`

### Adding a New Module

1. **Create directory**: `docs/module-N-name/`
2. **Create intro.md** with module overview
3. **Create assessments/** subdirectory
4. **Add lesson files** following naming conventions
5. **Update sidebars.ts** with new module structure
6. **Update navigation** links in config and footers

### Adding Assessment

1. **Create file** in `module-X/assessments/project-name.md`
2. **Include:**
   - Project overview and objectives
   - Detailed requirements
   - Grading rubric (100 points)
   - Deliverables and submission guidelines
   - Testing checklist
   - Resources and tips
3. **Link from module intro.md**
4. **Add to sidebars.ts** under Assessments category

## Validation Checklist

Before committing content changes, verify:

- [ ] All filenames follow kebab-case convention
- [ ] All files have complete frontmatter
- [ ] Sidebar positions are unique within modules
- [ ] Files are added to sidebars.ts
- [ ] Internal links use relative paths
- [ ] Build succeeds: `npm run build`
- [ ] Navigation works correctly
- [ ] Search indexes new content

## Maintenance Notes

### Updating Lesson Order

To reorder lessons:
1. Update `sidebar_position` in frontmatter
2. Ensure positions are unique
3. Update sidebars.ts if categories changed
4. Test navigation flow

### Renaming Files

To rename files:
1. Use `git mv` to preserve history
2. Update all internal references
3. Update sidebars.ts
4. Update module intro links
5. Run build to catch broken links

### Archiving Content

To archive outdated content:
1. Move to `docs/archive/` directory
2. Remove from sidebars.ts
3. Add redirect in docusaurus.config.ts if needed
4. Document in MIGRATION_SUMMARY.md

## Related Documentation

- **README.md**: Project overview and setup instructions
- **MIGRATION_SUMMARY.md**: History of structural changes
- **sidebars.ts**: Sidebar configuration
- **docusaurus.config.ts**: Site configuration
- **specs/003-book-content-structure/**: Design specifications

## Contact

For questions about content structure or naming conventions:
- Open an issue on GitHub
- Refer to spec: `specs/003-book-content-structure/spec.md`
- Review ADRs in `history/adr/`

---

**Last Updated:** 2025-12-10
**Version:** 1.0
**Maintained By:** Course Development Team
