# Data Model: Book Content Structure

**Feature**: 003-book-content-structure
**Date**: 2025-12-10
**Phase**: Phase 1 - Design

## Overview

This document defines the conceptual data model for the Physical AI & Humanoid Robotics textbook content structure. While this is a documentation project (not a database-driven application), understanding the entities and their relationships is crucial for organizing files, configuring sidebars, and maintaining consistency.

## Core Entities

### 1. Module

Represents one of the four main learning modules in the course.

**Attributes**:
- `id`: string (e.g., "module-1-ros2", "module-2-gazebo-unity")
- `number`: integer (1-4)
- `title`: string (e.g., "The Robotic Nervous System (ROS 2)")
- `shortTitle`: string (e.g., "ROS 2")
- `description`: string (overview paragraph)
- `weekRange`: string (e.g., "Weeks 3-5")
- `weekStart`: integer (e.g., 3)
- `weekEnd`: integer (e.g., 5)
- `learningObjectives`: array of strings
- `prerequisites`: array of module IDs (optional)
- `directoryPath`: string (e.g., "docs/module-1-ros2")

**Relationships**:
- One Module contains multiple Lessons (1:N)
- One Module has one IntroPage (1:1)
- One Module contains multiple Assessments (1:N)

**Validation Rules**:
- `id` must follow pattern: `module-{number}-{kebab-case-name}`
- `number` must be unique and between 1-4
- `weekRange` must not overlap with other modules
- `directoryPath` must exist under `docs/`

**State Transitions**:
N/A (static content)

**File Mapping**:
- Directory: `docs/{id}/`
- Intro: `docs/{id}/intro.md`
- Sidebar entry in `sidebars.ts`

---

### 2. Lesson

Represents an individual lesson/content page within a module.

**Attributes**:
- `id`: string (e.g., "week-3-lesson-1-ros2-architecture")
- `title`: string (e.g., "Week 3 Lesson 1: ROS 2 Architecture")
- `sidebarLabel`: string (e.g., "Lesson 1: Architecture")
- `sidebarPosition`: integer (e.g., 31 for week 3, lesson 1)
- `week`: integer (1-13)
- `lessonNumber`: integer (1-N within a week)
- `description`: string (brief summary)
- `tags`: array of strings (e.g., ["ros2", "architecture", "week-3"])
- `moduleId`: string (foreign key to Module)
- `filePath`: string (e.g., "docs/module-1-ros2/week-3-lesson-1-ros2-architecture.md")
- `estimatedReadTime`: integer (minutes) - optional
- `content`: markdown string

**Relationships**:
- Many Lessons belong to one Module (N:1)
- One Lesson may reference other Lessons (prerequisites) (N:N)
- One Lesson appears in one WeekGroup (N:1)

**Validation Rules**:
- `id` must follow pattern: `week-{week}-lesson-{number}-{kebab-case-topic}`
- `week` must be within the module's week range
- `sidebarPosition` should be unique within module (week * 10 + lessonNumber)
- `filePath` must end with `.md`
- `tags` must include module identifier and week identifier

**Frontmatter Template**:
```yaml
---
title: "{title}"
sidebar_label: "{sidebarLabel}"
sidebar_position: {sidebarPosition}
description: "{description}"
tags: {tags}
---
```

**File Mapping**:
- File: `docs/{moduleId}/{id}.md`
- Sidebar entry in `sidebars.ts` under appropriate module and week category

---

### 3. IntroPage

Represents the landing/overview page for a module.

**Attributes**:
- `id`: string (always "intro")
- `title`: string (e.g., "Module 1: The Robotic Nervous System")
- `moduleId`: string (foreign key to Module)
- `overview`: string (markdown content)
- `learningObjectives`: array of strings
- `weekBreakdown`: array of objects with {week: int, topics: string[]}
- `prerequisites`: array of strings (module names or topics)
- `filePath`: string (e.g., "docs/module-1-ros2/intro.md")

**Relationships**:
- One IntroPage belongs to one Module (1:1)

**Validation Rules**:
- Must always be named `intro.md`
- Must appear first in module's sidebar items
- Must include: overview, learningObjectives, weekBreakdown sections

**Required Sections**:
1. Overview paragraph
2. Learning Objectives (bulleted list)
3. Weekly Breakdown (what's covered each week)
4. Prerequisites (if applicable)
5. Assessments summary

**File Mapping**:
- File: `docs/{moduleId}/intro.md`
- Sidebar: First item in module category or as `link` target

---

### 4. WeekGroup

Logical grouping of lessons by week (exists in sidebar configuration, not as files).

**Attributes**:
- `week`: integer (1-13)
- `label`: string (e.g., "Week 3: ROS 2 Fundamentals")
- `moduleId`: string (which module this week belongs to)
- `lessonIds`: array of lesson IDs in this week
- `sidebarCategory`: object (sidebar.ts category configuration)

**Relationships**:
- One WeekGroup belongs to one Module (N:1)
- One WeekGroup contains multiple Lessons (1:N)

**Validation Rules**:
- `week` must be within module's week range
- Week labels should be descriptive and indicate topic focus
- Lessons within a week should be ordered by `lessonNumber`

**Sidebar Pattern**:
```typescript
{
  type: 'category',
  label: 'Week {week}: {topic}',
  items: [
    'module-id/week-{week}-lesson-1-...',
    'module-id/week-{week}-lesson-2-...',
  ]
}
```

---

### 5. Assessment

Represents a project, lab, quiz, or other graded/checkpointed work.

**Attributes**:
- `id`: string (e.g., "ros2-package-project")
- `title`: string (e.g., "ROS 2 Package Development Project")
- `type`: enum ["project", "lab", "quiz", "capstone"]
- `moduleId`: string (foreign key to Module)
- `dueWeek`: integer (recommended completion week)
- `description`: string
- `requirements`: array of strings
- `rubric`: markdown string (grading criteria)
- `filePath`: string (e.g., "docs/module-1-ros2/assessments/ros2-package-project.md")

**Relationships**:
- Many Assessments belong to one Module (N:1)
- One Assessment may span multiple Lessons (N:N)

**Validation Rules**:
- `id` must be kebab-case and descriptive
- Must reside in `{moduleId}/assessments/` directory
- Should include: overview, requirements, deliverables, rubric

**Required Sections**:
1. Overview
2. Learning Objectives (what skills this assesses)
3. Requirements (what to build/do)
4. Deliverables (what to submit)
5. Rubric (grading criteria)
6. Resources (helpful links, templates)

**File Mapping**:
- File: `docs/{moduleId}/assessments/{id}.md`
- Sidebar: Under "Assessments" category within module

---

### 6. SetupGuide

Represents hardware/software setup and configuration documentation.

**Attributes**:
- `id`: string (e.g., "hardware-requirements", "software-setup")
- `title`: string
- `category`: enum ["hardware", "software", "infrastructure"]
- `description`: string
- `content`: markdown string
- `filePath`: string (e.g., "docs/setup/hardware-requirements.md")
- `targetAudience`: enum ["all", "students", "instructors", "admins"]

**Relationships**:
- Independent entity (not tied to modules)
- May be referenced by IntroPages or Lessons

**Validation Rules**:
- Must reside in `docs/setup/` directory
- Should be accessible before Module 1 starts
- Should include prerequisites and verification steps

**Types of Setup Guides**:
1. **hardware-requirements.md**: RTX workstations, Jetson kits, sensors
2. **software-setup.md**: Ubuntu, ROS 2, Isaac Sim installation
3. **lab-infrastructure.md**: On-premise vs cloud setup options
4. **student-kit-guide.md**: Economy Jetson student kit assembly

**File Mapping**:
- File: `docs/setup/{id}.md`
- Sidebar: Under "Setup & Resources" top-level category

---

### 7. Resource

Represents supplementary materials (glossary, references, additional reading).

**Attributes**:
- `id`: string (e.g., "glossary", "references")
- `title`: string
- `type`: enum ["glossary", "references", "additional-reading", "tools"]
- `description`: string
- `content`: markdown string
- `filePath`: string (e.g., "docs/resources/glossary.md")

**Relationships**:
- Independent entity
- May be cross-referenced from Lessons

**Validation Rules**:
- Must reside in `docs/resources/` directory
- Glossary should be alphabetically organized
- References should include proper citations

**File Mapping**:
- File: `docs/resources/{id}.md`
- Sidebar: Under "Setup & Resources → Resources" category

---

## Entity Relationship Diagram (ERD)

```
┌─────────────────┐
│     Module      │
│  (4 instances)  │
└────────┬────────┘
         │
         │ 1:1
         │
    ┌────▼────────┐
    │  IntroPage  │
    │ (1 per mod) │
    └─────────────┘
         │
         │ 1:N
    ┌────▼──────────┐
    │   WeekGroup   │
    │ (weeks 1-13)  │
    └────┬──────────┘
         │
         │ 1:N
    ┌────▼────────┐
    │   Lesson    │
    │ (50+ total) │
    └─────────────┘

Module ──1:N──> Assessment

[Independent Entities]
SetupGuide (4-5 guides)
Resource (3-4 resources)
```

---

## Sidebar Configuration Model

The sidebar structure in `sidebars.ts` mirrors the entity relationships:

```typescript
type SidebarConfig = {
  textbookSidebar: SidebarItem[]
}

type SidebarItem =
  | DocItem          // Direct link to a document
  | CategoryItem     // Group of items

type CategoryItem = {
  type: 'category'
  label: string
  collapsed?: boolean
  link?: { type: 'doc', id: string }
  items: SidebarItem[]
}

type DocItem = string  // Document ID (path without docs/ and .md)
```

**Hierarchy**:
1. **Level 1**: Module categories (4 total)
2. **Level 2**: Week categories within modules (variable per module)
3. **Level 3**: Lesson documents (1-3 per week typically)
4. **Level 1**: Setup & Resources category (separate from modules)

---

## File System to Entity Mapping

### Directory Structure:
```
docs/
├── intro/
│   ├── intro.md                          → IntroPage (course-level)
│   ├── week-1-2-physical-ai-foundations.md  → Lesson
│   └── week-1-2-sensors-overview.md         → Lesson
│
├── module-1-ros2/                        → Module
│   ├── intro.md                          → IntroPage
│   ├── week-3-lesson-1-ros2-architecture.md → Lesson
│   ├── week-3-lesson-2-nodes-topics.md      → Lesson
│   └── assessments/
│       └── ros2-package-project.md       → Assessment
│
├── setup/
│   └── hardware-requirements.md          → SetupGuide
│
└── resources/
    └── glossary.md                       → Resource
```

### Entity Counts:
- **Modules**: 4 (ROS 2, Gazebo/Unity, Isaac, VLA)
- **IntroPages**: 5 (1 course intro + 4 module intros)
- **WeekGroups**: ~13 (mapped to sidebar categories)
- **Lessons**: 50+ (estimated 10-15 per module)
- **Assessments**: 4 (1 per module)
- **SetupGuides**: 4-5
- **Resources**: 3-4

---

## Validation Rules Summary

### Global Rules:
1. All file paths must use kebab-case
2. All `.md` files must have valid frontmatter
3. No orphaned files (all must be referenced in sidebars.ts)
4. No duplicate `sidebar_position` values within a module

### Module-Specific Rules:
1. Each module must have exactly one `intro.md`
2. Week ranges must not overlap between modules
3. Directory names must match module IDs

### Lesson-Specific Rules:
1. Filename must match pattern: `week-{N}-lesson-{M}-{topic}.md`
2. `sidebar_position` = week * 10 + lesson number (e.g., 31, 32, 41, 42)
3. Must include at least: title, sidebar_label, sidebar_position

### Sidebar Rules:
1. Maximum depth: 3 levels (Module → Week → Lesson)
2. Week categories should be collapsible
3. Module categories should include link to intro page

---

## Content Schema (TypeScript Types)

```typescript
// Frontmatter schema for Lesson files
interface LessonFrontmatter {
  title: string;                    // Required
  sidebar_label: string;             // Required
  sidebar_position: number;          // Required
  description?: string;              // Recommended
  tags?: string[];                   // Recommended
  estimated_read_time?: number;      // Optional
}

// Module configuration
interface Module {
  id: string;                        // e.g., "module-1-ros2"
  number: 1 | 2 | 3 | 4;
  title: string;
  shortTitle: string;
  weekRange: string;                 // e.g., "Weeks 3-5"
  weekStart: number;
  weekEnd: number;
  directoryPath: string;
}

// Sidebar category for week grouping
interface WeekCategory {
  type: 'category';
  label: string;                     // e.g., "Week 3: ROS 2 Fundamentals"
  items: string[];                   // Lesson IDs
  collapsed?: boolean;
}
```

---

## Migration Mapping

Current structure → New structure:

| Current Path | New Path | Entity Type |
|--------------|----------|-------------|
| `docs/intro.md` | `docs/intro/intro.md` | IntroPage (course) |
| `docs/ros2/intro.md` | `docs/module-1-ros2/intro.md` | IntroPage |
| `docs/ros2/architecture.md` | `docs/module-1-ros2/week-3-lesson-1-ros2-architecture.md` | Lesson |
| `docs/ros2/nodes-packages.md` | `docs/module-1-ros2/week-3-lesson-2-nodes-packages.md` | Lesson |
| `docs/gazebo-unity/intro.md` | `docs/module-2-gazebo-unity/intro.md` | IntroPage |
| New | `docs/setup/hardware-requirements.md` | SetupGuide |
| New | `docs/module-1-ros2/assessments/ros2-package-project.md` | Assessment |

---

## Summary

The data model defines 7 core entities organized in a hierarchical structure:
- **Modules** contain **IntroPages**, **WeekGroups**, **Lessons**, and **Assessments**
- **SetupGuides** and **Resources** are independent top-level entities
- The file system structure directly mirrors these entity relationships
- Sidebar configuration in `sidebars.ts` provides the navigation layer over these entities

All entities follow strict naming conventions and validation rules to ensure consistency and maintainability.
