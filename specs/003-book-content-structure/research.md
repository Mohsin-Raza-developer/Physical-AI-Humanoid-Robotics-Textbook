# Research: Book Content Structure

**Feature**: 003-book-content-structure
**Date**: 2025-12-10
**Phase**: Phase 0 - Research & Discovery

## Research Questions

### Q1: What is the optimal Docusaurus directory structure for a 13-week course with 4 modules?

**Decision**: Flat module-based structure with week-based file naming

**Rationale**:
- Docusaurus works best with flat directory structures (1-2 levels deep max)
- Weekly file naming (`week-X-lesson-Y-`) makes chronological navigation intuitive
- Module directories (`module-1-ros2/`, `module-2-gazebo-unity/`, etc.) provide clear organization
- Avoids deep nesting that complicates sidebar configuration and URL structure

**Alternatives Considered**:
- **Nested week folders** (`module-1-ros2/week-3/lesson-1.md`): Rejected because it creates 3-level nesting, complicates sidebar.ts, and creates longer URLs
- **Flat with module prefixes** (`ros2-week-3-lesson-1.md` all in `/docs/`): Rejected because it loses logical grouping and makes file management harder
- **Separate docs instance per module**: Rejected as over-engineering for a single textbook

**Implementation Notes**:
- Keep all module content under `/docs/` for single docs instance
- Use `intro.md` consistently as module landing page
- Group by module first, then chronologically by week

---

### Q2: How should weekly breakdown (Weeks 1-13) map to sidebar navigation?

**Decision**: Hierarchical sidebar with collapsible week categories within each module

**Rationale**:
- Students need to understand "where they are" in the 13-week progression
- Collapsible categories prevent sidebar from being overwhelming
- Week labels in sidebar categories make it easy to find content for specific class sessions
- Docusaurus sidebar categories support `link` property for module intro pages

**Alternatives Considered**:
- **Flat lesson list per module**: Rejected because 6+ lessons per module creates visual clutter
- **Sequential numbering only** (1-50): Rejected because it loses weekly context important for instructors
- **Calendar-based** (Jan Week 1, Jan Week 2): Rejected because course timing varies by institution

**Implementation Pattern**:
```typescript
{
  type: 'category',
  label: 'Module 1: ROS 2 (Weeks 3-5)',
  items: [
    'module-1-ros2/intro',
    {
      type: 'category',
      label: 'Week 3: ROS 2 Fundamentals',
      items: ['week-3-lesson-1', 'week-3-lesson-2']
    }
  ]
}
```

---

### Q3: What frontmatter fields are essential for lesson pages?

**Decision**: Minimal required fields + optional enhancement fields

**Rationale**:
- Docusaurus requires `title` and benefits from `sidebar_label` and `sidebar_position`
- `description` improves SEO and social sharing
- `tags` enable cross-module topic grouping (e.g., all "ROS 2" content, all "Week 3" content)
- Keep it simple to encourage consistent usage

**Required Fields**:
```yaml
---
title: "Week X Lesson Y: Full Descriptive Title"
sidebar_label: "Lesson Y: Short Title"
sidebar_position: XY  # Week number concatenated with lesson number
---
```

**Optional Enhancement Fields**:
```yaml
description: "Brief lesson summary for SEO"
tags: [module-name, week-X, topic-keywords]
```

**Alternatives Considered**:
- **Extensive metadata** (author, date, prerequisites, learning outcomes): Rejected as too burdensome for every file
- **No frontmatter** (rely on file naming): Rejected because Docusaurus needs explicit titles
- **Custom frontmatter fields** (estimated_time, difficulty): Rejected to avoid complexity

---

### Q4: How to handle hardware requirements and setup content?

**Decision**: Dedicated `/docs/setup/` section accessible from main navigation

**Rationale**:
- Hardware requirements are critical context students need before Week 1
- Setup is a distinct phase from learning modules (one-time vs. ongoing)
- Dedicated section makes it easy to reference from external sources
- Can be updated independently of course content

**Structure**:
```
docs/setup/
├── hardware-requirements.md    # RTX workstations, Jetson kits, sensors
├── software-setup.md          # Ubuntu, ROS 2, Isaac Sim installation
├── lab-infrastructure.md      # On-premise vs cloud options
└── student-kit-guide.md       # Economy Jetson setup
```

**Alternatives Considered**:
- **Embed in Module 1 intro**: Rejected because it's needed before Module 1 starts
- **Top-level "Getting Started" section**: Too generic; "Setup" is clearer
- **External wiki**: Rejected to keep all content in one place

---

### Q5: How to organize assessment and project files?

**Decision**: `/assessments/` subdirectory within each module directory

**Rationale**:
- Projects/assessments are tied to specific modules
- Subdirectory keeps them organized but separate from lesson content
- Easy to find all assessments for grading/review purposes
- Follows existing Docusaurus patterns (like `/api/` subdirectories)

**Structure per Module**:
```
module-1-ros2/
├── intro.md
├── week-3-lesson-1.md
├── week-3-lesson-2.md
...
└── assessments/
    └── ros2-package-project.md
```

**Alternatives Considered**:
- **Top-level `/docs/assessments/`**: Rejected because it separates assessments from module context
- **Inline in lesson files**: Rejected because projects span multiple lessons
- **Separate docs instance**: Over-engineering for 4 assessments

---

### Q6: How to handle migration from existing structure?

**Decision**: Rename directories and files, use git mv to preserve history

**Rationale**:
- Current structure has `docs/ros2/`, `docs/gazebo-unity/`, `docs/isaac/`, `docs/vla/`
- Need to rename to `docs/module-1-ros2/`, etc. for clarity
- Need to rename files to follow `week-X-lesson-Y-` pattern
- Git mv preserves file history better than delete + create

**Migration Steps**:
1. Rename directories: `git mv docs/ros2 docs/module-1-ros2`
2. Rename files: `git mv docs/module-1-ros2/architecture.md docs/module-1-ros2/week-3-lesson-1-ros2-architecture.md`
3. Update sidebars.ts with new paths and week categories
4. Update docusaurus.config.ts if needed
5. Test build and fix broken links

**Alternatives Considered**:
- **Create new structure, copy content**: Rejected because it loses git history
- **Gradual migration**: Rejected because partial migration creates confusion
- **Keep old structure, add new**: Rejected as technical debt

---

### Q7: What Docusaurus features should be leveraged?

**Decision**: Use built-in features; avoid custom plugins initially

**Rationale**:
- Docusaurus provides rich features out-of-box: versioning, search, i18n
- Custom plugins add maintenance burden
- Start simple, add complexity only when needed
- Premium UI/UX enhancements (from spec 002) already provide visual polish

**Features to Use**:
- ✅ Sidebar categories with collapsible sections
- ✅ Frontmatter for metadata
- ✅ Tags for cross-module topic organization
- ✅ Admonitions (:::tip, :::warning) for callouts
- ✅ Code blocks with syntax highlighting
- ✅ MDX for interactive components (if needed later)

**Features to Avoid Initially**:
- ❌ Docs versioning (single version for now)
- ❌ i18n (English only initially)
- ❌ Custom sidebar item types
- ❌ Custom page layouts

---

## Technology Decisions

### Docusaurus Version
- **Version**: 3.6.3 (current stable as of Dec 2024)
- **Rationale**: Future flags enabled for v4 compatibility, stable feature set

### File Format
- **Format**: Markdown (.md) with optional MDX
- **Rationale**: Simplicity for content authors, MDX available for interactive components if needed

### URL Structure
- **Pattern**: `/docs/module-X-name/week-Y-lesson-Z-topic`
- **Example**: `/docs/module-1-ros2/week-3-lesson-1-ros2-architecture`
- **Rationale**: Clean, readable URLs that reflect content hierarchy

### Navigation Pattern
- **Pattern**: Hierarchical with breadcrumbs, next/prev buttons, TOC
- **Rationale**: Multiple navigation paths support different learning styles

---

## Risk Mitigation

### Risk 1: Breaking existing links during migration
**Mitigation**:
- Document all URL changes
- Use Docusaurus redirects plugin if needed
- Test thoroughly before merging
- Update any external references

### Risk 2: Sidebar becomes too deep/complex
**Mitigation**:
- Keep hierarchy to 3 levels max: Module → Week → Lesson
- Use collapsible categories (collapsed by default except current)
- Test navigation UX with real users

### Risk 3: Inconsistent content structure across modules
**Mitigation**:
- Provide clear templates for lesson files
- Document file naming conventions in README
- Use linting/validation for frontmatter
- Establish review process

---

## Next Steps (Phase 1)

1. **Create data-model.md**: Document content entities (Module, Lesson, Assessment, Setup Guide)
2. **Create contracts/**: Not applicable for documentation project (no APIs)
3. **Create quickstart.md**: Guide for content authors on creating new lessons
4. **Update agent context**: Add Docusaurus-specific patterns to AI agent context

---

## Open Questions for Clarification

1. ✅ **Resolved**: Should labs be separate from lessons? → Integrated into lesson files
2. ✅ **Resolved**: Version control for course variations? → Single version initially, can add versioning later if needed
3. ✅ **Resolved**: Placeholder files for all 50+ lessons upfront? → Create incrementally as content is written
4. ⚠️ **Needs User Input**: Should there be a "Resources" section for external links, papers, tools? → Proposed `docs/resources/` section

---

## Research Summary

All technical unknowns have been resolved. The feature is ready for Phase 1 design work.

**Key Decisions**:
- Flat module-based directory structure
- Week-based file naming with descriptive titles
- Hierarchical sidebar with collapsible week categories
- Dedicated setup section for hardware/software
- Assessments subdirectories within modules
- Git mv for migration to preserve history
- Use built-in Docusaurus features, avoid custom plugins initially
