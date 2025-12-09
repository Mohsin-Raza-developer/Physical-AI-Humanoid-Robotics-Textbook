# Implementation Plan: Book Content Structure

**Branch**: `003-book-content-structure` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-book-content-structure/spec.md`

## Summary

Restructure the Physical AI & Humanoid Robotics textbook content to follow a consistent, maintainable organization system that maps the 13-week course structure to Docusaurus pages. This includes:

- Organizing content into 4 module directories with clear naming conventions
- Implementing week-based file naming for chronological navigation
- Creating hierarchical sidebar structure with collapsible week categories
- Migrating existing content from current structure to new structure
- Adding setup guides, assessments, and resource sections
- Providing quickstart guide for future content authors

**Technical Approach**: File system reorganization with git mv to preserve history, sidebar configuration updates, and documentation-driven templates for consistency.

## Technical Context

**Language/Version**: Markdown/MDX, TypeScript 5.x (for Docusaurus config)
**Primary Dependencies**: Docusaurus 3.6.3, React 18.x, Node.js 18+
**Storage**: File system (Markdown files in `docs/` directory)
**Testing**: Build validation (`npm run build`), link checking, sidebar structure validation
**Target Platform**: Static site deployment (GitHub Pages)
**Project Type**: Documentation/static site (not single/web/mobile app)
**Performance Goals**: Build time < 60 seconds, page load < 2 seconds
**Constraints**: Maintain existing premium UI/UX from spec 002, preserve git history during migration, zero broken links
**Scale/Scope**: 50+ lesson files, 4 modules, 13 weeks, 4 assessments, 5+ setup guides

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Status**: N/A - Constitution file is currently a template placeholder. For this documentation project, we follow these principles:

✅ **Simplicity**: Flat module structure, minimal nesting
✅ **Consistency**: Strict file naming conventions (`week-X-lesson-Y-topic.md`)
✅ **Maintainability**: Templates and quickstart guide for content authors
✅ **No Breaking Changes**: Migration preserves git history with `git mv`
✅ **Documentation-First**: Comprehensive spec, research, data model, and quickstart

**Re-evaluated Post-Design**: ✅ All principles maintained

## Project Structure

### Documentation (this feature)

```text
specs/003-book-content-structure/
├── spec.md              # Feature specification (user stories, requirements)
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0: Research decisions and rationale
├── data-model.md        # Phase 1: Content entity model
├── quickstart.md        # Phase 1: Content author guide
└── tasks.md             # Phase 2: Executable tasks (created by /sp.tasks)
```

### Source Code (repository root)

**Structure Decision**: Documentation project with Docusaurus static site

```text
docs/
├── intro/                              # Weeks 1-2: Introduction
│   ├── intro.md
│   ├── week-1-2-physical-ai-foundations.md
│   ├── week-1-2-sensors-overview.md
│   └── hardware-requirements.md
│
├── module-1-ros2/                      # Weeks 3-5: ROS 2
│   ├── intro.md
│   ├── week-3-lesson-1-ros2-architecture.md
│   ├── week-3-lesson-2-nodes-topics.md
│   ├── week-4-lesson-1-services-actions.md
│   ├── week-4-lesson-2-building-packages.md
│   ├── week-5-lesson-1-launch-files.md
│   ├── week-5-lesson-2-urdf-humanoids.md
│   └── assessments/
│       └── ros2-package-project.md
│
├── module-2-gazebo-unity/              # Weeks 6-7: Simulation
│   ├── intro.md
│   ├── week-6-lesson-1-gazebo-setup.md
│   ├── week-6-lesson-2-urdf-sdf.md
│   ├── week-7-lesson-1-physics-simulation.md
│   ├── week-7-lesson-2-unity-robotics.md
│   ├── week-7-lesson-3-sensor-simulation.md
│   └── assessments/
│       └── gazebo-simulation-project.md
│
├── module-3-isaac/                     # Weeks 8-10: NVIDIA Isaac
│   ├── intro.md
│   ├── week-8-lesson-1-isaac-sdk-intro.md
│   ├── week-8-lesson-2-isaac-sim-setup.md
│   ├── week-9-lesson-1-ai-perception.md
│   ├── week-9-lesson-2-manipulation.md
│   ├── week-10-lesson-1-reinforcement-learning.md
│   ├── week-10-lesson-2-sim-to-real.md
│   └── assessments/
│       └── isaac-perception-pipeline.md
│
├── module-4-vla/                       # Weeks 11-13: VLA & Capstone
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
├── setup/                              # Hardware/Software Setup
│   ├── hardware-requirements.md
│   ├── software-setup.md
│   ├── lab-infrastructure.md
│   └── student-kit-guide.md
│
└── resources/                          # Supplementary Materials
    ├── glossary.md
    ├── references.md
    └── additional-reading.md

sidebars.ts                             # Sidebar configuration
docusaurus.config.ts                    # Docusaurus configuration
```

## Complexity Tracking

*No constitution violations to justify. This is a straightforward documentation reorganization.*

## Implementation Phases

### Phase 0: Research & Discovery ✅ COMPLETED

**Output**: `research.md`

**Key Decisions Made**:
1. ✅ Flat module-based directory structure (vs. nested week folders)
2. ✅ Week-based file naming pattern (`week-X-lesson-Y-topic.md`)
3. ✅ Hierarchical sidebar with collapsible week categories
4. ✅ Dedicated `/docs/setup/` section for hardware/software guides
5. ✅ Assessments in subdirectories within each module
6. ✅ Git mv for migration to preserve history
7. ✅ Use built-in Docusaurus features, avoid custom plugins initially

**Research Questions Resolved**:
- Optimal directory structure for 13-week course
- Sidebar navigation mapping to weekly breakdown
- Essential frontmatter fields for lesson pages
- Organization of hardware requirements and setup content
- Assessment and project file organization
- Migration strategy from existing structure
- Docusaurus features to leverage

---

### Phase 1: Design & Contracts ✅ COMPLETED

**Output**: `data-model.md`, `quickstart.md`

#### 1.1 Data Model

**Entities Defined**:
1. **Module** - One of 4 main learning modules
2. **Lesson** - Individual content page within a module
3. **IntroPage** - Landing/overview page for each module
4. **WeekGroup** - Logical grouping of lessons by week (sidebar only)
5. **Assessment** - Projects, labs, quizzes for each module
6. **SetupGuide** - Hardware/software configuration docs
7. **Resource** - Supplementary materials (glossary, references)

**Relationships**:
- Module (1) → IntroPage (1)
- Module (1) → Lessons (N)
- Module (1) → WeekGroups (N)
- Module (1) → Assessments (N)
- WeekGroup (1) → Lessons (N)

**Validation Rules**:
- File naming: `week-X-lesson-Y-topic.md`
- Frontmatter: title, sidebar_label, sidebar_position required
- sidebar_position = week * 10 + lesson number
- No duplicate sidebar positions within a module
- All files must be referenced in sidebars.ts

#### 1.2 Contracts

**N/A** - This is a documentation project with no API contracts. The "contract" is the file system structure and sidebar configuration.

#### 1.3 Quickstart Guide

**Created**: Comprehensive guide for content authors covering:
- File naming conventions
- Frontmatter templates
- Directory structure reference
- Step-by-step lesson creation workflow
- Assessment creation workflow
- Setup guide creation workflow
- Sidebar configuration examples
- Testing checklist
- Troubleshooting common issues
- Content writing best practices

---

### Phase 2: Tasks Generation

**To be completed by**: `/sp.tasks` command

**Expected Output**: `tasks.md` with dependency-ordered implementation tasks

**Key Task Categories**:
1. **Migration**: Rename existing directories and files
2. **Structure Creation**: Create new directories (intro/, setup/, resources/)
3. **Content Extraction**: Extract hardware requirements from textbook
4. **Sidebar Updates**: Modify sidebars.ts with new structure
5. **Content Population**: Create new lesson files, intro pages, assessments
6. **Validation**: Build testing, link checking
7. **Documentation**: Update README with structure guide

---

## Migration Strategy

### From Current Structure → New Structure

| Current Path | New Path | Action |
|--------------|----------|--------|
| `docs/intro.md` | `docs/intro/intro.md` | Create new directory, move file |
| `docs/ros2/` | `docs/module-1-ros2/` | `git mv docs/ros2 docs/module-1-ros2` |
| `docs/ros2/intro.md` | `docs/module-1-ros2/intro.md` | No change needed |
| `docs/ros2/architecture.md` | `docs/module-1-ros2/week-3-lesson-1-ros2-architecture.md` | `git mv` with new name |
| `docs/ros2/nodes-packages.md` | `docs/module-1-ros2/week-3-lesson-2-nodes-packages.md` | `git mv` with new name |
| `docs/gazebo-unity/` | `docs/module-2-gazebo-unity/` | `git mv` |
| `docs/isaac/` | `docs/module-3-isaac/` | `git mv` |
| `docs/vla/` | `docs/module-4-vla/` | `git mv` |
| N/A | `docs/setup/` | Create new directory |
| N/A | `docs/resources/` | Create new directory |
| N/A | `docs/*/assessments/` | Create new subdirectories |

### Existing Content Inventory

**Current files to migrate**:
```
docs/intro.md
docs/ros2/intro.md
docs/ros2/architecture.md
docs/ros2/nodes-packages.md
docs/ros2/communication.md
docs/ros2/launch.md
docs/ros2/ros2-physical-ai.md
docs/gazebo-unity/intro.md
docs/gazebo-unity/gazebo-fundamentals.md
docs/gazebo-unity/unity-robotics.md
docs/gazebo-unity/physics-sensors.md
docs/gazebo-unity/simulation-physical-ai.md
docs/isaac/intro.md
docs/isaac/isaac-sim.md
docs/isaac/perception-pipelines.md
docs/isaac/nav-manipulation.md
docs/isaac/isaac-physical-ai.md
docs/vla/intro.md
docs/vla/vla-architectures.md
docs/vla/training-vla.md
docs/vla/vla-control.md
docs/vla/vla-physical-ai.md
```

**Mapping to weeks**:
- ROS 2 files → Weeks 3-5 lessons
- Gazebo/Unity files → Weeks 6-7 lessons
- Isaac files → Weeks 8-10 lessons
- VLA files → Weeks 11-13 lessons

---

## Sidebar Configuration

### New Structure (sidebars.ts)

```typescript
const sidebars: SidebarsConfig = {
  textbookSidebar: [
    // Introduction (Weeks 1-2)
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: [
        'intro/intro',
        'intro/week-1-2-physical-ai-foundations',
        'intro/week-1-2-sensors-overview',
        'intro/hardware-requirements',
      ],
    },

    // Module 1: ROS 2 (Weeks 3-5)
    {
      type: 'category',
      label: 'Module 1: ROS 2 (Weeks 3-5)',
      collapsed: true,
      link: {
        type: 'doc',
        id: 'module-1-ros2/intro',
      },
      items: [
        'module-1-ros2/intro',
        {
          type: 'category',
          label: 'Week 3: ROS 2 Fundamentals',
          items: [
            'module-1-ros2/week-3-lesson-1-ros2-architecture',
            'module-1-ros2/week-3-lesson-2-nodes-topics',
          ],
        },
        {
          type: 'category',
          label: 'Week 4: Advanced Concepts',
          items: [
            'module-1-ros2/week-4-lesson-1-services-actions',
            'module-1-ros2/week-4-lesson-2-building-packages',
          ],
        },
        {
          type: 'category',
          label: 'Week 5: URDF & Launch',
          items: [
            'module-1-ros2/week-5-lesson-1-launch-files',
            'module-1-ros2/week-5-lesson-2-urdf-humanoids',
          ],
        },
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module-1-ros2/assessments/ros2-package-project',
          ],
        },
      ],
    },

    // Module 2: Gazebo & Unity (Weeks 6-7)
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity (Weeks 6-7)',
      collapsed: true,
      link: {
        type: 'doc',
        id: 'module-2-gazebo-unity/intro',
      },
      items: [
        'module-2-gazebo-unity/intro',
        {
          type: 'category',
          label: 'Week 6: Gazebo Fundamentals',
          items: [
            'module-2-gazebo-unity/week-6-lesson-1-gazebo-setup',
            'module-2-gazebo-unity/week-6-lesson-2-urdf-sdf',
          ],
        },
        {
          type: 'category',
          label: 'Week 7: Physics & Unity',
          items: [
            'module-2-gazebo-unity/week-7-lesson-1-physics-simulation',
            'module-2-gazebo-unity/week-7-lesson-2-unity-robotics',
            'module-2-gazebo-unity/week-7-lesson-3-sensor-simulation',
          ],
        },
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module-2-gazebo-unity/assessments/gazebo-simulation-project',
          ],
        },
      ],
    },

    // Module 3: NVIDIA Isaac (Weeks 8-10)
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac (Weeks 8-10)',
      collapsed: true,
      link: {
        type: 'doc',
        id: 'module-3-isaac/intro',
      },
      items: [
        'module-3-isaac/intro',
        {
          type: 'category',
          label: 'Week 8: Isaac Platform',
          items: [
            'module-3-isaac/week-8-lesson-1-isaac-sdk-intro',
            'module-3-isaac/week-8-lesson-2-isaac-sim-setup',
          ],
        },
        {
          type: 'category',
          label: 'Week 9: Perception & Manipulation',
          items: [
            'module-3-isaac/week-9-lesson-1-ai-perception',
            'module-3-isaac/week-9-lesson-2-manipulation',
          ],
        },
        {
          type: 'category',
          label: 'Week 10: RL & Sim-to-Real',
          items: [
            'module-3-isaac/week-10-lesson-1-reinforcement-learning',
            'module-3-isaac/week-10-lesson-2-sim-to-real',
          ],
        },
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module-3-isaac/assessments/isaac-perception-pipeline',
          ],
        },
      ],
    },

    // Module 4: VLA & Capstone (Weeks 11-13)
    {
      type: 'category',
      label: 'Module 4: VLA & Capstone (Weeks 11-13)',
      collapsed: true,
      link: {
        type: 'doc',
        id: 'module-4-vla/intro',
      },
      items: [
        'module-4-vla/intro',
        {
          type: 'category',
          label: 'Week 11: Humanoid Robotics',
          items: [
            'module-4-vla/week-11-lesson-1-humanoid-kinematics',
            'module-4-vla/week-11-lesson-2-bipedal-locomotion',
          ],
        },
        {
          type: 'category',
          label: 'Week 12: Advanced Interaction',
          items: [
            'module-4-vla/week-12-lesson-1-manipulation-grasping',
            'module-4-vla/week-12-lesson-2-human-robot-interaction',
          ],
        },
        {
          type: 'category',
          label: 'Week 13: Conversational AI & Capstone',
          items: [
            'module-4-vla/week-13-lesson-1-conversational-robotics',
            'module-4-vla/week-13-lesson-2-gpt-integration',
            'module-4-vla/week-13-lesson-3-capstone-project',
          ],
        },
        {
          type: 'category',
          label: 'Assessments',
          items: [
            'module-4-vla/assessments/capstone-simulated-humanoid',
          ],
        },
      ],
    },

    // Setup & Resources
    {
      type: 'category',
      label: 'Setup & Resources',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Hardware & Software Setup',
          items: [
            'setup/hardware-requirements',
            'setup/software-setup',
            'setup/lab-infrastructure',
            'setup/student-kit-guide',
          ],
        },
        {
          type: 'category',
          label: 'Resources',
          items: [
            'resources/glossary',
            'resources/references',
            'resources/additional-reading',
          ],
        },
      ],
    },
  ],
};
```

---

## Testing & Validation Strategy

### Build Testing
```bash
npm run build  # Must succeed with 0 errors, 0 warnings
```

**Expected Checks**:
- All markdown files compile
- No broken internal links
- No duplicate sidebar positions
- All referenced files exist
- Frontmatter is valid YAML

### Manual Testing Checklist
- [ ] Navigate through all 4 modules
- [ ] Verify week labels are correct
- [ ] Check next/previous navigation works
- [ ] Verify breadcrumbs show correct hierarchy
- [ ] Test search functionality
- [ ] Verify mobile responsiveness
- [ ] Check dark mode compatibility
- [ ] Validate all images load
- [ ] Verify table of contents generates correctly

### Link Validation
- Use Docusaurus built-in link checker
- Verify external links (if any) are valid
- Check that all cross-references work

---

## Risk Analysis

### Risk 1: Breaking Existing Links
**Impact**: High
**Probability**: Medium
**Mitigation**:
- Use git mv to preserve history
- Test build before merging
- Document all URL changes
- Consider Docusaurus redirects if needed

### Risk 2: Sidebar Becomes Too Complex
**Impact**: Medium
**Probability**: Low
**Mitigation**:
- Limit hierarchy to 3 levels max
- Use collapsible categories
- Test with real users
- Iterate based on feedback

### Risk 3: Inconsistent Content Structure
**Impact**: Medium
**Probability**: Medium
**Mitigation**:
- Provide clear templates
- Document conventions in quickstart.md
- Establish review process
- Use validation scripts

---

## Success Criteria

### Phase Completion Criteria

**Phase 0 (Research)**: ✅ COMPLETED
- [x] All technical unknowns resolved
- [x] Research decisions documented with rationale
- [x] Alternatives considered and rejected reasons documented

**Phase 1 (Design)**: ✅ COMPLETED
- [x] Data model defines all 7 entities with relationships
- [x] Validation rules established for each entity
- [x] Quickstart guide provides clear workflows
- [x] Sidebar structure fully specified

**Phase 2 (Tasks)**: To be completed by `/sp.tasks`
- [ ] Tasks.md created with all implementation steps
- [ ] Tasks ordered by dependencies
- [ ] Each task has clear acceptance criteria
- [ ] Tasks map to success criteria from spec

### Overall Success Criteria (from spec.md)

- **SC-001**: All 4 modules have complete directory structures ✅ Designed
- **SC-002**: Sidebar navigation includes all modules ✅ Designed
- **SC-003**: Every module has intro.md ✅ Designed
- **SC-004**: File naming 100% consistent ✅ Conventions established
- **SC-005**: Next/previous navigation sequences correctly ✅ Sidebar structure supports this
- **SC-006**: Documentation includes structure guide ✅ quickstart.md created
- **SC-007**: Build completes without broken links ⏳ To be validated in implementation
- **SC-008**: All textbook content mapped ✅ Mapping documented in research.md

---

## Handoff to Implementation Phase

### What's Complete
✅ Research phase (research.md)
✅ Design phase (data-model.md, quickstart.md)
✅ Agent context updated
✅ Implementation plan documented (this file)

### Next Steps
1. Run `/sp.tasks` to generate tasks.md
2. Review and approve tasks
3. Execute tasks in dependency order
4. Validate with build and manual testing
5. Create pull request for review

### Key Artifacts Created
- `specs/003-book-content-structure/spec.md` - Feature specification
- `specs/003-book-content-structure/research.md` - Research decisions
- `specs/003-book-content-structure/data-model.md` - Content entity model
- `specs/003-book-content-structure/quickstart.md` - Content author guide
- `specs/003-book-content-structure/plan.md` - This implementation plan

### Branch Status
- Current branch: `003-book-content-structure`
- Base branch: `main`
- Ready for: Task generation (`/sp.tasks`)

---

**Plan Status**: ✅ COMPLETED
**Next Command**: `/sp.tasks` to generate implementation tasks
**Estimated Implementation Effort**: Medium (10-15 tasks, mostly file operations)
