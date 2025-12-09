# Feature Specification: Book Content Structure

**Feature Branch**: `003-book-content-structure`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Create a new specification. Input: @Physical-AI-Humanoid-Robotics-Textbook.md Goal: Define the content structure for the entire book. Include: Folder structure for all 4 Modules, File naming conventions, Sidebar organization, How content from the textbook maps to Docusaurus pages."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Complete Book Structure Navigation (Priority: P1)

Students and instructors need to navigate through all 4 modules of the Physical AI & Humanoid Robotics textbook with a clear, intuitive structure that maps weekly lessons to organized content pages.

**Why this priority**: This is the foundation of the entire textbook. Without proper structure, content cannot be organized or navigated effectively.

**Independent Test**: Can be fully tested by navigating through the sidebar structure from Module 1 (ROS 2) through Module 4 (VLA), verifying all links work and content is logically organized by week and topic.

**Acceptance Scenarios**:

1. **Given** a student visits the textbook homepage, **When** they view the sidebar, **Then** they see 4 main modules: "Introduction", "Module 1: ROS 2", "Module 2: Gazebo & Unity", "Module 3: NVIDIA Isaac", and "Module 4: VLA"
2. **Given** a student is on any lesson page, **When** they use next/previous navigation, **Then** they follow the logical sequence of the 13-week course structure
3. **Given** an instructor wants to reference Week 3 content, **When** they navigate to Module 1 (ROS 2), **Then** they find lessons organized by weeks 3-5

---

### User Story 2 - Consistent File Naming and Organization (Priority: P1)

Developers need a clear, predictable file naming convention that makes it easy to locate, create, and maintain content files across all modules.

**Why this priority**: Consistency in file organization is critical for maintainability, collaboration, and preventing confusion as the textbook grows.

**Independent Test**: Can be tested by examining the file structure, verifying that all files follow the naming pattern (intro.md, lesson-N.md, week-N.md) and are organized in their respective module directories.

**Acceptance Scenarios**:

1. **Given** a developer needs to add Week 4 content for ROS 2, **When** they navigate to `docs/module-1-ros2/`, **Then** they find existing lesson files following the pattern `week-3-lesson-1.md`, `week-4-lesson-1.md`
2. **Given** a content author creates a new lesson file, **When** they follow the naming convention, **Then** the file automatically appears in the correct sidebar position
3. **Given** a developer searches for introduction content, **When** they look in any module folder, **Then** they always find `intro.md` as the module overview

---

### User Story 3 - Weekly Breakdown Mapping (Priority: P2)

Instructors need to see how the textbook's 13-week structure maps to specific pages and lessons, enabling them to plan their teaching schedule.

**Why this priority**: The course is designed around a 13-week quarter structure. Clear mapping ensures the textbook supports the intended pedagogical flow.

**Independent Test**: Can be tested by reviewing the sidebar structure and verifying that weeks 1-2 map to Introduction, weeks 3-5 to Module 1, weeks 6-7 to Module 2, weeks 8-10 to Module 3, and weeks 11-13 to Module 4.

**Acceptance Scenarios**:

1. **Given** an instructor is planning Week 1-2 lessons, **When** they navigate to the Introduction section, **Then** they find content covering "Foundations of Physical AI" and "Overview of humanoid robotics landscape"
2. **Given** an instructor is teaching Week 8, **When** they navigate to Module 3 (Isaac), **Then** they find lessons on "NVIDIA Isaac SDK and Isaac Sim"
3. **Given** a student wants to review capstone project requirements, **When** they navigate to Module 4 (VLA), **Then** they find detailed content on the autonomous humanoid final project

---

### User Story 4 - Module Landing Pages (Priority: P2)

Each module needs a clear landing page (intro.md) that provides an overview, learning objectives, and context for the module's content.

**Why this priority**: Landing pages help students understand what they'll learn before diving into detailed lessons, improving comprehension and motivation.

**Independent Test**: Can be tested by navigating to each module's intro page and verifying it contains: module overview, learning objectives, weekly breakdown, and list of lessons.

**Acceptance Scenarios**:

1. **Given** a student clicks on "Module 1: ROS 2", **When** the intro page loads, **Then** they see an overview of ROS 2, learning objectives, and links to all week 3-5 lessons
2. **Given** a student is on Module 3 intro page, **When** they review the content, **Then** they see prerequisites from Modules 1 and 2 clearly stated
3. **Given** an instructor wants to share Module 4 overview, **When** they access the VLA intro page, **Then** they find a comprehensive summary of vision-language-action models and the capstone project

---

### Edge Cases

- What happens when a lesson spans multiple weeks (e.g., extended lab sessions)?
  - Use descriptive names like `week-8-10-isaac-rl-lab.md` to indicate multi-week content
- How does the system handle optional/bonus content not tied to specific weeks?
  - Create an `extras/` or `resources/` subdirectory within each module
- What if hardware requirements or setup guides need their own section?
  - Create a dedicated `setup/` top-level category with `hardware-requirements.md`, `software-setup.md`, etc.
- How are assessment and project files organized?
  - Create `assessments/` subdirectories within each module folder for projects, quizzes, and rubrics

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST organize content into 4 main module directories: `docs/intro/`, `docs/module-1-ros2/`, `docs/module-2-gazebo-unity/`, `docs/module-3-isaac/`, `docs/module-4-vla/`
- **FR-002**: System MUST use consistent file naming: `intro.md` for module overviews, `week-X-lesson-Y.md` for lessons, `week-X-Y-descriptive-name.md` for multi-week content
- **FR-003**: Sidebar MUST reflect the 13-week course structure with clear visual hierarchy (Introduction → Module 1 → Module 2 → Module 3 → Module 4)
- **FR-004**: Each module intro page MUST include: overview, learning objectives, weekly breakdown, prerequisites (if applicable)
- **FR-005**: System MUST support next/previous navigation that follows the logical course sequence
- **FR-006**: File paths MUST use kebab-case (lowercase with hyphens) for consistency and URL-friendliness
- **FR-007**: System MUST maintain the current premium UI/UX enhancements (from spec 002) while adding new content
- **FR-008**: Each lesson file MUST include frontmatter with: title, sidebar_label, sidebar_position, and tags
- **FR-009**: System MUST organize hardware requirements and setup content in a dedicated `setup/` section accessible from the main navigation

### Key Entities

- **Module**: Represents one of the 4 main learning modules (ROS 2, Gazebo/Unity, Isaac, VLA), contains multiple lessons and an intro page
- **Lesson**: Individual content page covering specific topics, typically aligned with 1-2 class sessions
- **Weekly Breakdown**: Grouping of lessons by course weeks (Weeks 1-2, 3-5, 6-7, 8-10, 11-13)
- **Sidebar Category**: Hierarchical navigation structure in sidebars.ts that organizes modules and lessons
- **Assessment**: Projects, labs, and quizzes associated with each module
- **Setup Guide**: Hardware and software configuration documentation

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 4 modules have complete directory structures under `docs/` with consistent naming
- **SC-002**: Sidebar navigation includes all modules with correct hierarchical relationships (can navigate from any page to any other page in ≤3 clicks)
- **SC-003**: Every module has an `intro.md` file with overview, learning objectives, and weekly breakdown
- **SC-004**: File naming follows the convention 100% consistently (intro.md, week-X-lesson-Y.md, week-X-Y-descriptive-name.md)
- **SC-005**: Next/previous navigation correctly sequences through all 13 weeks of content
- **SC-006**: Documentation includes a clear README or guide explaining the structure and naming conventions for future content authors
- **SC-007**: Build process completes without broken links or missing pages
- **SC-008**: Content from Physical-AI-Humanoid-Robotics-Textbook.md is fully mapped to appropriate Docusaurus pages with no orphaned content

## Detailed Content Structure

### Directory Structure

```
docs/
├── intro/
│   ├── intro.md                          # Course overview, learning outcomes
│   ├── week-1-2-physical-ai-foundations.md
│   ├── week-1-2-sensors-overview.md
│   └── hardware-requirements.md          # From textbook content
│
├── module-1-ros2/
│   ├── intro.md                          # Module 1 overview (Weeks 3-5)
│   ├── week-3-lesson-1-ros2-architecture.md
│   ├── week-3-lesson-2-nodes-topics.md
│   ├── week-4-lesson-1-services-actions.md
│   ├── week-4-lesson-2-building-packages.md
│   ├── week-5-lesson-1-launch-files.md
│   ├── week-5-lesson-2-urdf-humanoids.md
│   └── assessments/
│       └── ros2-package-project.md
│
├── module-2-gazebo-unity/
│   ├── intro.md                          # Module 2 overview (Weeks 6-7)
│   ├── week-6-lesson-1-gazebo-setup.md
│   ├── week-6-lesson-2-urdf-sdf.md
│   ├── week-7-lesson-1-physics-simulation.md
│   ├── week-7-lesson-2-unity-robotics.md
│   ├── week-7-lesson-3-sensor-simulation.md
│   └── assessments/
│       └── gazebo-simulation-project.md
│
├── module-3-isaac/
│   ├── intro.md                          # Module 3 overview (Weeks 8-10)
│   ├── week-8-lesson-1-isaac-sdk-intro.md
│   ├── week-8-lesson-2-isaac-sim-setup.md
│   ├── week-9-lesson-1-ai-perception.md
│   ├── week-9-lesson-2-manipulation.md
│   ├── week-10-lesson-1-reinforcement-learning.md
│   ├── week-10-lesson-2-sim-to-real.md
│   └── assessments/
│       └── isaac-perception-pipeline.md
│
├── module-4-vla/
│   ├── intro.md                          # Module 4 overview (Weeks 11-13)
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
├── setup/
│   ├── hardware-requirements.md          # Digital Twin Workstation, Edge Kit
│   ├── software-setup.md                 # Ubuntu, ROS 2, Isaac Sim installation
│   ├── lab-infrastructure.md             # On-premise vs Cloud setup
│   └── student-kit-guide.md              # Economy Jetson Student Kit
│
└── resources/
    ├── glossary.md
    ├── references.md
    └── additional-reading.md
```

### File Naming Conventions

1. **Module Introductions**: Always `intro.md` in each module directory
2. **Weekly Lessons**: `week-X-lesson-Y-descriptive-name.md`
   - X = week number (1-13)
   - Y = lesson number within that week (1, 2, 3...)
   - descriptive-name = kebab-case description of the topic
3. **Multi-week Content**: `week-X-Y-descriptive-name.md` (e.g., `week-8-10-isaac-rl-lab.md`)
4. **Assessments**: Store in `assessments/` subdirectory with descriptive names (e.g., `ros2-package-project.md`)
5. **Setup/Reference**: Descriptive kebab-case names (e.g., `hardware-requirements.md`, `software-setup.md`)

### Sidebar Organization (sidebars.ts)

```typescript
const sidebars: SidebarsConfig = {
  textbookSidebar: [
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

### Content Mapping from Textbook to Docusaurus

#### From "Quarter Overview" Section:
- **Module 1: The Robotic Nervous System (ROS 2)** → `docs/module-1-ros2/`
  - Lessons cover ROS 2 nodes, topics, services, rclpy, URDF
- **Module 2: The Digital Twin (Gazebo & Unity)** → `docs/module-2-gazebo-unity/`
  - Lessons cover physics simulation, Unity integration, sensor simulation
- **Module 3: The AI-Robot Brain (NVIDIA Isaac™)** → `docs/module-3-isaac/`
  - Lessons cover Isaac Sim, Isaac ROS, VSLAM, Nav2, path planning
- **Module 4: Vision-Language-Action (VLA)** → `docs/module-4-vla/`
  - Lessons cover voice commands, LLM planning, capstone project

#### From "Weekly Breakdown" Section:
- **Weeks 1-2** → `docs/intro/week-1-2-physical-ai-foundations.md`, `week-1-2-sensors-overview.md`
- **Weeks 3-5** → `docs/module-1-ros2/week-3-*`, `week-4-*`, `week-5-*`
- **Weeks 6-7** → `docs/module-2-gazebo-unity/week-6-*`, `week-7-*`
- **Weeks 8-10** → `docs/module-3-isaac/week-8-*`, `week-9-*`, `week-10-*`
- **Weeks 11-13** → `docs/module-4-vla/week-11-*`, `week-12-*`, `week-13-*`

#### From "Hardware Requirements" Section:
- **Digital Twin Workstation** → `docs/setup/hardware-requirements.md` (Section 1)
- **Physical AI Edge Kit** → `docs/setup/hardware-requirements.md` (Section 2)
- **Robot Lab Options** → `docs/setup/lab-infrastructure.md`
- **Cloud vs On-Premise** → `docs/setup/lab-infrastructure.md`
- **Economy Jetson Student Kit** → `docs/setup/student-kit-guide.md`

#### From "Assessments" Section:
- **ROS 2 package development project** → `docs/module-1-ros2/assessments/ros2-package-project.md`
- **Gazebo simulation implementation** → `docs/module-2-gazebo-unity/assessments/gazebo-simulation-project.md`
- **Isaac-based perception pipeline** → `docs/module-3-isaac/assessments/isaac-perception-pipeline.md`
- **Capstone: Simulated humanoid robot with conversational AI** → `docs/module-4-vla/assessments/capstone-simulated-humanoid.md`

### Frontmatter Template for Lesson Files

```yaml
---
title: "Week X Lesson Y: Descriptive Title"
sidebar_label: "Lesson Y: Short Title"
sidebar_position: XY
description: "Brief description of what this lesson covers"
tags: [ros2, gazebo, isaac, vla, week-X]
---
```

Example for Week 3, Lesson 1:
```yaml
---
title: "Week 3 Lesson 1: ROS 2 Architecture"
sidebar_label: "Lesson 1: Architecture"
sidebar_position: 31
description: "Understanding ROS 2 core architecture, DDS middleware, and the publish-subscribe pattern"
tags: [ros2, architecture, week-3, fundamentals]
---
```

## Migration Strategy

### Phase 1: Restructure Existing Content
1. Rename current directories to match new structure:
   - `docs/ros2/` → `docs/module-1-ros2/`
   - `docs/gazebo-unity/` → `docs/module-2-gazebo-unity/`
   - `docs/isaac/` → `docs/module-3-isaac/`
   - `docs/vla/` → `docs/module-4-vla/`

2. Rename existing files to follow week-lesson pattern:
   - `architecture.md` → `week-3-lesson-1-ros2-architecture.md`
   - `gazebo-fundamentals.md` → `week-6-lesson-1-gazebo-setup.md`
   - etc.

### Phase 2: Create New Structure Elements
1. Create `docs/intro/` directory with introduction content
2. Create `docs/setup/` directory with hardware/software guides
3. Create `docs/resources/` directory with glossary and references
4. Add `assessments/` subdirectories to each module

### Phase 3: Update Sidebar Configuration
1. Modify `sidebars.ts` to reflect new structure with weekly groupings
2. Add week labels to each lesson category
3. Include Setup & Resources section

### Phase 4: Content Population
1. Extract content from Physical-AI-Humanoid-Robotics-Textbook.md
2. Create lesson files following the mapping outlined above
3. Add frontmatter to all lesson files
4. Create assessment files for each module

### Phase 5: Validation
1. Build site and verify no broken links
2. Test navigation flow through all 13 weeks
3. Verify sidebar hierarchy and labels
4. Check that all textbook content is mapped

## Non-Functional Requirements

- **NFR-001**: Navigation should be intuitive, allowing students to find content within 3 clicks from homepage
- **NFR-002**: File structure should be maintainable by multiple content authors without requiring documentation lookup for every file
- **NFR-003**: Build time should not increase significantly (< 10% increase from current baseline)
- **NFR-004**: URLs should be clean, readable, and SEO-friendly (e.g., `/docs/module-1-ros2/week-3-lesson-1-ros2-architecture`)
- **NFR-005**: Sidebar should clearly indicate current module and week context

## Risks and Mitigation

1. **Risk**: Breaking existing links when renaming directories
   - **Mitigation**: Implement redirects in docusaurus.config.ts, test thoroughly before merge

2. **Risk**: Inconsistent content quality across 50+ lesson files
   - **Mitigation**: Create content templates, establish review process, use linting for frontmatter

3. **Risk**: Confusion about which week maps to which content
   - **Mitigation**: Add week numbers to sidebar labels, include weekly overview in each module intro

4. **Risk**: Over-complicated folder structure slows content creation
   - **Mitigation**: Provide clear documentation, use consistent patterns, create helper scripts if needed

## Open Questions

1. Should we create placeholder files for all 50+ lessons upfront, or create them iteratively as content is written?
2. Do we need a separate "Labs" or "Hands-on" section, or should labs be integrated into lesson files?
3. Should hardware requirements have their own top-level module, or remain in a setup section?
4. Do we need version control for different course variations (e.g., 10-week vs 13-week, with/without hardware)?

## Success Metrics

- **Metric 1**: 100% of textbook content mapped to Docusaurus pages (0 orphaned content)
- **Metric 2**: Navigation depth ≤ 3 clicks to reach any lesson
- **Metric 3**: Build completes without warnings about broken links or missing pages
- **Metric 4**: Sidebar structure approved by at least 2 reviewers (instructors or content authors)
- **Metric 5**: File naming convention followed in 100% of new content files
- **Metric 6**: Each module intro page includes all required sections (overview, objectives, weekly breakdown)
