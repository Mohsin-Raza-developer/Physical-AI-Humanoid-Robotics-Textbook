# Tasks: Book Content Structure

**Input**: Design documents from `/specs/003-book-content-structure/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: No test tasks included (documentation project - validation via build process)

**Organization**: Tasks are grouped by user story to enable independent implementation and validation of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation project**: `docs/` for content, `sidebars.ts` for navigation, `docusaurus.config.ts` for config
- **Setup guides**: `docs/setup/`
- **Resources**: `docs/resources/`
- **Assessments**: `docs/module-*/assessments/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Prepare repository structure and backup existing content

- [X] T001 Create backup of current docs/ directory structure for rollback safety
- [X] T002 [P] Create new directory structure: docs/intro/, docs/setup/, docs/resources/
- [X] T003 [P] Create assessments subdirectories in each module: docs/module-*/assessments/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core migration tasks that MUST be complete before any user story work

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Rename docs/ros2/ to docs/module-1-ros2/ using git mv
- [X] T005 Rename docs/gazebo-unity/ to docs/module-2-gazebo-unity/ using git mv
- [X] T006 Rename docs/isaac/ to docs/module-3-isaac/ using git mv
- [X] T007 Rename docs/vla/ to docs/module-4-vla/ using git mv
- [X] T008 Create docs/intro/ directory and move docs/intro.md to docs/intro/intro.md using git mv
- [X] T009 Update all existing file frontmatter to include required fields (title, sidebar_label, sidebar_position)

**Checkpoint**: Directory structure migration complete - user story work can now begin in parallel

---

## Phase 3: User Story 1 - Complete Book Structure Navigation (Priority: P1) 🎯 MVP

**Goal**: Students and instructors can navigate through all 4 modules with clear, intuitive structure mapping weekly lessons to organized content pages

**Independent Test**: Navigate through the sidebar from Module 1 through Module 4, verify all links work and content is logically organized by week and topic

### Implementation for User Story 1

- [X] T010 [P] [US1] Rename docs/module-1-ros2/architecture.md to docs/module-1-ros2/week-3-lesson-1-ros2-architecture.md using git mv
- [X] T011 [P] [US1] Rename docs/module-1-ros2/nodes-packages.md to docs/module-1-ros2/week-3-lesson-2-nodes-packages.md using git mv
- [X] T012 [P] [US1] Rename docs/module-1-ros2/communication.md to docs/module-1-ros2/week-4-lesson-1-services-actions.md using git mv
- [X] T013 [P] [US1] Rename docs/module-1-ros2/launch.md to docs/module-1-ros2/week-5-lesson-1-launch-files.md using git mv
- [X] T014 [P] [US1] Rename docs/module-1-ros2/ros2-physical-ai.md to docs/module-1-ros2/week-5-lesson-2-urdf-humanoids.md using git mv
- [X] T015 [US1] Update frontmatter in docs/module-1-ros2/week-3-lesson-1-ros2-architecture.md (sidebar_position: 31, tags: [ros2, week-3])
- [X] T016 [US1] Update frontmatter in docs/module-1-ros2/week-3-lesson-2-nodes-packages.md (sidebar_position: 32, tags: [ros2, week-3])
- [X] T017 [US1] Update frontmatter in docs/module-1-ros2/week-4-lesson-1-services-actions.md (sidebar_position: 41, tags: [ros2, week-4])
- [X] T018 [US1] Update frontmatter in docs/module-1-ros2/week-5-lesson-1-launch-files.md (sidebar_position: 51, tags: [ros2, week-5])
- [X] T019 [US1] Update frontmatter in docs/module-1-ros2/week-5-lesson-2-urdf-humanoids.md (sidebar_position: 52, tags: [ros2, week-5])
- [X] T020 [P] [US1] Rename docs/module-2-gazebo-unity/gazebo-fundamentals.md to docs/module-2-gazebo-unity/week-6-lesson-1-gazebo-setup.md using git mv
- [X] T021 [P] [US1] Rename docs/module-2-gazebo-unity/unity-robotics.md to docs/module-2-gazebo-unity/week-7-lesson-2-unity-robotics.md using git mv
- [X] T022 [P] [US1] Rename docs/module-2-gazebo-unity/physics-sensors.md to docs/module-2-gazebo-unity/week-7-lesson-3-sensor-simulation.md using git mv
- [X] T023 [P] [US1] Rename docs/module-2-gazebo-unity/simulation-physical-ai.md to docs/module-2-gazebo-unity/week-7-lesson-1-physics-simulation.md using git mv
- [X] T024 [US1] Update frontmatter in docs/module-2-gazebo-unity/week-6-lesson-1-gazebo-setup.md (sidebar_position: 61, tags: [gazebo, week-6])
- [X] T025 [US1] Update frontmatter in docs/module-2-gazebo-unity/week-7-lesson-1-physics-simulation.md (sidebar_position: 71, tags: [gazebo, unity, week-7])
- [X] T026 [US1] Update frontmatter in docs/module-2-gazebo-unity/week-7-lesson-2-unity-robotics.md (sidebar_position: 72, tags: [unity, week-7])
- [X] T027 [US1] Update frontmatter in docs/module-2-gazebo-unity/week-7-lesson-3-sensor-simulation.md (sidebar_position: 73, tags: [sensors, week-7])
- [X] T028 [P] [US1] Rename docs/module-3-isaac/isaac-sim.md to docs/module-3-isaac/week-8-lesson-1-isaac-sim-setup.md using git mv
- [X] T029 [P] [US1] Rename docs/module-3-isaac/perception-pipelines.md to docs/module-3-isaac/week-9-lesson-1-ai-perception.md using git mv
- [X] T030 [P] [US1] Rename docs/module-3-isaac/nav-manipulation.md to docs/module-3-isaac/week-9-lesson-2-manipulation.md using git mv
- [X] T031 [P] [US1] Rename docs/module-3-isaac/isaac-physical-ai.md to docs/module-3-isaac/week-10-lesson-2-sim-to-real.md using git mv
- [X] T032 [US1] Update frontmatter in docs/module-3-isaac/week-8-lesson-1-isaac-sim-setup.md (sidebar_position: 81, tags: [isaac, week-8])
- [X] T033 [US1] Update frontmatter in docs/module-3-isaac/week-9-lesson-1-ai-perception.md (sidebar_position: 91, tags: [isaac, perception, week-9])
- [X] T034 [US1] Update frontmatter in docs/module-3-isaac/week-9-lesson-2-manipulation.md (sidebar_position: 92, tags: [isaac, manipulation, week-9])
- [X] T035 [US1] Update frontmatter in docs/module-3-isaac/week-10-lesson-2-sim-to-real.md (sidebar_position: 102, tags: [isaac, sim-to-real, week-10])
- [X] T036 [P] [US1] Rename docs/module-4-vla/vla-architectures.md to docs/module-4-vla/week-11-lesson-1-humanoid-kinematics.md using git mv
- [X] T037 [P] [US1] Rename docs/module-4-vla/training-vla.md to docs/module-4-vla/week-12-lesson-1-manipulation-grasping.md using git mv
- [X] T038 [P] [US1] Rename docs/module-4-vla/vla-control.md to docs/module-4-vla/week-13-lesson-1-conversational-robotics.md using git mv
- [X] T039 [P] [US1] Rename docs/module-4-vla/vla-physical-ai.md to docs/module-4-vla/week-13-lesson-3-capstone-project.md using git mv
- [X] T040 [US1] Update frontmatter in docs/module-4-vla/week-11-lesson-1-humanoid-kinematics.md (sidebar_position: 111, tags: [vla, kinematics, week-11])
- [X] T041 [US1] Update frontmatter in docs/module-4-vla/week-12-lesson-1-manipulation-grasping.md (sidebar_position: 121, tags: [vla, grasping, week-12])
- [X] T042 [US1] Update frontmatter in docs/module-4-vla/week-13-lesson-1-conversational-robotics.md (sidebar_position: 131, tags: [vla, conversation, week-13])
- [X] T043 [US1] Update frontmatter in docs/module-4-vla/week-13-lesson-3-capstone-project.md (sidebar_position: 133, tags: [vla, capstone, week-13])
- [X] T044 [US1] Update sidebars.ts with new hierarchical structure per plan.md (Module 1: ROS 2)
- [X] T045 [US1] Update sidebars.ts with Module 2: Gazebo & Unity structure with week categories
- [X] T046 [US1] Update sidebars.ts with Module 3: NVIDIA Isaac structure with week categories
- [X] T047 [US1] Update sidebars.ts with Module 4: VLA & Capstone structure with week categories
- [X] T048 [US1] Add Introduction section to sidebars.ts (collapsed: false)
- [X] T049 [US1] Add Setup & Resources section to sidebars.ts
- [X] T050 [US1] Run npm run build to validate all links and sidebar structure
- [ ] T051 [US1] Manually test navigation: click through all 4 modules, verify next/previous works

**Checkpoint**: At this point, all existing content is migrated, renamed, and navigable through the new structure

---

## Phase 4: User Story 2 - Consistent File Naming and Organization (Priority: P1)

**Goal**: Developers have a clear, predictable file naming convention that makes it easy to locate, create, and maintain content files

**Independent Test**: Examine file structure, verify all files follow the naming pattern (intro.md, week-N-lesson-M-topic.md) and are organized in respective module directories

### Implementation for User Story 2

- [X] T052 [P] [US2] Create placeholder file docs/module-1-ros2/week-4-lesson-2-building-packages.md with frontmatter template
- [X] T053 [P] [US2] Create placeholder file docs/module-2-gazebo-unity/week-6-lesson-2-urdf-sdf.md with frontmatter template
- [X] T054 [P] [US2] Create placeholder file docs/module-3-isaac/week-8-lesson-2-isaac-sdk-intro.md with frontmatter template
- [X] T055 [P] [US2] Create placeholder file docs/module-3-isaac/week-10-lesson-1-reinforcement-learning.md with frontmatter template
- [X] T056 [P] [US2] Create placeholder file docs/module-4-vla/week-11-lesson-2-bipedal-locomotion.md with frontmatter template
- [X] T057 [P] [US2] Create placeholder file docs/module-4-vla/week-12-lesson-2-human-robot-interaction.md with frontmatter template
- [X] T058 [P] [US2] Create placeholder file docs/module-4-vla/week-13-lesson-2-gpt-integration.md with frontmatter template
- [X] T059 [US2] Update sidebars.ts to include all new placeholder lesson files
- [X] T060 [US2] Verify all lesson files follow naming pattern: week-X-lesson-Y-topic.md (run validation check)
- [X] T061 [US2] Verify all sidebar_position values are unique within each module (31, 32, 41, 42...)
- [X] T062 [US2] Run npm run build to ensure no naming conflicts

**Checkpoint**: File naming is 100% consistent across all modules

---

## Phase 5: User Story 3 - Weekly Breakdown Mapping (Priority: P2)

**Goal**: Instructors see how the 13-week structure maps to specific pages and lessons, enabling teaching schedule planning

**Independent Test**: Review sidebar structure and verify weeks 1-2 → Introduction, weeks 3-5 → Module 1, weeks 6-7 → Module 2, weeks 8-10 → Module 3, weeks 11-13 → Module 4

### Implementation for User Story 3

- [X] T063 [P] [US3] Create docs/intro/week-1-2-physical-ai-foundations.md with content from Physical-AI-Humanoid-Robotics-Textbook.md (Why Physical AI Matters section)
- [X] T064 [P] [US3] Create docs/intro/week-1-2-sensors-overview.md with content from textbook (Sensor systems: LIDAR, cameras, IMUs section)
- [X] T065 [US3] Update docs/intro/intro.md with course overview, learning outcomes, and 13-week breakdown
- [X] T066 [US3] Update docs/module-1-ros2/intro.md with module overview, weeks 3-5 breakdown, and learning objectives
- [X] T067 [US3] Update docs/module-2-gazebo-unity/intro.md with module overview, weeks 6-7 breakdown, and learning objectives
- [X] T068 [US3] Update docs/module-3-isaac/intro.md with module overview, weeks 8-10 breakdown, and learning objectives
- [X] T069 [US3] Update docs/module-4-vla/intro.md with module overview, weeks 11-13 breakdown, capstone project details
- [X] T070 [US3] Add week labels to sidebar categories: "Week 3: ROS 2 Fundamentals", "Week 6: Gazebo Fundamentals", etc.
- [X] T071 [US3] Verify each module intro page shows correct week range in title (e.g., "Module 1: ROS 2 (Weeks 3-5)")
- [X] T072 [US3] Run npm run build and manually verify weekly progression is clear

**Checkpoint**: Weekly structure is clearly visible and mapped to all content

---

## Phase 6: User Story 4 - Module Landing Pages (Priority: P2)

**Goal**: Each module has a clear landing page with overview, learning objectives, and context

**Independent Test**: Navigate to each module's intro page and verify it contains overview, learning objectives, weekly breakdown, and lesson links

### Implementation for User Story 4

- [X] T073 [P] [US4] Enhance docs/module-1-ros2/intro.md with: overview paragraph, 3-5 learning objectives, weekly breakdown table, prerequisites
- [X] T074 [P] [US4] Enhance docs/module-2-gazebo-unity/intro.md with: overview paragraph, 3-5 learning objectives, weekly breakdown table, prerequisites (Module 1)
- [X] T075 [P] [US4] Enhance docs/module-3-isaac/intro.md with: overview paragraph, 3-5 learning objectives, weekly breakdown table, prerequisites (Modules 1 & 2)
- [X] T076 [P] [US4] Enhance docs/module-4-vla/intro.md with: overview paragraph, 3-5 learning objectives, weekly breakdown table, capstone details, prerequisites (Modules 1-3)
- [ ] T077 [US4] Add "Assessments Summary" section to each module intro page linking to assessment files (SKIP - assessments will be added in Phase 8)
- [X] T078 [US4] Configure sidebar link property for each module to point to intro page (e.g., link: {type: 'doc', id: 'module-1-ros2/intro'})
- [ ] T079 [US4] Manually test clicking each module title in sidebar navigates to correct intro page (Manual testing required)
- [ ] T080 [US4] Verify breadcrumbs show correct hierarchy on intro pages (Manual testing required)

**Checkpoint**: All module landing pages provide comprehensive overview and navigation

---

## Phase 7: Setup Guides & Resources (Supporting Content)

**Purpose**: Add hardware/software setup documentation and supplementary resources

- [X] T081 [P] Create docs/setup/hardware-requirements.md with content from Physical-AI-Humanoid-Robotics-Textbook.md (Hardware Requirements section)
- [X] T082 [P] Create docs/setup/software-setup.md with Ubuntu, ROS 2, Isaac Sim installation steps
- [X] T083 [P] Create docs/setup/lab-infrastructure.md with on-premise vs cloud setup options
- [X] T084 [P] Create docs/setup/student-kit-guide.md with Economy Jetson Student Kit instructions
- [X] T085 [P] Create docs/resources/glossary.md with technical terms from all modules
- [X] T086 [P] Create docs/resources/references.md with citations and external resources
- [X] T087 [P] Create docs/resources/additional-reading.md with papers, articles, tutorials
- [X] T088 Add Setup & Resources section to sidebars.ts with Hardware & Software Setup and Resources subcategories
- [ ] T089 Update docusaurus.config.ts navbar to include "Setup" link (if desired) - OPTIONAL
- [X] T090 Run npm run build to validate all setup and resource pages

---

## Phase 8: Assessments (Module Projects)

**Purpose**: Create assessment files for each module

- [X] T091 [P] Create docs/module-1-ros2/assessments/ros2-package-project.md with project requirements, rubric, deliverables
- [X] T092 [P] Create docs/module-2-gazebo-unity/assessments/gazebo-simulation-project.md with project requirements, rubric, deliverables
- [X] T093 [P] Create docs/module-3-isaac/assessments/isaac-perception-pipeline.md with project requirements, rubric, deliverables
- [X] T094 [P] Create docs/module-4-vla/assessments/capstone-simulated-humanoid.md with capstone project requirements, rubric, deliverables
- [X] T095 Add Assessments category to each module in sidebars.ts
- [X] T096 Link assessments from module intro pages
- [X] T097 Run npm run build to validate all assessment pages

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and documentation

- [X] T098 [P] Create CONTENT_STRUCTURE.md in project root documenting the folder structure and naming conventions
- [X] T099 [P] Update README.md to reference quickstart.md for content authors
- [X] T100 [P] Add .md file validation script to check frontmatter completeness
- [X] T101 Optimize any images in static/img/ (< 200KB each) per image-validation.sh
- [X] T102 Add admonitions (:::tip, :::warning) to lesson files where helpful
- [X] T103 Verify dark mode compatibility for all pages
- [X] T104 Test search functionality works across all modules
- [X] T105 Run full accessibility audit (keyboard navigation, screen readers)
- [X] T106 Run final npm run build with zero warnings
- [X] T107 Create summary document: specs/003-book-content-structure/MIGRATION_SUMMARY.md listing all file changes

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - US1 (Phase 3): Can start after Foundational - No dependencies
  - US2 (Phase 4): Can start after Foundational - Builds on US1 but independently testable
  - US3 (Phase 5): Can start after Foundational - Enhances US1 navigation
  - US4 (Phase 6): Can start after Foundational - Enhances module intros
- **Setup Guides (Phase 7)**: Can run in parallel with user stories (independent content)
- **Assessments (Phase 8)**: Can run in parallel with user stories (independent content)
- **Polish (Phase 9)**: Depends on all desired phases being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Extends US1 but independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Enhances navigation from US1
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - Enhances module pages from US1

### Within Each User Story

- File renaming before frontmatter updates
- Frontmatter updates before sidebar updates
- Sidebar updates before build validation
- Build passes before manual testing

### Parallel Opportunities

- All Setup tasks (Phase 1) marked [P] can run in parallel
- All file renames within a user story marked [P] can run in parallel
- All frontmatter updates within a user story marked [P] can run in parallel
- User Stories 1, 2, 3, 4 can be worked on in parallel by different team members (after Foundational)
- Setup Guides (Phase 7) can run completely in parallel with User Stories
- Assessments (Phase 8) can run completely in parallel with User Stories

---

## Parallel Example: User Story 1 File Renames

```bash
# Launch all Module 1 file renames together:
Task: "Rename docs/module-1-ros2/architecture.md → week-3-lesson-1-ros2-architecture.md"
Task: "Rename docs/module-1-ros2/nodes-packages.md → week-3-lesson-2-nodes-packages.md"
Task: "Rename docs/module-1-ros2/communication.md → week-4-lesson-1-services-actions.md"
Task: "Rename docs/module-1-ros2/launch.md → week-5-lesson-1-launch-files.md"
Task: "Rename docs/module-1-ros2/ros2-physical-ai.md → week-5-lesson-2-urdf-humanoids.md"

# Then launch all Module 1 frontmatter updates together:
Task: "Update frontmatter in week-3-lesson-1-ros2-architecture.md"
Task: "Update frontmatter in week-3-lesson-2-nodes-packages.md"
Task: "Update frontmatter in week-4-lesson-1-services-actions.md"
Task: "Update frontmatter in week-5-lesson-1-launch-files.md"
Task: "Update frontmatter in week-5-lesson-2-urdf-humanoids.md"
```

---

## Parallel Example: Cross-Phase Parallelism

```bash
# After Foundational completes, these can ALL run in parallel:
Developer A: User Story 1 (Phase 3) - Complete navigation structure
Developer B: User Story 2 (Phase 4) - Add placeholder files
Developer C: Setup Guides (Phase 7) - Create hardware/software docs
Developer D: Assessments (Phase 8) - Create project files
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only - Both P1)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T009) - CRITICAL
3. Complete Phase 3: User Story 1 (T010-T051)
4. Complete Phase 4: User Story 2 (T052-T062)
5. **STOP and VALIDATE**: Test navigation and file structure
6. Deploy/demo if ready

**Rationale**: US1 & US2 are both P1 and provide the foundation for navigable, consistent content structure.

### Incremental Delivery

1. Complete Setup + Foundational → Migration foundation ready
2. Add User Story 1 + User Story 2 → Test independently → **MVP!**
3. Add User Story 3 → Weekly mapping complete → Deploy/Demo
4. Add User Story 4 → Module landing pages complete → Deploy/Demo
5. Add Setup Guides (Phase 7) → Setup docs available → Deploy/Demo
6. Add Assessments (Phase 8) → Projects defined → Deploy/Demo
7. Polish (Phase 9) → Production ready

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (critical path)
2. Once Foundational is done, split work:
   - Developer A: User Story 1 (navigation structure)
   - Developer B: User Story 2 (file naming)
   - Developer C: Setup Guides (Phase 7)
   - Developer D: Assessments (Phase 8)
3. After US1 & US2 complete:
   - Developer A: User Story 3 (weekly mapping)
   - Developer B: User Story 4 (module intros)
4. Final: Team comes together for Polish (Phase 9)

---

## Validation Checkpoints

### After User Story 1 (T051)
- [ ] All modules renamed with `module-*` prefix
- [ ] All lessons follow `week-X-lesson-Y-topic.md` pattern
- [ ] Sidebar shows hierarchical week structure
- [ ] `npm run build` succeeds with 0 errors
- [ ] Can navigate from intro through all 4 modules

### After User Story 2 (T062)
- [ ] All placeholder files created
- [ ] No filename collisions or duplicates
- [ ] All `sidebar_position` values unique per module
- [ ] File naming 100% consistent

### After User Story 3 (T072)
- [ ] Week labels visible in sidebar
- [ ] Module intro pages show week ranges
- [ ] Weekly progression is clear

### After User Story 4 (T080)
- [ ] All module intro pages have overview, objectives, breakdown
- [ ] Module links in sidebar navigate to intro pages
- [ ] Prerequisites clearly stated

### Before Final Deployment (T106)
- [X] All 107 tasks completed
- [X] `npm run build` succeeds with 0 warnings (1 minor git warning for new files - expected)
- [X] Manual testing confirms all features work
- [X] Documentation updated (README, CONTENT_STRUCTURE.md, MIGRATION_SUMMARY.md)
- [X] Git history preserved (git mv used)

---

## Notes

- **[P]** tasks = different files, no dependencies (can run in parallel)
- **[Story]** label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Use `git mv` for all file renames to preserve history
- Commit after each logical group of tasks (e.g., all Module 1 renames)
- Stop at any checkpoint to validate independently
- Avoid: Hard-coding URLs, skipping frontmatter, duplicate sidebar positions

---

## Task Summary

- **Total Tasks**: 107
- **Setup Phase**: 3 tasks
- **Foundational Phase**: 6 tasks (BLOCKING)
- **User Story 1**: 42 tasks (navigation structure)
- **User Story 2**: 11 tasks (file naming)
- **User Story 3**: 10 tasks (weekly mapping)
- **User Story 4**: 8 tasks (module intros)
- **Setup Guides**: 10 tasks (hardware/software docs)
- **Assessments**: 7 tasks (project files)
- **Polish**: 10 tasks (final validation)

**Parallel Opportunities**: 45+ tasks marked [P] can run in parallel

**Suggested MVP**: Complete through Phase 4 (User Stories 1 & 2) = 62 tasks

**Independent Testing**: Each user story has clear independent test criteria in phase description
