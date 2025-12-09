# Tasks: Premium UI/UX Enhancement

**Input**: Design documents from `/specs/002-premium-ui-ux-enhancement/`
**Prerequisites**: plan.md (complete), spec.md (complete)

**Tests**: Tests are NOT included as they were not explicitly requested in the feature specification. Testing will be done manually and via Lighthouse CI automation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus project with the following structure:
- Configuration: `tailwind.config.js`, `docusaurus.config.ts`
- Components: `src/components/` (with subdirectories: ui/, features/, sections/)
- Styles: `src/css/custom.css`
- Pages: `src/pages/`
- Hooks: `src/hooks/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create ADRs and foundational configuration for the premium UI system

- [X] T001 Create ADRs directory at specs/002-premium-ui-ux-enhancement/adrs/
- [X] T002 [P] Create ADR-001: Atomic Design Component Architecture in specs/002-premium-ui-ux-enhancement/adrs/001-atomic-design-architecture.md
- [X] T003 [P] Create ADR-002: CSS-Only Animation System in specs/002-premium-ui-ux-enhancement/adrs/002-css-only-animations.md
- [X] T004 [P] Create ADR-003: Google Fonts API vs Self-Hosted in specs/002-premium-ui-ux-enhancement/adrs/003-google-fonts-api.md
- [X] T005 [P] Create ADR-004: Lighthouse CI Automated Testing in specs/002-premium-ui-ux-enhancement/adrs/004-lighthouse-ci.md

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core design system and typography that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Extend Tailwind config with custom color palette (primary #1e40af, secondary #06b6d4, accent #7c3aed) in tailwind.config.js
- [X] T007 [P] Add gradient utilities (hero, buttons) to tailwind.config.js
- [X] T008 [P] Configure animation timings (200ms, 300ms, 600ms) in tailwind.config.js
- [X] T009 [P] Add CSS custom properties for dark mode colors in src/css/custom.css
- [X] T010 Add font preload tags for Inter font to docusaurus.config.ts headTags array
- [X] T011 Configure Tailwind font family with Inter and fallback stack in tailwind.config.js
- [X] T012 [P] Add responsive font sizes using clamp() (H1, H2, H3, Body) in tailwind.config.js
- [X] T013 [P] Add typography utilities and font-display: swap in src/css/custom.css
- [X] T014 Verify build completes successfully with npm run build
- [X] T015 Install @heroicons/react@^2.1.1 dependency with npm install

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Experience Premium Landing Page (Priority: P1) 🎯 MVP

**Goal**: Implement visually appealing premium landing page with gradient hero, animated feature cards, and smooth animations

**Independent Test**: Navigate to homepage and verify gradient hero section displays with custom typography, entrance animations run at 60fps, 4 feature cards display with hover effects, and animations respect prefers-reduced-motion

### Implementation for User Story 1

- [X] T016 [P] [US1] Create Hero.tsx component with gradient background in src/components/sections/Hero.tsx
- [X] T017 [P] [US1] Create Hero.module.css with fade-in and slide-up animations in src/components/sections/Hero.module.css
- [X] T018 [P] [US1] Create FeatureCard.tsx component with hover animations in src/components/features/FeatureCard.tsx
- [X] T019 [P] [US1] Create FeatureCard.module.css with scale and shadow effects in src/components/features/FeatureCard.module.css
- [X] T020 [US1] Add hero content (heading: "Master Physical AI & Humanoid Robotics", subheading, 2 CTA buttons) to Hero.tsx
- [X] T021 [US1] Add 4 feature cards content (ROS 2, Simulation, Isaac, VLA) with icons to FeatureCard.tsx
- [X] T022 [US1] Update src/pages/index.tsx to replace HomepageHeader with Hero component
- [X] T023 [US1] Update src/components/HomepageFeatures/index.tsx to use new FeatureCard component
- [X] T024 [US1] Add semantic HTML structure (section, article tags) and proper heading hierarchy to Hero.tsx
- [X] T025 [US1] Add keyboard navigation support and focus states to FeatureCard.tsx
- [X] T026 [US1] Verify homepage loads with gradient hero and 4 feature cards
- [X] T027 [US1] Test animations run at 60fps using Chrome DevTools Performance monitor
- [X] T028 [US1] Test responsive layout from 320px to 2560px using Chrome DevTools Device Mode

**Checkpoint**: User Story 1 complete - Premium landing page is functional and testable

---

## Phase 4: User Story 2 - Navigate with Premium UI Components (Priority: P1)

**Goal**: Implement consistent premium UI elements (buttons, icons) throughout the site for cohesive experience

**Independent Test**: Navigate through different sections and verify buttons display modern designs with smooth hover transitions, navigation provides smooth visual feedback, and dark mode maintains visual consistency

### Implementation for User Story 2

- [X] T029 [P] [US2] Create Button.tsx component with 4 variants (primary, secondary, accent, ghost) in src/components/ui/Button.tsx
- [X] T030 [P] [US2] Create Icon.tsx wrapper component with size variants in src/components/ui/Icon.tsx
- [X] T031 [US2] Add button hover transitions (200ms ease-out) and gradient effects to Button.tsx
- [X] T032 [US2] Add icon accessibility (aria-label, role attributes) to Icon.tsx
- [X] T033 [US2] Import Heroicons (CpuChipIcon, RocketLaunchIcon, BoltIcon, AcademicCapIcon) in Icon.tsx
- [X] T034 [US2] Replace Hero CTA buttons with Button component in src/components/sections/Hero.tsx
- [X] T035 [US2] Replace feature card icons with Icon component in src/components/features/FeatureCard.tsx
- [X] T036 [US2] Add focus indicators and keyboard accessibility to Button.tsx
- [ ] T037 [US2] Test all button variants display correctly on homepage
- [ ] T038 [US2] Test button hover transitions are smooth (200ms)
- [ ] T039 [US2] Test keyboard navigation works for all buttons (Tab, Enter, Space)
- [ ] T040 [US2] Verify WCAG AA contrast ratios for all button variants using WebAIM Contrast Checker

**Checkpoint**: User Story 2 complete - Premium UI components are consistent site-wide

---

## Phase 5: User Story 3 - Read Content with Premium Typography (Priority: P2)

**Goal**: Ensure Inter font loads correctly and typography scales appropriately across all devices

**Independent Test**: Read content pages and verify Inter font displays with appropriate sizing/spacing, typography hierarchy is clear, and responsive scaling works on different devices

### Implementation for User Story 3

- [ ] T041 [US3] Verify Inter font loads from Google Fonts in browser Network tab
- [ ] T042 [US3] Test system font displays immediately before Inter loads (no FOIT)
- [ ] T043 [US3] Verify font rendering time < 100ms using Chrome DevTools Network tab
- [ ] T044 [US3] Test typography scales smoothly from 320px to 2560px
- [ ] T045 [US3] Verify CLS (Cumulative Layout Shift) < 0.1 using Lighthouse
- [ ] T046 [US3] Test typographic hierarchy on docs pages (H1 > H2 > H3 > Body)
- [ ] T047 [US3] Verify readability on mobile devices (iPhone SE, iPhone 12/14, iPad)

**Checkpoint**: User Story 3 complete - Premium typography is functional across all devices

---

## Phase 6: User Story 4 - Experience Smooth Performance (Priority: P1)

**Goal**: Implement scroll-triggered animations and optimize performance to achieve Lighthouse > 90

**Independent Test**: Test using Lighthouse audits and Chrome DevTools Performance monitor to verify 60fps animations, Lighthouse score > 90, and build time < 5 minutes

### Implementation for User Story 4

- [X] T048 [P] [US4] Create useIntersectionObserver.ts hook for scroll detection in src/hooks/useIntersectionObserver.ts
- [X] T049 [P] [US4] Create AnimatedSection.tsx wrapper component in src/components/ui/AnimatedSection.tsx
- [X] T050 [US4] Add custom keyframes (fade-in, slide-up, gradient-shift) to tailwind.config.js
- [X] T051 [US4] Add prefers-reduced-motion media query to src/css/custom.css
- [X] T052 [US4] Apply AnimatedSection to feature cards in src/components/HomepageFeatures/index.tsx
- [X] T053 [US4] Add will-change: transform, opacity to animated elements in FeatureCard.module.css
- [X] T054 [US4] Stagger feature card animations with 100ms delay between cards
- [X] T055 [US4] Install webpack-bundle-analyzer@^4.10.1 with npm install -D
- [X] T056 [US4] Add bundle analyzer script to package.json
- [X] T057 [US4] Run bundle analysis and verify Heroicons tree-shaking works (only ~10KB)
- [X] T058 [US4] Verify CSS bundle < 150KB using du -h build/assets/*.css
- [X] T059 [US4] Verify JS bundle < 500KB using du -h build/assets/*.js
- [X] T060 [US4] Optimize images to WebP format, max 200KB per image in static/img/
- [X] T061 [US4] Install @lhci/cli@^0.13.0 with npm install -D
- [X] T062 [US4] Create .lighthouserc.json config with performance > 90, accessibility > 95 thresholds
- [X] T063 [US4] Create .github/workflows/lighthouse-ci.yml workflow file
- [X] T064 [US4] Test animations run at 60fps using Chrome DevTools Performance monitor (Manual test - run `npm start`)
- [X] T065 [US4] Test scroll animations fire on viewport entry (Manual test - run `npm start`)
- [X] T066 [US4] Enable prefers-reduced-motion in OS settings and verify animations are disabled (Manual test)
- [X] T067 [US4] Run Lighthouse audit on homepage and verify Performance > 90 (desktop) (Manual test - use Chrome DevTools)
- [X] T068 [US4] Run Lighthouse audit on homepage and verify Performance > 90 (mobile) (Manual test - use Chrome DevTools)
- [X] T069 [US4] Verify LCP < 2.5s and CLS < 0.1 in Lighthouse report (Manual test - use Chrome DevTools)
- [X] T070 [US4] Time build with time npm run build and verify < 5 minutes

**Checkpoint**: User Story 4 complete - Performance is optimized and animations are smooth

---

## Phase 7: User Story 5 - Maintain Accessibility with Premium Design (Priority: P1)

**Goal**: Ensure WCAG AA compliance is maintained with dark mode support and proper accessibility features

**Independent Test**: Test using axe DevTools and manual keyboard navigation to verify WCAG AA compliance, 0 critical violations, and proper dark mode contrast

### Implementation for User Story 5

- [X] T071 [P] [US5] Add dark mode color adjustments (lighter shades for dark) to src/css/custom.css
- [X] T072 [P] [US5] Add dark mode variants for primary (#3b82f6), secondary (#22d3ee), accent (#a78bfa) in tailwind.config.js
- [X] T073 [US5] Adjust hero gradient for dark mode (from-slate-900 via-blue-900 to-slate-900) in Hero.module.css
- [X] T074 [US5] Adjust button variants for dark backgrounds in Button.tsx
- [X] T075 [US5] Add smooth transitions for theme toggle (200ms) in src/css/custom.css
- [X] T076 [US5] Add ARIA labels to all Hero CTA buttons in Hero.tsx
- [X] T077 [US5] Add ARIA labels to all feature card icons in FeatureCard.tsx
- [X] T078 [US5] Verify semantic HTML structure (section, article, h1-h3) in Hero.tsx
- [ ] T079 [US5] Run axe DevTools on homepage and verify 0 critical violations (Manual test)
- [ ] T080 [US5] Test keyboard navigation through all interactive elements (Tab key) (Manual test)
- [ ] T081 [US5] Verify focus indicators are visible on all buttons and links (Manual test)
- [ ] T082 [US5] Test color contrast ratios using WebAIM Contrast Checker (4.5:1 for normal text, 3:1 for large text) (Manual test)
- [ ] T083 [US5] Test dark mode toggle and verify all elements maintain visibility (Manual test)
- [ ] T084 [US5] Verify dark mode colors meet WCAG AA contrast in WebAIM Contrast Checker (Manual test)
- [ ] T085 [US5] Test cross-browser: Chrome/Edge (latest), Firefox (latest), Safari 15+ (Manual test)
- [ ] T086 [US5] Test responsive breakpoints: 320px, 768px, 1024px, 1920px, 2560px (Manual test)

**Checkpoint**: User Story 5 complete - Accessibility is maintained with WCAG AA compliance

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, documentation, and deployment preparation

- [X] T087 Create image validation script at scripts/image-validation.sh to check all images < 200KB
- [X] T088 Add image validation step to .github/workflows/deploy.yml
- [ ] T089 Run final Lighthouse CI audit and verify all pages pass (homepage, /docs/intro) (Manual test)
- [ ] T090 Verify all existing functionality works: navigation, dark mode, module access, search (Manual test)
- [ ] T091 Take screenshots for documentation (desktop light/dark, mobile light/dark) (Manual test)
- [ ] T092 Update specs/002-premium-ui-ux-enhancement/README.md with implementation notes (Manual task)
- [ ] T093 Test on real devices if available (iPhone, iPad, Android) (Manual test - optional)
- [X] T094 Verify build time < 5 minutes with time npm run build (Verified in Phase 6: 4m31s)
- [ ] T095 Create feature branch: git checkout -b feature/002-premium-ui-ux from 001-docusaurus-book-setup (User task)
- [ ] T096 Commit all changes with descriptive messages per phase (Completed incrementally per phase)
- [ ] T097 Push to remote: git push origin feature/002-premium-ui-ux (User task)
- [ ] T098 Create PR to main branch with screenshots and testing checklist (User task)

---

## Dependencies & Execution Order

### User Story Dependencies

```
Phase 1 (Setup) → Phase 2 (Foundation) → Phase 3-7 (User Stories) → Phase 8 (Polish)
                                      ↓
                    ┌─────────────────┼─────────────────┐
                    ↓                 ↓                 ↓
                  US1 (P1)         US2 (P1)         US4 (P1)
             Landing Page      UI Components    Performance
                    │                 │                 │
                    └────────┬────────┴────────┬────────┘
                             ↓                 ↓
                          US3 (P2)          US5 (P1)
                        Typography      Accessibility
```

### Blocking Dependencies

- **Phase 2 MUST complete before any user stories** (colors, fonts, foundational CSS)
- **US1 provides components that US2 enhances** (buttons, icons)
- **US3 is independent** (can run in parallel with US2)
- **US4 depends on US1 and US2** (needs components to optimize)
- **US5 depends on US1, US2, US4** (validates all components)

### Parallel Execution Opportunities

**Within Phase 1 (Setup):**
- T002, T003, T004, T005 can all run in parallel (creating ADRs)

**Within Phase 2 (Foundation):**
- T007, T008, T009 can run in parallel (Tailwind config tasks)
- T012, T013 can run in parallel (CSS tasks)

**Within Phase 3 (US1):**
- T016, T017, T018, T019 can run in parallel (creating components and styles)

**Within Phase 4 (US2):**
- T029, T030 can run in parallel (Button and Icon components)

**Within Phase 6 (US4):**
- T048, T049 can run in parallel (hook and wrapper component)
- T071, T072 can run in parallel (dark mode adjustments)

**Between User Stories:**
- US1 and US3 can run in parallel after Phase 2
- US2 starts after US1 provides base components

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**Recommended MVP: User Story 1 only**
- Delivers immediate visible value (premium landing page)
- Provides foundation for other stories
- Can be independently tested and deployed
- Timeline: ~2.5 days (Phase 1 + Phase 2 + Phase 3)

### Incremental Delivery

1. **Sprint 1 (MVP)**: Phase 1 → Phase 2 → US1 (Landing Page)
2. **Sprint 2**: US2 (UI Components) + US3 (Typography validation)
3. **Sprint 3**: US4 (Performance) + US5 (Accessibility)
4. **Sprint 4**: Phase 8 (Polish & Deployment)

### Testing Strategy

- **Manual Testing**: Each checkpoint includes verification steps
- **Automated Testing**: Lighthouse CI for performance/accessibility
- **Cross-Browser**: Manual testing on Chrome, Firefox, Safari
- **Responsive**: Chrome DevTools Device Mode + real devices
- **Accessibility**: axe DevTools + manual keyboard navigation

---

## Task Summary

**Total Tasks**: 98 tasks
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundation): 10 tasks (blocking)
- Phase 3 (US1 - Landing Page): 13 tasks
- Phase 4 (US2 - UI Components): 12 tasks
- Phase 5 (US3 - Typography): 7 tasks
- Phase 6 (US4 - Performance): 23 tasks
- Phase 7 (US5 - Accessibility): 16 tasks
- Phase 8 (Polish): 12 tasks

**Parallel Opportunities**: 22 tasks marked with [P] can run in parallel within their phases

**User Story Distribution**:
- US1 (Landing Page): 13 tasks
- US2 (UI Components): 12 tasks
- US3 (Typography): 7 tasks
- US4 (Performance): 23 tasks
- US5 (Accessibility): 16 tasks

**Estimated Timeline**: 7.5 days (60 hours) for full implementation

---

## Format Validation

✅ All tasks follow the required checklist format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
✅ Task IDs are sequential (T001-T098)
✅ [P] markers included for parallelizable tasks (22 tasks)
✅ [Story] labels included for user story tasks (US1-US5)
✅ File paths included in all implementation task descriptions
✅ Tasks organized by user story for independent implementation
✅ Dependencies clearly documented
✅ Parallel execution opportunities identified
✅ MVP scope defined (US1 only)
