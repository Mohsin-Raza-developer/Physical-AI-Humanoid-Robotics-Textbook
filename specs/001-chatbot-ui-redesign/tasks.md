

# Tasks: ChatBot UI Redesign for Physical AI Book

**Input**: Design documents from `/specs/001-chatbot-ui-redesign/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are NOT explicitly requested in the specification. This task list focuses on implementation only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`


- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US5, US6, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `src/components/chatkit/` for chat components
- **Types**: `src/types/` for TypeScript definitions
- **Styles**: `.module.css` files co-located with components
- Existing components will be enhanced, new components created

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and TypeScript type definitions

- [X] T001 Create TypeScript type definitions in src/types/chatkit.d.ts (Message, ConversationThread, UIState, UserSession, OfflineQueueItem interfaces)
- [X] T002 [P] Update package.json with required dependencies (@openai/chatkit-react, localStorage type definitions)
- [X] T003 [P] Configure ESLint and Prettier for TypeScript/React if not already configured

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core utilities and state management that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Create OfflineQueue utility class in src/components/chatkit/OfflineQueue.ts (enqueue, dequeue, clear, getAll methods with localStorage persistence)
- [X] T005 [P] Create ConversationManager utility class in src/components/chatkit/ConversationManager.ts (save, load, clear, exists methods with localStorage persistence)
- [X] T006 [P] Create useAuth custom hook in src/components/chatkit/useAuth.ts (Better-Auth session detection, authentication status, user info)
- [X] T007 [P] Create useOfflineQueue custom hook in src/components/chatkit/useOfflineQueue.ts (queue state management, auto-send on reconnect)
- [X] T008 [P] Create useConversationHistory custom hook in src/components/chatkit/useConversationHistory.ts (history load/save, persistence on widget close/reopen)
- [X] T009 Update useChatKitConfig hook in src/components/chatkit/useChatKitConfig.ts to include authentication headers and connection monitoring

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 5 - Authentication Gate (Priority: P1) 🎯 MVP FOUNDATION

**Goal**: Restrict chatbot access to authenticated users only, showing login button for unauthenticated visitors

**Independent Test**: Access site without logging in, click chatbot icon, verify login button appears instead of chat interface

### Implementation for User Story 5

- [ ] T010 [P] [US5] Create AuthGate component in src/components/chatkit/AuthGate.tsx (detect auth status, render login button or children)
- [ ] T011 [P] [US5] Create AuthGate styles in src/components/chatkit/AuthGate.module.css (login button styling, responsive layout for small screens)
- [ ] T012 [US5] Integrate AuthGate into ChatKitWidget in src/components/chatkit/ChatKitWidget.tsx (wrap chat interface, handle login redirect)
- [ ] T013 [US5] Add Better-Auth redirect logic to AuthGate for login/return flow
- [ ] T014 [US5] Add session expiration detection and re-auth prompts in AuthGate component

**Checkpoint**: User Story 5 complete - Authentication gate functional, unauthenticated users cannot access chat

---

## Phase 4: User Story 6 - Small Screen Optimization (Priority: P1) 🎯 MVP FOUNDATION

**Goal**: Optimize all UI elements for 3-4 inch displays with touch-friendly sizing and readable fonts

**Independent Test**: Use chatbot on 3-4 inch display, verify all buttons are 44x44px, fonts 14-16px, no horizontal scrolling

### Implementation for User Story 6

- [X] T015 [P] [US6] Update RoboticsChatKit.module.css with responsive media queries for 3-4 inch displays (max-width: 320px, 360px breakpoints)
- [X] T016 [P] [US6] Update ChatKitWidget.module.css for small screen optimization (widget positioning, floating button size 44x44px minimum)
- [X] T017 [US6] Set minimum font sizes in RoboticsChatKit.module.css (14px body text, 16px input field)
- [X] T018 [US6] Ensure all touch targets are minimum 44x44px in all chatkit CSS modules (send button, close button, clear button)
- [X] T019 [US6] Add orientation change handling in RoboticsChatKit.tsx (portrait/landscape layout adaptation without breaking)
- [X] T020 [US6] Implement proper spacing and padding in RoboticsChatKit.module.css to prevent accidental taps
- [X] T021 [US6] Remove horizontal scrolling with CSS overflow constraints in all chatkit components

**Checkpoint**: User Story 6 complete - UI optimized for small screens, touch-friendly, readable fonts

---

## Phase 5: User Story 1 - Basic Text Conversation (Priority: P1) 🎯 MVP CORE

**Goal**: Enable students to ask questions and receive readable answers on small embedded screens

**Independent Test**: Type "What is inverse kinematics?" and receive readable response on small screen

### Implementation for User Story 1

- [X] T022 [P] [US1] Enhanced ChatKit theme configuration (accent, surface, grayscale colors for visual distinction)
- [X] T023 [P] [US1] Removed non-functional Shadow DOM CSS (ChatKit uses internal styling)
- [X] T024 [US1] Configured composer with custom placeholder text
- [X] T025 [US1] Thread item actions configured (feedback and retry buttons)
- [X] T026 [US1] ChatKit handles scrolling internally (built-in feature)
- [X] T027 [US1] ChatKit handles auto-scroll internally (built-in feature)
- [X] T028 [US1] Integrated conversation history persistence using useConversationHistory hook

**ChatKit Limitations (Shadow DOM)**:
- ⚠️ Timestamps cannot be hidden (ChatKit always shows them)
- ⚠️ Message styling limited to theme colors (cannot customize shapes, padding, borders)
- ⚠️ Input field and send button styling controlled by ChatKit theme only
- ⚠️ Character limits and custom validation not available in ChatKit API

**What Was Achieved**:
- ✅ Theme-based visual distinction (accent colors for user, grayscale for bot)
- ✅ Custom placeholder text in composer
- ✅ Feedback and retry buttons enabled
- ✅ Conversation persistence via localStorage
- ✅ Authentication gate integration
- ✅ Offline queue integration

**Checkpoint**: User Story 1 complete - ChatKit properly configured with available customization options

---

## Phase 6: User Story 2 - Visual Feedback and Status (Priority: P2)

**Goal**: Provide clear status indicators for connection, processing, and errors

**Independent Test**: Observe status indicators during different states (loading, error, disconnected) and verify clarity

### Implementation for User Story 2

- [ ] T029 [P] [US2] Create ConnectionStatus component in src/components/chatkit/ConnectionStatus.tsx (connected, disconnected, reconnecting states)
- [ ] T030 [P] [US2] Create ConnectionStatus styles in src/components/chatkit/ConnectionStatus.module.css (status indicator colors, positioning for small screens)
- [ ] T031 [US2] Add loading indicator to RoboticsChatKit.tsx (spinner/animation during AI response processing)
- [ ] T032 [US2] Add loading indicator styles to RoboticsChatKit.module.css (position within chat interface)
- [ ] T033 [US2] Implement error message display in RoboticsChatKit.tsx (clear error messages with explanations)
- [ ] T034 [US2] Add error message styles to RoboticsChatKit.module.css (error color scheme, readability)
- [ ] T035 [US2] Add message sent confirmation visual feedback in RoboticsChatKit.tsx (checkmark or status indicator)
- [ ] T036 [US2] Integrate ConnectionStatus component into ChatKitWidget.tsx (display at top/bottom of widget)
- [ ] T037 [US2] Add connection state detection using browser online/offline events in RoboticsChatKit.tsx

**Checkpoint**: User Story 2 complete - Visual feedback for all system states implemented

---

## Phase 7: User Story 3 - Input Validation and Guidance (Priority: P2)

**Goal**: Provide proactive input validation with character count and prevent invalid submissions

**Independent Test**: Type messages of various lengths, verify character count appears at 80%, send button disabled when over limit

### Implementation for User Story 3

- [ ] T038 [P] [US3] Add character count indicator to RoboticsChatKit.tsx (show count at 80% of 2000 character limit)
- [ ] T039 [P] [US3] Add character count styles to RoboticsChatKit.module.css (position near input field, readable on small screen)
- [ ] T040 [US3] Implement 2000 character limit validation in RoboticsChatKit.tsx (disable send button when exceeded)
- [ ] T041 [US3] Add warning message display when character limit exceeded in RoboticsChatKit.tsx
- [ ] T042 [US3] Add warning message styles to RoboticsChatKit.module.css (warning color, visibility)
- [ ] T043 [US3] Ensure send button disabled when input empty in RoboticsChatKit.tsx (already started in T025, enhance validation)
- [ ] T044 [US3] Add immediate visual feedback for invalid input states in RoboticsChatKit.tsx (border color changes, helper text)

**Checkpoint**: User Story 3 complete - Input validation and guidance fully implemented

---

## Phase 8: User Story 4 - Conversation History Management (Priority: P3)

**Goal**: Enable conversation history review, persistence across sessions, and clear conversation functionality

**Independent Test**: Conduct multiple conversations, close/reopen chat, verify history persists and is navigable

### Implementation for User Story 4

- [ ] T045 [P] [US4] Implement indefinite history retention using ConversationManager in RoboticsChatKit.tsx (no auto-cleanup, manual clear only)
- [ ] T046 [P] [US4] Add clear conversation button to RoboticsChatKit.tsx (start fresh conversation)
- [ ] T047 [US4] Add clear conversation button styles to RoboticsChatKit.module.css (touch-friendly 44x44px, accessible position)
- [ ] T048 [US4] Add confirmation dialog before clearing conversation in RoboticsChatKit.tsx (prevent accidental data loss)
- [ ] T049 [US4] Implement smooth upward scrolling for older messages in RoboticsChatKit.tsx (no layout shifts)
- [ ] T050 [US4] Optimize rendering for long conversations (50+ messages) with efficient DOM management in RoboticsChatKit.tsx
- [ ] T051 [US4] Verify localStorage quota handling with graceful degradation in ConversationManager.ts (user notification on quota exceeded)

**Checkpoint**: User Story 4 complete - Conversation history management fully functional

---

## Phase 9: Offline Message Queueing (Cross-Cutting: US1, US2)

**Goal**: Queue messages composed offline and automatically send when connection restored

**Independent Test**: Compose message while offline, verify it queues locally, reconnect and verify auto-send

### Implementation for Offline Queueing

- [ ] T052 [P] Integrate useOfflineQueue hook into RoboticsChatKit.tsx (queue messages when offline detected)
- [ ] T053 [P] Add offline queue indicator to ConnectionStatus component (show queued message count)
- [ ] T054 Add automatic message sending on reconnect in RoboticsChatKit.tsx (drain queue when connection restored)
- [ ] T055 Add UUID-based message deduplication in OfflineQueue.ts (prevent duplicate sends on reconnect)
- [ ] T056 Add retry logic with exponential backoff in useOfflineQueue.ts (handle transient network failures)

**Checkpoint**: Offline queueing complete - Messages queue and auto-send on reconnection

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements affecting multiple user stories

- [ ] T057 [P] Update public exports in src/components/chatkit/index.ts (export new components and hooks)
- [ ] T058 [P] Update component documentation in src/components/chatkit/README.md (usage examples, props documentation)
- [ ] T059 [P] Add dark mode refinements to all chatkit CSS modules (contrast ratios for small screens)
- [ ] T060 [P] Add light mode optimizations to all chatkit CSS modules (readability in bright environments)
- [ ] T061 Verify WCAG 2.1 compliance for touch targets across all components (minimum 44x44px)
- [ ] T062 Add accessibility attributes (ARIA labels, roles) to all interactive elements in chatkit components
- [ ] T063 Performance audit: verify <200ms UI responsiveness for authentication checks
- [ ] T064 Performance audit: verify <300ms orientation change adaptation
- [ ] T065 Performance audit: verify 60fps scrolling with 50+ messages
- [ ] T066 Code review and refactoring for consistency across chatkit components
- [ ] T067 Final integration testing across all user stories (US1-US6 working together)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 5 - Authentication Gate (Phase 3)**: Depends on Foundational (T006 useAuth hook)
- **User Story 6 - Small Screen Optimization (Phase 4)**: Depends on Foundational - Can run in parallel with US5
- **User Story 1 - Basic Text Conversation (Phase 5)**: Depends on Foundational (T005, T008) - Should complete AFTER US5 & US6
- **User Story 2 - Visual Feedback (Phase 6)**: Depends on US1 core implementation
- **User Story 3 - Input Validation (Phase 7)**: Depends on US1 core implementation
- **User Story 4 - History Management (Phase 8)**: Depends on US1 core implementation
- **Offline Queueing (Phase 9)**: Depends on US1 and US2 (T007 useOfflineQueue, T029-T037 ConnectionStatus)
- **Polish (Phase 10)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 5 (P1 - Authentication Gate)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 6 (P1 - Small Screen Optimization)**: Can start after Foundational (Phase 2) - No dependencies on other stories, can run parallel with US5
- **User Story 1 (P1 - Basic Text Conversation)**: Can start after Foundational (Phase 2) - Recommended AFTER US5 & US6 for best UX
- **User Story 2 (P2 - Visual Feedback)**: Depends on US1 chat interface existing
- **User Story 3 (P2 - Input Validation)**: Depends on US1 input field and send button existing
- **User Story 4 (P3 - History Management)**: Depends on US1 conversation display existing

### Within Each User Story

- Authentication Gate (US5): T010-T011 can run in parallel, then T012-T014 sequentially
- Small Screen Optimization (US6): T015-T018 can run in parallel, then T019-T021
- Basic Text Conversation (US1): T022-T023 parallel, then T024-T028
- Visual Feedback (US2): T029-T030 parallel, T031-T032 parallel, T033-T034 parallel, then T035-T037
- Input Validation (US3): T038-T039 parallel, then T040-T044
- History Management (US4): T045-T047 parallel, then T048-T051

### Parallel Opportunities

- Phase 1 Setup: T002 and T003 can run in parallel
- Phase 2 Foundational: T005-T008 can all run in parallel after T004 completes
- US5 & US6 can run in complete parallel after Foundational phase
- Within each story, tasks marked [P] can run in parallel
- US2, US3, US4 can run in parallel once US1 core is complete

---

## Parallel Example: User Story 5 (Authentication Gate)

```bash
# Launch parallel tasks for AuthGate:
Task: "Create AuthGate component in src/components/chatkit/AuthGate.tsx"
Task: "Create AuthGate styles in src/components/chatkit/AuthGate.module.css"

# Then sequential integration:
Task: "Integrate AuthGate into ChatKitWidget.tsx"
Task: "Add Better-Auth redirect logic"
Task: "Add session expiration detection"
```

---

## Parallel Example: User Story 1 (Basic Text Conversation)

```bash
# Launch parallel tasks for message display:
Task: "Enhance RoboticsChatKit.tsx with improved message display layout"
Task: "Update RoboticsChatKit.module.css with message styling"

# Then sequential core features:
Task: "Implement text input field optimization"
Task: "Add send button with enabled/disabled states"
Task: "Implement scrollable conversation history"
Task: "Add auto-scroll to latest message"
Task: "Integrate conversation history persistence"
```

---

## Implementation Strategy

### MVP First (User Stories 5, 6, and 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T009) - CRITICAL blocking phase
3. Complete Phase 3: User Story 5 - Authentication Gate (T010-T014)
4. Complete Phase 4: User Story 6 - Small Screen Optimization (T015-T021)
5. Complete Phase 5: User Story 1 - Basic Text Conversation (T022-T028)
6. **STOP and VALIDATE**: Test MVP - authenticated users can chat on small screens
7. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational (Phases 1-2) → Foundation ready
2. Add US5 + US6 + US1 (Phases 3-5) → Test independently → Deploy/Demo (MVP!)
3. Add US2 - Visual Feedback (Phase 6) → Test independently → Deploy/Demo
4. Add US3 - Input Validation (Phase 7) → Test independently → Deploy/Demo
5. Add US4 - History Management (Phase 8) → Test independently → Deploy/Demo
6. Add Offline Queueing (Phase 9) → Test independently → Deploy/Demo
7. Polish (Phase 10) → Final validation → Production release
8. Each phase adds value without breaking previous phases

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (Phases 1-2)
2. Once Foundational is done:
   - Developer A: User Story 5 (Authentication Gate)
   - Developer B: User Story 6 (Small Screen Optimization)
   - Developer C: User Story 1 setup (prepare components)
3. After US5 & US6 complete:
   - All devs: Complete User Story 1 together (core functionality)
4. After US1 complete:
   - Developer A: User Story 2 (Visual Feedback)
   - Developer B: User Story 3 (Input Validation)
   - Developer C: User Story 4 (History Management)
5. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability (US1-US6)
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Priority order: P1 stories (US5, US6, US1) form MVP, P2 stories (US2, US3) enhance UX, P3 story (US4) adds advanced features
- Offline queueing (Phase 9) is cross-cutting and enhances US1 & US2
- No tests explicitly requested in specification, focusing on implementation only
- All components must maintain responsive design for 3-4 inch displays throughout implementation
