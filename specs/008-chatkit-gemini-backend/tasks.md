# Tasks: ChatKit Backend with Google Gemini Integration

**Input**: Design documents from `/specs/008-chatkit-gemini-backend/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api-summary.md

**Tests**: Not explicitly requested in spec - focusing on implementation tasks only

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Project uses: `chatbot-backend/` at repository root with `app/` subdirectory structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create `chatbot-backend/` directory structure per plan.md section "Source Code (to be created)"
- [x] T002 Initialize Python 3.12+ project with `requirements.txt` from quickstart.md
- [x] T003 [P] Create `.env.example` with all required environment variables (DATABASE_URL, GEMINI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, JWT_SECRET, FRONTEND_URL)
- [x] T004 [P] Create `railway.toml` for Railway deployment configuration
- [x] T005 [P] Create `pytest.ini` for test configuration
- [x] T006 [P] Create `alembic.ini` for database migrations
- [x] T007 [P] Create `README.md` with setup instructions based on quickstart.md

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T008 Setup SQLAlchemy async engine in `chatbot-backend/app/database.py` with connection pooling (pool_size=20, max_overflow=10)
- [x] T009 [P] Create Pydantic settings configuration in `chatbot-backend/app/config.py` using pydantic-settings
- [x] T010 [P] Initialize FastAPI app in `chatbot-backend/app/main.py` with CORS middleware configuration
- [x] T011 [P] Implement JWT authentication dependency in `chatbot-backend/app/middleware/auth.py` (get_current_user function)
- [x] T012 [P] Setup structured logging with Structlog in `chatbot-backend/app/utils/logger.py`
- [x] T013 [P] Create custom exception classes in `chatbot-backend/app/utils/errors.py` (AuthenticationError, NotFoundError, ValidationError)
- [x] T014 Initialize Alembic for migrations in `chatbot-backend/alembic/` directory
- [x] T015 Create Thread SQLAlchemy model in `chatbot-backend/app/models/thread.py` per data-model.md schema
- [x] T016 Create Message SQLAlchemy model in `chatbot-backend/app/models/message.py` per data-model.md schema
- [x] T017 Generate Alembic migration `001_create_chatbot_tables` in `chatbot-backend/alembic/versions/` for threads and messages tables with indexes
- [x] T018 [P] Create Thread Pydantic schemas in `chatbot-backend/app/schemas/thread.py` (ThreadCreate, ThreadResponse, ThreadListResponse)
- [x] T019 [P] Create Message Pydantic schemas in `chatbot-backend/app/schemas/message.py` (MessageCreate, MessageResponse, MessageListResponse)
- [x] T020 Add health check endpoint GET `/health` in `chatbot-backend/app/main.py` returning {"status": "healthy"}

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Send Message and Receive Streaming Response (Priority: P1) 🎯 MVP

**Goal**: Enable users to send messages in threads and receive real-time streaming AI responses with knowledge base search

**Independent Test**: Create a thread, send "Explain inverse kinematics", verify streamed response begins within 2s and completes within 30s with knowledge base tool invocation visible

### Implementation for User Story 1

- [x] T021 [P] [US1] Configure Gemini AsyncOpenAI client in `chatbot-backend/app/services/agent_service.py` with base_url and API key
- [x] T022 [P] [US1] Initialize Cohere client and Qdrant client in `chatbot-backend/app/services/search_tool.py`
- [x] T023 [US1] Implement `search_knowledge_base` function tool in `chatbot-backend/app/services/search_tool.py` using @function_tool decorator with Cohere embedding and Qdrant search (collection: robotics_textbook_v1, limit=5, threshold=0.4)
- [x] T024 [US1] Create chat agent with Gemini model and search tool in `chatbot-backend/app/services/agent_service.py` (Agent name: "RoboticsAssistant", instructions per plan.md)
- [x] T025 [US1] Implement `generate_response` async function in `chatbot-backend/app/services/agent_service.py` using Runner.run with streaming support
- [x] T026 [US1] Create ThreadService class in `chatbot-backend/app/services/thread_service.py` with create_thread, get_threads, get_thread, delete_thread methods
- [x] T027 [US1] Create MessageService class in `chatbot-backend/app/services/message_service.py` with create_message, get_messages, get_next_sequence_number methods
- [x] T028 [US1] Implement POST `/api/threads` endpoint in `chatbot-backend/app/routers/threads.py` with JWT auth dependency
- [x] T029 [P] [US1] Implement GET `/api/threads` endpoint with pagination (limit, offset) in `chatbot-backend/app/routers/threads.py`
- [x] T030 [P] [US1] Implement GET `/api/threads/{thread_id}` endpoint with message history in `chatbot-backend/app/routers/threads.py`
- [x] T031 [P] [US1] Implement DELETE `/api/threads/{thread_id}` endpoint in `chatbot-backend/app/routers/threads.py`
- [x] T032 [US1] Implement POST `/api/threads/{thread_id}/messages` streaming endpoint in `chatbot-backend/app/routers/messages.py` using StreamingResponse with SSE format
- [x] T033 [US1] Add SSE event formatters in `chatbot-backend/app/routers/messages.py` for message_start, content_delta, message_end events
- [x] T034 [US1] Implement message persistence (save user message and assistant response) in POST messages endpoint
- [x] T035 [P] [US1] Implement GET `/api/threads/{thread_id}/messages` endpoint with pagination in `chatbot-backend/app/routers/messages.py`
- [x] T036 [US1] Add routers to FastAPI app in `chatbot-backend/app/main.py` (threads, messages)
- [x] T037 [US1] Add rate limiting middleware using SlowAPI in `chatbot-backend/app/middleware/rate_limit.py` (10 messages/min, 5 threads/min per user)
- [x] T038 [US1] Add error handling for Gemini API failures with retry logic in `chatbot-backend/app/services/agent_service.py`
- [x] T039 [US1] Add input validation (max content length 10,000 chars, no empty messages) in MessageCreate schema

**Checkpoint**: At this point, User Story 1 should be fully functional - users can create threads, send messages, receive streaming AI responses with knowledge base search

---

## Phase 4: User Story 2 - View Agentic Actions and Chain-of-Thought (Priority: P2)

**Goal**: Expose agent tool invocations and reasoning steps as structured events during response generation for transparency

**Independent Test**: Send message "Search the knowledge base for ROS 2 tutorials", verify action_event SSE events showing "Analyzing query", "Searching knowledge base", "Synthesizing results" with metadata (action_type, tool_name, status, timestamps)

### Implementation for User Story 2

- [ ] T040 [P] [US2] Create AgentAction Pydantic schema in `chatbot-backend/app/schemas/action.py` with fields: action_id, action_type, tool_name, input_params, output_result, status, created_at, duration_ms
- [ ] T041 [US2] Implement action event emitter in `chatbot-backend/app/services/agent_service.py` to capture tool call start/completion/failure
- [ ] T042 [US2] Add `action_event` SSE event formatter in `chatbot-backend/app/routers/messages.py` following contracts/api-summary.md schema
- [ ] T043 [US2] Modify streaming endpoint to emit action_event for each tool invocation (search_knowledge_base calls)
- [ ] T044 [US2] Add action metadata sanitization in `chatbot-backend/app/services/agent_service.py` (remove sensitive params before emitting)
- [ ] T045 [US2] Add error action events for failed tool calls with user-friendly error messages

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - basic chat with transparent agent actions

---

## Phase 5: User Story 3 - Interact with Widgets in Conversation (Priority: P2)

**Goal**: Enable embedding interactive widgets (buttons, forms) in AI responses with server-handled and client-only actions

**Independent Test**: Trigger response with button_group widget, verify widget specification in SSE stream, send widget action to backend, verify new response returned

### Implementation for User Story 3

- [ ] T046 [P] [US3] Create Widget Pydantic schemas in `chatbot-backend/app/schemas/widget.py` (ButtonWidget, ButtonGroupWidget, FormWidget, WidgetAction)
- [ ] T047 [P] [US3] Create ClientEffect Pydantic schema in `chatbot-backend/app/schemas/effect.py` with fields: effect_id, effect_type, target, parameters, execution_order
- [ ] T048 [US3] Add `widget` SSE event formatter in `chatbot-backend/app/routers/messages.py` following ChatKit widget schema
- [ ] T049 [US3] Add `client_effect` SSE event formatter in `chatbot-backend/app/routers/messages.py`
- [ ] T050 [US3] Implement widget specification builder in `chatbot-backend/app/services/agent_service.py` (helper functions to create widget specs from agent responses)
- [ ] T051 [US3] Modify streaming endpoint to detect and emit widget events when agent includes widgets in responses
- [ ] T052 [US3] Add POST `/api/widgets/{widget_id}/action` endpoint in `chatbot-backend/app/routers/widgets.py` for server-handled widget actions
- [ ] T053 [US3] Implement widget action handler in `chatbot-backend/app/services/widget_service.py` to process actions and return new agent responses
- [ ] T054 [US3] Add widget router to FastAPI app in `chatbot-backend/app/main.py`

**Checkpoint**: User Stories 1, 2, AND 3 should all work independently - chat with actions and interactive widgets

---

## Phase 6: User Story 4 - Receive Progress Updates During Long Operations (Priority: P2)

**Goal**: Emit real-time progress events during knowledge base searches and other long-running operations to prevent user frustration

**Independent Test**: Send message requiring knowledge base search, verify progress_update SSE event with message "Searching knowledge base..." emitted before results, verify completion event when done

### Implementation for User Story 4

- [ ] T055 [P] [US4] Create ProgressUpdate Pydantic schema in `chatbot-backend/app/schemas/progress.py` with fields: update_id, operation_id, progress_message, progress_percentage, timestamp
- [ ] T056 [US4] Add `progress_update` SSE event formatter in `chatbot-backend/app/routers/messages.py`
- [ ] T057 [US4] Implement progress emitter context manager in `chatbot-backend/app/services/agent_service.py` for tracking long operations
- [ ] T058 [US4] Modify `search_knowledge_base` tool in `chatbot-backend/app/services/search_tool.py` to emit progress events: "Searching knowledge base..." (start), "Filtering results..." (processing), "Complete" (end)
- [ ] T059 [US4] Add progress event emission to streaming endpoint for tool call lifecycle (started → in progress → completed/failed)
- [ ] T060 [US4] Add unique operation_id generation for each tool invocation to link progress updates

**Checkpoint**: All P2 user stories complete - full-featured chat with actions, widgets, and progress feedback

---

## Phase 7: User Story 5 - Automatic Thread Title Generation (Priority: P3)

**Goal**: Auto-generate concise, meaningful thread titles from first message to improve conversation navigation

**Independent Test**: Create thread, send first message "What is inverse kinematics?", verify title "Inverse Kinematics Explanation" (or similar) auto-generated and saved to thread without blocking main response

### Implementation for User Story 5

- [ ] T061 [P] [US5] Create title generation agent in `chatbot-backend/app/services/title_agent.py` with Gemini model and instructions to generate titles under 50 characters
- [ ] T062 [US5] Implement `generate_title` async function in `chatbot-backend/app/services/title_agent.py` with 5-second timeout and fallback to "New Conversation"
- [ ] T063 [US5] Add background task trigger in POST messages endpoint to call title generation after first message (sequence_number == 1)
- [ ] T064 [US5] Implement thread title update logic in `chatbot-backend/app/services/thread_service.py` (update_title method)
- [ ] T065 [US5] Add check to prevent overwriting manually set titles or existing auto-generated titles
- [ ] T066 [US5] Add error handling and logging for title generation failures (should not block main response)

**Checkpoint**: Thread title generation working - improves UX without impacting core chat functionality

---

## Phase 8: User Story 6 - Upload and Download Attachments (Priority: P3) ⚠️ DEFERRED

**Goal**: Enable file uploads/downloads for richer conversations

**Status**: ⚠️ **DEFERRED TO FUTURE PHASE** - This requires Cloudflare R2 integration which is not part of MVP (Phase 1-2). See plan.md Phase 3 for details.

**Future Tasks** (when R2 is implemented):
- Attachment model (already defined in data-model.md but not migrated)
- R2 client setup and signed URL generation
- Upload/download endpoints
- File validation and size limits

**Current Implementation**: Skip all attachment-related tasks for MVP

---

## Phase 9: User Story 7 - Tag Entities with @-Mentions (Priority: P3)

**Goal**: Enable explicit entity references in messages using @-mention syntax for power-user control

**Independent Test**: Send message "@KnowledgeBase what is SLAM?", verify system parses mention, identifies entity (search tool), triggers direct tool invocation

### Implementation for User Story 7

- [ ] T067 [P] [US7] Create EntityMention Pydantic schema in `chatbot-backend/app/schemas/entity.py` with fields: mention_id, entity_type, entity_id, entity_name, position_in_message
- [ ] T068 [P] [US7] Create entity registry in `chatbot-backend/app/services/entity_service.py` with available entities (tools: KnowledgeBase, agents, knowledge sources)
- [ ] T069 [US7] Implement @-mention parser in `chatbot-backend/app/services/entity_service.py` using regex to extract @EntityName patterns
- [ ] T070 [US7] Add GET `/api/entities/autocomplete` endpoint in `chatbot-backend/app/routers/entities.py` returning entity suggestions for "@" trigger
- [ ] T071 [US7] Modify message processing in `chatbot-backend/app/services/agent_service.py` to parse mentions and pass as structured context to agent
- [ ] T072 [US7] Add graceful handling for invalid @-mentions (treat as regular text, no request failure)
- [ ] T073 [US7] Add entity router to FastAPI app in `chatbot-backend/app/main.py`
- [ ] T074 [US7] Implement email address detection to exclude from @-mention parsing (prevent false positives)

**Checkpoint**: @-mention functionality complete - power users can explicitly control tool invocation

---

## Phase 10: User Story 8 - Push Client Effects from Server (Priority: P3)

**Goal**: Enable server-driven UI orchestration by sending client effect instructions (scroll, highlight, modal, navigate)

**Independent Test**: Send message triggering client effect "scroll_to_message", verify client_effect SSE event with effect_type, target message_id, and parameters received by client

### Implementation for User Story 8

- [ ] T075 [P] [US8] Expand ClientEffect schema in `chatbot-backend/app/schemas/effect.py` with all effect types (scroll_to_message, highlight_text, open_modal, navigate)
- [ ] T076 [US8] Implement client effect builder in `chatbot-backend/app/services/agent_service.py` to create effect specifications from agent decisions
- [ ] T077 [US8] Add client effect emission logic to streaming endpoint (emit multiple effects in sequence order)
- [ ] T078 [US8] Document supported client effects in `chatbot-backend/README.md` with parameters and expected client behavior
- [ ] T079 [US8] Add effect validation (ensure required fields present before emitting)
- [ ] T080 [US8] Implement graceful degradation (effects are optional, core message content always delivered even if effects fail)

**Checkpoint**: All user stories (US1-US5, US7-US8) complete - full-featured chatbot with all advanced features except attachments

---

## Phase 11: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and production readiness

- [ ] T081 [P] Add Sentry SDK integration in `chatbot-backend/app/main.py` for error tracking with DSN from environment
- [ ] T082 [P] Add comprehensive error handling for all endpoints with user-friendly error messages (use custom exception classes from T013)
- [ ] T083 [P] Add request/response logging for all API calls with sanitized content (PII removal)
- [ ] T084 [P] Create pytest fixtures in `chatbot-backend/tests/conftest.py` for async database, test client, mock Gemini responses
- [ ] T085 [P] Add API documentation improvements in FastAPI (title, description, version, contact, license)
- [ ] T086 Validate all environment variables on startup in `chatbot-backend/app/config.py` with clear error messages for missing vars
- [ ] T087 Add database connection retry logic with exponential backoff in `chatbot-backend/app/database.py`
- [ ] T088 Implement graceful shutdown handler in `chatbot-backend/app/main.py` to close database connections and background tasks
- [ ] T089 [P] Add performance monitoring with metrics emission (request latency, streaming duration, error rates) in middleware
- [ ] T090 [P] Create deployment documentation in `chatbot-backend/docs/DEPLOYMENT.md` with Railway setup steps
- [ ] T091 Run quickstart.md validation (setup, run server, hit health check, send test message)
- [ ] T092 Add CORS configuration validation and security headers (HSTS, X-Content-Type-Options) in `chatbot-backend/app/main.py`
- [ ] T093 Implement API rate limit headers (X-RateLimit-Limit, X-RateLimit-Remaining, X-RateLimit-Reset) in rate limit middleware
- [ ] T094 Add pagination validation (max limits) for thread and message list endpoints
- [ ] T095 Implement Gemini API usage tracking and logging for cost monitoring
- [ ] T096 Add thread and message count validation against usage limits
- [ ] T097 Final security audit: validate JWT signature algorithm, check for SQL injection vulnerabilities, validate input sanitization
- [ ] T098 Create production .env template with Railway environment variable mapping
- [ ] T099 Add automated health check tests for Railway deployment (ping /health endpoint)
- [ ] T100 Final integration test: create thread → send message → verify streaming → verify knowledge search → verify title generation → delete thread

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-10)**: All depend on Foundational phase completion
  - US1 (Phase 3): Can start after Foundational - No dependencies on other stories
  - US2 (Phase 4): Can start after Foundational - Depends on US1 streaming infrastructure
  - US3 (Phase 5): Can start after Foundational - Independent of US1/US2 but benefits from streaming
  - US4 (Phase 6): Can start after Foundational - Depends on US1/US2 for tool infrastructure
  - US5 (Phase 7): Can start after Foundational - Depends on US1 for thread/message creation
  - US6 (Phase 8): **DEFERRED** - Not implemented in MVP
  - US7 (Phase 9): Can start after Foundational - Independent but integrates with US1
  - US8 (Phase 10): Can start after Foundational - Independent but integrates with streaming
- **Polish (Phase 11)**: Depends on all desired user stories being complete

### User Story Priority Order

Suggested implementation sequence (incremental delivery):

1. **Phase 1-2**: Setup + Foundational (everyone works together)
2. **Phase 3 (US1)**: MVP - basic chat with streaming → **STOP, TEST, DEMO**
3. **Phase 4 (US2)**: Add agent actions → Test independently → Deploy
4. **Phase 5 (US3)**: Add widgets → Test independently → Deploy
5. **Phase 6 (US4)**: Add progress updates → Test independently → Deploy
6. **Phase 7 (US5)**: Add title generation → Test independently → Deploy
7. **Phase 9 (US7)**: Add @-mentions → Test independently → Deploy
8. **Phase 10 (US8)**: Add client effects → Test independently → Deploy
9. **Phase 11**: Polish and production hardening

### Within Each User Story

- Services/tools before endpoint implementation
- Core functionality before integrations
- Error handling after happy path works
- Validation after core logic complete

### Parallel Opportunities

**Phase 1 (Setup)**: All tasks T001-T007 can run in parallel (different files)

**Phase 2 (Foundational)**: Parallel groups:
- Group A: T009, T010, T012, T013 (independent configs/utils)
- Group B: T018, T019 (schemas - after models exist)
- Wait for T008 (database) and T014-T016 (models/migrations) to complete sequentially

**Phase 3 (US1)**: Parallel groups:
- Group A: T021, T022 (AI client setups)
- Group B: T028, T029, T030, T031, T035 (endpoint implementations after services ready)

**Phase 4-10**: Within each user story, tasks marked [P] can run in parallel

**Phase 11 (Polish)**: All tasks marked [P] can run in parallel (T081-T085, T089-T090, T094-T096)

**Multi-Developer Parallel Strategy**:
After Foundational phase complete, different developers can work on:
- Developer A: US1 (MVP critical path)
- Developer B: US2 (agent actions)
- Developer C: US3 (widgets)
Then integrate and proceed to US4-5, US7-8, Polish

---

## Parallel Example: User Story 1 (MVP)

```bash
# After Foundational phase complete, launch US1 parallel tasks:

# Parallel Group 1: AI client setup (different files, no dependencies)
Task T021: "Configure Gemini AsyncOpenAI client in agent_service.py"
Task T022: "Initialize Cohere/Qdrant clients in search_tool.py"

# Sequential: Build on Group 1
Task T023: "Implement search_knowledge_base tool" (needs T022)
Task T024: "Create chat agent with Gemini" (needs T021, T023)
Task T025: "Implement generate_response function" (needs T024)

# Parallel Group 2: Service layer (different files, both need models from Phase 2)
Task T026: "Create ThreadService class in thread_service.py"
Task T027: "Create MessageService class in message_service.py"

# Parallel Group 3: Endpoints (different endpoints, need services)
Task T028: "POST /api/threads endpoint"
Task T029: "GET /api/threads endpoint"
Task T030: "GET /api/threads/{id} endpoint"
Task T031: "DELETE /api/threads/{id} endpoint"

# Sequential: Streaming endpoint (needs T025 + T027)
Task T032: "POST /api/threads/{id}/messages streaming endpoint"
Task T033: "Add SSE event formatters"

# Parallel Group 4: Final touches (different aspects)
Task T037: "Add rate limiting middleware"
Task T038: "Add error handling for Gemini"
Task T039: "Add input validation"
```

---

## Implementation Strategy

### MVP First (US1 Only) - Recommended for Solo Developer

1. **Week 1**: Complete Phase 1 (Setup) + Phase 2 (Foundational)
2. **Week 2**: Complete Phase 3 (US1) - basic chat with streaming
3. **STOP and VALIDATE**:
   - Test US1 independently
   - Create thread, send message, verify streaming response
   - Verify knowledge base search works
   - Deploy to Railway, test with real frontend
4. **Decision Point**:
   - If US1 works perfectly → Proceed to US2-US5
   - If issues found → Fix before adding features

### Incremental Delivery - Recommended for Team

1. **Foundation Sprint** (Week 1): Phase 1-2 → All developers together
2. **MVP Sprint** (Week 2): Phase 3 (US1) → Test → Deploy → **DEMO TO STAKEHOLDERS**
3. **Enhancement Sprint 1** (Week 3): Phase 4-6 (US2-US4) → Test → Deploy
4. **Enhancement Sprint 2** (Week 4): Phase 7, 9-10 (US5, US7-US8) → Test → Deploy
5. **Polish Sprint** (Week 5): Phase 11 → Production hardening → Final deploy

Each sprint delivers working, testable increments without breaking previous functionality.

### Parallel Team Strategy (3+ Developers)

**Week 1**: All developers complete Phase 1-2 together (pair programming on complex parts)

**Week 2**: Once Foundational phase done:
- **Developer A**: Phase 3 (US1) - Critical path, full focus
- **Developer B**: Phase 4 (US2) - Agent actions in parallel
- **Developer C**: Phase 5 (US3) - Widgets in parallel
- End of week: Integrate all three, test together

**Week 3-4**: Rotate developers through US4-US5, US7-US8

**Week 5**: All developers on Phase 11 (Polish) together

---

## MVP Scope Recommendation

**Minimum Viable Product** = Phase 1 + Phase 2 + Phase 3 (US1 only)

This delivers:
- ✅ User can create conversation threads
- ✅ User can send messages and receive streaming AI responses
- ✅ AI agent automatically searches knowledge base when needed
- ✅ Real-time SSE streaming with proper event format
- ✅ JWT authentication working
- ✅ Database persistence (threads, messages)
- ✅ Deployed to Railway
- ✅ Error handling and rate limiting
- ✅ Health check endpoint

**Total MVP Tasks**: T001-T039 (39 tasks)
**Estimated Time**: 2-3 weeks for solo developer, 1-2 weeks for team

After MVP validation, add US2-US5, US7-US8 incrementally based on user feedback.

---

## Notes

- [P] tasks = different files, no shared dependencies, can run in parallel
- [Story] label (US1, US2, etc.) maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group of tasks
- Stop at any checkpoint to validate story independently
- **US6 (Attachments) is DEFERRED** - requires Cloudflare R2 setup (Phase 3, Week 4-5 per plan.md)
- Focus on MVP first (US1), validate thoroughly before adding advanced features
- All file paths are exact - no ambiguity about where to create code
- SSE event formats follow contracts/api-summary.md specifications
- Database schema follows data-model.md exactly (threads, messages only for MVP)

---

**Total Task Count**: 100 tasks (excluding deferred US6)
**MVP Task Count**: 39 tasks (Phase 1-3)
**Tasks per Story**:
- Setup: 7 tasks
- Foundational: 13 tasks
- US1: 19 tasks
- US2: 6 tasks
- US3: 9 tasks
- US4: 6 tasks
- US5: 6 tasks
- US6: DEFERRED
- US7: 8 tasks
- US8: 6 tasks
- Polish: 20 tasks

**Parallel Opportunities**: 40+ tasks marked [P] can run in parallel within their phases

**Independent Test Criteria**: Each user story (US1-US5, US7-US8) has clear validation steps defined in "Independent Test" sections
