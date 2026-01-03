# Tasks: ChatKit-Integrated Robotics Chatbot Backend

**Feature ID**: 008-chatkit-gemini-backend
**Input**: Design documents from `/specs/008-chatkit-gemini-backend/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not requested in specification - tasks focus on implementation only

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

## Path Conventions

Project structure (from plan.md):
```
app/
├── main.py                  # FastAPI application entry
├── config.py                # Environment configuration
├── models/                  # SQLAlchemy models
├── services/                # Business logic
├── middleware/              # Authentication, CORS, rate limiting
├── agents/                  # AI agent with tools
└── chatkit_server/          # ChatKit server implementation
```

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project directory structure (app/, tests/, .env.example)
- [X] T002 Initialize Python project with requirements.txt dependencies (FastAPI 0.115+, openai-chatkit 1.0+, openai-agents 0.2+, SQLAlchemy 2.0+, asyncpg, qdrant-client, cohere)
- [X] T003 [P] Create .env.example file with all required environment variables (DATABASE_URL, GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY, BETTER_AUTH_SECRET)
- [X] T004 [P] Create .gitignore file (exclude .env, venv/, __pycache__/, *.pyc)
- [X] T005 [P] Create README.md with project overview and setup instructions

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Create database schema using Neon MCP: threads table (thread_id, user_id, title, metadata, created_at, updated_at)
- [X] T007 Create database schema using Neon MCP: messages table (message_id, thread_id, role, content, sequence_number, created_at)
- [X] T008 Create database indexes using Neon MCP (idx_threads_user_id, idx_threads_updated_at, idx_messages_thread_id, uq_messages_thread_sequence)
- [X] T009 [P] Create config.py with Settings class using Pydantic (load DATABASE_URL, GEMINI_API_KEY, QDRANT_URL, etc. from environment)
- [X] T010 [P] Create app/models/__init__.py
- [X] T011 [P] Create Thread model in app/models/thread.py (SQLAlchemy model mapping to threads table)
- [X] T012 [P] Create Message model in app/models/message.py (SQLAlchemy model mapping to messages table)
- [X] T013 Create database.py with async engine setup (create_async_engine with asyncpg, connection pooling config)
- [X] T014 [P] Create app/middleware/__init__.py
- [X] T015 [P] Create Better Auth session validation middleware in app/middleware/auth.py (validate_session function, query Better Auth session table, extract user_id)
- [X] T016 [P] Create CORS middleware configuration in app/middleware/cors.py
- [X] T017 [P] Create rate limiting middleware in app/middleware/rate_limit.py (60 requests/minute per user)
- [X] T018 Create FastAPI application in app/main.py (initialize app, register middleware, configure CORS)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Student Asks Question with Citations 🎯 MVP

**Goal**: Student can ask a question and receive a streaming AI response with inline Markdown citations from the robotics textbook

**User Story** (from spec.md):
> Student types question: "Explain inverse kinematics in robotics"
> Chatbot searches knowledge base (Qdrant vector store) for relevant textbook sections
> Chatbot generates response with citations to specific chapters/sections
> Response streams to student in real-time
> Conversation is saved and persists across sessions

**Independent Test**:
1. Start server
2. POST to /chatkit with thread.create
3. POST to /chatkit with message.create containing "What is inverse kinematics?"
4. Verify SSE stream contains:
   - message_start event
   - action event (search_textbook started/completed)
   - content_delta events with text and citations [^1]
   - message_end event with full response
5. Verify response includes inline Markdown footnotes with chapter/section references
6. Query database to confirm thread and messages are persisted

### Implementation for User Story 1

- [X] T019 [P] [US1] Create app/services/__init__.py
- [X] T020 [P] [US1] Create QdrantService in app/services/qdrant_service.py (initialize Qdrant client, search method with Cohere embeddings)
- [X] T021 [P] [US1] Create app/agents/__init__.py
- [X] T022 [US1] Create search_textbook tool in app/agents/tools.py (@function_tool decorator, call QdrantService.search, format results with citation metadata)
- [X] T023 [US1] Create RoboticsAgent in app/agents/robotics_agent.py (Agent with instructions, search_textbook tool, Gemini model config)
- [X] T024 [P] [US1] Create app/chatkit_server/__init__.py
- [X] T025 [US1] Create PostgresStore in app/chatkit_server/postgres_store.py (implement ChatKit Store interface: generate_thread_id, load_thread, save_thread, load_threads, add_thread_item, load_thread_items with SQLAlchemy async queries)
- [X] T026 [US1] Create RoboticsChatbotServer in app/chatkit_server/chatkit_server.py (extend ChatKitServer, implement respond method with stream_agent_response, pass user_id in context)
- [X] T027 [US1] Create /chatkit endpoint in app/main.py (POST route, apply auth middleware, instantiate RoboticsChatbotServer, handle thread.create/message.create/threads.list operations)
- [X] T028 [US1] Add SSE streaming support in app/main.py (configure StreamingResponse for message.create, emit ChatKit events: message_start, content_delta, action, message_end)
- [X] T029 [US1] Add citation formatting logic in app/agents/robotics_agent.py (agent instructions to use [^1] syntax, format reference list at bottom)
- [X] T030 [US1] Add error handling in app/chatkit_server/chatkit_server.py (catch exceptions during streaming, emit error events, return 500 with error details)
- [X] T031 [US1] Add logging for ChatKit operations in app/chatkit_server/chatkit_server.py (log thread creation, message creation, search tool calls, streaming events)

**Checkpoint**: At this point, User Story 1 should be fully functional - student can ask question, get streaming response with citations, conversation persists

---

## Phase 4: User Story 2 - Multiple Conversation Threads

**Goal**: Student can create multiple conversation threads, switch between them, and each thread maintains independent context

**User Story** (from spec.md):
> Student starts new conversation on different topic
> Student can switch between conversation threads
> Each thread maintains independent context
> Students can manage at least 20 concurrent conversation threads
> Thread switching completes instantly (< 200ms)

**Independent Test**:
1. Create 3 separate threads via POST /chatkit (thread.create)
2. Send messages to each thread with different questions
3. Verify each thread maintains independent conversation history
4. List threads via POST /chatkit (threads.list)
5. Verify pagination works (limit=20)
6. Measure thread retrieval time (should be < 200ms)

### Implementation for User Story 2

- [X] T032 [P] [US2] Implement thread.get operation in app/chatkit_server/chatkit_server.py (handle thread.get request type, call PostgresStore.load_thread)
- [X] T033 [P] [US2] Implement threads.list operation in app/chatkit_server/chatkit_server.py (handle threads.list request type, call PostgresStore.load_threads with pagination)
- [X] T034 [P] [US2] Implement thread.delete operation in app/chatkit_server/chatkit_server.py (handle thread.delete request type, verify ownership via user_id, cascade delete messages)
- [X] T035 [US2] Add thread title auto-generation in app/chatkit_server/postgres_store.py (extract first user message, truncate to 50 chars, update thread.title if NULL)
- [X] T036 [US2] Add pagination support in app/chatkit_server/postgres_store.py (cursor-based pagination for load_threads, return Page[ThreadMetadata] with next_cursor)
- [X] T037 [US2] Add updated_at trigger in database using Neon MCP (PostgreSQL trigger to auto-update threads.updated_at when messages are added)
- [X] T038 [US2] Add authorization checks in app/chatkit_server/postgres_store.py (verify thread.user_id matches session user_id for all operations)
- [X] T039 [US2] Optimize thread listing query in app/chatkit_server/postgres_store.py (add index usage, ORDER BY updated_at DESC, limit 20)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - student can manage multiple threads, switch contexts, all conversations persist

---

## Phase 5: User Story 3 - Knowledge-Grounded Responses (Edge Cases)

**Goal**: Chatbot handles edge cases when knowledge base doesn't have relevant information, suggests related topics

**User Story** (from spec.md - Edge Cases):
> Chatbot clearly states when textbook doesn't cover topic
> Suggests related topics that are covered

**Independent Test**:
1. Send question outside textbook scope: "Explain quantum computing"
2. Verify response explicitly states "The robotics textbook does not cover this topic"
3. Verify response suggests related topics from textbook (e.g., "You might be interested in: Control Systems, Sensor Fusion")
4. Send question with no results from Qdrant search
5. Verify agent handles empty search results gracefully

### Implementation for User Story 3

- [X] T040 [P] [US3] Add no-results handling in app/agents/tools.py (check if Qdrant search returns empty results, return formatted message)
- [X] T041 [US3] Update agent instructions in app/agents/robotics_agent.py (add instructions to explicitly state when textbook doesn't cover topic, suggest related topics)
- [X] T042 [US3] Add related topics suggestion logic in app/agents/tools.py (if search results < threshold, run secondary broader search, format as suggestions)
- [X] T043 [US3] Add confidence scoring in app/services/qdrant_service.py (return relevance scores from Qdrant, filter results below confidence threshold)
- [X] T044 [US3] Add fallback behavior in app/agents/robotics_agent.py (when no citations available, provide general robotics context without claiming textbook coverage)

**Checkpoint**: All core user stories should now be independently functional - chatbot handles questions with citations, multiple threads, and edge cases gracefully

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories, production readiness

- [X] T045 [P] Add health check endpoint in app/main.py (GET /health, check database connection, Qdrant connection, return service status)
- [X] T046 [P] Add comprehensive error handling in app/main.py (global exception handler, return ChatKit-compatible error format)
- [X] T047 [P] Add request/response logging in app/middleware/logging.py (log all /chatkit requests, response times, user_id, thread_id)
- [X] T048 [P] Add input validation in app/chatkit_server/chatkit_server.py (validate message content length < 100,000 chars, validate thread IDs are UUIDs)
- [X] T049 [P] Add security headers in app/middleware/security.py (Content-Security-Policy, X-Frame-Options, Strict-Transport-Security)
- [X] T050 [P] Optimize database queries in app/chatkit_server/postgres_store.py (use connection pooling, add query timeouts, optimize message retrieval with LIMIT)
- [X] T051 [P] Add graceful shutdown handler in app/main.py (cleanup database connections, close Qdrant client on SIGTERM)
- [X] T052 Add deployment configuration (create Procfile or start.sh for production server, uvicorn with workers)
- [X] T053 [P] Update README.md with API documentation (link to contracts/chatkit-api.json, add usage examples)
- [X] T054 Run quickstart.md validation (follow quickstart steps, verify server starts, test all endpoints, document any issues) - Created VALIDATION-GUIDE.md, preflight-check.py, and FRONTEND-INTEGRATION.md

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (US1 → US2 → US3)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (Primary Flow)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (Multiple Threads)**: Can start after Foundational (Phase 2) - Builds on US1 but independently testable
- **User Story 3 (Edge Cases)**: Can start after Foundational (Phase 2) - Builds on US1 but independently testable

### Within Each User Story

- Models before services
- Services before agents/tools
- Agents before ChatKit server
- ChatKit server before FastAPI routes
- Core implementation before error handling/logging
- Story complete before moving to next priority

### Parallel Opportunities

**Phase 1 (Setup)**:
- T003 (.env.example), T004 (.gitignore), T005 (README.md) can run in parallel

**Phase 2 (Foundational)**:
- T009 (config.py), T010-T012 (models), T014-T017 (middleware) can run in parallel after database schema creation (T006-T008)

**Phase 3 (User Story 1)**:
- T019 (services/__init__), T020 (QdrantService), T021 (agents/__init__), T024 (chatkit_server/__init__) can run in parallel
- T022 (search_textbook tool) and T023 (RoboticsAgent) can start after T020 (QdrantService) completes

**Phase 4 (User Story 2)**:
- T032 (thread.get), T033 (threads.list), T034 (thread.delete) can run in parallel

**Phase 5 (User Story 3)**:
- T040 (no-results handling) and T041 (agent instructions update) can run in parallel

**Phase 6 (Polish)**:
- T045 (health check), T046 (error handling), T047 (logging), T048 (validation), T049 (security headers), T050 (query optimization), T053 (README update) can all run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch parallel setup tasks:
Task T019: "Create app/services/__init__.py"
Task T020: "Create QdrantService in app/services/qdrant_service.py"
Task T021: "Create app/agents/__init__.py"
Task T024: "Create app/chatkit_server/__init__.py"

# After QdrantService ready, launch agent tasks in parallel:
Task T022: "Create search_textbook tool in app/agents/tools.py"
Task T023: "Create RoboticsAgent in app/agents/robotics_agent.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

**Goal**: Get student asking questions with citations working end-to-end

1. **Complete Phase 1**: Setup (T001-T005) → Project initialized
2. **Complete Phase 2**: Foundational (T006-T018) → Database, models, auth ready (CRITICAL - blocks all stories)
3. **Complete Phase 3**: User Story 1 (T019-T031) → Streaming chatbot with citations working
4. **STOP and VALIDATE**:
   - Test with quickstart.md steps
   - Verify student can ask question and get cited response
   - Verify conversation persists in database
   - Verify streaming works in real-time
5. **Deploy/demo if ready** → MVP complete! ✅

**Estimated MVP**: 31 tasks (T001-T031)

### Incremental Delivery

1. **Complete Setup + Foundational** (T001-T018) → Foundation ready
2. **Add User Story 1** (T019-T031) → Test independently → Deploy/Demo (**MVP!** 🎯)
3. **Add User Story 2** (T032-T039) → Test independently → Deploy/Demo (multi-thread support added)
4. **Add User Story 3** (T040-T044) → Test independently → Deploy/Demo (edge cases handled)
5. **Add Polish** (T045-T054) → Production-ready → Final deploy
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. **Team completes Setup + Foundational together** (T001-T018)
2. **Once Foundational is done**:
   - Developer A: User Story 1 (T019-T031) - Primary flow
   - Developer B: User Story 2 (T032-T039) - Thread management (waits for US1 completion)
   - Developer C: User Story 3 (T040-T044) - Edge cases (waits for US1 completion)
3. **Stories integrate seamlessly** (all build on same foundation)
4. **Team completes Polish together** (T045-T054)

---

## Task Summary

**Total Tasks**: 54

**By Phase**:
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundational): 13 tasks (BLOCKING)
- Phase 3 (User Story 1 - MVP): 13 tasks
- Phase 4 (User Story 2): 8 tasks
- Phase 5 (User Story 3): 5 tasks
- Phase 6 (Polish): 10 tasks

**Parallel Opportunities**: 21 tasks marked [P] (39% parallelizable)

**MVP Scope** (Recommended for first deploy):
- Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (User Story 1)
- Total: 31 tasks
- Delivers: Student asks question → Gets streaming AI response with citations → Conversation persists

**User Story Breakdown**:
- US1 (Primary Flow): 13 tasks - Student asks question with citations
- US2 (Multiple Threads): 8 tasks - Manage multiple conversations
- US3 (Edge Cases): 5 tasks - Handle topics not in textbook

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- **Database setup uses Neon MCP** (no Alembic migrations per user preference)
- **Text-only chatbot** (no file upload support)
- **Citations use inline Markdown footnotes** ([^1] syntax)
- **Agent Tool pattern** for Qdrant search (agent decides when to search)
- Follow quickstart.md for setup validation
