---
description: "Task list for RAG-Powered Chatbot Backend API implementation"
---

# Tasks: RAG-Powered Chatbot Backend API

**Input**: Design documents from `/specs/007-chatbot-backend/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml, quickstart.md

**Tests**: Tests are NOT explicitly requested in the specification. Implementation tasks focus on building functional code with inline validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Project uses **web app structure**: `backend/` at repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend/ directory structure per implementation plan (main.py, agent.py, models.py, database.py, tools.py, session.py, config.py, middleware.py)
- [X] T002 Initialize Python project with requirements.txt containing FastAPI 0.115+, OpenAI Agents SDK, Cohere SDK, Qdrant Client, psycopg2-binary, python-dotenv, uvicorn
- [X] T003 [P] Create .env.example file in backend/ with template for GEMINI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, JWT_SECRET, RATE_LIMIT_PER_MINUTE
- [X] T004 [P] Create backend/README.md with setup instructions referencing quickstart.md
- [X] T005 [P] Configure .gitignore to exclude .env, __pycache__/, *.pyc, venv/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Implement config.py in backend/ to load environment variables using pydantic-settings (GEMINI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, JWT_SECRET, RATE_LIMIT_PER_MINUTE)
- [X] T007 Implement database connection lifecycle in backend/database.py with asynccontextmanager for lifespan management (Neon DB pool with SimpleConnectionPool minconn=2 maxconn=10, Qdrant client, Cohere client)
- [X] T008 Create Pydantic data models in backend/models.py (ChatRequest, ChatResponse, Citation, UserProfile, ConversationSession, ChatMessage, ErrorResponse with SoftwareLevel enum)
- [X] T009 Implement FastAPI app initialization in backend/main.py with lifespan context manager, CORS middleware, and health check endpoint GET /v1/health
- [X] T010 [P] Implement middleware.py in backend/ with CORS configuration, rate limiting using slowapi (20 req/min per user), and structured logging with correlation IDs
- [X] T011 Implement error handling in backend/main.py with custom ChatbotError exception class and exception handlers returning ErrorResponse format with user-friendly messages and error codes

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask Question and Receive Accurate Answer (Priority: P1) 🎯 MVP

**Goal**: Students can ask questions about textbook content and receive accurate, cited answers within 5 seconds

**Independent Test**: Send POST /v1/chat request with question "What is ROS 2 architecture?" and verify response contains relevant answer with Docusaurus-formatted citations like [Chapter Title](/docs/path)

### Implementation for User Story 1

- [X] T012 [P] [US1] Implement fetch_user_profile function in backend/database.py to query Neon DB users table using connection pool (getconn/putconn pattern)
- [X] T013 [P] [US1] Implement SafetyCheckOutput Pydantic model in backend/models.py (is_safe, is_relevant, reason fields) as output_type for guardrail agent
- [X] T014 [US1] Implement search_knowledge_base function tool in backend/tools.py using @function_tool decorator (embed query with Cohere embed-v4.0, search Qdrant robotics_textbook_v1 collection with limit=5 score_threshold=0.7, format results with Docusaurus citations)
- [X] T015 [US1] Configure Gemini client in backend/agent.py using AsyncOpenAI, OpenAIChatCompletionsModel, and RunConfig (base_url=https://generativelanguage.googleapis.com/v1beta/openai/, model=gemini-2.0-flash, tracing_disabled=True)
- [X] T016 [US1] Implement guardrail agent in backend/agent.py (name="Safety Guardrail", instructions for safety/relevance check, output_type=SafetyCheckOutput)
- [X] T017 [US1] Implement @input_guardrail decorator function safety_guardrail in backend/agent.py (calls Runner.run with guardrail agent, returns GuardrailFunctionOutput with tripwire_triggered logic)
- [X] T018 [US1] Implement main chatbot agent in backend/agent.py (name="Robotics Tutor", instructions with user profile context, tools=[search_knowledge_base], input_guardrails=[safety_guardrail])
- [X] T019 [US1] Implement session management functions in backend/session.py (create_session, get_session, initialize with user profile as messages[0] system message)
- [X] T020 [US1] Implement POST /v1/chat endpoint in backend/main.py using Runner.run (validate ChatRequest, manage session, await Runner.run(main_agent, input, run_config, session), catch InputGuardrailTripwireTriggered exception, extract final_output from RunResult, return ChatResponse)
- [X] T021 [US1] Add response processing in backend/agent.py to extract citations from RunResult.final_output and format as Citation objects with chapter_title, doc_url, relevance_score
- [X] T022 [US1] Add error handling for InputGuardrailTripwireTriggered exception in POST /v1/chat (access e.guardrail_result.output_info for SafetyCheckOutput, return ERR_VAL_004 for unsafe, ERR_VAL_005 for irrelevant)
- [X] T023 [US1] Add error handling for external service failures in backend/main.py (Qdrant unavailable → ERR_DB_001, Cohere embedding failure → ERR_TOOL_001, Gemini API error → ERR_AGENT_001, Neon DB error → ERR_DB_002)

**Checkpoint**: At this point, User Story 1 should be fully functional - students can ask questions and receive accurate cited answers

---

## Phase 4: User Story 2 - Maintain Conversation Context (Priority: P2)

**Goal**: Students can ask follow-up questions without repeating context, chatbot understands conversation history

**Independent Test**: Send sequence of messages in same session: "What is ROS 2?" followed by "How do I install it?" and verify second response understands "it" refers to ROS 2

### Implementation for User Story 2

- [ ] T022 [US2] Implement add_message function in backend/session.py to append user/assistant messages to ConversationSession.messages array while maintaining messages[0] system message
- [ ] T023 [US2] Implement get_conversation_context in backend/session.py to return full messages array for agent with profile system message as first element
- [ ] T024 [US2] Update main chatbot agent in backend/agent.py to accept full messages array as conversation history (not just latest message)
- [ ] T025 [US2] Implement token counting in backend/session.py using tiktoken or similar to track total tokens in messages array
- [ ] T026 [US2] Add context window management in backend/session.py to ensure messages array stays within 8,000 token limit (trim older messages if needed while preserving messages[0] system message)
- [ ] T027 [US2] Update POST /v1/chat endpoint in backend/main.py to handle session_id parameter (retrieve existing session or create new one)
- [ ] T028 [US2] Update ChatResponse in backend/models.py to include session_id, processing_time_ms, token_count fields
- [ ] T029 [US2] Add last_activity timestamp update in backend/session.py on each message to track session activity

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - students can have multi-turn conversations with context retention

---

## Phase 5: User Story 3 - Handle Out-of-Scope Questions Gracefully (Priority: P2)

**Goal**: Chatbot politely indicates it can only answer textbook questions without hallucinating when asked off-topic questions

**Independent Test**: Send off-topic question "What's the weather today?" and verify response indicates textbook-only scope without fabricating answers

### Implementation for User Story 3

- [ ] T030 [P] [US3] Enhance input guardrail agent in backend/agent.py to detect out-of-scope questions (check if query relates to robotics/course content)
- [ ] T031 [US3] Add out-of-scope detection logic in backend/agent.py using guardrail agent result (if irrelevant, return canned response without calling main agent)
- [ ] T032 [US3] Implement get_no_results_response function in backend/agent.py to return helpful message when Qdrant search returns no results above threshold ("I couldn't find information about that topic in the textbook. Please ask about course content.")
- [ ] T033 [US3] Add confidence scoring in backend/tools.py based on Qdrant relevance scores (if all results <0.5, flag as low confidence)
- [ ] T034 [US3] Update main agent system prompt in backend/agent.py to instruct model to decline answering when search results are not relevant ("Only answer based on provided textbook context. If context doesn't contain answer, say you don't have that information.")
- [ ] T035 [US3] Add response validation in backend/agent.py to detect hallucination patterns (check if response references information not in retrieved context)

**Checkpoint**: All three core user stories (P1, P2) should now be independently functional - complete RAG chatbot with safety

---

## Phase 6: User Story 4 - Track User Interactions for Analytics (Priority: P3)

**Goal**: Course administrators can see which questions students ask most frequently to identify knowledge gaps and improve content

**Independent Test**: Send various questions and verify query logs are created with user_id, question, response, timestamp for analytics queries

### Implementation for User Story 4

- [ ] T036 [P] [US4] Create analytics logging module in backend/analytics.py with log_interaction function (captures user_id, session_id, question, response, citations, confidence_score, processing_time, timestamp)
- [ ] T037 [US4] Integrate analytics logging in POST /v1/chat endpoint in backend/main.py to log every successful interaction
- [ ] T038 [US4] Add structured logging format in backend/analytics.py using structlog with JSON output for easy parsing
- [ ] T039 [US4] Implement log_error function in backend/analytics.py to track failed requests (error code, error message, user_id, query if available)
- [ ] T040 [P] [US4] Add query frequency tracking in backend/analytics.py (count similar questions using embedding similarity)
- [ ] T041 [P] [US4] Add low-quality response detection in backend/analytics.py (flag interactions with confidence_score <0.5 or no citations)

**Checkpoint**: All user stories complete - full-featured chatbot with analytics for continuous improvement

---

## Phase 7: Session Management Endpoints (Supporting Features)

**Purpose**: Additional API endpoints for session management

- [ ] T042 [P] Implement GET /v1/sessions/{session_id} endpoint in backend/main.py to retrieve session details (SessionDetails with session_id, user_id, created_at, last_activity, message_count, token_count, is_active)
- [ ] T043 [P] Implement DELETE /v1/sessions/{session_id} endpoint in backend/main.py to end active session (mark inactive, clear from memory)
- [ ] T044 [P] Implement session cleanup in backend/session.py to remove inactive sessions after 1 hour timeout
- [ ] T045 [P] Add session auto-termination in backend/session.py when user starts new session (enforce single active session per user)

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T046 [P] Add comprehensive docstrings to all functions in backend/agent.py, backend/tools.py, backend/session.py, backend/database.py
- [ ] T047 [P] Add type hints verification with mypy across all backend/ files
- [ ] T048 [P] Implement structured logging with correlation IDs in backend/middleware.py for request tracing
- [ ] T049 [P] Add health check dependency status in GET /v1/health endpoint (ping Neon DB, Qdrant, Cohere, Gemini and return latency)
- [ ] T050 [P] Create deployment documentation in backend/README.md with environment variables, dependencies, startup sequence
- [ ] T051 [P] Add performance monitoring instrumentation (request duration, token counts, API call latencies)
- [ ] T052 [P] Implement graceful shutdown in backend/main.py lifespan shutdown (close all DB connections with db_pool.closeall(), qdrant_client.close())
- [ ] T053 Validate implementation against quickstart.md test scenarios (health check, chat message, conversation continuity)
- [ ] T054 Run end-to-end validation of all user stories in priority order (US1 → US2 → US3 → US4)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories 🎯 **START HERE FOR MVP**
  - User Story 2 (P2): Can start after Foundational - Extends US1 session management but independently testable
  - User Story 3 (P2): Can start after Foundational - Enhances US1 guardrails but independently testable
  - User Story 4 (P3): Can start after Foundational - Adds analytics layer, completely independent
- **Session Management (Phase 7)**: Can start after Foundational, enhances US2
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

```
Foundational (Phase 2) ✅ MUST COMPLETE FIRST
    ↓
    ├─→ User Story 1 (P1) 🎯 MVP - Independent, no dependencies
    ├─→ User Story 2 (P2) - Independent, builds on US1 session concept
    ├─→ User Story 3 (P2) - Independent, enhances US1 guardrails
    └─→ User Story 4 (P3) - Independent, analytics layer
```

### Within Each User Story

**User Story 1** (Core RAG):
1. T012 (fetch profile) and T013 (format profile) can run in parallel [P]
2. T014 (search tool) after config/database setup
3. T015 (guardrail agent) independent [P]
4. T016 (main agent) after T014, T015
5. T017 (session management) after T013
6. T018 (chat endpoint) after T016, T017
7. T019-T021 (validation, errors) after T018

**User Story 2** (Context):
1. All tasks sequential (T022 → T023 → T024 → T025 → T026 → T027 → T028 → T029)

**User Story 3** (Safety):
1. T030 (guardrail enhancement) and T032 (no results) can run in parallel [P]
2. T031, T033, T034, T035 sequential after T030

**User Story 4** (Analytics):
1. T036 (logging module) first [P]
2. T037 (integration) after T036
3. T038, T039 extend T036 [P]
4. T040, T041 analytics features [P]

### Parallel Opportunities

- **Setup Phase**: T003, T004, T005 can all run in parallel [P]
- **Foundational Phase**: T010 can run parallel with others [P]
- **User Story 1**: T012 + T013 parallel, T015 parallel with others
- **User Story 3**: T030 + T032 parallel, T038 + T039 parallel, T040 + T041 parallel
- **Session Management**: All T042, T043, T044, T045 can run in parallel [P]
- **Polish Phase**: Most tasks T046-T052 can run in parallel [P]
- **Different user stories** can be worked on in parallel by different team members after Foundational phase

---

## Parallel Example: User Story 1

```bash
# Launch parallel tasks for User Story 1:

# Step 1: Profile management (parallel)
Task T012: "Implement fetch_user_profile function in backend/database.py"
Task T013: "Implement UserProfile.to_system_message() in backend/models.py"

# Step 2: Agent components (guardrail independent)
Task T015: "Implement input guardrail agent in backend/agent.py"

# While T015 runs, can work on:
Task T014: "Implement search_knowledge_base tool in backend/tools.py"

# After both complete:
Task T016: "Implement main chatbot agent in backend/agent.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only) 🎯 **RECOMMENDED START**

1. **Complete Phase 1: Setup** (T001-T005) - ~30 minutes
2. **Complete Phase 2: Foundational** (T006-T011) - ~3 hours (CRITICAL)
3. **Complete Phase 3: User Story 1** (T012-T021) - ~6 hours
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Send question: "What is ROS 2 architecture?"
   - Verify: Response with citations
   - Check: <5 second response time
   - Validate: Error handling works
5. **Deploy/demo if ready** - Working MVP chatbot! 🎉

**Total MVP Time**: ~10 hours for complete working chatbot

### Incremental Delivery

1. **Foundation** (Phase 1-2) → ~4 hours → Foundation ready
2. **+ User Story 1** (Phase 3) → ~6 hours → **Deploy/Demo MVP** 🎯
3. **+ User Story 2** (Phase 4) → ~3 hours → Multi-turn conversations → Deploy/Demo
4. **+ User Story 3** (Phase 5) → ~2 hours → Safety improvements → Deploy/Demo
5. **+ User Story 4** (Phase 6) → ~2 hours → Analytics for admins → Deploy/Demo
6. **+ Session APIs** (Phase 7) → ~1 hour → Complete session management
7. **+ Polish** (Phase 8) → ~3 hours → Production-ready

**Total Time**: ~21 hours for complete implementation

Each story adds value without breaking previous stories.

### Parallel Team Strategy

With multiple developers:

1. **Team completes Setup + Foundational together** (~4 hours)
2. **Once Foundational is done**, split work:
   - **Developer A**: User Story 1 (T012-T021) - Core RAG 🎯
   - **Developer B**: User Story 3 (T030-T035) - Safety features
   - **Developer C**: User Story 4 (T036-T041) - Analytics
3. **After US1 complete**: Developer A adds User Story 2 (T022-T029)
4. **Integration**: All stories work independently, minimal integration needed

**Team Time**: ~12 hours to complete all features

---

## Validation Checkpoints

### After Phase 2 (Foundational)

- ✅ FastAPI app starts with `uvicorn backend.main:app --reload`
- ✅ GET /v1/health returns 200 status
- ✅ Environment variables load correctly from .env
- ✅ Database connections initialize (check logs for "✅ Neon DB pool created", "✅ Qdrant client initialized", "✅ Cohere client initialized")

### After Phase 3 (User Story 1 - MVP)

- ✅ POST /v1/chat accepts question and returns answer
- ✅ Response includes Docusaurus-formatted citations
- ✅ Response time <5 seconds
- ✅ User profile fetched from Neon DB and used in system message
- ✅ Agent calls search_knowledge_base tool
- ✅ Qdrant returns relevant textbook chunks
- ✅ Guardrail agent blocks harmful queries
- ✅ Error handling returns user-friendly messages with codes

### After Phase 4 (User Story 2)

- ✅ Follow-up questions work in same session
- ✅ session_id returned and accepted in subsequent requests
- ✅ Conversation history maintained in messages array
- ✅ Profile system message remains as messages[0]
- ✅ Token count tracked and managed
- ✅ Context window stays within 8,000 tokens

### After Phase 5 (User Story 3)

- ✅ Off-topic questions receive polite decline message
- ✅ Harmful content blocked by guardrail
- ✅ No-results questions handled gracefully
- ✅ Low-confidence responses flagged appropriately
- ✅ No hallucinations in test scenarios

### After Phase 6 (User Story 4)

- ✅ All interactions logged with structured format
- ✅ Analytics data includes user_id, question, response, metadata
- ✅ Error interactions logged separately
- ✅ Query frequency tracking works
- ✅ Low-quality responses flagged in logs

### After Phase 8 (Polish)

- ✅ All functions have docstrings
- ✅ Type hints verified with mypy
- ✅ Health check shows all dependencies UP
- ✅ Structured logging with correlation IDs
- ✅ Graceful shutdown closes all connections
- ✅ quickstart.md validation passes
- ✅ All user stories tested end-to-end

---

## Notes

- **[P] tasks** = different files, no dependencies on other in-progress tasks
- **[Story] label** maps task to specific user story for traceability
- Each user story should be **independently completable and testable**
- **Tests are NOT included** as they were not requested in the specification
- Commit after each task or logical group of parallel tasks
- Stop at any checkpoint to validate story independently
- **Priority**: Focus on User Story 1 (P1) first for MVP, then P2 stories, then P3
- **Avoid**: Cross-story dependencies that break independence
- **Profile optimization**: Fetched once at session start (T012, T017) - 90% DB load reduction
- **Agent architecture**: Two-layer (guardrail → main agent) with tool-calling pattern (T014, T015, T016)
- **Citations**: Docusaurus-formatted clickable links in all responses (T014, T019)

---

## Task Count Summary

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 6 tasks ⚠️ BLOCKING
- **Phase 3 (User Story 1 - P1)**: 10 tasks 🎯 MVP
- **Phase 4 (User Story 2 - P2)**: 8 tasks
- **Phase 5 (User Story 3 - P2)**: 6 tasks
- **Phase 6 (User Story 4 - P3)**: 6 tasks
- **Phase 7 (Session Management)**: 4 tasks
- **Phase 8 (Polish)**: 9 tasks

**Total**: 54 tasks

**Parallel opportunities**: 22 tasks marked [P] can run in parallel with others

**MVP scope** (Phases 1-3): 21 tasks for working chatbot 🎯

**Full feature set** (All phases): 54 tasks for production-ready system
