# Feature Specification: ChatKit Backend with Google Gemini Integration

**Feature Branch**: `008-chatkit-gemini-backend`
**Created**: 2025-12-23
**Status**: Draft
**Input**: User description: "Build a production-ready FastAPI backend for OpenAI ChatKit that uses Google Gemini as the primary LLM for inference while maintaining compatibility with ChatKit's features."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Send Message and Receive Streaming Response (Priority: P1)

A user wants to send a message in a conversation thread and receive the AI's response in real-time as it's being generated, creating a natural conversational experience where text appears progressively rather than all at once.

**Why this priority**: This is the core value proposition of a chat interface. Without reliable message sending and streaming responses, the system has no functional value. This directly impacts user experience and perceived responsiveness.

**Independent Test**: Can be fully tested by creating a thread, sending a message, and verifying that the response streams back in chunks with appropriate streaming events. Delivers immediate value by enabling basic conversational interactions.

**Acceptance Scenarios**:

1. **Given** a user has an active conversation thread, **When** they send the message "Explain inverse kinematics", **Then** the system returns a streamed response that begins appearing within 2 seconds and completes the full answer within 30 seconds
2. **Given** a user sends a message, **When** the AI generates a response, **Then** response chunks are delivered incrementally via server-sent events or streaming protocol, allowing the UI to display text as it's generated
3. **Given** a streaming response is in progress, **When** a network interruption occurs mid-stream, **Then** the system gracefully handles the interruption and allows the user to retry without data loss
4. **Given** a user sends a complex multi-part question, **When** the AI processes it, **Then** the response maintains coherent structure while streaming (not broken mid-sentence or mid-thought)

---

### User Story 2 - View Agentic Actions and Chain-of-Thought (Priority: P2)

A user wants to see what the AI agent is doing behind the scenes when processing their request, including which tools it's using, what decisions it's making, and its reasoning process, to build trust and understand the AI's capabilities.

**Why this priority**: Transparency in AI decision-making builds user trust and helps debug issues. While not required for basic functionality, it significantly enhances the user experience and differentiates this from simple chat interfaces.

**Independent Test**: Can be tested by sending a message that requires tool use (e.g., "Search the knowledge base for ROS 2 tutorials"), and verifying that intermediate actions (search initiation, results retrieval, synthesis) are exposed as distinct events. Delivers value by making AI behavior transparent and educational.

**Acceptance Scenarios**:

1. **Given** a user asks a question requiring knowledge base search, **When** the agent processes the request, **Then** the system emits action events showing: "Analyzing query", "Searching knowledge base", "Synthesizing results"
2. **Given** the agent uses multiple tools sequentially, **When** processing a complex request, **Then** each tool invocation is exposed as a separate action with clear descriptions of purpose and outcome
3. **Given** an agent action completes, **When** the UI receives the action event, **Then** it includes metadata such as action type, tool name, input parameters (sanitized), and completion status
4. **Given** an agent encounters an error during tool execution, **When** the action fails, **Then** the system exposes the failure as an action event with user-friendly explanation

---

### User Story 3 - Interact with Widgets in Conversation (Priority: P2)

A user wants to interact with rich UI components embedded in the conversation (e.g., buttons, forms, visualizations) that can trigger server-side actions or client-side effects, enabling more engaging and functional conversations beyond plain text.

**Why this priority**: Interactive widgets transform the chat from simple Q&A to a powerful interface for complex tasks. This is essential for production-ready applications but can be built after basic messaging works.

**Independent Test**: Can be tested by triggering a message that returns a widget (e.g., a multiple-choice quiz or a file selector), interacting with the widget, and verifying that both server-handled actions and client-only interactions work correctly. Delivers value by enabling rich, interactive learning experiences.

**Acceptance Scenarios**:

1. **Given** the AI wants to present options to the user, **When** it generates a response, **Then** the system includes a widget specification (e.g., button group with options) that the client can render and interact with
2. **Given** a widget requires server-side action (e.g., submitting a form), **When** the user interacts with it, **Then** the action is sent to the backend, processed, and a new response (with or without another widget) is returned
3. **Given** a widget is client-only (e.g., expanding/collapsing content, showing a visualization), **When** the user interacts with it, **Then** no server request is required and the interaction is handled locally
4. **Given** multiple widgets are present in a conversation, **When** the user interacts with one, **Then** the system correctly identifies which widget was triggered and maintains conversation state

---

### User Story 4 - Receive Progress Updates During Long Operations (Priority: P2)

A user initiates a request that requires time-consuming operations (e.g., searching large knowledge bases, processing complex calculations), and wants to see real-time progress updates (e.g., "Searching...", "Found 50 results", "Analyzing...") so they know the system is working and not frozen.

**Why this priority**: Progress feedback prevents user frustration and abandonment during long-running operations. While not critical for MVP, it's essential for production quality and user satisfaction.

**Independent Test**: Can be tested by triggering a request that invokes a tool with artificial delay, and verifying that progress events are emitted at appropriate intervals with meaningful status messages. Delivers value by maintaining user engagement during wait times.

**Acceptance Scenarios**:

1. **Given** a user asks a question requiring knowledge base search, **When** the search tool begins execution, **Then** the system emits a progress event with message "Searching knowledge base..." before results are returned
2. **Given** a multi-step operation is underway (e.g., search → filter → synthesize), **When** each step begins, **Then** a progress event is emitted with a descriptive message (e.g., "Filtering results...", "Generating summary...")
3. **Given** a progress event is emitted, **When** the client receives it, **Then** it includes a unique identifier, timestamp, and human-readable status message
4. **Given** a long-running operation completes or fails, **When** the final state is reached, **Then** the progress updates stop and a completion or error event is emitted

---

### User Story 5 - Automatic Thread Title Generation (Priority: P3)

A user creates a new conversation thread and sends their first message, and the system automatically generates a concise, meaningful title for the thread based on the content of that first message, making it easy to identify and navigate between multiple conversations.

**Why this priority**: Auto-generated titles improve navigation and organization but are not essential for core functionality. This is a quality-of-life enhancement that can be added after messaging, streaming, and actions are working.

**Independent Test**: Can be tested by creating a new thread, sending a first message (e.g., "How do I set up ROS 2 on Ubuntu?"), and verifying that a title like "ROS 2 Ubuntu Setup" is automatically generated and associated with the thread. Delivers value by reducing cognitive load and improving conversation management.

**Acceptance Scenarios**:

1. **Given** a user creates a new thread and sends the first message "What is inverse kinematics?", **When** the system processes the message, **Then** a title agent generates a concise title (e.g., "Inverse Kinematics Explanation") and updates the thread metadata
2. **Given** a thread already has a manually set title, **When** the title agent runs, **Then** it does not overwrite the existing title (auto-generation only for new threads without titles)
3. **Given** the first message is very long or complex, **When** the title agent processes it, **Then** the generated title is kept under 50 characters and captures the main topic
4. **Given** title generation fails or times out, **When** the error occurs, **Then** the system falls back to a default title (e.g., "New Conversation") without blocking the main response

---

### User Story 6 - Upload and Download Attachments (Priority: P3)

A user wants to attach files (e.g., code snippets, diagrams, logs) to their messages for the AI to analyze, and wants to download files generated by the AI (e.g., generated code, visualizations, reports), enabling richer conversations beyond text.

**Why this priority**: File handling is important for many use cases (code review, debugging, data analysis) but is not required for basic text-based conversations. This can be implemented after core chat functionality is stable.

**Independent Test**: Can be tested by uploading a file (e.g., a Python script), sending a message referencing it, verifying the AI processes it, and then downloading a generated file from the AI's response. Delivers value by enabling file-based workflows.

**Acceptance Scenarios**:

1. **Given** a user wants to attach a file, **When** they initiate the upload, **Then** the system provides a signed upload URL that the client can use to upload the file directly to storage
2. **Given** a file has been uploaded, **When** the user sends a message referencing it, **Then** the message includes attachment metadata (file ID, name, type, size) that the backend can access
3. **Given** the AI generates a file as part of its response (e.g., generated code), **When** the response is returned, **Then** it includes a signed download URL that the client can use to retrieve the file
4. **Given** an attachment is no longer needed, **When** the thread is deleted, **Then** the system removes all associated files from storage to manage costs (attachments follow thread lifetime - deleted when thread is deleted)

---

### User Story 7 - Tag Entities with @-Mentions (Priority: P3)

A user wants to reference specific entities (e.g., other users, knowledge base documents, tools, agents) in their messages using @-mention syntax (e.g., "@SearchTool find ROS 2 tutorials"), and the system should recognize these mentions and enable entity-specific behaviors.

**Why this priority**: Entity tagging enables advanced workflows (multi-user collaboration, explicit tool invocation, knowledge graph navigation) but is not essential for single-user chat. This is a power-user feature that can be added later.

**Independent Test**: Can be tested by sending a message with an @-mention (e.g., "@KnowledgeBase what is SLAM?"), and verifying that the system parses the mention, identifies the entity, and triggers appropriate behavior (e.g., direct tool invocation). Delivers value by giving users explicit control over system behavior.

**Acceptance Scenarios**:

1. **Given** a user types "@" in their message, **When** the client detects this trigger, **Then** it provides autocomplete suggestions for available entities (tools, knowledge sources, agents)
2. **Given** a message contains an @-mention (e.g., "@SearchTool ROS 2"), **When** the backend processes the message, **Then** it parses the mention, identifies the entity, and passes context to the agent
3. **Given** multiple entities are mentioned in one message, **When** the agent processes it, **Then** it understands all mentions and can coordinate actions across multiple tools or knowledge sources
4. **Given** an @-mention references an invalid or unavailable entity, **When** the message is processed, **Then** the system treats the invalid mention as regular text and continues processing the message normally (graceful degradation - no request failure for typos)

---

### User Story 8 - Push Client Effects from Server (Priority: P3)

The backend needs to instruct the client to perform specific UI actions (e.g., scroll to a specific message, highlight text, open a modal, navigate to a different view) as part of the conversation flow, enabling server-driven UI orchestration beyond just displaying messages.

**Why this priority**: Client effects enable sophisticated UI workflows but are not necessary for basic chat. This is an advanced feature for building app-like experiences within chat and can be implemented after core features are stable.

**Independent Test**: Can be tested by triggering a server response that includes a client effect instruction (e.g., "scroll_to_message"), and verifying that the client receives and executes the effect. Delivers value by enabling dynamic, context-aware UI experiences.

**Acceptance Scenarios**:

1. **Given** the AI wants to reference a previous message, **When** it generates a response, **Then** it can include a client effect instruction to scroll to that message and highlight it temporarily
2. **Given** a conversation requires user action (e.g., confirming a choice), **When** the AI requests confirmation, **Then** it can include a client effect to open a confirmation modal or display a banner
3. **Given** multiple client effects are needed, **When** the backend sends the response, **Then** effects are ordered and executed sequentially by the client
4. **Given** a client effect fails to execute (e.g., unsupported effect type), **When** the error occurs, **Then** the client logs the error but continues to display the message content (graceful degradation)

---

### Edge Cases

- What happens when a user sends an empty message or only whitespace?
- How does the system handle extremely long messages (>10,000 characters)?
- What if the Gemini API is temporarily unavailable or rate-limited?
- How does the system respond when streaming is interrupted mid-response?
- What happens if a user tries to access a thread they don't have permission to view?
- How does the system handle concurrent message sends in the same thread?
- What if an attachment upload fails partway through?
- How does the system handle malformed @-mentions (e.g., "@" with no entity name)?
- What happens when a widget action times out or fails?
- How does the system handle progress updates that never complete (hanging operations)?
- What if the title generation agent produces an inappropriate or offensive title?
- How does the system handle messages containing only emojis or special characters?
- What happens when authentication tokens expire mid-conversation?
- How does the system handle requests that exceed the Gemini context window?

## Requirements *(mandatory)*

### Functional Requirements

#### Core Messaging & Storage
- **FR-001**: System MUST persist all conversation threads with unique identifiers, creation timestamps, and last activity timestamps in self-hosted storage
- **FR-002**: System MUST persist all messages within threads, including message ID, thread ID, author (user or assistant), content, timestamp, and ordering sequence
- **FR-003**: System MUST support creating new conversation threads with optional initial metadata (user-provided title, tags, context)
- **FR-004**: System MUST support sending messages to existing threads and appending them to the conversation history in chronological order
- **FR-005**: System MUST retrieve conversation history for a given thread, including all messages in chronological order

#### Authentication & Authorization
- **FR-006**: System MUST authenticate incoming requests using custom authentication headers provided by the client
- **FR-007**: System MUST implement a custom fetch method pattern that allows the ChatKit client to inject authentication tokens into all API requests
- **FR-008**: System MUST validate user permissions to access threads (users can only access their own threads or threads explicitly shared with them)
- **FR-009**: System MUST associate each thread with the authenticated user who created it

#### Response Streaming
- **FR-010**: System MUST generate AI responses using Google Gemini as the primary language model
- **FR-011**: System MUST stream AI responses in real-time as they are generated, rather than waiting for the complete response
- **FR-012**: System MUST use server-sent events (SSE) or a compatible streaming protocol to deliver response chunks to the client
- **FR-013**: System MUST maintain conversation context by sending the full thread history to the language model with each request
- **FR-014**: System MUST handle streaming errors gracefully, allowing clients to detect incomplete responses and retry

#### Agentic Actions & Chain-of-Thought
- **FR-015**: System MUST expose agent actions (tool invocations, decision steps, reasoning) as structured events during response generation
- **FR-016**: System MUST emit action events that include action type, tool name, input parameters (sanitized for security), status (started, completed, failed), and timestamps
- **FR-017**: System MUST support multiple sequential actions in a single request (agent chains multiple tool calls to complete a task)
- **FR-018**: System MUST differentiate between internal reasoning (chain-of-thought) and external actions (tool calls) in emitted events

#### Interactive Widgets
- **FR-019**: System MUST support embedding widget specifications in AI responses, including widget type, properties, and interaction handlers
- **FR-020**: System MUST distinguish between server-handled widget actions (require backend processing) and client-only actions (handled locally)
- **FR-021**: System MUST process server-handled widget actions by accepting widget interaction events, executing the associated logic, and returning updated responses
- **FR-022**: System MUST support common widget types including buttons, button groups, forms, select menus, and expandable content

#### Progress Updates
- **FR-023**: System MUST emit progress update events during long-running operations (tool calls, searches, computations) to inform users of current status
- **FR-024**: Progress update events MUST include a unique operation identifier, timestamp, progress message (human-readable), and optional progress percentage
- **FR-025**: System MUST emit progress updates at meaningful milestones (operation started, intermediate steps, operation completed or failed)
- **FR-026**: System MUST stop emitting progress updates once an operation completes, fails, or is cancelled

#### Client Effects
- **FR-027**: System MUST support sending client effect instructions as part of response payloads to trigger specific UI behaviors
- **FR-028**: Client effects MUST include effect type (e.g., scroll_to_message, highlight_text, open_modal, navigate), target identifiers (e.g., message ID), and parameters
- **FR-029**: System MUST allow multiple client effects in a single response, with execution order defined by array sequence
- **FR-030**: System MUST document supported client effect types and their expected parameters for client implementation

#### Thread Title Generation
- **FR-031**: System MUST invoke a title generation agent after the first user message in a new thread (threads without existing titles)
- **FR-032**: The title agent MUST analyze the first message and generate a concise, descriptive title (maximum 50 characters) that captures the main topic
- **FR-033**: System MUST update the thread metadata with the generated title asynchronously (should not block the main response)
- **FR-034**: System MUST NOT overwrite manually set titles or titles that already exist on threads

#### Attachment Handling
- **FR-035**: System MUST generate signed upload URLs that clients can use to upload attachments directly to storage without routing through the backend
- **FR-036**: System MUST accept messages with attachment metadata (file ID, name, type, size) and associate attachments with messages and threads
- **FR-037**: System MUST provide attachment context to the AI agent when processing messages that reference attachments
- **FR-038**: System MUST generate signed download URLs for attachments created by the AI or previously uploaded by users
- **FR-039**: System MUST enforce attachment size limits and validate file types to prevent abuse
- **FR-040**: System MUST implement attachment retention policies to manage storage costs and comply with data retention requirements

#### Entity Tagging (@-Mentions)
- **FR-041**: System MUST parse messages to detect @-mention syntax (e.g., @EntityName) and extract entity references
- **FR-042**: System MUST maintain a registry of mentionable entities (tools, knowledge sources, agents, users) with unique identifiers and display names
- **FR-043**: System MUST provide autocomplete suggestions for entities when requested by the client (triggered by "@" input)
- **FR-044**: System MUST pass parsed entity mentions to the AI agent as structured context to enable entity-specific behaviors
- **FR-045**: System MUST handle invalid @-mentions gracefully without failing the entire request

#### Error Handling & Resilience
- **FR-046**: System MUST return user-friendly error messages when requests fail, without exposing sensitive implementation details
- **FR-047**: System MUST implement retry logic with exponential backoff for transient failures in external services (Gemini API, storage)
- **FR-048**: System MUST validate all incoming request payloads and reject malformed requests with clear error messages
- **FR-049**: System MUST handle API rate limits from Gemini gracefully, queueing requests or returning informative errors

#### Logging & Observability
- **FR-050**: System MUST log all API requests including user ID, thread ID, message content (sanitized for PII), timestamps, and response status
- **FR-051**: System MUST log all errors with sufficient context for debugging (stack traces, request IDs, affected resources)
- **FR-052**: System MUST emit metrics for request latency, response streaming duration, error rates, and API usage
- **FR-053**: System MUST support distributed tracing to track requests across service boundaries (API → storage, API → Gemini)

### Key Entities

- **Thread**: Represents a conversation with attributes: thread_id (unique identifier), user_id (owner), title (optional, auto-generated or manual), created_at (timestamp), updated_at (timestamp), metadata (tags, context, custom fields), message_count
- **Message**: Represents a single message in a thread with attributes: message_id (unique identifier), thread_id (parent thread), role (user or assistant), content (text), attachments (list of attachment references), created_at (timestamp), sequence_number (order in thread)
- **Attachment**: Represents a file associated with a message with attributes: attachment_id (unique identifier), message_id (parent message), file_name, file_type (MIME type), file_size (bytes), storage_url (internal), upload_url (signed, temporary), download_url (signed, temporary), created_at, expires_at
- **AgentAction**: Represents an action taken by the AI agent with attributes: action_id (unique identifier), thread_id, message_id, action_type (tool_call, reasoning_step, decision), tool_name (if applicable), input_params (sanitized), output_result (sanitized), status (started, completed, failed), created_at, duration_ms
- **ProgressUpdate**: Represents a progress event during operations with attributes: update_id (unique identifier), operation_id, thread_id, message_id, progress_message (human-readable), progress_percentage (0-100, optional), timestamp
- **Widget**: Represents an interactive UI component with attributes: widget_id (unique identifier), widget_type (button, button_group, form, select, etc.), properties (configuration), action_handler (server or client), state (current values)
- **ClientEffect**: Represents a server instruction to the client with attributes: effect_id (unique identifier), effect_type (scroll_to_message, highlight_text, open_modal, navigate), target (message_id, element_id, etc.), parameters (effect-specific config), execution_order
- **EntityMention**: Represents an @-mention in a message with attributes: mention_id, message_id, entity_type (tool, knowledge_source, agent, user), entity_id (referenced entity), entity_name (display name), position_in_message (start, end)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can create a new thread and send a message, receiving a streamed response that begins within 2 seconds and completes within 30 seconds for typical queries
- **SC-002**: System successfully streams responses with latency under 100ms per chunk for 95% of streaming operations
- **SC-003**: System handles at least 100 concurrent conversation threads without degradation in response time or streaming performance
- **SC-004**: Agent actions are exposed as structured events in at least 90% of requests that involve tool usage or multi-step reasoning
- **SC-005**: Interactive widgets render correctly in client applications and server-handled actions complete within 3 seconds for 95% of interactions
- **SC-006**: Progress updates are emitted within 500ms of operation start for all long-running operations (>2 seconds duration)
- **SC-007**: Thread titles are automatically generated with relevance score above 80% (evaluated by human reviewers or test suite)
- **SC-008**: Attachment uploads complete successfully in under 10 seconds for files up to 10MB, with signed URL generation completing in under 500ms
- **SC-009**: @-mention autocomplete suggestions are returned within 300ms and match user intent in at least 85% of test cases
- **SC-010**: Client effects execute correctly in test clients with success rate above 95%
- **SC-011**: System uptime is at least 99.5% during normal operation (excluding planned maintenance)
- **SC-012**: Authentication failures are detected and rejected within 100ms with clear error messages
- **SC-013**: Error recovery succeeds in at least 80% of transient failure scenarios (network issues, API timeouts) without user intervention
- **SC-014**: Users report satisfaction score of 4.2/5 or higher when surveyed about conversation quality and responsiveness

## Assumptions *(if applicable)*

- Users have unique identifiers (user_id) provided by an existing authentication system
- The ChatKit Python SDK and OpenAI Agents SDK are compatible with Google Gemini when configured with appropriate adapters
- Google Gemini API supports streaming responses and function calling capabilities required for agentic workflows
- Clients (web, mobile) can handle server-sent events or equivalent streaming protocols for receiving real-time updates
- Self-hosted storage infrastructure (Neon PostgreSQL database for messages/threads, object storage for attachments) is available and scalable; new chatbot tables will be added to the existing auth database in the same schema for simplified data access and management
- Users primarily interact in English (internationalization is out of scope for this feature)
- Network connectivity between backend and external services (Gemini API, storage) is reliable with standard retry mechanisms
- Attachment storage costs are acceptable for the expected user base and usage patterns
- The existing authentication backend can generate and validate JWT tokens or equivalent authentication credentials
- Clients implement ChatKit UI components that can render widgets, display progress updates, and execute client effects
- @-mention syntax follows standard conventions (@ followed by entity name with optional spaces)
- Thread titles generated by the title agent do not require manual approval before being displayed to users

## Dependencies *(if applicable)*

- **ChatKit Python SDK**: Backend framework for building chat applications with built-in support for threads, messages, and UI components
- **OpenAI Agents SDK**: Agentic framework for building AI agents with tool calling, chain-of-thought, and orchestration capabilities
- **Google Gemini API**: Primary language model for generating conversational responses, reasoning, and tool usage
- **Self-Hosted Database**: Neon PostgreSQL (existing auth database) - new tables will be added for threads, messages, and attachments in the same database alongside existing auth tables
- **Object Storage**: S3-compatible storage (AWS S3, MinIO, Cloudflare R2) for attachment uploads and downloads
- **Authentication Service**: Existing or new service that generates and validates authentication tokens for API requests
- **ChatKit Client SDK**: Frontend SDKs (React, Vue, mobile) that implement ChatKit UI components and handle streaming, widgets, and effects

## Constraints *(if applicable)*

- Must use Google Gemini as the primary LLM (not OpenAI GPT, Claude, or other models) while maintaining compatibility with OpenAI Agents SDK
- Must self-host all user data (messages, threads, attachments) rather than relying on third-party managed services like Firebase or Supabase
- Must implement custom authentication rather than using ChatKit's default auth (if any), to integrate with existing user management
- Responses must be streamed in real-time; buffering full responses before sending is not acceptable
- Attachment uploads must use signed URLs and direct-to-storage patterns to avoid routing large files through the backend
- Widget schemas must be compatible with ChatKit's widget specifications to ensure client rendering works correctly
- Client effects must be advisory (clients can choose not to implement certain effects) and not critical to conversation functionality
- @-mention parsing must not interfere with messages that naturally contain "@" symbols in non-mention contexts (e.g., email addresses)
- System must be stateless at the API level to support horizontal scaling and load balancing
- Must leverage existing authentication system's GDPR-compliant data handling; no additional privacy compliance requirements needed beyond standard security practices for educational use

## Out of Scope *(if applicable)*

- Frontend chat UI implementation (clients are responsible for rendering conversations using ChatKit SDKs)
- User authentication system implementation (assumes existing auth service provides user_id and tokens)
- Multi-language support and internationalization (English-only for this feature)
- Voice input/output capabilities
- Video or image generation features
- Integration with external communication platforms (Slack, Discord, Teams)
- Advanced analytics dashboard for administrators (basic logging is in scope, dashboards are not)
- Payment processing or subscription management
- Mobile push notifications for new messages
- Real-time collaborative editing of messages
- Message encryption at rest (standard storage security applies, E2E encryption is out of scope)
- Custom AI model fine-tuning or training
- Multi-tenant architecture with organization-level isolation (single-tenant or user-level isolation only)

## Risks & Mitigations *(if applicable)*

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Gemini API compatibility issues with OpenAI Agents SDK | High | Medium | Validate adapter compatibility early with proof-of-concept; maintain abstraction layer to swap models if needed |
| Streaming response interruptions cause poor user experience | Medium | Medium | Implement robust reconnection logic, message persistence, and retry mechanisms; test under poor network conditions |
| Widget schemas incompatible between server and client | High | Low | Define strict widget schema contracts; validate schemas in automated tests; version widget specs |
| Attachment storage costs exceed budget | Medium | Medium | Implement file size limits, retention policies, and compression; monitor storage metrics closely |
| Gemini API rate limits impact user experience during peak usage | High | Medium | Implement request queuing, user-level rate limiting, and graceful degradation; consider premium API tiers |
| Authentication token validation adds latency to every request | Low | High | Implement token caching with short TTLs; use efficient validation algorithms (JWT signature verification) |
| Title generation produces inappropriate or low-quality titles | Medium | Low | Implement content filtering on generated titles; allow manual override; fall back to default titles on errors |
| Complex @-mention parsing causes false positives or negatives | Low | Medium | Use conservative parsing rules; provide clear documentation; allow disabling @-mentions if problematic |
| Concurrent message sends to same thread cause race conditions | Medium | Low | Implement optimistic locking or sequence numbers; ensure database transactions handle concurrency |
| Client effect execution failures cause confusion | Low | Medium | Design effects as optional enhancements; ensure core functionality works without effects; log client-side errors |

## Open Questions *(if applicable)*

- None - all critical ambiguities have been resolved through clarification

## References *(if applicable)*

- ChatKit Python SDK Documentation: https://openai.github.io/chatkit-python/
- OpenAI Agents SDK Quickstart: https://openai.github.io/openai-agents-python/quickstart/
- ChatKit Advanced Samples Repository: https://github.com/openai/openai-chatkit-advanced-samples
- Google Gemini API Documentation: https://ai.google.dev/docs
- OpenAI Agents SDK with Gemini Integration Pattern: (User-provided example code)
- Server-Sent Events Specification: https://html.spec.whatwg.org/multipage/server-sent-events.html
