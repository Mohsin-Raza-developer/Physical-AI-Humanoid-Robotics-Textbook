# Feature Specification: ChatKit-Integrated Robotics Chatbot Backend

**Feature ID**: 008-chatkit-gemini-backend
**Created**: 2025-12-25
**Status**: Draft
**Priority**: High

---

## Overview

### Purpose

Build a production-ready chatbot backend that integrates OpenAI's ChatKit framework with Google Gemini AI to provide an intelligent robotics learning assistant. The system will serve as the backend for an interactive textbook on Physical AI and Humanoid Robotics, enabling students to ask questions, receive contextual answers with citations, and interact with a pre-built chat interface.

### Background

The Physical AI and Humanoid Robotics textbook (built with Docusaurus) currently has user authentication via Better Auth but lacks an intelligent conversational interface. Students need to be able to ask questions about robotics concepts, receive answers grounded in the textbook content, and have their conversations persist across sessions. The backend must seamlessly integrate with the existing Better Auth authentication system and work with ChatKit's React components for a polished user experience.

### Business Value

- **Improved Learning Outcomes**: Students can get immediate, contextual help with robotics concepts
- **Reduced Support Burden**: Automated assistance reduces manual support requests
- **Enhanced User Engagement**: Conversational interface increases time spent with learning materials
- **Scalable Knowledge Delivery**: AI-powered responses can serve unlimited concurrent users
- **Measurable Learning Analytics**: Track commonly asked questions and knowledge gaps

---

## Clarifications

### Session 2025-12-25

- Q: File storage infrastructure for attachments? → A: Text-only chatbot permanently (no file/image upload support - not planned)
- Q: Qdrant vector store integration with agent? → A: Agent Tool pattern - Define `@function_tool` for knowledge base search, agent decides when to search, ChatKit automatically displays action events in UI
- Q: Citation format in AI responses? → A: Inline Markdown footnotes - Use `[^1]` syntax with reference list at bottom (e.g., "IK solves joint angles[^1]... [^1]: Chapter 5, Section 5.3")

---

## User Scenarios & Testing

### Primary User Flow: Student Asks Question

1. **Preconditions**:
   - Student is logged into the textbook (Better Auth session active)
   - Student navigates to chatbot page in Docusaurus

2. **Flow**:
   - Student types question: "Explain inverse kinematics in robotics"
   - Chatbot searches knowledge base (Qdrant vector store) for relevant textbook sections
   - Chatbot generates response with citations to specific chapters/sections
   - Response streams to student in real-time
   - Conversation is saved and persists across sessions

3. **Success Criteria**:
   - Response appears within 3 seconds
   - Citations include chapter/section references
   - Previous conversations load when student returns

4. **Variations**:
   - Student asks follow-up questions requiring conversation context
   - Student requests code examples
   - Student asks for detailed mathematical explanations

### Secondary User Flow: Multiple Conversation Threads

1. **Preconditions**:
   - Student has existing conversations saved

2. **Flow**:
   - Student starts new conversation on different topic
   - Student can switch between conversation threads
   - Each thread maintains independent context

3. **Success Criteria**:
   - Students can manage at least 20 concurrent conversation threads
   - Thread switching completes instantly (< 200ms)

### Edge Cases

1. **No Relevant Knowledge Found**:
   - Chatbot clearly states when textbook doesn't cover topic
   - Suggests related topics that are covered

2. **Session Expiry**:
   - User session expires during conversation
   - System prompts re-authentication without losing conversation draft

3. **Concurrent Users**:
   - System handles 10,000 simultaneous conversations
   - Response quality doesn't degrade under load

---

## Functional Requirements

### FR1: Authentication Integration

**Requirement**: System validates user identity using existing Better Auth sessions before allowing chatbot access.

**Acceptance Criteria**:
- Backend receives session ID from frontend in Authorization header
- Backend queries Better Auth's session table in Neon PostgreSQL
- Invalid/expired sessions receive 401 Unauthorized response
- Valid sessions extract user_id for conversation ownership
- Session validation completes in < 100ms

### FR2: Conversation Thread Management

**Requirement**: Users can create, retrieve, and delete conversation threads.

**Acceptance Criteria**:
- Each thread has unique ID, title, creation timestamp, and last-updated timestamp
- Users can create new threads on-demand
- Users can list all their threads with pagination (20 per page)
- Users can retrieve full message history for any owned thread
- Users can delete threads they own
- Deleted threads cascade-delete all contained messages
- Users cannot access threads owned by others

### FR3: Message Streaming

**Requirement**: AI responses stream to users in real-time as they're generated.

**Acceptance Criteria**:
- Backend uses Server-Sent Events (SSE) for streaming
- Frontend receives text chunks as they're produced
- Stream includes action events when tools are called
- Stream gracefully handles mid-response disconnections
- Complete responses are saved to database after streaming finishes
- Users see typing indicators during generation

### FR4: Knowledge-Grounded Responses

**Requirement**: Chatbot answers reference specific sections of the robotics textbook.

**Acceptance Criteria**:
- Every factual statement includes citation using inline Markdown footnotes (e.g., `[^1]`)
- Reference list appears at bottom of response with chapter/section details
- System searches vector database before generating response (via agent tool)
- Responses explicitly state when textbook doesn't cover a topic
- Citations use format: `[^1]: Chapter X: Title, Section Y.Z`
- Retrieval completes within 500ms
- Top 5 most relevant sections are considered for each query

### FR5: Conversation Context Maintenance

**Requirement**: Chatbot maintains conversation history within each thread.

**Acceptance Criteria**:
- All messages in thread are available as context
- Last 10 messages are included in AI prompts
- Older messages are retrievable but not sent to AI by default
- Context switching between threads doesn't leak information
- Message order is preserved chronologically

### FR6: Data Persistence

**Requirement**: All conversations are stored permanently until explicitly deleted.

**Acceptance Criteria**:
- Threads persist across user sessions
- Messages include full text content, role (user/assistant), and timestamp
- Sequence numbers prevent message ordering conflicts
- Database survives backend restarts without data loss
- Messages remain accessible across backend restarts

### FR7: ChatKit Protocol Compliance

**Requirement**: Backend implements ChatKit server protocol for seamless frontend integration.

**Acceptance Criteria**:
- Single POST endpoint handles all ChatKit operations
- Responses use ChatKit's event format
- Widget rendering events are supported
- Action events for tool calls are emitted
- Thread metadata follows ChatKit schema
- Client-side tools can trigger server actions

---

## Success Criteria

**User-Facing Metrics**:
- 95% of queries receive relevant responses within 3 seconds
- Students rate answer quality 4+/5 stars in 80% of cases
- 70% of users return for multiple conversation sessions
- Average conversation thread length is 8+ messages

**System Performance**:
- System handles 10,000 concurrent users without degradation
- 99.5% uptime over 30-day periods
- Knowledge retrieval completes in < 500ms for 95% of queries
- Authentication validation completes in < 100ms

**Business Outcomes**:
- 40% reduction in support tickets about robotics concepts
- 60% of students use chatbot at least once per week
- Average session duration increases by 25%

---

## Assumptions

1. **Existing Infrastructure**: Better Auth is already deployed and managing user sessions in Neon PostgreSQL
2. **Frontend Integration**: Docusaurus frontend will embed ChatKit React components
3. **Knowledge Base**: Textbook content is already embedded and stored in Qdrant vector database
4. **OpenAI Access**: Google Gemini API access is available via OpenAI-compatible endpoint
5. **Deployment Environment**: Application will deploy to production environment with PostgreSQL access
6. **Scale**: Initial deployment expects < 1,000 concurrent users, scaling to 10,000
7. **Content Updates**: Textbook content updates will trigger vector database re-indexing
8. **Session Duration**: Better Auth sessions are valid for at least 24 hours

---

## Constraints

1. **Authentication**: Must use existing Better Auth sessions (cannot implement new auth system)
2. **Database**: Must use existing Neon PostgreSQL database
3. **AI Model**: Must use Google Gemini 2.5 Flash via compatible endpoint
4. **Framework**: Backend must use ChatKit Python SDK for ChatKit protocol compliance
5. **Latency**: Response generation must start within 3 seconds of user query
6. **Concurrency**: Must support at least 10,000 concurrent conversations

---

## Dependencies

**External Services**:
- Better Auth (authentication provider)
- Neon PostgreSQL (database)
- Qdrant Cloud (vector database)
- Google Gemini API (AI model)
- Cohere API (embeddings generation)

**Internal Dependencies**:
- Docusaurus frontend (hosts ChatKit React components)
- Textbook content (source material for knowledge base)

**Technical Dependencies**:
- openai-chatkit (Python SDK for ChatKit server)
- openai-agents (agent orchestration framework)
- FastAPI (web framework)
- SQLAlchemy (database ORM)
- qdrant-client (vector database client)

---

## Scope

### In Scope

- Backend API implementing ChatKit protocol
- Integration with Better Auth for session validation
- Conversation thread and message management
- Real-time streaming responses via SSE
- Knowledge base search using Qdrant vector store
- Response citation generation with chapter/section links
- Data persistence in PostgreSQL
- Text-based conversational interface

### Out of Scope

- Frontend UI development (handled by ChatKit React components)
- User authentication system (handled by Better Auth)
- Vector database indexing pipeline (assumed pre-existing)
- Textbook content management
- User management and permissions
- Analytics dashboard
- Mobile app development
- Multi-language support
- **File/image upload support** (text-only chatbot by design)
- **Multi-modal AI capabilities** (image analysis, diagram recognition)

### Future Considerations

- Code execution sandbox for running robotics simulations
- Integration with ROS 2 simulator
- Collaborative study sessions (multi-user threads)
- Voice input/output support
- Advanced analytics and learning insights
- Integration with assignment grading system

---

## Key Entities

### Thread

**Attributes**:
- thread_id (UUID, primary key)
- user_id (UUID, foreign key to Better Auth users table)
- title (string, optional)
- created_at (timestamp)
- updated_at (timestamp)

**Relationships**:
- Belongs to one User
- Has many Messages

### Message

**Attributes**:
- message_id (UUID, primary key)
- thread_id (UUID, foreign key)
- role (enum: 'user' | 'assistant')
- content (text)
- created_at (timestamp)
- sequence_number (integer, unique per thread)

**Relationships**:
- Belongs to one Thread

### User (Read-Only)

**Attributes**:
- id (UUID, primary key, managed by Better Auth)
- email (string)
- name (string, optional)
- software_level (enum: 'Beginner' | 'Intermediate' | 'Advanced')
- hardware_access (enum: 'Laptop/Cloud' | 'Physical Robot')

**Relationships**:
- Has many Threads

**Note**: User data is managed by Better Auth. Backend only reads for FK relationships.

---

## Non-Functional Requirements

### Performance

- **Response Time**: 95% of AI responses begin streaming within 3 seconds
- **Database Queries**: 99% complete in < 100ms
- **Vector Search**: 95% complete in < 500ms
- **Throughput**: Support 10,000 concurrent conversations
- **Message History Load**: Retrieve 50-message thread in < 200ms

### Reliability

- **Uptime**: 99.5% over 30-day rolling window
- **Data Durability**: Zero message loss after successful API response
- **Error Recovery**: Automatic retry for transient failures (max 3 attempts)
- **Graceful Degradation**: Return cached/fallback responses if AI unavailable

### Scalability

- **Horizontal Scaling**: Stateless backend supports multiple instances
- **Database Connection Pooling**: Efficient connection reuse
- **Rate Limiting**: Per-user limits prevent abuse (60 requests/minute)
- **Resource Limits**: Maximum 30-second AI response timeout

### Security

- **Authentication**: Every request requires valid Better Auth session
- **Authorization**: Users can only access their own threads
- **Data Encryption**: All data encrypted in transit (TLS 1.3)
- **Input Validation**: Sanitize all text inputs to prevent injection attacks
- **Rate Limiting**: Prevent abuse through per-user request throttling

### Maintainability

- **Logging**: Structured logs for all requests, errors, and AI interactions
- **Monitoring**: Health check endpoint for uptime monitoring
- **Error Tracking**: Detailed error messages with stack traces
- **Code Organization**: Clear separation of concerns (routes, services, models)

---

## Out of Scope

The following items are explicitly not part of this feature:

- Designing or implementing the ChatKit React frontend components
- Creating or updating the Better Auth authentication system
- Building the vector database indexing pipeline
- Developing content management system for textbook
- Implementing user registration or password reset flows
- Creating analytics or reporting dashboards
- Building mobile applications
- Supporting languages other than English
- Integrating with external learning management systems (LMS)
- Implementing payment or subscription systems
- Creating admin panel or moderation tools
- **File/image upload and storage** (text-only chatbot by design)
- **Multi-modal AI capabilities** (image analysis, PDF parsing)

---

## Questions and Open Items

*No open items at this time. All requirements are clearly defined based on ChatKit framework specifications and existing infrastructure.*

---

## Related Documents

- OpenAI ChatKit Python SDK Documentation: https://openai.github.io/chatkit-python/
- OpenAI ChatKit React SDK Documentation: https://github.com/openai/chatkit-js
- OpenAI Agents SDK Documentation: https://openai.github.io/openai-agents-python/
- Better Auth Documentation: https://www.better-auth.com/
- OpenAI ChatKit Advanced Samples: https://github.com/openai/openai-chatkit-advanced-samples

---

## Notes

**Implementation Strategy**:
- Use ChatKit's Store interface with PostgreSQL backend for conversation persistence
- Implement ChatKitServer subclass with custom respond() method for AI integration
- Leverage OpenAI Agents SDK for tool calling and streaming responses
- Use `@function_tool` decorator to create `search_textbook` tool for Qdrant vector search
- Agent autonomously decides when to invoke knowledge base search
- Use stream_agent_response() helper to convert Agent SDK events to ChatKit format
- ChatKit automatically emits action events for tool calls (displayed in UI as "🔍 Searching textbook...")
- Follow reference architecture from openai-chatkit-advanced-samples repository

**Key Design Decisions**:
- **Single Endpoint Architecture**: ChatKit protocol uses single `/chatkit` POST endpoint for all operations
- **JSON Column Storage**: Store thread metadata as JSON for schema flexibility
- **Session-Based Auth**: Better Auth sessions provide user identity without JWT complexity
- **Agent Context Pattern**: Pass user_id and request context through all operations
- **Streaming First**: All responses use SSE streaming for real-time user experience

**Known Limitations**:
- Gemini API accessed via OpenAI-compatible adapter may have different rate limits
- Text-only interface (no file/image upload support by design)
- Maximum context window limited by Gemini model (message history truncation may be needed)
- Real-time collaboration (multiple users in same thread) not supported

---

