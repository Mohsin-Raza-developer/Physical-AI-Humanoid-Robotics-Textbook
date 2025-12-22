# Feature Specification: RAG-Powered Chatbot Backend API

**Feature Branch**: `007-chatbot-backend`
**Created**: 2025-12-20
**Status**: Draft
**Input**: User description: "Create a RAG Backend API using Python, FastAPI, and OpenAI Agents SDK using Gemini 2.0 Flash as the LLM. Database: Qdrant Cloud (already populated with Cohere embed-v4.0 vectors). Goal: Backend API for chatbot. Tech Stack: LLM - gemini-2.0-flash via OpenAI SDK Adapter, Embeddings - Cohere embed-v4.0, Database - Qdrant Cloud, Framework - FastAPI with POST /api/chat endpoint, Agent SDK - OpenAI Agents SDK (agents library)."

## Clarifications

### Session 2025-12-20

- Q: Should the chatbot include source citations (file path, module, week) with every answer? → A: Yes, always include source citations with every answer, formatted as clickable Docusaurus links (e.g., [Chapter Title](/docs/path/to/chapter)) so students can navigate directly to the source material
- Q: What is the desired conversation history limit (number of turns or token count)? → A: Session-based approach - keep full conversation history in memory for the duration of the active session, cleared when session closes. No strict message pair limit as long as it fits within the LLM's 8,000 token context window
- Q: Should the system personalize responses based on user profile data? → A: Yes, system must fetch first_name, last_name, and software_level from Neon DB users table and pass this data to the LLM in every request using a structured messages array format
- Q: Should the system support multiple concurrent conversation sessions per user? → A: No, single active session per user - opening a new session ends the previous one
- Q: What level of detail is expected in error messages returned to users? → A: Simple user-friendly messages with error codes - provide generic, actionable messages for users (e.g., "We're having trouble connecting. Please try again.") along with error codes for support troubleshooting, without exposing technical details or security-sensitive information
- Q: Should the system implement any content filtering or safety checks beyond scope validation? → A: Yes, basic safety filtering - every user query must be validated for safety and relevance before processing. Queries that are safe and relevant proceed to the chatbot; harmful or irrelevant queries are blocked with appropriate user feedback. Implementation preference: use input guardrails pattern with dedicated validation logic
- Q: How should the agentic workflow be structured? → A: Use agent-based workflow with input guardrails (safety/relevance check), main chatbot agent (Gemini 2.0), and tool-calling pattern where the agent calls a search_knowledge_base function tool (not direct database access). Tool internally embeds query (Cohere 1536-dim), searches Qdrant, and returns formatted context to the agent
- Q: When should user personalization data be fetched and injected? → A: Fetch user profile (first_name, last_name, software_level) from Neon DB only once at the start of a new conversation session. Format this data into a system message and store it as the first item in the session's chat history to minimize database load and API latency

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Question and Receive Accurate Answer (Priority: P1)

A student studying the robotics textbook needs to ask a specific question about course content (e.g., "What is ROS 2 architecture?") and receive an accurate, contextual answer drawn from the book's content within seconds.

**Why this priority**: This is the core value proposition of the chatbot. Without accurate question-answering based on the textbook content, the chatbot provides no value to students. This directly impacts learning outcomes and user satisfaction.

**Independent Test**: Can be fully tested by sending a question via the chat API and verifying that the response contains relevant information from the textbook with clickable source citations formatted as Docusaurus links. Delivers immediate value by enabling self-service learning assistance.

**Acceptance Scenarios**:

1. **Given** the chatbot API is running and Qdrant contains indexed textbook content, **When** a student sends the question "What is ROS 2 architecture?", **Then** the system returns a response that accurately describes ROS 2 architecture based on textbook content within 5 seconds and includes clickable citations to source chapters
2. **Given** a student asks "How do I create a URDF file for a humanoid robot?", **When** the system processes the query, **Then** the response includes step-by-step instructions from the relevant lesson with specific code examples or references
3. **Given** a question about module-specific content (e.g., "Explain Isaac Sim setup"), **When** the system searches for relevant content, **Then** results are drawn from the correct module and lesson sections
4. **Given** a vague or ambiguous question (e.g., "tell me about robots"), **When** the system processes it, **Then** the response provides a focused answer based on the most relevant textbook sections and suggests more specific follow-up questions

---

### User Story 2 - Maintain Conversation Context (Priority: P2)

A student needs to ask follow-up questions in a conversation without repeating context each time (e.g., first ask "What is ROS 2?", then follow up with "How do I install it?"), and the chatbot should understand the conversation history.

**Why this priority**: Natural conversations require context retention. While the basic Q&A (P1) can work without context, multi-turn conversations significantly improve user experience and learning efficiency. This is essential for production but not blocking for MVP.

**Independent Test**: Can be tested by sending a sequence of related questions in a single conversation session and verifying that later responses reference earlier context. Delivers value by enabling more natural, efficient learning interactions.

**Acceptance Scenarios**:

1. **Given** a student asks "What is ROS 2?" and receives an answer, **When** they follow up with "How do I install it?", **Then** the system understands "it" refers to ROS 2 and provides installation instructions
2. **Given** an ongoing conversation about a specific topic, **When** the student asks a pronoun-based question (e.g., "Can you explain that in simpler terms?"), **Then** the system correctly interprets "that" based on conversation history
3. **Given** a conversation approaching the 8,000 token limit, **When** the user asks a new question, **Then** the system maintains relevant context while staying within the token limit for the active session

---

### User Story 3 - Handle Out-of-Scope Questions Gracefully (Priority: P2)

A student asks a question that is not covered in the textbook content (e.g., "What's the weather today?" or "How do I get a job at Boston Dynamics?"), and the chatbot should politely indicate it can only answer textbook-related questions without hallucinating.

**Why this priority**: Preventing hallucinations and scope creep is critical for trust and accuracy, but it's a safety feature rather than core functionality. Can be implemented after basic Q&A works.

**Independent Test**: Can be tested by sending off-topic questions and verifying the system responds appropriately without fabricating answers. Delivers value by maintaining trust and credibility.

**Acceptance Scenarios**:

1. **Given** a student asks "What's the weather today?", **When** the system processes the query, **Then** it responds with a message like "I can only answer questions about the Physical AI and Humanoid Robotics textbook. Please ask about course content."
2. **Given** a question about topics not in the textbook (e.g., "How do I apply to MIT?"), **When** the system searches the knowledge base and finds no relevant content, **Then** it informs the user that the topic is not covered in the course materials
3. **Given** a partially relevant question (e.g., "What programming languages are used?" when textbook only covers Python and C++), **When** the system responds, **Then** it answers based on textbook content and clarifies the scope of coverage
4. **Given** a student submits harmful or inappropriate content, **When** the safety validation runs, **Then** the query is blocked before reaching the chatbot and the user receives feedback explaining that the content violates community guidelines

---

### User Story 4 - Track User Interactions for Analytics (Priority: P3)

Course administrators need to see which questions students are asking most frequently, where students get stuck, and which topics generate the most queries to improve course content and identify knowledge gaps.

**Why this priority**: Analytics provide valuable insights for course improvement but are not essential for the chatbot to function. This is a future enhancement that adds strategic value after core functionality is proven.

**Independent Test**: Can be tested by sending various questions and verifying that query logs capture user_id, question text, response quality, and timestamp. Delivers value by enabling data-driven course improvements.

**Acceptance Scenarios**:

1. **Given** students are using the chatbot, **When** administrators query the analytics system, **Then** they can see the top 10 most frequently asked questions
2. **Given** a conversation session occurs, **When** the interaction completes, **Then** the system logs user_id, question text, response content, timestamp, and relevance scores
3. **Given** analytics data is collected, **When** administrators analyze it, **Then** they can identify topics with high query volume but low answer quality (indicating content gaps)

---

### Edge Cases

- What happens when a user sends an empty message or only whitespace?
- How does the system handle extremely long questions (>1000 words)?
- What if Qdrant vector database is temporarily unavailable?
- How does the system respond when no relevant content is found for a valid question?
- What happens if the embedding API (Cohere) fails or returns an error?
- How does the system handle concurrent requests from multiple users?
- What if a user sends the exact same question twice in a row?
- How does the system handle special characters, code snippets, or mathematical notation in questions?
- What happens when conversation history exceeds memory limits?
- How does the system handle rate limiting if too many requests are sent?
- What if user profile data (first_name, last_name, software_level) is missing or incomplete in the Neon database?
- How does the system handle Neon database connection failures when fetching user profile data?
- What happens if the safety validation system itself fails or times out?
- How does the system handle edge cases where content is borderline (potentially offensive but not clearly harmful)?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept conversational questions from students about textbook content via a chat interface
- **FR-002**: System MUST provide the agent with a knowledge base search capability through a dedicated function tool that searches the existing vector database of textbook embeddings to find relevant content matching the user's question
- **FR-003**: System MUST generate natural language responses that accurately answer questions based on retrieved textbook content
- **FR-004**: System MUST return responses within 5 seconds for 95% of queries under normal load
- **FR-005**: System MUST maintain full conversation context in memory for the duration of an active session, with automatic clearing when the session closes, respecting the 8,000 token context window limit
- **FR-006**: System MUST validate every user query for safety and relevance before processing, blocking harmful or inappropriate content with user-friendly feedback
- **FR-006a**: System MUST identify and reject questions that are outside the scope of the textbook content without fabricating answers
- **FR-007**: System MUST handle concurrent requests from multiple users without degradation
- **FR-008**: System MUST log all user interactions including user_id, question text, response content, and timestamp for analytics
- **FR-009**: System MUST gracefully handle errors from external dependencies (vector database, embedding API, language model) with simple user-friendly messages and error codes (e.g., "We're having trouble connecting. Please try again." + error code ERR_DB_001) without exposing technical details or security-sensitive information
- **FR-010**: System MUST validate incoming requests to ensure required fields (message, user_id) are present
- **FR-011**: System MUST return structured responses including the answer text with source citations formatted as clickable Docusaurus links (e.g., [Chapter Title](/docs/path/to/chapter)) to enable students to navigate directly to source material
- **FR-012**: System MUST convert user questions into vector embeddings using the same embedding model as the indexed content (dimension 1536)
- **FR-013**: System MUST retrieve and synthesize information from multiple relevant textbook sections when a question requires comprehensive context
- **FR-014**: System MUST authenticate API requests to ensure only authorized users can access the chatbot
- **FR-015**: System MUST handle edge cases such as empty messages, excessively long input, and malformed requests with user-friendly error messages and error codes that guide users toward resolution without exposing system internals
- **FR-016**: System MUST fetch user profile data (first_name, last_name, software_level) from the Neon database users table only once at the start of a new conversation session to minimize database load
- **FR-017**: System MUST format user profile data (first_name, last_name, software_level) into a system message and store it as the first item in the session's chat history to enable personalized responses throughout the conversation without repeated database queries
- **FR-017a**: System MUST pass conversation context using a structured messages array format where each message contains role (system/user/assistant) and content fields, with the system message containing user profile context as the first array item
- **FR-018**: System MUST structure conversation history as an array of message objects, each containing role (system/user/assistant) and content fields, maintaining the profile system message throughout the session
- **FR-019**: System MUST enforce a single active session per user, automatically terminating any previous session when a new session is initiated
- **FR-020**: System MUST implement an agent-based workflow where user queries first pass through a safety/relevance validation layer before reaching the main conversational agent
- **FR-021**: System MUST provide the main conversational agent with knowledge retrieval capability through a function tool interface (not direct database access), where the tool handles query embedding, vector search, and context formatting internally

### Key Entities

- **ChatMessage**: Represents a single message in a conversation formatted as `{"role": "system" | "user" | "assistant", "content": "message text"}` within the messages array, where system messages include user profile context at session start
- **ConversationSession**: Represents an active dialogue session with attributes: session identifier, user identifier, messages array (list of ChatMessage objects with user profile system message as first item), creation timestamp, last activity timestamp, token count
- **UserProfile**: Represents student profile data with attributes: first_name, last_name, software_level (fetched from Neon DB users table)
- **KnowledgeChunk**: Represents a retrieved piece of textbook content with attributes: content text, source file path, relevance score, module/week metadata, chapter title for citation formatting
- **ChatResponse**: Represents the system's reply with attributes: answer text, source citations as Docusaurus-formatted clickable links with chapter titles and paths, confidence score, processing time, error status with user-friendly message and error code (if applicable)
- **FunctionTool**: Represents the knowledge base search capability provided to the agent with attributes: tool name (search_knowledge_base), input parameters (query string), internal operations (query embedding via Cohere 1536-dim, Qdrant vector search, context formatting), output (formatted context string returned to agent)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can ask questions and receive relevant answers in under 5 seconds for 95% of queries
- **SC-002**: Answer accuracy rate is at least 85% when evaluated against a test set of 50 textbook-related questions
- **SC-003**: System handles at least 100 concurrent users without response time degradation beyond 10%
- **SC-004**: Out-of-scope question detection achieves 90% accuracy (correctly identifies when questions are not textbook-related)
- **SC-005**: Conversation context is maintained correctly within the 8,000 token limit throughout an active session, with accurate context preservation across multiple turns
- **SC-006**: System uptime is at least 99% during normal operation
- **SC-007**: Students report satisfaction score of 4/5 or higher when surveyed about chatbot usefulness
- **SC-008**: Zero instances of fabricated information (hallucinations) when answering questions about textbook content in testing
- **SC-009**: Safety validation correctly blocks at least 95% of harmful or inappropriate content in test scenarios while allowing legitimate educational queries to proceed
- **SC-010**: Agent-based workflow correctly routes queries through validation layer first, then to main agent, with function tool successfully retrieving relevant knowledge base content in at least 90% of test queries
- **SC-011**: User profile data is fetched from Neon DB exactly once per conversation session (at session start), with zero additional database queries for profile data during the session lifetime

## Assumptions *(if applicable)*

- The Qdrant vector database is already populated with embeddings of all textbook content using Cohere embed-v4.0 model (1536 dimensions)
- The existing embeddings were generated from the same content that students will ask questions about
- Students have unique user identifiers (user_id) available from the authentication system
- The existing authentication backend can provide user validation for API requests
- The Neon database users table contains complete and accurate profile data (first_name, last_name, software_level) for all authenticated users
- Network connectivity to external services (Qdrant Cloud, Cohere API, Gemini API, Neon DB) is reliable with standard retry mechanisms
- Students primarily ask questions in English (the language of the textbook content)
- The chatbot will be accessed via HTTP API initially (web UI integration is out of scope for this feature)
- Conversation sessions are ephemeral and do not need long-term persistence beyond analytics logging
- The software_level field in the users table contains standardized values (e.g., "beginner", "intermediate", "advanced") that the LLM can use to tailor response complexity
- Each user maintains only one active conversation session at a time; initiating a new session automatically terminates any previous session
- User profile data (first_name, last_name, software_level) is fetched once per session and remains constant throughout the session; profile changes require starting a new session to take effect

## Dependencies *(if applicable)*

- **006-embeddings-qdrant**: Requires the Qdrant vector database to be populated with textbook embeddings
- **005-docusaurus-auth**: Requires the authentication system to provide user_id for request validation and analytics
- **Neon Database**: Requires access to the users table containing first_name, last_name, and software_level fields for user personalization
- **External APIs**:
  - Gemini 2.0 Flash API (for natural language generation)
  - Cohere embed-v4.0 API (for query embedding)
  - Qdrant Cloud (for vector search)

## Constraints *(if applicable)*

- Must use the existing Qdrant collection (`robotics_textbook_v1`) without modification
- Must use Cohere embed-v4.0 for query embeddings to ensure compatibility with indexed content
- Responses must be based solely on textbook content; no external web search or general knowledge should be introduced
- Must maintain user privacy: no sharing of conversation logs with third parties
- Must comply with API rate limits of external services (Cohere, Gemini)
- System must be stateless at the API level to enable horizontal scaling
- Must be deployable to standard cloud platforms (Vercel, AWS, GCP, Azure)
- Input validation and safety filtering should be implemented using a guardrails pattern with dedicated validation logic before query processing
- Agent-based architecture should use a two-layer approach: input guardrail agent for safety/relevance validation, followed by main conversational agent with function tool access
- The main conversational agent should not have direct database access; knowledge retrieval must be mediated through a function tool interface (search_knowledge_base) that handles embedding generation, vector search, and context formatting internally
- User profile data (first_name, last_name, software_level) should be fetched once at session start, formatted into a system message, and stored as the first item in the chat history to minimize database load and enable personalized responses throughout the session

## Out of Scope *(if applicable)*

- Frontend chat UI (this feature focuses on backend API only)
- User authentication implementation (handled by existing auth-backend)
- Embedding generation or vector database population (already completed in 006-embeddings-qdrant)
- Multi-language support (Urdu, etc.)
- Voice input/output capabilities
- Integration with external learning management systems (LMS)
- Advanced features like tutoring mode, quiz generation, or personalized learning paths
- Real-time collaboration or group chat features
- Payment or subscription management
- Mobile native app support (API is platform-agnostic)

## Risks & Mitigations *(if applicable)*

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| External API failures (Gemini, Cohere) cause service unavailability | High | Medium | Implement retry logic with exponential backoff, circuit breakers, and graceful degradation with informative error messages |
| Language model generates hallucinated or incorrect information | High | Medium | Implement strict retrieval-augmented generation (RAG) pattern where responses must be grounded in retrieved content; add confidence scoring |
| Qdrant vector search returns irrelevant results | Medium | Low | Fine-tune similarity thresholds, implement query refinement, and fall back to "no relevant answer found" rather than guessing |
| API rate limits exceeded during peak usage | Medium | Medium | Implement request queuing, rate limiting at API gateway, and consider upgraded API tiers for production |
| Conversation context grows too large and exceeds token limits | Medium | Medium | Implement context window management with summarization or truncation of older messages |
| Security vulnerability in API endpoint (injection attacks, DoS) | High | Low | Implement input validation, sanitization, rate limiting, and standard API security best practices |
| Poor answer quality reduces student trust and adoption | High | Low | Establish quality benchmarks with test question set, implement feedback mechanism, and iterate on prompt engineering |

## Open Questions *(if applicable)*

- None remaining - all critical ambiguities have been resolved through clarification session

## References *(if applicable)*

- OpenAI Agents SDK Documentation: https://openai.github.io/openai-agents-python/
- Cohere Embeddings API Reference: https://docs.cohere.com/reference/embed
- FastAPI Framework Documentation: https://fastapi.tiangolo.com/
- Qdrant Vector Search Concepts: https://qdrant.tech/documentation/concepts/search/
- Retrieval-Augmented Generation (RAG) Pattern: https://arxiv.org/abs/2005.11401
- Gemini API Documentation: https://ai.google.dev/docs
- OpenAi Agents SDK Guardrails https://openai.github.io/openai-agents-python/guardrails/#input-guardrails
