# Implementation Plan: RAG-Powered Chatbot Backend API

**Branch**: `007-chatbot-backend` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)

---

## Summary

Build a production-ready RAG (Retrieval-Augmented Generation) chatbot backend API using FastAPI, OpenAI Agents SDK with Gemini 2.0 Flash, Cohere embeddings, Qdrant vector database, and Neon PostgreSQL. The system implements an agent-based architecture with input guardrails for safety validation, a main conversational agent with tool-calling capabilities, and optimized session management with once-per-session profile caching.

**Key Features**:
- Agent-based workflow (Guardrail → Main Agent → Knowledge Tool)
- RAG pattern for accurate, cited answers from textbook content
- Session-based personalization (90% DB load reduction)
- Docusaurus-formatted clickable citations
- Comprehensive error handling with user-friendly messages

---

## Technical Context

**Language/Version**: Python 3.12+
**Primary Dependencies**: FastAPI 0.115+, OpenAI Agents SDK, Cohere SDK, Qdrant Client, psycopg2
**Storage**: Qdrant Cloud (vector DB), Neon Serverless Postgres (user profiles)
**Testing**: pytest with async support, pytest-cov for coverage
**Target Platform**: Linux/macOS server (Railway/Render deployment)
**Project Type**: Web API (single service)
**Performance Goals**: <5s response time (95th percentile), 100+ concurrent users
**Constraints**: 8,000 token context window, free tier API limits (Gemini, Cohere, Qdrant)
**Scale/Scope**: MVP chatbot for ~1,000 students, ~10k textbook embeddings

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Educational Excellence
- **Status**: PASS
- **Evidence**: RAG system provides accurate answers with source citations, enabling students to verify information and learn from textbook content directly
- **Implementation**: Knowledge base search tool retrieves relevant textbook sections, agent synthesizes answers with markdown citations

### ✅ Hands-On Learning
- **Status**: PASS
- **Evidence**: While this is backend infrastructure, it supports hands-on learning by providing instant answers to simulation/code questions
- **Implementation**: API enables frontend to integrate chatbot into interactive learning experience

### ✅ Industry Alignment
- **Status**: PASS
- **Evidence**: Uses production-level tools (FastAPI, Gemini 2.0 Flash, Qdrant, Neon) following industry best practices
- **Implementation**: OpenAPI spec, type-safe models, async patterns, comprehensive testing, structured logging

### ✅ Accessibility
- **Status**: PASS
- **Evidence**: Personalization based on software_level (beginner/intermediate/advanced) ensures responses match student experience
- **Implementation**: User profile fetched once per session, formatted into system message, agent tailors complexity accordingly

### ✅ Professional Quality
- **Status**: PASS
- **Evidence**: Production-ready code with type hints, error handling, validation, testing, and documentation
- **Implementation**: Pydantic models, comprehensive error codes, pytest suite, OpenAPI documentation

### Backend Requirements Compliance

- ✅ **FastAPI (Python 3.12+)**: Core framework with async/await patterns
- ✅ **Neon Serverless Postgres**: User profile storage (first_name, last_name, software_level)
- ✅ **Qdrant Cloud Free Tier**: Vector search for RAG (pre-populated robotics_textbook_v1 collection)
- ✅ **Better-Auth Integration**: JWT authentication for secure API access
- ✅ **OpenAI Agents/ChatKit SDK**: Agent-based architecture with Gemini 2.0 Flash

### RAG Chatbot Requirements

- ✅ **Question Answering**: Accurate responses from textbook content with <5s latency
- ✅ **Source Citations**: Docusaurus-formatted clickable links in all responses
- ✅ **Personalization**: Software level-based response tailoring
- ✅ **Error Handling**: User-friendly messages with error codes
- ✅ **Performance**: >90% accuracy, 99% uptime target

---

## Project Structure

### Documentation (this feature)

```text
specs/007-chatbot-backend/
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0: Architectural decisions & best practices
├── data-model.md        # Phase 1: Data structures & database schemas
├── quickstart.md        # Phase 1: Developer setup guide
├── contracts/           # Phase 1: API specifications
│   └── openapi.yaml     # OpenAPI 3.1 specification
└── tasks.md             # Phase 2: NOT created by /sp.plan (use /sp.tasks)
```

### Source Code (repository root)

```text
backend/                 # Backend service root
├── main.py              # FastAPI app entrypoint
├── agent.py             # Agent & guardrail definitions
├── models.py            # Pydantic data models
├── database.py          # Database connections (Neon, Qdrant)
├── tools.py             # Knowledge retrieval tool (@function_tool)
├── session.py           # Session management
├── config.py            # Configuration & environment variables
├── middleware.py        # CORS, rate limiting, logging
├── requirements.txt     # Python dependencies
├── .env.example         # Environment variable template
├── tests/               # Test suite
│   ├── test_agent.py    # Agent behavior tests
│   ├── test_tools.py    # Tool functionality tests
│   ├── test_api.py      # API endpoint tests
│   └── conftest.py      # Pytest fixtures
└── README.md            # Detailed documentation
```

---

## Phase 0: Research & Architectural Decisions

**Status**: ✅ Complete
**Output**: `research.md`

### Key Decisions Documented

1. **Web Framework**: FastAPI 0.115+ (async performance, type safety, auto-docs)
2. **Agent SDK**: OpenAI Agents SDK (guardrails, tool calling, Gemini support)
3. **LLM**: Gemini 2.0 Flash (fast, free tier, OpenAI-compatible)
4. **Embeddings**: Cohere embed-v4.0 (1536-dim, existing data compatibility)
5. **Vector DB**: Qdrant Cloud (pre-populated, sub-100ms search)
6. **User DB**: Neon Serverless Postgres (serverless, low latency)
7. **Architecture**: Agent-based (Guardrail → Main Agent → Tool)
8. **Profile Caching**: Once-per-session (90% DB load reduction)
9. **Citation Format**: Docusaurus markdown links
10. **Error Handling**: Structured error codes with user-friendly messages

### Performance Optimizations

- **Async/Await**: Throughout stack for I/O-bound operations
- **Connection Pooling**: Reuse DB connections
- **Profile Caching**: Fetch once per session (messages[0])
- **Streaming**: Future enhancement for real-time responses

**See `research.md` for complete architectural analysis.**

---

## Phase 1: Design & Contracts

**Status**: ✅ Complete
**Outputs**: `data-model.md`, `contracts/openapi.yaml`, `quickstart.md`

### Data Model

**Request/Response Models**:
- `ChatRequest`: User message input
- `ChatResponse`: Agent answer with citations
- `ErrorResponse`: Structured error format

**Domain Entities**:
- `ChatMessage`: Single message (system/user/assistant)
- `ConversationSession`: Active dialogue state
- `UserProfile`: Student profile from Neon DB
- `KnowledgeChunk`: Search result from Qdrant

**Database Schemas**:
- Neon `users` table (id, first_name, last_name, software_level)
- Qdrant `robotics_textbook_v1` collection (1536-dim vectors, metadata)

**Message Format**:
```python
messages = [
    {"role": "system", "content": "You are... Student: John Doe, Level: intermediate"},
    {"role": "user", "content": "What is ROS 2?"},
    {"role": "assistant", "content": "ROS 2 is... [Citation](/docs/...)"}
]
```

**See `data-model.md` for complete specifications.**

### API Contracts

**Endpoints**:
- `POST /v1/chat`: Send chat message
- `GET /v1/sessions/{session_id}`: Get session details
- `DELETE /v1/sessions/{session_id}`: End session
- `GET /v1/health`: Health check

**Authentication**: Bearer token (JWT from Better-Auth)

**Rate Limiting**: 20 requests/minute per user

**See `contracts/openapi.yaml` for complete OpenAPI 3.1 specification.**

### Quick Start Guide

Comprehensive developer guide with:
- Prerequisites (Python 3.12+, API keys)
- 5-minute setup instructions
- Environment variable configuration
- Verification tests
- Troubleshooting tips

**See `quickstart.md` for complete setup instructions.**

---

## Implementation Architecture

### Agent-Based Workflow (OpenAI Agents SDK)

**CRITICAL**: Uses OpenAI Agents SDK with proper Runner execution and input guardrails

```
User Query (ChatRequest)
    ↓
API Layer (FastAPI)
    ↓
Session Management
    ├─ New Session?
    │   ├─ Fetch Profile (Neon DB) → UserProfile
    │   ├─ Format System Message
    │   └─ Create ConversationSession (messages[0] = system message)
    └─ Existing Session?
        └─ Load from Memory
    ↓
Add User Message (messages[N])
    ↓
Configure Run Environment
    ├─ AsyncOpenAI client (Gemini base_url)
    ├─ OpenAIChatCompletionsModel (gemini-2.0-flash)
    └─ RunConfig (model, tracing settings)
    ↓
Runner.run(main_agent, input, run_config, session)
    ↓
    ├─── @input_guardrail decorator triggers ─────┐
    │                                              │
    │   Guardrail Agent                           │
    │   ├─ name: "Safety Guardrail"               │
    │   ├─ instructions: "Check safety/relevance" │
    │   ├─ output_type: SafetyCheckOutput         │
    │   │   ├─ is_safe: bool                      │
    │   │   ├─ is_relevant: bool                  │
    │   │   └─ reason: str                        │
    │   ↓                                          │
    │   Runner.run(guardrail_agent, input)        │
    │   ↓                                          │
    │   GuardrailFunctionOutput                   │
    │   ├─ output_info: SafetyCheckOutput         │
    │   └─ tripwire_triggered: bool               │
    │       ├─ True? → InputGuardrailTripwireTriggered exception
    │       │          → Catch in endpoint → Return ERR_VAL_004/005
    │       └─ False? ↓                           │
    └───────────────────────────────────────────────┘
    ↓
Main Chatbot Agent (Gemini 2.0 Flash)
    ├─ name: "Robotics Tutor"
    ├─ instructions: "You are a helpful robotics tutor..."
    ├─ tools: [search_knowledge_base]
    ├─ input_guardrails: [safety_guardrail]
    │
    ├─ Receives: Full conversation context (session.messages)
    ├─ messages[0]: System message with user profile
    ├─ messages[1..N]: Conversation history
    │
    └─ Decision: Needs textbook context?
        ├─ Yes → Calls search_knowledge_base tool
        │   ├─ @function_tool decorator
        │   ├─ Embed Query (Cohere embed-v4.0, 1536-dim)
        │   ├─ Search Qdrant (collection: robotics_textbook_v1)
        │   │   ├─ limit: 5
        │   │   └─ score_threshold: 0.7
        │   ├─ Format Results: "[Chapter Title](/docs/path)"
        │   └─ Return: Context string to agent
        └─ No → Generate response from conversation context
    ↓
Agent generates response with inline citations
    ↓
Runner returns RunResult
    ├─ final_output: str (agent's response)
    ├─ messages: list (full conversation)
    └─ tool_calls: list (search_knowledge_base calls)
    ↓
Extract Citations (regex parse markdown links)
    ↓
Add Assistant Message to session (messages[N+1])
    ↓
Build ChatResponse
    ├─ response: str (with citations)
    ├─ session_id: str
    ├─ citations: list[Citation]
    ├─ processing_time_ms: int
    └─ token_count: int
    ↓
Return to User
```

**Key Implementation Details**:

1. **Gemini Configuration**:
   ```python
   from agents import AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig

   external_client = AsyncOpenAI(
       api_key=settings.gemini_api_key,
       base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
   )

   model = OpenAIChatCompletionsModel(
       model="gemini-2.0-flash",
       openai_client=external_client
   )

   config = RunConfig(
       model=model,
       tracing_disabled=True
   )
   ```

2. **Guardrail Pattern**:
   ```python
   from agents import (
       Agent, GuardrailFunctionOutput, InputGuardrailTripwireTriggered,
       RunContextWrapper, Runner, input_guardrail
   )
   from pydantic import BaseModel

   class SafetyCheckOutput(BaseModel):
       is_safe: bool
       is_relevant: bool
       reason: str

   guardrail_agent = Agent(
       name="Safety Guardrail",
       instructions="Check if query is safe and relevant to robotics textbook",
       output_type=SafetyCheckOutput
   )

   @input_guardrail
   async def safety_guardrail(
       ctx: RunContextWrapper[None],
       agent: Agent,
       input: str | list[TResponseInputItem]
   ) -> GuardrailFunctionOutput:
       result = await Runner.run(guardrail_agent, input, context=ctx.context)

       return GuardrailFunctionOutput(
           output_info=result.final_output,
           tripwire_triggered=not (result.final_output.is_safe and result.final_output.is_relevant)
       )
   ```

3. **Agent Execution**:
   ```python
   main_agent = Agent(
       name="Robotics Tutor",
       instructions="You are a helpful robotics tutor...",
       tools=[search_knowledge_base],
       input_guardrails=[safety_guardrail]
   )

   try:
       result = await Runner.run(
           starting_agent=main_agent,
           input=user_message,
           run_config=config,
           session=conversation_session
       )
       assistant_response = result.final_output

   except InputGuardrailTripwireTriggered as e:
       # Query blocked by guardrail
       raise ChatbotError(message="Query blocked", code="ERR_VAL_004")
   ```

### Knowledge Retrieval Tool

```python
@function_tool
def search_knowledge_base(query: str) -> str:
    """
    Searches robotics textbook for relevant content.

    Args:
        query: Search query (question or topic)

    Returns:
        Formatted context with Docusaurus citations
    """
    # 1. Embed query
    embedding = cohere_client.embed(
        texts=[query],
        model="embed-english-v4.0",
        input_type="search_query"
    ).embeddings[0]

    # 2. Search Qdrant
    results = qdrant_client.search(
        collection_name="robotics_textbook_v1",
        query_vector=embedding,
        limit=5,
        score_threshold=0.7
    )

    # 3. Format with citations
    context_parts = []
    for result in results:
        content = result.payload["content"]
        chapter_title = result.payload["chapter_title"]
        source_file = result.payload["source_file"]

        # Convert file path to Docusaurus URL
        doc_url = source_file.replace("content/docs/", "/docs/").replace(".md", "")
        citation = f"[{chapter_title}]({doc_url})"

        context_parts.append(f"{content}\n\nSource: {citation}")

    return "\n\n---\n\n".join(context_parts)
```

### Session Management

**New Session Flow**:
1. Fetch user profile from Neon DB (1 query)
2. Format profile into system message
3. Create ConversationSession with messages[0] = system message
4. Store in memory (or Redis for production)

**Existing Session Flow**:
1. Retrieve session from memory by session_id
2. Validate user_id matches
3. Use existing messages array (includes profile in messages[0])

**Performance**: Profile fetched once (session start) vs N times (per turn) = 90% reduction

---

### Application Lifecycle Management

**Purpose**: Manage database connection pools throughout the application lifecycle

**Startup Flow**:
```python
from contextlib import asynccontextmanager
from fastapi import FastAPI
from psycopg2.pool import SimpleConnectionPool
from qdrant_client import QdrantClient
import cohere
import os

# Global clients
db_pool = None
qdrant_client = None
cohere_client = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application lifecycle: startup and shutdown"""

    # ========== STARTUP ==========
    global db_pool, qdrant_client, cohere_client

    print("🚀 Initializing database connections...")

    # Neon DB Connection Pool
    db_pool = SimpleConnectionPool(
        minconn=2,      # 2 connections always ready
        maxconn=10,     # Maximum 10 concurrent connections
        dsn=os.getenv("DATABASE_URL")
    )
    print("✅ Neon DB pool created (2-10 connections)")

    # Qdrant Vector DB Client
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
        timeout=10.0
    )
    print("✅ Qdrant client initialized")

    # Cohere Embeddings Client
    cohere_client = cohere.Client(
        api_key=os.getenv("COHERE_API_KEY")
    )
    print("✅ Cohere client initialized")

    print("✅ All connections initialized successfully")

    yield  # Application runs here

    # ========== SHUTDOWN ==========
    print("🛑 Shutting down gracefully...")

    # Close Neon DB Pool (closes all connections)
    if db_pool:
        db_pool.closeall()
        print("✅ Neon DB pool closed")

    # Close Qdrant Client
    if qdrant_client:
        qdrant_client.close()
        print("✅ Qdrant client closed")

    # Cohere HTTP client closes automatically
    print("✅ Cohere client closed")

    print("✅ All connections closed gracefully")

# Create FastAPI app with lifespan
app = FastAPI(
    title="RAG Chatbot Backend API",
    version="1.0.0",
    lifespan=lifespan
)
```

**Per-Request Database Access**:
```python
async def fetch_user_profile(user_id: str) -> UserProfile:
    """Fetch user profile from Neon DB using connection pool"""
    conn = db_pool.getconn()  # Get connection from pool (<1ms)
    try:
        cursor = conn.cursor()
        cursor.execute(
            "SELECT user_id, first_name, last_name, software_level FROM users WHERE user_id = %s",
            (user_id,)
        )
        row = cursor.fetchone()
        if not row:
            raise ValueError(f"User {user_id} not found")

        return UserProfile(
            user_id=row[0],
            first_name=row[1],
            last_name=row[2],
            software_level=row[3]
        )
    finally:
        db_pool.putconn(conn)  # Return connection to pool (ALWAYS)
```

**Lifecycle Timeline**:
```
App Startup (once)
    ↓
    lifespan startup event
    ↓
    Initialize connection pools
    ├─ Neon: 2-10 connections in pool
    ├─ Qdrant: 1 persistent connection
    └─ Cohere: HTTP client
    ↓
    App ready to handle requests

Per Request (thousands of times)
    ↓
    conn = db_pool.getconn()      ← Get from pool (<1ms)
    ↓
    Execute query
    ↓
    db_pool.putconn(conn)          ← Return to pool
    ↓
    Connection reused by next request

App Shutdown (once)
    ↓
    CTRL+C or kill signal
    ↓
    lifespan shutdown event
    ↓
    db_pool.closeall()             ← Close all Neon connections
    qdrant_client.close()          ← Close Qdrant connection
    ↓
    Clean exit (no hanging connections)
```

**Benefits**:
- **Performance**: Connection reuse avoids 50-100ms handshake overhead per request
- **Scalability**: Limits concurrent connections to 10 (prevents DB overload)
- **Resource Safety**: Ensures all connections properly closed on shutdown
- **Error Handling**: `finally` block ensures connection return even on exceptions
- **Memory Leak Prevention**: Proper cleanup prevents resource leaks

---

## Technology Stack Details

### Core Dependencies

```
# Web Framework
fastapi>=0.115.0
uvicorn[standard]>=0.30.0
pydantic>=2.9.0

# OpenAI Agents SDK
openai-agents>=1.0.0
openai>=1.50.0

# LLM & Embeddings
cohere>=5.11.0

# Vector Database
qdrant-client>=1.12.0

# PostgreSQL
psycopg2-binary>=2.9.9

# Utilities
python-dotenv>=1.0.0
python-multipart>=0.0.12
pydantic-settings>=2.6.0

# Security & Auth
python-jose[cryptography]>=3.3.0
passlib[bcrypt]>=1.7.4

# Rate Limiting
slowapi>=0.1.9

# Testing
pytest>=8.3.0
pytest-asyncio>=0.24.0
pytest-cov>=6.0.0
httpx>=0.27.0

# Logging
structlog>=24.4.0
```

### External Services

1. **Gemini API**
   - Model: `gemini-2.0-flash-exp`
   - Base URL: `https://generativelanguage.googleapis.com/v1beta/openai/`
   - Free Tier: 60 requests/minute
   - Cost: $0 (free tier sufficient for MVP)

2. **Cohere API**
   - Model: `embed-english-v4.0`
   - Dimensions: 1536
   - Free Tier: 100 requests/minute
   - Cost: $0 (free tier sufficient for MVP)

3. **Qdrant Cloud**
   - Collection: `robotics_textbook_v1`
   - Vectors: ~10,000 embeddings
   - Free Tier: 1GB storage
   - Cost: $0 (free tier sufficient)

4. **Neon Serverless Postgres**
   - Table: `users`
   - Rows: ~1,000 students
   - Free Tier: 3GB storage, 100 hours compute/month
   - Cost: $0 (free tier sufficient)

---

## Error Handling Strategy

### Error Code System

| Code Range | Category | Examples |
|------------|----------|----------|
| ERR_DB_001-099 | Database | Neon connection failed, Qdrant timeout |
| ERR_AUTH_001-099 | Authentication | Invalid token, expired session |
| ERR_AGENT_001-099 | Agent/LLM | Gemini API error, rate limit |
| ERR_TOOL_001-099 | Knowledge Retrieval | Cohere embedding failed, no results |
| ERR_VAL_001-099 | Input Validation | Empty message, invalid user_id |

### User-Friendly Messages

```python
{
    "ERR_DB_001": "We're having trouble connecting to our knowledge base. Please try again.",
    "ERR_AUTH_001": "Authentication required. Please log in to continue.",
    "ERR_AGENT_003": "The chatbot is temporarily unavailable. Please try again in a moment.",
    "ERR_TOOL_001": "Could not retrieve textbook content. Please try rephrasing your question.",
    "ERR_VAL_001": "Your message appears to be empty. Please ask a question."
}
```

---

## Testing Strategy

### Unit Tests

- **Agent Behavior**: Test guardrail blocking, tool calling, response generation
- **Tools**: Test embedding, search, citation formatting
- **Models**: Test Pydantic validation, serialization
- **Database**: Test connection, queries, error handling

**Coverage Target**: >80%

### Integration Tests

- **End-to-End**: Full request → response flow
- **Session Management**: Profile fetching, caching, continuity
- **Error Scenarios**: DB failures, API errors, rate limiting
- **Performance**: Response time, concurrent requests

### Test Data

- **Mock User Profiles**: beginner/intermediate/advanced levels
- **Mock Qdrant Results**: Predefined search results
- **Test Questions**: Set of 50 textbook-related queries

---

## Deployment Plan

### Environment Configuration

**Development**:
- Local FastAPI server (`uvicorn --reload`)
- Qdrant Cloud free tier (dev collection)
- Neon dev database
- Development API keys

**Staging**:
- Railway/Render deployment
- Qdrant Cloud (staging collection)
- Neon staging database
- Production API keys (with lower rate limits)

**Production**:
- Railway/Render (autoscaling enabled)
- Qdrant Cloud production
- Neon production database
- Production API keys (full rate limits)

### Monitoring

- **Health Checks**: `/v1/health` endpoint (every 30s)
- **Metrics**: Response time, error rate, token usage
- **Logging**: Structured logs (JSON format)
- **Alerts**: Slack/email for errors, downtime

---

## Security Considerations

### Input Validation

- **Pydantic Models**: Schema validation on all requests
- **Guardrail Agent**: Content safety/relevance check
- **Sanitization**: Strip whitespace, validate lengths
- **Injection Prevention**: Parameterized SQL queries

### Authentication & Authorization

- **JWT Tokens**: Bearer auth from Better-Auth
- **Session Validation**: user_id matches session owner
- **Rate Limiting**: Per-user and global limits
- **CORS**: Whitelist frontend origins only

### Data Privacy

- **No Conversation Persistence**: Sessions ephemeral (in-memory)
- **Analytics Logging**: Anonymize user_id for analytics
- **No Third-Party Sharing**: All data stays within system
- **GDPR Compliance**: User can request data deletion

---

## Performance Targets

| Metric | Target | Measurement |
|--------|--------|-------------|
| Response Time (p95) | <5 seconds | API endpoint monitoring |
| Response Time (p50) | <2 seconds | API endpoint monitoring |
| Concurrent Users | 100+ | Load testing |
| Availability | 99%+ | Uptime monitoring |
| Answer Accuracy | >85% | Manual evaluation (50-question test set) |
| Out-of-Scope Detection | >90% | Manual evaluation |
| Profile Fetch Reduction | ~90% | Database query logging |

---

## Future Enhancements (Post-MVP)

### v1.1 (Short Term)
- **Streaming Responses**: Real-time answer generation
- **Enhanced Citations**: Preview snippets on hover
- **Analytics Dashboard**: Most asked questions, response quality
- **Query Caching**: Cache common questions for faster responses

### v2.0 (Long Term)
- **Multi-Modal**: Image upload support (diagrams, code screenshots)
- **Voice Input**: Whisper integration for voice queries
- **Conversation Export**: Download chat history (PDF/markdown)
- **Advanced Personalization**: ML-based difficulty adjustment
- **Multi-Language**: Urdu, Spanish, Chinese support

---

## Next Steps

### For Implementation

1. **Run `/sp.tasks`** - Generate detailed task breakdown
2. **Set up development environment** - Follow `quickstart.md`
3. **Implement core components** - Agent, tools, API endpoints
4. **Write tests** - Unit and integration test coverage
5. **Deploy to staging** - Test in staging environment
6. **Performance testing** - Load test, optimize bottlenecks
7. **Deploy to production** - Production release

### For Review

1. **Code Review** - Ensure code quality, best practices
2. **Security Audit** - Validate input sanitization, auth
3. **Performance Review** - Verify response times, concurrency
4. **Documentation Review** - Complete README, API docs
5. **User Acceptance Testing** - Test with real students

---

## Appendix

### Related Documents

- [Feature Specification](./spec.md) - Business requirements
- [Research](./research.md) - Architectural decisions
- [Data Model](./data-model.md) - Data structures
- [API Contract](./contracts/openapi.yaml) - OpenAPI spec
- [Quick Start](./quickstart.md) - Developer setup

### References

- [FastAPI Docs](https://fastapi.tiangolo.com/)
- [OpenAI Agents SDK](https://openai.github.io/openai-agents-python/)
- [Cohere Embeddings](https://docs.cohere.com/reference/embed)
- [Qdrant Docs](https://qdrant.tech/documentation/)
- [Neon Docs](https://neon.tech/docs)

---

**Status**: ✅ Planning Complete - Ready for `/sp.tasks`

**Next Command**: `/sp.tasks` to generate implementation task breakdown
