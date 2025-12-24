# Research & Technology Decisions: ChatKit Gemini Backend

**Feature**: 008-chatkit-gemini-backend
**Date**: 2025-12-23
**Purpose**: Document all technology choices, integration patterns, and architectural decisions for the self-hosted ChatKit backend with Google Gemini

---

## Executive Summary

This backend integrates OpenAI ChatKit Python SDK with Google Gemini (via OpenAI Agents SDK) to create a production-ready, self-hosted chat application. Key decisions prioritize leveraging existing infrastructure (Neon PostgreSQL, existing auth system), using SDK-provided patterns (ChatKit for API structure, Agents SDK for AI logic), and maintaining compatibility with ChatKit React frontend components.

---

## 1. Primary Language & Runtime

### Decision: Python 3.12+

**Rationale**:
- **Constitution Alignment**: Backend requirements specify "FastAPI (Python 3.12+) with async/await patterns"
- **SDK Compatibility**: Both ChatKit Python SDK and OpenAI Agents SDK are Python-native
- **Async Support**: Python 3.12's improved async/await performance critical for streaming responses
- **Team Familiarity**: Existing codebase uses Python (embeddings generation, auth backend uses Next.js but API logic could integrate)

**Alternatives Considered**:
-  TypeScript/Node.js: Rejected - ChatKit Python SDK is the official backend SDK; TypeScript would require custom implementation
- Python 3.11: Rejected - 3.12 offers performance improvements for async operations

**Implementation Notes**:
- Use `pyenv` or `asdf` for Python version management
- Specify `python = "^3.12"` in pyproject.toml
- Leverage `async`/`await` throughout for I/O operations

---

## 2. Web Framework

### Decision: FastAPI 0.115+

**Rationale**:
- **Constitution Mandate**: Explicitly required in backend requirements
- **Async-First**: Native async support essential for SSE streaming and concurrent requests
- **ChatKit Integration**: ChatKit Python SDK designed to work seamlessly with FastAPI/Starlette
- **Performance**: Handles 100+ concurrent users requirement (FR-003 success criteria)
- **Developer Experience**: Auto-generated OpenAPI docs, type safety with Pydantic

**Alternatives Considered**:
- Flask: Rejected - lacks native async support, would require extensions
- Django: Rejected - too heavyweight for API-only service, slower async adoption

**Implementation Notes**:
```python
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import uvicorn

app = FastAPI()

# ChatKit will provide route handlers
# OpenAI Agents SDK handles AI logic within routes
```

**Key Dependencies**:
- `fastapi[standard]>=0.115.0` (includes uvicorn, pydantic v2)
- `uvicorn[standard]` for ASGI server with WebSocket/SSE support
-  `python-multipart` for file upload handling

---

## 3. AI/LLM Integration

### Decision: Google Gemini 2.0 Flash via OpenAI Agents SDK

**Rationale**:
- **Specification Requirement**: "Must use Google Gemini as the primary LLM"
- **Agents SDK Compatibility**: OpenAI Agents SDK supports custom LLM backends via `AsyncOpenAI` client
- **Proven Pattern**: User provided working example code demonstrating Gemini + Agents SDK integration
- **Cost Efficiency**: Gemini 2.0 Flash offers competitive pricing vs GPT-4
- **Streaming Support**: Gemini API supports streaming responses required by FR-011

**Integration Pattern** (from user-provided example):
```python
from agents import Agent, AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig, Runner,function_tool
from dotenv import load_dotenv
import os

# Configure Gemini via OpenAI-compatible endpoint
gemini_client = AsyncOpenAI(
    api_key=os.environ["GEMINI_API_KEY"],
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=gemini_client
)

config = RunConfig(model=model, tracing_disabled=True)

@function_tool
async def tool_name(args : str):
    return "tool"

chat_agent = Agent(
    name="Chatbot",
    instructions="You are a helpful robotics tutor for the Physical AI and Humanoid Robotics course.",
    tools = [tool_name]
)


# Usage in endpoint
result = await Runner.run(
    starting_agent=chat_agent,
    input=user_message,
    run_config=config
)
```

**Alternatives Considered**:
- OpenAI GPT-4: Rejected per spec constraint - must use Gemini
- Anthropic Claude: Rejected - spec mandates Gemini; Claude integration more complex
- Local LLaMA/Mistral: Rejected - lacks streaming quality, requires GPU infrastructure

**Implementation Notes**:
- Store `GEMINI_API_KEY` in environment variables
- Implement retry logic with exponential backoff for API failures (FR-047)
- Monitor rate limits (Gemini free tier: 15 requests/minute, 1500/day)
- Consider upgrading to paid tier for production (1000 requests/minute)

**API Documentation**:
- Gemini API: https://ai.google.dev/docs
- OpenAI Agents SDK: https://openai.github.io/openai-agents-python/quickstart/

---

## 4. ChatKit SDK Integration

### Decision: ChatKit Python SDK for Backend API Structure

**Rationale**:
- **Specification Requirement**: "Build a production-ready FastAPI backend for OpenAI ChatKit"
- **Built-In Features**: Provides streaming, widgets, progress events, client effects out-of-the-box
- **Frontend Compatibility**: Ensures API contract matches ChatKit React SDK expectations
- **Reduced Implementation**: Eliminates need to manually implement SSE streaming, widget schemas, event formats
- **Proven Patterns**: Official SDK provides best practices for chat applications

**Key ChatKit Features Used**:
- **Streaming Responses**: Built-in SSE support for FR-011
- **Widget System**: Pre-defined widget types for FR-019-022
- **Progress Events**: Event emitters for FR-023-026
- **Client Effects**: Effect payload structure for FR-027-030

**Integration Approach**:
```python
from chatkit import ChatKitApp, Thread, Message, StreamingResponse

app = ChatKitApp(fastapi_app=fastapi_app)

@app.post("/threads/{thread_id}/messages")
async def send_message(thread_id: str, message: MessageCreate):
    # OpenAI Agents SDK handles AI logic
    agent_result = await Runner.run(agent, input=message.content)

    # ChatKit SDK handles streaming format
    return await app.stream_response(agent_result)
```

**Alternatives Considered**:
- Custom Implementation: Rejected - reinventing ChatKit's streaming/widget system error-prone, more code
- LangChain LangServe: Rejected - doesn't provide ChatKit-compatible API structure

**Implementation Notes**:
- Install: `pip install chatkit-python`
- Reference: https://openai.github.io/chatkit-python/
- Advanced samples: https://github.com/openai/openai-chatkit-advanced-samples

---

## 5. Knowledge Base Tool

### Decision: search_knowledge_base Function Tool (Cohere + Qdrant)

**Rationale**:
- **Existing Infrastructure**: User already has embeddings generated with Cohere embed-v4.0 in Qdrant Cloud
- **Working Implementation**: User provided `tool_example.py` with proven integration pattern
- **Agents SDK Compatibility**: `@function_tool` decorator integrates seamlessly with OpenAI Agents SDK
- **Automatic Tool Calling**: Agent autonomously decides when to invoke knowledge search (no manual triggers)

**Implementation** (from user-provided code):
```python
from agents import function_tool
from qdrant_client import QdrantClient
import cohere
import asyncio

@function_tool
def search_knowledge_base(query: str) -> str:
    """Search the robotics textbook knowledge base for relevant content."""
    async def _run():
        # Embed query with Cohere
        embedding = cohere_client.embed(
            texts=[query],
            model="embed-v4.0",
            input_type="search_query"
        ).embeddings[0]

        # Vector search in Qdrant
        results = qdrant_client.query_points(
            collection_name="robotics_textbook_v1",
            query=embedding,
            limit=5,
            score_threshold=0.4
        ).points

        if not results:
            return "No relevant content found."

        return "\n".join(f"- {r.payload.get('text','')}" for r in results)

    return asyncio.run(_run())

# Agent automatically calls this tool when needed
agent = Agent(
    name="Chatbot",
    instructions="...",
    tools=[search_knowledge_base]
)
```

**Configuration**:
- **Qdrant**: Cloud Free Tier (1GB storage, 1M vectors)
- **Cohere**: Free tier (100 API calls/month for embeddings)
- **Collection**: `robotics_textbook_v1` (already populated)
- **Embedding Model**: `embed-v4.0` (1536 dimensions)
- **Similarity Threshold**: 0.4 (balances precision/recall)

**Alternatives Considered**:
- Direct database access from agent: Rejected - violates separation of concerns, agent should use tools
- Different embedding model: Rejected - must match existing Cohere embeddings for compatibility
- Different vector DB: Rejected - Qdrant already set up and populated

**Implementation Notes**:
- Reuse existing Qdrant credentials (`QDRANT_URL`, `QDRANT_API_KEY`)
- Reuse existing Cohere credentials (`COHERE_API_KEY`)
- Tool context management pattern (AsyncExitStack) from user's example code
- Monitor Cohere API usage (free tier limits)

---

## 6. Database & Storage

### Decision: Neon PostgreSQL (Existing Auth Database)

**Rationale**:
- **User Decision**: Confirmed during clarification - use existing Neon database
- **Simplified Architecture**: Single database for auth + chatbot reduces operational complexity
- **Data Integration**: Easy to query user profiles (first_name, last_name, software_level) via joins
- **Cost Efficiency**: No additional database subscription needed
- **Proven Reliability**: Already in production for auth system

**Schema Approach**:
- **Same Database, Same Schema**: Add new tables alongside existing `users`, `sessions` tables
- **New Tables**: `threads`, `messages`, `attachments`
- **Foreign Keys**: `threads.user_id` → `users.id` for data integrity

**Database Tables** (detailed design in data-model.md):
```sql
-- New tables to add
CREATE TABLE threads (
    thread_id UUID PRIMARY KEY,
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    title VARCHAR(255),
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE TABLE messages (
    message_id UUID PRIMARY KEY,
    thread_id UUID NOT NULL REFERENCES threads(thread_id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    sequence_number INTEGER NOT NULL
);

CREATE TABLE attachments (
    attachment_id UUID PRIMARY KEY,
    message_id UUID NOT NULL REFERENCES messages(message_id) ON DELETE CASCADE,
    file_name VARCHAR(255) NOT NULL,
    file_type VARCHAR(100),
    file_size BIGINT,
    storage_url TEXT NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT NOW()
);
```

**ORM Decision**: SQLAlchemy 2.0+ with AsyncIO support
- **Async ORM**: `asyncpg` driver for async database operations
- **Type Safety**: Pydantic integration for request/response models
- **Migrations**: Alembic for schema versioning

**Alternatives Considered**:
- Separate Neon instance for chatbot: Rejected - unnecessary complexity, extra cost
- MongoDB: Rejected - spec requires relational data, user chose PostgreSQL
- SQLite: Rejected - not suitable for production, lacks async support

**Implementation Notes**:
- Use existing Neon connection string
- Add connection pooling configuration (`max_connections=20`)
- Implement database migrations with Alembic
- Add indexes on frequently queried fields (`user_id`, `thread_id`, `created_at`)

---

## 7. Object Storage (Attachments)

### Decision: Cloudflare R2 (S3-Compatible)

> **Note**: This is a **Phase 3 feature** (Week 4-5). NOT required for MVP. The MVP (Phase 1) focuses on text-only conversations without file attachments. See plan.md for implementation timeline.

**Rationale**:
- **Cost Efficiency**: R2 offers free egress (no bandwidth charges), S3 charges per GB downloaded
- **S3 Compatibility**: Works with existing S3 SDKs (`boto3`)
- **Generous Free Tier**: 10GB storage free, sufficient for MVP
- **Signed URL Support**: Pre-signed URLs for direct client uploads/downloads (FR-035, FR-038)
- **Global CDN**: Low latency for file downloads

**Configuration**:
```python
import boto3
from botocore.client import Config

r2_client = boto3.client(
    's3',
    endpoint_url=os.environ['R2_ENDPOINT_URL'],
    aws_access_key_id=os.environ['R2_ACCESS_KEY_ID'],
    aws_secret_access_key=os.environ['R2_SECRET_ACCESS_KEY'],
    config=Config(signature_version='s3v4'),
    region_name='auto'
)

# Generate upload URL
upload_url = r2_client.generate_presigned_url(
    'put_object',
    Params={'Bucket': 'chatbot-attachments', 'Key': file_key},
    ExpiresIn=3600  # 1 hour
)
```

**Alternatives Considered**:
- AWS S3: Higher cost due to egress fees; R2 more economical for educational project
- MinIO (self-hosted): Requires infrastructure management; R2 managed service preferred
- Neon Blob Storage: Not available; Neon is PostgreSQL-focused

**Implementation Notes**:
- Bucket structure: `chatbot-attachments/{user_id}/{thread_id}/{file_id}`
- Retention policy: Delete attachments when thread is deleted (CASCADE)
- File size limit: 10MB per file (FR-039)
- Allowed MIME types: Whitelist for security

---

## 8. Authentication Integration

### Decision: JWT Token Validation (Existing Auth System)

**Rationale**:
- **Existing Infrastructure**: Auth backend already issues JWT tokens
- **Custom Fetch Pattern**: ChatKit React SDK supports custom fetch with auth headers (FR-007)
- **Stateless**: No session storage needed in chatbot backend
- **User Context**: Decode JWT to get `user_id` for thread ownership (FR-009)

**Integration Flow**:
```
1. User logs in → Auth Backend → JWT token issued
2. Frontend stores JWT
3. ChatKit React SDK → Custom fetch injects Authorization header
4. ChatBot Backend → Validate JWT signature
5. Extract user_id → Associate with thread
```

**JWT Validation** (FastAPI dependency):
```python
from fastapi import Depends, HTTPException, Header
import jwt

async def get_current_user(authorization: str = Header()):
    try:
        token = authorization.replace("Bearer ", "")
        payload = jwt.decode(
            token,
            os.environ["JWT_SECRET"],
            algorithms=["HS256"]
        )
        return payload["user_id"]
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")

# Use in routes
@app.post("/threads")
async def create_thread(user_id: str = Depends(get_current_user)):
    # user_id authenticated and available
```

**Alternatives Considered**:
- OAuth2 Password Flow: Rejected - adds complexity, existing auth system works
- Session Cookies: Rejected - stateless JWT preferred for API

**Implementation Notes**:
- Share `JWT_SECRET` between auth backend and chatbot backend (environment variable)
- Implement token expiration checks
- Add rate limiting per user_id to prevent abuse

---

## 9. API Structure & Endpoints

### Decision: ChatKit-Compatible REST API

**Rationale**:
- **ChatKit SDK Requirement**: Frontend ChatKit React SDK expects specific endpoint structure
- **RESTful Conventions**: Standard HTTP methods for resource operations
- **SSE Streaming**: Server-Sent Events for real-time response delivery

**Core Endpoints** (detailed in contracts/openapi.yaml):
```
POST   /api/threads                    # Create new thread
GET    /api/threads                    # List user's threads
GET    /api/threads/{thread_id}        # Get thread details
DELETE /api/threads/{thread_id}        # Delete thread

POST   /api/threads/{thread_id}/messages  # Send message (streaming response)
GET    /api/threads/{thread_id}/messages  # Get message history

POST   /api/attachments/upload-url     # Generate signed upload URL
GET    /api/attachments/{id}/download-url # Generate signed download URL
```

**Response Format** (ChatKit standard):
```json
{
  "type": "message",
  "id": "msg_123",
  "thread_id": "thread_456",
  "role": "assistant",
  "content": "Here's the answer...",
  "created_at": "2025-12-23T10:00:00Z",
  "metadata": {
    "actions": [...],       // Agentic actions (FR-015)
    "widgets": [...],       // Interactive widgets (FR-019)
    "progress": [...],      // Progress updates (FR-023)
    "client_effects": [...]  // UI effects (FR-027)
  }
}
```

**Streaming Format** (SSE):
```
event: message_start
data: {"type": "message_start", "message_id": "msg_123"}

event: content_delta
data: {"type": "content_delta", "delta": "Hello"}

event: content_delta
data: {"type": "content_delta", "delta": " world"}

event: message_end
data: {"type": "message_end"}
```

**Implementation Notes**:
- Use FastAPI `StreamingResponse` for SSE
- Add CORS middleware for frontend integration
- Implement request/response logging for debugging

---

## 10. Development & Deployment

### Decision: Railway/Render for Hosting

**Rationale**:
- **Constitution Alignment**: "Backend: Railway (FastAPI service)"
- **Free Tier**: Both offer free tiers sufficient for MVP
- **Python Support**: Native Python/FastAPI support
- **Auto-Deploy**: GitHub integration for CI/CD
- **Environment Variables**: Secure secret management

**Recommended**: **Railway**
- More generous free tier (500 hours/month)
- Better async/WebSocket support for SSE
- Easier database integration (Neon)

**Deployment Configuration**:
```toml
# railway.toml
[build]
builder = "NIXPACKS"
buildCommand = "pip install -r requirements.txt"

[deploy]
startCommand = "uvicorn main:app --host 0.0.0.0 --port $PORT"
healthcheckPath = "/health"
healthcheckTimeout = 100
restartPolicyType = "ON_FAILURE"
restartPolicyMaxRetries = 10
```

**Environment Variables** (Railway dashboard):
```
DATABASE_URL=postgresql://...
GEMINI_API_KEY=...
COHERE_API_KEY=...
QDRANT_URL=...
QDRANT_API_KEY=...
R2_ENDPOINT_URL=...
R2_ACCESS_KEY_ID=...
R2_SECRET_ACCESS_KEY=...
JWT_SECRET=...
```

**Alternatives Considered**:
- Vercel: Rejected - optimized for Next.js, not ideal for long-running SSE connections
- Fly.io: Valid alternative, slightly more complex configuration
- AWS EC2: Rejected - over-engineered for MVP, requires infrastructure management

**Implementation Notes**:
- Use `.env.example` for local development
- Never commit `.env` to Git
- Implement health check endpoint (`/health`)
- Add logging with structured format (JSON)

---

## 11. Testing Strategy

### Decision: Pytest + FastAPI TestClient

**Rationale**:
- **Constitution Requirement**: Testing framework must be specified
- **Async Support**: Pytest-asyncio for testing async routes
- **FastAPI Integration**: TestClient simulates requests without running server
- **Coverage**: pytest-cov for code coverage metrics

**Test Levels**:
1. **Unit Tests**: Individual functions (tools, utilities)
2. **Integration Tests**: API endpoints with mocked LLM
3. **Contract Tests**: Validate ChatKit API compliance
4. **E2E Tests**: Full flow with test database

**Example Test**:
```python
import pytest
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

@pytest.mark.asyncio
async def test_create_thread():
    response = client.post(
        "/api/threads",
        headers={"Authorization": "Bearer test_token"},
        json={"title": "Test Thread"}
    )
    assert response.status_code == 201
    assert "thread_id" in response.json()
```

**Mocking Strategy**:
- Mock Gemini API responses (avoid API costs in tests)
- Mock Qdrant searches (use fixture data)
- Mock Cohere embeddings (deterministic vectors)

**Implementation Notes**:
- Install: `pytest`, `pytest-asyncio`, `pytest-cov`, `httpx`
- Target coverage: >80%
- Run in CI/CD pipeline

---

## 12. Monitoring & Observability

### Decision: Structured Logging + Sentry (Error Tracking)

**Rationale**:
- **FR-050-053**: Logging requirements explicitly defined
- **Production Readiness**: Observability critical for debugging issues
- **Free Tiers**: Sentry offers free error tracking

**Logging Implementation**:
```python
import logging
import structlog

# Configure structured logging
structlog.configure(
    processors=[
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.JSONRenderer()
    ],
    logger_factory=structlog.stdlib.LoggerFactory(),
)

logger = structlog.get_logger()

# Usage
logger.info(
    "message_sent",
    user_id=user_id,
    thread_id=thread_id,
    message_length=len(content)
)
```

**Sentry Integration**:
```python
import sentry_sdk
from sentry_sdk.integrations.fastapi import FastAPIIntegration

sentry_sdk.init(
    dsn=os.environ["SENTRY_DSN"],
    integrations=[FastAPIIntegration()],
    traces_sample_rate=0.1,  # 10% of requests
)
```

**Metrics to Track** (FR-052):
- Request latency (p50, p95, p99)
- Streaming duration
- Error rates (by endpoint, by error type)
- Gemini API usage (requests, tokens)
- Qdrant search latency

**Implementation Notes**:
- Use environment variable for log level (`LOG_LEVEL=INFO`)
- Sanitize PII from logs (don't log message content, only metadata)
- Set up Sentry alerts for error spikes

---

## 13. Title Generation Agent

### Decision: Separate Gemini Agent (Lightweight Prompt)

**Rationale**:
- **FR-031-034**: Title generation requirements
- **Async Execution**: Don't block main response (FR-033)
- **Cost Efficiency**: Use shorter prompt, no knowledge base search needed

**Implementation**:
```python
title_agent = Agent(
    name="TitleGenerator",
    instructions="""Generate a concise, descriptive title (max 50 characters)
    for a conversation based on the first user message. Focus on the main topic.
    Return only the title, no explanation."""
)

async def generate_title(first_message: str) -> str:
    try:
        result = await Runner.run(
            starting_agent=title_agent,
            input=first_message,
            run_config=config
        )
        title = result.final_output[:50]  # Enforce max length
        return title if title else "New Conversation"
    except Exception:
        return "New Conversation"  # Graceful fallback

# Call asynchronously after sending first message response
asyncio.create_task(generate_title_and_update(thread_id, message))
```

**Alternatives Considered**:
- Rule-based extraction: Rejected - less accurate, misses context
- Same agent as main chatbot: Rejected - requires separate instructions

**Implementation Notes**:
- Run in background task (don't await)
- Implement timeout (5 seconds max)
- Fallback to "New Conversation" on failure
- Don't overwrite manually set titles

---

## 14. Widget & Progress Event Schema

### Decision: ChatKit Standard Event Format

**Rationale**:
- **Frontend Compatibility**: ChatKit React SDK expects specific event structures
- **Reduced Implementation**: SDK provides parsers/renderers
- **Extensibility**: Can add custom widget types later

**Widget Schema Example**:
```json
{
  "type": "button_group",
  "id": "widget_123",
  "properties": {
    "buttons": [
      {"label": "Option A", "value": "a"},
      {"label": "Option B", "value": "b"}
    ]
  },
  "action_handler": "server"  // or "client"
}
```

**Progress Event Schema**:
```json
{
  "type": "progress_update",
  "operation_id": "op_456",
  "message": "Searching knowledge base...",
  "progress_percentage": 30,
  "timestamp": "2025-12-23T10:00:00Z"
}
```

**Implementation Notes**:
- Reference ChatKit docs for full schema
- Validate widget/event schemas with Pydantic models
- Emit progress events at tool execution start/end

---

## 15. Security Considerations

### Decision: Multi-Layer Security Approach

**Rationale**:
- **FR-048**: Input validation required
- **Production Readiness**: Security critical for educational platform

**Security Measures**:

1. **Input Validation**:
   - Pydantic models for request validation
   - Max message length (10,000 chars)
   - Sanitize user inputs (prevent injection)

2. **Rate Limiting**:
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@app.post("/api/threads/{thread_id}/messages")
@limiter.limit("10/minute")  # 10 messages per minute per user
async def send_message(...):
    ...
```

3. **CORS Configuration**:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[os.environ["FRONTEND_URL"]],  # Specific origin
    allow_credentials=True,
    allow_methods=["GET", "POST", "DELETE"],
    allow_headers=["Authorization", "Content-Type"],
)
```

4. **Secrets Management**:
   - All API keys in environment variables
   - Never log secrets
   - Rotate keys periodically

5. **File Upload Security**:
   - Validate file types (whitelist)
   - Scan for malware (ClamAV integration optional)
   - Enforce size limits

**Implementation Notes**:
- Add security headers (HSTS, X-Content-Type-Options)
- Implement CSRF protection if using cookies
- Regular dependency updates (Dependabot)

---

## Open Questions & Future Research

### Resolved in Planning Phase:
- ✅ Database choice: Neon PostgreSQL (existing)
- ✅ ChatKit SDK usage: Yes (backend + frontend)
- ✅ Development order: Backend first
- ✅ Attachment retention: Thread lifetime

### Deferred to Implementation:
- **Performance Tuning**: Exact connection pool sizes, cache TTLs (determine during load testing)
- **Monitoring Dashboards**: Grafana/Prometheus setup (post-MVP)
- **Advanced Features**: Multi-user collaboration, thread sharing (future phases)

---

## Technology Stack Summary

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| **Language** | Python | 3.12+ | Async support, SDK compatibility |
| **Web Framework** | FastAPI | 0.115+ | Async-first, ChatKit integration |
| **LLM** | Google Gemini | 2.0 Flash | Spec requirement, cost-efficient |
| **AI Framework** | OpenAI Agents SDK | Latest | Agentic workflows, tool calling |
| **Chat SDK** | ChatKit Python | Latest | Backend API structure |
| **Database** | Neon PostgreSQL | Latest | Existing infra, relational data |
| **ORM** | SQLAlchemy | 2.0+ | Async support, migrations |
| **Object Storage** | Cloudflare R2 | - | S3-compatible, free egress |
| **Vector DB** | Qdrant Cloud | Free Tier | Existing embeddings |
| **Embeddings** | Cohere embed-v4.0 | - | Existing model, 1536-dim |
| **Auth** | JWT Validation | - | Existing auth system |
| **Testing** | Pytest | Latest | Async testing, coverage |
| **Logging** | Structlog | Latest | Structured JSON logs |
| **Error Tracking** | Sentry | Free Tier | Production monitoring |
| **Deployment** | Railway | - | Free tier, Python support |

---

## Next Steps

1. ✅ **Research Complete** → Proceed to Phase 1 (Data Model & Contracts)
2. Create `data-model.md` with database schema
3. Generate OpenAPI spec in `contracts/openapi.yaml`
4. Write `quickstart.md` for developer onboarding
5. Update agent context with new technologies
6. Begin implementation with `/sp.tasks`

---

**Research Completed By**: Claude (Sonnet 4.5)
**Approval Required**: Confirm technology choices before proceeding to design phase
**Status**: ✅ Ready for Phase 1 (Data Model & API Contracts)
