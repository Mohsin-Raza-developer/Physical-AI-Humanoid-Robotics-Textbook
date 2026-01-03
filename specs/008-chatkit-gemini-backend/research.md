# Research: ChatKit-Integrated Robotics Chatbot Backend

**Feature ID**: 008-chatkit-gemini-backend
**Research Date**: 2025-12-25
**Status**: Complete

---

## Research Objectives

1. Understand ChatKit Store interface requirements for PostgreSQL implementation
2. Learn Agent SDK integration patterns with ChatKit
3. Determine best practices for function_tool decorators
4. Research streaming patterns for SSE responses
5. Understand context passing and authentication patterns

---

## Research Source

**Primary Source**: Context7 - `/openai/chatkit-python` (ChatKit Python SDK)
- **Code Snippets**: 40
- **Source Reputation**: High
- **Benchmark Score**: 59.3

**Reference Implementation**: `/openai/openai-chatkit-advanced-samples`
- **Code Snippets**: 103
- **Source Reputation**: High
- **Benchmark Score**: 61

---

## Key Findings

### 1. ChatKit Store Interface Implementation

#### Decision
Implement custom PostgreSQL Store class extending `chatkit.stores.Store` base class.

#### Rationale
- ChatKit provides abstract `Store` base class with 14 required methods
- Allows custom database implementation (PostgreSQL via SQLAlchemy)
- Type-safe with generic `TContext` for passing request context
- Supports thread metadata, items, and attachments (though we're text-only)

#### Required Methods (from Context7 research)

**From ChatKit Python SDK documentation**:

```python
from abc import ABC
from typing import Generic, TypeVar
from chatkit.models import Attachment, Page, ThreadItem, ThreadMetadata

TContext = TypeVar("TContext")

class Store(ABC, Generic[TContext]):
    # ID Generation
    def generate_thread_id(self, context: TContext) -> str: ...
    def generate_item_id(
        self,
        item_type: Literal["message", "tool_call", "task", "workflow", "attachment"],
        thread: ThreadMetadata,
        context: TContext,
    ) -> str: ...

    # Thread Operations
    async def load_thread(self, thread_id: str, context: TContext) -> ThreadMetadata: ...
    async def save_thread(self, thread: ThreadMetadata, context: TContext) -> None: ...
    async def load_threads(
        self,
        limit: int,
        after: str | None,
        order: str,
        context: TContext,
    ) -> Page[ThreadMetadata]: ...
    async def delete_thread(self, thread_id: str, context: TContext) -> None: ...

    # Thread Items (Messages)
    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: TContext,
    ) -> Page[ThreadItem]: ...
    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: TContext
    ) -> None: ...
    async def save_item(
        self, thread_id: str, item: ThreadItem, context: TContext
    ) -> None: ...
    async def load_item(
        self, thread_id: str, item_id: str, context: TContext
    ) -> ThreadItem: ...

    # Attachments (NOT NEEDED - text-only chatbot)
    async def save_attachment(self, attachment: Attachment, context: TContext) -> None: ...
    async def load_attachment(self, attachment_id: str, context: TContext) -> Attachment: ...
    async def delete_attachment(self, attachment_id: str, context: TContext) -> None: ...
```

**Our Implementation Strategy**:
- Implement all required methods
- Use SQLAlchemy async ORM for database operations
- Use UUID for `generate_thread_id()` and `generate_item_id()`
- Map ChatKit `ThreadItem` to our `Message` model
- Map ChatKit `ThreadMetadata` to our `Thread` model
- Skip attachment methods (raise `NotImplementedError` - text-only chatbot)

**Context Type**:
```python
TContext = dict  # {"user_id": str, "request": Request}
```

**Alternatives Considered**:
- MemoryStore (pre-built) - Rejected: Not persistent
- Custom SQL without ORM - Rejected: More complex, less type-safe

---

### 2. Agent SDK Integration with ChatKit

#### Decision
Use `stream_agent_response()` helper to convert Agent SDK events to ChatKit format.

#### Rationale
- ChatKit provides `stream_agent_response()` utility function
- Automatically converts Agent SDK run events to ChatKit `ThreadStreamEvent`
- Handles action events (tool calls) automatically
- No manual SSE formatting needed

#### Implementation Pattern (from Context7)

```python
from chatkit.server import ChatKitServer, ThreadStreamEvent
from chatkit.models import UserMessageItem, ThreadMetadata
from agents.sdk.agent import Agent
from agents.sdk.runner import Runner
from agents.sdk.utils import simple_to_agent_input
from chatkit.agents import AgentContext, stream_agent_response
from typing import AsyncIterator, Any

class RoboticsChatbotServer(ChatKitServer):
    def __init__(self, data_store: Store):
        super().__init__(data_store, attachment_store=None)  # No attachments

    # Define agent as class attribute
    assistant_agent = Agent(
        model="gemini-2.5-flash",
        name="Robotics Tutor",
        instructions="You are an expert robotics tutor...",
        tools=[search_textbook]  # Our custom tool
    )

    async def respond(
        self,
        thread: ThreadMetadata,
        input: UserMessageItem | None,
        context: Any,
    ) -> AsyncIterator[ThreadStreamEvent]:
        """Generate AI response with knowledge base search."""

        # Create AgentContext
        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,  # Contains user_id, request
        )

        # Run agent with streaming
        result = Runner.run_streamed(
            self.assistant_agent,
            await simple_to_agent_input(input) if input else [],
            context=agent_context,
        )

        # Stream response (auto-converts to ChatKit events)
        async for event in stream_agent_response(agent_context, result):
            yield event
```

**Key Functions**:
- `simple_to_agent_input()` - Converts ChatKit `UserMessageItem` to Agent SDK input format
- `stream_agent_response()` - Converts Agent SDK events to ChatKit `ThreadStreamEvent`
- `AgentContext` - Wrapper containing thread, store, and request context

**Event Conversion (automatic)**:
```
Agent SDK Event         →  ChatKit Event Type
──────────────────────     ──────────────────
AgentRunStarted         →  message_start
ToolCallStarted         →  action (status="started")
ToolCallCompleted       →  action (status="completed")
ContentDelta            →  content_delta
AgentRunCompleted       →  message_end
```

**Alternatives Considered**:
- Manual SSE formatting - Rejected: ChatKit helper handles this
- Direct OpenAI API - Rejected: ChatKit requires Agent SDK integration

---

### 3. Function Tool Decorator for Knowledge Base Search

#### Decision
Use `@function_tool` decorator with `RunContextWrapper[AgentContext]` for Qdrant integration.

#### Rationale
- Agent SDK standard pattern for custom tools
- Provides typed context access (`ctx.context`)
- Allows streaming widgets (not needed for text-only, but future-proof)
- Automatic action event emission in ChatKit

#### Implementation Pattern (from Context7)

```python
from agents.sdk.decorators import function_tool
from agents.sdk.types import RunContextWrapper
from chatkit.agents import AgentContext
import json

@function_tool(description_override="Search the robotics textbook for relevant content.")
async def search_textbook(
    ctx: RunContextWrapper[AgentContext],
    query: str
) -> str:
    """
    Search the Physical AI textbook for relevant content.

    Args:
        query: Search query (e.g., "inverse kinematics")

    Returns:
        JSON string with search results and citation metadata
    """
    # Access Qdrant client
    from app.services.qdrant_service import qdrant_client

    results = await qdrant_client.search(
        collection_name="textbook_content",
        query_text=query,
        limit=5
    )

    # Format results for agent
    formatted = []
    for idx, result in enumerate(results, 1):
        formatted.append({
            "ref_num": idx,
            "content": result.payload["text"],
            "chapter": result.payload["chapter"],
            "section": result.payload["section"],
            "title": result.payload["title"]
        })

    # Return JSON (agent will parse and use in response)
    return json.dumps(formatted, ensure_ascii=False)
```

**Context Access**:
```python
# Within tool function, access AgentContext:
thread = ctx.context.thread  # ThreadMetadata
store = ctx.context.store    # Store instance
request_context = ctx.context.request_context  # {"user_id": ...}
```

**UI Display (automatic)**:
- Tool call emits `action` event with `status="started"`
- Frontend displays: "🔍 Searching textbook..."
- Tool completion emits `action` event with `status="completed"`
- Frontend displays: "✅ Found content (450ms)"

**Alternatives Considered**:
- Pre-retrieval (search before agent run) - Rejected: Less flexible, agent can't decide when to search
- Client-side tool - Rejected: Qdrant must stay server-side

---

### 4. Server Context Passing for Authentication

#### Decision
Pass `user_id` via server context, validate in Store methods.

#### Rationale
- ChatKit supports arbitrary context dictionary
- Context flows through all Store methods
- Enables authorization checks (thread ownership)
- Better Auth session ID → user_id extracted by middleware

#### Implementation Pattern (from Context7)

```python
# In FastAPI endpoint
@app.post("/chatkit")
async def chatkit_endpoint(request: Request, user_id: str = Depends(get_current_user)):
    """
    ChatKit endpoint with Better Auth session validation.

    Middleware extracts user_id from Authorization header.
    """
    result = await server.process(
        await request.body(),
        context={"user_id": user_id, "request": request}  # ← Context
    )

    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    else:
        return Response(content=result.json, media_type="application/json")
```

```python
# In Store implementation
class PostgresStore(Store):
    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        """Load thread with ownership check."""
        user_id = context["user_id"]

        # Query database
        thread = await self.db.query(Thread).filter_by(thread_id=thread_id).first()

        if not thread:
            raise NotFoundError("Thread not found")

        # Authorization check
        if str(thread.user_id) != user_id:
            raise ForbiddenError("Access denied")

        return thread.to_metadata()
```

**Context Structure**:
```python
{
    "user_id": "uuid-string",      # From Better Auth session
    "request": Request,            # FastAPI request object
}
```

**Alternatives Considered**:
- Thread-local storage - Rejected: Not async-safe
- Global state - Rejected: Not thread-safe with concurrent requests

---

### 5. Thread Metadata Management

#### Decision
Use `thread.metadata` dict for storing `previous_response_id` and custom fields.

#### Rationale
- ChatKit `ThreadMetadata.metadata` is arbitrary JSON dict
- Allows storing Agent SDK `response_id` for efficient re-runs
- Can store custom fields (e.g., auto-title flag)

#### Implementation Pattern (from Context7)

```python
async def respond(
    self,
    thread: ThreadMetadata,
    input: UserMessageItem | None,
    context: Any,
) -> AsyncIterator[ThreadStreamEvent]:
    """Generate response with metadata management."""

    # Get previous response_id from thread metadata
    previous_response_id = thread.metadata.get("previous_response_id")

    # Create agent context
    agent_context = AgentContext(
        thread=thread,
        store=self.store,
        request_context=context,
    )

    # Run agent with previous_response_id (optimization)
    result = Runner.run_streamed(
        self.assistant_agent,
        await simple_to_agent_input(input) if input else [],
        context=agent_context,
        previous_response_id=previous_response_id,  # ← Reuse previous run
    )

    # Stream response
    async for event in stream_agent_response(agent_context, result):
        yield event

    # Save new response_id for next run
    thread.metadata["previous_response_id"] = result.response_id
    await self.store.save_thread(thread, context)
```

**Metadata Fields We'll Use**:
```python
{
    "previous_response_id": "uuid-string",  # Agent SDK optimization
    "auto_titled": bool,                    # Whether title auto-generated
}
```

**Alternatives Considered**:
- Separate metadata table - Rejected: ChatKit already provides metadata dict
- Store in message content - Rejected: Not appropriate

---

### 6. Automatic Thread Title Generation

#### Decision
Use separate agent run to generate title from first user message (optional).

#### Rationale
- ChatKit pattern for automatic thread titling
- Runs asynchronously (doesn't block response)
- Only runs if thread.title is None

#### Implementation Pattern (from Context7)

```python
async def maybe_update_thread_title(
    self,
    thread: ThreadMetadata,
    input_item: UserMessageItem,
) -> None:
    """Generate thread title from first message."""
    if thread.title is not None:
        return  # Already has title

    # Create title generation agent
    title_agent = Agent(
        model="gemini-2.5-flash",
        name="Title Generator",
        instructions="Generate a concise 3-5 word title for this conversation."
    )

    # Convert user message to agent input
    agent_input = await simple_to_agent_input(input_item)

    # Run title generation (non-streaming)
    run = await Runner.run(title_agent, input=agent_input)

    # Update thread title
    thread.title = run.final_output
    await self.store.save_thread(thread, {})

async def respond(
    self,
    thread: ThreadMetadata,
    input: UserMessageItem | None,
    context: Any,
) -> AsyncIterator[ThreadStreamEvent]:
    """Generate response with auto-titling."""

    # Trigger auto-title (async, non-blocking)
    if input is not None:
        asyncio.create_task(self.maybe_update_thread_title(thread, input))

    # Generate model response (main flow)
    # ...
```

**Decision for Our Project**: **NOT IMPLEMENTING** in MVP
- User can provide title when creating thread
- Reduces API calls to Gemini
- Simpler implementation
- Can add later if needed

**Alternatives Considered**:
- Title from first N words - Rejected: Not meaningful
- Manual title only - Accepted: Simpler for MVP

---

### 7. FastAPI Integration

#### Decision
Single `/chatkit` POST endpoint with StreamingResponse for SSE.

#### Rationale
- ChatKit protocol uses single endpoint for all operations
- Action determined by request body (`action` field)
- Returns SSE stream or JSON based on response type

#### Implementation Pattern (from Context7)

```python
from fastapi import FastAPI, Request
from fastapi.responses import Response, StreamingResponse
from chatkit.server import StreamingResult

app = FastAPI()

# Initialize server
data_store = PostgresStore()
server = RoboticsChatbotServer(data_store)

@app.post("/chatkit")
async def chatkit_endpoint(
    request: Request,
    user_id: str = Depends(get_current_user)
):
    """
    ChatKit protocol endpoint.

    Handles all ChatKit operations:
    - create_thread
    - list_threads
    - get_thread
    - delete_thread
    - send_message
    - get_messages
    """
    # Process request
    result = await server.process(
        await request.body(),
        context={"user_id": user_id, "request": request}
    )

    # Return appropriate response type
    if isinstance(result, StreamingResult):
        # SSE stream (for send_message)
        return StreamingResponse(
            result,
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no"  # Disable nginx buffering
            }
        )
    else:
        # JSON response (for other operations)
        return Response(
            content=result.json,
            media_type="application/json"
        )
```

**Request Routing** (automatic by ChatKit):
- `server.process()` handles routing based on `action` field
- No need for multiple endpoints
- ChatKit Server internally routes to appropriate method

**Alternatives Considered**:
- Multiple endpoints (/threads, /messages) - Rejected: ChatKit protocol requires single endpoint
- WebSocket - Rejected: ChatKit uses SSE (simpler, HTTP-based)

---

## Technology Decisions

### 1. Database: PostgreSQL with SQLAlchemy

**Decision**: Use Neon PostgreSQL with SQLAlchemy async ORM

**Rationale**:
- Already in use for Better Auth
- SQLAlchemy provides type-safe async operations
- Alembic for migrations
- Connection pooling built-in

**Implementation**:
```python
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker

engine = create_async_engine(
    os.getenv("DATABASE_URL"),
    echo=False,
    pool_size=20,
    max_overflow=10
)

async_session = sessionmaker(
    engine, class_=AsyncSession, expire_on_commit=False
)

async def get_db():
    async with async_session() as session:
        yield session
```

---

### 2. AI Model: Google Gemini 2.5 Flash

**Decision**: Access via OpenAI-compatible endpoint

**Rationale**:
- Agent SDK supports OpenAI client
- Gemini 2.5 Flash is cost-effective
- Supports function calling (for our search_textbook tool)

**Implementation**:
```python
from openai import OpenAI

client = OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

agent = Agent(
    model="gemini-2.5-flash",
    client=client,
    instructions="...",
    tools=[search_textbook]
)
```

---

### 3. Vector Search: Qdrant Cloud (Pre-existing)

**Decision**: Use existing Qdrant collection with Cohere embeddings

**Rationale**:
- Textbook content already indexed
- Cohere embed-v4.0 embeddings already generated
- No need to re-index

**Implementation**:
```python
from qdrant_client import QdrantClient

qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=5.0
)

# Search (inside function_tool)
results = qdrant_client.search(
    collection_name="textbook_content",
    query_text=query,
    limit=5
)
```

---

### 4. Authentication: Better Auth (Pre-existing)

**Decision**: Validate sessions via database query

**Rationale**:
- Better Auth manages users and sessions
- Backend is read-only
- Session validation < 100ms (spec requirement)

**Implementation**:
```python
from sqlalchemy import text

async def validate_session(session_id: str, db: AsyncSession) -> str:
    """Validate Better Auth session, return user_id."""
    query = text("""
        SELECT "userId", "expiresAt"
        FROM session
        WHERE id = :session_id
    """)

    result = await db.execute(query, {"session_id": session_id})
    row = result.fetchone()

    if not row:
        raise AuthenticationError("Invalid session")

    user_id, expires_at = row

    # Check expiration
    if expires_at < datetime.now(timezone.utc):
        raise AuthenticationError("Session expired")

    return str(user_id)
```

---

## Performance Considerations

### 1. Database Connection Pooling

**Research Finding**: SQLAlchemy async pool recommended for concurrent requests

**Configuration**:
```python
engine = create_async_engine(
    DATABASE_URL,
    pool_size=20,        # Max connections per instance
    max_overflow=10,     # Additional connections if needed
    pool_pre_ping=True,  # Verify connection before use
)
```

---

### 2. Qdrant Search Timeout

**Research Finding**: Vector search should complete in < 500ms (spec requirement)

**Configuration**:
```python
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    timeout=5.0  # 5 second timeout (allows for network latency)
)
```

---

### 3. Agent SDK Response Caching

**Research Finding**: Use `previous_response_id` for efficient re-runs

**Implementation**:
```python
# Store response_id in thread metadata
result = Runner.run_streamed(
    agent,
    input=...,
    previous_response_id=thread.metadata.get("previous_response_id")
)

# Save for next run
thread.metadata["previous_response_id"] = result.response_id
```

---

## Security Best Practices

### 1. Context-Based Authorization

**Research Finding**: Use context to pass user_id, check in Store methods

**Pattern**:
```python
async def load_thread(self, thread_id: str, context: dict):
    user_id = context["user_id"]
    thread = await db.query(Thread).filter_by(thread_id=thread_id).first()

    if thread.user_id != user_id:
        raise ForbiddenError("Access denied")

    return thread
```

---

### 2. Input Validation

**Research Finding**: ChatKit handles protocol validation, but sanitize tool inputs

**Pattern**:
```python
@function_tool
async def search_textbook(ctx, query: str) -> str:
    # Sanitize query
    query = query.strip()[:500]  # Max 500 chars

    if not query:
        return json.dumps({"error": "Empty query"})

    # Proceed with search
    ...
```

---

## Testing Strategies

### 1. Store Interface Testing

**Research Finding**: Test all required Store methods

**Example**:
```python
@pytest.mark.asyncio
async def test_store_load_thread():
    store = PostgresStore()
    context = {"user_id": "test-user"}

    # Create thread
    thread_id = store.generate_thread_id(context)
    await store.save_thread(ThreadMetadata(id=thread_id), context)

    # Load thread
    loaded = await store.load_thread(thread_id, context)
    assert loaded.id == thread_id
```

---

### 2. Agent Tool Testing

**Research Finding**: Test function_tool independently

**Example**:
```python
@pytest.mark.asyncio
async def test_search_textbook_tool():
    # Mock context
    class MockContext:
        thread = ThreadMetadata(id="test")
        store = None
        request_context = {}

    ctx = RunContextWrapper(context=MockContext())

    # Test tool
    result = await search_textbook(ctx, "inverse kinematics")
    data = json.loads(result)

    assert len(data) <= 5
    assert all("content" in item for item in data)
```

---

## Known Limitations

### 1. Gemini API Rate Limits

**Research Finding**: Gemini API may have different rate limits than OpenAI

**Mitigation**:
- Monitor rate limit headers
- Implement exponential backoff
- Consider API tier upgrade if needed

---

### 2. ChatKit Store Type Shape Changes

**Research Finding**: ChatKit may change ThreadItem/ThreadMetadata types across versions

**Mitigation**:
- Version-lock `openai-chatkit` dependency
- Test after upgrades
- Use flexible metadata dict (not typed fields)

---

## Alternatives Not Chosen

### 1. Custom SSE Implementation (Rejected)

**Why Rejected**: ChatKit provides `stream_agent_response()` helper
- Manual SSE formatting is error-prone
- ChatKit helper handles all event types
- Frontend expects ChatKit event format

---

### 2. Pre-Retrieval Pattern (Rejected)

**Why Rejected**: Agent Tool pattern is more flexible
- Agent decides when to search (not always needed for follow-ups)
- Better user experience (shows "Searching..." action)
- Reduces unnecessary Qdrant calls

---

### 3. JWT-Based Authentication (Rejected)

**Why Rejected**: Better Auth uses session-based auth
- Better Auth already manages sessions in database
- No need to introduce JWT complexity
- Session validation is fast (< 100ms)

---

## Open Questions (Resolved)

1. **How to pass user_id through ChatKit?** → Use server context dict
2. **How to display tool calls in UI?** → ChatKit automatically emits action events
3. **How to format citations?** → Inline Markdown footnotes (`[^1]`)
4. **How to handle Qdrant timeout?** → Set 5s timeout, retry on failure
5. **Do we need AttachmentStore?** → No, text-only chatbot

---

## Next Steps

1. ✅ Create `data-model.md` with entity definitions
2. ✅ Create `contracts/` with OpenAPI schema
3. ✅ Create `quickstart.md` for developer onboarding
4. ⏳ Implement PostgresStore class
5. ⏳ Implement RoboticsChatbotServer class
6. ⏳ Implement search_textbook function_tool
7. ⏳ Write unit tests for Store and Agent
8. ⏳ Integration testing with Better Auth

---

**Research Status**: ✅ Complete
**Key Decisions**: 7 major decisions documented
**Implementation Ready**: Yes - All patterns researched and validated via Context7
