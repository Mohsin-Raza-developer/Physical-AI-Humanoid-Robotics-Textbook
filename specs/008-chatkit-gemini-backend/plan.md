# Implementation Plan: ChatKit-Integrated Robotics Chatbot Backend

**Feature ID**: 008-chatkit-gemini-backend
**Created**: 2025-12-25
**Status**: Planning
**Branch**: 008-chatkit-gemini-backend

---

## Executive Summary

This plan outlines the implementation of a production-ready text-only chatbot backend that integrates OpenAI's ChatKit framework with Google Gemini AI. The system provides an intelligent robotics learning assistant for the Physical AI and Humanoid Robotics textbook, featuring knowledge-grounded responses with inline Markdown citations, real-time streaming via SSE, and seamless integration with existing Better Auth authentication.

**Key Architectural Decisions**:
- Single `/chatkit` POST endpoint (ChatKit protocol)
- Agent Tool pattern for Qdrant vector search
- Inline Markdown footnote citations (`[^1]` syntax)
- Text-only interface (no file upload)
- PostgreSQL Store implementation (3 entities: Thread, Message, User)

---

## Phase 0: Technical Context & Research

### Technology Stack

**Backend Framework**:
- FastAPI 0.115+ (async web framework)
- Python 3.12+
- Uvicorn (ASGI server)

**AI & Chat Framework**:
- openai-chatkit 1.0+ (ChatKit server protocol)
- openai-agents 0.2+ (agent orchestration)
- openai 1.0+ (client library)

**Database & Storage**:
- Neon PostgreSQL (managed database)
- SQLAlchemy 2.0+ with asyncpg driver
- Alembic (migrations)

**Vector Search & Embeddings**:
- Qdrant Cloud (vector database - pre-existing)
- Cohere embed-v4.0 (embeddings - pre-existing)

**Authentication**:
- Better Auth (pre-existing, managed by frontend)

**Deployment**:
- Production environment with PostgreSQL access
- TLS 1.3 encryption

### Architecture Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                    FRONTEND (Docusaurus)                     │
│                                                               │
│  Better Auth Session ────┐                                   │
│  ChatKit React SDK       │                                   │
│  ChatView Component      │                                   │
└──────────────────────────┼───────────────────────────────────┘
                           │
                           │ POST /chatkit
                           │ Authorization: Bearer <session_id>
                           │
┌──────────────────────────▼───────────────────────────────────┐
│                    BACKEND (FastAPI)                         │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  ChatKit Server (RoboticsChatbotServer)             │    │
│  │                                                       │    │
│  │  handle_chatkit_request()                           │    │
│  │         ↓                                            │    │
│  │  respond() ──→ Agent Tool: search_textbook()       │    │
│  │         ↓                  ↓                         │    │
│  │  stream_agent_response()   Qdrant Search           │    │
│  │         ↓                  ↓                         │    │
│  │  SSE Events ←─────── Citations (Markdown)          │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  PostgreSQL Store (PostgresStore)                    │    │
│  │                                                       │    │
│  │  - create_thread()                                   │    │
│  │  - get_thread()                                      │    │
│  │  - create_message()                                  │    │
│  │  - get_messages()                                    │    │
│  │  - delete_thread()                                   │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Better Auth Session Middleware                      │    │
│  │                                                       │    │
│  │  validate_session() → Query session table           │    │
│  └─────────────────────────────────────────────────────┘    │
└───────────────────────────────────────────────────────────────┘
                           │
                    ┌──────┴──────┐
                    │             │
                    ▼             ▼
          ┌─────────────┐  ┌──────────────┐
          │   Neon      │  │   Qdrant     │
          │ PostgreSQL  │  │    Cloud     │
          │             │  │              │
          │ - users     │  │ - textbook_  │
          │ - session   │  │   content    │
          │ - threads   │  │   (vectors)  │
          │ - messages  │  │              │
          └─────────────┘  └──────────────┘
                    │
                    ▼
          ┌──────────────────┐
          │  Google Gemini   │
          │   2.5 Flash      │
          │ (via OpenAI API) │
          └──────────────────┘
```

### Research Decisions

#### 1. ChatKit Store Interface Implementation

**Decision**: Custom PostgreSQL implementation using SQLAlchemy ORM

**Rationale**:
- ChatKit provides abstract `Store` interface requiring 20+ methods
- PostgreSQL already in use for Better Auth (Neon database)
- SQLAlchemy provides type-safe async operations
- No attachments table needed (text-only chatbot)

**Alternatives Considered**:
- MemoryStore (not persistent - rejected)
- Custom raw SQL implementation (more complex - rejected)

**Implementation Pattern**:
```python
from chatkit import Store
from sqlalchemy.ext.asyncio import AsyncSession

class PostgresStore(Store):
    def __init__(self, db_session: AsyncSession):
        self.db = db_session

    async def create_thread(self, user_id, metadata):
        # SQLAlchemy insert
        pass

    async def get_thread(self, thread_id):
        # SQLAlchemy query
        pass

    # ... 18+ more required methods
```

#### 2. Agent Tool Pattern for Qdrant Integration

**Decision**: Define `@function_tool` decorator for knowledge base search, agent decides when to invoke

**Rationale**:
- OpenAI Agents SDK standard pattern
- Agent can intelligently decide when search is needed
- ChatKit automatically emits action events for UI display
- Flexible for follow-up questions (agent can skip search if context sufficient)

**Alternatives Considered**:
- Pre-retrieval (always search before every response - less flexible)
- Hybrid approach (initial + tool) - unnecessary complexity

**Implementation Pattern**:
```python
from openai.agents import Agent, function_tool

@function_tool
async def search_textbook(query: str) -> str:
    """Search robotics textbook for relevant content with citations."""
    results = await qdrant_client.search(
        collection_name="textbook_content",
        query_text=query,
        limit=5
    )

    # Format results with citation metadata
    formatted = []
    for idx, result in enumerate(results, 1):
        formatted.append({
            "ref_num": idx,
            "content": result.payload["text"],
            "chapter": result.payload["chapter"],
            "section": result.payload["section"],
            "title": result.payload["title"]
        })

    return json.dumps(formatted)

agent = Agent(
    model="gemini-2.5-flash",
    instructions="""You are a robotics tutor for the Physical AI textbook.

    When answering questions:
    1. Use search_textbook tool to find relevant textbook content
    2. Cite sources using inline Markdown footnotes: [^1], [^2], etc.
    3. Add reference list at bottom: [^1]: Chapter X: Title, Section Y.Z
    4. Be concise, educational, and accurate
    5. If textbook doesn't cover topic, clearly state this and suggest related topics
    """,
    tools=[search_textbook]
)
```

#### 3. Citation Format: Inline Markdown Footnotes

**Decision**: Use `[^1]` syntax with reference list at response bottom

**Rationale**:
- Standard Markdown footnote syntax
- ChatKit React SDK automatically renders Markdown
- Natural reading flow (citations don't interrupt text)
- Frontend can easily convert to clickable links

**Alternatives Considered**:
- Structured JSON metadata (requires custom rendering - rejected)
- Inline URLs (clutters text readability - rejected)

**Example Output**:
```markdown
Inverse kinematics is the process of determining joint angles[^1] needed to achieve a desired end-effector position and orientation[^2]. This is essential for robot motion planning[^3].

References:
[^1]: Chapter 5: Robot Kinematics, Section 5.3
[^2]: Chapter 5: Robot Kinematics, Section 5.4
[^3]: Chapter 6: Motion Planning, Section 6.1
```

#### 4. Better Auth Session Validation

**Decision**: Query `session` table directly using raw SQL via SQLAlchemy text()

**Rationale**:
- Better Auth manages users/sessions tables
- Backend is read-only for these tables
- Session validation must complete in < 100ms (spec requirement)
- Simple query: `SELECT userId, expiresAt FROM session WHERE id = $1`

**Implementation Pattern**:
```python
from sqlalchemy import text
from datetime import datetime, timezone

async def validate_session(session_id: str, db: AsyncSession) -> str:
    """Validate Better Auth session and return user_id."""
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

    if expires_at.replace(tzinfo=timezone.utc) < datetime.now(timezone.utc):
        raise AuthenticationError("Session expired")

    return str(user_id)
```

#### 5. SSE Streaming Implementation

**Decision**: Use ChatKit's `stream_agent_response()` helper to convert Agent SDK events to SSE format

**Rationale**:
- ChatKit provides automatic conversion
- Handles all event types (content_delta, action, message_start, message_end)
- Frontend (ChatKit React SDK) expects ChatKit event format
- No manual SSE formatting needed

**Event Flow**:
```
Agent SDK Events          ChatKit Events           Frontend Display
─────────────────         ──────────────           ────────────────
AgentRunStarted    →      message_start      →     "🚀 Thinking..."
ToolCallStarted    →      action (started)   →     "🔍 Searching textbook..."
ToolCallCompleted  →      action (completed) →     "✅ Found content (450ms)"
ContentDelta       →      content_delta      →     "Inverse kinematics..."
AgentRunCompleted  →      message_end        →     "✓ Complete"
```

---

## Phase 1: Data Model & API Contracts

### Database Schema

#### Entity: Thread

**Table**: `threads`

```sql
CREATE TABLE threads (
    thread_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    title VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL
);

CREATE INDEX idx_threads_user_id ON threads(user_id);
CREATE INDEX idx_threads_created_at ON threads(created_at);
```

**Attributes**:
- `thread_id`: UUID (primary key, auto-generated)
- `user_id`: UUID (foreign key to Better Auth users table)
- `title`: String (optional, max 255 chars)
- `created_at`: Timestamp with timezone
- `updated_at`: Timestamp with timezone

**Relationships**:
- Belongs to one User (Better Auth)
- Has many Messages

**Validation Rules**:
- `user_id` must exist in `users` table
- `title` can be NULL (auto-generated from first message)
- `updated_at` auto-updates on thread modification

---

#### Entity: Message

**Table**: `messages`

```sql
CREATE TABLE messages (
    message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    thread_id UUID NOT NULL REFERENCES threads(thread_id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,
    sequence_number INTEGER NOT NULL,
    UNIQUE(thread_id, sequence_number)
);

CREATE INDEX idx_messages_thread_id ON messages(thread_id);
CREATE INDEX idx_messages_created_at ON messages(created_at);
```

**Attributes**:
- `message_id`: UUID (primary key, auto-generated)
- `thread_id`: UUID (foreign key to threads)
- `role`: Enum ('user' | 'assistant')
- `content`: Text (unlimited length)
- `created_at`: Timestamp with timezone
- `sequence_number`: Integer (unique per thread, auto-incremented)

**Relationships**:
- Belongs to one Thread

**Validation Rules**:
- `role` must be either 'user' or 'assistant'
- `content` cannot be empty
- `sequence_number` must be unique within thread (prevents ordering conflicts)
- Messages cascade-delete when thread is deleted

---

#### Entity: User (Read-Only)

**Table**: `users` (managed by Better Auth, read-only for chatbot backend)

```sql
-- Existing table (managed by Better Auth)
-- Backend only reads for FK relationships

SELECT id, email, first_name, last_name, software_level, hardware_access
FROM users
WHERE id = $1;
```

**Attributes** (read-only):
- `id`: UUID (primary key, managed by Better Auth)
- `email`: String
- `first_name`: String (optional)
- `last_name`: String (optional)
- `software_level`: Enum ('Beginner' | 'Intermediate' | 'Advanced')
- `hardware_access`: Enum ('Laptop/Cloud' | 'Physical Robot')

**Relationships**:
- Has many Threads

**Note**: Backend never inserts/updates users table. All user management handled by Better Auth frontend.

---

### API Contract: ChatKit Protocol

**Endpoint**: `POST /chatkit`

**Authentication**: Bearer token (Better Auth session ID)

**Content-Type**: `application/json`

**Request Format** (ChatKit protocol):

```json
{
  "action": "create_thread" | "list_threads" | "get_thread" | "delete_thread" |
            "send_message" | "get_messages",
  "thread_id": "uuid-string" (required for thread-specific actions),
  "content": "message text" (required for send_message),
  "metadata": {
    "title": "optional thread title"
  },
  "limit": 20 (optional, for pagination),
  "offset": 0 (optional, for pagination)
}
```

**Response Formats**:

#### 1. Create Thread

```json
{
  "thread_id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "title": "ROS 2 Questions",
  "created_at": "2025-12-25T10:30:00Z",
  "updated_at": "2025-12-25T10:30:00Z"
}
```

#### 2. List Threads

```json
{
  "threads": [
    {
      "thread_id": "...",
      "title": "...",
      "created_at": "...",
      "updated_at": "...",
      "message_count": 12
    }
  ],
  "total": 42,
  "limit": 20,
  "offset": 0
}
```

#### 3. Send Message (SSE Stream)

**Content-Type**: `text/event-stream`

```
event: message_start
data: {"type":"message_start","sequence_number":2}

event: action_event
data: {"type":"action","status":"started","name":"search_textbook","input":{"query":"inverse kinematics"}}

event: action_event
data: {"type":"action","status":"completed","name":"search_textbook","duration_ms":450,"output":"..."}

event: content_delta
data: {"type":"content_delta","delta":"Inverse"}

event: content_delta
data: {"type":"content_delta","delta":" kinematics"}

event: content_delta
data: {"type":"content_delta","delta":" is..."}

event: message_end
data: {"type":"message_end","message_id":"uuid-456"}
```

#### 4. Get Messages

```json
{
  "messages": [
    {
      "message_id": "...",
      "role": "user",
      "content": "Explain inverse kinematics",
      "created_at": "...",
      "sequence_number": 1
    },
    {
      "message_id": "...",
      "role": "assistant",
      "content": "Inverse kinematics is...[^1]\n\nReferences:\n[^1]: Chapter 5: Robot Kinematics, Section 5.3",
      "created_at": "...",
      "sequence_number": 2
    }
  ],
  "total": 24,
  "limit": 50,
  "offset": 0
}
```

#### 5. Error Response

```json
{
  "error": {
    "code": "ERR_NOT_FOUND" | "ERR_FORBIDDEN" | "ERR_INVALID_REQUEST" | "ERR_INTERNAL",
    "message": "Thread not found",
    "details": {}
  }
}
```

**Status Codes**:
- `200 OK`: Successful operation
- `401 Unauthorized`: Invalid/expired session
- `403 Forbidden`: User doesn't own resource
- `404 Not Found`: Thread/message not found
- `422 Unprocessable Entity`: Invalid request payload
- `500 Internal Server Error`: Backend error

---

### Additional API Endpoints

#### Health Check

```
GET /health

Response:
{
  "status": "healthy",
  "environment": "production",
  "database": "connected",
  "qdrant": "connected"
}
```

---

## Phase 2: Component Architecture

### Directory Structure

```
chatbot-backend/
├── app/
│   ├── main.py                    # FastAPI application entry point
│   ├── chatkit_server.py          # RoboticsChatbotServer (ChatKitServer subclass)
│   ├── database.py                # Database session management
│   ├── config.py                  # Environment configuration
│   │
│   ├── middleware/
│   │   ├── __init__.py
│   │   └── auth.py                # Better Auth session validation
│   │
│   ├── models/
│   │   ├── __init__.py
│   │   ├── user.py                # User model (read-only)
│   │   ├── thread.py              # Thread model
│   │   └── message.py             # Message model
│   │
│   ├── store/
│   │   ├── __init__.py
│   │   └── postgres_store.py      # PostgresStore (Store interface implementation)
│   │
│   ├── services/
│   │   ├── __init__.py
│   │   ├── agent_service.py       # Agent + search_textbook tool
│   │   └── qdrant_service.py      # Qdrant vector search client
│   │
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── thread.py              # Pydantic schemas for threads
│   │   └── message.py             # Pydantic schemas for messages
│   │
│   └── utils/
│       ├── __init__.py
│       ├── errors.py              # Custom exception classes
│       └── logger.py              # Structured logging setup
│
├── alembic/
│   ├── versions/
│   │   └── 001_initial_schema.py  # Create threads & messages tables
│   └── env.py
│
├── tests/
│   ├── test_chatkit_server.py     # ChatKit server tests
│   ├── test_store.py              # PostgreSQL store tests
│   ├── test_agent.py              # Agent tool tests
│   └── test_auth.py               # Session validation tests
│
├── .env                           # Environment variables
├── requirements.txt               # Python dependencies
├── alembic.ini                    # Alembic configuration
└── README.md                      # Setup instructions
```

---

### Component Specifications

#### 1. ChatKit Server (`app/chatkit_server.py`)

**Responsibilities**:
- Implement ChatKitServer subclass
- Handle `respond()` method for AI responses
- Integrate with PostgresStore
- Stream responses via SSE

**Key Methods**:
```python
class RoboticsChatbotServer(ChatKitServer):
    def __init__(self, store: PostgresStore, auth_middleware):
        super().__init__(store=store)
        self.auth = auth_middleware
        self.agent = create_robotics_agent()

    async def respond(self, request):
        """Generate AI response with knowledge base search."""
        # 1. Validate session (middleware already did this)
        # 2. Get conversation history from store
        # 3. Stream agent response with search_textbook tool
        # 4. Save assistant message to store
        pass
```

**Dependencies**:
- PostgresStore (conversation persistence)
- Agent service (AI response generation)
- Better Auth middleware (session validation)

---

#### 2. PostgreSQL Store (`app/store/postgres_store.py`)

**Responsibilities**:
- Implement ChatKit Store interface (20+ methods)
- Manage threads and messages in PostgreSQL
- Handle cascade deletions
- Maintain sequence numbers

**Key Methods**:
```python
class PostgresStore(Store):
    async def create_thread(self, user_id, metadata) -> str:
        """Create new thread, return thread_id."""
        pass

    async def get_thread(self, thread_id) -> dict:
        """Retrieve thread by ID."""
        pass

    async def list_threads(self, user_id, limit, offset) -> list:
        """List user's threads with pagination."""
        pass

    async def delete_thread(self, thread_id):
        """Delete thread and cascade messages."""
        pass

    async def create_message(self, thread_id, role, content) -> str:
        """Create message with auto-incremented sequence_number."""
        pass

    async def get_messages(self, thread_id, limit, offset) -> list:
        """Get thread messages ordered by sequence_number."""
        pass
```

**Database Transactions**:
- All write operations use transactions
- Read operations use read-committed isolation
- Cascade deletes handled by foreign key constraints

---

#### 3. Agent Service (`app/services/agent_service.py`)

**Responsibilities**:
- Create Agent with search_textbook tool
- Define agent instructions for citation format
- Handle tool execution and results formatting

**Implementation**:
```python
from openai.agents import Agent, function_tool
import json

@function_tool
async def search_textbook(query: str) -> str:
    """Search robotics textbook for relevant content with citations."""
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

    return json.dumps(formatted, ensure_ascii=False)

def create_robotics_agent() -> Agent:
    """Create Agent instance with search tool."""
    return Agent(
        model="gemini-2.5-flash",
        instructions="""You are an expert robotics tutor for the Physical AI and Humanoid Robotics textbook.

        Guidelines:
        1. Always use search_textbook tool to find relevant content before answering
        2. Cite all factual statements using inline Markdown footnotes: [^1], [^2], etc.
        3. Add reference list at bottom in format: [^1]: Chapter X: Title, Section Y.Z
        4. Be concise, educational, and accurate
        5. If the textbook doesn't cover a topic, clearly state this and suggest related topics that are covered
        6. For follow-up questions, use conversation context first; only search if new information needed

        Example response format:

        Inverse kinematics is the mathematical process of calculating joint angles[^1] required to position a robot's end-effector at a specific location[^2].

        References:
        [^1]: Chapter 5: Robot Kinematics, Section 5.3
        [^2]: Chapter 5: Robot Kinematics, Section 5.4
        """,
        tools=[search_textbook]
    )
```

---

#### 4. Better Auth Middleware (`app/middleware/auth.py`)

**Responsibilities**:
- Validate session ID from Authorization header
- Query Better Auth session table
- Check session expiration
- Return user_id for valid sessions

**Implementation**:
```python
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import text
from datetime import datetime, timezone

security = HTTPBearer()

async def get_current_user(
    credentials = Depends(security),
    db: AsyncSession = Depends(get_db)
) -> str:
    """Dependency to validate Better Auth session and extract user_id."""
    session_id = credentials.credentials

    # Query Better Auth session table
    query = text("""
        SELECT "userId", "expiresAt"
        FROM session
        WHERE id = :session_id
    """)

    result = await db.execute(query, {"session_id": session_id})
    row = result.fetchone()

    if not row:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid session"
        )

    user_id, expires_at = row

    # Check expiration
    if expires_at.replace(tzinfo=timezone.utc) < datetime.now(timezone.utc):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session expired"
        )

    return str(user_id)
```

**Performance Target**: < 100ms (spec requirement)

---

#### 5. Qdrant Service (`app/services/qdrant_service.py`)

**Responsibilities**:
- Initialize Qdrant client connection
- Perform vector similarity search
- Format search results

**Implementation**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, SearchRequest

class QdrantService:
    def __init__(self, url: str, api_key: str):
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
            timeout=5.0  # 500ms target (spec requirement)
        )

    async def search(
        self,
        collection_name: str,
        query_text: str,
        limit: int = 5
    ):
        """Search for relevant textbook content."""
        # Cohere embeddings already exist in Qdrant
        # Search using text query (Qdrant handles embedding internally if configured)
        results = self.client.search(
            collection_name=collection_name,
            query_text=query_text,
            limit=limit
        )

        return results

# Singleton instance
qdrant_client = QdrantService(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)
```

**Performance Target**: < 500ms for 95% of queries (spec requirement)

---

## Phase 3: Integration Points

### 1. Better Auth Integration

**Session Table Query** (read-only):
```sql
SELECT "userId", "expiresAt"
FROM session
WHERE id = $1;
```

**User Table Query** (read-only):
```sql
SELECT id, email, first_name, last_name, software_level, hardware_access
FROM users
WHERE id = $1;
```

**Authentication Flow**:
```
1. Frontend: User logs in via Better Auth
2. Frontend: Receives session ID
3. Frontend: Sends chatbot request with Authorization: Bearer <session_id>
4. Backend: Middleware validates session (get_current_user)
5. Backend: If valid, extracts user_id
6. Backend: Uses user_id for thread ownership checks
```

**No Write Operations**: Backend never modifies users or session tables.

---

### 2. Qdrant Vector Store Integration

**Collection**: `textbook_content` (pre-existing)

**Document Schema** (expected in Qdrant):
```json
{
  "text": "Inverse kinematics is the mathematical process...",
  "chapter": 5,
  "section": "5.3",
  "title": "Robot Kinematics",
  "page": 142,
  "embedding": [0.123, 0.456, ...]  // Cohere embed-v4.0
}
```

**Search Query**:
```python
results = qdrant_client.search(
    collection_name="textbook_content",
    query_text="inverse kinematics in robotics",
    limit=5
)
```

**Expected Response**:
```python
[
    ScoredPoint(
        id="uuid-1",
        score=0.92,
        payload={
            "text": "Inverse kinematics solves for joint angles...",
            "chapter": 5,
            "section": "5.3",
            "title": "Robot Kinematics"
        }
    ),
    # ... 4 more results
]
```

**No Write Operations**: Backend only reads from Qdrant. Vector indexing handled separately.

---

### 3. ChatKit React SDK Integration

**Frontend Setup** (Docusaurus):
```tsx
import { ChatKitProvider, ChatView } from '@openai/chatkit-react';

function ChatbotPage() {
  const sessionId = getBetterAuthSession(); // From Better Auth context

  return (
    <ChatKitProvider
      endpoint="https://api.yourdomain.com/chatkit"
      headers={{
        Authorization: `Bearer ${sessionId}`
      }}
    >
      <ChatView />
    </ChatKitProvider>
  );
}
```

**Event Handling** (automatic):
- `message_start` → Display "Thinking..." indicator
- `action (started)` → Display "🔍 Searching textbook..."
- `action (completed)` → Display "✅ Found content (450ms)"
- `content_delta` → Append text to message
- `message_end` → Mark message complete

**No Custom UI**: ChatKit React SDK handles all rendering.

---

### 4. Google Gemini API Integration

**Access Method**: Via OpenAI-compatible endpoint

**Configuration**:
```python
from openai import OpenAI

client = OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Agent uses this client
agent = Agent(
    model="gemini-2.5-flash",
    client=client,
    ...
)
```

**Rate Limits**: Monitor via Gemini API dashboard (rate limits may differ from OpenAI)

**Cost Optimization**:
- Use Gemini 2.5 Flash (lowest cost tier)
- Context window management (last 10 messages)
- Cache search results temporarily (avoid redundant searches)

---

## Phase 4: Security & Error Handling

### Security Measures

#### 1. Authentication

- **Every request** requires valid Better Auth session
- Session validation via middleware (FastAPI dependency)
- 401 Unauthorized for invalid/expired sessions

#### 2. Authorization

- **Thread ownership checks**: Users can only access their own threads
- Implemented in PostgresStore methods:
  ```python
  async def get_thread(self, thread_id, user_id):
      thread = await db.query(Thread).filter_by(thread_id=thread_id).first()
      if thread.user_id != user_id:
          raise ForbiddenError("Access denied")
      return thread
  ```

#### 3. Input Validation

- **Sanitize text inputs**: Remove potentially harmful content
- **Length limits**:
  - Thread title: 255 chars max
  - Message content: 10,000 chars max (prevent abuse)
- **SQL injection prevention**: Use parameterized queries (SQLAlchemy ORM)

#### 4. Rate Limiting

- **Per-user limits**: 60 requests/minute
- Implementation: slowapi middleware
  ```python
  from slowapi import Limiter
  from slowapi.util import get_remote_address

  limiter = Limiter(key_func=get_remote_address)

  @app.post("/chatkit")
  @limiter.limit("60/minute")
  async def chatkit_endpoint(...):
      pass
  ```

#### 5. Data Encryption

- **In transit**: TLS 1.3 (handled by deployment infrastructure)
- **At rest**: Neon PostgreSQL encrypted by default

---

### Error Handling Strategy

#### Error Categories

**1. Authentication Errors** (401):
- Invalid session ID
- Expired session
- Missing Authorization header

**2. Authorization Errors** (403):
- User doesn't own thread
- User trying to access another user's data

**3. Not Found Errors** (404):
- Thread doesn't exist
- Message doesn't exist

**4. Validation Errors** (422):
- Invalid request payload
- Missing required fields
- Field type mismatches

**5. External Service Errors** (500/503):
- Qdrant timeout
- Gemini API unavailable
- Database connection failure

#### Error Response Format

```json
{
  "error": {
    "code": "ERR_CODE",
    "message": "Human-readable message",
    "details": {
      "field": "value",
      "trace_id": "uuid"
    }
  }
}
```

#### Retry Strategy

**Transient Failures**:
- Qdrant search: 3 retries with exponential backoff (100ms, 200ms, 400ms)
- Gemini API: 2 retries (rate limit errors)
- Database queries: 2 retries (connection errors)

**Non-Retryable**:
- Authentication failures
- Validation errors
- Not found errors

#### Graceful Degradation

**Qdrant Unavailable**:
```python
try:
    results = await qdrant_client.search(...)
except QdrantTimeout:
    # Fallback: Respond without citations
    return "I'm having trouble accessing the textbook right now. Please try again shortly."
```

**Gemini API Unavailable**:
```python
try:
    response = await agent.run(...)
except GeminiAPIError:
    # Return cached/fallback response
    return "The AI service is temporarily unavailable. Please try again in a moment."
```

---

### Logging & Monitoring

#### Structured Logging

**Log Format** (JSON):
```json
{
  "timestamp": "2025-12-25T10:30:00.123Z",
  "level": "INFO",
  "service": "chatbot-backend",
  "event": "message_created",
  "user_id": "uuid-123",
  "thread_id": "uuid-456",
  "message_id": "uuid-789",
  "duration_ms": 450,
  "trace_id": "uuid-abc"
}
```

**Log Events**:
- `session_validated` - Successful auth
- `thread_created` - New thread
- `message_sent` - User message
- `search_executed` - Qdrant search
- `response_generated` - AI response complete
- `error_occurred` - Any error

**Log Levels**:
- `DEBUG`: Development details
- `INFO`: Business events (message created, search executed)
- `WARNING`: Retryable errors, degraded performance
- `ERROR`: Non-retryable errors, failed requests
- `CRITICAL`: System-wide failures

#### Health Check Endpoint

```python
@app.get("/health")
async def health_check():
    """System health check."""
    checks = {
        "status": "healthy",
        "database": await check_database(),
        "qdrant": await check_qdrant(),
        "timestamp": datetime.now(timezone.utc).isoformat()
    }

    if any(v != "healthy" for k, v in checks.items() if k != "timestamp"):
        checks["status"] = "degraded"
        return JSONResponse(content=checks, status_code=503)

    return checks
```

**Monitoring Metrics**:
- Request rate (requests/minute)
- Response latency (p50, p95, p99)
- Error rate (by error type)
- Qdrant search latency
- Gemini API latency
- Database query latency
- Active connections

---

## Phase 5: Testing Strategy

### Unit Tests

**Test Files**:
- `tests/test_store.py` - PostgreSQL store operations
- `tests/test_agent.py` - Agent tool execution
- `tests/test_auth.py` - Session validation
- `tests/test_qdrant.py` - Vector search

**Example Test** (Store):
```python
import pytest
from app.store.postgres_store import PostgresStore

@pytest.mark.asyncio
async def test_create_thread():
    """Test thread creation."""
    store = PostgresStore(db_session)

    thread_id = await store.create_thread(
        user_id="test-user-uuid",
        metadata={"title": "Test Thread"}
    )

    assert thread_id is not None
    assert len(thread_id) == 36  # UUID format

    # Verify thread exists
    thread = await store.get_thread(thread_id)
    assert thread["title"] == "Test Thread"
    assert thread["user_id"] == "test-user-uuid"
```

**Example Test** (Agent Tool):
```python
@pytest.mark.asyncio
async def test_search_textbook_tool():
    """Test search_textbook tool execution."""
    from app.services.agent_service import search_textbook

    result = await search_textbook("inverse kinematics")
    data = json.loads(result)

    assert len(data) <= 5  # Max 5 results
    assert all("content" in item for item in data)
    assert all("chapter" in item for item in data)
    assert all("section" in item for item in data)
```

---

### Integration Tests

**Test Files**:
- `tests/integration/test_chatkit_flow.py` - Full ChatKit request/response
- `tests/integration/test_auth_flow.py` - Better Auth integration
- `tests/integration/test_streaming.py` - SSE streaming

**Example Test** (Full Flow):
```python
@pytest.mark.asyncio
async def test_send_message_with_citation():
    """Test complete message flow with knowledge base search."""
    # Setup: Create thread
    thread_id = await create_test_thread()

    # Send message
    events = []
    async for event in send_message_stream(
        thread_id=thread_id,
        content="Explain inverse kinematics",
        session_id=TEST_SESSION_ID
    ):
        events.append(event)

    # Verify events
    assert any(e["type"] == "message_start" for e in events)
    assert any(e["type"] == "action" and e["name"] == "search_textbook" for e in events)
    assert any(e["type"] == "content_delta" for e in events)
    assert any(e["type"] == "message_end" for e in events)

    # Verify message saved
    messages = await get_messages(thread_id)
    assert len(messages) == 2  # User + assistant
    assert "[^1]" in messages[1]["content"]  # Has citation
```

---

### Performance Tests

**Targets** (from spec):
- Session validation: < 100ms
- Qdrant search: < 500ms (95th percentile)
- AI response start: < 3 seconds
- Message history load: < 200ms (50 messages)

**Load Test** (10,000 concurrent users):
```python
import asyncio
from locust import HttpUser, task, between

class ChatbotUser(HttpUser):
    wait_time = between(1, 5)

    @task
    def send_message(self):
        """Simulate user sending message."""
        self.client.post(
            "/chatkit",
            json={
                "action": "send_message",
                "thread_id": self.thread_id,
                "content": "What is inverse kinematics?"
            },
            headers={"Authorization": f"Bearer {self.session_id}"}
        )
```

---

## Phase 6: Deployment & Operations

### Environment Variables

**`.env` File**:
```bash
# Database
DATABASE_URL=postgresql+asyncpg://user:pass@host/db?ssl=require

# Better Auth (read-only access)
# Uses same DATABASE_URL

# Qdrant
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-api-key

# Gemini AI
GEMINI_API_KEY=your-gemini-key

# Application
ENVIRONMENT=production
LOG_LEVEL=INFO
CORS_ORIGINS=https://yourdomain.com

# Security
JWT_SECRET=your-secret-key  # For internal use (not Better Auth)
RATE_LIMIT_PER_MINUTE=60
```

### Database Migrations

**Alembic Migration** (`alembic/versions/001_initial_schema.py`):
```python
"""Initial schema: threads and messages tables

Revision ID: 001
Revises:
Create Date: 2025-12-25
"""

def upgrade():
    # Create threads table
    op.create_table(
        'threads',
        sa.Column('thread_id', postgresql.UUID(), primary_key=True),
        sa.Column('user_id', postgresql.UUID(), nullable=False),
        sa.Column('title', sa.String(255), nullable=True),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.func.now()),
        sa.Column('updated_at', sa.TIMESTAMP(timezone=True), server_default=sa.func.now()),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ondelete='CASCADE')
    )

    op.create_index('idx_threads_user_id', 'threads', ['user_id'])
    op.create_index('idx_threads_created_at', 'threads', ['created_at'])

    # Create messages table
    op.create_table(
        'messages',
        sa.Column('message_id', postgresql.UUID(), primary_key=True),
        sa.Column('thread_id', postgresql.UUID(), nullable=False),
        sa.Column('role', sa.String(20), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.func.now()),
        sa.Column('sequence_number', sa.Integer(), nullable=False),
        sa.ForeignKeyConstraint(['thread_id'], ['threads.thread_id'], ondelete='CASCADE'),
        sa.UniqueConstraint('thread_id', 'sequence_number')
    )

    op.create_index('idx_messages_thread_id', 'messages', ['thread_id'])

def downgrade():
    op.drop_table('messages')
    op.drop_table('threads')
```

**Run Migration**:
```bash
alembic upgrade head
```

### Deployment Steps

1. **Pre-deployment**:
   ```bash
   # Install dependencies
   pip install -r requirements.txt

   # Run database migrations
   alembic upgrade head

   # Verify environment variables
   python -c "from app.config import settings; print(settings)"
   ```

2. **Start Application**:
   ```bash
   uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4
   ```

3. **Health Check**:
   ```bash
   curl https://api.yourdomain.com/health
   ```

4. **Monitor Logs**:
   ```bash
   # Structured JSON logs
   tail -f logs/chatbot.log | jq
   ```

### Scaling Strategy

**Horizontal Scaling**:
- Stateless backend (no session storage in memory)
- Multiple instances behind load balancer
- Database connection pooling (max 20 connections per instance)

**Database Optimization**:
- Indexes on frequently queried columns
- Connection pooling (SQLAlchemy async pool)
- Read replicas for message history queries (optional)

**Caching** (optional future enhancement):
- Redis cache for frequently accessed threads
- Cache invalidation on updates

---

## Phase 7: Success Metrics & Monitoring

### Key Performance Indicators (KPIs)

**User-Facing**:
- ✅ 95% of queries receive responses within 3 seconds
- ✅ 80% user satisfaction (4+/5 stars)
- ✅ 70% return users (multiple sessions)
- ✅ 8+ messages average thread length

**System Performance**:
- ✅ 10,000 concurrent users without degradation
- ✅ 99.5% uptime (30-day rolling window)
- ✅ < 500ms knowledge retrieval (95th percentile)
- ✅ < 100ms session validation

**Business Outcomes**:
- ✅ 40% reduction in support tickets
- ✅ 60% weekly active users
- ✅ 25% increase in session duration

### Monitoring Dashboard

**Metrics to Track**:
1. **Request Metrics**:
   - Requests per minute (total, by action type)
   - Response latency (p50, p95, p99)
   - Error rate (by error code)

2. **External Service Metrics**:
   - Qdrant search latency
   - Qdrant search success rate
   - Gemini API latency
   - Gemini API success rate
   - Database query latency

3. **Business Metrics**:
   - Active users (daily, weekly)
   - Messages sent per user
   - Average thread length
   - Citation usage rate

4. **Resource Metrics**:
   - CPU usage
   - Memory usage
   - Database connections
   - Network I/O

### Alerts

**Critical Alerts** (immediate action):
- Error rate > 5% (5 minutes)
- Response latency p95 > 5 seconds (5 minutes)
- Database connection failures
- Qdrant unreachable
- Gemini API quota exceeded

**Warning Alerts** (investigate soon):
- Error rate > 1% (15 minutes)
- Response latency p95 > 3 seconds (15 minutes)
- Memory usage > 80%
- Database connection pool > 80% utilized

---

## Phase 8: Risks & Mitigation

### Technical Risks

#### 1. Gemini API Rate Limits

**Risk**: Gemini API may have different rate limits than OpenAI, causing throttling.

**Mitigation**:
- Monitor rate limit headers
- Implement exponential backoff
- Queue requests during peak load
- Consider upgrading API tier if needed

#### 2. Qdrant Search Performance

**Risk**: Vector search may exceed 500ms target under load.

**Mitigation**:
- Optimize Qdrant collection indexes
- Reduce search limit from 5 to 3 if needed
- Cache recent search results (Redis)
- Monitor p95 latency closely

#### 3. Database Connection Pooling

**Risk**: High concurrent load may exhaust database connections.

**Mitigation**:
- Connection pool size: 20 per instance
- Connection timeout: 30 seconds
- Implement connection retry logic
- Scale horizontally (more instances)

#### 4. Better Auth Session Table Schema Changes

**Risk**: Better Auth may update session table schema in future versions.

**Mitigation**:
- Use raw SQL queries (not ORM) for session validation
- Version-lock Better Auth dependencies in frontend
- Monitor Better Auth release notes
- Test session validation after Better Auth upgrades

---

### Operational Risks

#### 1. Knowledge Base Staleness

**Risk**: Textbook content updates not reflected in Qdrant.

**Mitigation**:
- Document re-indexing process
- Automated re-indexing pipeline (separate from chatbot backend)
- Version tracking for knowledge base
- Clear communication when knowledge base is updated

#### 2. Citation Link Breakage

**Risk**: Frontend URLs change, breaking citation links.

**Mitigation**:
- Use stable URL patterns (e.g., `/book/chapter-5/section-3`)
- Document URL schema
- Implement URL redirect layer if patterns change
- Include raw chapter/section in citations as fallback

#### 3. Cost Overruns

**Risk**: Gemini API costs exceed budget.

**Mitigation**:
- Monitor API usage daily
- Set usage quotas per user (rate limiting)
- Implement cost tracking dashboard
- Cache common responses (future enhancement)

---

## Implementation Timeline

**Phase 0 (Research)**: Complete ✅
**Phase 1 (Data Model & Contracts)**: 2-3 days
**Phase 2 (Component Implementation)**: 5-7 days
**Phase 3 (Integration)**: 2-3 days
**Phase 4 (Security & Error Handling)**: 2 days
**Phase 5 (Testing)**: 3-4 days
**Phase 6 (Deployment)**: 1-2 days

**Total Estimate**: 15-21 days for full implementation

---

## Next Steps

1. **Review Plan**: User approval of architecture and design decisions
2. **Generate Tasks**: Run `/sp.tasks` to create implementation checklist
3. **Setup Environment**: Create `.env` file with credentials
4. **Database Migration**: Run Alembic migration for threads/messages tables
5. **Implement Core Components**: Start with PostgresStore, then Agent Service
6. **Integration Testing**: Test with Better Auth and Qdrant
7. **Deploy to Staging**: Verify all integrations work end-to-end
8. **Production Deployment**: Deploy with monitoring enabled

---

**Plan Status**: ✅ **Ready for Task Generation**
**Next Command**: `/sp.tasks` to generate implementation checklist
