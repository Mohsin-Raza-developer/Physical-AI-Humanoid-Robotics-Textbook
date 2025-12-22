# Data Model
## RAG-Powered Chatbot Backend API

**Feature**: 007-chatbot-backend
**Date**: 2025-12-20
**Status**: Complete

---

## Overview

This document defines all data structures for the RAG Chatbot Backend API, including request/response models, domain entities, database schemas, and message formats.

---

## 1. API Request/Response Models

### 1.1 ChatRequest

**Purpose**: Incoming chat message from user

```python
from pydantic import BaseModel, Field, validator

class ChatRequest(BaseModel):
    """Request model for POST /api/chat endpoint"""

    message: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question or message"
    )

    user_id: str = Field(
        ...,
        regex="^[a-zA-Z0-9_-]+$",
        description="Authenticated user identifier"
    )

    session_id: str | None = Field(
        None,
        description="Optional session ID for continuing conversation"
    )

    @validator('message')
    def validate_message_content(cls, v):
        if not v.strip():
            raise ValueError("Message cannot be empty or whitespace only")
        return v.strip()

    class Config:
        json_schema_extra = {
            "example": {
                "message": "What is ROS 2 architecture?",
                "user_id": "user_12345",
                "session_id": "sess_abc123"
            }
        }
```

**Validation Rules**:
- `message`: 1-2000 characters, cannot be empty/whitespace
- `user_id`: Alphanumeric with underscores/hyphens only
- `session_id`: Optional, used for conversation continuity

---

### 1.2 ChatResponse

**Purpose**: Response from chatbot with answer and metadata

```python
from pydantic import BaseModel, Field

class Citation(BaseModel):
    """Source citation with clickable link"""

    chapter_title: str = Field(..., description="Display name of source chapter")
    doc_url: str = Field(..., description="Docusaurus URL path")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Search relevance (0-1)")

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_title": "ROS 2 Architecture Overview",
                "doc_url": "/docs/module-1/week-3/ros2-architecture",
                "relevance_score": 0.92
            }
        }


class ChatResponse(BaseModel):
    """Response model for POST /api/chat endpoint"""

    response: str = Field(..., description="Agent's answer with inline citations")
    session_id: str = Field(..., description="Session ID for conversation continuity")
    citations: list[Citation] = Field(default_factory=list, description="Source references")
    confidence_score: float | None = Field(None, ge=0.0, le=1.0, description="Response confidence (0-1)")
    processing_time_ms: int = Field(..., description="Total processing time in milliseconds")
    token_count: int = Field(..., description="Total tokens in conversation context")

    class Config:
        json_schema_extra = {
            "example": {
                "response": "ROS 2 (Robot Operating System 2) is a middleware framework... [ROS 2 Architecture](/docs/module-1/week-3/ros2-architecture)",
                "session_id": "sess_abc123",
                "citations": [
                    {
                        "chapter_title": "ROS 2 Architecture Overview",
                        "doc_url": "/docs/module-1/week-3/ros2-architecture",
                        "relevance_score": 0.92
                    }
                ],
                "confidence_score": 0.88,
                "processing_time_ms": 1245,
                "token_count": 523
            }
        }
```

**Response Fields**:
- `response`: Main answer text with inline markdown citations
- `session_id`: For client to continue conversation
- `citations`: List of source references (can be empty)
- `confidence_score`: Optional quality metric
- `processing_time_ms`: Performance tracking
- `token_count`: Context window usage

---

### 1.3 ErrorResponse

**Purpose**: Structured error messages with codes

```python
from pydantic import BaseModel, Field

class ErrorResponse(BaseModel):
    """Standard error response format"""

    error: str = Field(..., description="User-friendly error message")
    code: str = Field(..., description="Error code for support/debugging")
    details: dict[str, any] | None = Field(None, description="Optional additional context")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "We're having trouble connecting to our knowledge base. Please try again.",
                "code": "ERR_DB_001",
                "details": {"retry_after_seconds": 30}
            }
        }
```

**Error Code Categories**:
- `ERR_DB_001-099`: Database errors (Neon, Qdrant)
- `ERR_AUTH_001-099`: Authentication failures
- `ERR_AGENT_001-099`: Agent/LLM errors
- `ERR_TOOL_001-099`: Knowledge retrieval errors
- `ERR_VAL_001-099`: Input validation errors

---

### 1.4 SafetyCheckOutput (Guardrail Agent)

**Purpose**: Structured output from input guardrail agent for safety/relevance validation

**CRITICAL**: This is the `output_type` for the guardrail agent that runs via `@input_guardrail` decorator

```python
from pydantic import BaseModel, Field

class SafetyCheckOutput(BaseModel):
    """Output model for safety guardrail agent validation"""

    is_safe: bool = Field(
        ...,
        description="Whether query contains safe, appropriate content"
    )

    is_relevant: bool = Field(
        ...,
        description="Whether query is relevant to robotics textbook content"
    )

    reason: str = Field(
        ...,
        description="Explanation of validation decision"
    )

    class Config:
        json_schema_extra = {
            "examples": [
                {
                    "is_safe": True,
                    "is_relevant": True,
                    "reason": "Query asks about ROS 2 architecture - safe and relevant to textbook"
                },
                {
                    "is_safe": True,
                    "is_relevant": False,
                    "reason": "Query asks about weather - safe but not relevant to robotics course"
                },
                {
                    "is_safe": False,
                    "is_relevant": False,
                    "reason": "Query contains inappropriate content"
                }
            ]
        }
```

**Usage in Guardrail Agent**:

```python
from agents import Agent, GuardrailFunctionOutput, RunContextWrapper, Runner, input_guardrail

# 1. Create guardrail agent with SafetyCheckOutput as output_type
guardrail_agent = Agent(
    name="Safety Guardrail",
    instructions="Check if user query is safe and relevant to robotics textbook content. Return is_safe=True if content is appropriate, is_relevant=True if about robotics/course topics.",
    output_type=SafetyCheckOutput  # ← CRITICAL: Structured output
)

# 2. Use in @input_guardrail decorator
@input_guardrail
async def safety_guardrail(
    ctx: RunContextWrapper[None],
    agent: Agent,
    input: str | list[TResponseInputItem]
) -> GuardrailFunctionOutput:
    # Run guardrail agent
    result = await Runner.run(guardrail_agent, input, context=ctx.context)

    # result.final_output is SafetyCheckOutput instance
    safety_output: SafetyCheckOutput = result.final_output

    # Return GuardrailFunctionOutput
    return GuardrailFunctionOutput(
        output_info=safety_output,  # Store for debugging/logging
        tripwire_triggered=not (safety_output.is_safe and safety_output.is_relevant)
        # ↑ If unsafe OR irrelevant, trigger tripwire → InputGuardrailTripwireTriggered exception
    )

# 3. Main agent uses the guardrail
main_agent = Agent(
    name="Robotics Tutor",
    instructions="You are a helpful robotics tutor...",
    tools=[search_knowledge_base],
    input_guardrails=[safety_guardrail]  # ← Guardrail runs on every input
)
```

**Exception Handling**:

```python
from agents import InputGuardrailTripwireTriggered

try:
    result = await Runner.run(main_agent, user_message, run_config=config)
    assistant_response = result.final_output

except InputGuardrailTripwireTriggered as e:
    # Query blocked by guardrail
    # Access validation details via e.guardrail_result.output_info
    safety_output: SafetyCheckOutput = e.guardrail_result.output_info

    if not safety_output.is_safe:
        raise ChatbotError(
            message="Your question contains inappropriate content. Please rephrase.",
            code="ERR_VAL_004"
        )
    elif not safety_output.is_relevant:
        raise ChatbotError(
            message="I can only answer questions about the robotics textbook. Please ask about course content.",
            code="ERR_VAL_005"
        )
```

---

## 2. Domain Entities

### 2.1 ChatMessage

**Purpose**: Single message in conversation history

```python
from pydantic import BaseModel, Field
from enum import Enum

class MessageRole(str, Enum):
    """Message sender role"""
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"


class ChatMessage(BaseModel):
    """Individual message in messages array"""

    role: MessageRole = Field(..., description="Message sender (system/user/assistant)")
    content: str = Field(..., min_length=1, description="Message text content")
    timestamp: datetime | None = Field(None, description="Message creation time")

    class Config:
        json_schema_extra = {
            "examples": [
                {
                    "role": "system",
                    "content": "You are a helpful robotics tutor. Student: John Doe, Level: intermediate",
                    "timestamp": "2025-12-20T14:30:00Z"
                },
                {
                    "role": "user",
                    "content": "What is ROS 2?",
                    "timestamp": "2025-12-20T14:30:15Z"
                },
                {
                    "role": "assistant",
                    "content": "ROS 2 is... [Citation](/docs/module-1/ros2)",
                    "timestamp": "2025-12-20T14:30:18Z"
                }
            ]
        }
```

**Message Structure**:
- `role`: system (profile context), user (questions), assistant (answers)
- `content`: Text content (with markdown citations for assistant)
- `timestamp`: Optional for audit trail

---

### 2.2 ConversationSession

**Purpose**: Active dialogue session with user

```python
from pydantic import BaseModel, Field
from datetime import datetime

class ConversationSession(BaseModel):
    """Active conversation session state"""

    session_id: str = Field(..., description="Unique session identifier")
    user_id: str = Field(..., description="User who owns this session")
    messages: list[ChatMessage] = Field(default_factory=list, description="Conversation history")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Session start time")
    last_activity: datetime = Field(default_factory=datetime.utcnow, description="Last message timestamp")
    token_count: int = Field(default=0, description="Total tokens in context window")
    is_active: bool = Field(default=True, description="Session active status")

    def add_message(self, message: ChatMessage) -> None:
        """Add message to conversation history"""
        self.messages.append(message)
        self.last_activity = datetime.utcnow()

    def get_messages_for_agent(self) -> list[dict]:
        """Convert to format expected by OpenAI SDK"""
        return [{"role": msg.role.value, "content": msg.content} for msg in self.messages]

    def estimate_tokens(self) -> int:
        """Rough token count estimation (4 chars ≈ 1 token)"""
        total_chars = sum(len(msg.content) for msg in self.messages)
        return total_chars // 4

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "sess_abc123",
                "user_id": "user_12345",
                "messages": [...],
                "created_at": "2025-12-20T14:30:00Z",
                "last_activity": "2025-12-20T14:35:42Z",
                "token_count": 523,
                "is_active": True
            }
        }
```

**Session Lifecycle**:
1. **Start**: Fetch user profile → Create system message → Initialize session
2. **Active**: Add user/assistant messages, track token count
3. **End**: Mark inactive when user closes or starts new session

---

### 2.3 UserProfile

**Purpose**: Student profile data from Neon DB

```python
from pydantic import BaseModel, Field
from enum import Enum

class SoftwareLevel(str, Enum):
    """Student programming experience level"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class UserProfile(BaseModel):
    """User profile data from Neon database"""

    user_id: str = Field(..., description="User identifier")
    first_name: str = Field(..., description="User's first name")
    last_name: str = Field(..., description="User's last name")
    software_level: SoftwareLevel = Field(..., description="Programming experience level")
    email: str | None = Field(None, description="User email (optional)")

    def to_system_message(self) -> str:
        """Format profile as system message for agent"""
        return (
            f"You are a helpful robotics tutor. "
            f"Student: {self.first_name} {self.last_name}, "
            f"Level: {self.software_level.value}. "
            f"Tailor your responses to their experience level."
        )

    class Config:
        json_schema_extra = {
            "example": {
                "user_id": "user_12345",
                "first_name": "John",
                "last_name": "Doe",
                "software_level": "intermediate",
                "email": "john.doe@example.com"
            }
        }
```

**Profile Usage**:
- Fetched **once** at session start from Neon DB
- Formatted into system message (messages[0])
- Stored in session for entire conversation
- Enables personalized response tailoring

---

### 2.4 KnowledgeChunk

**Purpose**: Retrieved content from Qdrant vector database

```python
from pydantic import BaseModel, Field

class KnowledgeChunk(BaseModel):
    """Search result from Qdrant vector database"""

    content: str = Field(..., description="Textbook content snippet")
    source_file: str = Field(..., description="Source file path in repository")
    chapter_title: str = Field(..., description="Chapter/section title")
    module: str | None = Field(None, description="Module identifier (e.g., 'module-1')")
    week: str | None = Field(None, description="Week identifier (e.g., 'week-3')")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Search similarity score")

    def to_docusaurus_url(self) -> str:
        """Convert source file path to Docusaurus URL"""
        # Example: content/docs/module-1/week-3/ros2.md → /docs/module-1/week-3/ros2
        path = self.source_file.replace("content/docs/", "/docs/")
        path = path.replace(".md", "")
        return path

    def to_citation(self) -> Citation:
        """Convert to API citation format"""
        return Citation(
            chapter_title=self.chapter_title,
            doc_url=self.to_docusaurus_url(),
            relevance_score=self.relevance_score
        )

    class Config:
        json_schema_extra = {
            "example": {
                "content": "ROS 2 uses a DDS (Data Distribution Service) middleware layer...",
                "source_file": "content/docs/module-1/week-3/ros2-architecture.md",
                "chapter_title": "ROS 2 Architecture Overview",
                "module": "module-1",
                "week": "week-3",
                "relevance_score": 0.92
            }
        }
```

**Search Result Processing**:
1. Embed user query (Cohere)
2. Search Qdrant (top 5 results)
3. Convert to KnowledgeChunk objects
4. Format as context string with citations
5. Pass to agent as tool output

---

## 3. Database Schemas

### 3.1 Neon PostgreSQL Schema (users table)

**Purpose**: Store user profile data

```sql
CREATE TABLE users (
    id VARCHAR(255) PRIMARY KEY,
    first_name VARCHAR(100) NOT NULL,
    last_name VARCHAR(100) NOT NULL,
    software_level VARCHAR(20) NOT NULL CHECK (software_level IN ('beginner', 'intermediate', 'advanced')),
    email VARCHAR(255) UNIQUE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);
```

**Field Descriptions**:
- `id`: User identifier (from auth system)
- `first_name`, `last_name`: For personalization in system message
- `software_level`: Enum (beginner/intermediate/advanced)
- `email`: Optional, for account management
- `created_at`, `updated_at`: Audit timestamps

**Query Pattern** (fetch profile):
```sql
SELECT first_name, last_name, software_level
FROM users
WHERE id = $1;
```

**Performance**:
- Primary key index on `id` for fast lookups
- Expected latency: <50ms per query
- Cached in session (1 query per session, not per request)

---

### 3.2 Qdrant Collection Schema

**Purpose**: Vector embeddings of textbook content

**Collection Name**: `robotics_textbook_v1`

**Vector Configuration**:
```python
{
    "size": 1536,  # Cohere embed-v4.0 dimension
    "distance": "Cosine"  # Similarity metric
}
```

**Payload Schema** (metadata):
```python
{
    "content": str,  # Textbook content snippet
    "source_file": str,  # File path in repo
    "chapter_title": str,  # Chapter/section title
    "module": str | None,  # Module identifier
    "week": str | None,  # Week identifier
    "chunk_id": str  # Unique chunk identifier
}
```

**Search Query**:
```python
client.search(
    collection_name="robotics_textbook_v1",
    query_vector=embedding,  # 1536-dim from Cohere
    limit=5,
    score_threshold=0.7  # Min relevance score
)
```

**Performance**:
- HNSW index for fast approximate search
- Expected latency: <100ms per search
- Top 5 results per query

---

## 4. Message Format (Agent SDK)

### 4.1 Messages Array Structure

**Purpose**: Conversation history format for OpenAI Agents SDK

```python
messages: list[dict[str, str]] = [
    {
        "role": "system",
        "content": "You are a helpful robotics tutor. Student: John Doe, Level: intermediate. Tailor your responses to their experience level."
    },
    {
        "role": "user",
        "content": "What is ROS 2?"
    },
    {
        "role": "assistant",
        "content": "ROS 2 (Robot Operating System 2) is a middleware framework for robotic applications... [ROS 2 Architecture](/docs/module-1/week-3/ros2-architecture)"
    },
    {
        "role": "user",
        "content": "How do I install it?"
    },
    {
        "role": "assistant",
        "content": "To install ROS 2, follow these steps... [Installation Guide](/docs/module-1/week-1/installation)"
    }
]
```

**Rules**:
- **messages[0]**: Always system message with user profile
- **Alternating**: user → assistant → user → assistant...
- **Citations**: Inline markdown links in assistant messages
- **Context Window**: Stay under 8,000 tokens

---

### 4.2 Tool Call Format

**Purpose**: Knowledge base search tool interface

**Tool Definition**:
```python
@function_tool
def search_knowledge_base(query: str) -> str:
    """
    Searches the robotics textbook knowledge base for relevant content.

    Args:
        query: The search query (question or topic to find information about)

    Returns:
        Formatted context string with relevant textbook sections and source citations
    """
    # Implementation in tool layer
    pass
```

**Tool Output Format**:
```
ROS 2 uses a DDS (Data Distribution Service) middleware layer for communication between nodes...

Source: [ROS 2 Architecture Overview](/docs/module-1/week-3/ros2-architecture)

---

ROS 2 supports multiple DDS implementations including Fast-DDS and Cyclone DDS...

Source: [DDS Middleware](/docs/module-1/week-4/dds-middleware)
```

**Agent Integration**:
- Agent calls tool when it needs textbook context
- Tool returns formatted string with citations
- Agent incorporates into final response

---

## 5. State Management

### 5.1 Session Storage (In-Memory)

**Purpose**: Temporary session state during active conversation

```python
from typing import Dict

# Global session store (replace with Redis for production)
active_sessions: Dict[str, ConversationSession] = {}

def get_or_create_session(user_id: str, session_id: str | None = None) -> ConversationSession:
    """Get existing session or create new one"""
    if session_id and session_id in active_sessions:
        session = active_sessions[session_id]
        if session.user_id == user_id and session.is_active:
            return session

    # Create new session
    new_session = ConversationSession(
        session_id=generate_session_id(),
        user_id=user_id,
        messages=[]
    )
    active_sessions[new_session.session_id] = new_session
    return new_session

def cleanup_inactive_sessions():
    """Remove sessions inactive for >1 hour"""
    cutoff = datetime.utcnow() - timedelta(hours=1)
    to_remove = [
        sid for sid, session in active_sessions.items()
        if session.last_activity < cutoff
    ]
    for sid in to_remove:
        del active_sessions[sid]
```

**Session Lifecycle**:
1. **New User**: Create session, fetch profile, add system message
2. **Existing Session**: Retrieve from store, validate user_id
3. **Inactive Timeout**: Auto-cleanup after 1 hour
4. **User Logout**: Mark inactive, schedule cleanup

---

### 5.2 Profile Caching Strategy

**Purpose**: Minimize Neon DB queries

**Cache Pattern**:
```python
# Profile stored in session messages[0]
def initialize_session_with_profile(user_id: str) -> ConversationSession:
    """Create new session with user profile"""

    # Fetch profile from Neon DB (ONLY ONCE)
    profile = fetch_user_profile(user_id)

    # Format as system message
    system_message = ChatMessage(
        role=MessageRole.SYSTEM,
        content=profile.to_system_message()
    )

    # Create session with profile as messages[0]
    session = ConversationSession(
        session_id=generate_session_id(),
        user_id=user_id,
        messages=[system_message]
    )

    return session
```

**Performance Impact**:
- **Before Optimization**: N DB queries per session (1 per turn)
- **After Optimization**: 1 DB query per session (90% reduction)
- **Latency Savings**: ~50ms per turn

---

## 6. Validation Rules

### 6.1 Input Validation

```python
from pydantic import BaseModel, Field, validator

class ValidationRules:
    """Centralized validation rules"""

    # Message constraints
    MESSAGE_MIN_LENGTH = 1
    MESSAGE_MAX_LENGTH = 2000

    # Session constraints
    MAX_MESSAGES_PER_SESSION = 100
    MAX_TOKENS_PER_SESSION = 8000

    # User ID format
    USER_ID_PATTERN = r"^[a-zA-Z0-9_-]+$"

    # Session ID format
    SESSION_ID_PATTERN = r"^sess_[a-zA-Z0-9]+$"

    @staticmethod
    def validate_message_length(message: str) -> None:
        """Validate message is within length limits"""
        if len(message) < ValidationRules.MESSAGE_MIN_LENGTH:
            raise ValueError("Message too short")
        if len(message) > ValidationRules.MESSAGE_MAX_LENGTH:
            raise ValueError(f"Message exceeds {ValidationRules.MESSAGE_MAX_LENGTH} characters")

    @staticmethod
    def validate_token_limit(session: ConversationSession) -> None:
        """Validate session hasn't exceeded token limit"""
        if session.estimate_tokens() > ValidationRules.MAX_TOKENS_PER_SESSION:
            raise ValueError("Conversation too long, please start a new session")
```

---

### 6.2 Data Quality Rules

**User Profile**:
- `first_name`, `last_name`: Required, non-empty
- `software_level`: Must be beginner/intermediate/advanced
- `email`: Valid email format (if provided)

**Chat Message**:
- `role`: Must be system/user/assistant
- `content`: Non-empty, trimmed whitespace
- `timestamp`: UTC timezone

**Search Results**:
- `relevance_score`: 0.0 to 1.0
- `source_file`: Valid file path format
- `chapter_title`: Non-empty string

---

## 7. Data Flow Diagram

```
User Request (ChatRequest)
    ↓
Validate Input (Pydantic)
    ↓
Get/Create Session (ConversationSession)
    ├─ New Session?
    │   ├─ Fetch Profile (Neon DB) → UserProfile
    │   └─ Create System Message → ChatMessage (messages[0])
    └─ Existing Session?
        └─ Load from Memory
    ↓
Add User Message → ChatMessage (messages[N])
    ↓
Input Guardrail Agent
    ├─ Blocked? → ErrorResponse
    └─ Approved? ↓
Main Chatbot Agent
    ├─ Needs Context? → search_knowledge_base Tool
    │   ├─ Embed Query (Cohere) → [1536-dim vector]
    │   ├─ Search Qdrant → [KnowledgeChunk, ...]
    │   └─ Format Citations → str
    └─ Generate Response
    ↓
Add Assistant Message → ChatMessage (messages[N+1])
    ↓
Build ChatResponse (with citations, metadata)
    ↓
Return to User
```

---

## Conclusion

This data model provides a complete, type-safe structure for the RAG Chatbot Backend API. All entities are designed for:

- **Performance**: Optimized DB queries, efficient caching
- **Type Safety**: Pydantic validation, Python type hints
- **Maintainability**: Clear structure, documented fields
- **Extensibility**: Easy to add new fields, metadata

**Next Steps**: Generate API contracts (OpenAPI spec)
