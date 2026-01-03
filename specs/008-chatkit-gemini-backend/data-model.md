# Data Model Specification

**Feature**: ChatKit-Integrated Robotics Chatbot Backend
**Feature ID**: 008-chatkit-gemini-backend
**Created**: 2025-12-25
**Status**: Draft

---

## Overview

This document defines the complete data model for the robotics chatbot backend. The system uses **PostgreSQL** (Neon) for persistent storage and implements **ChatKit's Store interface pattern** for conversation management.

### Design Principles

1. **Text-Only Architecture**: No file/attachment storage (permanently out of scope)
2. **Session-Based Auth**: User identity derived from Better Auth sessions (read-only)
3. **Conversation-Centric**: Threads and messages are the core domain entities
4. **Audit Trail**: All entities include creation and update timestamps
5. **Soft Deletes**: Deleted threads cascade to messages (hard delete)

---

## Entity Relationship Diagram

```
┌─────────────────────────┐
│ User (Better Auth)      │
│ ─────────────────────── │
│ • id (PK)               │
│ • email                 │
│ • name                  │
│ • software_level        │
│ • hardware_access       │
└───────┬─────────────────┘
        │ 1
        │ owns
        │ N
┌───────▼─────────────────┐
│ Thread                  │
│ ─────────────────────── │
│ • thread_id (PK)        │
│ • user_id (FK)          │
│ • title                 │
│ • metadata (JSONB)      │
│ • created_at            │
│ • updated_at            │
└───────┬─────────────────┘
        │ 1
        │ contains
        │ N
┌───────▼─────────────────┐
│ Message                 │
│ ─────────────────────── │
│ • message_id (PK)       │
│ • thread_id (FK)        │
│ • role                  │
│ • content               │
│ • sequence_number       │
│ • created_at            │
└─────────────────────────┘
```

---

## Entity Definitions

### 1. Thread

**Purpose**: Represents a conversation thread between a user and the chatbot.

**Table Name**: `threads`

**Attributes**:

| Column        | Type                  | Constraints                     | Description                                      |
|---------------|----------------------|--------------------------------|--------------------------------------------------|
| thread_id     | UUID                 | PRIMARY KEY, NOT NULL          | Unique thread identifier                         |
| user_id       | UUID                 | NOT NULL, FOREIGN KEY (users)  | Owner of the thread (Better Auth user)           |
| title         | VARCHAR(255)         | NULLABLE                       | Optional thread title (auto-generated or user-set) |
| metadata      | JSONB                | NOT NULL, DEFAULT '{}'         | ChatKit metadata (tags, custom fields)           |
| created_at    | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW()    | Thread creation timestamp                        |
| updated_at    | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW()    | Last message timestamp (auto-updated)            |

**Indexes**:
- `idx_threads_user_id` on `user_id` (for listing user's threads)
- `idx_threads_updated_at` on `updated_at DESC` (for sorting by recent activity)
- `idx_threads_created_at` on `created_at DESC` (for pagination)

**Foreign Keys**:
- `user_id` REFERENCES `user(id)` ON DELETE CASCADE

**Validation Rules**:
- `thread_id` must be a valid UUIDv4
- `user_id` must exist in Better Auth's `user` table
- `title` length: 0-255 characters (NULL allowed)
- `metadata` must be valid JSON object (not array or primitive)
- `created_at` must be <= `updated_at`

**State Transitions**:
- **Created**: New thread with no messages
- **Active**: Thread with >= 1 message
- **Updated**: `updated_at` changes when new message added
- **Deleted**: Hard delete with CASCADE to messages

**Business Rules**:
1. Users can only access threads they own (`user_id` match)
2. Thread title auto-generated from first user message if NULL
3. `updated_at` automatically updated when messages are added
4. Threads with zero messages can exist (user opened thread, didn't send message yet)

**ChatKit Mapping**:
```python
from chatkit.models import ThreadMetadata

# Thread → ThreadMetadata
thread_metadata = ThreadMetadata(
    id=thread.thread_id,
    created_at=thread.created_at.isoformat(),
    updated_at=thread.updated_at.isoformat(),
    metadata={
        "title": thread.title,
        **thread.metadata  # JSONB column
    }
)
```

---

### 2. Message

**Purpose**: Represents a single message within a conversation thread (user or assistant).

**Table Name**: `messages`

**Attributes**:

| Column          | Type                     | Constraints                     | Description                                      |
|-----------------|-------------------------|---------------------------------|--------------------------------------------------|
| message_id      | UUID                    | PRIMARY KEY, NOT NULL           | Unique message identifier                        |
| thread_id       | UUID                    | NOT NULL, FOREIGN KEY (threads) | Parent thread                                    |
| role            | VARCHAR(20)             | NOT NULL, CHECK IN ('user', 'assistant') | Message sender role                              |
| content         | TEXT                    | NOT NULL                        | Message text content                             |
| sequence_number | INTEGER                 | NOT NULL                        | Message order within thread (1-indexed)          |
| created_at      | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW()        | Message creation timestamp                       |

**Indexes**:
- `idx_messages_thread_id` on `thread_id` (for retrieving thread messages)
- `idx_messages_sequence` on `(thread_id, sequence_number)` (UNIQUE, for ordering)
- `idx_messages_created_at` on `created_at` (for temporal queries)

**Foreign Keys**:
- `thread_id` REFERENCES `threads(thread_id)` ON DELETE CASCADE

**Validation Rules**:
- `message_id` must be a valid UUIDv4
- `thread_id` must exist in `threads` table
- `role` must be exactly 'user' or 'assistant' (case-sensitive)
- `content` must not be empty string (length >= 1)
- `content` maximum length: 100,000 characters (safety limit)
- `sequence_number` must be >= 1
- `sequence_number` must be unique per thread (enforced by unique index)

**State Transitions**:
- **Created**: New message added to thread
- **Immutable**: Messages are never updated (append-only log)
- **Deleted**: Hard delete via CASCADE when thread deleted

**Business Rules**:
1. Messages are **append-only** (no updates after creation)
2. `sequence_number` auto-increments per thread (1, 2, 3, ...)
3. First message in thread sets thread's `updated_at`
4. Messages alternate between 'user' and 'assistant' roles (best practice, not enforced)
5. Text-only content (no file references or attachments)

**ChatKit Mapping**:
```python
from chatkit.models import ThreadItem, UserMessageItem, AssistantMessageItem

# Message → ThreadItem
if message.role == 'user':
    item = UserMessageItem(
        id=message.message_id,
        thread_id=message.thread_id,
        created_at=message.created_at.isoformat(),
        text=message.content
    )
elif message.role == 'assistant':
    item = AssistantMessageItem(
        id=message.message_id,
        thread_id=message.thread_id,
        created_at=message.created_at.isoformat(),
        text=message.content
    )
```

---

### 3. User (Read-Only)

**Purpose**: User accounts managed by Better Auth (referenced for foreign keys only).

**Table Name**: `user` (Better Auth managed)

**Attributes**:

| Column           | Type         | Constraints       | Description                                      |
|------------------|-------------|-------------------|--------------------------------------------------|
| id               | UUID        | PRIMARY KEY       | Unique user identifier                           |
| email            | VARCHAR(255) | NOT NULL, UNIQUE  | User's email address                             |
| name             | VARCHAR(255) | NULLABLE          | User's display name                              |
| software_level   | VARCHAR(50)  | NULLABLE          | User's software skill level (Beginner/Intermediate/Advanced) |
| hardware_access  | VARCHAR(50)  | NULLABLE          | User's hardware access (Laptop/Cloud or Physical Robot) |

**Note**: This table is **read-only** for the chatbot backend. All user management is handled by Better Auth.

**Relationships**:
- One user → Many threads (1:N)

**Usage in Backend**:
- `user_id` extracted from Better Auth session validation
- User details (name, software_level) MAY be used for personalized responses (future enhancement)
- No direct writes to this table from chatbot backend

---

## Database Schema (SQL DDL)

### Table Creation

```sql
-- ============================================
-- Extension Requirements
-- ============================================
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- ============================================
-- Threads Table
-- ============================================
CREATE TABLE threads (
    thread_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    title VARCHAR(255),
    metadata JSONB NOT NULL DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),

    -- Constraints
    CONSTRAINT chk_threads_timestamps CHECK (created_at <= updated_at),
    CONSTRAINT chk_threads_metadata_object CHECK (jsonb_typeof(metadata) = 'object')
);

-- Indexes for threads
CREATE INDEX idx_threads_user_id ON threads(user_id);
CREATE INDEX idx_threads_updated_at ON threads(updated_at DESC);
CREATE INDEX idx_threads_created_at ON threads(created_at DESC);

-- Trigger to auto-update updated_at
CREATE OR REPLACE FUNCTION update_thread_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER trg_threads_updated_at
BEFORE UPDATE ON threads
FOR EACH ROW
EXECUTE FUNCTION update_thread_timestamp();

-- ============================================
-- Messages Table
-- ============================================
CREATE TABLE messages (
    message_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    thread_id UUID NOT NULL REFERENCES threads(thread_id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL CHECK (LENGTH(content) > 0 AND LENGTH(content) <= 100000),
    sequence_number INTEGER NOT NULL CHECK (sequence_number >= 1),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),

    -- Unique constraint for sequence numbering per thread
    CONSTRAINT uq_messages_thread_sequence UNIQUE (thread_id, sequence_number)
);

-- Indexes for messages
CREATE INDEX idx_messages_thread_id ON messages(thread_id);
CREATE INDEX idx_messages_created_at ON messages(created_at);

-- Trigger to update thread's updated_at when message added
CREATE OR REPLACE FUNCTION update_thread_on_message()
RETURNS TRIGGER AS $$
BEGIN
    UPDATE threads
    SET updated_at = NEW.created_at
    WHERE thread_id = NEW.thread_id;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER trg_messages_update_thread
AFTER INSERT ON messages
FOR EACH ROW
EXECUTE FUNCTION update_thread_on_message();
```

### Sample Queries

**Create New Thread**:
```sql
INSERT INTO threads (thread_id, user_id, title, metadata)
VALUES (
    uuid_generate_v4(),
    'a1b2c3d4-e5f6-7890-1234-567890abcdef',  -- user_id from Better Auth
    NULL,  -- Auto-generated later
    '{"tags": ["robotics", "kinematics"]}'::jsonb
)
RETURNING thread_id, created_at;
```

**Add Message to Thread**:
```sql
-- Get next sequence number
WITH next_seq AS (
    SELECT COALESCE(MAX(sequence_number), 0) + 1 AS seq
    FROM messages
    WHERE thread_id = 'thread-uuid-here'
)
INSERT INTO messages (message_id, thread_id, role, content, sequence_number)
SELECT
    uuid_generate_v4(),
    'thread-uuid-here',
    'user',
    'Explain inverse kinematics in robotics',
    seq
FROM next_seq
RETURNING message_id, sequence_number;
```

**Load Thread with Last 10 Messages**:
```sql
SELECT
    t.thread_id,
    t.title,
    t.metadata,
    t.created_at AS thread_created_at,
    t.updated_at AS thread_updated_at,
    json_agg(
        json_build_object(
            'message_id', m.message_id,
            'role', m.role,
            'content', m.content,
            'sequence_number', m.sequence_number,
            'created_at', m.created_at
        ) ORDER BY m.sequence_number DESC
    ) FILTER (WHERE m.message_id IS NOT NULL) AS messages
FROM threads t
LEFT JOIN LATERAL (
    SELECT * FROM messages
    WHERE thread_id = t.thread_id
    ORDER BY sequence_number DESC
    LIMIT 10
) m ON true
WHERE t.thread_id = 'thread-uuid-here'
  AND t.user_id = 'user-uuid-here'  -- Authorization check
GROUP BY t.thread_id;
```

**List User's Threads (Paginated)**:
```sql
SELECT
    thread_id,
    title,
    metadata,
    created_at,
    updated_at,
    (SELECT COUNT(*) FROM messages WHERE thread_id = threads.thread_id) AS message_count
FROM threads
WHERE user_id = 'user-uuid-here'
ORDER BY updated_at DESC
LIMIT 20 OFFSET 0;  -- First page
```

**Delete Thread (Cascades to Messages)**:
```sql
DELETE FROM threads
WHERE thread_id = 'thread-uuid-here'
  AND user_id = 'user-uuid-here';  -- Authorization check
-- CASCADE automatically deletes all messages
```

---

## Data Validation Rules

### Thread Validation

**At Application Layer** (Python/SQLAlchemy):
```python
from uuid import UUID
from datetime import datetime
from pydantic import BaseModel, Field, validator

class ThreadCreate(BaseModel):
    user_id: UUID
    title: str | None = Field(None, max_length=255)
    metadata: dict = Field(default_factory=dict)

    @validator('title')
    def validate_title(cls, v):
        if v is not None and len(v.strip()) == 0:
            return None  # Treat empty strings as NULL
        return v

    @validator('metadata')
    def validate_metadata(cls, v):
        if not isinstance(v, dict):
            raise ValueError("Metadata must be a dictionary")
        return v

class ThreadUpdate(BaseModel):
    title: str | None = Field(None, max_length=255)
    metadata: dict | None = None
```

### Message Validation

**At Application Layer**:
```python
class MessageCreate(BaseModel):
    thread_id: UUID
    role: Literal['user', 'assistant']
    content: str = Field(..., min_length=1, max_length=100000)

    @validator('content')
    def validate_content(cls, v):
        if not v.strip():
            raise ValueError("Content cannot be empty or whitespace-only")
        return v

class MessageResponse(BaseModel):
    message_id: UUID
    thread_id: UUID
    role: str
    content: str
    sequence_number: int
    created_at: datetime
```

---

## Performance Considerations

### Indexing Strategy

1. **Thread Lookups by User**:
   - `idx_threads_user_id` enables fast `WHERE user_id = ?` queries
   - Expected performance: < 10ms for 10K threads per user

2. **Recent Threads Sorting**:
   - `idx_threads_updated_at` DESC enables fast sorting
   - Used for "Recent Conversations" view

3. **Message Retrieval**:
   - `idx_messages_thread_id` + `uq_messages_thread_sequence` enable fast ordered retrieval
   - Expected performance: < 20ms for 50-message threads

4. **Pagination**:
   - `idx_threads_created_at` supports offset-based pagination
   - For very large datasets (10K+ threads), consider cursor-based pagination

### Query Optimization

**N+1 Query Prevention**:
```python
# BAD: N+1 queries
threads = db.query(Thread).filter_by(user_id=user_id).all()
for thread in threads:
    messages = db.query(Message).filter_by(thread_id=thread.thread_id).all()  # N queries

# GOOD: Single query with JOIN
threads_with_messages = (
    db.query(Thread)
    .outerjoin(Message, Message.thread_id == Thread.thread_id)
    .filter(Thread.user_id == user_id)
    .options(joinedload(Thread.messages))
    .all()
)
```

**Message Window Loading** (Last 10 Messages):
```python
from sqlalchemy import func

# Efficient subquery for last N messages
last_10_messages = (
    db.query(Message)
    .filter(Message.thread_id == thread_id)
    .order_by(Message.sequence_number.desc())
    .limit(10)
    .subquery()
)

messages = db.query(last_10_messages).order_by(last_10_messages.c.sequence_number.asc()).all()
```

### Connection Pooling

**Recommended Settings** (asyncpg):
```python
from sqlalchemy.ext.asyncio import create_async_engine

engine = create_async_engine(
    DATABASE_URL,
    pool_size=20,          # 20 connections per instance
    max_overflow=10,       # +10 overflow connections
    pool_timeout=30,       # 30s wait for connection
    pool_recycle=3600,     # Recycle connections after 1 hour
    pool_pre_ping=True,    # Verify connection health before use
)
```

---

## Security Considerations

### Row-Level Authorization

**Thread Access**:
```python
async def get_thread_by_id(thread_id: UUID, user_id: UUID) -> Thread:
    """Retrieve thread with authorization check."""
    thread = await db.query(Thread).filter(
        Thread.thread_id == thread_id,
        Thread.user_id == user_id  # Critical: user can only access own threads
    ).first()

    if not thread:
        raise HTTPException(status_code=404, detail="Thread not found")

    return thread
```

**Message Access**:
```python
async def get_messages_by_thread(thread_id: UUID, user_id: UUID) -> list[Message]:
    """Retrieve messages with ownership validation."""
    # First verify thread ownership
    thread = await get_thread_by_id(thread_id, user_id)

    # Then retrieve messages (already authorized via thread check)
    messages = await db.query(Message).filter(
        Message.thread_id == thread_id
    ).order_by(Message.sequence_number.asc()).all()

    return messages
```

### Input Sanitization

**SQL Injection Prevention**:
- ✅ Use SQLAlchemy ORM with parameterized queries (automatic)
- ✅ Never construct raw SQL with user input
- ✅ Validate UUIDs before querying

**Content Validation**:
```python
import bleach

def sanitize_message_content(content: str) -> str:
    """Remove potentially harmful content while preserving formatting."""
    # For text-only chatbot, we primarily validate length and non-empty
    # No HTML/XSS risk since content is not rendered as HTML
    if not content or len(content.strip()) == 0:
        raise ValueError("Content cannot be empty")

    if len(content) > 100000:
        raise ValueError("Content exceeds maximum length (100K characters)")

    return content.strip()
```

---

## Migration Strategy

### Initial Schema Deployment

**Using Alembic** (recommended):
```bash
# Initialize Alembic
alembic init alembic

# Create initial migration
alembic revision --autogenerate -m "Initial schema: threads and messages"

# Apply migration
alembic upgrade head
```

**Alembic Migration File** (`versions/001_initial_schema.py`):
```python
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects.postgresql import UUID, JSONB

def upgrade():
    # Create threads table
    op.create_table(
        'threads',
        sa.Column('thread_id', UUID(as_uuid=True), primary_key=True, server_default=sa.text('uuid_generate_v4()')),
        sa.Column('user_id', UUID(as_uuid=True), nullable=False),
        sa.Column('title', sa.String(255), nullable=True),
        sa.Column('metadata', JSONB, nullable=False, server_default='{}'),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.func.now()),
        sa.Column('updated_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.func.now()),
        sa.ForeignKeyConstraint(['user_id'], ['user.id'], ondelete='CASCADE'),
        sa.CheckConstraint('created_at <= updated_at', name='chk_threads_timestamps'),
    )

    # Create indexes
    op.create_index('idx_threads_user_id', 'threads', ['user_id'])
    op.create_index('idx_threads_updated_at', 'threads', [sa.desc('updated_at')])

    # Create messages table
    op.create_table(
        'messages',
        sa.Column('message_id', UUID(as_uuid=True), primary_key=True, server_default=sa.text('uuid_generate_v4()')),
        sa.Column('thread_id', UUID(as_uuid=True), nullable=False),
        sa.Column('role', sa.String(20), nullable=False),
        sa.Column('content', sa.Text, nullable=False),
        sa.Column('sequence_number', sa.Integer, nullable=False),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.func.now()),
        sa.ForeignKeyConstraint(['thread_id'], ['threads.thread_id'], ondelete='CASCADE'),
        sa.CheckConstraint("role IN ('user', 'assistant')", name='chk_messages_role'),
        sa.CheckConstraint('LENGTH(content) > 0 AND LENGTH(content) <= 100000', name='chk_messages_content_length'),
        sa.UniqueConstraint('thread_id', 'sequence_number', name='uq_messages_thread_sequence'),
    )

    # Create indexes
    op.create_index('idx_messages_thread_id', 'messages', ['thread_id'])

def downgrade():
    op.drop_table('messages')
    op.drop_table('threads')
```

### Future Schema Changes

**Adding Columns** (e.g., `archived` flag):
```python
def upgrade():
    op.add_column('threads', sa.Column('archived', sa.Boolean, nullable=False, server_default='false'))
    op.create_index('idx_threads_archived', 'threads', ['user_id', 'archived'])

def downgrade():
    op.drop_index('idx_threads_archived')
    op.drop_column('threads', 'archived')
```

---

## Testing Data

### Test Fixtures

**Pytest Fixtures** (`tests/fixtures/data.py`):
```python
import pytest
from uuid import uuid4
from datetime import datetime

@pytest.fixture
def test_user_id():
    """Better Auth user ID for testing."""
    return uuid4()

@pytest.fixture
def test_thread(db_session, test_user_id):
    """Create test thread."""
    from app.models import Thread

    thread = Thread(
        thread_id=uuid4(),
        user_id=test_user_id,
        title="Test Thread",
        metadata={"tags": ["test"]}
    )
    db_session.add(thread)
    db_session.commit()
    return thread

@pytest.fixture
def test_messages(db_session, test_thread):
    """Create test messages."""
    from app.models import Message

    messages = [
        Message(
            message_id=uuid4(),
            thread_id=test_thread.thread_id,
            role='user',
            content='What is inverse kinematics?',
            sequence_number=1
        ),
        Message(
            message_id=uuid4(),
            thread_id=test_thread.thread_id,
            role='assistant',
            content='Inverse kinematics is...',
            sequence_number=2
        )
    ]
    db_session.add_all(messages)
    db_session.commit()
    return messages
```

### Sample Test Data

**SQL Insert** (`tests/data/sample_data.sql`):
```sql
-- Test user (Better Auth managed)
INSERT INTO "user" (id, email, name, software_level, hardware_access)
VALUES
    ('a1b2c3d4-e5f6-7890-1234-567890abcdef', 'test@example.com', 'Test User', 'Intermediate', 'Laptop/Cloud');

-- Test thread
INSERT INTO threads (thread_id, user_id, title, metadata)
VALUES
    ('thread-0001-0000-0000-000000000001', 'a1b2c3d4-e5f6-7890-1234-567890abcdef', 'Kinematics Questions', '{"tags": ["kinematics", "robotics"]}');

-- Test messages
INSERT INTO messages (message_id, thread_id, role, content, sequence_number)
VALUES
    ('msg-0001-0000-0000-000000000001', 'thread-0001-0000-0000-000000000001', 'user', 'What is inverse kinematics?', 1),
    ('msg-0002-0000-0000-000000000002', 'thread-0001-0000-0000-000000000001', 'assistant', 'Inverse kinematics (IK) is the process of calculating joint angles needed to place a robot''s end-effector at a desired position and orientation.[^1]\n\n[^1]: Chapter 5: Robot Kinematics, Section 5.3', 2),
    ('msg-0003-0000-0000-000000000003', 'thread-0001-0000-0000-000000000001', 'user', 'Give me a code example in Python', 3);
```

---

## Related Documents

- **Specification**: `spec.md` - Functional requirements and acceptance criteria
- **Plan**: `plan.md` - Implementation plan with architecture decisions
- **Research**: `research.md` - ChatKit integration patterns and best practices
- **Contracts**: `contracts/chatkit-api.json` - API request/response schemas

---

## Changelog

| Date       | Change                                      | Author     |
|------------|---------------------------------------------|------------|
| 2025-12-25 | Initial data model specification            | AI Assistant |
