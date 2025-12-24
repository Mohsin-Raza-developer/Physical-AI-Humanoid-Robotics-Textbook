# Data Model: ChatKit Gemini Backend

**Feature**: 008-chatkit-gemini-backend
**Date**: 2025-12-23
**Database**: Neon PostgreSQL (existing auth database)
**ORM**: SQLAlchemy 2.0+ (async)

---

## Overview

This document defines the database schema for the ChatKit Gemini backend. All tables will be added to the existing Neon PostgreSQL database alongside the auth tables (`users`, `sessions`, `password_resets`). The schema supports conversation threads and messages (text-only chat) with proper referential integrity.

**Note**: Attachment functionality has been deferred to a future phase. This MVP focuses on text-based conversations only.

---

## Entity Relationship Diagram

```
┌─────────────────┐
│     users       │  (existing auth table)
│─────────────────│
│ id (UUID) PK    │
│ email           │
│ first_name      │
│ last_name       │
│ software_level  │
│ created_at      │
└────────┬────────┘
         │
         │ 1:N
         │
         ↓
┌─────────────────┐
│    threads      │
│─────────────────│
│ thread_id (UUID) PK
│ user_id (UUID) FK
│ title (VARCHAR)
│ created_at
│ updated_at
└────────┬─────────────┘
         │
         │ 1:N
         │
         ↓
┌─────────────────┐
│    messages     │
│─────────────────│
│ message_id (UUID) PK
│ thread_id (UUID) FK
│ role (ENUM)
│ content (TEXT)
│ created_at
│ sequence_number (INT)
└─────────────────┘
```

---

## Table Definitions

### 1. threads

Stores conversation threads owned by users.

**Columns**:

| Column       | Type                | Constraints                           | Description                                    |
|--------------|---------------------|---------------------------------------|------------------------------------------------|
| thread_id    | UUID                | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier for the thread               |
| user_id      | UUID                | NOT NULL, FOREIGN KEY → users(id) ON DELETE CASCADE | Owner of the thread                            |
| title        | VARCHAR(255)        | NULL                                  | Thread title (auto-generated or user-provided) |
| created_at   | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW()               | Thread creation timestamp                      |
| updated_at   | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW()               | Last activity timestamp (updated on new messages) |

**Indexes**:
- `idx_threads_user_id` on `user_id` (for listing user's threads)
- `idx_threads_created_at` on `created_at DESC` (for sorting by recency)

**SQL**:
```sql
CREATE TABLE threads (
    thread_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    title VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_threads_user_id ON threads(user_id);
CREATE INDEX idx_threads_created_at ON threads(created_at DESC);
```

**Validation Rules**:
- `title` max length: 255 characters
- `user_id` must exist in `users` table
- `updated_at` must be >= `created_at`

**State Transitions**:
- Created → Active (on first message)
- Active → Active (on new messages, update `updated_at`)
- Active → Deleted (cascade delete messages/attachments)

---

### 2. messages

Stores individual messages within threads (user and assistant messages).

**Columns**:

| Column          | Type                | Constraints                           | Description                                    |
|-----------------|---------------------|---------------------------------------|------------------------------------------------|
| message_id      | UUID                | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier for the message              |
| thread_id       | UUID                | NOT NULL, FOREIGN KEY → threads(thread_id) ON DELETE CASCADE | Parent thread                                  |
| role            | VARCHAR(20)         | NOT NULL, CHECK (role IN ('user', 'assistant')) | Message author (user or AI assistant)          |
| content         | TEXT                | NOT NULL                              | Message content (plain text or markdown)       |
| created_at      | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW()               | Message creation timestamp                     |
| sequence_number | INTEGER             | NOT NULL                              | Order of message in thread (1, 2, 3, ...)      |

**Indexes**:
- `idx_messages_thread_id` on `thread_id` (for retrieving thread messages)
- `idx_messages_created_at` on `created_at` (for chronological ordering)
- `idx_messages_sequence` on `(thread_id, sequence_number)` UNIQUE (enforce order)

**SQL**:
```sql
CREATE TABLE messages (
    message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    thread_id UUID NOT NULL REFERENCES threads(thread_id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    sequence_number INTEGER NOT NULL,
    UNIQUE (thread_id, sequence_number)
);

CREATE INDEX idx_messages_thread_id ON messages(thread_id);
CREATE INDEX idx_messages_created_at ON messages(created_at);
CREATE INDEX idx_messages_sequence ON messages(thread_id, sequence_number);
```

**Validation Rules**:
- `content` max length: 10,000 characters (application-level check)
- `role` must be either 'user' or 'assistant'
- `sequence_number` must be unique per thread and sequential (1-based)
- Thread must exist

**Sequence Number Logic**:
```python
# Get next sequence number
next_seq = (
    await session.execute(
        select(func.max(Message.sequence_number))
        .where(Message.thread_id == thread_id)
    )
).scalar() or 0
new_seq = next_seq + 1
```

---

### 3. attachments

Stores metadata for files attached to messages.

**Columns**:

| Column         | Type                | Constraints                           | Description                                    |
|----------------|---------------------|---------------------------------------|------------------------------------------------|
| attachment_id  | UUID                | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier for the attachment           |
| message_id     | UUID                | NOT NULL, FOREIGN KEY → messages(message_id) ON DELETE CASCADE | Parent message                                 |
| file_name      | VARCHAR(255)        | NOT NULL                              | Original filename (e.g., "code.py")            |
| file_type      | VARCHAR(100)        | NULL                                  | MIME type (e.g., "text/plain", "image/png")   |
| file_size      | BIGINT              | NOT NULL                              | File size in bytes                             |
| storage_url    | TEXT                | NOT NULL                              | R2 storage URL (internal, not signed)          |
| created_at     | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW()               | Upload timestamp                               |

**Indexes**:
- `idx_attachments_message_id` on `message_id` (for retrieving message attachments)

**SQL**:
```sql
CREATE TABLE attachments (
    attachment_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(message_id) ON DELETE CASCADE,
    file_name VARCHAR(255) NOT NULL,
    file_type VARCHAR(100),
    file_size BIGINT NOT NULL,
    storage_url TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_attachments_message_id ON attachments(message_id);
```

**Validation Rules**:
- `file_name` max length: 255 characters
- `file_size` max: 10MB (10,485,760 bytes)
- `file_type` whitelist (application-level):
  - Text: `text/plain`, `text/markdown`
  - Code: `text/x-python`, `text/x-c++src`
  - Images: `image/png`, `image/jpeg`
  - Documents: `application/pdf`
- `storage_url` format: `s3://chatbot-attachments/{user_id}/{thread_id}/{attachment_id}`

**Retention Policy**:
- Attachments are deleted when parent thread is deleted (CASCADE)
- Lifecycle: Upload → Active → Deleted (with thread)

---

## SQLAlchemy Models

### Base Configuration

```python
from sqlalchemy.ext.asyncio import AsyncAttrs, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase
from sqlalchemy import Column, String, Integer, BigInteger, Text, ForeignKey, TIMESTAMP, CheckConstraint
from sqlalchemy.dialects.postgresql import UUID
import uuid
from datetime import datetime, timezone

# Async engine
engine = create_async_engine(
    os.environ["DATABASE_URL"],
    echo=True,  # Set to False in production
    pool_size=20,
    max_overflow=10
)

async_session = async_sessionmaker(engine, expire_on_commit=False)

class Base(AsyncAttrs, DeclarativeBase):
    pass
```

### Thread Model

```python
from sqlalchemy.orm import relationship

class Thread(Base):
    __tablename__ = "threads"

    thread_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False)
    title = Column(String(255), nullable=True)
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, default=lambda: datetime.now(timezone.utc))
    updated_at = Column(TIMESTAMP(timezone=True), nullable=False, default=lambda: datetime.now(timezone.utc), onupdate=lambda: datetime.now(timezone.utc))

    # Relationships
    messages = relationship("Message", back_populates="thread", cascade="all, delete-orphan")
    user = relationship("User", foreign_keys=[user_id])  # Assuming User model exists

    def __repr__(self):
        return f"<Thread(thread_id={self.thread_id}, title={self.title})>"
```

### Message Model

```python
class Message(Base):
    __tablename__ = "messages"
    __table_args__ = (
        CheckConstraint("role IN ('user', 'assistant')", name="check_role"),
    )

    message_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    thread_id = Column(UUID(as_uuid=True), ForeignKey("threads.thread_id", ondelete="CASCADE"), nullable=False)
    role = Column(String(20), nullable=False)
    content = Column(Text, nullable=False)
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, default=lambda: datetime.now(timezone.utc))
    sequence_number = Column(Integer, nullable=False)

    # Relationships
    thread = relationship("Thread", back_populates="messages")
    attachments = relationship("Attachment", back_populates="message", cascade="all, delete-orphan")

    def __repr__(self):
        return f"<Message(message_id={self.message_id}, role={self.role}, seq={self.sequence_number})>"
```

### Attachment Model

```python
class Attachment(Base):
    __tablename__ = "attachments"

    attachment_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    message_id = Column(UUID(as_uuid=True), ForeignKey("messages.message_id", ondelete="CASCADE"), nullable=False)
    file_name = Column(String(255), nullable=False)
    file_type = Column(String(100), nullable=True)
    file_size = Column(BigInteger, nullable=False)
    storage_url = Column(Text, nullable=False)
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, default=lambda: datetime.now(timezone.utc))

    # Relationships
    message = relationship("Message", back_populates="attachments")

    def __repr__(self):
        return f"<Attachment(attachment_id={self.attachment_id}, file_name={self.file_name})>"
```

---

## Pydantic Schemas (Request/Response Models)

### Thread Schemas

```python
from pydantic import BaseModel, Field
from datetime import datetime
from uuid import UUID
from typing import Optional

class ThreadCreate(BaseModel):
    title: Optional[str] = Field(None, max_length=255)

class ThreadResponse(BaseModel):
    thread_id: UUID
    user_id: UUID
    title: Optional[str]
    created_at: datetime
    updated_at: datetime

    model_config = {"from_attributes": True}

class ThreadListResponse(BaseModel):
    threads: list[ThreadResponse]
    total: int
```

### Message Schemas

```python
class MessageCreate(BaseModel):
    content: str = Field(..., min_length=1, max_length=10000)
    attachment_ids: Optional[list[UUID]] = None

class MessageResponse(BaseModel):
    message_id: UUID
    thread_id: UUID
    role: str
    content: str
    created_at: datetime
    sequence_number: int
    attachments: list["AttachmentResponse"] = []

    model_config = {"from_attributes": True}

class MessageListResponse(BaseModel):
    messages: list[MessageResponse]
    total: int
```

### Attachment Schemas

```python
class AttachmentUploadRequest(BaseModel):
    file_name: str = Field(..., max_length=255)
    file_type: str = Field(..., max_length=100)
    file_size: int = Field(..., gt=0, le=10485760)  # Max 10MB

class AttachmentUploadResponse(BaseModel):
    attachment_id: UUID
    upload_url: str  # Signed R2 URL
    expires_in: int  # Seconds

class AttachmentResponse(BaseModel):
    attachment_id: UUID
    message_id: UUID
    file_name: str
    file_type: Optional[str]
    file_size: int
    download_url: Optional[str]  # Signed URL (generated on-demand)
    created_at: datetime

    model_config = {"from_attributes": True}
```

---

## Database Migrations (Alembic)

### Initial Migration

```python
# alembic/versions/001_create_chatbot_tables.py
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

def upgrade():
    # Create threads table
    op.create_table(
        'threads',
        sa.Column('thread_id', postgresql.UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('users.id', ondelete='CASCADE'), nullable=False),
        sa.Column('title', sa.String(255), nullable=True),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('updated_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text('NOW()'))
    )
    op.create_index('idx_threads_user_id', 'threads', ['user_id'])
    op.create_index('idx_threads_created_at', 'threads', ['created_at'], postgresql_ops={'created_at': 'DESC'})

    # Create messages table
    op.create_table(
        'messages',
        sa.Column('message_id', postgresql.UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('thread_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('threads.thread_id', ondelete='CASCADE'), nullable=False),
        sa.Column('role', sa.String(20), nullable=False),
        sa.Column('content', sa.Text, nullable=False),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('sequence_number', sa.Integer, nullable=False),
        sa.CheckConstraint("role IN ('user', 'assistant')", name='check_role'),
        sa.UniqueConstraint('thread_id', 'sequence_number', name='uq_thread_sequence')
    )
    op.create_index('idx_messages_thread_id', 'messages', ['thread_id'])
    op.create_index('idx_messages_created_at', 'messages', ['created_at'])
    op.create_index('idx_messages_sequence', 'messages', ['thread_id', 'sequence_number'])

    # Create attachments table
    op.create_table(
        'attachments',
        sa.Column('attachment_id', postgresql.UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('message_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('messages.message_id', ondelete='CASCADE'), nullable=False),
        sa.Column('file_name', sa.String(255), nullable=False),
        sa.Column('file_type', sa.String(100), nullable=True),
        sa.Column('file_size', sa.BigInteger, nullable=False),
        sa.Column('storage_url', sa.Text, nullable=False),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text('NOW()'))
    )
    op.create_index('idx_attachments_message_id', 'attachments', ['message_id'])

def downgrade():
    op.drop_table('attachments')
    op.drop_table('messages')
    op.drop_table('threads')
```

---

## Query Patterns

### Common Queries

#### 1. List User's Threads (Most Recent First)

```python
from sqlalchemy import select
from sqlalchemy.orm import selectinload

async def get_user_threads(user_id: UUID, limit: int = 20, offset: int = 0):
    async with async_session() as session:
        stmt = (
            select(Thread)
            .where(Thread.user_id == user_id)
            .order_by(Thread.updated_at.desc())
            .limit(limit)
            .offset(offset)
        )
        result = await session.execute(stmt)
        threads = result.scalars().all()
        return threads
```

#### 2. Get Thread with Messages (Eager Loading)

```python
async def get_thread_with_messages(thread_id: UUID):
    async with async_session() as session:
        stmt = (
            select(Thread)
            .where(Thread.thread_id == thread_id)
            .options(
                selectinload(Thread.messages).selectinload(Message.attachments)
            )
        )
        result = await session.execute(stmt)
        thread = result.scalar_one_or_none()
        return thread
```

#### 3. Create Message with Auto-Sequence

```python
from sqlalchemy import func

async def create_message(thread_id: UUID, role: str, content: str):
    async with async_session() as session:
        # Get next sequence number
        next_seq_stmt = select(func.max(Message.sequence_number)).where(Message.thread_id == thread_id)
        max_seq = (await session.execute(next_seq_stmt)).scalar() or 0

        # Create message
        message = Message(
            thread_id=thread_id,
            role=role,
            content=content,
            sequence_number=max_seq + 1
        )
        session.add(message)

        # Update thread's updated_at
        thread_stmt = select(Thread).where(Thread.thread_id == thread_id)
        thread = (await session.execute(thread_stmt)).scalar_one()
        thread.updated_at = datetime.now(timezone.utc)

        await session.commit()
        await session.refresh(message)
        return message
```

#### 4. Delete Thread (Cascade Delete Messages/Attachments)

```python
async def delete_thread(thread_id: UUID, user_id: UUID):
    async with async_session() as session:
        stmt = select(Thread).where(
            Thread.thread_id == thread_id,
            Thread.user_id == user_id  # Ensure ownership
        )
        thread = (await session.execute(stmt)).scalar_one_or_none()

        if not thread:
            raise HTTPException(status_code=404, detail="Thread not found")

        await session.delete(thread)  # CASCADE deletes messages and attachments
        await session.commit()
```

---

## Performance Considerations

### Connection Pooling

```python
# Recommended settings for Railway/Render
engine = create_async_engine(
    DATABASE_URL,
    pool_size=10,           # Max persistent connections
    max_overflow=5,          # Additional connections under load
    pool_recycle=3600,       # Recycle connections every hour
    pool_pre_ping=True       # Verify connection before use
)
```

### Query Optimization

1. **Use Indexes**: All foreign keys are indexed
2. **Eager Loading**: Use `selectinload()` to avoid N+1 queries
3. **Pagination**: Always use `LIMIT`/`OFFSET` for list endpoints
4. **Partial Loading**: Use `defer()` for large TEXT columns when not needed

### Estimated Data Growth

| Table       | Rows/Month (100 users) | Storage Impact    |
|-------------|-------------------------|-------------------|
| threads     | ~2,000                  | Minimal (<1MB)    |
| messages    | ~40,000                 | Moderate (~50MB)  |
| attachments | ~500                    | Links only (<1MB) |

**Total DB Growth**: ~51MB/month (Neon Free Tier: 3GB)

---

## Testing Data

### Seed Data for Development

```python
async def seed_test_data():
    async with async_session() as session:
        # Create test thread
        thread = Thread(
            user_id=uuid.UUID("test-user-id"),
            title="Test Conversation about ROS 2"
        )
        session.add(thread)
        await session.flush()

        # Add messages
        messages = [
            Message(thread_id=thread.thread_id, role="user", content="What is ROS 2?", sequence_number=1),
            Message(thread_id=thread.thread_id, role="assistant", content="ROS 2 is...", sequence_number=2)
        ]
        session.add_all(messages)
        await session.commit()
```

---

## Next Steps

1. ✅ **Data Model Complete** → Proceed to API Contracts
2. Run Alembic migration in development environment
3. Test CRUD operations with pytest
4. Validate cascade deletes
5. Monitor query performance with `EXPLAIN ANALYZE`

---

**Status**: ✅ Ready for Implementation
**Dependencies**: Requires Neon database access, SQLAlchemy 2.0+ installed
