"""
Thread SQLAlchemy model.

Represents conversation threads owned by users.
"""
from sqlalchemy import Column, String, DateTime, ForeignKey, Index
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from app.database import Base
import uuid


class Thread(Base):
    """
    Conversation thread model.

    A thread represents a single conversation between a user and the AI assistant.
    Threads contain multiple messages and have an optional title (auto-generated or manual).
    """

    __tablename__ = "threads"

    # Columns
    thread_id = Column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4,
        comment="Unique identifier for the thread"
    )
    user_id = Column(
        UUID(as_uuid=True),
        ForeignKey("users.id", ondelete="CASCADE"),
        nullable=False,
        comment="Owner of the thread"
    )
    title = Column(
        String(255),
        nullable=True,
        comment="Thread title (auto-generated or user-provided)"
    )
    created_at = Column(
        DateTime(timezone=True),
        server_default=func.now(),
        nullable=False,
        comment="Thread creation timestamp"
    )
    updated_at = Column(
        DateTime(timezone=True),
        server_default=func.now(),
        onupdate=func.now(),
        nullable=False,
        comment="Last activity timestamp (updated on new messages)"
    )

    # Relationships
    messages = relationship(
        "Message",
        back_populates="thread",
        cascade="all, delete-orphan",
        lazy="selectin"
    )

    # Indexes
    __table_args__ = (
        Index("idx_threads_user_id", "user_id"),
        Index("idx_threads_created_at", "created_at desc"),
    )

    def __repr__(self):
        return f"<Thread(thread_id={self.thread_id}, user_id={self.user_id}, title='{self.title}')>"
