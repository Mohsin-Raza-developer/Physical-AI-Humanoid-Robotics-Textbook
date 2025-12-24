"""
Message SQLAlchemy model.

Represents individual messages within conversation threads.
"""
from sqlalchemy import Column, String, Text, Integer, DateTime, ForeignKey, Index, CheckConstraint, UniqueConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from app.database import Base
import uuid


class Message(Base):
    """
    Message model for conversation messages.

    A message belongs to a thread and can be from either the user or the AI assistant.
    Messages are ordered by sequence_number within each thread.
    """

    __tablename__ = "messages"

    # Columns
    message_id = Column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid.uuid4,
        comment="Unique identifier for the message"
    )
    thread_id = Column(
        UUID(as_uuid=True),
        ForeignKey("threads.thread_id", ondelete="CASCADE"),
        nullable=False,
        comment="Parent thread"
    )
    role = Column(
        String(20),
        nullable=False,
        comment="Message author (user or assistant)"
    )
    content = Column(
        Text,
        nullable=False,
        comment="Message content (plain text or markdown)"
    )
    created_at = Column(
        DateTime(timezone=True),
        server_default=func.now(),
        nullable=False,
        comment="Message creation timestamp"
    )
    sequence_number = Column(
        Integer,
        nullable=False,
        comment="Order of message in thread (1, 2, 3, ...)"
    )

    # Relationships
    thread = relationship("Thread", back_populates="messages")

    # Constraints
    __table_args__ = (
        CheckConstraint("role IN ('user', 'assistant')", name="check_role"),
        UniqueConstraint("thread_id", "sequence_number", name="uq_thread_sequence"),
        Index("idx_messages_thread_id", "thread_id"),
        Index("idx_messages_created_at", "created_at"),
        Index("idx_messages_sequence", "thread_id", "sequence_number"),
    )

    def __repr__(self):
        return f"<Message(message_id={self.message_id}, thread_id={self.thread_id}, role='{self.role}', seq={self.sequence_number})>"
