"""
Message service for managing conversation messages.

Provides operations for creating and retrieving messages.
"""
from sqlalchemy import select, func, desc
from sqlalchemy.ext.asyncio import AsyncSession
from app.models.message import Message
from app.schemas.message import MessageCreate, MessageResponse
from app.utils.logger import get_logger
from uuid import UUID
from datetime import datetime

logger = get_logger(__name__)


class MessageService:
    """Service for message management operations."""

    @staticmethod
    async def get_next_sequence_number(
        db: AsyncSession,
        thread_id: str
    ) -> int:
        """
        Get next sequence number for a message in a thread.

        Args:
            db: Database session
            thread_id: Thread ID

        Returns:
            int: Next sequence number (1-based)
        """
        query = select(func.max(Message.sequence_number)).where(
            Message.thread_id == UUID(thread_id)
        )
        max_seq = await db.scalar(query)
        return (max_seq or 0) + 1

    @staticmethod
    async def create_message(
        db: AsyncSession,
        thread_id: str,
        role: str,
        content: str
    ) -> Message:
        """
        Create a new message in a thread.

        Args:
            db: Database session
            thread_id: Parent thread ID
            role: Message role ('user' or 'assistant')
            content: Message content

        Returns:
            Message: Created message
        """
        # Get next sequence number
        sequence_number = await MessageService.get_next_sequence_number(
            db, thread_id
        )

        message = Message(
            thread_id=UUID(thread_id),
            role=role,
            content=content,
            sequence_number=sequence_number
        )

        db.add(message)
        await db.commit()
        await db.refresh(message)

        logger.info(
            "message_created",
            message_id=str(message.message_id),
            thread_id=thread_id,
            role=role,
            sequence=sequence_number,
            content_length=len(content)
        )

        # Update thread's updated_at timestamp
        from app.models.thread import Thread
        thread_query = select(Thread).where(Thread.thread_id == UUID(thread_id))
        thread_result = await db.execute(thread_query)
        thread = thread_result.scalar_one_or_none()

        if thread:
            thread.updated_at = datetime.utcnow()
            await db.commit()

        return message

    @staticmethod
    async def get_messages(
        db: AsyncSession,
        thread_id: str,
        limit: int = 50,
        offset: int = 0
    ) -> tuple[list[Message], int]:
        """
        Get paginated messages for a thread.

        Args:
            db: Database session
            thread_id: Thread ID
            limit: Maximum messages to return
            offset: Number of messages to skip

        Returns:
            tuple: (List of messages, total count)
        """
        # Get total count
        count_query = select(func.count(Message.message_id)).where(
            Message.thread_id == UUID(thread_id)
        )
        total = await db.scalar(count_query)

        # Get messages ordered by sequence
        query = (
            select(Message)
            .where(Message.thread_id == UUID(thread_id))
            .order_by(Message.sequence_number)
            .limit(limit)
            .offset(offset)
        )

        result = await db.execute(query)
        messages = result.scalars().all()

        logger.info(
            "messages_retrieved",
            thread_id=thread_id,
            count=len(messages),
            total=total
        )

        return messages, total

    @staticmethod
    async def get_conversation_history(
        db: AsyncSession,
        thread_id: str,
        max_messages: int = 10
    ) -> list[dict]:
        """
        Get recent conversation history for context.

        Args:
            db: Database session
            thread_id: Thread ID
            max_messages: Maximum number of recent messages

        Returns:
            list: List of message dicts with role and content
        """
        query = (
            select(Message)
            .where(Message.thread_id == UUID(thread_id))
            .order_by(desc(Message.sequence_number))
            .limit(max_messages)
        )

        result = await db.execute(query)
        messages = result.scalars().all()

        # Reverse to get chronological order
        messages = list(reversed(messages))

        history = [
            {
                "role": msg.role,
                "content": msg.content
            }
            for msg in messages
        ]

        return history
