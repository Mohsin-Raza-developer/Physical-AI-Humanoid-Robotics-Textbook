"""
Thread service for managing conversation threads.

Provides CRUD operations for threads with user ownership validation.
"""
from sqlalchemy import select, func, desc
from sqlalchemy.ext.asyncio import AsyncSession
from app.models.thread import Thread
from app.schemas.thread import ThreadCreate, ThreadResponse, ThreadListItem
from app.utils.logger import get_logger
from app.utils.errors import NotFoundError, ForbiddenError
from uuid import UUID
from datetime import datetime

logger = get_logger(__name__)


class ThreadService:
    """Service for thread management operations."""

    @staticmethod
    async def create_thread(
        db: AsyncSession,
        user_id: str,
        thread_data: ThreadCreate
    ) -> Thread:
        """
        Create a new conversation thread.

        Args:
            db: Database session
            user_id: Owner user ID
            thread_data: Thread creation data

        Returns:
            Thread: Created thread
        """
        thread = Thread(
            user_id=UUID(user_id),
            title=thread_data.title
        )

        db.add(thread)
        await db.commit()
        await db.refresh(thread)

        logger.info(
            "thread_created",
            thread_id=str(thread.thread_id),
            user_id=user_id,
            has_title=bool(thread.title)
        )

        return thread

    @staticmethod
    async def get_threads(
        db: AsyncSession,
        user_id: str,
        limit: int = 20,
        offset: int = 0
    ) -> tuple[list[Thread], int]:
        """
        Get paginated list of user's threads.

        Args:
            db: Database session
            user_id: Owner user ID
            limit: Maximum number of threads to return
            offset: Number of threads to skip

        Returns:
            tuple: (List of threads, total count)
        """
        # Get total count
        count_query = select(func.count(Thread.thread_id)).where(
            Thread.user_id == UUID(user_id)
        )
        total = await db.scalar(count_query)

        # Get threads ordered by most recent first
        query = (
            select(Thread)
            .where(Thread.user_id == UUID(user_id))
            .order_by(desc(Thread.updated_at))
            .limit(limit)
            .offset(offset)
        )

        result = await db.execute(query)
        threads = result.scalars().all()

        logger.info(
            "threads_retrieved",
            user_id=user_id,
            count=len(threads),
            total=total
        )

        return threads, total

    @staticmethod
    async def get_thread(
        db: AsyncSession,
        thread_id: str,
        user_id: str
    ) -> Thread:
        """
        Get a specific thread with ownership validation.

        Args:
            db: Database session
            thread_id: Thread ID
            user_id: User ID (for ownership check)

        Returns:
            Thread: Thread with messages

        Raises:
            NotFoundError: If thread doesn't exist
            ForbiddenError: If user doesn't own the thread
        """
        query = select(Thread).where(Thread.thread_id == UUID(thread_id))
        result = await db.execute(query)
        thread = result.scalar_one_or_none()

        if not thread:
            raise NotFoundError("Thread", thread_id)

        if str(thread.user_id) != user_id:
            logger.warning(
                "unauthorized_thread_access",
                thread_id=thread_id,
                user_id=user_id,
                owner_id=str(thread.user_id)
            )
            raise ForbiddenError("You don't have permission to access this thread")

        logger.info("thread_retrieved", thread_id=thread_id, user_id=user_id)
        return thread

    @staticmethod
    async def delete_thread(
        db: AsyncSession,
        thread_id: str,
        user_id: str
    ) -> None:
        """
        Delete a thread (cascade deletes messages).

        Args:
            db: Database session
            thread_id: Thread ID
            user_id: User ID (for ownership check)

        Raises:
            NotFoundError: If thread doesn't exist
            ForbiddenError: If user doesn't own the thread
        """
        # Get thread and validate ownership
        thread = await ThreadService.get_thread(db, thread_id, user_id)

        await db.delete(thread)
        await db.commit()

        logger.info(
            "thread_deleted",
            thread_id=thread_id,
            user_id=user_id
        )

    @staticmethod
    async def update_thread_title(
        db: AsyncSession,
        thread_id: str,
        title: str
    ) -> Thread:
        """
        Update thread title (for auto-generation).

        Args:
            db: Database session
            thread_id: Thread ID
            title: New title

        Returns:
            Thread: Updated thread
        """
        query = select(Thread).where(Thread.thread_id == UUID(thread_id))
        result = await db.execute(query)
        thread = result.scalar_one_or_none()

        if not thread:
            raise NotFoundError("Thread", thread_id)

        # Only update if no existing title
        if not thread.title:
            thread.title = title
            thread.updated_at = datetime.utcnow()
            await db.commit()
            await db.refresh(thread)

            logger.info(
                "thread_title_updated",
                thread_id=thread_id,
                title=title
            )

        return thread
