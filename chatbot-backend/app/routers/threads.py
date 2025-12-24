"""
Thread API endpoints.

Provides CRUD operations for conversation threads.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.ext.asyncio import AsyncSession
from app.database import get_db
from app.middleware.auth import get_current_user
from app.services.thread_service import ThreadService
from app.schemas.thread import (
    ThreadCreate,
    ThreadResponse,
    ThreadListResponse,
    ThreadListItem,
    ThreadDetailResponse
)
from app.schemas.message import MessageResponse
from app.utils.errors import NotFoundError, ForbiddenError
from app.utils.logger import get_logger

logger = get_logger(__name__)

router = APIRouter(prefix="/api/threads", tags=["Threads"])


@router.post("/", response_model=ThreadResponse, status_code=status.HTTP_201_CREATED)
async def create_thread(
    thread_data: ThreadCreate,
    db: AsyncSession = Depends(get_db),
    user_id: str = Depends(get_current_user)
):
    """
    Create a new conversation thread.

    Args:
        thread_data: Thread creation data (optional title)
        db: Database session
        user_id: Authenticated user ID

    Returns:
        ThreadResponse: Created thread
    """
    try:
        thread = await ThreadService.create_thread(db, user_id, thread_data)
        return ThreadResponse.from_orm(thread)
    except Exception as e:
        logger.error("create_thread_error", error=str(e), user_id=user_id)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create thread"
        )


@router.get("/", response_model=ThreadListResponse)
async def get_threads(
    limit: int = Query(default=20, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
    db: AsyncSession = Depends(get_db),
    user_id: str = Depends(get_current_user)
):
    """
    Get paginated list of user's threads.

    Args:
        limit: Maximum threads to return (1-100)
        offset: Number of threads to skip
        db: Database session
        user_id: Authenticated user ID

    Returns:
        ThreadListResponse: List of threads with total count
    """
    try:
        threads, total = await ThreadService.get_threads(db, user_id, limit, offset)

        thread_items = [
            ThreadListItem(
                thread_id=thread.thread_id,
                title=thread.title,
                created_at=thread.created_at,
                updated_at=thread.updated_at
            )
            for thread in threads
        ]

        return ThreadListResponse(threads=thread_items, total=total)
    except Exception as e:
        logger.error("get_threads_error", error=str(e), user_id=user_id)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve threads"
        )


@router.get("/{thread_id}", response_model=ThreadDetailResponse)
async def get_thread(
    thread_id: str,
    db: AsyncSession = Depends(get_db),
    user_id: str = Depends(get_current_user)
):
    """
    Get thread details with message history.

    Args:
        thread_id: Thread ID
        db: Database session
        user_id: Authenticated user ID

    Returns:
        ThreadDetailResponse: Thread with messages

    Raises:
        HTTPException: 404 if thread not found, 403 if not owned by user
    """
    try:
        thread = await ThreadService.get_thread(db, thread_id, user_id)

        # Convert messages to response format
        messages = [
            MessageResponse.from_orm(msg)
            for msg in thread.messages
        ]

        return ThreadDetailResponse(
            thread_id=thread.thread_id,
            title=thread.title,
            messages=messages
        )
    except NotFoundError as e:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=str(e))
    except ForbiddenError as e:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail=str(e))
    except Exception as e:
        logger.error("get_thread_error", error=str(e), thread_id=thread_id)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve thread"
        )


@router.delete("/{thread_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_thread(
    thread_id: str,
    db: AsyncSession = Depends(get_db),
    user_id: str = Depends(get_current_user)
):
    """
    Delete a thread (cascade deletes messages).

    Args:
        thread_id: Thread ID
        db: Database session
        user_id: Authenticated user ID

    Raises:
        HTTPException: 404 if thread not found, 403 if not owned by user
    """
    try:
        await ThreadService.delete_thread(db, thread_id, user_id)
    except NotFoundError as e:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=str(e))
    except ForbiddenError as e:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail=str(e))
    except Exception as e:
        logger.error("delete_thread_error", error=str(e), thread_id=thread_id)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete thread"
        )
