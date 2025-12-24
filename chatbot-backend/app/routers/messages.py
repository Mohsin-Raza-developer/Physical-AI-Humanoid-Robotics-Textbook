"""
Message API endpoints with Server-Sent Events (SSE) streaming.

Provides message sending with real-time AI response streaming.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Query
from fastapi.responses import StreamingResponse
from sqlalchemy.ext.asyncio import AsyncSession
from app.database import get_db
from app.middleware.auth import get_current_user
from app.services.message_service import MessageService
from app.services.thread_service import ThreadService
from app.services.agent_service import generate_response_stream
from app.schemas.message import MessageCreate, MessageListResponse, MessageResponse
from app.utils.errors import NotFoundError, ForbiddenError
from app.utils.logger import get_logger
import json
import asyncio

logger = get_logger(__name__)

router = APIRouter(prefix="/api/threads", tags=["Messages"])


def format_sse_event(event_type: str, data: dict) -> str:
    """
    Format data as Server-Sent Event.

    Args:
        event_type: Event type name
        data: Event data dictionary

    Returns:
        str: Formatted SSE message
    """
    return f"event: {event_type}\ndata: {json.dumps(data)}\n\n"


@router.post("/{thread_id}/messages")
async def send_message(
    thread_id: str,
    message: MessageCreate,
    db: AsyncSession = Depends(get_db),
    user_id: str = Depends(get_current_user)
):
    """
    Send a message and receive streaming AI response via SSE.

    Args:
        thread_id: Thread ID
        message: Message content
        db: Database session
        user_id: Authenticated user ID

    Returns:
        StreamingResponse: SSE stream with AI response
    """

    async def event_generator():
        """Generate SSE events for streaming response."""
        try:
            # Verify thread ownership
            thread = await ThreadService.get_thread(db, thread_id, user_id)

            # Save user message to database
            user_message = await MessageService.create_message(
                db, thread_id, "user", message.content
            )

            logger.info(
                "user_message_saved",
                thread_id=thread_id,
                message_id=str(user_message.message_id)
            )

            # Get conversation history for context
            conversation_history = await MessageService.get_conversation_history(
                db, thread_id, max_messages=10
            )

            # Emit message_start event
            assistant_sequence = await MessageService.get_next_sequence_number(
                db, thread_id
            )

            yield format_sse_event("message_start", {
                "type": "message_start",
                "sequence_number": assistant_sequence
            })

            # Generate and stream AI response
            full_response = ""

            async for chunk_event in generate_response_stream(
                message.content,
                conversation_history
            ):
                if chunk_event["type"] == "content_delta":
                    full_response += chunk_event["delta"]
                    yield format_sse_event("content_delta", chunk_event)

            # Save assistant message to database
            assistant_message = await MessageService.create_message(
                db, thread_id, "assistant", full_response
            )

            logger.info(
                "assistant_message_saved",
                thread_id=thread_id,
                message_id=str(assistant_message.message_id),
                response_length=len(full_response)
            )

            # Emit message_end event
            yield format_sse_event("message_end", {
                "type": "message_end",
                "message_id": str(assistant_message.message_id)
            })

        except NotFoundError as e:
            yield format_sse_event("error", {
                "type": "error",
                "code": "ERR_NOT_FOUND",
                "message": str(e)
            })
        except ForbiddenError as e:
            yield format_sse_event("error", {
                "type": "error",
                "code": "ERR_FORBIDDEN",
                "message": str(e)
            })
        except Exception as e:
            logger.error(
                "streaming_error",
                error=str(e),
                thread_id=thread_id
            )
            yield format_sse_event("error", {
                "type": "error",
                "code": "ERR_INTERNAL",
                "message": "Failed to generate response"
            })

    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no"  # Disable nginx buffering
        }
    )


@router.get("/{thread_id}/messages", response_model=MessageListResponse)
async def get_messages(
    thread_id: str,
    limit: int = Query(default=50, ge=1, le=200),
    offset: int = Query(default=0, ge=0),
    db: AsyncSession = Depends(get_db),
    user_id: str = Depends(get_current_user)
):
    """
    Get paginated message history for a thread.

    Args:
        thread_id: Thread ID
        limit: Maximum messages to return (1-200)
        offset: Number of messages to skip
        db: Database session
        user_id: Authenticated user ID

    Returns:
        MessageListResponse: List of messages with total count
    """
    try:
        # Verify thread ownership
        await ThreadService.get_thread(db, thread_id, user_id)

        # Get messages
        messages, total = await MessageService.get_messages(
            db, thread_id, limit, offset
        )

        message_responses = [
            MessageResponse.from_orm(msg)
            for msg in messages
        ]

        return MessageListResponse(messages=message_responses, total=total)

    except NotFoundError as e:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=str(e))
    except ForbiddenError as e:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail=str(e))
    except Exception as e:
        logger.error("get_messages_error", error=str(e), thread_id=thread_id)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve messages"
        )
