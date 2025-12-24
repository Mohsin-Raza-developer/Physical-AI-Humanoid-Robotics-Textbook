"""
Thread Pydantic schemas for request/response validation.

Defines data transfer objects for thread-related API operations.
"""
from pydantic import BaseModel, Field, UUID4
from datetime import datetime
from typing import Optional, List


class ThreadCreate(BaseModel):
    """Schema for creating a new thread."""

    title: Optional[str] = Field(
        None,
        max_length=255,
        description="Optional thread title (can be auto-generated later)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "title": "ROS 2 Setup Help"
            }
        }


class ThreadResponse(BaseModel):
    """Schema for thread response (single thread)."""

    thread_id: UUID4 = Field(..., description="Unique thread identifier")
    user_id: UUID4 = Field(..., description="Owner user ID")
    title: Optional[str] = Field(None, description="Thread title")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last activity timestamp")

    class Config:
        from_attributes = True
        json_schema_extra = {
            "example": {
                "thread_id": "123e4567-e89b-12d3-a456-426614174000",
                "user_id": "987fcdeb-51a2-43f1-b123-123456789abc",
                "title": "ROS 2 Setup Help",
                "created_at": "2025-12-24T10:00:00Z",
                "updated_at": "2025-12-24T11:00:00Z"
            }
        }


class ThreadListItem(BaseModel):
    """Schema for thread in list response (without messages)."""

    thread_id: UUID4
    title: Optional[str]
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class ThreadListResponse(BaseModel):
    """Schema for paginated list of threads."""

    threads: List[ThreadListItem] = Field(..., description="List of threads")
    total: int = Field(..., description="Total number of threads for this user")

    class Config:
        json_schema_extra = {
            "example": {
                "threads": [
                    {
                        "thread_id": "123e4567-e89b-12d3-a456-426614174000",
                        "title": "ROS 2 Setup Help",
                        "created_at": "2025-12-24T10:00:00Z",
                        "updated_at": "2025-12-24T11:00:00Z"
                    }
                ],
                "total": 150
            }
        }


class ThreadDetailResponse(BaseModel):
    """Schema for detailed thread response (with messages)."""

    thread_id: UUID4
    title: Optional[str]
    messages: List = Field(default_factory=list, description="List of messages in thread")

    class Config:
        from_attributes = True
