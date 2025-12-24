"""
Message Pydantic schemas for request/response validation.

Defines data transfer objects for message-related API operations.
"""
from pydantic import BaseModel, Field, UUID4, validator
from datetime import datetime
from typing import List, Literal


class MessageCreate(BaseModel):
    """Schema for creating a new message (user message)."""

    content: str = Field(
        ...,
        min_length=1,
        max_length=10000,
        description="Message content (1-10,000 characters)"
    )

    @validator("content")
    def validate_content(cls, v):
        """Validate message content is not empty or whitespace only."""
        if not v or not v.strip():
            raise ValueError("Message content cannot be empty or whitespace only")
        return v.strip()

    class Config:
        json_schema_extra = {
            "example": {
                "content": "What is inverse kinematics?"
            }
        }


class MessageResponse(BaseModel):
    """Schema for message response."""

    message_id: UUID4 = Field(..., description="Unique message identifier")
    thread_id: UUID4 = Field(..., description="Parent thread ID")
    role: Literal["user", "assistant"] = Field(..., description="Message author")
    content: str = Field(..., description="Message content")
    created_at: datetime = Field(..., description="Creation timestamp")
    sequence_number: int = Field(..., description="Message order in thread")

    class Config:
        from_attributes = True
        json_schema_extra = {
            "example": {
                "message_id": "123e4567-e89b-12d3-a456-426614174000",
                "thread_id": "987fcdeb-51a2-43f1-b123-123456789abc",
                "role": "user",
                "content": "What is inverse kinematics?",
                "created_at": "2025-12-24T10:00:00Z",
                "sequence_number": 1
            }
        }


class MessageListResponse(BaseModel):
    """Schema for paginated list of messages."""

    messages: List[MessageResponse] = Field(..., description="List of messages")
    total: int = Field(..., description="Total number of messages in thread")

    class Config:
        json_schema_extra = {
            "example": {
                "messages": [
                    {
                        "message_id": "123e4567-e89b-12d3-a456-426614174000",
                        "thread_id": "987fcdeb-51a2-43f1-b123-123456789abc",
                        "role": "user",
                        "content": "What is inverse kinematics?",
                        "created_at": "2025-12-24T10:00:00Z",
                        "sequence_number": 1
                    }
                ],
                "total": 42
            }
        }
