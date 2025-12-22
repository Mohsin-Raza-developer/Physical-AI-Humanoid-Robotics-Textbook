"""
Pydantic data models for RAG Chatbot Backend API.

Includes:
- Request/Response models (ChatRequest, ChatResponse, ErrorResponse)
- Domain entities (ChatMessage, ConversationSession, Citation)
- Enums (MessageRole)
"""

from pydantic import BaseModel, Field, field_validator
from enum import Enum
from datetime import datetime
from dataclasses import dataclass
from typing import Optional


# ========== Enums ==========

class MessageRole(str, Enum):
    """Message sender role in conversation."""
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"





# ========== API Request/Response Models ==========

class ChatRequest(BaseModel):
    """Request model for POST /v1/chat endpoint."""

    message: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question or message"
    )

    user_id: str = Field(
        ...,
        pattern=r"^[a-zA-Z0-9_-]+$",
        description="Authenticated user identifier"
    )

    session_id: Optional[str] = Field(
        None,
        description="Optional session ID for continuing conversation"
    )

    @field_validator('message')
    @classmethod
    def validate_message_content(cls, v: str) -> str:
        """Validate message is not empty or whitespace only."""
        if not v.strip():
            raise ValueError("Message cannot be empty or whitespace only")
        return v.strip()

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "message": "What is ROS 2 architecture?",
                "user_id": "user_12345",
                "session_id": "sess_abc123"
            }]
        }
    }


class Citation(BaseModel):
    """Source citation with clickable link."""

    chapter_title: str = Field(..., description="Display name of source chapter")
    doc_url: str = Field(..., description="Docusaurus URL path")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Search relevance (0-1)")

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "chapter_title": "ROS 2 Architecture Overview",
                "doc_url": "/docs/module-1/week-3/ros2-architecture",
                "relevance_score": 0.92
            }]
        }
    }


class ChatResponse(BaseModel):
    """Response model for POST /v1/chat endpoint."""

    response: str = Field(..., description="Agent's answer with inline citations")
    session_id: str = Field(..., description="Session ID for conversation continuity")
    citations: list[Citation] = Field(default_factory=list, description="Source references")
    confidence_score: Optional[float] = Field(None, ge=0.0, le=1.0, description="Response confidence (0-1)")
    processing_time_ms: int = Field(..., description="Total processing time in milliseconds")
    token_count: int = Field(..., description="Total tokens in conversation context")

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "response": "ROS 2 (Robot Operating System 2) is a middleware framework... [ROS 2 Architecture](/docs/module-1/week-3/ros2-architecture)",
                "session_id": "sess_abc123",
                "citations": [
                    {
                        "chapter_title": "ROS 2 Architecture Overview",
                        "doc_url": "/docs/module-1/week-3/ros2-architecture",
                        "relevance_score": 0.92
                    }
                ],
                "confidence_score": 0.88,
                "processing_time_ms": 1245,
                "token_count": 523
            }]
        }
    }


class ErrorResponse(BaseModel):
    """Standard error response format."""

    error: str = Field(..., description="User-friendly error message")
    code: str = Field(..., description="Error code for support/debugging")
    details: Optional[dict] = Field(None, description="Optional additional context")

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "error": "We're having trouble connecting to our knowledge base. Please try again.",
                "code": "ERR_DB_001",
                "details": {"retry_after_seconds": 30}
            }]
        }
    }


# ========== Domain Entities ==========

class ChatMessage(BaseModel):
    """Individual message in conversation history."""

    role: MessageRole = Field(..., description="Message sender (system/user/assistant)")
    content: str = Field(..., min_length=1, description="Message text content")
    timestamp: Optional[datetime] = Field(None, description="Message creation time")

    # model_config = {
    #     "json_schema_extra": {
    #         "examples": [
    #             {
    #                 "role": "system",
    #                 "content": "You are a helpful robotics tutor. Student: John Doe, Level: intermediate",
    #                 "timestamp": "2025-12-20T14:30:00Z"
    #             },
    #             {
    #                 "role": "user",
    #                 "content": "What is ROS 2?",
    #                 "timestamp": "2025-12-20T14:30:15Z"
    #             },
    #             {
    #                 "role": "assistant",
    #                 "content": "ROS 2 is... [Citation](/docs/module-1/ros2)",
    #                 "timestamp": "2025-12-20T14:30:18Z"
    #             }
    #         ]
    #     }
    # }





class ConversationSession(BaseModel):
    """Active conversation session state."""

    session_id: str = Field(..., description="Unique session identifier")
    user_id: str = Field(..., description="User who owns this session")
    messages: list[ChatMessage] = Field(default_factory=list, description="Conversation history")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Session start time")
    last_activity: datetime = Field(default_factory=datetime.utcnow, description="Last message timestamp")
    token_count: int = Field(default=0, description="Total tokens in context window")
    is_active: bool = Field(default=True, description="Session active status")

    def add_message(self, message: ChatMessage) -> None:
        """
        Add message to conversation history.

        Args:
            message: ChatMessage to append to history
        """
        self.messages.append(message)
        self.last_activity = datetime.utcnow()

    def get_messages_for_agent(self) -> list[dict]:
        """
        Convert to format expected by OpenAI SDK.

        Returns:
            List of message dicts with 'role' and 'content' keys
        """
        return [{"role": msg.role.value, "content": msg.content} for msg in self.messages]

    def estimate_tokens(self) -> int:
        """
        Rough token count estimation (4 chars H 1 token).

        Returns:
            Estimated token count for all messages
        """
        total_chars = sum(len(msg.content) for msg in self.messages)
        return total_chars // 4

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "session_id": "sess_abc123",
                "user_id": "user_12345",
                "messages": [],
                "created_at": "2025-12-20T14:30:00Z",
                "last_activity": "2025-12-20T14:35:42Z",
                "token_count": 523,
                "is_active": True
            }]
        }
    }


class KnowledgeChunk(BaseModel):
    """Search result from Qdrant vector database."""

    content: str = Field(..., description="Textbook content snippet")
    source_file: str = Field(..., description="Source file path in repository")
    chapter_title: str = Field(..., description="Chapter/section title")
    module: Optional[str] = Field(None, description="Module identifier (e.g., 'module-1')")
    week: Optional[str] = Field(None, description="Week identifier (e.g., 'week-3')")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Search similarity score")

    def to_docusaurus_url(self) -> str:
        """
        Convert source file path to Docusaurus URL.

        Example:
            content/docs/module-1/week-3/ros2.md � /docs/module-1/week-3/ros2

        Returns:
            Docusaurus-formatted URL path
        """
        path = self.source_file.replace("content/docs/", "/docs/")
        path = path.replace(".md", "")
        return path

    def to_citation(self) -> Citation:
        """
        Convert to API citation format.

        Returns:
            Citation object for API response
        """
        return Citation(
            chapter_title=self.chapter_title,
            doc_url=self.to_docusaurus_url(),
            relevance_score=self.relevance_score
        )

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "content": "ROS 2 uses a DDS (Data Distribution Service) middleware layer...",
                "source_file": "content/docs/module-1/week-3/ros2-architecture.md",
                "chapter_title": "ROS 2 Architecture Overview",
                "module": "module-1",
                "week": "week-3",
                "relevance_score": 0.92
            }]
        }
    }

# @dataclass
class SafetyCheckOutput(BaseModel):
    """
    Output model for safety guardrail agent validation.

    This is the output_type for the guardrail agent that validates
    user queries for safety and relevance before processing.
    """
    is_safe: bool = Field(...,description="Whether query contains safe, appropriate content")
    is_relevant: bool = Field(...,description="Whether query is relevant to robotics textbook content")
    reason: str = Field(...,description="Explanation of validation decision")

    # model_config = {
    #     "json_schema_extra": {
    #         "examples": [
    #             {
    #                 "is_safe": True,
    #                 "is_relevant": True,
    #                 "reason": "Query asks about ROS 2 architecture - safe and relevant to textbook"
    #             },
    #             {
    #                 "is_safe": True,
    #                 "is_relevant": False,
    #                 "reason": "Query asks about weather - safe but not relevant to robotics course"
    #             },
    #             {
    #                 "is_safe": False,
    #                 "is_relevant": False,
    #                 "reason": "Query contains inappropriate content"
    #             }
    #         ]
    #     }
    # }
