"""
Session management for RAG Chatbot Backend API.

Handles:
- Creating and retrieving conversation sessions
- Managing conversation history with user profile context
- Token counting and context window management
"""

import uuid
from datetime import datetime, timedelta
from typing import Dict, Optional
from models import ConversationSession, ChatMessage, MessageRole

from config import settings


# In-memory session store (replace with Redis for production)
active_sessions: Dict[str, ConversationSession] = {}


def generate_session_id() -> str:
    """
    Generate a unique session identifier.

    Returns:
        Session ID in format 'sess_<uuid>'
    """
    return f"sess_{uuid.uuid4().hex[:12]}"


def create_session(user_id: str) -> ConversationSession:
    """
    Create a new conversation session.

    Initializes a session with a generic system message.

    Args:
        user_id: User identifier

    Returns:
        New ConversationSession instance
    """
    # Generic system message
    system_message_content = (
        "You are a helpful robotics tutor for the Physical AI and Humanoid Robotics course. "
        "Answer questions based on textbook content retrieved through search_knowledge_base tool."
    )

    # Create system message
    system_message = ChatMessage(
        role=MessageRole.SYSTEM,
        content=system_message_content,
        timestamp=datetime.utcnow()
    )

    # Create new session
    session_id = generate_session_id()
    session = ConversationSession(
        session_id=session_id,
        user_id=user_id,
        messages=[system_message],
        created_at=datetime.utcnow(),
        last_activity=datetime.utcnow(),
        token_count=len(system_message_content) // 4,  # Rough estimate
        is_active=True
    )

    # Store in memory
    active_sessions[session_id] = session

    return session


def get_session(session_id: str) -> Optional[ConversationSession]:
    """
    Retrieve an existing session.

    Args:
        session_id: Session identifier

    Returns:
        ConversationSession if found and active, None otherwise
    """
    session = active_sessions.get(session_id)

    if session and session.is_active:
        return session

    return None


def get_or_create_session(user_id: str, session_id: Optional[str] = None) -> ConversationSession:
    """
    Get existing session or create new one.

    Args:
        user_id: User identifier
        session_id: Optional session ID for continuing conversation

    Returns:
        ConversationSession instance
    """
    # Try to get existing session
    if session_id:
        session = get_session(session_id)
        if session and session.user_id == user_id:
            return session

    # Create new session
    return create_session(user_id)


def add_message(session: ConversationSession, role: MessageRole, content: str) -> None:
    """
    Add a message to the conversation session.

    Args:
        session: ConversationSession instance
        role: Message role (user or assistant)
        content: Message content
    """
    message = ChatMessage(
        role=role,
        content=content,
        timestamp=datetime.utcnow()
    )

    session.add_message(message)
    session.token_count = session.estimate_tokens()


def get_conversation_context(session: ConversationSession) -> list[dict]:
    """
    Get full conversation context for agent.

    Returns messages array with system message as first element.

    Args:
        session: ConversationSession instance

    Returns:
        List of message dicts with 'role' and 'content' keys
    """
    return session.get_messages_for_agent()


def cleanup_inactive_sessions() -> int:
    """
    Remove sessions inactive for longer than timeout.

    Returns:
        Number of sessions removed
    """
    cutoff = datetime.utcnow() - timedelta(hours=settings.session_timeout_hours)

    to_remove = [
        sid for sid, session in active_sessions.items()
        if session.last_activity < cutoff
    ]

    for sid in to_remove:
        del active_sessions[sid]

    return len(to_remove)


def end_session(session_id: str) -> bool:
    """
    End an active session.

    Args:
        session_id: Session identifier

    Returns:
        True if session was ended, False if not found
    """
    session = active_sessions.get(session_id)

    if session:
        session.is_active = False
        del active_sessions[session_id]
        return True

    return False
