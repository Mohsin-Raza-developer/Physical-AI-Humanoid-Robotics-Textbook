"""
Rate limiting middleware using SlowAPI.

Prevents API abuse by limiting request rates per user.
"""
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from fastapi import Request
from app.utils.logger import get_logger

logger = get_logger(__name__)


def get_user_id_or_ip(request: Request) -> str:
    """
    Get user ID from JWT token or fall back to IP address.

    Args:
        request: FastAPI request object

    Returns:
        str: User identifier for rate limiting
    """
    try:
        # Try to extract user_id from Authorization header
        auth_header = request.headers.get("Authorization", "")
        if auth_header.startswith("Bearer "):
            token = auth_header.replace("Bearer ", "")
            # Parse JWT to get user_id (simplified - in production use proper JWT decode)
            # For now, use IP as fallback
            return get_remote_address(request)
        return get_remote_address(request)
    except Exception:
        return get_remote_address(request)


# Create limiter instance
limiter = Limiter(
    key_func=get_user_id_or_ip,
    default_limits=["100/minute"],  # Global default
    storage_uri="memory://",  # In-memory storage (use Redis in production)
    headers_enabled=True  # Add rate limit headers to responses
)


# Rate limit decorators for specific endpoints
THREAD_CREATION_LIMIT = "5/minute"  # Max 5 threads per minute
MESSAGE_SEND_LIMIT = "10/minute"  # Max 10 messages per minute
THREAD_LIST_LIMIT = "30/minute"  # Max 30 list requests per minute


def setup_rate_limiting(app):
    """
    Set up rate limiting for the FastAPI app.

    Args:
        app: FastAPI application instance
    """
    app.state.limiter = limiter
    app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

    logger.info("rate_limiting_configured", limits={
        "thread_creation": THREAD_CREATION_LIMIT,
        "message_send": MESSAGE_SEND_LIMIT,
        "thread_list": THREAD_LIST_LIMIT
    })
