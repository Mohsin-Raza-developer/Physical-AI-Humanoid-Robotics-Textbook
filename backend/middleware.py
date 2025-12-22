"""
Middleware configuration for RAG Chatbot Backend API.

Includes:
- Rate limiting using slowapi (20 req/min per user)
- Structured logging with correlation IDs
- Request/response timing
"""

import time
import uuid
import logging
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware

from config import settings


# ========== Rate Limiting ==========

def get_user_id_from_request(request: Request) -> str:
    """
    Extract user ID from request for rate limiting.

    For authenticated requests, use user_id from JWT token or request body.
    For unauthenticated requests, fall back to IP address.

    Args:
        request: FastAPI Request object

    Returns:
        User identifier string for rate limiting
    """
    # TODO: Extract user_id from JWT token when authentication is implemented
    # For now, use remote address as fallback
    return get_remote_address(request)


limiter = Limiter(
    key_func=get_user_id_from_request,
    default_limits=[f"{settings.rate_limit_per_minute}/minute"]
)


# ========== Structured Logging ==========

# Configure structured logging
logging.basicConfig(
    level=settings.log_level,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger("chatbot-backend")


class LoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware for structured logging with correlation IDs.

    Logs:
    - Request method, path, headers
    - Response status code, processing time
    - Correlation ID for request tracing
    """

    async def dispatch(self, request: Request, call_next):
        """
        Process request and log details.

        Args:
            request: Incoming request
            call_next: Next middleware/handler in chain

        Returns:
            Response with correlation ID header
        """
        # Generate correlation ID
        correlation_id = str(uuid.uuid4())
        request.state.correlation_id = correlation_id

        # Log request
        logger.info(
            f"[{correlation_id}] {request.method} {request.url.path}",
            extra={
                "correlation_id": correlation_id,
                "method": request.method,
                "path": request.url.path,
                "client_ip": request.client.host if request.client else None
            }
        )

        # Track processing time
        start_time = time.time()

        # Process request
        response = await call_next(request)

        # Calculate processing time
        processing_time_ms = int((time.time() - start_time) * 1000)

        # Log response
        logger.info(
            f"[{correlation_id}] {response.status_code} - {processing_time_ms}ms",
            extra={
                "correlation_id": correlation_id,
                "status_code": response.status_code,
                "processing_time_ms": processing_time_ms
            }
        )

        # Add correlation ID to response headers
        response.headers["X-Correlation-ID"] = correlation_id
        response.headers["X-Processing-Time-Ms"] = str(processing_time_ms)

        return response


# ========== Middleware Registration Helper ==========

def register_middleware(app):
    """
    Register all middleware on FastAPI app.

    Args:
        app: FastAPI application instance

    Usage:
        from middleware import register_middleware
        register_middleware(app)
    """
    # Add logging middleware
    app.add_middleware(LoggingMiddleware)

    # Register rate limiter
    app.state.limiter = limiter
    app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

    logger.info(" Middleware registered: Logging, Rate Limiting")
