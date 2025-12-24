"""
Structured logging setup using Structlog.

Provides JSON-formatted logs for production environments.
"""
import logging
import structlog
from app.config import settings


def setup_logging():
    """
    Configure structured logging with Structlog.

    Logs are formatted as JSON in production, human-readable in development.
    """
    # Configure logging level
    logging.basicConfig(
        format="%(message)s",
        level=getattr(logging, settings.LOG_LEVEL),
    )

    # Structlog processors
    processors = [
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.stdlib.PositionalArgumentsFormatter(),
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.processors.UnicodeDecoder(),
    ]

    # Add JSON processor for production, console for development
    if settings.ENVIRONMENT == "production":
        processors.append(structlog.processors.JSONRenderer())
    else:
        processors.append(structlog.dev.ConsoleRenderer())

    # Configure Structlog
    structlog.configure(
        processors=processors,
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        cache_logger_on_first_use=True,
    )


# Initialize logging on module import
setup_logging()


def get_logger(name: str = __name__) -> structlog.BoundLogger:
    """
    Get a structured logger instance.

    Usage:
        from app.utils.logger import get_logger
        logger = get_logger(__name__)

        logger.info("user_action", user_id="123", action="create_thread")
        logger.error("api_error", error="Gemini timeout", retry_count=3)

    Args:
        name: Logger name (typically __name__)

    Returns:
        structlog.BoundLogger: Configured logger
    """
    return structlog.get_logger(name)
