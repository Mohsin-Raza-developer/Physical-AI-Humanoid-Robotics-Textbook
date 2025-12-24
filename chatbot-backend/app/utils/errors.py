"""
Custom exception classes for the application.

Provides structured error handling with specific exception types.
"""


class ChatbotBaseException(Exception):
    """Base exception for all chatbot-related errors."""

    def __init__(self, message: str, code: str = None):
        self.message = message
        self.code = code or self.__class__.__name__
        super().__init__(self.message)


class AuthenticationError(ChatbotBaseException):
    """
    Raised when authentication fails.

    Examples:
        - Invalid JWT token
        - Expired token
        - Missing authentication header
    """

    def __init__(self, message: str = "Authentication failed"):
        super().__init__(message, code="ERR_AUTH_INVALID")


class NotFoundError(ChatbotBaseException):
    """
    Raised when a requested resource is not found.

    Examples:
        - Thread not found
        - Message not found
        - User not found
    """

    def __init__(self, resource: str, identifier: str = None):
        message = f"{resource} not found"
        if identifier:
            message = f"{resource} with ID '{identifier}' not found"
        super().__init__(message, code="ERR_NOT_FOUND")


class ValidationError(ChatbotBaseException):
    """
    Raised when input validation fails.

    Examples:
        - Empty message content
        - Message exceeds max length
        - Invalid pagination parameters
    """

    def __init__(self, field: str, message: str):
        full_message = f"Validation error for '{field}': {message}"
        super().__init__(full_message, code="ERR_VALIDATION")


class ForbiddenError(ChatbotBaseException):
    """
    Raised when user doesn't have permission to access a resource.

    Examples:
        - Accessing another user's thread
        - Deleting a message you don't own
    """

    def __init__(self, message: str = "Access forbidden"):
        super().__init__(message, code="ERR_FORBIDDEN")


class RateLimitError(ChatbotBaseException):
    """
    Raised when rate limit is exceeded.

    Examples:
        - Too many messages sent in a minute
        - Too many threads created
    """

    def __init__(self, message: str = "Rate limit exceeded"):
        super().__init__(message, code="ERR_RATE_LIMIT")


class ExternalServiceError(ChatbotBaseException):
    """
    Raised when external service fails.

    Examples:
        - Gemini API timeout
        - Qdrant connection error
        - Cohere API rate limit
    """

    def __init__(self, service: str, message: str):
        full_message = f"{service} error: {message}"
        super().__init__(full_message, code="ERR_EXTERNAL_SERVICE")


class DatabaseError(ChatbotBaseException):
    """
    Raised when database operation fails.

    Examples:
        - Connection timeout
        - Constraint violation
        - Transaction rollback
    """

    def __init__(self, message: str = "Database operation failed"):
        super().__init__(message, code="ERR_DATABASE")
