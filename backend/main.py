"""
FastAPI application entrypoint for RAG Chatbot Backend API.

Includes:
- Application lifecycle management
- Health check endpoint
- CORS middleware configuration
- Comprehensive error handling with user-friendly messages
"""

from fastapi import FastAPI, status, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from pydantic import ValidationError
import logging

from database import lifespan
from config import settings
from models import ErrorResponse, ChatRequest, ChatResponse, Citation, MessageRole, SafetyCheckOutput
from middleware import register_middleware
from session import get_or_create_session, add_message, get_conversation_context
from chatbot_agents import create_chatbot_agent, extract_citations_from_response, run_config
from agents import Runner, InputGuardrailTripwireTriggered
import time


# ========== Logging Setup ==========

logger = logging.getLogger("chatbot-backend")


# ========== Custom Exceptions ==========

class ChatbotError(Exception):
    """Base exception for chatbot-specific errors."""

    def __init__(self, message: str, code: str, details: dict = None):
        self.message = message
        self.code = code
        self.details = details or {}
        super().__init__(self.message)


# ========== Error Code Mappings ==========

ERROR_MESSAGES = {
    # Database Errors (ERR_DB_001-099)
    "ERR_DB_001": "We're having trouble connecting to our knowledge base. Please try again.",
    "ERR_DB_002": "We're having trouble accessing user profile. Please try again.",
    "ERR_DB_003": "Database operation timed out. Please try again.",

    # Authentication Errors (ERR_AUTH_001-099)
    "ERR_AUTH_001": "Authentication required. Please log in to continue.",
    "ERR_AUTH_002": "Invalid or expired authentication token.",
    "ERR_AUTH_003": "You don't have permission to access this resource.",

    # Agent/LLM Errors (ERR_AGENT_001-099)
    "ERR_AGENT_001": "The chatbot is temporarily unavailable. Please try again in a moment.",
    "ERR_AGENT_002": "We received too many requests. Please wait a moment and try again.",
    "ERR_AGENT_003": "Failed to generate response. Please try again.",

    # Tool/Knowledge Retrieval Errors (ERR_TOOL_001-099)
    "ERR_TOOL_001": "Could not retrieve textbook content. Please try rephrasing your question.",
    "ERR_TOOL_002": "Embedding service is temporarily unavailable. Please try again.",
    "ERR_TOOL_003": "No relevant content found for your question.",

    # Input Validation Errors (ERR_VAL_001-099)
    "ERR_VAL_001": "Your message appears to be empty. Please ask a question.",
    "ERR_VAL_002": "Your message is too long. Please keep it under 2000 characters.",
    "ERR_VAL_003": "Invalid request format. Please check your input.",
    "ERR_VAL_004": "Your question contains inappropriate content. Please rephrase.",
    "ERR_VAL_005": "Your question is not related to the course content.",
}


# Create FastAPI app with lifespan context manager
app = FastAPI(
    title="RAG Chatbot Backend API",
    version="1.0.0",
    description="RAG-powered chatbot backend using Gemini 2.0 Flash, Cohere embeddings, and Qdrant vector database",
    lifespan=lifespan
)


# ========== CORS Configuration ==========

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register additional middleware (rate limiting, logging)
register_middleware(app)


# ========== Exception Handlers ==========

@app.exception_handler(ChatbotError)
async def chatbot_error_handler(request: Request, exc: ChatbotError):
    """
    Handle custom ChatbotError exceptions.

    Args:
        request: Incoming request
        exc: ChatbotError exception

    Returns:
        JSONResponse with ErrorResponse format
    """
    logger.error(
        f"ChatbotError: {exc.code} - {exc.message}",
        extra={
            "code": exc.code,
            "details": exc.details,
            "path": request.url.path
        }
    )

    return JSONResponse(
        status_code=status.HTTP_400_BAD_REQUEST,
        content=ErrorResponse(
            error=exc.message,
            code=exc.code,
            details=exc.details
        ).model_dump()
    )


@app.exception_handler(RequestValidationError)
async def validation_error_handler(request: Request, exc: RequestValidationError):
    """
    Handle Pydantic validation errors from request body.

    Args:
        request: Incoming request
        exc: RequestValidationError exception

    Returns:
        JSONResponse with user-friendly error message
    """
    logger.warning(
        f"Validation error: {exc.errors()}",
        extra={
            "errors": exc.errors(),
            "path": request.url.path
        }
    )

    return JSONResponse(
        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
        content=ErrorResponse(
            error="Invalid request format. Please check your input.",
            code="ERR_VAL_003",
            details={"validation_errors": exc.errors()}
        ).model_dump()
    )


@app.exception_handler(ValidationError)
async def pydantic_validation_error_handler(request: Request, exc: ValidationError):
    """
    Handle Pydantic validation errors from models.

    Args:
        request: Incoming request
        exc: ValidationError exception

    Returns:
        JSONResponse with user-friendly error message
    """
    logger.warning(
        f"Pydantic validation error: {exc.errors()}",
        extra={
            "errors": exc.errors(),
            "path": request.url.path
        }
    )

    return JSONResponse(
        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
        content=ErrorResponse(
            error="Invalid data format. Please check your input.",
            code="ERR_VAL_003",
            details={"validation_errors": exc.errors()}
        ).model_dump()
    )


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """
    Handle unexpected exceptions.

    Args:
        request: Incoming request
        exc: Exception

    Returns:
        JSONResponse with generic error message
    """
    logger.error(
        f"Unexpected error: {str(exc)}",
        extra={
            "exception": str(exc),
            "type": type(exc).__name__,
            "path": request.url.path
        },
        exc_info=True
    )

    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content=ErrorResponse(
            error="An unexpected error occurred. Please try again later.",
            code="ERR_INTERNAL_001",
            details={"type": type(exc).__name__}
        ).model_dump()
    )


# ========== Chat Endpoint ==========

@app.post(
    "/v1/chat",
    status_code=status.HTTP_200_OK,
    response_model=ChatResponse,
    tags=["Chat"]
)
async def chat(request: ChatRequest):
    """
    Send a chat message and receive AI-generated response using Runner.run().

    Process:
    1. Get or create conversation session
    2. Add user message to session
    3. Run agent with Runner.run() (guardrail runs automatically)
    4. Handle InputGuardrailTripwireTriggered exception
    5. Extract final_output from RunResult
    6. Add assistant response to session
    7. Return response with citations

    Args:
        request: ChatRequest with message, user_id, optional session_id

    Returns:
        ChatResponse with answer, citations, and metadata

    Raises:
        ChatbotError: If validation fails or processing error occurs
        InputGuardrailTripwireTriggered: If query blocked by guardrail
    """
    start_time = time.time()

    try:
        # Step 1: Get or create session
        session = get_or_create_session(request.user_id, request.session_id)

        # Step 2: Add user message to session
        add_message(session, MessageRole.USER, request.message)

        # Step 3: Create chatbot agent
        agent = create_chatbot_agent()

        # Step 4: Get full conversation context (includes system message + history)
        conversation_messages = get_conversation_context(session)

        # Step 5: Run agent with Runner.run()
        # NOTE: Input guardrail runs automatically via @input_guardrail decorator
        # Passing full conversation history for context-aware responses
        try:
            result = await Runner.run(
                starting_agent=agent,
                input=conversation_messages,  # Full conversation history
                run_config=run_config
            )

            # Extract final output from RunResult
            assistant_message = result.final_output

        except InputGuardrailTripwireTriggered as e:
            print("\n\n\n",f"InputGuardrailTripwireTriggered as e:{e}","\n\n\n")
            # Query blocked by guardrail - extract SafetyCheckOutput
            safety_output: SafetyCheckOutput = e.guardrail_result.output_info

            logger.warning(
                f"Guardrail blocked query: {safety_output.reason}",
                extra={
                    "user_id": request.user_id,
                    "is_safe": safety_output.is_safe,
                    "is_relevant": safety_output.is_relevant,
                    "reason": safety_output.reason
                }
            )

            # Return appropriate error based on reason
            if not safety_output.is_safe:
                raise ChatbotError(
                    message="Your question contains inappropriate content. Please rephrase.",
                    code="ERR_VAL_004"
                )
            elif not safety_output.is_relevant:
                raise ChatbotError(
                    message="I can only answer questions about the Physical AI and Humanoid Robotics textbook. Please ask about course content.",
                    code="ERR_VAL_005"
                )
            else:
                # Should not reach here, but handle gracefully
                raise ChatbotError(
                    message="Your question could not be processed. Please try rephrasing.",
                    code="ERR_VAL_001"
                )

        except Exception as e:
            logger.error(f"Agent error: {str(e)}", exc_info=True)
            raise ChatbotError(
                message="The chatbot is temporarily unavailable. Please try again in a moment.",
                code="ERR_AGENT_001"
            )

        # Step 5: Add assistant response to session
        add_message(session, MessageRole.ASSISTANT, assistant_message)

        # Step 6: Extract citations from response
        citations = extract_citations_from_response(assistant_message)

        # Convert to Citation objects
        citation_objects = [
            Citation(**cite) for cite in citations
        ]

        # Step 7: Calculate metrics
        processing_time_ms = int((time.time() - start_time) * 1000)
        token_count = session.token_count

        # Step 8: Return response
        return ChatResponse(
            response=assistant_message,
            session_id=session.session_id,
            citations=citation_objects,
            confidence_score=None,  # Optional for now
            processing_time_ms=processing_time_ms,
            token_count=token_count
        )

    except ChatbotError:
        # Re-raise ChatbotError to be handled by exception handler
        raise
    except ValueError as e:
        # User not found in database
        logger.error(f"User profile error: {str(e)}")
        raise ChatbotError(
            message="We're having trouble accessing your profile. Please try again.",
            code="ERR_DB_002"
        )
    except Exception as e:
        # Unexpected error
        logger.error(f"Unexpected error in chat endpoint: {str(e)}", exc_info=True)
        raise


# ========== Health Check Endpoint ==========

@app.get(
    "/v1/health",
    status_code=status.HTTP_200_OK,
    response_model=dict,
    tags=["Health"]
)
async def health_check():
    """
    Health check endpoint.

    Returns:
        JSON response with status and version

    Status Codes:
        200: Service is healthy
    """
    return {
        "status": "healthy",
        "version": "1.0.0",
        "service": "rag-chatbot-backend"
    }


# ========== Root Endpoint ==========

@app.get("/", tags=["Root"])
async def root():
    """
    Root endpoint with API information.

    Returns:
        JSON response with API details
    """
    return {
        "message": "RAG Chatbot Backend API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/v1/health"
    }
