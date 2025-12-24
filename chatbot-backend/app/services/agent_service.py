"""
AI agent service using Google Gemini and OpenAI Agents SDK.

Provides chat agent with knowledge base search capability.
"""
from agents import Agent, AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig, Runner
from app.config import settings
from app.services.search_tool import search_knowledge_base
from app.utils.logger import get_logger
from app.utils.errors import ExternalServiceError
import asyncio
from tenacity import (
    retry,
    stop_after_attempt,
    wait_exponential,
    retry_if_exception_type,
    before_sleep_log
)
import httpx

logger = get_logger(__name__)

# Configure Gemini client via AsyncOpenAI adapter
gemini_client = AsyncOpenAI(
    api_key=settings.GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Configure Gemini model
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=gemini_client
)

# Run configuration
config = RunConfig(
    model=model,
    tracing_disabled=True  # Disable tracing for production
)

# Main chatbot agent
chat_agent = Agent(
    name="RoboticsAssistant",
    instructions="""You are a helpful AI tutor for the Physical AI and Humanoid Robotics course.

Your role:
- Answer questions about robotics, ROS 2, Gazebo, Isaac Sim, and related topics
- Use the search_knowledge_base tool to find relevant information from the textbook
- Provide clear, educational explanations suitable for students
- Break down complex concepts into understandable steps
- Reference specific textbook content when available

Guidelines:
- Always search the knowledge base before answering technical questions
- Cite sources when using textbook content
- If information is not in the knowledge base, acknowledge it and provide general guidance
- Be encouraging and supportive to learners
- Use examples and analogies to clarify difficult concepts""",
    tools=[search_knowledge_base]
)


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10),
    retry=retry_if_exception_type((httpx.TimeoutException, httpx.ConnectError, httpx.HTTPStatusError)),
    before_sleep=before_sleep_log(logger, "WARNING"),
    reraise=True
)
async def generate_response(
    user_message: str,
    conversation_history: list[dict] = None
) -> str:
    """
    Generate AI response using Gemini agent with knowledge base search.

    Args:
        user_message: User's input message
        conversation_history: Previous messages (optional, for context)

    Returns:
        str: AI-generated response

    Raises:
        ExternalServiceError: If Gemini API fails after retries
    """
    try:
        logger.info(
            "generating_response",
            message_length=len(user_message),
            has_history=bool(conversation_history)
        )

        # Run agent with user message
        result = await Runner.run(
            starting_agent=chat_agent,
            input=user_message,
            run_config=config
        )

        logger.info(
            "response_generated",
            response_length=len(result.final_output) if result.final_output else 0
        )

        return result.final_output

    except (httpx.TimeoutException, httpx.ConnectError, httpx.HTTPStatusError) as e:
        # These will trigger retry
        logger.warning(
            "gemini_api_retryable_error",
            error=str(e),
            error_type=type(e).__name__
        )
        raise

    except Exception as e:
        # Non-retryable errors
        logger.error(
            "gemini_api_error",
            error=str(e),
            error_type=type(e).__name__
        )
        raise ExternalServiceError("Gemini", f"Failed to generate response: {str(e)}")


async def generate_response_stream(
    user_message: str,
    conversation_history: list[dict] = None
):
    """
    Generate AI response with streaming support (async generator).

    Args:
        user_message: User's input message
        conversation_history: Previous messages (optional)

    Yields:
        dict: Streaming events (text chunks, tool calls, etc.)

    Raises:
        ExternalServiceError: If Gemini API fails after retries
    """
    try:
        logger.info(
            "generating_streaming_response",
            message_length=len(user_message)
        )

        # For now, generate full response and yield it
        # TODO: Implement true streaming when Agents SDK supports it
        # Note: generate_response has retry logic built-in
        response = await generate_response(user_message, conversation_history)

        # Simulate streaming by chunking response
        words = response.split()
        for i in range(0, len(words), 3):  # Yield 3 words at a time
            chunk = " ".join(words[i:i+3]) + " "
            yield {"type": "content_delta", "delta": chunk}
            await asyncio.sleep(0.05)  # Small delay to simulate streaming

    except ExternalServiceError:
        # Re-raise service errors (already logged in generate_response)
        raise

    except Exception as e:
        # Unexpected streaming errors
        logger.error("streaming_error", error=str(e), error_type=type(e).__name__)
        raise ExternalServiceError("Gemini", f"Streaming failed: {str(e)}")
