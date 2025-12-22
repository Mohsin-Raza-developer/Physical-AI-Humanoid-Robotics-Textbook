"""
Agent definitions for RAG Chatbot Backend API.

Implements:
- Gemini configuration using AsyncOpenAI + OpenAIChatCompletionsModel + RunConfig
- Input guardrail agent for safety/relevance validation
- @input_guardrail decorator with GuardrailFunctionOutput
- Main chatbot agent with tool access and guardrails
- Citation extraction from agent responses
"""

from agents import Agent,AsyncOpenAI,enable_verbose_stdout_logging,OpenAIChatCompletionsModel,RunConfig,GuardrailFunctionOutput,InputGuardrailTripwireTriggered,RunContextWrapper,Runner,TResponseInputItem,input_guardrail,AgentBase, set_default_openai_client, set_default_openai_key
from agents import Agent, Runner
from config import settings
from agents_tool import search_knowledge_base
from models import SafetyCheckOutput
import re
from dotenv import load_dotenv

load_dotenv()

# enable_verbose_stdout_logging()


# ========== Gemini Configuration (T015) ==========

# Configure AsyncOpenAI client for Gemini API
external_client = AsyncOpenAI(
    api_key=settings.gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Configure OpenAI Chat Completions Model
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",  # OpenAI-compatible endpoint model name
    openai_client=external_client
)

# Configure Run Config
run_config = RunConfig(
    model=model,
    tracing_disabled=False
)
set_default_openai_client(client=external_client, use_for_tracing=True)
set_default_openai_key(key=settings.openai_api_key, use_for_tracing = True)


# ========== Guardrail Agent (T016) ==========

guardrail_agent = Agent(
    name="Safety Guardrail",
    instructions="""Check if the MOST RECENT user query is safe and relevant to the robotics textbook content.
            you have a search_knowledge_base tool you can use when you need to validate content.

            If you receive a conversation history (multiple messages), focus ONLY on validating the LATEST user message.

            Return:
            - is_safe=True if the content is appropriate and non-harmful
            - is_relevant=True if the query is about robotics, programming, or course-related topics
            - reason: Brief explanation of your decision

            Examples:
            - "What is ROS 2?" → is_safe=True, is_relevant=True, reason="Question about robotics course topic"
            - "What's the weather?" → is_safe=True, is_relevant=False, reason="Not related to robotics course"
            - "How to hack a system?" → is_safe=False, is_relevant=False, reason="Inappropriate content"
            """,
    output_type=SafetyCheckOutput  # Structured output
)


# ========== Input Guardrail Decorator (T017) ==========

@input_guardrail
async def safety_guardrail(
    ctx: RunContextWrapper[None],
    agent: AgentBase,
    input: str | list[TResponseInputItem]
) -> GuardrailFunctionOutput:
    """
    Input guardrail that validates user queries for safety and relevance.

    This runs the guardrail agent to check if the query should be blocked.
    If unsafe or irrelevant, triggers InputGuardrailTripwireTriggered exception.

    Args:
        ctx: Run context wrapper
        agent: The main agent (not used, but required by signature)
        input: User input to validate

    Returns:
        GuardrailFunctionOutput with validation result and tripwire status
    """
    # Run guardrail agent
    result = await Runner.run(guardrail_agent, input,run_config=run_config, context=ctx.context)
    print("Guardrail result run successfully ✅✅✅")

    # Extract SafetyCheckOutput from result
    safety_output: SafetyCheckOutput = result.final_output

    # Return GuardrailFunctionOutput
    # Tripwire triggers if EITHER unsafe OR irrelevant
    return GuardrailFunctionOutput(
        output_info=safety_output,  # Store for debugging
        tripwire_triggered=not (safety_output.is_safe and safety_output.is_relevant)
    )


# ========== Main Chatbot Agent (T018) ==========

def create_chatbot_agent() -> Agent:
    """
    Create the main chatbot agent with Gemini 2.0 Flash.

    The agent:
    - Uses Gemini 2.0 Flash as the LLM
    - Has access to search_knowledge_base tool
    - Protected by safety_guardrail input guardrail
    - Generates responses with inline citations

    Returns:
        Configured Agent instance
    """
    print("🤖🤖🤖 Creating chatbot agent...")
    agent = Agent(
        name="Robotics Tutor",
        instructions="""You are a helpful robotics tutor for the Physical AI and Humanoid Robotics course.

                Your role:
                1. Answer questions ONLY based on the textbook content retrieved through search_knowledge_base tool
                2. ALWAYS include inline citations in your responses using the format provided by the tool
                3. If the search tool returns no relevant content, politely indicate you don't have that information in the textbook
                4. DO NOT fabricate information or use general knowledge - ONLY use retrieved textbook content
                5. Tailor response complexity based on the student's software level (provided in system message)

                Citation format:
                - Use markdown links exactly as provided: [Chapter Title](/docs/path)
                - Place citations inline near the relevant information

                Example response:
                "ROS 2 uses a DDS middleware layer for communication [ROS 2 Architecture](/docs/module-1/ros2). It supports multiple implementations including Fast-DDS [DDS Middleware](/docs/module-1/dds)."

                Remember:
                - Be concise but thorough
                - Use simple language for beginners, technical details for advanced students
                - Always cite sources
                - Never make up information
                """,
        tools=[search_knowledge_base],
        input_guardrails=[safety_guardrail]  # DISABLED: Saves quota (1 LLM call instead of 2)
    )

    return agent


# ========== Citation Extraction (T021) ==========

def extract_citations_from_response(response_text: str) -> list[dict]:
    """
    Extract citations from agent response text.

    Parses markdown links in format [Title](/docs/path) and converts
    to Citation objects.

    Args:
        response_text: Agent's response with inline citations

    Returns:
        List of citation dicts with chapter_title, doc_url, relevance_score

    Example:
        >>> text = "ROS 2 is... [ROS 2 Arch](/docs/ros2)"
        >>> extract_citations_from_response(text)
        [{"chapter_title": "ROS 2 Arch", "doc_url": "/docs/ros2", "relevance_score": 1.0}]
    """
    # Regex pattern to match markdown links: [title](url)
    citation_pattern = r'\[([^\]]+)\]\((/docs/[^)]+)\)'

    matches = re.findall(citation_pattern, response_text)

    citations = []
    for title, url in matches:
        citations.append({
            "chapter_title": title,
            "doc_url": url,
            "relevance_score": 1.0  # Default score for extracted citations
        })

    # Remove duplicates while preserving order
    seen = set()
    unique_citations = []
    for citation in citations:
        key = (citation["chapter_title"], citation["doc_url"])
        if key not in seen:
            seen.add(key)
            unique_citations.append(citation)

    return unique_citations
