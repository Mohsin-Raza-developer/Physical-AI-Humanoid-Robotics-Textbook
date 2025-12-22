"""
Knowledge retrieval tools for RAG Chatbot Backend API.

Provides function tools for the agent to search the textbook knowledge base.
"""

from agents import function_tool
from database import get_qdrant_client, get_cohere_client
from config import settings
from models import KnowledgeChunk


@function_tool
def search_knowledge_base(query: str) -> str:
    """
    Searches the robotics textbook knowledge base for relevant content.

    This tool:
    1. Embeds the query using Cohere embed-v4.0 (1536 dimensions)
    2. Searches Qdrant vector database for top 5 relevant chunks
    3. Formats results with Docusaurus-formatted citations

    Args:
        query: The search query (question or topic to find information about)

    Returns:
        Formatted context string with relevant textbook sections and clickable
        Docusaurus citations (e.g., [Chapter Title](/docs/path))

    Example:
        >>> search_knowledge_base("What is ROS 2 architecture?")
        "ROS 2 uses a DDS middleware layer...

        Source: [ROS 2 Architecture](/docs/module-1/week-3/ros2-architecture)

        ---

        ROS 2 supports multiple DDS implementations...

        Source: [DDS Middleware](/docs/module-1/week-4/dds-middleware)"
    """
    # Get clients
    cohere = get_cohere_client()
    qdrant = get_qdrant_client()

    # Step 1: Embed query using Cohere
    embedding_response = cohere.embed(
        texts=[query],
        model="embed-v4.0",
        input_type="search_query"
    )
    query_embedding = embedding_response.embeddings[0]  # 1536-dim vector

    # Step 2: Search Qdrant vector database
    search_results = qdrant.query_points(
        collection_name="robotics_textbook_v1",
        query=query_embedding,
        limit=5,
        score_threshold=0.4  # Minimum relevance score (lowered for better recall)
    ).points

    # Step 3: Format results with citations
    if not search_results:
        return "No relevant content found in the textbook for this question."

    formatted_chunks = []
    for result in search_results:
        # Extract payload metadata
        content = result.payload.get("content", "")
        source_file = result.payload.get("source_file", "")
        chapter_title = result.payload.get("chapter_title", "")
        module = result.payload.get("module")
        week = result.payload.get("week")
        relevance_score = result.score

        # Create KnowledgeChunk
        chunk = KnowledgeChunk(
            content=content,
            source_file=source_file,
            chapter_title=chapter_title,
            module=module,
            week=week,
            relevance_score=relevance_score
        )

        # Format with Docusaurus citation
        doc_url = chunk.to_docusaurus_url()
        citation = f"[{chapter_title}]({doc_url})"

        # Truncate content to first 300 characters for concise responses
        truncated_content = content[:300] + "..." if len(content) > 300 else content

        formatted_chunk = f"{truncated_content}\n\nSource: {citation}"
        formatted_chunks.append(formatted_chunk)

    # Join all chunks with separator
    return "\n\n---\n\n".join(formatted_chunks)
