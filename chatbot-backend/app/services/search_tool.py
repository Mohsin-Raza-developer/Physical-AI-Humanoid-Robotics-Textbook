"""
Knowledge base search tool using Qdrant and Cohere.

Provides semantic search over robotics textbook content.
"""
from agents import function_tool
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
import cohere
from app.config import settings
from app.utils.logger import get_logger
from app.utils.errors import ExternalServiceError
import asyncio

logger = get_logger(__name__)

# Initialize Cohere client for embeddings
cohere_client = cohere.Client(api_key=settings.COHERE_API_KEY)

# Initialize Qdrant client for vector search
qdrant_client = QdrantClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY
)

# Collection name (must exist in Qdrant)
COLLECTION_NAME = "robotics_textbook_v1"


@function_tool
def search_knowledge_base(query: str) -> str:
    """
    Search the robotics textbook knowledge base for relevant information.

    This tool performs semantic search over textbook content using embeddings.
    Use this when the user asks questions about robotics, ROS 2, Gazebo, or
    related technical topics covered in the course.

    Args:
        query: Search query describing what information to find

    Returns:
        str: Relevant textbook content matching the query
    """
    async def _async_search():
        try:
            logger.info("knowledge_search_started", query=query)

            # Generate embedding for query using Cohere
            embedding_response = cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",
                input_type="search_query"
            )
            query_embedding = embedding_response.embeddings[0]

            logger.info(
                "embedding_generated",
                embedding_dim=len(query_embedding)
            )

            # Search Qdrant for similar content
            search_results = qdrant_client.search(
                collection_name=COLLECTION_NAME,
                query_vector=query_embedding,
                limit=5,
                score_threshold=0.4
            )

            logger.info(
                "search_completed",
                results_count=len(search_results)
            )

            # Format results
            if not search_results:
                return "No relevant content found in the knowledge base for this query."

            formatted_results = []
            for idx, result in enumerate(search_results, 1):
                text = result.payload.get("text", "")
                source = result.payload.get("source", "Unknown")
                score = result.score

                formatted_results.append(
                    f"[Result {idx}] (Score: {score:.2f}, Source: {source})\n{text}"
                )

            return "\n\n".join(formatted_results)

        except Exception as e:
            logger.error(
                "search_error",
                error=str(e),
                error_type=type(e).__name__
            )
            # Return error message but don't raise to allow agent to continue
            return f"Error searching knowledge base: {str(e)}"

    # Run async function in sync context (required by function_tool)
    return asyncio.run(_async_search())


async def test_search_connection():
    """
    Test connection to Qdrant and verify collection exists.

    Returns:
        dict: Connection status and collection info

    Raises:
        ExternalServiceError: If connection fails
    """
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if COLLECTION_NAME not in collection_names:
            raise ExternalServiceError(
                "Qdrant",
                f"Collection '{COLLECTION_NAME}' not found. Available: {collection_names}"
            )

        # Get collection info
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)

        logger.info(
            "qdrant_connection_verified",
            collection=COLLECTION_NAME,
            vectors_count=collection_info.vectors_count
        )

        return {
            "status": "connected",
            "collection": COLLECTION_NAME,
            "vectors_count": collection_info.vectors_count
        }

    except Exception as e:
        logger.error("qdrant_connection_error", error=str(e))
        raise ExternalServiceError("Qdrant", str(e))
