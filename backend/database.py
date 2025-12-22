"""
Database connection lifecycle management for RAG Chatbot Backend API.

Manages connection pools for:
- Qdrant Cloud (vector embeddings)
- Cohere (embeddings API client)
"""

from contextlib import asynccontextmanager
from fastapi import FastAPI
from qdrant_client import QdrantClient
import cohere
from typing import Optional
from config import settings # Added this import back as it was removed by mistake in previous steps.
qdrant_client: Optional[QdrantClient] = None
cohere_client: Optional[cohere.Client] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Manage application lifecycle: startup and shutdown.

    Startup:
    - Initialize Qdrant vector database client
    - Initialize Cohere embeddings client

    Shutdown:
    - Close all database connections gracefully
    """
    # ========== STARTUP ==========
    global qdrant_client, cohere_client

    print("= Initializing database connections...")



    # Qdrant Vector DB Client
    qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        timeout=settings.qdrant_timeout
    )
    print(" Qdrant client initialized")

    # Cohere Embeddings Client
    cohere_client = cohere.Client(
        api_key=settings.cohere_api_key
    )
    print(" Cohere client initialized")

    print(" All connections initialized successfully")

    yield  # Application runs here

    # ========== SHUTDOWN ==========
    print("= Shutting down gracefully...")



    # Close Qdrant Client
    if qdrant_client:
        qdrant_client.close()
        print(" Qdrant client closed")

    # Cohere HTTP client closes automatically
    print(" Cohere client closed")

    print(" All connections closed gracefully")








def get_qdrant_client() -> QdrantClient:
    """
    Get the Qdrant vector database client.

    Returns:
        QdrantClient instance

    Raises:
        RuntimeError: If client not initialized
    """
    if qdrant_client is None:
        raise RuntimeError("Qdrant client not initialized. Ensure lifespan context is running.")

    return qdrant_client


def get_cohere_client() -> cohere.Client:
    """
    Get the Cohere embeddings client.

    Returns:
        Cohere Client instance

    Raises:
        RuntimeError: If client not initialized
    """
    if cohere_client is None:
        raise RuntimeError("Cohere client not initialized. Ensure lifespan context is running.")

    return cohere_client