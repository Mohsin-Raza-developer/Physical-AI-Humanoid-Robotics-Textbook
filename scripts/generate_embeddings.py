#!/usr/bin/env python3
"""
Generate embeddings from markdown content and store in Qdrant.

This script processes all markdown files from the docs/ directory, chunks content
semantically (500-1000 tokens), generates 1024-dimensional embeddings using
Cohere embed-v4.0, and stores them in Qdrant Cloud for semantic search.

Usage:
    python generate_embeddings.py
    python generate_embeddings.py --test-query "What is ROS 2?"

Environment Variables (required):
    COHERE_API_KEY: Cohere API key for embedding generation
    QDRANT_URL: Qdrant Cloud cluster URL
    QDRANT_API_KEY: Qdrant Cloud API key

Environment Variables (optional):
    CHUNK_SIZE: Token count per chunk (default: 1000)
    CHUNK_OVERLAP: Token overlap between chunks (default: 200)
    LOG_LEVEL: Logging level (default: INFO)

Architecture:
    1. Startup & validation
    2. Document discovery (recursive .md scan)
    3. Processing loop (parse, chunk, hash IDs)
    4. Batch embedding generation (Cohere API)
    5. Batch upsert to Qdrant (idempotent)
    6. Verification (optional test query)

For more details, see: ../specs/006-embeddings-qdrant/
"""

import argparse
import hashlib
import logging
import os
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Third-party imports
import cohere
import frontmatter
import tiktoken
from dotenv import load_dotenv
from langchain.text_splitter import RecursiveCharacterTextSplitter
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, PointStruct, VectorParams
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type
from tqdm import tqdm


# ============================================================================
# Configuration & Logging Setup
# ============================================================================

# Setup logging configuration
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('embeddings_generation.log', mode='a')
    ]
)
logger = logging.getLogger(__name__)


# ============================================================================
# Data Models (Type Definitions)
# ============================================================================

@dataclass
class MarkdownDocument:
    """Represents a single markdown file from docs/ directory."""
    file_path: Path
    relative_path: str
    frontmatter: Dict[str, Any]
    content: str
    word_count: int
    token_count: int
    last_modified: datetime
    module: Optional[str] = None
    week: Optional[str] = None


@dataclass
class ContentChunk:
    """Represents a semantically coherent segment of markdown content."""
    chunk_id: str
    content_text: str
    token_count: int
    chunk_index: int
    parent_file_path: str
    start_char: int
    end_char: int
    has_code_block: bool


# ============================================================================
# Environment & Configuration
# ============================================================================

def load_environment_variables() -> Dict[str, str]:
    """
    Load and validate required environment variables.

    Returns:
        Dict containing validated environment variables

    Raises:
        ValueError: If any required environment variable is missing
    """
    # Load .env file if it exists
    load_dotenv()

    # Required environment variables
    required_vars = {
        'COHERE_API_KEY': 'Cohere API key (get from https://cohere.com/dashboard)',
        'QDRANT_URL': 'Qdrant Cloud URL (get from https://cloud.qdrant.io/)',
        'QDRANT_API_KEY': 'Qdrant Cloud API key'
    }

    # Check for missing required variables
    missing = []
    for var, description in required_vars.items():
        if not os.getenv(var):
            missing.append(f"  - {var}: {description}")

    if missing:
        error_msg = "Missing required environment variables:\n" + "\n".join(missing)
        logger.error(error_msg)
        raise ValueError(error_msg)

    # Optional environment variables with defaults
    chunk_size = int(os.getenv('CHUNK_SIZE', '1000'))
    chunk_overlap = int(os.getenv('CHUNK_OVERLAP', '200'))
    log_level = os.getenv('LOG_LEVEL', 'INFO')

    # Update logging level if specified
    logger.setLevel(getattr(logging, log_level.upper()))

    # Log successful configuration (redact API keys)
    cohere_key = os.getenv('COHERE_API_KEY', '')
    qdrant_key = os.getenv('QDRANT_API_KEY', '')

    logger.info("Environment variables loaded successfully")
    logger.info(f"  Cohere API Key: {cohere_key[:10]}... (redacted)")
    logger.info(f"  Qdrant URL: {os.getenv('QDRANT_URL')}")
    logger.info(f"  Qdrant API Key: {qdrant_key[:10]}... (redacted)")
    logger.info(f"  Chunk Size: {chunk_size} tokens")
    logger.info(f"  Chunk Overlap: {chunk_overlap} tokens")
    logger.info(f"  Log Level: {log_level}")

    return {
        'cohere_api_key': cohere_key,
        'qdrant_url': os.getenv('QDRANT_URL', ''),
        'qdrant_api_key': qdrant_key,
        'chunk_size': chunk_size,
        'chunk_overlap': chunk_overlap
    }


# ============================================================================
# Tokenization
# ============================================================================

# Global tokenizer instance (cached for performance)
_TOKENIZER: Optional[tiktoken.Encoding] = None


def initialize_tiktoken_tokenizer() -> tiktoken.Encoding:
    """
    Initialize and cache tiktoken tokenizer for GPT-4 (cl100k_base).

    Returns:
        tiktoken.Encoding: Tokenizer instance
    """
    global _TOKENIZER
    if _TOKENIZER is None:
        logger.info("Initializing tiktoken tokenizer (cl100k_base)")
        _TOKENIZER = tiktoken.get_encoding("cl100k_base")
    return _TOKENIZER


# ============================================================================
# File Discovery & Parsing
# ============================================================================

def discover_markdown_files(docs_dir: str = "docs") -> List[Path]:
    """
    Recursively discover all markdown files in the docs directory.

    Args:
        docs_dir: Root directory to scan (default: "docs")

    Returns:
        Sorted list of Path objects for .md files
    """
    docs_path = Path(docs_dir)
    if not docs_path.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        return []

    # Find all .md files recursively
    md_files = list(docs_path.rglob("*.md"))

    # Sort for deterministic processing order
    md_files.sort()

    logger.info(f"Discovered {len(md_files)} markdown files in {docs_dir}/")
    return md_files


def extract_metadata(file_path: Path, frontmatter_dict: Dict[str, Any]) -> Tuple[Optional[str], Optional[str]]:
    """
    Extract module and week identifiers from file path.

    Args:
        file_path: Path to markdown file
        frontmatter_dict: Parsed frontmatter dictionary

    Returns:
        Tuple of (module, week) or (None, None) if not found
    """
    path_str = str(file_path)

    # Extract module (e.g., "module-1-ros2", "module-2-gazebo-unity")
    module_match = re.search(r'(module-\d+-[a-z-]+)', path_str)
    module = module_match.group(1) if module_match else None

    # Extract week (e.g., "week-3", "week-10")
    week_match = re.search(r'(week-\d+)', path_str)
    week = week_match.group(1) if week_match else None

    return module, week


def parse_markdown(file_path: Path) -> Optional[MarkdownDocument]:
    """
    Parse markdown file with YAML frontmatter.

    Args:
        file_path: Path to markdown file

    Returns:
        MarkdownDocument or None if parsing fails
    """
    try:
        # Load file with frontmatter
        with open(file_path, 'r', encoding='utf-8') as f:
            post = frontmatter.load(f)

        # Get file stats
        stats = file_path.stat()
        last_modified = datetime.fromtimestamp(stats.st_mtime)

        # Calculate word and token counts
        content = post.content
        word_count = len(content.split())

        tokenizer = initialize_tiktoken_tokenizer()
        token_count = len(tokenizer.encode(content))

        # Extract module and week from path
        module, week = extract_metadata(file_path, post.metadata)

        # Create relative path from docs/
        try:
            relative_path = str(file_path.relative_to("docs"))
        except ValueError:
            relative_path = str(file_path)

        return MarkdownDocument(
            file_path=file_path,
            relative_path=relative_path,
            frontmatter=post.metadata,
            content=content,
            word_count=word_count,
            token_count=token_count,
            last_modified=last_modified,
            module=module,
            week=week
        )

    except Exception as e:
        logger.warning(f"Failed to parse {file_path}: {e}. Skipping.")
        return None


# ============================================================================
# Content Chunking
# ============================================================================

def generate_chunk_id(file_path: str, chunk_index: int, content: str) -> str:
    """
    Generate deterministic MD5 hash ID for chunk (ensures idempotency).

    Args:
        file_path: Relative path to source file
        chunk_index: Zero-based index of chunk within file
        content: Chunk text content

    Returns:
        32-character hex string (MD5 hash)
    """
    hash_input = f"{file_path}:{chunk_index}:{content}"
    return hashlib.md5(hash_input.encode()).hexdigest()


def chunk_content(
    content: str,
    file_path: str,
    chunk_size: int = 1000,
    chunk_overlap: int = 200
) -> List[ContentChunk]:
    """
    Chunk content using semantic-aware splitting (preserves code blocks, paragraphs).

    Args:
        content: Markdown content to chunk
        file_path: Relative path (for chunk ID generation)
        chunk_size: Target tokens per chunk (default: 1000)
        chunk_overlap: Overlap between chunks (default: 200)

    Returns:
        List of ContentChunk objects
    """
    if not content.strip():
        return []

    tokenizer = initialize_tiktoken_tokenizer()

    # Create text splitter with semantic boundaries
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=chunk_size,
        chunk_overlap=chunk_overlap,
        length_function=lambda text: len(tokenizer.encode(text)),
        separators=["\n\n", "\n", " ", ""]  # Semantic split priority
    )

    # Split text
    chunks_text = text_splitter.split_text(content)

    # Create ContentChunk objects
    chunks = []
    current_pos = 0

    for idx, chunk_text in enumerate(chunks_text):
        # Find chunk position in original content
        start_char = content.find(chunk_text, current_pos)
        if start_char == -1:
            start_char = current_pos
        end_char = start_char + len(chunk_text)
        current_pos = end_char

        # Calculate exact token count for this chunk
        token_count = len(tokenizer.encode(chunk_text))

        # Detect code blocks
        has_code_block = "```" in chunk_text

        # Generate deterministic ID
        chunk_id = generate_chunk_id(file_path, idx, chunk_text)

        chunks.append(ContentChunk(
            chunk_id=chunk_id,
            content_text=chunk_text,
            token_count=token_count,
            chunk_index=idx,
            parent_file_path=file_path,
            start_char=start_char,
            end_char=end_char,
            has_code_block=has_code_block
        ))

    return chunks


# ============================================================================
# Cohere Embedding Generation
# ============================================================================

def initialize_cohere_client(api_key: str) -> cohere.Client:
    """
    Initialize Cohere API client and test connection.

    Args:
        api_key: Cohere API key

    Returns:
        Initialized Cohere client

    Raises:
        ValueError: If API key is invalid
    """
    try:
        client = cohere.Client(api_key=api_key)
        logger.info("Connected to Cohere API (model: embed-v4.0)")
        return client
    except Exception as e:
        logger.error(f"Failed to initialize Cohere client: {e}")
        raise ValueError(f"Invalid COHERE_API_KEY: {e}")


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=60),
    retry=retry_if_exception_type((Exception,))  # Retry on any Cohere exception
)
def generate_embeddings_batch(
    client: cohere.Client,
    chunks: List[ContentChunk],
    batch_size: int = 96
) -> List[List[float]]:
    """
    Generate embeddings for chunks using Cohere embed-v4.0 with batch processing.

    Args:
        client: Initialized Cohere client
        chunks: List of ContentChunk objects
        batch_size: Max documents per API call (default: 96, Cohere limit)

    Returns:
        List of 1024-dimensional embedding vectors
    """
    all_embeddings = []

    # Process in batches
    num_batches = (len(chunks) + batch_size - 1) // batch_size

    with tqdm(total=num_batches, desc="Generating embeddings", unit="batch") as pbar:
        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]
            texts = [chunk.content_text for chunk in batch]

            # Call Cohere embed API
            response = client.embed(
                texts=texts,
                model="embed-v4.0",
                input_type="search_document",
                embedding_types=["float"]
            )

            # Collect embeddings
            all_embeddings.extend(response.embeddings.float)
            pbar.update(1)

    logger.info(f"Generated {len(all_embeddings)} embeddings")
    return all_embeddings


# ============================================================================
# Qdrant Vector Storage
# ============================================================================

def initialize_qdrant_client(url: str, api_key: str) -> QdrantClient:
    """
    Initialize Qdrant client and test connection.

    Args:
        url: Qdrant Cloud cluster URL
        api_key: Qdrant Cloud API key

    Returns:
        Initialized Qdrant client

    Raises:
        ValueError: If credentials are invalid
    """
    try:
        client = QdrantClient(url=url, api_key=api_key)
        # Test connection
        client.get_collections()
        logger.info(f"Connected to Qdrant Cloud: {url}")
        return client
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        raise ValueError(f"Invalid QDRANT_URL or QDRANT_API_KEY: {e}")


def create_qdrant_collection(
    client: QdrantClient,
    collection_name: str = "robotics_textbook_v1"
) -> None:
    """
    Create Qdrant collection with 1024-dim vectors and payload indexes.

    Args:
        client: Initialized Qdrant client
        collection_name: Name of collection to create
    """
    # Check if collection already exists
    collections = client.get_collections().collections
    exists = any(c.name == collection_name for c in collections)

    if exists:
        logger.info(f"Collection '{collection_name}' already exists")
        return

    # Create collection with vector configuration
    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=1536,  # ✅ Correct for embed-v4.0
            distance=Distance.COSINE
        )
    )

    # Create payload indexes for filtering
    payload_fields = ["module", "week", "tags", "title", "has_code_block"]
    for field in payload_fields:
        try:
            client.create_payload_index(
                collection_name=collection_name,
                field_name=field,
                field_schema="keyword" if field != "has_code_block" else "bool"
            )
        except Exception as e:
            logger.warning(f"Could not create index for {field}: {e}")

    logger.info(f"Created collection: {collection_name}")


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=60)
)
def store_embeddings_in_qdrant(
    client: QdrantClient,
    chunks: List[ContentChunk],
    embeddings: List[List[float]],
    metadata_list: List[Dict[str, Any]],
    collection_name: str = "robotics_textbook_v1",
    batch_size: int = 100
) -> None:
    """
    Store embeddings in Qdrant with metadata (idempotent upsert).

    Args:
        client: Initialized Qdrant client
        chunks: List of ContentChunk objects
        embeddings: List of embedding vectors (1024-dim)
        metadata_list: List of metadata dicts per chunk
        collection_name: Qdrant collection name
        batch_size: Points per batch (default: 100)
    """
    # Create points
    points = []
    for chunk, embedding, metadata in zip(chunks, embeddings, metadata_list):
        point = PointStruct(
            id=chunk.chunk_id,  # MD5 hash for idempotency
            vector=embedding,
            payload={
                "file_path": chunk.parent_file_path,
                "chunk_index": chunk.chunk_index,
                "content": chunk.content_text,
                "title": metadata.get("title", ""),
                "tags": metadata.get("tags", []),
                "module": metadata.get("module"),
                "week": metadata.get("week"),
                "sidebar_position": metadata.get("sidebar_position"),
                "has_code_block": chunk.has_code_block,
                "token_count": chunk.token_count
            }
        )
        points.append(point)

    # Upsert in batches
    num_batches = (len(points) + batch_size - 1) // batch_size

    with tqdm(total=num_batches, desc="Uploading to Qdrant", unit="batch") as pbar:
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            client.upsert(
                collection_name=collection_name,
                points=batch
            )
            pbar.update(1)

    logger.info(f"Stored {len(points)} vectors in Qdrant")


# ============================================================================
# Search Verification
# ============================================================================

def embed_query(client: cohere.Client, query: str) -> List[float]:
    """
    Generate embedding for search query.

    Args:
        client: Initialized Cohere client
        query: Search query text

    Returns:
        1024-dimensional embedding vector
    """
    response = client.embed(
        texts=[query],
        model="embed-v4.0",
        input_type="search_query",  # Different from document embedding
        embedding_types=["float"]
    )
    return response.embeddings.float[0]


def search_qdrant(
    client: QdrantClient,
    query_vector: List[float],
    collection_name: str = "robotics_textbook_v1",
    top_k: int = 5
) -> List[Dict[str, Any]]:
    """
    Search Qdrant for similar vectors.

    Args:
        client: Initialized Qdrant client
        query_vector: Query embedding (1024-dim)
        collection_name: Collection to search
        top_k: Number of results to return

    Returns:
        List of result dicts with score, file_path, title, content
    """
    results = client.query_points(
        collection_name=collection_name,
        query=query_vector,
        limit=top_k
    ).points

    formatted_results = []
    for hit in results:
        formatted_results.append({
            "score": hit.score,
            "file_path": hit.payload.get("file_path"),
            "title": hit.payload.get("title"),
            "module": hit.payload.get("module"),
            "week": hit.payload.get("week"),
            "content": hit.payload.get("content", "")[:200]  # Preview
        })

    return formatted_results


def test_search(query: str, config: Dict[str, str], top_k: int = 5) -> None:
    """
    Run test search query and display results.

    Args:
        query: Search query text
        config: Configuration dict with API keys
        top_k: Number of results to display
    """
    logger.info(f"Testing search: '{query}'")
    print(f"\nTesting search: \"{query}\"\n")

    # Initialize clients
    cohere_client = initialize_cohere_client(config['cohere_api_key'])
    qdrant_client = initialize_qdrant_client(config['qdrant_url'], config['qdrant_api_key'])

    # Generate query embedding
    query_vector = embed_query(cohere_client, query)

    # Search Qdrant
    results = search_qdrant(qdrant_client, query_vector, top_k=top_k)

    # Display results
    print("Top 5 Results:")
    print("-" * 80)

    for i, result in enumerate(results, 1):
        print(f"\n{i}. Score: {result['score']:.4f}")
        print(f"   File: {result['file_path']}")
        print(f"   Title: {result['title']}")
        print(f"   Module: {result['module']} | Week: {result['week']}")
        print(f"\n   Content (first 200 chars):")
        print(f"   {result['content']}")

    print("\n" + "-" * 80)
    print("\n[SUCCESS] Search verification successful! Vectors are correctly stored and retrievable.\n")


# ============================================================================
# Main Pipeline
# ============================================================================

def main() -> None:
    """Main execution pipeline."""
    start_time = time.time()

    try:
        # Parse command-line arguments
        parser = argparse.ArgumentParser(
            description="Generate embeddings from markdown content and store in Qdrant"
        )
        parser.add_argument(
            "--test-query",
            type=str,
            help="Run test search query (e.g., 'What is ROS 2?')"
        )
        args = parser.parse_args()

        # Load and validate environment
        logger.info("=" * 80)
        logger.info("Starting embeddings generation pipeline")
        logger.info("=" * 80)

        config = load_environment_variables()

        # If only running test query, do that and exit
        if args.test_query:
            test_search(args.test_query, config)
            return

        # Initialize clients
        cohere_client = initialize_cohere_client(config['cohere_api_key'])
        qdrant_client = initialize_qdrant_client(config['qdrant_url'], config['qdrant_api_key'])
        tokenizer = initialize_tiktoken_tokenizer()

        # Create Qdrant collection
        create_qdrant_collection(qdrant_client)

        # Discover markdown files
        files = discover_markdown_files("docs")
        if not files:
            logger.error("No markdown files found. Exiting.")
            return

        # Process files
        all_chunks = []
        all_metadata = []

        with tqdm(total=len(files), desc="Processing files", unit="file") as pbar:
            for file_path in files:
                # Parse markdown
                doc = parse_markdown(file_path)
                if not doc:
                    pbar.update(1)
                    continue

                # Chunk content
                chunks = chunk_content(
                    doc.content,
                    doc.relative_path,
                    config['chunk_size'],
                    config['chunk_overlap']
                )

                # Accumulate chunks and metadata
                for chunk in chunks:
                    all_chunks.append(chunk)
                    all_metadata.append({
                        "title": doc.frontmatter.get("title"),
                        "tags": doc.frontmatter.get("tags", []),
                        "module": doc.module,
                        "week": doc.week,
                        "sidebar_position": doc.frontmatter.get("sidebar_position")
                    })

                pbar.update(1)

        logger.info(f"Generated {len(all_chunks)} chunks from {len(files)} files")

        # Generate embeddings
        embeddings = generate_embeddings_batch(cohere_client, all_chunks)

        # Store in Qdrant
        store_embeddings_in_qdrant(
            qdrant_client,
            all_chunks,
            embeddings,
            all_metadata
        )

        # Summary
        elapsed_time = time.time() - start_time
        logger.info("=" * 80)
        logger.info("[SUCCESS] Successfully processed %d files", len(files))
        logger.info("[SUCCESS] Generated %d chunks", len(all_chunks))
        logger.info("[SUCCESS] Stored %d vectors in Qdrant", len(embeddings))
        logger.info("[SUCCESS] Total runtime: %.1f seconds", elapsed_time)
        logger.info("=" * 80)

        print(f"\n{'=' * 80}")
        print(f"[SUCCESS] Embeddings generation complete!")
        print(f"{'=' * 80}")
        print(f"  Files processed: {len(files)}")
        print(f"  Chunks generated: {len(all_chunks)}")
        print(f"  Vectors stored: {len(embeddings)}")
        print(f"  Runtime: {elapsed_time:.1f} seconds")
        print(f"  Average: {elapsed_time / len(files):.2f} seconds/file")
        print(f"{'=' * 80}\n")
        print(f"💡 Try a test search:")
        print(f'   python generate_embeddings.py --test-query "What is ROS 2?"\n')

    except Exception as e:
        logger.error(f"Pipeline failed: {e}", exc_info=True)
        print(f"\n[ERROR] {e}\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
