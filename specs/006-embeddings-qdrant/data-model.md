# Data Model: Embeddings Generation & Qdrant Vector Storage

**Feature**: 006-embeddings-qdrant
**Date**: 2025-12-19
**Status**: Complete

## Overview

This document defines the data entities and their relationships for the embeddings generation pipeline. The system processes markdown documents, chunks them, generates embeddings, and stores them in Qdrant with rich metadata.

---

## Entity Definitions

### 1. MarkdownDocument

Represents a single markdown file from the docs/ directory.

**Purpose**: Encapsulate file metadata and content for processing pipeline.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `file_path` | `Path` | Yes | Absolute path to markdown file | Must exist, end with `.md` |
| `relative_path` | `str` | Yes | Path relative to docs/ directory | For display/logging |
| `frontmatter` | `dict[str, Any]` | Yes | Parsed YAML frontmatter metadata | Empty dict if no frontmatter |
| `content` | `str` | Yes | Markdown body (excluding frontmatter) | Can be empty string |
| `word_count` | `int` | Yes | Word count of content | >= 0 |
| `token_count` | `int` | Yes | Token count (tiktoken cl100k_base) | >= 0 |
| `last_modified` | `datetime` | Yes | File modification timestamp | ISO 8601 format |
| `module` | `str | None` | Extracted module name (e.g., "module-1-ros2") | Parsed from path |
| `week` | `str | None` | Extracted week identifier (e.g., "week-3") | Parsed from path |

**Example**:
```python
MarkdownDocument(
    file_path=Path("/mnt/d/.../docs/module-1-ros2/week-3-lesson-1-ros2-architecture.md"),
    relative_path="module-1-ros2/week-3-lesson-1-ros2-architecture.md",
    frontmatter={
        "title": "ROS 2 Architecture",
        "sidebar_label": "Lesson 1: Architecture",
        "sidebar_position": 31,
        "description": "Understanding ROS 2 graph...",
        "tags": ["ros2", "week-3", "architecture"]
    },
    content="# ROS 2 Architecture\n\n**Estimated Time**: 40 minutes...",
    word_count=3245,
    token_count=4123,
    last_modified=datetime(2025, 12, 18, 16, 30, 0),
    module="module-1-ros2",
    week="week-3"
)
```

**Relationships**:
- 1 MarkdownDocument → Many ContentChunk (one-to-many)

---

### 2. ContentChunk

Represents a semantically coherent segment of markdown content.

**Purpose**: Split large documents into embedding-friendly chunks while preserving context.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `chunk_id` | `str` | Yes | MD5 hash of (file_path + chunk_index + content) | 32-char hex string |
| `content_text` | `str` | Yes | The actual chunk text | 1-2000 tokens |
| `token_count` | `int` | Yes | Exact token count via tiktoken | 500-1000 target, max 2000 |
| `chunk_index` | `int` | Yes | Zero-based index within parent document | >= 0 |
| `parent_document` | `MarkdownDocument` | Yes | Reference to source document | Not None |
| `start_char` | `int` | Yes | Character offset in original content | >= 0 |
| `end_char` | `int` | Yes | Character offset end in original content | > start_char |
| `has_code_block` | `bool` | Yes | Contains code fence (```) | Detected during chunking |
| `has_frontmatter` | `bool` | Yes | Is first chunk with frontmatter data | Only true for chunk_index=0 |

**Example**:
```python
ContentChunk(
    chunk_id="a3f5d8c9e1b4f6a2d7e9c4b8f1a3e5d7",
    content_text="## 1. Real-World Analogy: Restaurant Kitchen\n\nBefore diving...",
    token_count=856,
    chunk_index=0,
    parent_document=<MarkdownDocument object>,
    start_char=0,
    end_char=1247,
    has_code_block=False,
    has_frontmatter=True
)
```

**Chunk ID Generation**:
```python
import hashlib

def generate_chunk_id(file_path: str, chunk_index: int, content: str) -> str:
    """Generate deterministic ID for idempotent upserts."""
    hash_input = f"{file_path}:{chunk_index}:{content}"
    return hashlib.md5(hash_input.encode()).hexdigest()
```

**Relationships**:
- Many ContentChunk → 1 MarkdownDocument (many-to-one)
- 1 ContentChunk → 1 EmbeddingVector (one-to-one)

---

### 3. EmbeddingVector

Represents a Cohere embed-v4.0 generated vector embedding.

**Purpose**: Store vector representation for semantic search in Qdrant.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `vector_id` | `str` | Yes | Same as chunk_id (for Qdrant point ID) | 32-char hex string |
| `embedding` | `list[float]` | Yes | 1024-dimensional float vector | Length = 1024 |
| `model_version` | `str` | Yes | Cohere model identifier | "embed-english-v4.0" |
| `input_type` | `str` | Yes | Embedding input type | "search_document" |
| `chunk` | `ContentChunk` | Yes | Reference to source chunk | Not None |
| `metadata` | `dict[str, Any]` | Yes | Qdrant payload metadata | See MetadataPayload |
| `created_at` | `datetime` | Yes | Timestamp of generation | ISO 8601 UTC |

**Example**:
```python
EmbeddingVector(
    vector_id="a3f5d8c9e1b4f6a2d7e9c4b8f1a3e5d7",
    embedding=[0.0234, -0.1456, 0.0892, ...],  # 1024 floats
    model_version="embed-english-v4.0",
    input_type="search_document",
    chunk=<ContentChunk object>,
    metadata={
        "file_path": "module-1-ros2/week-3-lesson-1-ros2-architecture.md",
        "chunk_index": 0,
        "content": "## 1. Real-World Analogy...",
        "title": "ROS 2 Architecture",
        "tags": ["ros2", "week-3", "architecture"],
        "module": "module-1-ros2",
        "week": "week-3",
        "sidebar_position": 31,
        "has_code_block": False,
        "token_count": 856
    },
    created_at=datetime(2025, 12, 19, 10, 30, 0)
)
```

**Relationships**:
- 1 EmbeddingVector → 1 ContentChunk (one-to-one)
- 1 EmbeddingVector → 1 QdrantPoint (one-to-one)

---

### 4. QdrantPoint

Represents a point (vector + payload) in the Qdrant collection.

**Purpose**: Storage format for Qdrant vector database operations.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `id` | `str` | Yes | Point ID (same as vector_id/chunk_id) | Unique in collection |
| `vector` | `list[float]` | Yes | 1024-dimensional embedding | Length = 1024 |
| `payload` | `MetadataPayload` | Yes | Searchable metadata | See MetadataPayload schema |

**Example** (Qdrant Python Client format):
```python
from qdrant_client.models import PointStruct

point = PointStruct(
    id="a3f5d8c9e1b4f6a2d7e9c4b8f1a3e5d7",
    vector=[0.0234, -0.1456, 0.0892, ...],  # 1024 floats
    payload={
        "file_path": "module-1-ros2/week-3-lesson-1-ros2-architecture.md",
        "chunk_index": 0,
        "content": "## 1. Real-World Analogy: Restaurant Kitchen...",
        "title": "ROS 2 Architecture",
        "tags": ["ros2", "week-3", "architecture"],
        "module": "module-1-ros2",
        "week": "week-3",
        "sidebar_position": 31,
        "has_code_block": False,
        "token_count": 856
    }
)
```

**Relationships**:
- 1 QdrantPoint → 1 EmbeddingVector (one-to-one)

---

### 5. MetadataPayload

Defines the structured metadata stored in Qdrant payload for filtering and ranking.

**Purpose**: Enable metadata-based filtering in RAG queries (e.g., "search only ROS 2 module").

**Schema**:

| Field | Type | Required | Description | Indexed |
|-------|------|----------|-------------|---------|
| `file_path` | `str` | Yes | Relative path from docs/ | Yes |
| `chunk_index` | `int` | Yes | Chunk position in document | Yes |
| `content` | `str` | Yes | Full chunk text (for display) | No |
| `title` | `str` | No | Document title from frontmatter | Yes |
| `tags` | `list[str]` | No | Tags from frontmatter | Yes |
| `module` | `str` | No | Module identifier (e.g., "module-1-ros2") | Yes |
| `week` | `str` | No | Week identifier (e.g., "week-3") | Yes |
| `sidebar_position` | `int` | No | Docusaurus sidebar ordering | No |
| `has_code_block` | `bool` | Yes | Contains code examples | Yes |
| `token_count` | `int` | Yes | Chunk token count | No |

**Filtering Examples**:
```python
# Filter: Only ROS 2 module
client.search(
    collection_name="robotics_textbook_v1",
    query_vector=query_embedding,
    query_filter={"must": [{"key": "module", "match": {"value": "module-1-ros2"}}]},
    limit=5
)

# Filter: Chunks with code blocks in week 3
client.search(
    collection_name="robotics_textbook_v1",
    query_vector=query_embedding,
    query_filter={
        "must": [
            {"key": "has_code_block", "match": {"value": True}},
            {"key": "week", "match": {"value": "week-3"}}
        ]
    },
    limit=5
)
```

---

### 6. QdrantCollection

Represents the Qdrant vector database collection configuration.

**Purpose**: Define collection schema and settings for the vector database.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `collection_name` | `str` | Yes | Name of Qdrant collection | "robotics_textbook_v1" |
| `vector_size` | `int` | Yes | Embedding dimension | 1024 (Cohere embed-v4.0) |
| `distance_metric` | `Distance` | Yes | Similarity metric | COSINE |
| `total_vectors` | `int` | No | Current vector count | Retrieved via API |
| `indexed_fields` | `list[str]` | Yes | Payload fields with indexes | ["module", "week", "tags", "title"] |

**Collection Configuration** (Qdrant):
```python
from qdrant_client.models import Distance, VectorParams

client.recreate_collection(
    collection_name="robotics_textbook_v1",
    vectors_config=VectorParams(
        size=1024,
        distance=Distance.COSINE
    )
)

# Create payload indexes for filtering
client.create_payload_index(
    collection_name="robotics_textbook_v1",
    field_name="module",
    field_schema="keyword"
)
client.create_payload_index(
    collection_name="robotics_textbook_v1",
    field_name="tags",
    field_schema="keyword"
)
```

**Storage Estimate**:
- Vector size: 1024 × 4 bytes (float32) = 4 KB per vector
- Payload size: ~2 KB per vector (metadata)
- Total per vector: ~6 KB
- Expected vectors: ~172 (43 files × 4 chunks avg)
- **Total storage**: ~1 MB (well under 1GB free tier limit)

---

## Data Flow Diagram

```
┌─────────────────┐
│ Markdown Files  │
│   (docs/*.md)   │
└────────┬────────┘
         │
         ▼
┌─────────────────────────┐
│ 1. Parse Frontmatter    │
│    + Content Extraction │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│   MarkdownDocument      │
│ - file_path             │
│ - frontmatter           │
│ - content               │
│ - module, week          │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 2. Semantic Chunking    │
│    (RecursiveCharText   │
│     Splitter, tiktoken) │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│     ContentChunk[]      │
│ - chunk_id (MD5)        │
│ - content_text          │
│ - chunk_index           │
│ - token_count           │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 3. Batch Embedding      │
│    (Cohere embed-v4.0)  │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│   EmbeddingVector[]     │
│ - vector_id             │
│ - embedding[1024]       │
│ - metadata              │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 4. Upsert to Qdrant     │
│    (Idempotent)         │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│   Qdrant Collection     │
│ "robotics_textbook_v1"  │
│                         │
│  QdrantPoint[]          │
│  - id (chunk_id)        │
│  - vector[1024]         │
│  - payload (metadata)   │
└─────────────────────────┘
```

---

## State Transitions

### Processing Pipeline States

```
                           ┌─────────────┐
                           │  PENDING    │ Initial state
                           └──────┬──────┘
                                  │
                         discover_markdown_files()
                                  │
                                  ▼
                           ┌─────────────┐
                           │  DISCOVERED │ File found
                           └──────┬──────┘
                                  │
                            parse_markdown()
                                  │
                                  ▼
                           ┌─────────────┐
                           │   PARSED    │ Frontmatter + content extracted
                           └──────┬──────┘
                                  │
                            chunk_content()
                                  │
                                  ▼
                           ┌─────────────┐
                           │   CHUNKED   │ ContentChunk[] created
                           └──────┬──────┘
                                  │
                         generate_embeddings()
                                  │
                                  ▼
                           ┌─────────────┐
                           │  EMBEDDED   │ EmbeddingVector[] created
                           └──────┬──────┘
                                  │
                            store_in_qdrant()
                                  │
                                  ▼
                           ┌─────────────┐
                           │   STORED    │ Points in Qdrant
                           └─────────────┘

Error States:
- PARSE_FAILED: Malformed frontmatter → Log warning, skip file
- EMBED_FAILED: API error → Retry with backoff (max 3x)
- UPSERT_FAILED: Qdrant error → Retry, log error if persistent
```

---

## Validation Rules

### MarkdownDocument Validation

```python
def validate_markdown_document(doc: MarkdownDocument) -> bool:
    """Validate MarkdownDocument meets requirements."""
    assert doc.file_path.exists(), f"File not found: {doc.file_path}"
    assert doc.file_path.suffix == ".md", f"Not a markdown file: {doc.file_path}"
    assert doc.token_count >= 0, "Token count must be non-negative"
    assert isinstance(doc.frontmatter, dict), "Frontmatter must be dict"
    return True
```

### ContentChunk Validation

```python
def validate_content_chunk(chunk: ContentChunk) -> bool:
    """Validate ContentChunk meets size and format requirements."""
    assert 1 <= chunk.token_count <= 2000, f"Chunk size {chunk.token_count} outside 1-2000 range"
    assert chunk.chunk_index >= 0, "Chunk index must be non-negative"
    assert chunk.end_char > chunk.start_char, "Invalid character range"
    assert len(chunk.chunk_id) == 32, "Chunk ID must be 32-char MD5 hash"
    assert chunk.content_text.strip(), "Chunk cannot be empty/whitespace"
    return True
```

### EmbeddingVector Validation

```python
def validate_embedding_vector(vec: EmbeddingVector) -> bool:
    """Validate EmbeddingVector format and dimensions."""
    assert len(vec.embedding) == 1024, f"Expected 1024 dimensions, got {len(vec.embedding)}"
    assert all(isinstance(x, float) for x in vec.embedding), "Embedding must be list of floats"
    assert vec.model_version == "embed-english-v4.0", "Invalid model version"
    assert vec.vector_id == vec.chunk.chunk_id, "Vector ID must match chunk ID"
    return True
```

---

## Indexing Strategy

### Qdrant Payload Indexes

Create indexes on frequently filtered fields for query performance:

```python
# Module filtering (e.g., "only ROS 2 content")
client.create_payload_index(
    collection_name="robotics_textbook_v1",
    field_name="module",
    field_schema="keyword"
)

# Week filtering (e.g., "week 3 content only")
client.create_payload_index(
    collection_name="robotics_textbook_v1",
    field_name="week",
    field_schema="keyword"
)

# Tag filtering (e.g., "ros2" or "urdf" topics)
client.create_payload_index(
    collection_name="robotics_textbook_v1",
    field_name="tags",
    field_schema="keyword"
)

# Code block filtering (e.g., "show me examples")
client.create_payload_index(
    collection_name="robotics_textbook_v1",
    field_name="has_code_block",
    field_schema="bool"
)
```

---

## Type Definitions (Python)

```python
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

@dataclass
class MarkdownDocument:
    file_path: Path
    relative_path: str
    frontmatter: dict[str, Any]
    content: str
    word_count: int
    token_count: int
    last_modified: datetime
    module: Optional[str] = None
    week: Optional[str] = None

@dataclass
class ContentChunk:
    chunk_id: str
    content_text: str
    token_count: int
    chunk_index: int
    parent_document: MarkdownDocument
    start_char: int
    end_char: int
    has_code_block: bool
    has_frontmatter: bool

@dataclass
class EmbeddingVector:
    vector_id: str
    embedding: list[float]
    model_version: str
    input_type: str
    chunk: ContentChunk
    metadata: dict[str, Any]
    created_at: datetime

@dataclass
class QdrantCollection:
    collection_name: str
    vector_size: int
    distance_metric: str  # "COSINE"
    total_vectors: int
    indexed_fields: list[str]
```

---

## Summary

This data model provides:
1. **Clear entity boundaries** for modular development
2. **Idempotency** via content-based hashing (chunk_id = vector_id)
3. **Rich metadata** for filtering and ranking in RAG queries
4. **Validation rules** ensuring data quality
5. **Type safety** with Python dataclasses and type hints

All entities are designed to support the functional requirements (FR-001 through FR-013) defined in the feature specification.
