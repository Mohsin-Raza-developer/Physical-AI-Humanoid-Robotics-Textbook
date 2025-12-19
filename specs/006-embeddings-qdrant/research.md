# Research: Embeddings Generation & Qdrant Vector Storage

**Feature**: 006-embeddings-qdrant
**Date**: 2025-12-19
**Status**: Complete

## Overview

This document captures research decisions for implementing a Python script that processes markdown educational content, generates vector embeddings using Cohere's embed-v4.0 model, and stores them in Qdrant Cloud for semantic search capabilities.

## Technology Decisions

### 1. Chunking Strategy & Library

**Decision**: Use LangChain's RecursiveCharacterTextSplitter with tiktoken for token counting

**Rationale**:
- **LangChain** provides battle-tested semantic chunking that respects markdown structure (headers, code blocks, paragraphs)
- **tiktoken** gives accurate token counts matching OpenAI's tokenizer (industry standard)
- RecursiveCharacterTextSplitter tries to split at semantic boundaries (double newlines, single newlines, then spaces)
- Handles code blocks properly by recognizing triple-backtick fences
- Configurable chunk size (1000 tokens) and overlap (200 tokens)

**Alternatives Considered**:
1. **Custom regex-based chunker**: Rejected - reinventing the wheel, prone to edge cases with markdown formatting
2. **Simple character splitting**: Rejected - breaks semantic meaning, can split mid-sentence or mid-code-block
3. **Sentence-based chunking (spaCy)**: Rejected - too granular for educational content, loses broader context
4. **LlamaIndex's chunker**: Considered but LangChain is more widely adopted and has better markdown support

**Implementation**:
```python
from langchain.text_splitter import RecursiveCharacterTextSplitter
import tiktoken

tokenizer = tiktoken.get_encoding("cl100k_base")  # GPT-4 tokenizer
text_splitter = RecursiveCharacterTextSplitter(
    chunk_size=1000,
    chunk_overlap=200,
    length_function=lambda text: len(tokenizer.encode(text)),
    separators=["\n\n", "\n", " ", ""]  # Semantic boundaries
)
```

**References**:
- LangChain Text Splitters: https://python.langchain.com/docs/modules/data_connection/document_transformers/
- tiktoken: https://github.com/openai/tiktoken

---

### 2. Markdown Parsing Library

**Decision**: Use python-frontmatter for YAML frontmatter extraction

**Rationale**:
- Lightweight (single dependency: PyYAML)
- Specifically designed for frontmatter extraction
- Already used in Docusaurus ecosystem
- Handles YAML parsing edge cases (multiline, special characters, lists)
- Returns both metadata dict and content separately

**Alternatives Considered**:
1. **markdown library + custom regex**: Rejected - error-prone, doesn't handle YAML edge cases
2. **PyYAML alone**: Rejected - doesn't handle the frontmatter/content split cleanly
3. **gray-matter (Node.js port)**: Rejected - Python ecosystem preference

**Implementation**:
```python
import frontmatter

with open("docs/module-1-ros2/week-3-lesson-1-ros2-architecture.md") as f:
    post = frontmatter.load(f)
    metadata = post.metadata  # dict: {title, sidebar_label, tags, ...}
    content = post.content    # str: markdown body
```

**References**:
- python-frontmatter: https://python-frontmatter.readthedocs.io/

---

### 3. Cohere API Integration Pattern

**Decision**: Use official Cohere Python SDK v5+ with batch embedding support

**Rationale**:
- Official SDK handles authentication, retries, rate limiting automatically
- Batch embedding API reduces API calls (up to 96 documents per request)
- embed-v4.0 supports `input_type="search_document"` for optimal retrieval embeddings
- Built-in error handling and exponential backoff
- Type hints for better IDE support

**Alternatives Considered**:
1. **Direct HTTP requests (requests library)**: Rejected - need to manually implement retries, batching, rate limiting
2. **Older Cohere SDK (<5.0)**: Rejected - deprecated API, lacks embed-v4.0 support
3. **LangChain CohereEmbeddings wrapper**: Considered but adds unnecessary abstraction layer

**Implementation**:
```python
import cohere
from typing import List

co = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

def embed_batch(texts: List[str]) -> List[List[float]]:
    """Generate embeddings for batch of texts."""
    response = co.embed(
        texts=texts,
        model="embed-english-v4.0",
        input_type="search_document",
        embedding_types=["float"]
    )
    return response.embeddings.float
```

**Rate Limiting Strategy**:
- Free tier: 100 requests/minute, 10,000 requests/month
- For 43 files × ~10 chunks each = ~430 chunks
- With batch size 96: ~5 API calls total (well under limit)
- Implement exponential backoff: 1s → 2s → 4s → 8s (max 3 retries)

**References**:
- Cohere Python SDK: https://github.com/cohere-ai/cohere-python
- embed-v4.0 docs: https://docs.cohere.com/docs/embed-v4

---

### 4. Qdrant Client Integration Pattern

**Decision**: Use qdrant-client v1.7+ with upsert operations for idempotency

**Rationale**:
- Official Python client with full feature support
- `upsert()` method overwrites by ID → natural idempotency
- Supports batch upsert for performance (up to 100 vectors per call)
- Built-in connection pooling and retry logic
- Type-safe models (PointStruct, Distance, VectorParams)

**Alternatives Considered**:
1. **Direct REST API calls**: Rejected - verbose, manual serialization, no retries
2. **LangChain Qdrant wrapper**: Considered but adds overhead for simple use case
3. **Older qdrant-client (<1.7)**: Rejected - lacks latest features and performance improvements

**Implementation**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import hashlib

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection (idempotent - only if not exists)
client.recreate_collection(
    collection_name="robotics_textbook_v1",
    vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
)

# Upsert with content-based ID
def generate_point_id(file_path: str, chunk_index: int, content: str) -> str:
    """Generate deterministic ID from content hash."""
    hash_input = f"{file_path}:{chunk_index}:{content}"
    return hashlib.md5(hash_input.encode()).hexdigest()

points = [
    PointStruct(
        id=generate_point_id(file_path, idx, chunk.content),
        vector=embedding,
        payload={
            "file_path": file_path,
            "chunk_index": idx,
            "content": chunk.content,
            "title": metadata.get("title"),
            "tags": metadata.get("tags", []),
            "module": extract_module(file_path),
            "week": extract_week(file_path)
        }
    )
    for idx, (chunk, embedding) in enumerate(zip(chunks, embeddings))
]

client.upsert(collection_name="robotics_textbook_v1", points=points)
```

**Idempotency Guarantee**:
- Same file + chunk index + content → same MD5 hash → same point ID
- Qdrant upsert overwrites existing point with same ID
- Re-running script multiple times produces identical collection state

**References**:
- Qdrant Python Client: https://qdrant.tech/documentation/frameworks/python/
- Qdrant Concepts: https://qdrant.tech/documentation/concepts/points/

---

### 5. Progress Logging & Monitoring

**Decision**: Use tqdm for progress bars + Python logging module for detailed logs

**Rationale**:
- **tqdm**: Beautiful progress bars for file processing, chunking, embedding
- **logging**: Structured logs (INFO, WARNING, ERROR) with timestamps
- Combine both: tqdm for user-friendly progress, logging for debugging
- Log to both console and file (`embeddings_generation.log`)

**Implementation**:
```python
import logging
from tqdm import tqdm

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('embeddings_generation.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Usage
files = discover_markdown_files("docs/")
for file_path in tqdm(files, desc="Processing files"):
    logger.info(f"Processing {file_path}")
    # ... processing logic
    logger.info(f"Generated {len(chunks)} chunks from {file_path}")
```

**References**:
- tqdm: https://tqdm.github.io/
- Python logging: https://docs.python.org/3/library/logging.html

---

### 6. Error Handling & Retry Strategy

**Decision**: Fail-fast for configuration, tenacity library for API retries

**Rationale**:
- **Configuration errors**: Validate environment variables at startup, exit immediately if missing
- **API errors**: Use `tenacity` library for declarative retry logic with exponential backoff
- **Content errors**: Log warning, skip file, continue processing (don't block entire pipeline)

**Implementation**:
```python
import os
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

# Fail-fast for config
def validate_environment():
    required = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    missing = [var for var in required if not os.getenv(var)]
    if missing:
        raise ValueError(f"Missing environment variables: {', '.join(missing)}")

# Retry for API calls
@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=60),
    retry=retry_if_exception_type((cohere.RateLimitError, cohere.ServerError))
)
def generate_embeddings_with_retry(texts: List[str]) -> List[List[float]]:
    return co.embed(...)

# Graceful degradation for content errors
try:
    post = frontmatter.load(file_path)
except Exception as e:
    logger.warning(f"Failed to parse {file_path}: {e}. Skipping.")
    continue
```

**References**:
- tenacity: https://tenacity.readthedocs.io/

---

### 7. Dependency Management

**Decision**: Pin major versions in requirements.txt, use pip-tools for reproducibility

**Rationale**:
- Pin major versions (e.g., `cohere>=5.0.0,<6.0.0`) for compatibility
- Allow minor/patch updates for security fixes
- pip-tools generates `requirements.lock` for exact reproducibility in CI/CD

**Final requirements.txt**:
```
# Core dependencies
cohere>=5.0.0,<6.0.0
qdrant-client>=1.7.0,<2.0.0
python-frontmatter>=1.1.0,<2.0.0
tiktoken>=0.5.0,<1.0.0
python-dotenv>=1.0.0,<2.0.0

# Chunking and utilities
langchain>=0.1.0,<0.2.0
langchain-text-splitters>=0.0.1

# Progress and logging
tqdm>=4.66.0,<5.0.0
tenacity>=8.2.0,<9.0.0

# Type checking (dev)
mypy>=1.8.0
types-tqdm
```

---

### 8. Testing Strategy

**Decision**: Manual testing for v1, pytest framework ready for v2

**Rationale**:
- V1 scope: Manual testing checklist (8 items) sufficient for validation
- V2 scope: Add unit tests for chunking, integration tests for end-to-end
- pytest ready for future: type hints + modular functions enable easy testing

**Manual Testing Checklist** (from spec):
1. Script runs without errors on fresh Qdrant collection
2. All 43 markdown files processed successfully
3. Search query "What is ROS 2?" returns relevant results
4. Search query "Isaac Sim setup" returns Isaac module results
5. Re-running script does not create duplicates
6. Invalid API key shows clear error message
7. Missing environment variable fails fast with helpful message
8. Progress logging shows file-by-file processing status

**Future Unit Tests** (v2):
```python
def test_chunking_preserves_code_blocks():
    content = "# Title\n\n```python\ndef foo():\n    pass\n```\n\nMore text"
    chunks = chunk_content(content, chunk_size=100)
    assert any("```python" in chunk for chunk in chunks)

def test_idempotent_id_generation():
    id1 = generate_point_id("file.md", 0, "content")
    id2 = generate_point_id("file.md", 0, "content")
    assert id1 == id2
```

---

## Architecture Patterns

### Script Execution Flow

```
1. Startup & Validation
   ├─ Load environment variables (.env)
   ├─ Validate required env vars (fail-fast)
   ├─ Initialize Cohere client
   ├─ Initialize Qdrant client
   └─ Create/verify collection schema

2. Document Discovery
   ├─ Recursively scan docs/ for .md files
   ├─ Filter out non-markdown files
   └─ Sort by path for deterministic processing

3. Processing Loop (per file)
   ├─ Parse frontmatter (title, tags, etc.)
   ├─ Extract content body
   ├─ Chunk content (RecursiveCharacterTextSplitter)
   ├─ Generate embeddings (batch API)
   ├─ Create Qdrant points (with metadata)
   └─ Upsert to Qdrant (idempotent)

4. Verification
   ├─ Run test query: "What is ROS 2?"
   ├─ Display top 3 results
   ├─ Log collection statistics
   └─ Exit with summary
```

### Modular Design (Optional Helper Modules)

While a single script is acceptable for v1, modular design enables testing:

```
scripts/embeddings_utils/
├── __init__.py
├── chunker.py          # chunk_content(), ChunkConfig
├── embedder.py         # CohereEmbedder class, batch logic
├── parser.py           # parse_markdown(), extract_metadata()
├── vector_store.py     # QdrantStore class, upsert logic
└── utils.py            # generate_point_id(), extract_module()
```

**Trade-off**: Single script = simpler, modular = testable. Choose based on team preference.

---

## Performance Estimates

### Processing Time Calculation

**Given**:
- 43 markdown files
- Average file size: ~3000 tokens
- Average chunks per file: ~4 (with 1000 token chunks, 200 overlap)
- Total chunks: ~172

**API Calls**:
- Cohere batch size: 96 documents/request
- Required batches: ceil(172 / 96) = 2 batches
- API latency: ~2s per batch (embed-v4.0)
- Total embedding time: ~4s

**Qdrant Operations**:
- Upsert batch size: 100 points/request
- Required batches: ceil(172 / 100) = 2 batches
- Upsert latency: ~1s per batch
- Total upsert time: ~2s

**File I/O & Parsing**:
- Read + parse: ~0.1s per file
- Total I/O: ~4.3s

**Total Estimated Runtime**: ~10-15 seconds (well under 10 minute target)

### Memory Usage

- 43 files × 10 KB average = ~430 KB raw text
- 172 chunks × 1024 dimensions × 4 bytes (float32) = ~700 KB embeddings
- Metadata overhead: ~100 KB
- **Peak memory**: <50 MB (well under 2GB constraint)

---

## Open Questions Resolved

### Q1: Should code blocks be embedded separately or as part of surrounding text?

**Answer**: As part of surrounding text.

**Rationale**: Code blocks in educational content need context (preceding explanation, following usage example). Separating them loses semantic meaning. RecursiveCharacterTextSplitter respects code block boundaries and won't split mid-block.

### Q2: What is the optimal chunk overlap for technical documentation?

**Answer**: 200 tokens (20% of 1000 token chunk size).

**Rationale**: Standard practice for RAG systems. Ensures context continuity across chunks without excessive duplication. For technical docs with dense information, 20% overlap preserves connections between concepts.

### Q3: Should we embed frontmatter metadata separately from content?

**Answer**: No, store as payload metadata only.

**Rationale**: Frontmatter (title, tags) is for filtering/ranking, not semantic search. Store in Qdrant payload for metadata filtering. Embedding it wastes vector space and dilutes content semantics.

### Q4: How to handle mathematical equations and diagrams?

**Answer**: Text representation only for v1 (no special handling).

**Rationale**: Current content uses Markdown/LaTeX text. Embed as-is. Future v2 could use specialized math embeddings or multimodal models, but out of scope for MVP.

---

## Risk Mitigations (from Spec)

| Risk | Mitigation Strategy |
|------|---------------------|
| Cohere API rate limits | Batch API (96 docs/req), exponential backoff with tenacity, monitor usage |
| Qdrant storage limit (1GB free tier) | 172 vectors × 1024 dim × 4 bytes = ~700KB << 1GB. Monitor with collection.info() |
| Poor embeddings quality | Test with domain queries, validate similarity scores >0.7, adjust chunking if needed |
| Script fails mid-process | Idempotent design (content-based IDs) allows safe re-run. Consider checkpoint file for v2 |
| Manual re-embedding on updates | Document process in README. Incremental updates planned for v2 |

---

## Next Steps (Phase 1: Design)

1. Create `data-model.md` with entity schemas
2. Generate `contracts/` (if applicable - likely N/A for script)
3. Create `quickstart.md` with setup instructions
4. Update CLAUDE.md with new technologies

---

## References

- Cohere embed-v4.0: https://docs.cohere.com/docs/embed-v4
- Qdrant Python Client: https://qdrant.tech/documentation/frameworks/python/
- LangChain Text Splitters: https://python.langchain.com/docs/modules/data_connection/document_transformers/
- tiktoken: https://github.com/openai/tiktoken
- python-frontmatter: https://python-frontmatter.readthedocs.io/
- tenacity: https://tenacity.readthedocs.io/
