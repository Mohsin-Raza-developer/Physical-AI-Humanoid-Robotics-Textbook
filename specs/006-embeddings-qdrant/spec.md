# Feature Specification: Embeddings Generation & Qdrant Vector Storage

**Feature Branch**: `006-embeddings-qdrant`
**Created**: 2025-12-19
**Status**: Draft
**Input**: User description: "Create a Python script to generate embeddings from book content and store in Qdrant vector database."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Content Ingestion & Embedding Generation (Priority: P1)

A developer needs to process all Markdown content from the docs/ directory, generate vector embeddings, and store them in Qdrant Cloud to enable semantic search functionality for the RAG chatbot.

**Why this priority**: This is the foundational capability required for the RAG chatbot feature. Without embeddings stored in Qdrant, semantic search and question-answering functionality cannot work.

**Independent Test**: Can be fully tested by running the script with sample markdown files and verifying embeddings are successfully stored in Qdrant. Delivers immediate value by enabling vector search on course content.

**Acceptance Scenarios**:

1. **Given** docs/ directory contains 43 Markdown files, **When** developer runs `python scripts/generate_embeddings.py`, **Then** all files are processed, chunked, and embeddings are generated using Cohere embed-v4.0
2. **Given** embeddings are generated, **When** script uploads to Qdrant, **Then** all chunks are stored in `robotics_textbook_v1` collection with metadata (file_path, chunk_index, content)
3. **Given** environment variables are configured, **When** script authenticates with Cohere and Qdrant, **Then** API connections succeed without errors
4. **Given** a markdown file with 5000 tokens, **When** chunking algorithm runs, **Then** content is split into 5-10 chunks of 500-1000 tokens each, preserving semantic boundaries

---

### User Story 2 - Semantic Search Verification (Priority: P1)

A developer needs to verify that the embeddings enable accurate semantic search by querying the vector database with course-related questions.

**Why this priority**: Verification ensures the embeddings are correctly stored and retrievable, which is critical for RAG chatbot functionality. This validates the entire pipeline end-to-end.

**Independent Test**: Can be tested by running a search query script that retrieves top-k similar chunks for "What is ROS 2?" and verifying relevant content is returned.

**Acceptance Scenarios**:

1. **Given** embeddings are stored in Qdrant, **When** developer queries "What is ROS 2?", **Then** top 5 results include chunks from week-3-lesson-1-ros2-architecture.md with relevance scores > 0.7
2. **Given** a technical query about "URDF humanoid models", **When** semantic search is performed, **Then** relevant chunks from week-5-lesson-2-urdf-humanoids.md are returned
3. **Given** a query about "Isaac Sim setup", **When** search runs, **Then** chunks from module-3-isaac/week-8-lesson-1-isaac-sim-setup.md appear in top 3 results

---

### User Story 3 - Incremental Updates & Re-indexing (Priority: P2)

A content author updates or adds new Markdown files and needs to update the vector database without re-processing all existing content.

**Why this priority**: While important for production efficiency, this is not critical for MVP. Initial implementation can re-process all files, with incremental updates added in later iterations.

**Independent Test**: Can be tested by modifying a single file, running the script with an `--incremental` flag, and verifying only changed files are re-embedded.

**Acceptance Scenarios**:

1. **Given** embeddings exist for all files, **When** one file is modified, **Then** script detects change via timestamp/hash and only re-processes that file
2. **Given** a new lesson file is added to docs/, **When** script runs incrementally, **Then** only the new file is processed and added to existing collection
3. **Given** a file is deleted from docs/, **When** script runs with cleanup flag, **Then** corresponding chunks are removed from Qdrant

---

### User Story 4 - Error Handling & Resilience (Priority: P2)

A developer runs the script and encounters API rate limits, network failures, or malformed content, and needs clear error messages with recovery options.

**Why this priority**: Essential for production reliability but not blocking for initial development. Can be implemented after core functionality works.

**Independent Test**: Can be tested by simulating API failures (invalid keys, rate limits) and verifying graceful error handling with informative messages.

**Acceptance Scenarios**:

1. **Given** Cohere API rate limit is reached, **When** embedding generation fails, **Then** script implements exponential backoff and retries up to 3 times
2. **Given** Qdrant connection fails mid-upload, **When** network error occurs, **Then** script saves progress checkpoint and allows resume from last successful chunk
3. **Given** a markdown file has malformed frontmatter, **When** parsing fails, **Then** script logs error, skips file, and continues processing remaining files
4. **Given** invalid API credentials, **When** script initializes, **Then** clear error message indicates which credential is missing/invalid

---

### Edge Cases

- What happens when a markdown file contains only frontmatter with no content?
- How does the system handle markdown files larger than 20,000 tokens (e.g., capstone project specs)?
- What if Qdrant collection already exists with same name but different schema?
- How does chunking handle code blocks that span multiple chunks?
- What happens if two chunks have identical embeddings (duplicated content)?
- How does the system handle non-English text (e.g., Urdu translations in future)?
- What if a file is empty or contains only whitespace?
- How are special characters and LaTeX equations handled during chunking?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST recursively read all `.md` files from `docs/` directory and subdirectories
- **FR-002**: System MUST parse markdown frontmatter to extract metadata (title, tags, description, sidebar_position)
- **FR-003**: System MUST chunk content into segments of 500-1000 tokens using semantic-aware splitting (preserve code blocks, paragraphs, sections)
- **FR-004**: System MUST generate embeddings using Cohere embed-v4.0 model with dimension 1024
- **FR-005**: System MUST store embeddings in Qdrant Cloud collection named `robotics_textbook_v1`
- **FR-006**: Each vector entry MUST include metadata: `file_path`, `chunk_index`, `content_text`, `title`, `tags`, `module`, `week`
- **FR-007**: System MUST authenticate with Cohere using `COHERE_API_KEY` environment variable
- **FR-008**: System MUST authenticate with Qdrant using `QDRANT_URL` and `QDRANT_API_KEY` environment variables
- **FR-009**: System MUST log progress (files processed, chunks generated, embeddings created) to console and log file
- **FR-010**: System MUST provide test query functionality to verify search works with example: "What is ROS 2?"
- **FR-011**: System MUST handle API rate limits with exponential backoff (initial delay 1s, max delay 60s, max retries 3)
- **FR-012**: Script MUST be idempotent - running multiple times should not create duplicate embeddings
- **FR-013**: System MUST validate environment variables on startup and fail fast with clear error messages if missing

### Key Entities

- **MarkdownDocument**: Represents a single .md file with attributes: `file_path`, `frontmatter`, `content`, `word_count`, `last_modified`
- **ContentChunk**: Represents a segment of content with attributes: `chunk_id`, `content_text`, `token_count`, `chunk_index`, `parent_document`, `start_char`, `end_char`
- **EmbeddingVector**: Represents a generated embedding with attributes: `vector_id`, `embedding` (1024-dim float array), `chunk`, `metadata`, `created_at`
- **QdrantCollection**: Represents the vector database collection with attributes: `collection_name`, `vector_size`, `distance_metric`, `total_vectors`

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 43 markdown files from docs/ directory are successfully processed and stored in Qdrant within 10 minutes runtime
- **SC-002**: Search query "What is ROS 2?" returns top 3 results from ROS 2 module with similarity scores > 0.75
- **SC-003**: Script completes end-to-end execution without manual intervention (fully automated pipeline)
- **SC-004**: Each chunk contains between 500-1000 tokens (95th percentile within range)
- **SC-005**: Zero duplicate embeddings in Qdrant collection (verified by content hash)
- **SC-006**: Script handles API failures gracefully with automatic retry and clear error logging
- **SC-007**: Generated embeddings enable RAG chatbot to answer course-specific questions with >90% accuracy (tested on 20 sample questions)
- **SC-008**: Documentation (README.md) allows new developer to set up and run script in under 15 minutes

## Technical Constraints

### Technology Stack

- **Python Version**: 3.10+ (for modern type hints and async support)
- **Embedding Model**: Cohere embed-v4.0 (1024 dimensions)
  - API: https://api.cohere.ai/v1/embed
  - Docs: https://docs.cohere.com/docs/embed-v4
  - Input type: `search_document` for document embeddings
- **Vector Database**: Qdrant Cloud Free Tier
  - Collection: `robotics_textbook_v1`
  - Distance metric: Cosine similarity
  - Vector size: 1024
  - Docs: https://qdrant.tech/documentation/quick-start/
- **Chunking**: tiktoken library for token counting (OpenAI tokenizer)
- **Markdown Parsing**: python-frontmatter + markdown libraries

### Dependencies (requirements.txt)

```
cohere>=5.0.0
qdrant-client>=1.7.0
python-frontmatter>=1.1.0
tiktoken>=0.5.0
python-dotenv>=1.0.0
tqdm>=4.66.0
```

### Environment Variables

```bash
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here

# Optional Configuration
CHUNK_SIZE=1000  # Default: 1000 tokens
CHUNK_OVERLAP=200  # Default: 200 tokens
LOG_LEVEL=INFO  # Default: INFO
```

### Directory Structure

```
scripts/
├── generate_embeddings.py       # Main script
├── requirements.txt             # Python dependencies
├── README.md                    # Setup and usage documentation
├── .env.example                 # Example environment variables
└── embeddings_utils/            # Helper modules (optional)
    ├── __init__.py
    ├── chunker.py               # Chunking logic
    ├── embedder.py              # Cohere integration
    └── vector_store.py          # Qdrant integration
```

## Design Decisions

### Chunking Strategy

**Decision**: Use semantic-aware chunking with 1000 token target and 200 token overlap.

**Rationale**:
- Overlap ensures context continuity across chunk boundaries
- 1000 tokens balances between context richness and embedding model limits
- Semantic splitting (at paragraph/section boundaries) preserves meaning

**Alternatives Considered**:
- Fixed character count: Rejected due to loss of semantic context
- Sentence-based chunking: Too granular, loses broader context
- No overlap: Rejected due to potential loss of context at boundaries

### Idempotency Approach

**Decision**: Use content-based hashing (MD5 of file content + metadata) as vector ID.

**Rationale**:
- Same content always generates same hash → prevents duplicates
- Qdrant upsert operation overwrites existing vectors with same ID
- Enables incremental updates without manual deduplication

**Alternatives Considered**:
- UUID generation: Rejected, would create duplicates on re-runs
- Manual deduplication: Rejected, adds complexity and performance overhead

### Error Handling Strategy

**Decision**: Fail-fast for configuration errors, graceful degradation for API errors.

**Rationale**:
- Invalid credentials should stop execution immediately (fail-fast)
- Transient API errors should retry with backoff (resilience)
- Malformed content should log and skip (don't block entire pipeline)

## Non-Functional Requirements

### Performance

- **NFR-001**: Script MUST process at least 5 files per minute (average)
- **NFR-002**: Memory usage MUST stay below 2GB for entire document corpus
- **NFR-003**: API calls MUST implement connection pooling and batch processing where possible

### Reliability

- **NFR-004**: Script MUST handle network interruptions gracefully with checkpoint/resume capability
- **NFR-005**: Logging MUST capture all errors with stack traces for debugging

### Security

- **NFR-006**: API keys MUST never be hardcoded or committed to git
- **NFR-007**: Environment variables MUST be validated before processing sensitive data
- **NFR-008**: API keys MUST be loaded from .env file or environment (never from code)

### Maintainability

- **NFR-009**: Code MUST include type hints for all function signatures
- **NFR-010**: Functions MUST be modular with single responsibility (max 50 lines per function)
- **NFR-011**: Critical logic MUST include inline comments explaining "why" not "what"

## Deliverables

### 1. scripts/generate_embeddings.py

Main Python script with the following structure:

```python
#!/usr/bin/env python3
"""
Generate embeddings from markdown content and store in Qdrant.

Usage:
    python generate_embeddings.py [--incremental] [--test-query "query"]
"""

# Functions:
# - load_environment_variables()
# - discover_markdown_files(docs_dir: str) -> list[Path]
# - parse_markdown(file_path: Path) -> MarkdownDocument
# - chunk_content(content: str, chunk_size: int) -> list[ContentChunk]
# - generate_embeddings(chunks: list[ContentChunk]) -> list[EmbeddingVector]
# - store_in_qdrant(vectors: list[EmbeddingVector])
# - test_search(query: str, top_k: int = 5)
# - main()
```

### 2. scripts/requirements.txt

Python dependencies with pinned versions for reproducibility.

### 3. scripts/README.md

Setup and usage documentation including:
- Prerequisites (Python 3.10+, API accounts)
- Installation steps
- Environment variable configuration
- Usage examples
- Troubleshooting guide
- Expected output examples

### 4. scripts/.env.example

Example environment variables file for easy setup.

## Testing Strategy

### Unit Tests (Optional for v1)

- Test chunking algorithm with various input sizes
- Test markdown parsing with different frontmatter formats
- Test error handling for API failures

### Integration Tests

1. **End-to-End Test**: Process sample documents and verify Qdrant storage
2. **Search Quality Test**: Run 10 test queries and validate result relevance
3. **Idempotency Test**: Run script twice and verify no duplicate vectors

### Manual Testing Checklist

- [ ] Script runs without errors on fresh Qdrant collection
- [ ] All 43 markdown files are processed successfully
- [ ] Search query "What is ROS 2?" returns relevant results from ROS 2 module
- [ ] Search query "Isaac Sim setup" returns results from Isaac module
- [ ] Re-running script does not create duplicates
- [ ] Invalid API key shows clear error message
- [ ] Missing environment variable fails fast with helpful message
- [ ] Progress logging shows file-by-file processing status

## Future Enhancements (Out of Scope for v1)

- **Incremental updates**: Track file modification times, only re-embed changed files
- **Multi-language support**: Generate separate embeddings for Urdu translations
- **Metadata filtering**: Enable filtering by module, week, or tags during search
- **Embedding model versioning**: Support switching between Cohere models
- **Batch processing**: Process files in parallel for faster execution
- **Web dashboard**: View embedding statistics and search quality metrics
- **Automatic reindexing**: Trigger re-embedding via GitHub Actions on content updates

## Dependencies on Other Features

- **003-book-content-structure**: Requires stable markdown structure in docs/ directory
- **005-docusaurus-auth**: RAG chatbot will consume these embeddings for authenticated users

## Risks & Mitigations

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Cohere API rate limits hit during bulk processing | High | Medium | Implement exponential backoff, batch requests, consider paid tier |
| Qdrant free tier storage limit (1GB) exceeded | High | Low | Monitor collection size, implement chunking optimization |
| Embeddings quality poor for technical content | High | Low | Test with domain-specific queries, fine-tune chunking strategy |
| Script fails mid-process, needs full re-run | Medium | Medium | Implement checkpoint/resume with progress tracking |
| Content updates require manual re-embedding | Low | High | Document process, plan incremental updates for v2 |

## Open Questions

- Should code blocks be embedded separately or as part of surrounding text?
- What is the optimal chunk overlap for technical documentation?
- Should we embed frontmatter metadata separately from content?
- How to handle mathematical equations and diagrams (currently text-only)?

## References

- Cohere Embed API: https://docs.cohere.com/reference/embed
- Cohere RAG Guide: https://docs.cohere.com/docs/retrieval-augmented-generation-rag
- Qdrant Quickstart: https://qdrant.tech/documentation/quick-start/
- Qdrant Python Client: https://qdrant.tech/documentation/frameworks/python/  or https://github.com/qdrant/qdrant-client  or for installation command pip install qdrant-client
- tiktoken (OpenAI tokenizer): https://github.com/openai/tiktoken
