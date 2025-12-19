# Implementation Plan: Embeddings Generation & Qdrant Vector Storage

**Branch**: `006-embeddings-qdrant` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-embeddings-qdrant/spec.md`

## Summary

Build a Python CLI script that processes all markdown educational content from the `docs/` directory (43 files), chunks text into semantic segments (500-1000 tokens), generates 1024-dimensional embeddings using Cohere's embed-v4.0 model, and stores them in Qdrant Cloud vector database for semantic search. This enables the RAG (Retrieval-Augmented Generation) chatbot to answer student questions about course content with context-aware responses.

**Primary Requirement**: Enable semantic search over robotics textbook content for RAG chatbot integration.

**Technical Approach** (from research):
- LangChain RecursiveCharacterTextSplitter for semantic-aware chunking
- Cohere Python SDK v5+ with batch embedding API (96 docs/request)
- Qdrant Python client v1.7+ with idempotent upsert operations
- Content-based hashing (MD5) for vector IDs to ensure idempotency
- Exponential backoff retry logic for API resilience

---

## Technical Context

**Language/Version**: Python 3.10+ (for modern type hints, dataclasses, and async support)
**Primary Dependencies**:
  - `cohere>=5.0.0` (embed-v4.0 API)
  - `qdrant-client>=1.7.0` (vector database client)
  - `langchain>=0.1.0` (text splitters)
  - `tiktoken>=0.5.0` (token counting)
  - `python-frontmatter>=1.1.0` (YAML frontmatter parsing)
  - `tenacity>=8.2.0` (retry logic)
  - `tqdm>=4.66.0` (progress bars)

**Storage**:
  - Input: Markdown files in `docs/` directory (local filesystem)
  - Output: Qdrant Cloud (managed vector database, free tier: 1GB)
  - Logs: `embeddings_generation.log` (local file)

**Testing**:
  - Manual testing checklist (8 items) for v1
  - pytest framework ready for future unit/integration tests
  - Test query verification: "What is ROS 2?"

**Target Platform**:
  - Development: Ubuntu 22.04, macOS, Windows WSL2
  - Python 3.10+ on any OS
  - CLI script (not deployed service)

**Project Type**: Single Python script with optional helper modules

**Performance Goals**:
  - Process all 43 files in <10 minutes
  - Generate ~172 chunks total
  - Memory usage <2GB
  - Files processed: ≥5 files/minute average

**Constraints**:
  - Cohere free tier: 100 requests/min, 10,000 requests/month
  - Qdrant free tier: 1GB storage (~170,000 vectors max)
  - Batch processing required to stay within rate limits
  - Idempotent execution (re-running should not create duplicates)

**Scale/Scope**:
  - 43 markdown files
  - ~172 chunks (4 chunks per file average)
  - 1024-dimensional embeddings
  - ~1 MB total storage in Qdrant

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Educational Excellence ✅
- **Compliance**: Script enables semantic search for educational content, directly supporting RAG chatbot feature
- **Validation**: Embeddings will power context-aware answers to student questions about ROS 2, Gazebo, Isaac, and VLA topics

### Hands-On Learning ✅
- **Compliance**: Not directly a hands-on learning feature, but infrastructure supporting it
- **Validation**: Enables students to ask questions and get contextual answers from course materials

### Industry Alignment ✅
- **Compliance**: Uses production-level tools (Cohere embed-v4.0, Qdrant Cloud, LangChain)
- **Validation**: Same technologies used in industry RAG systems (e.g., LangChain is standard for LLM applications)

### Accessibility ✅
- **Compliance**: CLI script with clear documentation, runs on any Python 3.10+ environment
- **Validation**: Works on Ubuntu, macOS, Windows WSL2; free tier APIs (no paid requirements)

### Professional Quality ✅
- **Compliance**: Type hints, modular design, comprehensive error handling, detailed logging
- **Validation**: Code follows constitution's maintainability standards (NFR-009, NFR-010, NFR-011)

### Code Quality Standards ✅
- **Compliance**:
  - Type hints for all function signatures (NFR-009)
  - Functions <50 lines with single responsibility (NFR-010)
  - Inline comments explaining "why" not "what" (NFR-011)
- **Validation**: See data-model.md for type definitions; research.md for architectural patterns

### Security Standards ✅
- **Compliance**:
  - API keys in .env file, never hardcoded (NFR-006, NFR-008)
  - Environment variable validation on startup (NFR-007)
  - .env in .gitignore (already configured)
- **Validation**: See quickstart.md section 3 for security setup

### Performance Standards ✅
- **Compliance**:
  - <10 minute runtime (SC-001)
  - <2GB memory (NFR-002)
  - Batch API calls for efficiency (NFR-003)
- **Validation**: See research.md "Performance Estimates" section (estimated 10-15 seconds runtime)

**GATE STATUS**: ✅ **PASS** - All constitution principles satisfied

---

## Project Structure

### Documentation (this feature)

```text
specs/006-embeddings-qdrant/
├── spec.md              # Feature specification
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0 output (technology decisions)
├── data-model.md        # Phase 1 output (entity schemas)
├── quickstart.md        # Phase 1 output (setup guide)
└── tasks.md             # Phase 2 output (NOT created by /sp.plan, use /sp.tasks)
```

### Source Code (repository root)

```text
scripts/
├── generate_embeddings.py       # Main CLI script (400-500 lines)
├── requirements.txt             # Python dependencies (pinned versions)
├── .env.example                 # Environment variable template
├── README.md                    # Usage documentation
└── embeddings_utils/            # Optional helper modules (v2)
    ├── __init__.py
    ├── chunker.py               # chunk_content(), ChunkConfig
    ├── embedder.py              # CohereEmbedder class, batch logic
    ├── parser.py                # parse_markdown(), extract_metadata()
    └── vector_store.py          # QdrantStore class, upsert logic

docs/                            # Input: 43 markdown files
├── module-1-ros2/
│   ├── week-3-lesson-1-ros2-architecture.md
│   ├── week-3-lesson-2-nodes-packages.md
│   └── ...
├── module-2-gazebo-unity/
├── module-3-isaac/
└── module-4-vla/

logs/                            # Output: execution logs
└── embeddings_generation.log

.env                             # Environment variables (in .gitignore)
```

**Structure Decision**: Single project with optional modular helpers.

**Rationale**:
- v1: Single `generate_embeddings.py` script (~500 lines) is simplest for MVP
- v2: Split into `embeddings_utils/` modules if testing/reusability needed
- No backend/frontend split (this is infrastructure, not user-facing app)
- Logs directory created automatically on first run

---

## Complexity Tracking

No violations to justify. All complexity is minimal and necessary:

| Aspect | Justification |
|--------|---------------|
| Dependencies | All dependencies are industry-standard with no alternatives (Cohere for embeddings, Qdrant for vector DB, LangChain for chunking) |
| Single script vs modules | Single script chosen for simplicity (v1 MVP); modular design available for v2 if testing needed |
| Batch processing | Required to stay within Cohere API rate limits (100 req/min); batching 96 docs/request reduces from ~172 to ~2 API calls |
| Idempotency via hashing | Simplest approach for preventing duplicates; alternative (tracking processed files) adds state management complexity |

---

## Phase 0: Research & Technology Decisions

**Status**: ✅ Complete

**Output**: [research.md](./research.md)

### Key Decisions Made

1. **Chunking**: LangChain RecursiveCharacterTextSplitter + tiktoken
   - Rationale: Respects markdown structure (headers, code blocks, paragraphs)
   - Alternative rejected: Custom regex chunker (error-prone, reinventing wheel)

2. **Markdown Parsing**: python-frontmatter
   - Rationale: Lightweight, handles YAML edge cases, used in Docusaurus ecosystem
   - Alternative rejected: PyYAML alone (doesn't split frontmatter/content cleanly)

3. **Embedding API**: Cohere Python SDK v5+ with batch API
   - Rationale: Official SDK, batch support (96 docs/request), auto-retry, embed-v4.0 support
   - Alternative rejected: Direct HTTP requests (manual retry/batch logic)

4. **Vector Database**: Qdrant Python client v1.7+ with upsert
   - Rationale: Upsert provides natural idempotency, batch support (100 points/request)
   - Alternative rejected: Direct REST API (verbose, manual serialization)

5. **Error Handling**: tenacity library for declarative retries
   - Rationale: Exponential backoff with configurable retry logic
   - Alternative rejected: Manual retry loops (harder to test/maintain)

6. **Idempotency**: MD5 content hashing for vector IDs
   - Rationale: Same content → same hash → same ID → upsert overwrites → no duplicates
   - Alternative rejected: UUID (creates duplicates on re-run)

See [research.md](./research.md) for full details on all 8 technology decisions.

---

## Phase 1: Design & Data Model

**Status**: ✅ Complete

**Output**:
- [data-model.md](./data-model.md) - Entity schemas and relationships
- [quickstart.md](./quickstart.md) - Setup and usage guide
- contracts/ - N/A (CLI script, no REST/GraphQL API)

### Entity Summary

1. **MarkdownDocument**: Represents input .md file
   - Key fields: `file_path`, `frontmatter`, `content`, `module`, `week`
   - Relationship: 1 → Many ContentChunk

2. **ContentChunk**: Semantically coherent text segment
   - Key fields: `chunk_id` (MD5), `content_text`, `token_count`, `chunk_index`
   - Relationship: Many → 1 MarkdownDocument, 1 → 1 EmbeddingVector

3. **EmbeddingVector**: 1024-dim Cohere embedding
   - Key fields: `vector_id`, `embedding[1024]`, `metadata`, `model_version`
   - Relationship: 1 → 1 ContentChunk, 1 → 1 QdrantPoint

4. **QdrantPoint**: Vector database point
   - Key fields: `id`, `vector[1024]`, `payload` (MetadataPayload)
   - Relationship: 1 → 1 EmbeddingVector

5. **MetadataPayload**: Structured metadata for filtering
   - Key fields: `file_path`, `title`, `tags`, `module`, `week`, `has_code_block`
   - Indexed fields: `module`, `week`, `tags`, `title` (for fast filtering)

6. **QdrantCollection**: Collection configuration
   - Key fields: `collection_name`, `vector_size=1024`, `distance_metric=COSINE`

See [data-model.md](./data-model.md) for complete schemas, validation rules, and data flow diagram.

---

## Architecture

### High-Level Processing Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. STARTUP & VALIDATION                                         │
├─────────────────────────────────────────────────────────────────┤
│ • Load .env file (python-dotenv)                                │
│ • Validate COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY          │
│ • Initialize Cohere client (SDK v5+)                            │
│ • Initialize Qdrant client (v1.7+)                              │
│ • Create/verify collection: robotics_textbook_v1                │
│   - Vector size: 1024, Distance: COSINE                         │
│   - Payload indexes: module, week, tags, title                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. DOCUMENT DISCOVERY                                           │
├─────────────────────────────────────────────────────────────────┤
│ • Recursively scan docs/ for *.md files (Path.rglob)            │
│ • Filter out non-markdown files                                 │
│ • Sort by path (deterministic processing order)                 │
│ • Result: List[Path] (43 files)                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. PROCESSING LOOP (for each file with tqdm progress bar)      │
├─────────────────────────────────────────────────────────────────┤
│ • Parse frontmatter (python-frontmatter)                        │
│   - Extract: title, tags, description, sidebar_position        │
│   - Extract module/week from file path (regex)                  │
│                                                                  │
│ • Chunk content (LangChain RecursiveCharacterTextSplitter)      │
│   - Chunk size: 1000 tokens (configurable via CHUNK_SIZE)      │
│   - Chunk overlap: 200 tokens (configurable via CHUNK_OVERLAP) │
│   - Token counting: tiktoken cl100k_base                        │
│   - Semantic splits: \n\n → \n → space → char                  │
│   - Result: List[ContentChunk] (~4 per file)                    │
│                                                                  │
│ • Generate chunk IDs (MD5 hash)                                 │
│   - Input: f"{file_path}:{chunk_index}:{content}"              │
│   - Output: 32-char hex string (idempotent)                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. BATCH EMBEDDING GENERATION                                   │
├─────────────────────────────────────────────────────────────────┤
│ • Collect all chunks into batches (96 chunks/batch)             │
│ • Call Cohere embed API with retry logic (tenacity):            │
│   - Model: embed-english-v4.0                                   │
│   - Input type: search_document                                 │
│   - Retry: 3 attempts, exp backoff 1s → 2s → 4s → 8s          │
│   - Rate limit: 100 req/min (batch = ~2 total requests)        │
│ • Result: List[List[float]] (172 × 1024 embeddings)            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. QDRANT UPSERT (IDEMPOTENT)                                   │
├─────────────────────────────────────────────────────────────────┤
│ • Create PointStruct for each chunk:                            │
│   - id: chunk_id (MD5 hash)                                     │
│   - vector: embedding[1024]                                     │
│   - payload: {file_path, chunk_index, content, title, tags,    │
│               module, week, sidebar_position, has_code_block,   │
│               token_count}                                      │
│                                                                  │
│ • Batch upsert to Qdrant (100 points/batch)                     │
│   - Upsert overwrites by ID (idempotent)                        │
│   - Collection: robotics_textbook_v1                            │
│ • Result: 172 points stored in Qdrant                           │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ 6. VERIFICATION & LOGGING                                       │
├─────────────────────────────────────────────────────────────────┤
│ • Log summary statistics:                                       │
│   - Files processed: 43                                         │
│   - Chunks generated: 172                                       │
│   - Vectors stored: 172                                         │
│   - Total runtime: ~14 seconds                                  │
│                                                                  │
│ • Optional: Run test query (--test-query flag)                  │
│   - Embed query: "What is ROS 2?"                               │
│   - Search Qdrant (top 5 results)                               │
│   - Display: score, file, title, module, content preview        │
│                                                                  │
│ • Exit with success code (0)                                    │
└─────────────────────────────────────────────────────────────────┘
```

### Error Handling Strategy

```
┌─────────────────────────────────────────────────────────────────┐
│ ERROR TYPE           │ HANDLING STRATEGY                        │
├──────────────────────┼──────────────────────────────────────────┤
│ Missing env vars     │ Fail-fast: Validate at startup, exit 1   │
│ Invalid credentials  │ Fail-fast: Test API connection, exit 1   │
│ Malformed markdown   │ Log warning, skip file, continue         │
│ API rate limit       │ Retry with exp backoff (3x), then fail   │
│ Network error        │ Retry with exp backoff (3x), then fail   │
│ Qdrant upsert fail   │ Retry (3x), log error, continue          │
│ Out of memory        │ Log error, exit 1 (should not happen)    │
└─────────────────────────────────────────────────────────────────┘
```

### Idempotency Guarantee

```python
# Vector ID generation (deterministic)
def generate_chunk_id(file_path: str, chunk_index: int, content: str) -> str:
    hash_input = f"{file_path}:{chunk_index}:{content}"
    return hashlib.md5(hash_input.encode()).hexdigest()

# Property: Same input → Same ID
# Qdrant upsert: Same ID → Overwrites existing point
# Result: Re-running script = No duplicates
```

**Proof**:
1. File content unchanged → Same chunks → Same chunk_id
2. Qdrant `upsert(id=chunk_id)` → Replaces existing point with same ID
3. Collection point count remains constant on re-run

---

## Implementation Tasks (Phase 2)

**Note**: Detailed tasks will be generated by `/sp.tasks` command. High-level breakdown:

1. **Setup & Dependencies**
   - Create `scripts/requirements.txt` with pinned versions
   - Create `scripts/.env.example` template
   - Create `scripts/README.md` with setup instructions

2. **Core Functions**
   - `load_environment_variables()` - Validate env vars, fail-fast
   - `discover_markdown_files(docs_dir)` - Recursive .md scan
   - `parse_markdown(file_path)` - Frontmatter + content extraction
   - `extract_metadata(file_path, frontmatter)` - Module/week parsing
   - `chunk_content(content, chunk_size, overlap)` - LangChain splitter
   - `generate_chunk_id(file_path, index, content)` - MD5 hashing
   - `generate_embeddings(chunks)` - Cohere batch API with retry
   - `store_in_qdrant(vectors, metadata)` - Batch upsert
   - `test_search(query, top_k)` - Verification query

3. **Main Script Flow**
   - Argument parsing (--test-query, --incremental for v2)
   - Logging setup (console + file)
   - Progress bars (tqdm)
   - Error handling (try/except with logging)

4. **Testing**
   - Manual checklist (8 items from spec)
   - Test queries: "What is ROS 2?", "Isaac Sim setup", "URDF models"

5. **Documentation**
   - Update scripts/README.md with usage examples
   - Add troubleshooting section
   - Document environment variables

---

## Testing Strategy

### Manual Testing Checklist (v1)

| # | Test Case | Expected Result | Status |
|---|-----------|-----------------|--------|
| 1 | Run on fresh Qdrant collection | Collection created, 172 vectors stored | ⬜ |
| 2 | All 43 files processed | No parsing errors, all files logged | ⬜ |
| 3 | Search "What is ROS 2?" | Top 3 from module-1-ros2, score >0.75 | ⬜ |
| 4 | Search "Isaac Sim setup" | Results from module-3-isaac | ⬜ |
| 5 | Re-run script | Same 172 vectors, no duplicates | ⬜ |
| 6 | Invalid API key | Clear error: "Invalid COHERE_API_KEY" | ⬜ |
| 7 | Missing env var | Fail-fast: "Missing: QDRANT_URL" | ⬜ |
| 8 | Progress logging | File-by-file logs, tqdm bars | ⬜ |

### Integration Tests (Future v2)

```python
# pytest tests/test_embeddings.py

def test_chunking_preserves_code_blocks():
    content = "# Title\n\n```python\ndef foo():\n    pass\n```"
    chunks = chunk_content(content, chunk_size=100)
    assert any("```python" in c for c in chunks)

def test_idempotent_id_generation():
    id1 = generate_chunk_id("file.md", 0, "content")
    id2 = generate_chunk_id("file.md", 0, "content")
    assert id1 == id2

def test_metadata_extraction():
    frontmatter = {"title": "ROS 2", "tags": ["ros2", "week-3"]}
    file_path = "docs/module-1-ros2/week-3-lesson-1.md"
    meta = extract_metadata(file_path, frontmatter)
    assert meta["module"] == "module-1-ros2"
    assert meta["week"] == "week-3"
```

---

## Performance Estimates

### Expected Runtime Breakdown

| Phase | Operation | Time (est) | Bottleneck |
|-------|-----------|------------|------------|
| 1 | Startup validation | 1-2s | API connections |
| 2 | File discovery | <1s | Disk I/O |
| 3 | Parse + chunk 43 files | 4-5s | CPU (frontmatter, tiktoken) |
| 4 | Embed 172 chunks (2 batches) | 4-6s | Cohere API latency |
| 5 | Upsert 172 points (2 batches) | 2-3s | Qdrant API latency |
| 6 | Test query | 1-2s | Optional |
| **Total** | **End-to-end** | **~14s** | **API latency** |

### Resource Usage

- **Memory**: <50 MB peak (172 chunks × 6 KB avg)
- **Disk**: Minimal (logs only, ~1 MB)
- **Network**: ~1-2 MB (embeddings + metadata upload)
- **CPU**: Low (parsing, hashing, minimal computation)

### Scaling Projections

| Files | Chunks | Embed Batches | Upsert Batches | Est Runtime |
|-------|--------|---------------|----------------|-------------|
| 43 (current) | 172 | 2 | 2 | ~14s |
| 100 | 400 | 5 | 4 | ~30s |
| 500 | 2000 | 21 | 20 | ~2min |
| 1000 | 4000 | 42 | 40 | ~4min |

**Conclusion**: Scales linearly with file count. Bottleneck is API latency, not compute.

---

## Deployment & Operations

### Deployment

**Type**: Developer-run CLI script (not deployed service)

**Environment**:
- Local machine (developer workstation)
- CI/CD pipeline (optional, for automated re-indexing)

**Steps**:
1. Clone repository
2. Install Python 3.10+
3. Create virtual environment
4. Install dependencies: `pip install -r scripts/requirements.txt`
5. Configure `.env` with API keys
6. Run: `python scripts/generate_embeddings.py`

See [quickstart.md](./quickstart.md) for detailed setup guide.

### Monitoring

- **Logs**: `embeddings_generation.log` (INFO level by default)
- **Progress**: tqdm progress bars in terminal
- **Qdrant Dashboard**: https://cloud.qdrant.io/ (view collection stats)

### Maintenance

**Re-indexing** (when content updates):
```bash
# Option 1: Full re-run (idempotent)
python scripts/generate_embeddings.py

# Option 2: Incremental (v2 feature)
python scripts/generate_embeddings.py --incremental
```

**Cleanup** (delete collection):
```python
from qdrant_client import QdrantClient
client = QdrantClient(url=..., api_key=...)
client.delete_collection("robotics_textbook_v1")
```

---

## Risks & Mitigations

| Risk | Impact | Likelihood | Mitigation | Status |
|------|--------|------------|------------|--------|
| Cohere rate limits exceeded | High | Low | Batch API (96 docs/req), exp backoff, monitor usage | ✅ Addressed |
| Qdrant storage limit (1GB) | High | Very Low | 172 vectors = ~1 MB << 1GB, monitor collection size | ✅ Addressed |
| Poor embedding quality for technical terms | Medium | Low | Test with domain queries, validate scores >0.7 | ⬜ Test needed |
| Script fails mid-run | Medium | Low | Idempotent design (content-based IDs), safe re-run | ✅ Addressed |
| Manual re-indexing burden | Low | High | Document process, plan incremental updates for v2 | ⬜ v2 feature |
| API credentials leaked | High | Low | .env in .gitignore, validate at startup | ✅ Addressed |

---

## Success Criteria

### Measurable Outcomes (from spec)

- ✅ **SC-001**: All 43 files processed in <10 minutes → Estimated 14s (✅ under target)
- ⬜ **SC-002**: "What is ROS 2?" query returns relevant results (score >0.75) → Test after implementation
- ⬜ **SC-003**: Script runs end-to-end without manual intervention → Verify during testing
- ⬜ **SC-004**: Chunks are 500-1000 tokens (95th percentile) → Verify with logging
- ⬜ **SC-005**: Zero duplicate embeddings → Test with re-run
- ⬜ **SC-006**: API failures handled gracefully → Test with invalid credentials
- ⬜ **SC-007**: RAG chatbot >90% accuracy (20 test questions) → Test after integration
- ⬜ **SC-008**: Setup guide enables new developer in <15 minutes → Verify with quickstart.md

### Acceptance Criteria

**Minimum viable implementation**:
1. Script runs without errors on fresh environment
2. All 43 markdown files are processed
3. Embeddings stored in Qdrant with metadata
4. Test query returns relevant results
5. Re-running does not create duplicates
6. Clear error messages for missing credentials

---

## Future Enhancements (Out of Scope for v1)

1. **Incremental Updates** (v2)
   - Track file modification times
   - Only re-embed changed files
   - Delete vectors for removed files

2. **Multi-language Support** (v2)
   - Separate collections for Urdu translations
   - Cohere multilingual model

3. **Metadata Filtering** (v2)
   - Enable filtering by module, week, tags during RAG queries
   - Pre-filter before vector search for performance

4. **Parallel Processing** (v2)
   - Process files in parallel (multiprocessing)
   - Reduce runtime from ~14s to ~5s

5. **Automated Reindexing** (v2)
   - GitHub Actions trigger on content updates
   - Auto-deploy to Qdrant on merge to main

6. **Web Dashboard** (v2)
   - View embedding statistics
   - Search quality metrics
   - Collection analytics

---

## Dependencies on Other Features

- **003-book-content-structure**: Requires stable markdown structure in `docs/` directory with consistent frontmatter format
- **005-docusaurus-auth**: RAG chatbot will consume these embeddings for authenticated users (downstream dependency)

---

## References

- **Feature Spec**: [spec.md](./spec.md)
- **Research**: [research.md](./research.md)
- **Data Model**: [data-model.md](./data-model.md)
- **Quickstart**: [quickstart.md](./quickstart.md)

**External Documentation**:
- Cohere embed-v4.0: https://docs.cohere.com/docs/embed-v4
- Qdrant Python Client: https://qdrant.tech/documentation/frameworks/python/
- LangChain Text Splitters: https://python.langchain.com/docs/modules/data_connection/document_transformers/
- tiktoken: https://github.com/openai/tiktoken
- tenacity: https://tenacity.readthedocs.io/

---

## Next Steps

1. **Run `/sp.tasks`** to generate detailed implementation tasks
2. **Review tasks.md** with team for estimates and assignments
3. **Run `/sp.implement`** to execute tasks
4. **Manual testing** using checklist above
5. **Create ADR** if architectural decisions emerge during implementation

---

**Plan Status**: ✅ Complete - Ready for task generation (`/sp.tasks`)
**Branch**: 006-embeddings-qdrant
**Last Updated**: 2025-12-19
