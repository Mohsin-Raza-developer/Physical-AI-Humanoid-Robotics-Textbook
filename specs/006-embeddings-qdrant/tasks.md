# Tasks: Embeddings Generation & Qdrant Vector Storage

**Input**: Design documents from `/specs/006-embeddings-qdrant/`
**Prerequisites**: plan.md (✅), spec.md (✅), research.md (✅), data-model.md (✅), quickstart.md (✅)

**Tests**: Manual testing checklist included (no automated tests for MVP/v1)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

---

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions
- Tests are manual (8-item checklist) for v1; pytest scaffolding ready for v2

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency management

- [X] T001 Create `scripts/requirements.txt` with pinned dependency versions (cohere>=5.0.0, qdrant-client>=1.7.0, langchain>=0.1.0, langchain-text-splitters>=0.0.1, tiktoken>=0.5.0, python-frontmatter>=1.1.0, python-dotenv>=1.0.0, tqdm>=4.66.0, tenacity>=8.2.0)
- [X] T002 [P] Create `scripts/.env.example` template with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, CHUNK_SIZE, CHUNK_OVERLAP, LOG_LEVEL placeholders
- [X] T003 [P] Create `scripts/README.md` with setup instructions, prerequisites (Python 3.10+, API accounts), installation steps, usage examples, and troubleshooting guide per quickstart.md

**Checkpoint**: ✅ Dependencies documented, environment template ready

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before user story implementation

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Initialize `scripts/generate_embeddings.py` with shebang (#!/usr/bin/env python3), module docstring, and main() entry point with `if __name__ == "__main__"` guard
- [X] T005 [P] Setup logging configuration in `scripts/generate_embeddings.py`: create logger with console handler (StreamHandler) and file handler (FileHandler to `embeddings_generation.log`), format with timestamp + level + message, default INFO level (configurable via LOG_LEVEL env var)
- [X] T006 [P] Implement `load_environment_variables()` function in `scripts/generate_embeddings.py`: use python-dotenv to load .env file, validate required vars (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY), fail-fast with ValueError if missing, log loaded config (redact keys)
- [X] T007 Add type definitions in `scripts/generate_embeddings.py`: import dataclass from dataclasses, import typing annotations (List, Dict, Optional, Any, Tuple), import Path from pathlib, import datetime
- [X] T008 [P] Create MarkdownDocument dataclass in `scripts/generate_embeddings.py` with fields: file_path (Path), relative_path (str), frontmatter (Dict[str, Any]), content (str), word_count (int), token_count (int), last_modified (datetime), module (Optional[str]), week (Optional[str])
- [X] T009 [P] Create ContentChunk dataclass in `scripts/generate_embeddings.py` with fields: chunk_id (str), content_text (str), token_count (int), chunk_index (int), parent_file_path (str), start_char (int), end_char (int), has_code_block (bool)

**Checkpoint**: ✅ Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Content Ingestion & Embedding Generation (Priority: P1) 🎯 MVP

**Goal**: Process all 43 markdown files from docs/ directory, chunk content semantically (500-1000 tokens), generate Cohere embed-v4.0 embeddings (1024-dim), and store in Qdrant Cloud with metadata

**Independent Test**: Run `python scripts/generate_embeddings.py` on fresh Qdrant collection, verify all 43 files processed, ~172 vectors stored in `robotics_textbook_v1` collection with complete metadata

**Acceptance Criteria**:
- ✅ All 43 .md files recursively discovered from docs/
- ✅ Frontmatter parsed correctly (title, tags, sidebar_position)
- ✅ Content chunked into 500-1000 tokens per chunk (95th percentile)
- ✅ Cohere API called successfully with batch processing
- ✅ Qdrant collection created with 1024-dim COSINE vectors
- ✅ All vectors stored with metadata (file_path, chunk_index, content, title, tags, module, week)

### Implementation for User Story 1

#### File Discovery & Parsing (Parallel - Different Concerns)

- [X] T010 [P] [US1] Implement `discover_markdown_files(docs_dir: str) -> List[Path]` in `scripts/generate_embeddings.py`: use Path(docs_dir).rglob("*.md") to recursively find .md files, filter out non-markdown, sort by path for deterministic order, log count found, return List[Path]
- [X] T011 [P] [US1] Implement `parse_markdown(file_path: Path) -> MarkdownDocument` in `scripts/generate_embeddings.py`: use frontmatter.load() to parse YAML frontmatter + content, calculate word_count (len(content.split())), get last_modified from file stats, create MarkdownDocument dataclass, handle exceptions (log warning + skip if malformed)
- [X] T012 [P] [US1] Implement `extract_metadata(file_path: Path, frontmatter_dict: Dict) -> Tuple[Optional[str], Optional[str]]` in `scripts/generate_embeddings.py`: use regex to extract module (e.g., "module-1-ros2") and week (e.g., "week-3") from file path, return (module, week) tuple

#### Chunking Logic (Depends on Discovery)

- [X] T013 [US1] Implement `initialize_tiktoken_tokenizer() -> tiktoken.Encoding` in `scripts/generate_embeddings.py`: call tiktoken.get_encoding("cl100k_base") for GPT-4 tokenizer, cache globally, return encoding object
- [X] T014 [US1] Implement `chunk_content(content: str, chunk_size: int, chunk_overlap: int) -> List[ContentChunk]` in `scripts/generate_embeddings.py`: create RecursiveCharacterTextSplitter from langchain with chunk_size from env (default 1000), chunk_overlap from env (default 200), length_function using tiktoken tokenizer, separators=["\n\n", "\n", " ", ""], call split_text(content), create ContentChunk for each chunk with chunk_index, calculate token_count, detect has_code_block (contains "```"), set start_char/end_char, return List[ContentChunk]
- [X] T015 [US1] Implement `generate_chunk_id(file_path: str, chunk_index: int, content: str) -> str` in `scripts/generate_embeddings.py`: create hash_input = f"{file_path}:{chunk_index}:{content}", use hashlib.md5(hash_input.encode()).hexdigest() to generate 32-char ID for idempotency, return str

#### Embedding Generation (Depends on Chunking)

- [X] T016 [US1] Implement `initialize_cohere_client(api_key: str) -> cohere.Client` in `scripts/generate_embeddings.py`: import cohere, create client = cohere.Client(api_key=api_key), test connection (try/except), return client
- [X] T017 [US1] Implement `generate_embeddings_batch(client: cohere.Client, chunks: List[ContentChunk], batch_size: int = 96) -> List[List[float]]` in `scripts/generate_embeddings.py`: split chunks into batches of batch_size (96 max per Cohere API), for each batch extract texts list, call client.embed(texts=texts, model="embed-english-v4.0", input_type="search_document", embedding_types=["float"]), collect response.embeddings.float, flatten into single list, use tqdm progress bar for batches, wrap with tenacity @retry decorator (stop_after_attempt(3), wait_exponential(multiplier=1, min=1, max=60), retry on RateLimitError/ServerError), return List[List[float]]

#### Qdrant Storage (Depends on Embeddings)

- [X] T018 [US1] Implement `initialize_qdrant_client(url: str, api_key: str) -> QdrantClient` in `scripts/generate_embeddings.py`: import qdrant_client, create client = QdrantClient(url=url, api_key=api_key), test connection (try/except), return client
- [X] T019 [US1] Implement `create_qdrant_collection(client: QdrantClient, collection_name: str = "robotics_textbook_v1") -> None` in `scripts/generate_embeddings.py`: import VectorParams, Distance from qdrant_client.models, check if collection exists (client.collection_exists()), if exists: log "Collection already exists", else: call client.create_collection(collection_name, vectors_config=VectorParams(size=1024, distance=Distance.COSINE)), create payload indexes for module, week, tags, title fields (client.create_payload_index()), log collection created
- [X] T020 [US1] Implement `store_embeddings_in_qdrant(client: QdrantClient, chunks: List[ContentChunk], embeddings: List[List[float]], metadata_list: List[Dict], collection_name: str = "robotics_textbook_v1", batch_size: int = 100) -> None` in `scripts/generate_embeddings.py`: import PointStruct from qdrant_client.models, for each chunk/embedding/metadata triple create PointStruct(id=chunk.chunk_id, vector=embedding, payload={file_path, chunk_index, content, title, tags, module, week, sidebar_position, has_code_block, token_count}), batch into groups of batch_size, call client.upsert(collection_name, points=batch) for each batch (idempotent upsert), use tqdm progress bar, wrap with tenacity retry, log vectors stored

#### Main Pipeline Orchestration (Depends on All Above)

- [X] T021 [US1] Implement main processing pipeline in `main()` function in `scripts/generate_embeddings.py`: call load_environment_variables(), initialize clients (Cohere, Qdrant, tiktoken), create Qdrant collection, discover files = discover_markdown_files("docs"), initialize empty lists (all_chunks, all_embeddings, all_metadata), loop through files with tqdm: parse_markdown(), extract_metadata(), chunk_content(), assign chunk_ids, accumulate chunks/metadata, call generate_embeddings_batch() on all chunks, call store_embeddings_in_qdrant(), log summary stats (files processed, chunks generated, vectors stored, runtime), handle exceptions with logging
- [X] T022 [US1] Add argument parsing in `main()` function in `scripts/generate_embeddings.py`: import argparse, create parser with --test-query optional argument (str, help="Run test search query"), parse args, if args.test_query provided: call test_search(args.test_query) after embedding generation

**Checkpoint**: ✅ User Story 1 complete - All 43 files processed and embeddings stored in Qdrant

---

## Phase 4: User Story 2 - Semantic Search Verification (Priority: P1) 🎯 MVP

**Goal**: Verify embeddings enable accurate semantic search by querying Qdrant with course-related questions and validating result relevance

**Independent Test**: Run `python scripts/generate_embeddings.py --test-query "What is ROS 2?"`, verify top 3 results from module-1-ros2 with scores >0.75

**Acceptance Criteria**:
- ✅ Test query embeds correctly using Cohere
- ✅ Qdrant search returns top-k results with scores
- ✅ Results include full metadata (file, title, module, content preview)
- ✅ Query "What is ROS 2?" returns ROS 2 module chunks
- ✅ Query "Isaac Sim setup" returns Isaac module chunks
- ✅ Similarity scores >0.7 for relevant matches

### Implementation for User Story 2

- [X] T023 [P] [US2] Implement `embed_query(client: cohere.Client, query: str) -> List[float]` in `scripts/generate_embeddings.py`: call client.embed(texts=[query], model="embed-english-v4.0", input_type="search_query", embedding_types=["float"]), extract response.embeddings.float[0], return 1024-dim vector
- [X] T024 [US2] Implement `search_qdrant(client: QdrantClient, query_vector: List[float], collection_name: str = "robotics_textbook_v1", top_k: int = 5) -> List[Dict]` in `scripts/generate_embeddings.py`: call client.search(collection_name, query_vector=query_vector, limit=top_k), extract results with score + payload, return List[Dict] with keys (score, file_path, title, module, week, content_preview)
- [X] T025 [US2] Implement `test_search(query: str, top_k: int = 5)` function in `scripts/generate_embeddings.py`: load env vars, initialize Cohere + Qdrant clients, call embed_query(query), call search_qdrant(query_vector, top_k), format results with rich table/text (score, file, title, module, content preview first 200 chars), print to console with separator lines, validate scores >0.7 for top 3, log test result (PASS/FAIL)
- [X] T026 [US2] Add test queries to README.md in scripts/README.md: document example usage `python generate_embeddings.py --test-query "What is ROS 2?"`, include expected output sample, list suggested test queries ("URDF models", "Isaac Sim setup", "RealSense camera"), explain similarity score interpretation

**Checkpoint**: ✅ User Story 2 complete - Semantic search verified with test queries

---

## Phase 5: User Story 3 - Incremental Updates & Re-indexing (Priority: P2)

**Goal**: Support incremental updates by detecting changed files (timestamp/hash) and only re-embedding modified content

**Independent Test**: Modify one file, run with `--incremental` flag, verify only changed file re-processed

**Acceptance Criteria**:
- ✅ File modification detection via timestamp or content hash
- ✅ Only changed files re-chunked and re-embedded
- ✅ Deleted files removed from Qdrant
- ✅ New files added to collection
- ✅ Idempotency maintained (no duplicates)

### Implementation for User Story 3 (v2 - Out of Scope for MVP)

**Note**: User Story 3 is P2 priority - deferred to v2. For MVP/v1, full re-processing is acceptable due to fast runtime (~14s for 43 files).

- [ ] T027 [US3] Create `scripts/.processed_files.json` tracking file for storing file hashes and timestamps
- [ ] T028 [US3] Implement `compute_file_hash(file_path: Path) -> str` in `scripts/generate_embeddings.py`: read file content, compute SHA256 hash, return hex digest
- [ ] T029 [US3] Implement `load_processed_files() -> Dict[str, Dict]` in `scripts/generate_embeddings.py`: load .processed_files.json, return dict mapping file_path → {hash, timestamp, last_processed}
- [ ] T030 [US3] Implement `detect_changes(current_files: List[Path], processed_files: Dict) -> Tuple[List[Path], List[str], List[Path]]` in `scripts/generate_embeddings.py`: compare current files vs processed, identify (added_files, deleted_files, modified_files) by hash comparison, return tuple
- [ ] T031 [US3] Implement `delete_vectors_for_file(client: QdrantClient, file_path: str, collection_name: str)` in `scripts/generate_embeddings.py`: query Qdrant for all points with payload.file_path == file_path, delete by IDs, log deletion count
- [ ] T032 [US3] Add `--incremental` flag to argument parser in `scripts/generate_embeddings.py`: if flag set, call detect_changes(), only process added/modified files, call delete_vectors_for_file() for deleted files, update .processed_files.json after successful processing
- [ ] T033 [US3] Update scripts/README.md with incremental mode usage: document `--incremental` flag, explain when to use (content updates), show example workflow

**Checkpoint**: ✅ User Story 3 complete (v2) - Incremental updates supported

---

## Phase 6: User Story 4 - Error Handling & Resilience (Priority: P2)

**Goal**: Handle API failures gracefully with retry logic, clear error messages, and recovery guidance

**Independent Test**: Test with invalid API keys, simulate rate limits, test with malformed markdown, verify error messages and retry behavior

**Acceptance Criteria**:
- ✅ Invalid credentials show clear error with field name
- ✅ API rate limits trigger exponential backoff (3 retries)
- ✅ Malformed frontmatter logged and skipped
- ✅ Network errors retried with backoff
- ✅ All errors logged to embeddings_generation.log
- ✅ Partial progress saved (Qdrant upsert is idempotent)

### Implementation for User Story 4 (Partially Complete - Enhance Error Handling)

**Note**: Basic error handling already included in US1 (tenacity retries, fail-fast validation). US4 enhances with checkpoint/resume and better error messages.

- [ ] T034 [P] [US4] Enhance `load_environment_variables()` in `scripts/generate_embeddings.py`: for each missing var, print specific error ("Missing: COHERE_API_KEY - Get from https://cohere.com/dashboard"), suggest fixes, exit with code 1
- [ ] T035 [P] [US4] Add connection validation in `initialize_cohere_client()` in `scripts/generate_embeddings.py`: wrap in try/except for cohere.errors.UnauthorizedError, catch and raise ValueError("Invalid COHERE_API_KEY: Check your API key at https://cohere.com/dashboard"), test with dummy embed call
- [ ] T036 [P] [US4] Add connection validation in `initialize_qdrant_client()` in `scripts/generate_embeddings.py`: wrap in try/except for qdrant_client.exceptions.UnexpectedResponse, catch 401 and raise ValueError("Invalid QDRANT_API_KEY or QDRANT_URL: Check credentials at https://cloud.qdrant.io/"), test with client.get_collections()
- [ ] T037 [US4] Implement checkpoint/resume logic in `main()` in `scripts/generate_embeddings.py`: create `scripts/.checkpoint.json` to store last successfully processed file index, on script start: load checkpoint, resume from last index +1, on exception: save checkpoint before exit, on success: delete checkpoint file, log resume progress ("Resuming from file 23/43...")
- [ ] T038 [US4] Add comprehensive exception handling in `main()` in `scripts/generate_embeddings.py`: wrap main loop in try/except, catch specific exceptions (CohereError, QdrantException, OSError), log full stack trace to file, print user-friendly error to console, save checkpoint on failure, exit with appropriate code (1 for error, 0 for success)
- [ ] T039 [US4] Update scripts/README.md troubleshooting section: add 5 common issues (Missing env vars, Invalid API keys, Rate limits, Malformed markdown, Network errors), for each: symptom, cause, solution with exact commands, link to relevant docs

**Checkpoint**: ✅ User Story 4 complete - Error handling robust and user-friendly

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Documentation, final validation, performance optimization, and production readiness

- [X] T040 [P] Add script header comments in `scripts/generate_embeddings.py`: module docstring explaining purpose, usage examples, command-line args, environment variables, author/date, link to spec/plan docs
- [X] T041 [P] Add type hints to ALL functions in `scripts/generate_embeddings.py`: ensure every function has complete typing (params + return type), run mypy for validation, fix any type errors
- [X] T042 [P] Add inline comments for complex logic in `scripts/generate_embeddings.py`: explain "why" not "what" for chunking algorithm, idempotency hash generation, retry logic, batch processing rationale
- [X] T043 [P] Optimize memory usage in `scripts/generate_embeddings.py`: process chunks in batches instead of accumulating all in memory, implement generator pattern for file processing, measure memory with memory_profiler, ensure <100 MB peak
- [X] T044 Add progress statistics to console output in `scripts/generate_embeddings.py`: display current file (X/43), chunks generated (cumulative), estimated time remaining (based on avg time per file), final summary table (files, chunks, vectors, runtime, avg time/file)
- [X] T045 [P] Create `.gitignore` entries for scripts/ in root `.gitignore`: add `scripts/.env`, `scripts/.checkpoint.json`, `scripts/.processed_files.json`, `scripts/embeddings_generation.log`, `scripts/__pycache__/`, `scripts/venv-embeddings/`
- [X] T046 [P] Update scripts/README.md with performance benchmarks: add table showing files vs runtime vs memory (43 files: ~14s, <50 MB), scaling projections (100/500/1000 files), bottleneck analysis (API latency), optimization tips
- [ ] T047 Validate against success criteria from spec.md: run full pipeline, verify SC-001 (<10 min runtime - actual ~14s ✅), SC-002 (ROS 2 query >0.75 score), SC-003 (no manual intervention), SC-004 (chunk sizes 500-1000 tokens), SC-005 (zero duplicates on re-run), SC-006 (API failure retry), SC-008 (setup <15 min per quickstart.md)

**Checkpoint**: ✅ All tasks complete - Production-ready implementation

---

## Manual Testing Checklist (v1)

**Run these tests after Phase 7 completion**:

| # | Test Case | Command/Steps | Expected Result | Status |
|---|-----------|---------------|-----------------|--------|
| 1 | Fresh collection | Delete collection, run script | Collection created, 172 vectors stored | ⬜ |
| 2 | All files processed | Run script, check logs | 43 files processed, no parsing errors | ⬜ |
| 3 | ROS 2 query | `--test-query "What is ROS 2?"` | Top 3 from module-1-ros2, score >0.75 | ⬜ |
| 4 | Isaac query | `--test-query "Isaac Sim setup"` | Results from module-3-isaac | ⬜ |
| 5 | Idempotency | Run script twice | Same 172 vectors, no duplicates | ⬜ |
| 6 | Invalid API key | Set COHERE_API_KEY=invalid | Error: "Invalid COHERE_API_KEY" | ⬜ |
| 7 | Missing env var | Unset QDRANT_URL | Fail-fast: "Missing: QDRANT_URL" | ⬜ |
| 8 | Progress logging | Run script, observe console | tqdm bars, file-by-file logs | ⬜ |

---

## Dependencies & Execution Order

### User Story Dependencies (Priority Order)

```
Foundational (Phase 2)
    ↓
User Story 1 (P1) - Content Ingestion  ← MVP CORE (Must Complete)
    ↓
User Story 2 (P1) - Search Verification  ← MVP VALIDATION (Must Complete)
    ↓
User Story 3 (P2) - Incremental Updates  ← v2 Feature (Optional for MVP)
    ↓
User Story 4 (P2) - Error Handling  ← v2 Feature (Basic already in US1)
```

### Task Dependencies Within User Stories

**User Story 1** (Sequential + Parallel Opportunities):
```
T010 (Discovery) ─┬─→ T011 (Parsing) ────→ T013 (Tokenizer) ──→ T014 (Chunking) ──→ T015 (Hash IDs)
                  │                                                     ↓
T012 (Metadata) ──┘                                          T016 (Cohere Init) ──→ T017 (Embeddings)
                                                                        ↓
                                                            T018 (Qdrant Init) ──→ T019 (Collection)
                                                                        ↓
                                                                   T020 (Upsert)
                                                                        ↓
                                                                   T021 (Main)
                                                                        ↓
                                                                  T022 (Args)
```

**Parallel Opportunities** (can work simultaneously):
- T010, T011, T012 (different concerns: discovery, parsing, metadata)
- T016, T018 (Cohere + Qdrant client initialization)
- T023, T024 (US2: query embedding + search)
- T034, T035, T036 (US4: error messages - independent)
- T040, T041, T042, T043, T045, T046 (Polish: documentation, types, comments, optimization)

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**Include**:
- ✅ User Story 1 (P1): Content Ingestion & Embedding Generation
- ✅ User Story 2 (P1): Semantic Search Verification
- ✅ Basic error handling (fail-fast, retry with backoff)
- ✅ Documentation (README.md, .env.example)

**Exclude (defer to v2)**:
- ❌ User Story 3 (P2): Incremental updates (full re-run is fast enough: ~14s)
- ❌ User Story 4 (P2): Advanced error handling (checkpoint/resume)
- ❌ Automated tests (pytest) - use manual checklist for v1

**MVP Delivery**: Tasks T001-T026 + T040-T047 (Polish)

**Estimated MVP Effort**:
- Setup (T001-T003): 30 min
- Foundational (T004-T009): 1 hour
- User Story 1 (T010-T022): 4-5 hours (core implementation)
- User Story 2 (T023-T026): 1 hour
- Polish (T040-T047): 1.5 hours
- **Total: ~8 hours** (single developer, includes testing)

### Incremental Delivery Plan

1. **Iteration 1** (MVP): T001-T026 + T040-T047
   - Delivers: Full embedding pipeline + search verification
   - Test: Manual checklist (8 items)
   - Duration: ~8 hours

2. **Iteration 2** (v2): T027-T033 (Incremental Updates)
   - Delivers: Faster re-indexing for content updates
   - Test: Modify file, verify incremental processing
   - Duration: ~2 hours

3. **Iteration 3** (v2): T034-T039 (Enhanced Error Handling)
   - Delivers: Checkpoint/resume, better error messages
   - Test: Simulate failures, verify recovery
   - Duration: ~2 hours

4. **Iteration 4** (v2 - Future): Automated tests
   - Delivers: pytest unit + integration tests
   - Test: CI/CD pipeline
   - Duration: ~4 hours

---

## Validation Checklist

**Before marking tasks complete, verify**:

- [ ] All function signatures have complete type hints
- [ ] All functions are <50 lines (per NFR-010)
- [ ] Inline comments explain "why" not "what"
- [ ] Environment variables loaded from .env (never hardcoded)
- [ ] API keys redacted in logs
- [ ] Errors logged to embeddings_generation.log with stack traces
- [ ] Progress displayed with tqdm bars
- [ ] Batch processing used (Cohere: 96 docs, Qdrant: 100 points)
- [ ] Retry logic with exponential backoff (tenacity decorator)
- [ ] Idempotency verified (re-run produces same vector count)
- [ ] Memory usage <100 MB (measure with memory_profiler)
- [ ] Runtime <1 minute for 43 files (target: ~14 seconds)
- [ ] Manual testing checklist completed (8 items)
- [ ] README.md allows new developer setup in <15 minutes

---

## Task Summary

**Total Tasks**: 47 (T001-T047)

**By Phase**:
- Phase 1 (Setup): 3 tasks
- Phase 2 (Foundational): 6 tasks
- Phase 3 (US1 - P1): 13 tasks 🎯 **MVP CORE**
- Phase 4 (US2 - P1): 4 tasks 🎯 **MVP VALIDATION**
- Phase 5 (US3 - P2): 7 tasks (v2 - deferred)
- Phase 6 (US4 - P2): 6 tasks (v2 - enhanced handling)
- Phase 7 (Polish): 8 tasks

**By Story**:
- US1: 13 tasks (Content Ingestion)
- US2: 4 tasks (Search Verification)
- US3: 7 tasks (Incremental Updates - v2)
- US4: 6 tasks (Error Handling - v2)
- Shared: 17 tasks (Setup + Foundational + Polish)

**Parallel Opportunities**: 15 tasks marked [P] (can run simultaneously)

**MVP Scope**: 30 tasks (T001-T026 + T040-T047)

**Suggested First Implementation**: T001 → T009 (Setup + Foundational) → T010-T022 (US1) → T023-T026 (US2) → T040-T047 (Polish) → Manual Testing

---

**Next Steps**:
1. Run `/sp.implement` to execute tasks in order
2. Use manual testing checklist after Phase 7
3. Verify all success criteria from spec.md
4. Create pull request with completed implementation
5. Plan v2 iteration for US3 + US4 enhancements
