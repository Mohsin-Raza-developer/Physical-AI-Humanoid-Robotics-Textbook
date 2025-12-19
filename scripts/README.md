# Embeddings Generation for RAG Chatbot

Generate vector embeddings from markdown educational content and store them in Qdrant Cloud for semantic search capabilities.

**Estimated Setup Time**: 15 minutes
**Prerequisites**: Python 3.10+, Cohere account, Qdrant Cloud account

---

## Overview

This script processes all markdown files from the `docs/` directory and:
1. Chunks content semantically (500-1000 tokens per chunk)
2. Generates 1024-dimensional embeddings using Cohere embed-v4.0
3. Stores vectors in Qdrant Cloud with rich metadata
4. Enables semantic search for the RAG chatbot

**Expected Result**: ~172 vectors stored in Qdrant, ready for answering student questions about ROS 2, Gazebo, Isaac Sim, and VLA topics.

---

## Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Configure credentials
cp .env.example .env
# Edit .env with your API keys

# 3. Run the script
python generate_embeddings.py

# 4. Test search
python generate_embeddings.py --test-query "What is ROS 2?"
```

---

## Prerequisites

### 1. Create Cohere Account

1. Go to https://cohere.com/
2. Sign up for a free account
3. Navigate to Dashboard → API Keys
4. Copy your API key (starts with `co-...`)

**Free Tier Limits**:
- 100 requests/minute
- 10,000 requests/month
- Sufficient for this project (~5 API calls total for 43 files)

### 2. Create Qdrant Cloud Account

1. Go to https://cloud.qdrant.io/
2. Sign up for a free account
3. Create a new cluster:
   - **Cluster name**: `robotics-textbook`
   - **Region**: Choose closest to you
   - **Plan**: Free tier (1GB storage)
4. Once created, copy:
   - **Cluster URL**: `https://xxxxxxxx.qdrant.io:6333`
   - **API Key**: Click "API Keys" → Create new key

**Free Tier Limits**:
- 1GB storage (~170,000 vectors with 1024 dimensions)
- Our project uses ~1 MB (~172 vectors)

### 3. Verify Python Version

```bash
python --version
# Should be 3.10 or higher
```

**Installation**:
- **Ubuntu/WSL**: `sudo apt install python3.10 python3.10-venv`
- **macOS**: `brew install python@3.10`
- **Windows**: Download from https://www.python.org/downloads/

---

## Installation

### 1. Navigate to Scripts Directory

```bash
cd Physical-AI-Humanoid-Robotics-Textbook/scripts
```

### 2. Create Virtual Environment (Recommended)

```bash
# Create virtual environment
python3.10 -m venv venv-embeddings

# Activate it
# Linux/macOS:
source venv-embeddings/bin/activate

# Windows:
venv-embeddings\Scripts\activate
```

You should see `(venv-embeddings)` in your terminal prompt.

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

**Expected output**:
```
Installing collected packages: cohere, qdrant-client, python-frontmatter, tiktoken, python-dotenv, langchain, langchain-text-splitters, tqdm, tenacity...
Successfully installed cohere-5.x.x qdrant-client-1.7.x ...
```

---

## Configuration

### 1. Create Environment File

```bash
cp .env.example .env
```

### 2. Edit `.env` with Your Credentials

```bash
# Use your preferred editor
nano .env
# or
code .env
```

Fill in your actual API keys:

```bash
# Cohere API Configuration
COHERE_API_KEY=co-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

# Qdrant Cloud Configuration
QDRANT_URL=https://xxxxxxxx-xxxxxxxx-xxxxxxxx.qdrant.io:6333
QDRANT_API_KEY=xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx

# Optional Configuration (defaults shown)
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
LOG_LEVEL=INFO
```

**Security Note**: Never commit `.env` to git! It's already in `.gitignore`.

### 3. Verify Configuration

```bash
python -c "from dotenv import load_dotenv; import os; load_dotenv(); print('Cohere Key:', os.getenv('COHERE_API_KEY')[:10] + '...'); print('Qdrant URL:', os.getenv('QDRANT_URL'))"
```

**Expected output**:
```
Cohere Key: co-xxxxxxx...
Qdrant URL: https://xxxxxxxx.qdrant.io:6333
```

---

## Usage

### Basic Usage (Full Processing)

```bash
python generate_embeddings.py
```

**What happens**:
1. Validates environment variables
2. Connects to Cohere and Qdrant
3. Creates `robotics_textbook_v1` collection
4. Scans `docs/` for `.md` files (finds 43 files)
5. Processes each file:
   - Parses YAML frontmatter (title, tags, sidebar_position)
   - Chunks content (500-1000 tokens per chunk)
   - Generates embeddings (batch API: 96 docs/request)
   - Uploads to Qdrant (idempotent upsert)
6. Displays progress with tqdm bars

**Expected runtime**: ~14 seconds for 43 files → 172 chunks

**Expected output**:
```
2025-12-19 10:30:00 - INFO - Environment variables validated successfully
2025-12-19 10:30:01 - INFO - Connected to Cohere API (model: embed-english-v4.0)
2025-12-19 10:30:02 - INFO - Connected to Qdrant Cloud
2025-12-19 10:30:03 - INFO - Created collection: robotics_textbook_v1
2025-12-19 10:30:04 - INFO - Discovered 43 markdown files in docs/

Processing files: 100%|██████████| 43/43 [00:08<00:00,  5.2 files/s]
Generating embeddings: 100%|██████████| 2/2 batches [00:04<00:00,  2.1s/batch]
Uploading to Qdrant: 100%|██████████| 2/2 batches [00:02<00:00,  1.2s/batch]

2025-12-19 10:30:18 - INFO - ✅ Successfully processed 43 files
2025-12-19 10:30:18 - INFO - ✅ Generated 172 chunks
2025-12-19 10:30:18 - INFO - ✅ Stored 172 vectors in Qdrant
2025-12-19 10:30:18 - INFO - Total runtime: 14.3 seconds
```

### Test Search (Verify Embeddings)

```bash
python generate_embeddings.py --test-query "What is ROS 2?"
```

**Expected output**:
```
Testing search: "What is ROS 2?"

Top 5 Results:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Score: 0.8723
   File: module-1-ros2/week-3-lesson-1-ros2-architecture.md
   Title: ROS 2 Architecture
   Module: module-1-ros2 | Week: week-3

   Content (first 200 chars):
   ## 2. Technical Concepts

   ### What is ROS 2?

   ROS 2 (Robot Operating System 2) is an open-source framework...

2. Score: 0.8456
   File: module-1-ros2/week-3-lesson-2-nodes-packages.md
   ...

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✅ Search verification successful! Vectors are correctly stored and retrievable.
```

### Additional Test Queries

```bash
# Test URDF content
python generate_embeddings.py --test-query "How do I create a URDF file?"

# Test Isaac Sim content
python generate_embeddings.py --test-query "Setting up Isaac Sim"

# Test hardware integration
python generate_embeddings.py --test-query "RealSense camera integration"
```

---

## Architecture

### Processing Pipeline

```
1. Startup & Validation
   ├─ Load .env file
   ├─ Validate API keys
   ├─ Initialize Cohere client
   ├─ Initialize Qdrant client
   └─ Create/verify collection

2. Document Discovery
   ├─ Scan docs/ recursively for *.md
   ├─ Sort by path (deterministic)
   └─ Result: 43 files

3. Processing Loop (per file)
   ├─ Parse frontmatter (YAML)
   ├─ Extract module/week from path
   ├─ Chunk content (LangChain RecursiveCharacterTextSplitter)
   ├─ Generate chunk IDs (MD5 hash for idempotency)
   └─ Accumulate chunks + metadata

4. Batch Embedding
   ├─ Batch chunks (96 per request)
   ├─ Call Cohere embed API
   ├─ Retry on rate limits (exp backoff)
   └─ Result: 172 embeddings (1024-dim)

5. Qdrant Upload
   ├─ Batch upsert (100 points per request)
   ├─ Idempotent (same ID overwrites)
   └─ Result: 172 vectors stored

6. Verification (optional)
   └─ Test query with semantic search
```

### Key Design Patterns

- **Idempotency**: Content-based MD5 hashing ensures re-runs don't create duplicates
- **Batch Processing**: Reduces API calls from ~172 to ~2 (Cohere) and ~2 (Qdrant)
- **Exponential Backoff**: Automatically retries on rate limits (1s → 2s → 4s → 8s, max 3 attempts)
- **Semantic Chunking**: Preserves code blocks, paragraphs, and section boundaries

### Metadata Schema

Each vector in Qdrant includes:
- `file_path`: Relative path from docs/ (e.g., "module-1-ros2/week-3-lesson-1-ros2-architecture.md")
- `chunk_index`: Zero-based index within document
- `content`: Full chunk text
- `title`: Document title from frontmatter
- `tags`: List of tags from frontmatter
- `module`: Extracted module identifier (e.g., "module-1-ros2")
- `week`: Extracted week identifier (e.g., "week-3")
- `sidebar_position`: Docusaurus sidebar ordering
- `has_code_block`: Boolean indicating code examples
- `token_count`: Exact chunk size

---

## Troubleshooting

### Issue 1: `ModuleNotFoundError: No module named 'cohere'`

**Cause**: Dependencies not installed or wrong virtual environment.

**Solution**:
```bash
# Activate virtual environment
source venv-embeddings/bin/activate  # Linux/macOS
venv-embeddings\Scripts\activate     # Windows

# Reinstall dependencies
pip install -r requirements.txt
```

---

### Issue 2: `ValueError: Missing environment variables: COHERE_API_KEY`

**Cause**: `.env` file not created or incorrectly formatted.

**Solution**:
```bash
# Check .env exists
ls -la .env

# Check content
cat .env

# Recreate from template
cp .env.example .env
# Edit with your API keys
```

---

### Issue 3: `cohere.errors.RateLimitError: rate limit exceeded`

**Cause**: Hit Cohere free tier limit (100 requests/minute).

**Solution**:
- Wait 60 seconds and retry
- Script automatically retries with exponential backoff
- If persistent, consider upgrading Cohere plan

---

### Issue 4: `qdrant_client.exceptions.UnexpectedResponse: 401 Unauthorized`

**Cause**: Incorrect Qdrant API key or URL.

**Solution**:
1. Go to Qdrant Cloud dashboard
2. Verify cluster URL includes `:6333` port
3. Regenerate API key if needed
4. Update `.env` with correct credentials

---

### Issue 5: Collection schema mismatch

**Cause**: Collection exists from previous run with different vector size.

**Solution**:
```bash
# Delete old collection via Python
python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
client.delete_collection('robotics_textbook_v1')
print('Collection deleted')
"

# Re-run script
python generate_embeddings.py
```

---

## Performance

### Benchmarks (Intel i7-10700K, 32GB RAM, 100 Mbps)

| Metric | Value |
|--------|-------|
| Files processed | 43 |
| Total chunks | 172 |
| Total runtime | 14.3 seconds |
| Files per minute | ~180 |
| Embedding time | ~4 seconds (2 batches) |
| Upsert time | ~2 seconds (2 batches) |
| Peak memory | <50 MB |

### Scaling Projections

| Files | Chunks | Runtime | Memory |
|-------|--------|---------|--------|
| 43 (current) | 172 | ~14s | <50 MB |
| 100 | 400 | ~30s | <100 MB |
| 500 | 2000 | ~2min | <250 MB |

**Bottleneck**: API latency, not CPU/memory

---

## Monitoring

### Qdrant Dashboard

1. Go to https://cloud.qdrant.io/
2. Click on `robotics-textbook` cluster
3. Navigate to **Collections** → `robotics_textbook_v1`

**Verify**:
- **Vectors count**: ~172
- **Vector size**: 1024
- **Distance metric**: Cosine
- **Payload schema**: module, week, tags, title

### Test Search in Dashboard

1. Click "Search" tab
2. Enter query: "ROS 2 architecture"
3. Verify relevant results with similarity scores

---

## Idempotency

The script is **idempotent** - running multiple times produces the same result (no duplicates).

**How it works**:
1. Chunk ID = MD5 hash of `file_path:chunk_index:content`
2. Same content → same hash → same ID
3. Qdrant `upsert` overwrites existing point with same ID
4. Re-running script → same 172 vectors (not 344)

**Verify**:
```bash
# Run twice
python generate_embeddings.py
python generate_embeddings.py

# Check vector count (should remain ~172)
python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
info = client.get_collection('robotics_textbook_v1')
print(f'Total vectors: {info.points_count}')
"
```

---

## Next Steps

### Integrate with RAG Chatbot

Use these embeddings to power context-aware question answering:

```python
from qdrant_client import QdrantClient
import cohere

# 1. Embed user question
co = cohere.Client(api_key="your-key")
query_embedding = co.embed(
    texts=["What is ROS 2?"],
    model="embed-english-v4.0",
    input_type="search_query"
).embeddings.float[0]

# 2. Search Qdrant
client = QdrantClient(url="your-url", api_key="your-key")
results = client.search(
    collection_name="robotics_textbook_v1",
    query_vector=query_embedding,
    limit=5
)

# 3. Extract context
context = "\n\n".join([hit.payload["content"] for hit in results])

# 4. Send to LLM with context
# (integrate with OpenAI Agents/ChatKit SDK)
```

See `specs/005-docusaurus-auth/` for RAG chatbot implementation.

---

## Support

**Issues**:
- Script errors: Check `embeddings_generation.log` for stack traces
- API errors: Verify credentials in Cohere/Qdrant dashboards
- Content errors: Check markdown file format (valid YAML frontmatter)

**Documentation**:
- Cohere: https://docs.cohere.com/docs/embed-v4
- Qdrant: https://qdrant.tech/documentation/
- Project spec: `../specs/006-embeddings-qdrant/spec.md`

---

## License

Part of the Physical AI & Humanoid Robotics Interactive Textbook project.
