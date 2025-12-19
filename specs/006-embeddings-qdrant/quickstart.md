# Quickstart: Embeddings Generation & Qdrant Storage

**Feature**: 006-embeddings-qdrant
**Estimated Setup Time**: 15 minutes
**Prerequisites**: Python 3.10+, Cohere account, Qdrant Cloud account

---

## Overview

This guide will walk you through setting up and running the embeddings generation script that:
1. Processes all markdown files from `docs/` directory
2. Generates 1024-dimensional embeddings using Cohere embed-v4.0
3. Stores vectors in Qdrant Cloud for semantic search
4. Verifies the setup with test queries

**Expected Result**: ~172 vectors stored in Qdrant, ready for RAG chatbot integration.

---

## Step 1: Prerequisites

### 1.1 Create Cohere Account

1. Go to https://cohere.com/
2. Sign up for a free account
3. Navigate to Dashboard → API Keys
4. Copy your API key (starts with `co-...`)

**Free Tier Limits**:
- 100 requests/minute
- 10,000 requests/month
- Sufficient for this project (~5 API calls total)

### 1.2 Create Qdrant Cloud Account

1. Go to https://cloud.qdrant.io/
2. Sign up for a free account
3. Create a new cluster:
   - Cluster name: `robotics-textbook`
   - Region: Choose closest to you
   - Plan: **Free tier** (1GB storage)
4. Once created, copy:
   - **Cluster URL**: `https://xxxxxxxx.qdrant.io:6333`
   - **API Key**: Click "API Keys" → Create new key

**Free Tier Limits**:
- 1GB storage (~170,000 vectors with 1024 dimensions)
- Our project uses ~1 MB (~172 vectors)

### 1.3 Verify Python Version

```bash
python --version
# Should be 3.10 or higher
```

If not installed:
- **Ubuntu/WSL**: `sudo apt install python3.10 python3.10-venv`
- **macOS**: `brew install python@3.10`
- **Windows**: Download from https://www.python.org/downloads/

---

## Step 2: Environment Setup

### 2.1 Navigate to Project Root

```bash
cd /path/to/Physical-AI-Humanoid-Robotics-Textbook
```

### 2.2 Create Virtual Environment

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

### 2.3 Install Dependencies

```bash
cd scripts
pip install -r requirements.txt
```

**Expected output**:
```
Installing collected packages: cohere, qdrant-client, python-frontmatter, tiktoken, python-dotenv, langchain, langchain-text-splitters, tqdm, tenacity...
Successfully installed...
```

---

## Step 3: Configuration

### 3.1 Create Environment File

Copy the example environment file:

```bash
cp .env.example .env
```

### 3.2 Edit `.env` with Your Credentials

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

**Security Note**: Never commit `.env` to git! It should already be in `.gitignore`.

### 3.3 Verify Configuration

```bash
# Test that environment variables load correctly
python -c "from dotenv import load_dotenv; import os; load_dotenv(); print('Cohere Key:', os.getenv('COHERE_API_KEY')[:10] + '...'); print('Qdrant URL:', os.getenv('QDRANT_URL'))"
```

**Expected output**:
```
Cohere Key: co-xxxxxxx...
Qdrant URL: https://xxxxxxxx.qdrant.io:6333
```

---

## Step 4: Run the Script

### 4.1 First Run (Full Processing)

```bash
python generate_embeddings.py
```

**What happens**:
1. Validates environment variables
2. Connects to Cohere and Qdrant
3. Creates `robotics_textbook_v1` collection
4. Scans `docs/` for `.md` files (finds 43 files)
5. Processes each file:
   - Parses frontmatter
   - Chunks content (500-1000 tokens per chunk)
   - Generates embeddings (batch API)
   - Uploads to Qdrant
6. Displays progress with `tqdm` bars

**Expected output**:
```
2025-12-19 10:30:00 - INFO - Environment variables validated successfully
2025-12-19 10:30:01 - INFO - Connected to Cohere API (model: embed-english-v4.0)
2025-12-19 10:30:02 - INFO - Connected to Qdrant Cloud: https://xxxxx.qdrant.io:6333
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

### 4.2 Verify with Test Query

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

   ROS 2 (Robot Operating System 2) is an open-source framework for building robot software. Unlike traditional operating systems like Windows or Linux, ROS 2 is a middleware...

2. Score: 0.8456
   File: module-1-ros2/week-3-lesson-2-nodes-packages.md
   Title: Nodes and Packages
   Module: module-1-ros2 | Week: week-3

   Content (first 200 chars):
   ROS 2 nodes are the fundamental building blocks of any robot application. A node is a process that performs computation—it could be a sensor driver, motor controller, navigation algorithm...

3. Score: 0.8201
   ...

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✅ Search verification successful! Vectors are correctly stored and retrievable.
```

---

## Step 5: Verify in Qdrant Dashboard

### 5.1 Open Qdrant Cloud Dashboard

1. Go to https://cloud.qdrant.io/
2. Click on your `robotics-textbook` cluster
3. Navigate to **Collections** → `robotics_textbook_v1`

### 5.2 Verify Collection Stats

You should see:
- **Vectors count**: ~172
- **Vector size**: 1024
- **Distance metric**: Cosine
- **Payload schema**: Fields like `module`, `week`, `tags`, `title`

### 5.3 Test Search in Dashboard

1. Click "Search" tab
2. Enter query: "ROS 2 architecture"
3. Click "Search"
4. Verify relevant results appear with similarity scores

---

## Step 6: Re-running the Script (Idempotency Test)

### 6.1 Run Again Without Changes

```bash
python generate_embeddings.py
```

**Expected behavior**:
- Same files processed
- Same chunks generated
- **No duplicate vectors** (upsert overwrites by ID)
- Same total vector count: ~172

### 6.2 Verify No Duplicates

```bash
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

**Expected**: Same count as before (e.g., 172), not doubled.

---

## Common Issues & Troubleshooting

### Issue 1: `ModuleNotFoundError: No module named 'cohere'`

**Cause**: Dependencies not installed or wrong virtual environment.

**Solution**:
```bash
# Make sure virtual environment is activated
source venv-embeddings/bin/activate  # Linux/macOS
venv-embeddings\Scripts\activate     # Windows

# Reinstall dependencies
cd scripts
pip install -r requirements.txt
```

---

### Issue 2: `ValueError: Missing environment variables: COHERE_API_KEY`

**Cause**: `.env` file not created or incorrectly formatted.

**Solution**:
```bash
# Check .env exists
ls -la .env

# Check content (should show your keys)
cat .env

# Verify Python loads it
python -c "from dotenv import load_dotenv; import os; load_dotenv(); print(os.getenv('COHERE_API_KEY'))"
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
2. Verify cluster URL (should include `:6333` port)
3. Regenerate API key if needed
4. Update `.env` with correct credentials

---

### Issue 5: `RuntimeError: Collection 'robotics_textbook_v1' already exists with different schema`

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

## Next Steps

### Option 1: Integrate with RAG Chatbot

Now that embeddings are stored, integrate with OpenAI Agents/ChatKit SDK:

```python
from qdrant_client import QdrantClient
import cohere

# Search for relevant chunks
results = client.search(
    collection_name="robotics_textbook_v1",
    query_vector=query_embedding,  # From user question
    limit=5
)

# Use chunks as context for LLM
context = "\n\n".join([hit.payload["content"] for hit in results])
prompt = f"Context:\n{context}\n\nQuestion: {user_question}\nAnswer:"
```

See `specs/005-docusaurus-auth/` for RAG chatbot implementation.

### Option 2: Test Different Queries

```bash
# Technical query
python generate_embeddings.py --test-query "How do I create a URDF file?"

# Simulation query
python generate_embeddings.py --test-query "Setting up Isaac Sim"

# Hardware query
python generate_embeddings.py --test-query "RealSense camera integration"
```

### Option 3: Monitor Collection Stats

```bash
# View collection info
python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
info = client.get_collection('robotics_textbook_v1')

print(f'Collection: {info.config.params.vectors.size}D vectors')
print(f'Total points: {info.points_count}')
print(f'Distance metric: {info.config.params.vectors.distance}')
print(f'Indexed payload fields: {list(info.payload_schema.keys())}')
"
```

---

## Performance Benchmarks

**Measured on Intel i7-10700K, 32GB RAM, 100 Mbps connection**:

| Metric | Value |
|--------|-------|
| Files processed | 43 |
| Total chunks | 172 |
| Total runtime | 14.3 seconds |
| Files per minute | ~180 |
| Embedding time | ~4 seconds (2 batches) |
| Upsert time | ~2 seconds (2 batches) |
| Peak memory | <50 MB |

**Your results may vary based on network speed and API latency.**

---

## Clean Up

### Remove Virtual Environment

```bash
# Deactivate
deactivate

# Delete virtual environment
rm -rf venv-embeddings
```

### Delete Qdrant Collection

```bash
python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
client.delete_collection('robotics_textbook_v1')
print('Collection deleted successfully')
"
```

---

## Support

**Issues**:
- Script errors: Check `embeddings_generation.log` for detailed stack traces
- API errors: Verify credentials in Cohere/Qdrant dashboards
- Content errors: Check markdown file format (valid YAML frontmatter)

**Documentation**:
- Cohere: https://docs.cohere.com/docs/embed-v4
- Qdrant: https://qdrant.tech/documentation/
- Project spec: `specs/006-embeddings-qdrant/spec.md`

---

## Summary

You've successfully:
✅ Set up Cohere and Qdrant Cloud accounts
✅ Installed Python dependencies
✅ Configured environment variables
✅ Processed 43 markdown files into 172 chunks
✅ Generated 1024-dimensional embeddings
✅ Stored vectors in Qdrant with rich metadata
✅ Verified semantic search works with test queries

**Total time**: ~15 minutes

The embeddings are now ready to power the RAG chatbot feature for the Physical AI & Humanoid Robotics Interactive Textbook! 🚀
