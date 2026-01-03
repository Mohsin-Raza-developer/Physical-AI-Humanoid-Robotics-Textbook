# Quickstart Guide: ChatKit Robotics Chatbot Backend

**Feature**: ChatKit-Integrated Robotics Chatbot Backend
**Feature ID**: 008-chatkit-gemini-backend
**Created**: 2025-12-25
**Target Audience**: Backend developers

---

## Overview

This guide helps developers set up and run the robotics chatbot backend locally in **< 30 minutes**. The backend implements OpenAI's ChatKit protocol using FastAPI, PostgreSQL (Neon), Qdrant vector search, and Google Gemini AI.

### What You'll Build

By the end of this guide, you'll have:
- ✅ Working FastAPI server running on `http://localhost:8000`
- ✅ PostgreSQL database with `threads` and `messages` tables
- ✅ Connection to Qdrant vector database (cloud)
- ✅ Integration with Google Gemini AI via OpenAI-compatible endpoint
- ✅ ChatKit endpoint responding to test requests

---

## Prerequisites

### Required Software

| Software           | Minimum Version | Purpose                          | Install Link                              |
|--------------------|-----------------|----------------------------------|-------------------------------------------|
| **Python**         | 3.12+           | Runtime environment              | https://www.python.org/downloads/         |
| **Git**            | 2.30+           | Version control                  | https://git-scm.com/downloads             |
| **PostgreSQL**     | 14+             | Local development (optional)     | https://www.postgresql.org/download/      |
| **curl** or **httpie** | Any         | API testing                      | Pre-installed (macOS/Linux) or https://httpie.io/ |

**Note**: PostgreSQL is optional for local development if using Neon cloud database.

### Required Accounts & API Keys

| Service            | Purpose                          | Sign Up Link                              |
|--------------------|----------------------------------|-------------------------------------------|
| **Neon**           | PostgreSQL database (cloud)      | https://neon.tech/                        |
| **Qdrant Cloud**   | Vector database                  | https://cloud.qdrant.io/                  |
| **Google AI Studio** | Gemini API key                 | https://ai.google.dev/                    |
| **Better Auth**    | User authentication (pre-existing) | N/A (already deployed)                  |

### Verify Python Installation

```bash
python3 --version
# Expected: Python 3.12.0 or higher
```

If using Windows:
```cmd
python --version
```

---

## Installation

### 1. Clone Repository

```bash
git clone https://github.com/Physical-AI-Humanoid-Robotics-Textbook/chatbot-backend.git
cd chatbot-backend
```

### 2. Create Virtual Environment

**macOS/Linux**:
```bash
python3 -m venv venv
source venv/bin/activate
```

**Windows**:
```cmd
python -m venv venv
venv\Scripts\activate
```

**Verify activation** (you should see `(venv)` in your terminal prompt):
```bash
which python
# Expected: /path/to/chatbot-backend/venv/bin/python
```

### 3. Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

**Expected packages** (verify):
```bash
pip list | grep -E 'fastapi|chatkit|openai-agents|sqlalchemy|qdrant|cohere'
```

You should see:
- `fastapi>=0.115.0`
- `openai-chatkit>=1.0.0`
- `openai-agents>=1.0.0`
- `sqlalchemy>=2.0.0`
- `qdrant-client>=1.7.0`
- `cohere>=5.0.0`

---

## Environment Configuration

### 1. Create `.env` File

Copy the template:
```bash
cp .env.example .env
```

### 2. Configure Environment Variables

Open `.env` in your editor and fill in the following:

```bash
# ============================================
# Database Configuration (Neon PostgreSQL)
# ============================================
DATABASE_URL=postgresql+asyncpg://user:password@host.neon.tech/dbname?sslmode=require
# Get from: Neon Console → Project → Connection String

# ============================================
# Better Auth Configuration
# ============================================
BETTER_AUTH_DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require
# Same database URL as above (Better Auth uses sync driver)

BETTER_AUTH_SECRET=your-better-auth-secret-here
# Get from: Better Auth environment variables

# ============================================
# Qdrant Vector Database
# ============================================
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=textbook_content
# Get from: Qdrant Cloud Console → Cluster → API Keys

# ============================================
# Google Gemini AI
# ============================================
GEMINI_API_KEY=your-gemini-api-key-here
# Get from: https://ai.google.dev/gemini-api/docs/api-key

GEMINI_MODEL=gemini-2.0-flash-exp
# Model name (use latest Flash model for speed)

# ============================================
# OpenAI-Compatible Endpoint (for Gemini)
# ============================================
OPENAI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
OPENAI_API_KEY=${GEMINI_API_KEY}
# OpenAI SDK compatibility layer for Gemini

# ============================================
# Cohere Embeddings (for Qdrant search)
# ============================================
COHERE_API_KEY=your-cohere-api-key-here
# Get from: https://dashboard.cohere.com/api-keys

# ============================================
# Server Configuration
# ============================================
HOST=0.0.0.0
PORT=8000
RELOAD=true
LOG_LEVEL=info

# ============================================
# Security
# ============================================
CORS_ORIGINS=http://localhost:3000,https://textbook.example.com
# Comma-separated list of allowed origins

RATE_LIMIT_PER_MINUTE=60
# Requests per user per minute
```

### 3. Get Neon Database URL

1. Go to https://console.neon.tech/
2. Select your project
3. Navigate to **Dashboard** → **Connection String**
4. Copy the connection string (should look like):
   ```
   postgresql://user:password@ep-cool-name-123456.us-east-2.aws.neon.tech/dbname?sslmode=require
   ```
5. Convert to asyncpg format:
   ```
   postgresql+asyncpg://user:password@ep-cool-name-123456.us-east-2.aws.neon.tech/dbname?sslmode=require
   ```

### 4. Get Qdrant API Key

1. Go to https://cloud.qdrant.io/
2. Navigate to **Clusters** → Your Cluster
3. Click **API Keys** → **Create API Key**
4. Copy the API key and cluster URL

### 5. Get Gemini API Key

1. Go to https://ai.google.dev/
2. Click **Get API Key** → **Create API Key**
3. Copy the API key

**Verify `.env` is loaded**:
```bash
cat .env | grep DATABASE_URL
# Should show: DATABASE_URL=postgresql+asyncpg://...
```

---

## Database Setup

**Note**: Database tables will be automatically created during the implementation phase. For manual setup, use the SQL scripts below.

### 1. Verify Neon Database Connection

```bash
# Test connection
python3 -c "
import asyncio
from sqlalchemy.ext.asyncio import create_async_engine
import os
from dotenv import load_dotenv

load_dotenv()
engine = create_async_engine(os.getenv('DATABASE_URL'))

async def test_connection():
    async with engine.connect() as conn:
        print('✅ Database connection successful!')

asyncio.run(test_connection())
"
```

### 2. Create Tables (SQL Script)

The implementation will automatically create tables, but for reference, here's the SQL:

**Create `threads` table**:
```sql
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

CREATE TABLE threads (
    thread_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    title VARCHAR(255),
    metadata JSONB NOT NULL DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    CONSTRAINT chk_threads_timestamps CHECK (created_at <= updated_at)
);

CREATE INDEX idx_threads_user_id ON threads(user_id);
CREATE INDEX idx_threads_updated_at ON threads(updated_at DESC);
```

**Create `messages` table**:
```sql
CREATE TABLE messages (
    message_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    thread_id UUID NOT NULL REFERENCES threads(thread_id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL CHECK (LENGTH(content) > 0 AND LENGTH(content) <= 100000),
    sequence_number INTEGER NOT NULL CHECK (sequence_number >= 1),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    CONSTRAINT uq_messages_thread_sequence UNIQUE (thread_id, sequence_number)
);

CREATE INDEX idx_messages_thread_id ON messages(thread_id);
```

### 3. Verify Tables

**Using psql** (if PostgreSQL client installed):
```bash
psql $DATABASE_URL -c "\dt"
```

**Expected output**:
```
             List of relations
 Schema |     Name     | Type  |  Owner
--------+--------------+-------+---------
 public | threads      | table | user
 public | messages     | table | user
```

**Using Python**:
```python
python3 -c "
import asyncio
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy import text
import os
from dotenv import load_dotenv

load_dotenv()
engine = create_async_engine(os.getenv('DATABASE_URL'))

async def check_tables():
    async with engine.connect() as conn:
        result = await conn.execute(text(\"SELECT table_name FROM information_schema.tables WHERE table_schema='public'\"))
        print('Tables:', [row[0] for row in result])

asyncio.run(check_tables())
"
```

---

## Running the Server

### 1. Start Development Server

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 2. Verify Server is Running

Open http://localhost:8000 in your browser.

**Expected response**:
```json
{
  "message": "ChatKit Robotics Chatbot API",
  "version": "1.0.0",
  "status": "running"
}
```

### 3. Check Health Endpoint

```bash
curl http://localhost:8000/health
```

**Expected response**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-25T10:00:00Z",
  "services": {
    "database": "connected",
    "vector_db": "connected",
    "ai_model": "available"
  }
}
```

If any service shows `"disconnected"` or `"unavailable"`, check:
- Database URL is correct (`.env` → `DATABASE_URL`)
- Qdrant API key is valid (`.env` → `QDRANT_API_KEY`)
- Gemini API key is valid (`.env` → `GEMINI_API_KEY`)

---

## Testing the API

### 1. Create a Test Thread

**Using curl**:
```bash
curl -X POST http://localhost:8000/chatkit \
  -H "Authorization: Bearer your-better-auth-session-token" \
  -H "Content-Type: application/json" \
  -d '{
    "type": "thread.create",
    "metadata": {
      "title": "Test Thread"
    }
  }'
```

**Using httpie**:
```bash
http POST http://localhost:8000/chatkit \
  Authorization:"Bearer your-better-auth-session-token" \
  type=thread.create \
  metadata:='{"title": "Test Thread"}'
```

**Expected response**:
```json
{
  "type": "thread.created",
  "thread": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "created_at": "2025-12-25T10:00:00Z",
    "updated_at": "2025-12-25T10:00:00Z",
    "metadata": {
      "title": "Test Thread"
    }
  }
}
```

**Copy the `thread.id`** for the next step.

### 2. Send a Test Message

```bash
curl -X POST http://localhost:8000/chatkit \
  -H "Authorization: Bearer your-better-auth-session-token" \
  -H "Content-Type: application/json" \
  -H "Accept: text/event-stream" \
  -N \
  -d '{
    "type": "message.create",
    "thread_id": "550e8400-e29b-41d4-a716-446655440000",
    "message": {
      "role": "user",
      "text": "What is inverse kinematics?"
    }
  }'
```

**Expected response** (SSE stream):
```
event: message_start
data: {"type":"message_start","message":{"id":"msg-uuid","role":"assistant"}}

event: action
data: {"type":"action","action":{"name":"search_textbook","status":"started"}}

event: content_delta
data: {"type":"content_delta","delta":"Inverse kinematics"}

event: content_delta
data: {"type":"content_delta","delta":" is the process..."}

event: message_end
data: {"type":"message_end","message":{...}}
```

### 3. List Threads

```bash
curl -X POST http://localhost:8000/chatkit \
  -H "Authorization: Bearer your-better-auth-session-token" \
  -H "Content-Type: application/json" \
  -d '{
    "type": "threads.list",
    "limit": 20
  }'
```

**Expected response**:
```json
{
  "type": "threads.list",
  "threads": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440000",
      "created_at": "2025-12-25T10:00:00Z",
      "updated_at": "2025-12-25T10:05:00Z",
      "metadata": {
        "title": "Test Thread"
      }
    }
  ],
  "next_cursor": null
}
```

---

## Common Troubleshooting

### Issue 1: `ModuleNotFoundError: No module named 'app'`

**Cause**: Python cannot find the `app` module.

**Fix**:
```bash
# Ensure you're in the project root directory
pwd
# Should show: /path/to/chatbot-backend

# Check PYTHONPATH
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# Run server again
uvicorn app.main:app --reload
```

---

### Issue 2: `sqlalchemy.exc.OperationalError: connection refused`

**Cause**: Cannot connect to PostgreSQL database.

**Fix**:
1. Verify database URL:
   ```bash
   echo $DATABASE_URL
   ```
2. Test connection:
   ```bash
   psql $DATABASE_URL -c "SELECT 1"
   ```
3. Check Neon project is running (Neon Console → Project → Status)

---

### Issue 3: `qdrant_client.exceptions.UnexpectedResponse: 401 Unauthorized`

**Cause**: Invalid Qdrant API key.

**Fix**:
1. Verify API key:
   ```bash
   echo $QDRANT_API_KEY
   ```
2. Test Qdrant connection:
   ```python
   from qdrant_client import QdrantClient
   import os
   from dotenv import load_dotenv

   load_dotenv()
   client = QdrantClient(
       url=os.getenv('QDRANT_URL'),
       api_key=os.getenv('QDRANT_API_KEY')
   )
   print(client.get_collections())
   ```
3. Regenerate API key in Qdrant Cloud Console

---

### Issue 4: `openai.AuthenticationError: Invalid API key`

**Cause**: Invalid Gemini API key.

**Fix**:
1. Verify API key:
   ```bash
   echo $GEMINI_API_KEY
   ```
2. Test Gemini API:
   ```bash
   curl -X POST https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:generateContent \
     -H "Content-Type: application/json" \
     -H "x-goog-api-key: $GEMINI_API_KEY" \
     -d '{"contents":[{"parts":[{"text":"Hello"}]}]}'
   ```
3. Regenerate API key in Google AI Studio

---

### Issue 5: `401 Unauthorized` when testing ChatKit endpoint

**Cause**: Invalid or missing Better Auth session token.

**Fix**:
1. Get valid session token from Better Auth frontend
2. Extract token from browser cookies (`better-auth.session_token`)
3. Use token in `Authorization` header:
   ```bash
   curl -X POST http://localhost:8000/chatkit \
     -H "Authorization: Bearer <session-token-here>" \
     ...
   ```

**For local testing without Better Auth**:
- Create a test session in Better Auth database
- Or disable auth middleware temporarily (NOT for production)

---

## Next Steps

### 1. Frontend Integration

Integrate ChatKit React components in the Docusaurus textbook:

```bash
cd ../textbook-frontend
npm install @openai/chatkit-react
```

**Example React component**:
```tsx
import { ChatKitProvider, Chat } from '@openai/chatkit-react';

function RoboticsChatbot() {
  return (
    <ChatKitProvider
      apiUrl="http://localhost:8000/chatkit"
      getAuthToken={() => getBetterAuthSessionToken()}
    >
      <Chat />
    </ChatKitProvider>
  );
}
```

### 2. Development Workflow

**Making code changes**:
1. Edit files in `app/` directory
2. Server auto-reloads (if using `--reload` flag)
3. Test changes with curl/httpie
4. Commit changes to Git

**Database schema changes**:
1. Modify SQLAlchemy models in `app/models/`
2. Update SQL scripts in documentation
3. Apply changes directly to Neon database (during implementation)

### 3. Testing

**Run unit tests**:
```bash
pytest tests/
```

**Run integration tests**:
```bash
pytest tests/integration/
```

**Check code coverage**:
```bash
pytest --cov=app tests/
```

### 4. Deployment

See `deployment.md` for production deployment instructions.

---

## Useful Commands

### Database

```bash
# List all tables
psql $DATABASE_URL -c "\dt"

# Describe table schema
psql $DATABASE_URL -c "\d threads"

# Count threads
psql $DATABASE_URL -c "SELECT COUNT(*) FROM threads"

# Count messages
psql $DATABASE_URL -c "SELECT COUNT(*) FROM messages"

# View recent threads
psql $DATABASE_URL -c "SELECT thread_id, title, created_at FROM threads ORDER BY created_at DESC LIMIT 10"
```

### Server

```bash
# Start with auto-reload
uvicorn app.main:app --reload

# Start with specific host/port
uvicorn app.main:app --host 0.0.0.0 --port 8080

# Start with multiple workers (production)
uvicorn app.main:app --workers 4 --host 0.0.0.0 --port 8000
```

### Testing

```bash
# Run all tests
pytest

# Run specific test file
pytest tests/test_chatkit.py

# Run with coverage
pytest --cov=app --cov-report=html

# Run with verbose output
pytest -v
```

### Dependency Management

```bash
# Add new dependency
pip install package-name
pip freeze > requirements.txt

# Update dependencies
pip install --upgrade -r requirements.txt

# Check for security vulnerabilities
pip-audit
```

---

## Resources

**Official Documentation**:
- ChatKit Python SDK: https://openai.github.io/chatkit-python/
- OpenAI Agents SDK: https://openai.github.io/openai-agents-python/
- FastAPI: https://fastapi.tiangolo.com/
- SQLAlchemy 2.0: https://docs.sqlalchemy.org/en/20/
- Qdrant: https://qdrant.tech/documentation/
- Google Gemini: https://ai.google.dev/gemini-api/docs
- Neon PostgreSQL: https://neon.tech/docs

**Project Documentation**:
- Specification: `spec.md`
- Architecture Plan: `plan.md`
- Data Model: `data-model.md`
- API Contracts: `contracts/chatkit-api.json`
- Research: `research.md`

**Community**:
- Project GitHub: https://github.com/Physical-AI-Humanoid-Robotics-Textbook
- OpenAI ChatKit Samples: https://github.com/openai/openai-chatkit-advanced-samples

---

## Support

If you encounter issues not covered in this guide:

1. Check the project documentation in `specs/008-chatkit-gemini-backend/`
2. Review error logs: `tail -f logs/app.log`
3. Search GitHub issues: https://github.com/Physical-AI-Humanoid-Robotics-Textbook/chatbot-backend/issues
4. Create a new issue with:
   - Steps to reproduce
   - Error messages
   - Environment details (`python --version`, `pip list`)

---

## Changelog

| Date       | Change                                      | Author       |
|------------|---------------------------------------------|--------------|
| 2025-12-25 | Initial quickstart guide                    | AI Assistant |
