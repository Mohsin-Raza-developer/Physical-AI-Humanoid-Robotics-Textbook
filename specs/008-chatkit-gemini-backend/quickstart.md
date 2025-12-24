# Quick Start Guide: ChatKit Gemini Backend

**Feature**: 008-chatkit-gemini-backend
**Target Audience**: Developers implementing the backend
**Prerequisites**: Python 3.12+, Neon PostgreSQL access, API keys (Gemini, Cohere, Qdrant, R2)

---

## 1. Environment Setup

### Install Python 3.12+

```bash
# Using pyenv (recommended)
pyenv install 3.12.0
pyenv local 3.12.0

# Verify
python --version  # Should show 3.12.x
```

### Clone Repository & Create Virtual Environment

```bash
cd /path/to/Physical-AI-Humanoid-Robotics-Textbook
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

---

## 2. Install Dependencies

### Create `requirements.txt` (Backend Directory)

```txt
# Web Framework
fastapi[standard]>=0.115.0
uvicorn[standard]>=0.27.0
python-multipart>=0.0.6

# ChatKit & AI
chatkit-python>=0.1.0
openai-agents-sdk>=0.1.0

# Database
sqlalchemy[asyncio]>=2.0.25
asyncpg>=0.29.0
alembic>=1.13.0
psycopg2-binary>=2.9.9

# Vector DB & Embeddings
qdrant-client>=1.7.0
cohere>=4.38.0

# Authentication
pyjwt>=2.8.0
python-jose[cryptography]>=3.3.0

# Utilities
python-dotenv>=1.0.0
pydantic>=2.5.0
pydantic-settings>=2.1.0

# Logging & Monitoring
structlog>=23.3.0
sentry-sdk[fastapi]>=1.39.0

# Rate Limiting
slowapi>=0.1.9

# Testing
pytest>=7.4.0
pytest-asyncio>=0.21.0
pytest-cov>=4.1.0
httpx>=0.26.0
```

### Install

```bash
pip install -r requirements.txt
```

---

## 3. Configuration

### Create `.env` File

```bash
# Database
DATABASE_URL=postgresql+asyncpg://user:password@neon-host/dbname

# AI Services
GEMINI_API_KEY=your_gemini_api_key_here
COHERE_API_KEY=your_cohere_api_key_here

# Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Authentication
JWT_SECRET=your_jwt_secret_from_auth_backend

# Application
FRONTEND_URL=https://mohsin-raza-developer.github.io
LOG_LEVEL=INFO
ENVIRONMENT=development

# Monitoring (optional)
SENTRY_DSN=your_sentry_dsn_here
```

---

## 4. Database Migration

### Initialize Alembic

```bash
# Create alembic directory structure
alembic init alembic

# Edit alembic/env.py to use async engine
# (See data-model.md for migration code)
```

### Run Migration

```bash
# Create migration
alembic revision --autogenerate -m "Create chatbot tables"

# Apply migration
alembic upgrade head
```

### Verify Tables

```sql
-- Connect to Neon database
\dt

-- Should see:
-- threads
-- messages
-- attachments
```

---

## 5. Project Structure

```
chatbot-backend/
├── app/
│   ├── __init__.py
│   ├── main.py                 # FastAPI app entry point
│   ├── config.py               # Settings (load from .env)
│   ├── database.py             # SQLAlchemy async engine
│   ├── models/
│   │   ├── __init__.py
│   │   ├── thread.py           # Thread SQLAlchemy model
│   │   ├── message.py          # Message SQLAlchemy model
│   │   └── attachment.py       # Attachment SQLAlchemy model
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── thread.py           # Pydantic schemas
│   │   ├── message.py
│   │   └── attachment.py
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── threads.py          # Thread endpoints
│   │   ├── messages.py         # Message endpoints (SSE)
│   │   └── attachments.py      # Attachment endpoints
│   ├── services/
│   │   ├── __init__.py
│   │   ├── agent_service.py    # Gemini + Agents SDK logic
│   │   ├── storage_service.py  # R2 signed URLs
│   │   └── search_tool.py      # Knowledge base search tool
│   ├── middleware/
│   │   ├── __init__.py
│   │   ├── auth.py             # JWT validation
│   │   └── rate_limit.py       # Rate limiting
│   └── utils/
│       ├── __init__.py
│       └── logger.py           # Structured logging setup
├── alembic/
│   └── versions/
├── tests/
│   ├── test_threads.py
│   ├── test_messages.py
│   └── test_auth.py
├── .env
├── .env.example
├── requirements.txt
└── README.md
```

---

## 6. Implement Core Components

### A. Main FastAPI App (`app/main.py`)

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routers import threads, messages, attachments
from app.config import settings
import sentry_sdk

# Initialize Sentry (optional)
if settings.SENTRY_DSN:
    sentry_sdk.init(dsn=settings.SENTRY_DSN)

app = FastAPI(
    title="ChatKit Gemini Backend",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[settings.FRONTEND_URL],
    allow_credentials=True,
    allow_methods=["GET", "POST", "DELETE"],
    allow_headers=["Authorization", "Content-Type"],
)

# Include routers
app.include_router(threads.router, prefix="/api")
app.include_router(messages.router, prefix="/api")
app.include_router(attachments.router, prefix="/api")

@app.get("/health")
async def health_check():
    return {"status": "healthy"}
```

### B. Agent Service (`app/services/agent_service.py`)

```python
from agents import Agent, AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig, Runner
from app.services.search_tool import search_knowledge_base
from app.config import settings

# Configure Gemini
gemini_client = AsyncOpenAI(
    api_key=settings.GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=gemini_client
)

config = RunConfig(model=model, tracing_disabled=True)

# Main chatbot agent
chat_agent = Agent(
    name="RoboticsAssistant",
    instructions="You are a helpful robotics tutor...",
    tools=[search_knowledge_base]
)

async def generate_response(user_message: str, conversation_history: list):
    """Generate AI response using Gemini + knowledge base."""
    result = await Runner.run(
        starting_agent=chat_agent,
        input=user_message,
        run_config=config
    )
    return result.final_output
```

### C. Search Tool (`app/services/search_tool.py`)

```python
from agents import function_tool
from qdrant_client import QdrantClient
import cohere
from app.config import settings
import asyncio

qdrant = QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY)
cohere_client = cohere.Client(api_key=settings.COHERE_API_KEY)

@function_tool
def search_knowledge_base(query: str) -> str:
    """Search robotics textbook knowledge base."""
    async def _run():
        # Embed query
        embedding = cohere_client.embed(
            texts=[query],
            model="embed-v4.0",
            input_type="search_query"
        ).embeddings[0]

        # Search Qdrant
        results = qdrant.query_points(
            collection_name="robotics_textbook_v1",
            query=embedding,
            limit=5,
            score_threshold=0.4
        ).points

        if not results:
            return "No relevant content found."

        return "\n".join(f"- {r.payload.get('text','')}" for r in results)

    return asyncio.run(_run())
```

### D. Message Router with SSE (`app/routers/messages.py`)

```python
from fastapi import APIRouter, Depends
from fastapi.responses import StreamingResponse
from app.services import agent_service
from app.middleware.auth import get_current_user

router = APIRouter()

@router.post("/threads/{thread_id}/messages")
async def send_message(
    thread_id: str,
    message: MessageCreate,
    user_id: str = Depends(get_current_user)
):
    async def event_stream():
        yield f"event: message_start\ndata: {json.dumps({'type':'message_start'})}\n\n"

        # Generate response
        response = await agent_service.generate_response(message.content, [])

        # Stream chunks
        for chunk in response.split():
            yield f"event: content_delta\ndata: {json.dumps({'delta':chunk})}\n\n"

        yield f"event: message_end\ndata: {json.dumps({'type':'message_end'})}\n\n"

    return StreamingResponse(event_stream(), media_type="text/event-stream")
```

---

## 7. Run Development Server

```bash
uvicorn app.main:app --reload --port 8000
```

**Visit**: http://localhost:8000/docs (Swagger UI)

---

## 8. Testing

```bash
# Run all tests
pytest

# With coverage
pytest --cov=app --cov-report=html

# View coverage
open htmlcov/index.html
```

---

## 9. Deployment (Railway)

### Create `railway.toml`

```toml
[build]
builder = "NIXPACKS"
buildCommand = "pip install -r requirements.txt"

[deploy]
startCommand = "uvicorn app.main:app --host 0.0.0.0 --port $PORT"
healthcheckPath = "/health"
healthcheckTimeout = 100
restartPolicyType = "ON_FAILURE"
```

### Deploy Steps

1. Push code to GitHub
2. Connect Railway to repository
3. Add environment variables in Railway dashboard
4. Deploy automatically on push

---

## 10. Troubleshooting

### Database Connection Issues

```bash
# Test connection
python -c "import asyncpg; asyncio.run(asyncpg.connect(os.environ['DATABASE_URL']))"
```

### Gemini API Errors

```bash
# Verify API key
curl -H "Authorization: Bearer $GEMINI_API_KEY" \
  https://generativelanguage.googleapis.com/v1beta/models
```

### Qdrant Connection

```python
from qdrant_client import QdrantClient
client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
print(client.get_collections())
```

---

## Next Steps

1. Implement remaining endpoints (threads, attachments)
2. Add comprehensive error handling
3. Set up logging with Structlog
4. Configure Sentry for error tracking
5. Write integration tests
6. Deploy to Railway
7. Integrate with frontend ChatKit React SDK

---

**Status**: ✅ Quick Start Guide Complete
**Estimated Setup Time**: 30-45 minutes
