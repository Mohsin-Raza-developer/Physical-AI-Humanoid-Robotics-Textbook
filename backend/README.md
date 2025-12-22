# RAG-Powered Chatbot Backend API

Backend API for the Physical AI & Humanoid Robotics Interactive Textbook chatbot.

## Features

- **RAG Architecture**: Retrieval-Augmented Generation using Gemini 2.0 Flash
- **Agent-Based Workflow**: Input guardrail → Main agent → Knowledge tool
- **Vector Search**: Qdrant Cloud for semantic search over textbook content
- **Personalization**: User profile-based response tailoring
- **Session Management**: Conversation context with once-per-session profile caching
- **Docusaurus Citations**: Clickable source links in all responses

## Tech Stack

- **Framework**: FastAPI 0.115+ (Python 3.12+)
- **LLM**: Gemini 2.0 Flash (via OpenAI-compatible API)
- **Embeddings**: Cohere embed-v4.0 (1536-dimensional)
- **Vector DB**: Qdrant Cloud (pre-populated with robotics textbook)
- **User DB**: Neon Serverless Postgres
- **Agent SDK**: OpenAI Agents SDK with tool-calling pattern

## Quick Start

For detailed setup instructions, see: [../specs/007-chatbot-backend/quickstart.md](../specs/007-chatbot-backend/quickstart.md)

### Prerequisites

- Python 3.12+
- Gemini API Key ([Get it here](https://ai.google.dev/))
- Cohere API Key ([Get it here](https://cohere.com/))
- Qdrant Cloud credentials (provided by project admin)
- Neon Database URL (provided by project admin)

### Installation

1. **Create virtual environment**:
   ```bash
   python3.12 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. **Install dependencies**:
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

3. **Configure environment variables**:
   ```bash
   cp .env.example .env
   # Edit .env with your actual API keys and credentials
   ```

4. **Run the server**:
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

5. **Verify installation**:
   ```bash
   curl http://localhost:8000/v1/health
   ```

   Expected response:
   ```json
   {
     "status": "healthy",
     "timestamp": "2025-12-21T...",
     "dependencies": {
       "neon_db": {"status": "up", "latency_ms": 45},
       "qdrant": {"status": "up", "latency_ms": 78},
       "cohere": {"status": "up", "latency_ms": 120},
       "gemini": {"status": "up", "latency_ms": 250}
     }
   }
   ```

## API Documentation

Once the server is running:

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc
- **OpenAPI Spec**: http://localhost:8000/openapi.json

## Project Structure

```
backend/
├── main.py              # FastAPI app entrypoint with lifespan management
├── agent.py             # Agent definitions (guardrail + main agent)
├── models.py            # Pydantic data models
├── database.py          # Database connections (Neon, Qdrant)
├── tools.py             # Knowledge retrieval tool (@function_tool)
├── session.py           # Session management
├── config.py            # Configuration & environment variables
├── middleware.py        # CORS, rate limiting, logging
├── requirements.txt     # Python dependencies
├── .env.example         # Environment variable template
├── tests/               # Test suite
│   ├── test_agent.py    # Agent behavior tests
│   ├── test_tools.py    # Tool functionality tests
│   ├── test_api.py      # API endpoint tests
│   └── conftest.py      # Pytest fixtures
└── README.md            # This file
```

## Development

### Run Tests

```bash
pytest tests/ -v
```

### Run Tests with Coverage

```bash
pytest tests/ --cov=. --cov-report=html
```

### Format Code

```bash
black .
isort .
```

### Lint Code

```bash
flake8 .
mypy .
```

### Type Checking

```bash
pyright .
```

## Configuration

All configuration is managed via environment variables in `.env`:

| Variable | Description | Required |
|----------|-------------|----------|
| `GEMINI_API_KEY` | Gemini 2.0 Flash API key | Yes |
| `COHERE_API_KEY` | Cohere embedding API key | Yes |
| `QDRANT_URL` | Qdrant Cloud URL | Yes |
| `QDRANT_API_KEY` | Qdrant API key | Yes |
| `DATABASE_URL` | Neon Postgres connection string | Yes |
| `JWT_SECRET` | JWT signing secret | Yes |
| `RATE_LIMIT_PER_MINUTE` | API rate limit per user | No (default: 20) |
| `API_HOST` | Server bind address | No (default: 0.0.0.0) |
| `API_PORT` | Server port | No (default: 8000) |
| `LOG_LEVEL` | Logging level | No (default: INFO) |

## Architecture

### Agent Workflow

```
User Query → FastAPI Endpoint
    ↓
Session Management (fetch profile once, cache as messages[0])
    ↓
Input Guardrail Agent (safety/relevance validation)
    ↓
Main Chatbot Agent (Gemini 2.0 Flash)
    ├─ System Message: User profile context
    ├─ Conversation History: Full messages array
    └─ Tool: search_knowledge_base
        ├─ Embed query (Cohere 1536-dim)
        ├─ Search Qdrant (top 5 results)
        └─ Return formatted context with citations
    ↓
Generate Response with Docusaurus Citations
    ↓
Return ChatResponse
```

### Database Connections

Application lifecycle managed with FastAPI `lifespan` events:

- **Startup**: Initialize connection pools (Neon, Qdrant, Cohere)
- **Per Request**: Reuse connections from pool (<1ms overhead)
- **Shutdown**: Close all connections gracefully

## API Endpoints

### Health Check

```bash
GET /v1/health
```

Returns service health status and dependency checks.

### Chat

```bash
POST /v1/chat
Content-Type: application/json

{
  "message": "What is ROS 2 architecture?",
  "user_id": "user_12345",
  "session_id": "sess_abc123"  // Optional for follow-up questions
}
```

Returns:

```json
{
  "response": "ROS 2 is... [ROS 2 Architecture](/docs/module-1/ros2)",
  "session_id": "sess_abc123",
  "citations": [
    {
      "chapter_title": "ROS 2 Architecture Overview",
      "doc_url": "/docs/module-1/week-3/ros2-architecture",
      "relevance_score": 0.92
    }
  ],
  "confidence_score": 0.88,
  "processing_time_ms": 1245,
  "token_count": 523
}
```

### Session Management

```bash
GET /v1/sessions/{session_id}      # Get session details
DELETE /v1/sessions/{session_id}   # End session
```

## Troubleshooting

### Server won't start

```bash
# Check if port 8000 is in use
lsof -i :8000  # macOS/Linux
netstat -ano | findstr :8000  # Windows

# Use different port
uvicorn main:app --reload --port 8001
```

### Database connection failed

```bash
# Verify DATABASE_URL
echo $DATABASE_URL

# Test connection
psql $DATABASE_URL
```

### Import errors

```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Reinstall dependencies
pip install -r requirements.txt
```

## Performance

- **Response Time**: <5s for 95% of queries
- **Concurrent Users**: 100+ without degradation
- **Profile Optimization**: 90% DB load reduction (once-per-session caching)
- **Connection Pooling**: 2-10 connections reused across requests

## Security

- JWT authentication required for all endpoints
- Rate limiting (20 req/min per user)
- Input validation and sanitization
- Safety guardrails for harmful content
- No exposure of technical errors to users

## Deployment

See [../specs/007-chatbot-backend/plan.md](../specs/007-chatbot-backend/plan.md) Section 7 for deployment architecture details.

## Support

- **Documentation**: See `../specs/007-chatbot-backend/` directory
- **Issues**: Create GitHub issue with error details
- **Contact**: [Your contact email]

## License

[Your license information]
