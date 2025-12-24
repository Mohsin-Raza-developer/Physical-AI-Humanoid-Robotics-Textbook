# ChatKit Gemini Backend

Production-ready FastAPI backend for the Physical AI & Humanoid Robotics chatbot, powered by Google Gemini and knowledge base search.

## Features

- 🤖 **AI-Powered Chat**: Google Gemini 2.0 Flash integration via OpenAI Agents SDK
- 📚 **Knowledge Base Search**: Automatic search of robotics textbook content (Qdrant + Cohere)
- ⚡ **Real-time Streaming**: Server-Sent Events (SSE) for streaming AI responses
- 🔐 **JWT Authentication**: Secure user authentication
- 💾 **PostgreSQL Database**: Thread and message persistence (Neon)
- 🚀 **Railway Deployment**: One-click deploy with auto-scaling

## Tech Stack

- **Framework**: FastAPI 0.115+ (async-first)
- **AI**: Google Gemini 2.0 Flash, OpenAI Agents SDK
- **Database**: Neon PostgreSQL + SQLAlchemy 2.0 (async)
- **Vector Search**: Qdrant Cloud + Cohere embed-v4.0
- **Deployment**: Railway

## Quick Start

### 1. Prerequisites

- Python 3.12+
- Neon PostgreSQL database
- API keys: Gemini, Cohere, Qdrant, JWT secret

### 2. Installation

```bash
# Clone repository
cd chatbot-backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Configuration

Create `.env` file:

```bash
cp .env.example .env
# Edit .env with your actual credentials
```

Required environment variables:
- `DATABASE_URL`: Neon PostgreSQL connection string
- `GEMINI_API_KEY`: Google Gemini API key
- `COHERE_API_KEY`: Cohere API key
- `QDRANT_URL`: Qdrant cluster URL
- `QDRANT_API_KEY`: Qdrant API key
- `JWT_SECRET`: JWT signing secret
- `FRONTEND_URL`: Frontend origin for CORS

### 4. Database Setup

```bash
# Initialize Alembic
alembic upgrade head
```

### 5. Run Development Server

```bash
uvicorn app.main:app --reload --port 8000
```

Visit: http://localhost:8000/docs for API documentation (Swagger UI)

### 6. Run Tests

```bash
pytest
```

## API Endpoints

### Threads
- `POST /api/threads` - Create new conversation
- `GET /api/threads` - List user's threads (paginated)
- `GET /api/threads/{id}` - Get thread with message history
- `DELETE /api/threads/{id}` - Delete thread

### Messages
- `POST /api/threads/{id}/messages` - Send message (SSE streaming response)
- `GET /api/threads/{id}/messages` - Get message history (paginated)

### Health
- `GET /health` - Health check endpoint

## Project Structure

```
chatbot-backend/
├── app/
│   ├── main.py              # FastAPI application
│   ├── config.py            # Configuration settings
│   ├── database.py          # Database connection
│   ├── models/              # SQLAlchemy models
│   ├── schemas/             # Pydantic schemas
│   ├── routers/             # API endpoints
│   ├── services/            # Business logic
│   │   ├── agent_service.py    # Gemini agent
│   │   ├── search_tool.py      # Knowledge base search
│   │   ├── thread_service.py   # Thread management
│   │   └── message_service.py  # Message management
│   ├── middleware/          # Auth, CORS, rate limiting
│   └── utils/               # Logging, errors
├── alembic/                 # Database migrations
├── tests/                   # Tests
├── .env.example             # Environment template
├── requirements.txt         # Python dependencies
├── railway.toml             # Railway deployment config
└── README.md                # This file
```

## Deployment (Railway)

1. Push code to GitHub
2. Connect Railway to repository
3. Add environment variables in Railway dashboard
4. Deploy automatically on push

Railway will use `railway.toml` for build and start commands.

## Development

### Adding New Features

1. Create database model in `app/models/`
2. Create Pydantic schema in `app/schemas/`
3. Create service in `app/services/`
4. Create router in `app/routers/`
5. Register router in `app/main.py`
6. Generate migration: `alembic revision --autogenerate -m "description"`
7. Apply migration: `alembic upgrade head`

### Code Quality

- **Linting**: Use `ruff` or `flake8`
- **Formatting**: Use `black`
- **Type Checking**: Use `mypy`
- **Testing**: Maintain >80% coverage

## Troubleshooting

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

## License

MIT License - See LICENSE file for details

## Support

For issues and questions, see project documentation in `/specs/008-chatkit-gemini-backend/`
