# Quick Start Guide
## RAG-Powered Chatbot Backend API

**Feature**: 007-chatbot-backend
**Date**: 2025-12-20
**Status**: Development Ready

---

## Overview

This guide will help you set up and run the RAG Chatbot Backend API locally in under 10 minutes. The backend uses FastAPI, Gemini 2.0 Flash, Cohere embeddings, Qdrant vector database, and Neon PostgreSQL.

---

## Prerequisites

### Required Software

- **Python 3.12+**: [Download](https://www.python.org/downloads/)
- **Git**: [Download](https://git-scm.com/downloads/)
- **VS Code** (recommended): [Download](https://code.visualstudio.com/)

### Required Accounts & API Keys

1. **Gemini API Key** (Free Tier)
   - Visit: https://ai.google.dev/
   - Create API key
   - Copy key for `.env` file

2. **Cohere API Key** (Free Tier)
   - Visit: https://cohere.com/
   - Sign up and get API key
   - Copy key for `.env` file

3. **Qdrant Cloud** (Free Tier - already populated)
   - URL and API key provided by project admin
   - Collection `robotics_textbook_v1` pre-populated

4. **Neon Database** (Free Tier)
   - URL and credentials provided by project admin
   - `users` table already created

---

## Quick Setup (5 Minutes)

### Step 1: Clone Repository

```bash
# Clone the repository
git clone https://github.com/your-org/robotics-textbook.git
cd robotics-textbook

# Switch to feature branch
git checkout 007-chatbot-backend
```

### Step 2: Create Virtual Environment

```bash
# Create virtual environment
python3.12 -m venv venv

# Activate virtual environment
# On macOS/Linux:
source venv/bin/activate

# On Windows:
venv\Scripts\activate
```

### Step 3: Install Dependencies

```bash
# Upgrade pip
pip install --upgrade pip

# Install dependencies
pip install -r backend/requirements.txt
```

### Step 4: Configure Environment Variables

Create `.env` file in `backend/` directory:

```bash
# Navigate to backend directory
cd backend

# Create .env file
cat > .env << 'EOF'
# Gemini API
GEMINI_API_KEY=your_gemini_api_key_here

# Cohere API
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Neon Database
DATABASE_URL=postgresql://user:password@host/database

# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
API_ENV=development

# Security
JWT_SECRET=your_jwt_secret_here

# Rate Limiting
RATE_LIMIT_PER_MINUTE=20
EOF
```

**Important**: Replace all `your_*_here` placeholders with actual values!

### Step 5: Run the Server

```bash
# Run FastAPI server with auto-reload
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

**Expected Output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using statreload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

---

## Verify Installation

### Test 1: Health Check

```bash
# In a new terminal
curl http://localhost:8000/v1/health
```

**Expected Response**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-20T14:30:00Z",
  "dependencies": {
    "neon_db": {"status": "up", "latency_ms": 45},
    "qdrant": {"status": "up", "latency_ms": 78},
    "cohere": {"status": "up", "latency_ms": 120},
    "gemini": {"status": "up", "latency_ms": 250}
  }
}
```

### Test 2: Send Chat Message

```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer test_token" \
  -d '{
    "message": "What is ROS 2?",
    "user_id": "test_user"
  }'
```

**Expected Response**:
```json
{
  "response": "ROS 2 (Robot Operating System 2) is a middleware framework for robotic applications... [ROS 2 Architecture](/docs/module-1/week-3/ros2-architecture)",
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
  "token_count": 152
}
```

### Test 3: Continue Conversation

```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer test_token" \
  -d '{
    "message": "How do I install it?",
    "user_id": "test_user",
    "session_id": "sess_abc123"
  }'
```

---

## Interactive API Documentation

FastAPI provides automatic interactive API documentation:

### Swagger UI
Visit: http://localhost:8000/docs

Features:
- Interactive API explorer
- Try out endpoints directly
- View request/response schemas
- See example payloads

### ReDoc
Visit: http://localhost:8000/redoc

Features:
- Clean documentation layout
- Search functionality
- Downloadable OpenAPI spec

---

## Project Structure

```
backend/
├── main.py                 # FastAPI app entrypoint
├── agent.py                # Agent & tool definitions
├── models.py               # Pydantic data models
├── database.py             # Database connections
├── tools.py                # Knowledge retrieval tools
├── session.py              # Session management
├── config.py               # Configuration loader
├── requirements.txt        # Python dependencies
├── .env                    # Environment variables (DO NOT COMMIT)
├── tests/
│   ├── test_agent.py       # Agent tests
│   ├── test_tools.py       # Tool tests
│   └── test_api.py         # API endpoint tests
└── README.md               # Detailed documentation
```

---

## Common Commands

### Development

```bash
# Run server with auto-reload
uvicorn main:app --reload

# Run tests
pytest tests/ -v

# Run tests with coverage
pytest tests/ --cov=. --cov-report=html

# Format code
black .
isort .

# Lint code
flake8 .
mypy .

# Check type hints
pyright .
```

### Database

```bash
# Test Neon connection
python -c "import psycopg2; conn = psycopg2.connect('$DATABASE_URL'); print('Connected!')"

# Test Qdrant connection
python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='$QDRANT_URL', api_key='$QDRANT_API_KEY'); print(client.get_collections())"
```

### Debugging

```bash
# Enable debug logging
export LOG_LEVEL=DEBUG
uvicorn main:app --reload

# Run with Python debugger
python -m pdb -m uvicorn main:app --reload
```

---

## Development Workflow

### 1. Make Code Changes

Edit files in `backend/` directory:
- `agent.py`: Modify agent logic, tools
- `models.py`: Update data models
- `main.py`: Add/modify API endpoints

### 2. Test Changes

```bash
# Unit tests
pytest tests/test_agent.py -v

# Integration tests
pytest tests/test_api.py -v

# Manual testing via Swagger UI
# Visit http://localhost:8000/docs
```

### 3. Commit Changes

```bash
git add .
git commit -m "feat: add new chatbot feature"
git push origin 007-chatbot-backend
```

---

## Troubleshooting

### Issue: Import Error

**Symptom**: `ModuleNotFoundError: No module named 'X'`

**Solution**:
```bash
# Ensure virtual environment is activated
source venv/bin/activate  # or venv\Scripts\activate on Windows

# Reinstall dependencies
pip install -r backend/requirements.txt
```

### Issue: Database Connection Failed

**Symptom**: `ERR_DB_001: Could not connect to database`

**Solution**:
```bash
# Verify DATABASE_URL in .env
echo $DATABASE_URL

# Test connection manually
psql $DATABASE_URL
```

### Issue: Qdrant Search Returns Empty

**Symptom**: No search results returned

**Solution**:
```bash
# Verify collection exists
python -c "
from qdrant_client import QdrantClient
client = QdrantClient(url='$QDRANT_URL', api_key='$QDRANT_API_KEY')
print(client.get_collection('robotics_textbook_v1'))
"

# Check collection size
# Should show ~1000+ points if populated
```

### Issue: Gemini API Rate Limit

**Symptom**: `ERR_AGENT_003: Rate limit exceeded`

**Solution**:
- Wait 60 seconds before retrying
- Consider upgrading API tier
- Implement request queue for production

### Issue: Server Won't Start

**Symptom**: `Address already in use`

**Solution**:
```bash
# Find process using port 8000
lsof -i :8000  # macOS/Linux
netstat -ano | findstr :8000  # Windows

# Kill the process
kill -9 <PID>  # macOS/Linux
taskkill /PID <PID> /F  # Windows

# Or use different port
uvicorn main:app --reload --port 8001
```

---

## Next Steps

### For Developers

1. **Read Documentation**
   - `research.md`: Architectural decisions
   - `data-model.md`: Data structures
   - `contracts/openapi.yaml`: API specification

2. **Run Tests**
   ```bash
   pytest tests/ -v --cov
   ```

3. **Explore Code**
   - Start with `main.py` (entry point)
   - Review `agent.py` (core logic)
   - Check `tools.py` (knowledge retrieval)

4. **Make Changes**
   - Follow `/sp.tasks` workflow
   - Write tests first (TDD)
   - Keep commits focused

### For Testing

1. **Use Swagger UI**
   - Visit http://localhost:8000/docs
   - Test all endpoints interactively
   - Verify request/response formats

2. **Test Different Scenarios**
   - New conversation
   - Multi-turn dialogue
   - Out-of-scope questions
   - Error handling

3. **Monitor Logs**
   ```bash
   tail -f logs/app.log
   ```

### For Deployment

1. **Review `research.md` Section 7**: Deployment Architecture
2. **Set up staging environment**
3. **Configure production environment variables**
4. **Deploy to Railway/Render**
5. **Monitor with health checks**

---

## Useful Resources

### Documentation

- **FastAPI**: https://fastapi.tiangolo.com/
- **OpenAI Agents SDK**: https://openai.github.io/openai-agents-python/
- **Cohere Embeddings**: https://docs.cohere.com/reference/embed
- **Qdrant**: https://qdrant.tech/documentation/
- **Pydantic**: https://docs.pydantic.dev/

### Tools

- **Postman**: API testing alternative to Swagger UI
- **DBeaver**: Database GUI for exploring Neon DB
- **Qdrant Dashboard**: Explore vector collections

### Community

- **GitHub Issues**: Report bugs, request features
- **Slack/Discord**: Ask questions, get help
- **Stack Overflow**: Search existing solutions

---

## Support

### Get Help

- **Documentation**: Start with `README.md` in `backend/`
- **Issues**: Create GitHub issue with error details
- **Contact**: Email support@example.com

### Contributing

1. Fork repository
2. Create feature branch
3. Make changes with tests
4. Submit pull request
5. Respond to review feedback

---

## Summary

You should now have a fully functional RAG Chatbot Backend API running locally! 🎉

**Verify Checklist**:
- ✅ Server running on http://localhost:8000
- ✅ Health check returns `{"status": "healthy"}`
- ✅ Chat endpoint responds to test queries
- ✅ Swagger UI accessible at http://localhost:8000/docs
- ✅ All dependencies (Gemini, Cohere, Qdrant, Neon) connected

**Next**: Review the generated `tasks.md` file (created by `/sp.tasks` command) for detailed implementation tasks.

Happy coding! 🚀
