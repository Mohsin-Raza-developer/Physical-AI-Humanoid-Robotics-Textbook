---
id: 0001
title: RAG Chatbot Backend Specification
stage: spec
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 007-chatbot-backend
branch: 007-chatbot-backend
user: mohsin
command: /sp.specify
labels: ["rag", "chatbot", "backend-api", "gemini", "fastapi", "agents-sdk"]
links:
  spec: specs/007-chatbot-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-chatbot-backend/spec.md
  - specs/007-chatbot-backend/checklists/requirements.md
tests:
  - Specification quality validation (all checks passed)
---

## Prompt

Create a RAG Backend API using Python, FastAPI, and OpenAI Agents SDK using Gemini 2.0 Flash as the LLM.

## GIT BRANCH INSTRUCTION
**CRITICAL:** Before generating any code, validat that the user is on a new git branch named `007-chatboot-backend`. If not, instruct to create it:
`git checkout -b 007-chatboot-backend`

## CONTEXT
- **Database:** Qdrant Cloud (already populated with Cohere `embed-v4.0` vectors).
- **Goal:** Backend API for chatbot.

## TECH STACK
- **LLM:** `gemini-2.0-flash` (via OpenAI SDK Adapter).
- **Embeddings:** Cohere `embed-v4.0` (Must use this to query Qdrant).
- **Database:** Qdrant Cloud.
- **Framework:** FastAPI (`POST /api/chat`).
- **Agent SDK:** OpenAI Agents SDK (`agents` library).

## ARCHITECTURE
1. **Tool Definition:**
   - Create a function tool `@function_tool` named `search_knowledge_base`.
   - **Logic:**
     1. Accept `query` string.
     2. Convert query to vector using **Cohere API** (Critical: Dimension 1536).
     3. Search **Qdrant** with that vector.
     4. Return relevant chunks as text.

2. **Agent Setup:**
   - Initialize `Agent` with Gemini 2.0 model (Base URL: `https://generativelanguage.googleapis.com/v1beta/openai/`).
   - Give it the `search_knowledge_base` tool.
   - Instruction: "You are a helpful assistant. Use the search tool to answer questions about the book."

3. **API Endpoint:**
   - `POST /api/chat`
   - Accepts: `{ "message": "...", "user_id": "..." }`
   - Returns: `{ "response": "AI answer..." }`

## ENVIRONMENT VARIABLES
- GEMINI_API_KEY
- COHERE_API_KEY
- QDRANT_URL
- QDRANT_API_KEY

## REFERENCE DOCS
- **OpenAI Agents SDK:** https://openai.github.io/openai-agents-python/
- **Cohere Embeddings:** https://docs.cohere.com/reference/embed
- **FastAPI:** https://fastapi.tiangolo.com/
- **Qdrant Search:** https://qdrant.tech/documentation/concepts/search/

## DELIVERABLES
1. `backend/agent.py` (Agent & Tool logic)
2. `backend/main.py` (FastAPI app)
3. `backend/requirements.txt`
4. `backend/README.md`

## Response snapshot

**Specification Created**: RAG-Powered Chatbot Backend API

**Branch & Feature**: 007-chatbot-backend

**Key Sections Completed**:

1. **User Scenarios (4 prioritized stories)**:
   - P1: Ask Question and Receive Accurate Answer - Core chatbot functionality enabling students to get instant answers from textbook content
   - P2: Maintain Conversation Context - Multi-turn dialogue support for natural learning interactions
   - P2: Handle Out-of-Scope Questions - Prevents hallucinations by rejecting non-textbook questions
   - P3: Track User Interactions for Analytics - Enables course improvement through usage insights

2. **Functional Requirements (15 FRs)**:
   - FR-001 to FR-015 covering: question acceptance, vector search, response generation, context maintenance, error handling, validation, authentication, edge cases
   - All requirements testable and measurable (e.g., "MUST return responses within 5 seconds")

3. **Success Criteria (8 SCs)**:
   - Response time: <5 seconds for 95% of queries
   - Accuracy: 85% on test question set
   - Concurrency: 100 concurrent users without degradation
   - Scope detection: 90% accuracy
   - Context retention: 10 turns in dialogue
   - Uptime: 99%
   - User satisfaction: 4/5 or higher
   - Zero hallucinations in testing

4. **Additional Sections**:
   - 10 edge cases identified (empty messages, long queries, API failures, etc.)
   - 4 key entities defined (ChatMessage, ConversationSession, KnowledgeChunk, ChatResponse)
   - Dependencies documented (006-embeddings, 005-auth, external APIs)
   - Constraints specified (must use existing Qdrant collection, Cohere embed-v4.0, etc.)
   - Out of scope clearly bounded (no frontend UI, no auth implementation, no multi-language)
   - 7 risks with mitigations (API failures, hallucinations, rate limits, etc.)
   - 5 open questions for clarification (source citations, history limits, concurrent sessions, etc.)

**Quality Validation**: All checklist items passed ✅
- Technology-agnostic specification
- Measurable success criteria
- Clear user value focus
- Testable requirements
- No [NEEDS CLARIFICATION] markers (reasonable defaults used)

## Outcome

- ✅ Impact: Complete technology-agnostic specification created for RAG chatbot backend API
- 🧪 Tests: Specification quality checklist validation (all 14 items passed)
- 📁 Files:
  - specs/007-chatbot-backend/spec.md (comprehensive specification)
  - specs/007-chatbot-backend/checklists/requirements.md (quality validation)
- 🔁 Next prompts: Ready for `/sp.clarify` (if open questions need answers) or `/sp.plan` (to design architecture)
- 🧠 Reflection: Successfully translated technical requirements into business-focused specification. Avoided implementation details while capturing user's intent for RAG architecture with Gemini + Cohere + Qdrant stack. Prioritized user stories ensure MVP delivers core value (P1: Q&A) before enhancements (P2/P3).

## Evaluation notes (flywheel)

- Failure modes observed: None - specification passed all quality checks
- Graders run and results (PASS/FAIL):
  - Content Quality: PASS (4/4 items)
  - Requirement Completeness: PASS (8/8 items)
  - Feature Readiness: PASS (4/4 items)
- Prompt variant (if applicable): Standard /sp.specify workflow
- Next experiment (smallest change to try): Use /sp.plan to create technical architecture plan translating spec requirements into implementation design
