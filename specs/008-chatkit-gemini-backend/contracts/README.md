# API Contracts

**Feature**: ChatKit-Integrated Robotics Chatbot Backend
**Feature ID**: 008-chatkit-gemini-backend
**Created**: 2025-12-25

---

## Overview

This directory contains API contract specifications for the robotics chatbot backend. All contracts are defined using **OpenAPI 3.0.3** format for standardization and tooling support.

### Purpose

API contracts serve as:
1. **Single Source of Truth** - Canonical definition of request/response formats
2. **Frontend-Backend Agreement** - Clear interface for ChatKit React components
3. **Validation Schema** - Automated request/response validation
4. **Documentation** - Human-readable API documentation
5. **Code Generation** - Basis for client SDK generation (future)

---

## Files

### `chatkit-api.json`

**OpenAPI 3.0.3 specification** for the ChatKit protocol endpoint.

**Endpoints Defined**:
- `POST /chatkit` - Main ChatKit protocol endpoint (all operations)
- `GET /health` - Health check endpoint

**Operations Supported** (via `/chatkit` endpoint):
1. `thread.create` - Create new conversation thread
2. `message.create` - Send user message and stream AI response
3. `threads.list` - List user's threads (paginated)
4. `thread.get` - Get thread details with messages
5. `thread.delete` - Delete thread and all messages

**Authentication**:
- **Scheme**: HTTP Bearer (Better Auth session token)
- **Header**: `Authorization: Bearer <session-token>`
- **Validation**: Session verified against Better Auth database

**Response Formats**:
- **JSON**: For non-streaming operations (thread creation, listing, deletion)
- **Server-Sent Events (SSE)**: For streaming AI responses (message creation)

---

## Request/Response Examples

### 1. Create Thread

**Request**:
```http
POST /chatkit HTTP/1.1
Host: chatbot-backend.example.com
Authorization: Bearer <better-auth-session-token>
Content-Type: application/json

{
  "type": "thread.create",
  "metadata": {
    "title": "Inverse Kinematics Questions"
  }
}
```

**Response** (200 OK):
```json
{
  "type": "thread.created",
  "thread": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "created_at": "2025-12-25T10:00:00Z",
    "updated_at": "2025-12-25T10:00:00Z",
    "metadata": {
      "title": "Inverse Kinematics Questions"
    }
  }
}
```

---

### 2. Send Message (Streaming Response)

**Request**:
```http
POST /chatkit HTTP/1.1
Host: chatbot-backend.example.com
Authorization: Bearer <better-auth-session-token>
Content-Type: application/json
Accept: text/event-stream

{
  "type": "message.create",
  "thread_id": "550e8400-e29b-41d4-a716-446655440000",
  "message": {
    "role": "user",
    "text": "Explain inverse kinematics in robotics"
  }
}
```

**Response** (200 OK - SSE Stream):
```http
HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-cache
Connection: keep-alive

event: message_start
data: {"type":"message_start","message":{"id":"msg-uuid-here","role":"assistant"}}

event: action
data: {"type":"action","action":{"name":"search_textbook","status":"started","description":"Searching the robotics textbook for relevant content"}}

event: action
data: {"type":"action","action":{"name":"search_textbook","status":"completed"}}

event: content_delta
data: {"type":"content_delta","delta":"Inverse kinematics"}

event: content_delta
data: {"type":"content_delta","delta":" (IK) is the process"}

event: content_delta
data: {"type":"content_delta","delta":" of calculating joint angles"}

event: content_delta
data: {"type":"content_delta","delta":" needed to place a robot's"}

event: content_delta
data: {"type":"content_delta","delta":" end-effector at a desired"}

event: content_delta
data: {"type":"content_delta","delta":" position and orientation.[^1]\n\n"}

event: content_delta
data: {"type":"content_delta","delta":"[^1]: Chapter 5: Robot Kinematics, Section 5.3"}

event: message_end
data: {"type":"message_end","message":{"id":"msg-uuid-here","thread_id":"550e8400-e29b-41d4-a716-446655440000","role":"assistant","text":"Inverse kinematics (IK) is the process of calculating joint angles needed to place a robot's end-effector at a desired position and orientation.[^1]\n\n[^1]: Chapter 5: Robot Kinematics, Section 5.3","created_at":"2025-12-25T10:00:05Z"}}

```

---

### 3. List Threads

**Request**:
```http
POST /chatkit HTTP/1.1
Host: chatbot-backend.example.com
Authorization: Bearer <better-auth-session-token>
Content-Type: application/json

{
  "type": "threads.list",
  "limit": 20,
  "cursor": null
}
```

**Response** (200 OK):
```json
{
  "type": "threads.list",
  "threads": [
    {
      "id": "thread-1",
      "created_at": "2025-12-25T10:00:00Z",
      "updated_at": "2025-12-25T11:30:00Z",
      "metadata": {
        "title": "Inverse Kinematics Questions"
      }
    },
    {
      "id": "thread-2",
      "created_at": "2025-12-24T15:20:00Z",
      "updated_at": "2025-12-24T16:45:00Z",
      "metadata": {
        "title": "Forward Kinematics Tutorial"
      }
    }
  ],
  "next_cursor": "eyJ0aHJlYWRfaWQiOiJ0aHJlYWQtMiJ9"
}
```

**Pagination** (using cursor):
```http
POST /chatkit HTTP/1.1
Content-Type: application/json

{
  "type": "threads.list",
  "limit": 20,
  "cursor": "eyJ0aHJlYWRfaWQiOiJ0aHJlYWQtMiJ9"
}
```

---

### 4. Get Thread Details

**Request**:
```http
POST /chatkit HTTP/1.1
Host: chatbot-backend.example.com
Authorization: Bearer <better-auth-session-token>
Content-Type: application/json

{
  "type": "thread.get",
  "thread_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Response** (200 OK):
```json
{
  "type": "thread",
  "thread": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "created_at": "2025-12-25T10:00:00Z",
    "updated_at": "2025-12-25T10:05:00Z",
    "metadata": {
      "title": "Inverse Kinematics Questions"
    }
  },
  "messages": [
    {
      "id": "msg-1",
      "thread_id": "550e8400-e29b-41d4-a716-446655440000",
      "role": "user",
      "text": "Explain inverse kinematics in robotics",
      "created_at": "2025-12-25T10:00:02Z"
    },
    {
      "id": "msg-2",
      "thread_id": "550e8400-e29b-41d4-a716-446655440000",
      "role": "assistant",
      "text": "Inverse kinematics (IK) is the process of calculating joint angles needed to place a robot's end-effector at a desired position and orientation.[^1]\n\n[^1]: Chapter 5: Robot Kinematics, Section 5.3",
      "created_at": "2025-12-25T10:00:05Z"
    }
  ]
}
```

---

### 5. Delete Thread

**Request**:
```http
POST /chatkit HTTP/1.1
Host: chatbot-backend.example.com
Authorization: Bearer <better-auth-session-token>
Content-Type: application/json

{
  "type": "thread.delete",
  "thread_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Response** (200 OK):
```json
{
  "type": "thread.deleted",
  "thread_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

---

## Error Responses

All error responses follow the same format:

```json
{
  "error": {
    "type": "error_category",
    "message": "Human-readable error message",
    "details": {
      "field": "field_name",
      "constraint": "validation_rule"
    }
  }
}
```

### Error Types

| HTTP Status | Error Type             | Description                                    | Example                                  |
|-------------|------------------------|------------------------------------------------|------------------------------------------|
| 401         | `authentication_error` | Invalid or expired session token               | Session token expired                    |
| 403         | `authorization_error`  | User doesn't own the requested resource        | Cannot access thread owned by another user |
| 404         | `not_found_error`      | Thread or message not found                    | Thread ID does not exist                 |
| 422         | `validation_error`     | Request validation failed                      | Message content cannot be empty          |
| 429         | `rate_limit_error`     | Too many requests (rate limit exceeded)        | 60 requests/minute limit exceeded        |
| 500         | `internal_error`       | Unexpected server error                        | Database connection failed               |

### Example Error Responses

**401 Unauthorized**:
```json
{
  "error": {
    "type": "authentication_error",
    "message": "Invalid or expired session token"
  }
}
```

**403 Forbidden**:
```json
{
  "error": {
    "type": "authorization_error",
    "message": "You do not have permission to access this thread"
  }
}
```

**404 Not Found**:
```json
{
  "error": {
    "type": "not_found_error",
    "message": "Thread not found"
  }
}
```

**422 Validation Error**:
```json
{
  "error": {
    "type": "validation_error",
    "message": "Message content cannot be empty",
    "details": {
      "field": "message.text",
      "constraint": "min_length"
    }
  }
}
```

**429 Rate Limit**:
```json
{
  "error": {
    "type": "rate_limit_error",
    "message": "Rate limit exceeded. Please wait before making another request.",
    "details": {
      "limit": "60 requests/minute",
      "retry_after": 30
    }
  }
}
```

**500 Internal Error**:
```json
{
  "error": {
    "type": "internal_error",
    "message": "An unexpected error occurred. Please try again later."
  }
}
```

---

## SSE Stream Event Types

When creating a message, the response is a **Server-Sent Events (SSE)** stream with the following event types:

### Event: `message_start`

**Emitted**: When AI response generation begins
**Purpose**: Notify frontend that assistant message is starting

**Format**:
```
event: message_start
data: {"type":"message_start","message":{"id":"msg-uuid","role":"assistant"}}
```

---

### Event: `content_delta`

**Emitted**: For each text chunk generated
**Purpose**: Stream response text in real-time

**Format**:
```
event: content_delta
data: {"type":"content_delta","delta":"Inverse kinematics"}
```

---

### Event: `action`

**Emitted**: When agent tool is invoked (e.g., `search_textbook`)
**Purpose**: Show user that AI is searching knowledge base

**Statuses**:
- `started` - Tool invocation began
- `completed` - Tool invocation succeeded
- `failed` - Tool invocation failed

**Format**:
```
event: action
data: {"type":"action","action":{"name":"search_textbook","status":"started","description":"Searching the robotics textbook for relevant content"}}
```

```
event: action
data: {"type":"action","action":{"name":"search_textbook","status":"completed"}}
```

---

### Event: `message_end`

**Emitted**: When AI response generation completes
**Purpose**: Notify frontend of completion and provide full message

**Format**:
```
event: message_end
data: {"type":"message_end","message":{"id":"msg-uuid","thread_id":"thread-uuid","role":"assistant","text":"Full response text...","created_at":"2025-12-25T10:00:05Z"}}
```

---

## Health Check Endpoint

### GET /health

**Purpose**: Monitor backend service health

**Request**:
```http
GET /health HTTP/1.1
Host: chatbot-backend.example.com
```

**Response** (200 OK - Healthy):
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

**Response** (503 Service Unavailable - Unhealthy):
```json
{
  "status": "unhealthy",
  "timestamp": "2025-12-25T10:00:00Z",
  "services": {
    "database": "disconnected",
    "vector_db": "connected",
    "ai_model": "available"
  }
}
```

---

## Validation Rules

### Thread Creation

- `metadata.title`: Optional, max 255 characters
- `metadata`: Must be valid JSON object (not array or primitive)

### Message Creation

- `thread_id`: Must be valid UUID
- `message.role`: Must be exactly `"user"`
- `message.text`: Min 1 character, max 100,000 characters
- `message.text`: Cannot be empty or whitespace-only

### Threads Listing

- `limit`: Min 1, max 100 (default: 20)
- `cursor`: Optional, base64-encoded pagination token

### Thread Retrieval

- `thread_id`: Must be valid UUID
- User must own the thread (authorization check)

### Thread Deletion

- `thread_id`: Must be valid UUID
- User must own the thread (authorization check)
- Cascades to all messages (cannot be undone)

---

## Authorization Model

**Session-Based Authorization**:
1. Frontend sends Better Auth session token in `Authorization` header
2. Backend validates session against Better Auth's `session` table
3. Extract `user_id` from valid session
4. All thread/message operations filtered by `user_id`

**Authorization Rules**:
- Users can **only access threads they own** (`thread.user_id = session.user_id`)
- Thread ownership checked on every read/write operation
- No cross-user thread access (403 Forbidden if violated)

**Example Authorization Flow**:
```
1. Request: POST /chatkit with Authorization: Bearer <token>
2. Middleware: Validate <token> against Better Auth session table
3. Extract: user_id from session
4. Query: SELECT * FROM threads WHERE thread_id = ? AND user_id = ?
5. Success: 200 OK with thread data
6. Failure: 403 Forbidden (thread owned by another user) or 404 Not Found
```

---

## Rate Limiting

**Per-User Limits**:
- **60 requests per minute** per user
- Applies to all `/chatkit` operations
- Enforced at middleware level

**Response** (429 Too Many Requests):
```json
{
  "error": {
    "type": "rate_limit_error",
    "message": "Rate limit exceeded. Please wait before making another request.",
    "details": {
      "limit": "60 requests/minute",
      "retry_after": 30
    }
  }
}
```

**Headers** (included in 429 response):
```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 0
X-RateLimit-Reset: 1735128030
Retry-After: 30
```

---

## Testing

### Using OpenAPI Validators

**Python (openapi-core)**:
```python
from openapi_core import Spec
from openapi_core.validation.request import openapi_request_validator
from openapi_core.validation.response import openapi_response_validator

# Load spec
spec = Spec.from_file_path('contracts/chatkit-api.json')

# Validate request
result = openapi_request_validator.validate(spec, request)
if result.errors:
    raise ValidationError(result.errors)

# Validate response
result = openapi_response_validator.validate(spec, request, response)
if result.errors:
    raise ValidationError(result.errors)
```

### Using Swagger UI

1. Install Swagger UI:
   ```bash
   npm install -g swagger-ui-watcher
   ```

2. Serve OpenAPI spec:
   ```bash
   swagger-ui-watcher contracts/chatkit-api.json
   ```

3. Open http://localhost:8000 in browser to explore API

---

## Tools and Resources

**OpenAPI Validators**:
- Python: `openapi-core` (request/response validation)
- Node.js: `express-openapi-validator` (middleware)
- Java: `swagger-request-validator` (Spring Boot)

**Documentation Generators**:
- **Swagger UI**: Interactive API explorer
- **Redoc**: Clean, three-panel API documentation
- **Stoplight**: Collaborative API design platform

**Code Generators**:
- **OpenAPI Generator**: Client SDKs in 40+ languages
- **Prism**: Mock server from OpenAPI spec

---

## Related Documents

- **Specification**: `../spec.md` - Functional requirements
- **Data Model**: `../data-model.md` - Database schema and entities
- **Plan**: `../plan.md` - Implementation architecture
- **Research**: `../research.md` - ChatKit integration patterns

---

## Changelog

| Date       | Change                                      | Author       |
|------------|---------------------------------------------|--------------|
| 2025-12-25 | Initial API contracts (OpenAPI 3.0.3)       | AI Assistant |
