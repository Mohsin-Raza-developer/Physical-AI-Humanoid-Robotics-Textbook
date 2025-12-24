# API Contracts Summary: ChatKit Gemini Backend

**Feature**: 008-chatkit-gemini-backend
**Date**: 2025-12-23
**Protocol**: REST + Server-Sent Events (SSE)
**Base URL**: `https://api.chatbot.example.com/api` (production)

---

## Authentication

All endpoints require JWT authentication via Authorization header:

```
Authorization: Bearer <jwt_token>
```

Token validation extracts `user_id` for request authorization.

---

## Core Endpoints

### Threads

#### POST /api/threads
Create a new conversation thread.

**Request**:
```json
{
  "title": "Optional Thread Title"
}
```

**Response** (201 Created):
```json
{
  "thread_id": "uuid-here",
  "user_id": "uuid-here",
  "title": null,
  "created_at": "2025-12-23T10:00:00Z",
  "updated_at": "2025-12-23T10:00:00Z"
}
```

---

#### GET /api/threads
List user's threads (most recent first).

**Query Parameters**:
- `limit` (int, default=20, max=100)
- `offset` (int, default=0)

**Response** (200 OK):
```json
{
  "threads": [
    {
      "thread_id": "uuid",
      "title": "ROS 2 Setup Help",
      "created_at": "2025-12-23T10:00:00Z",
      "updated_at": "2025-12-23T11:00:00Z"
    }
  ],
  "total": 150
}
```

---

#### GET /api/threads/{thread_id}
Get thread details with message history.

**Response** (200 OK):
```json
{
  "thread_id": "uuid",
  "title": "ROS 2 Setup",
  "messages": [
    {
      "message_id": "uuid",
      "role": "user",
      "content": "How do I install ROS 2?",
      "created_at": "2025-12-23T10:00:00Z",
      "sequence_number": 1,
      "attachments": []
    },
    {
      "message_id": "uuid",
      "role": "assistant",
      "content": "To install ROS 2...",
      "created_at": "2025-12-23T10:00:05Z",
      "sequence_number": 2,
      "attachments": []
    }
  ]
}
```

---

#### DELETE /api/threads/{thread_id}
Delete a thread (cascade deletes messages/attachments).

**Response** (204 No Content)

---

### Messages

#### POST /api/threads/{thread_id}/messages
Send a message and receive streamed AI response.

**Request**:
```json
{
  "content": "What is inverse kinematics?",
  "attachment_ids": ["uuid1", "uuid2"]  // optional (Phase 3 only, omit for MVP)
}
```

**Response** (200 OK, SSE Stream):
```
event: message_start
data: {"type":"message_start","message_id":"uuid","sequence_number":3}

event: content_delta
data: {"type":"content_delta","delta":"Inverse"}

event: content_delta
data: {"type":"content_delta","delta":" kinematics"}

event: action_event
data: {"type":"action","name":"search_knowledge_base","status":"started"}

event: progress_update
data: {"type":"progress","operation_id":"op1","message":"Searching...","progress":30}

event: action_event
data: {"type":"action","name":"search_knowledge_base","status":"completed","result":"Found 5 results"}

event: content_delta
data: {"type":"content_delta","delta":" is a method..."}

event: message_end
data: {"type":"message_end","message_id":"uuid"}
```

---

#### GET /api/threads/{thread_id}/messages
Get message history (paginated).

**Query Parameters**:
- `limit` (int, default=50, max=200)
- `offset` (int, default=0)

**Response** (200 OK):
```json
{
  "messages": [...],
  "total": 42
}
```

---

### Attachments

> **⚠️ Phase 3 Only**: Attachment endpoints are **NOT part of MVP** (Phase 1). These will be implemented in Phase 3 (Week 4-5) with Cloudflare R2 integration. Skip this section for initial implementation.

#### POST /api/attachments/upload-url
Generate signed URL for file upload.

**Request**:
```json
{
  "file_name": "robot_code.py",
  "file_type": "text/x-python",
  "file_size": 2048
}
```

**Response** (200 OK):
```json
{
  "attachment_id": "uuid",
  "upload_url": "https://r2.cloudflarestorage.com/...",
  "expires_in": 3600
}
```

**Usage**: Client uploads file directly to `upload_url` via PUT request.

---

#### GET /api/attachments/{attachment_id}/download-url
Generate signed URL for file download.

**Response** (200 OK):
```json
{
  "download_url": "https://r2.cloudflarestorage.com/...",
  "expires_in": 3600
}
```

---

## Event Schemas

### SSE Event Types

1. **message_start**: Indicates response generation has started
2. **content_delta**: Incremental text chunks
3. **action_event**: Agent tool invocation (search, reasoning)
4. **progress_update**: Long-running operation status
5. **widget**: Interactive UI component
6. **client_effect**: UI action instruction
7. **message_end**: Response generation complete

### Action Event Schema

```json
{
  "type": "action",
  "action_id": "uuid",
  "name": "search_knowledge_base",
  "status": "started" | "completed" | "failed",
  "input": {"query": "ROS 2 architecture"},
  "output": "Search results...",
  "timestamp": "2025-12-23T10:00:00Z"
}
```

### Progress Update Schema

```json
{
  "type": "progress",
  "operation_id": "uuid",
  "message": "Searching knowledge base...",
  "progress_percentage": 30,
  "timestamp": "2025-12-23T10:00:00Z"
}
```

### Widget Schema

```json
{
  "type": "widget",
  "widget_id": "uuid",
  "widget_type": "button_group",
  "properties": {
    "buttons": [
      {"label": "Option A", "value": "a"},
      {"label": "Option B", "value": "b"}
    ]
  },
  "action_handler": "server"
}
```

---

## Error Responses

### Standard Error Format

```json
{
  "error": {
    "code": "ERR_AUTH_INVALID",
    "message": "Invalid authentication token",
    "details": {}
  }
}
```

### Error Codes

- `ERR_AUTH_INVALID` (401): Invalid or expired JWT
- `ERR_NOT_FOUND` (404): Resource not found
- `ERR_FORBIDDEN` (403): User doesn't own resource
- `ERR_VALIDATION` (422): Invalid request payload
- `ERR_RATE_LIMIT` (429): Too many requests
- `ERR_INTERNAL` (500): Server error

---

## Rate Limits

- **Messages**: 10 requests/minute per user
- **Thread Creation**: 5 requests/minute per user
- **Attachment Uploads**: 3 requests/minute per user (Phase 3 only)

**Response Header**:
```
X-RateLimit-Limit: 10
X-RateLimit-Remaining: 7
X-RateLimit-Reset: 1703332800
```

---

## CORS Configuration

**Allowed Origins**: `https://mohsin-raza-developer.github.io`
**Allowed Methods**: GET, POST, DELETE
**Allowed Headers**: Authorization, Content-Type
**Credentials**: true

---

## Next Steps

1. Generate full OpenAPI 3.0 specification (openapi.yaml)
2. Implement endpoints in FastAPI
3. Add API documentation with Swagger UI
4. Test with Postman/Thunder Client

---

**Status**: ✅ Contract Summary Complete
**Full OpenAPI Spec**: To be generated in implementation phase
