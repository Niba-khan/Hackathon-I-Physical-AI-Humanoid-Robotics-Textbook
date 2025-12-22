# API Contract: RAG Chatbot Service

## Overview
This document defines the API contract for the RAG Chatbot service embedded in the AI-Native Textbook. The API enables users to ask questions about the book content in two modes: book-wide Q&A and selected-text-only Q&A.

## Base URL
`https://api.example.com` (to be replaced with actual deployment URL)

## Authentication
All API requests require an API key to be included in the header:
```
Authorization: Bearer {API_KEY}
```

## Endpoints

### POST /api/chat
Initiates a chat conversation or continues an existing one.

#### Request
**Headers:**
- `Content-Type: application/json`
- `Authorization: Bearer {API_KEY}`

**Body:**
```json
{
  "query": {
    "type": "string",
    "description": "The user's question",
    "example": "What is retrieval-augmented generation?"
  },
  "mode": {
    "type": "string",
    "enum": ["book-wide", "selected-text-only"],
    "description": "The query mode to use",
    "example": "book-wide"
  },
  "selected_text": {
    "type": "string",
    "description": "The user-selected text (required for selected-text-only mode)",
    "example": "Retrieval-augmented generation (RAG) is a technique that...",
    "nullable": true
  },
  "session_id": {
    "type": "string",
    "description": "Session identifier for maintaining conversation history",
    "example": "sess_abc123",
    "nullable": true
  }
}
```

#### Response
**Success Response (200 OK):**
```json
{
  "response": {
    "type": "string",
    "description": "The chatbot's answer to the query",
    "example": "Retrieval-augmented generation (RAG) is a technique that combines..."
  },
  "sources": {
    "type": "array",
    "items": {
      "type": "object",
      "properties": {
        "document_id": {
          "type": "string",
          "description": "ID of the source document",
          "example": "chapter-3"
        },
        "section_ref": {
          "type": "string",
          "description": "Reference to the specific section",
          "example": "3.2.1"
        },
        "content": {
          "type": "string",
          "description": "The source content snippet",
          "example": "RAG combines a retrieval component with a generation component..."
        }
      }
    },
    "description": "List of sources used to generate the response"
  },
  "session_id": {
    "type": "string",
    "description": "The session identifier (newly created or existing)",
    "example": "sess_abc123"
  },
  "query_mode": {
    "type": "string",
    "enum": ["book-wide", "selected-text-only"],
    "description": "The mode used for this query",
    "example": "book-wide"
  }
}
```

**Error Response (400 Bad Request):**
```json
{
  "error": {
    "type": "string",
    "description": "Error message explaining what went wrong",
    "example": "Query is required"
  }
}
```

**Error Response (500 Internal Server Error):**
```json
{
  "error": {
    "type": "string",
    "description": "Error message indicating server-side issue",
    "example": "Failed to generate response due to service unavailability"
  }
}
```

### GET /health
Checks the health status of the service.

#### Response
**Success Response (200 OK):**
```json
{
  "status": {
    "type": "string",
    "description": "Health status of the service",
    "example": "healthy"
  },
  "timestamp": {
    "type": "string",
    "format": "date-time",
    "description": "Timestamp of the health check",
    "example": "2025-12-19T10:00:00Z"
  }
}
```

## Error Handling

### Standard Error Format
All error responses follow this format:
```json
{
  "error": "Descriptive error message",
  "error_code": "Unique error code",
  "timestamp": "ISO 8601 formatted timestamp"
}
```

### Common Error Codes
- `INVALID_INPUT`: Request body does not match the required schema
- `MISSING_SELECTED_TEXT`: Selected text is required for selected-text-only mode
- `NO_CONTEXT_AVAILABLE`: No relevant context found to answer the question
- `SERVICE_UNAVAILABLE`: External service (Cohere, Qdrant) is unavailable
- `RATE_LIMIT_EXCEEDED`: API rate limit has been exceeded

## Rate Limiting
- Each API key is limited to 100 requests per minute
- Exceeding the limit results in a 429 status code
- Clients should implement exponential backoff for retries

## Security Considerations
- All API keys must be transmitted over HTTPS only
- Input sanitization is performed on all user queries
- Selected text is validated to prevent injection attacks
- No sensitive information is logged in plain text