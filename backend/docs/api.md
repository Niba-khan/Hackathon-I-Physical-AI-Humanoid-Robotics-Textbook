# API Documentation: RAG Chatbot Service

## Overview
This document provides detailed information about the API endpoints for the RAG Chatbot service embedded in the AI-Native Textbook.

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
  "query": "Your question here",
  "mode": "book-wide" | "selected-text-only",
  "selected_text": "Text selected by user (required for selected-text-only mode)",
  "session_id": "unique session identifier (optional)"
}
```

**Parameters:**
- `query`: The user's question (required)
- `mode`: The query mode - either "book-wide" or "selected-text-only" (required)
- `selected_text`: The user-selected text (required for selected-text-only mode)
- `session_id`: Session identifier for maintaining conversation history (optional)

#### Response
**Success Response (200 OK):**
```json
{
  "response": "The chatbot's answer to the query",
  "sources": [
    {
      "document_id": "chapter-3",
      "section_ref": "3.2.1",
      "content": "RAG combines a retrieval component with a generation component...",
      "score": 0.85
    }
  ],
  "session_id": "sess_abc123",
  "query_mode": "book-wide"
}
```

**Error Response (400 Bad Request):**
```json
{
  "error": "Query is required",
  "error_code": "INVALID_INPUT",
  "timestamp": "2025-12-19T10:00:00Z"
}
```

**Error Response (500 Internal Server Error):**
```json
{
  "error": "Failed to generate response due to service unavailability",
  "error_code": "INTERNAL_ERROR",
  "timestamp": "2025-12-19T10:00:00Z"
}
```

### POST /api/retrieval/search
Advanced retrieval endpoint with score thresholding.

#### Request
**Headers:**
- `Content-Type: application/json`
- `Authorization: Bearer {API_KEY}`

**Body:**
```json
{
  "query": "Your search query here",
  "top_k": 5,
  "threshold": 0.7
}
```

**Parameters:**
- `query`: The search query (required)
- `top_k`: Number of top results to retrieve (default: 5)
- `threshold`: Minimum similarity score for retrieval (default: 0.7)

#### Response
**Success Response (200 OK):**
```json
{
  "results": [
    {
      "id": "chunk_123",
      "content": "Retrieval-augmented generation (RAG) is a technique that...",
      "document_id": "chapter-3",
      "section_ref": "3.2.1",
      "score": 0.85,
      "metadata": {}
    }
  ]
}
```

### GET /api/health
Checks the health status of the service.

#### Response
**Success Response (200 OK):**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-19T10:00:00Z",
  "version": "1.0.0"
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
- `QUERY_TOO_LONG`: Query exceeds maximum length

## Rate Limiting
- Each API key is limited to 100 requests per minute
- Exceeding the limit results in a 429 status code
- Clients should implement exponential backoff for retries

## Security Considerations
- All API keys must be transmitted over HTTPS only
- Input sanitization is performed on all user queries
- Selected text is validated to prevent injection attacks
- No sensitive information is logged in plain text