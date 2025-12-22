# API Documentation: RAG Chatbot Backend

## Overview
This document describes the API endpoints for the RAG Chatbot backend that is designed to be integrated with a Docusaurus-based textbook frontend.

## Base URL
The backend API is typically served at:
```
http://localhost:8000
```
For production deployments, replace with the appropriate domain.

## API Endpoints

### 1. Chat Endpoint
**POST** `/api/chat`

Main endpoint for chat interactions with the RAG system.

#### Request Body
```json
{
  "query": "Your question here",
  "mode": "book-wide" | "selected-text-only",
  "selected_text": "Text selected by user (required for selected-text-only mode)",
  "session_id": "unique session identifier (optional)"
}
```

#### Parameters
- `query` (string, required): The user's question
- `mode` (string, required): Either "book-wide" or "selected-text-only"
- `selected_text` (string, optional): Text content for selected-text-only mode
- `session_id` (string, optional): Session identifier to maintain conversation history

#### Response
```json
{
  "response": "The chatbot's answer",
  "sources": [
    {
      "document_id": "string",
      "section_ref": "string",
      "content": "string",
      "score": "number"
    }
  ],
  "session_id": "unique session identifier",
  "query_mode": "string"
}
```

#### Error Responses
- `422 Unprocessable Entity`: Invalid input parameters
- `500 Internal Server Error`: Processing error

### 2. Health Check Endpoint
**GET** `/health`

Endpoint to check the health status of the service.

#### Response
```json
{
  "status": "healthy",
  "timestamp": "2025-12-19T10:00:00Z",
  "version": "1.0.0"
}
```

## Frontend Integration Guide

### 1. Initialization
When the frontend loads, it should establish a connection to the backend API. No authentication is required for the basic endpoints.

### 2. Session Management
- The backend will generate a session ID if one is not provided
- The frontend should store and reuse the session ID to maintain conversation history
- Session IDs can be stored in browser's localStorage or sessionStorage

### 3. Mode Selection
The frontend should provide users with two interaction modes:
- **Book-wide Q&A**: Uses vector search across the entire book corpus
- **Selected-text-only Q&A**: Answers only from user-selected text

### 4. Handling Responses
- Display the `response` field as the chatbot's answer
- Show the `sources` with document references to maintain transparency
- Update the session ID if a new one is returned

### 5. Error Handling
- Display user-friendly error messages for API errors
- Implement retry logic for network failures
- Consider exponential backoff for repeated failures

## CORS Configuration
The backend should be configured to allow requests from the frontend domain. In development, this is typically:
```
Access-Control-Allow-Origin: http://localhost:3000
```

## Security Considerations
- All API keys are handled server-side, so the frontend doesn't need to manage secrets
- Input sanitization is performed on the backend
- Rate limiting is implemented on the backend