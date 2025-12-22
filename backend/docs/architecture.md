# Integrated RAG Chatbot Architecture

## Overview
The Integrated RAG Chatbot for AI-Native Textbook is designed to provide students with accurate answers based on book content using Retrieval-Augmented Generation (RAG) technology. The system strictly follows the principle of source-grounded responses with zero hallucinations.

## Architecture Components

### Frontend (Docusaurus Book UI)
- Embedded chat widget
- Two interaction modes:
  - Book-wide Q&A (vector search enabled)
  - Selected-text-only Q&A (vector search disabled)

### Backend (FastAPI)
- Stateless API layer
- Cohere integration for LLM and embeddings
- Input sanitization and rate limiting
- Session management

### Vector Store (Qdrant Cloud)
- Stores embeddings of book chunks
- Semantic search with score thresholding
- Metadata for traceability

### Data Store (Neon Serverless Postgres)
- Chat session persistence
- Query/response logging
- Embedding metadata storage

## RAG Pipeline

1. **Content Ingestion**: Book content is parsed, chunked, and indexed
2. **Embedding**: Content chunks are converted to embeddings using Cohere
3. **Query Processing**: User queries are received and processed
4. **Retrieval**: For book-wide mode, semantic search retrieves relevant chunks
5. **Generation**: Cohere generates responses based on retrieved context
6. **Response**: Answers are returned with source citations

## Security & Compliance
- All credentials loaded via environment variables
- Rate limiting on chat endpoint
- Input sanitization to prevent injection attacks
- No secrets committed to repository