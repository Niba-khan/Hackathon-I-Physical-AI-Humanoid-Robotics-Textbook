# Research Summary: Integrated RAG Chatbot for AI-Native Textbook

## Decision: Cohere API Implementation
**Rationale**: The project constitution mandates the use of Cohere API as the LLM provider with OpenAI strictly prohibited. Cohere's Command model is well-suited for Q&A tasks and provides the necessary controls for preventing hallucinations.

**Alternatives considered**: 
- OpenAI GPT models (prohibited by constitution)
- Open source models like Llama (would require more infrastructure management)

## Decision: Qdrant Vector Database
**Rationale**: The constitution specifies Qdrant Cloud (Free Tier) as the vector store. Qdrant provides excellent performance for semantic search with built-in scoring mechanisms that support the required threshold filtering.

**Alternatives considered**:
- Pinecone (not constitution-compliant)
- Weaviate (not constitution-compliant)
- Custom PostgreSQL with vector extensions (more complex than needed)

## Decision: FastAPI Framework
**Rationale**: The constitution mandates FastAPI as the API framework. FastAPI provides excellent performance, automatic API documentation, and built-in validation that supports the security requirements.

**Alternatives considered**:
- Flask (not constitution-compliant)
- Django (not constitution-compliant)
- Express.js (not constitution-compliant for this Python project)

## Decision: Neon Serverless Postgres
**Rationale**: The constitution specifies Neon Serverless Postgres for the relational database. Neon provides serverless scaling, which is cost-effective for varying academic usage patterns.

**Alternatives considered**:
- Traditional PostgreSQL (not constitution-compliant)
- MySQL (not constitution-compliant)
- MongoDB (not constitution-compliant for this use case)

## Decision: Selected-text Isolation Architecture
**Rationale**: The constitution requires strict enforcement of selected-text-only mode where the chatbot answers ONLY from provided text with external retrieval completely disabled. This will be implemented by bypassing the vector store when in selected-text mode.

**Alternatives considered**:
- Hybrid approach (violates constitution requirement)
- Separate models for each mode (unnecessarily complex)