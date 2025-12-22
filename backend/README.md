# Integrated RAG Chatbot for AI-Native Textbook

This project implements a production-ready Retrieval-Augmented Generation (RAG) chatbot embedded inside a Docusaurus book that can answer questions using the full book corpus or strictly from user-selected text only. The implementation uses Cohere API for LLM and embeddings, Qdrant Cloud for vector storage, and FastAPI with Neon Serverless Postgres for the backend. The system enforces zero hallucinations and maintains strict content grounding.

## Features

- Book-wide Q&A mode using vector search across the entire book corpus
- Selected-text-only Q&A mode that answers only from user-selected text
- Zero hallucination tolerance
- Content fidelity with answers reflecting source material verbatim when possible
- Deterministic behavior for educational use
- Modular, auditable AI architecture

## Tech Stack

- Python 3.11
- FastAPI
- Cohere API
- Qdrant Cloud
- Neon Serverless Postgres
- Pydantic
- SQLAlchemy

## Setup

1. Clone the repository
2. Create a virtual environment: `python -m venv venv`
3. Activate the virtual environment:
   - On Windows: `venv\Scripts\activate`
   - On macOS/Linux: `source venv/bin/activate`
4. Install dependencies: `pip install -r requirements.txt`
5. Set environment variables (see `.env.example`)
6. Run database migrations: `alembic upgrade head`
7. Start the server: `uvicorn src.main:app --reload`

## Environment Variables

Create a `.env` file with the following variables:

```
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
NEON_DATABASE_URL=your_neon_database_url
SECRET_KEY=your_secret_key_for_session_management
```

## API Endpoints

- `POST /api/chat` - Main chat endpoint supporting both book-wide and selected-text-only modes
- `GET /health` - Health check endpoint

## Testing

Run tests with: `pytest`

- Unit tests: `pytest tests/unit/`
- Integration tests: `pytest tests/integration/`
- Contract tests: `pytest tests/contract/`