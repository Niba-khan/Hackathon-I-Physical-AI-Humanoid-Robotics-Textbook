# Implementation Plan: Integrated RAG Chatbot for AI-Native Textbook

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a production-ready Retrieval-Augmented Generation (RAG) chatbot embedded inside a Docusaurus book that can answer questions using the full book corpus or strictly from user-selected text only. The implementation will use Cohere API for LLM and embeddings, Qdrant Cloud for vector storage, and FastAPI with Neon Serverless Postgres for the backend. The system will enforce zero hallucinations and maintain strict content grounding according to the project constitution.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, Cohere SDK, Qdrant client, Pydantic, SQLAlchemy, psycopg2
**Storage**: Neon Serverless Postgres (relational data), Qdrant Cloud (vector embeddings)
**Testing**: pytest with unit, integration, and contract tests
**Target Platform**: Linux server (backend API), Web browser (frontend widget)
**Project Type**: Web application (backend API + frontend widget integration)
**Performance Goals**: Response time under 3 seconds for 95% of queries, support 100 concurrent users
**Constraints**: Cohere API rate limits, Qdrant Cloud Free Tier limitations, token limits for context window
**Scale/Scope**: Single textbook with multiple chapters, 1000+ daily users during academic periods

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### RAG & AI Standards Compliance
- [x] LLM Provider: Cohere API only (OpenAI strictly prohibited)
- [x] Embedding Model: Cohere embeddings
- [x] Vector Store: Qdrant Cloud (Free Tier)
- [x] Retrieval: Top-k semantic search with score thresholding
- [x] Context Window Control: Hard limit on retrieved tokens
- [x] Answer generation must cite retrieved chunks internally (traceable)

### Source-Grounded & Fidelity Requirements
- [x] No hallucinations: All responses must be grounded in retrieved content
- [x] Content fidelity: Answers must reflect source material verbatim when possible
- [x] Zero tolerance for fabricated information
- [x] Clear fallback responses when knowledge is missing

### Architecture & Security Standards
- [x] API Framework: FastAPI
- [x] Database: Neon Serverless Postgres
- [x] API keys loaded via environment variables only
- [x] No secrets committed to repository
- [x] Rate limiting enabled
- [x] Input sanitization required

### Quality & Success Criteria
- [x] Deterministic behavior for educational use
- [x] Modular, auditable AI architecture
- [x] Selected-text mode enforces strict isolation
- [x] System passes manual adversarial testing

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── document.py
│   │   ├── chat_session.py
│   │   ├── embedding_metadata.py
│   │   └── selected_text.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── selected_text_service.py
│   │   ├── vector_search_service.py
│   │   └── cohere_service.py
│   ├── api/
│   │   ├── chat.py
│   │   └── retrieval.py
│   ├── core/
│   │   ├── config.py
│   │   ├── security.py
│   │   └── database.py
│   └── main.py
├── tests/
│   ├── unit/
│   │   ├── test_rag_service.py
│   │   └── test_chat_endpoints.py
│   ├── integration/
│   │   └── test_rag_flow.py
│   └── contract/
│       └── test_api_contracts.py
├── requirements.txt
├── pyproject.toml
└── README.md
```

**Structure Decision**: Web application with backend API using the recommended structure. The backend contains models, services for RAG functionality, API endpoints, and core utilities. The tests are organized by type (unit, integration, contract). This structure supports the modular, auditable architecture required by the constitution.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
