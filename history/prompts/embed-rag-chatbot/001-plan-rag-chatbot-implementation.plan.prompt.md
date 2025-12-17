---
id: 001
title: Plan RAG Chatbot Implementation
stage: plan
date: 2025-12-15
surface: agent
model: Qwen
feature: embed-rag-chatbot
branch: 002-embed-rag-chatbot
user: HT
command: /sp.plan
labels: [rag,chatbot,planning,architecture]
links:
  spec: specs/002-embed-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-embed-rag-chatbot/plan.md
 - specs/002-embed-rag-chatbot/research.md
 - specs/002-embed-rag-chatbot/data-model.md
 - specs/002-embed-rag-chatbot/quickstart.md
 - specs/002-embed-rag-chatbot/contracts/api-contract.yaml
 - QWEN.md
tests:
 - none
---

## Prompt

```
Project: Embedded RAG Chatbot for AI-Native Docusaurus Book

Planning objective:
Define a clear, production-ready architecture and execution plan
for building an embedded Retrieval-Augmented Generation (RAG)
chatbot that answers questions strictly from book content and
supports selected-text–only querying.

System architecture (high-level):

1. Content Layer
   - Source: Docusaurus Markdown / MDX files
   - Responsibility:
     - Provide authoritative knowledge base
     - Serve as the only source of truth

2. Ingestion & Indexing Layer
   - Parse MDX content
   - Chunk text with overlap
   - Attach metadata:
     - page_id
     - section_title
     - chunk_id
   - Generate embeddings using non-OpenAI embedding model
   - Store vectors + metadata in Qdrant Cloud

3. Vector Retrieval Layer
   - Perform semantic similarity search
   - Support:
     - Full-book retrieval
     - Selected-text scoped retrieval
   - Return top-k chunks only
   - Apply strict relevance filtering

4. Backend API Layer (FastAPI)
   - Stateless REST API
   - Core responsibilities:
     - Query preprocessing
     - Retrieval orchestration
     - Context assembly
     - LLM invocation
     - Response validation
   - Database integration:
     - Neon Postgres for chat logs and query traces

5. Generation Layer (LLM)
   - Non-OpenAI model accessed via third-party API
   - Prompted with:
     - Retrieved chunks only
     - Explicit grounding instructions
   - Forbidden:
     - External knowledge
     - Speculation or inference

6. Frontend Integration Layer
   - Embedded chat widget inside Docusaurus
   - UI features:
     - Normal chat mode
     - "Ask from selected text" mode
   - Clear feedback for unavailable answers

7. Observability & Safety Layer
   - Error handling for:
     - Empty retrieval
     - LLM failures
     - API rate limits
   - Graceful fallback responses
   - Logged retrieval metadata for debugging

---

Data flow (Full-book query):

User Question
   ↓
Frontend Chat UI
   ↓
FastAPI /query endpoint
   ↓
Semantic search in Qdrant
   ↓
Top-k book chunks retrieved
   ↓
Context-limited prompt sent to LLM
   ↓
Grounded answer generated
   ↓
Response returned to UI

---

Data flow (Selected-text query):

User selects text in book
   ↓
Selected text + question sent to backend
   ↓
No global vector search performed
   ↓
Context restricted to selected text
   ↓
LLM generates answer using selection only
   ↓
Response returned with restriction notice

---

Execution phases:

Phase 1 — Content ingestion
- Parse MDX files
- Chunk and embed content
- Populate Qdrant collection

Phase 2 — Backend RAG API
- Implement retrieval logic
- Connect Neon Postgres
- Enforce grounding rules

Phase 3 — Frontend embedding
- Build chat UI component
- Integrate with Docusaurus layout
- Add selected-text interaction

Phase 4 — Validation & hardening
- Test hallucination resistance
- Verify selected-text isolation
- Handle edge cases and failures

---

Key architectural decisions:
- Book content is the only knowledge source
- Retrieval precedes generation at all times
- Selected-text queries bypass vector database
- Credentials handled via environment variables only
- Free-tier cloud services respected

Constraints:
- No OpenAI SDK usage
- No web search or external documents
- No fine-tuning or training
- No authentication system

Risk mitigation:
- Empty retrieval → explicit "not available" response
- Overlong context → chunk limit enforcement
- API failure → graceful fallback message

Deliverables:
- RAG system architecture definition
- Data flow specifications
- Phase-wise execution plan
- Grounding and safety guarantees

Plan acceptance criteria:
- Architecture is implementable on free-tier infra
- Data flow supports both query modes
- System enforces strict grounding
- Spec aligns with constitution and specify steps
```

## Response snapshot

Completed comprehensive implementation planning for the Embedded RAG Chatbot for Docusaurus Book. Created the following artifacts:

1. Implementation Plan (plan.md): Detailed architecture with technical context, constitution check, and project structure
2. Research Summary (research.md): Key decisions on architecture, technology stack, content processing, LLM integration, frontend integration, retrieval strategy, and error handling
3. Data Model (data-model.md): Complete data model specification for BookChunk, ChatSession, and QueryResult entities
4. API Contracts (contracts/api-contract.yaml): OpenAPI specification for the backend API endpoints
5. Quickstart Guide (quickstart.md): High-level guide for setting up and running the system
6. Agent Context Update: Updated QWEN.md with project-specific technologies (Python 3.11, FastAPI, Qdrant, etc.)

The implementation plan includes a 4-phase execution approach (content ingestion, backend RAG API, frontend embedding, validation/hardening) with appropriate technology choices (Python, FastAPI, Qdrant Cloud, Neon Postgres) that align with project constraints (non-OpenAI LLM, free-tier infrastructure).

## Outcome

- ✅ Impact: Comprehensive architecture and implementation plan created for RAG chatbot
- 🧪 Tests: No automated tests for planning artifacts
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, API contract, and QWEN.md updated
- 🔁 Next prompts: Ready for /sp.tasks to break down implementation into specific tasks
- 🧠 Reflection: Planning phase successfully completed with all constitutional requirements addressed

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin with /sp.tasks to generate implementation tasks