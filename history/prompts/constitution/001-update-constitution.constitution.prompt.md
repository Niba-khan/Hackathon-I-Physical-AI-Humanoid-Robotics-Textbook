---
id: 001
title: Update Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: Qwen
feature: none
branch: main
user: HT
command: /sp.constitution
labels: [constitution,rag,chatbot,docusaurus]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
Project: Integrated RAG Chatbot for AI-Native Technical Book (Docusaurus)

Purpose:
Design and implement a production-grade Retrieval-Augmented Generation (RAG) chatbot
embedded within a published Docusaurus book, capable of answering user questions strictly
based on the book's content and optionally restricted to user-selected text.

Core principles:
- Grounded generation: All responses must be based only on retrieved book content
- Zero hallucination tolerance: If information is not found, explicitly state "Not available in the book"
- Context discipline: Selected-text queries must ignore all other sources
- AI-native workflow: Spec-first, architecture-driven, implementation second
- Pedagogical clarity: Responses must be clear, concise, and beginner–intermediate friendly
- Production realism: Design must reflect real-world backend, vector DB, and API constraints

Knowledge boundaries:
- The chatbot MUST NOT answer using general world knowledge
- The chatbot MUST NOT infer beyond retrieved passages
- The chatbot MUST clearly indicate uncertainty or missing data
- The chatbot MUST prioritize exact textual evidence over paraphrasing

LLM constraints:
- Use a non-OpenAI LLM accessed via a third-party ("chore") API key
- OpenAI SDKs or OpenAI-hosted models are strictly prohibited
- Model access must be abstracted via a service layer

Retrieval standards:
- Vector storage: Qdrant Cloud (Free Tier)
- Embeddings must be generated only from book content (Markdown/MDX)
- Each chunk must include:
  - Source page
  - Section heading
  - Chunk ID
- Retrieval must support:
  - Full-book semantic search
  - Selected-text scoped search

Backend standards:
- API framework: FastAPI
- Database: Neon Serverless Postgres
- Store:
  - Chat sessions
  - User queries
  - Retrieved chunk references
- All endpoints must be stateless and documented

Frontend integration:
- Chatbot must be embedded inside the Docusaurus site
- UI must support:
  - Normal chat mode
  - "Ask from selected text" mode
- Clear visual distinction between:
  - User input
  - Model response
  - Source-based answers

Security & reliability:
- API keys must be stored in environment variables
- No secrets hard-coded in frontend or backend
- Rate limiting and input validation required
- Fail gracefully on retrieval or model errors

Quality standards:
- Responses must be:
  - Technically accurate
  - Aligned with book terminology
  - Written in simple, learner-friendly language
- Prefer structured answers (bullets, steps) where possible

Documentation & reproducibility:
- All system behavior must be explainable via specs
- Architecture and data flow must be documented
- RAG pipeline must be reproducible from raw book content

Success criteria:
- Chatbot answers are fully grounded in book content
- Selected-text queries reference only the provided text
- No hallucinated or external knowledge detected
- System deploys successfully with FastAPI + Qdrant + Neon
- Embedded chatbot works seamlessly inside Docusaurus
```

## Response snapshot

Updated the constitution file to reflect the new project focus on an Integrated RAG Chatbot for AI-Native Technical Book instead of the previous Physical AI & Humanoid Robotics textbook. The changes included:

1. Updated the project name and description
2. Revised all core principles to focus on RAG chatbot functionality
3. Added sections for Knowledge boundaries, LLM constraints, Retrieval standards, Backend standards, Frontend integration, Security & reliability, Quality standards, and Documentation & reproducibility
4. Updated the version from 1.0.0 to 2.0.0 to reflect the major changes
5. Maintained the governance section

## Outcome

- ✅ Impact: Constitution fully updated to reflect new RAG chatbot project
- 🧪 Tests: No tests required for constitution update
- 📁 Files: .specify/memory/constitution.md updated successfully
- 🔁 Next prompts: Ready to proceed with RAG chatbot planning and implementation
- 🧠 Reflection: The new constitution properly captures the project's scope and constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin with /sp.specify to create feature specifications