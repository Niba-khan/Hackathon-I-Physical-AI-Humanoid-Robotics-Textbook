# Implementation Plan: Physical AI & Humanoid Robotics — AI-Native Textbook + RAG Chatbot

**Branch**: `main-book-project` | **Date**: 2025-12-08 | **Spec**: [link to spec.md]
**Input**: Feature specification from `/specs/main-book-project/spec.md`

## Summary

Create a comprehensive AI-Native textbook for Physical AI & Humanoid Robotics with an integrated RAG chatbot. The project includes five core modules covering ROS 2, Digital Twins, AI-Robot Brains, Vision-Language-Action, and a Capstone. The textbook will be built with Docusaurus using MDX for rich content, and will include a RAG system using FastAPI, Neon Postgres, and Qdrant.

## Technical Context

**Language/Version**: Python 3.8+, JavaScript/TypeScript, MDX format for Docusaurus
**Primary Dependencies**: Docusaurus, FastAPI, Neon Postgres, Qdrant, OpenAI models, Tailwind CSS, shadcn/ui
**Storage**: Neon Serverless Postgres for metadata, Qdrant for vector storage
**Testing**: Unit tests for backend components, content validation, RAG accuracy tests
**Target Platform**: Web deployment via GitHub Pages for textbook, cloud deployment for RAG backend
**Project Type**: Full-stack application (book frontend + RAG backend)
**Performance Goals**: Fast content delivery via GitHub Pages, sub-2s response time for RAG queries
**Constraints**: Strictly follow Tailwind + shadcn/ui, no hallucinations in RAG responses, beginner-intermediate audience appropriate
**Scale/Scope**: Five core modules, RAG system, reusable components, validation checklists

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution:
- ✅ AI-Native, Spec-First Workflow: Following the spec-first approach as outlined in the feature spec
- ✅ Beginner-Intermediate Friendly Explanations: Content will be designed for the target audience with progressive complexity
- ✅ Technical Content Verification: All content will be verified against official documentation
- ✅ Modular MDX Structure: Content organized in MDX format with reusable components
- ✅ Full Transparency in Decision Making: All architectural decisions documented in Spec-Kit
- ✅ Zero Hallucination Safety Policy: RAG system will strictly answer from indexed content only

## Project Structure

### Documentation (this feature)

```text
specs/main-book-project/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Full-stack project
docs/                    # Docusaurus documentation structure (the book)
├── sidebar.js
├── src/
│   ├── components/      # Reusable React components (diagrams, callouts, etc.)
│   │   ├── diagrams/
│   │   ├── callouts/
│   │   └── code-blocks/
│   └── theme/
│       └── MDXComponents.js
├── static/              # Static assets
│   └── images/
└── docusaurus.config.js

backend/                 # FastAPI backend for RAG chatbot
├── app/
│   ├── main.py          # Application entry point
│   ├── models/          # Data models
│   ├── schemas/         # Pydantic schemas
│   ├── services/        # Business logic
│   ├── routers/         # API routes
│   ├── database/        # Database interactions
│   ├── rag/             # RAG-specific logic
│   └── tests/           # Backend tests
├── requirements.txt
└── alembic/             # Database migrations

frontend/                # Additional frontend if needed beyond Docusaurus
├── pages/
└── components/

package.json            # Root package file for the project
README.md               # Project documentation
```

**Structure Decision**: Full-stack project structure chosen as the project includes both the textbook (frontend) and RAG system (backend).