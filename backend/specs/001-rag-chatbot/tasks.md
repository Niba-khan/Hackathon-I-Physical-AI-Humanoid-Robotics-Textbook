---

description: "Task list for Integrated RAG Chatbot for AI-Native Textbook"
---

# Tasks: Integrated RAG Chatbot for AI-Native Textbook

**Input**: Design documents from `/specs/001-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan
- [x] T002 Initialize Python 3.11 project with FastAPI, Cohere SDK, Qdrant client, Pydantic, SQLAlchemy dependencies
- [x] T003 [P] Configure linting and formatting tools (Black, Flake8, MyPy)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [x] T004 Setup database schema and migrations framework using Neon Serverless Postgres
- [x] T005 [P] Implement Cohere API integration with proper environment variable configuration
- [x] T006 [P] Setup Qdrant Cloud vector store connection
- [x] T007 Create base models/entities that all stories depend on
- [x] T008 Configure error handling and logging infrastructure
- [x] T009 Setup environment configuration management with secure API key handling
- [x] T010 [P] Implement Cohere embedding model integration
- [x] T011 Setup rate limiting middleware for security compliance
- [x] T012 Implement input sanitization middleware

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Book-wide Q&A (Priority: P1) 🎯 MVP

**Goal**: Enable students to ask questions about the book content and receive accurate answers based on the entire book corpus

**Independent Test**: Can be fully tested by asking various questions about the book content and verifying that responses are grounded in the book text without hallucinations

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T013 [P] [US1] Contract test for POST /api/chat in tests/contract/test_chat_api.py
- [ ] T014 [P] [US1] Integration test for book-wide Q&A user journey in tests/integration/test_book_wide_qa.py

### Implementation for User Story 1

- [x] T015 [P] [US1] Create Document model in src/models/document.py for book content storage
- [x] T016 [P] [US1] Create ChatSession model in src/models/chat_session.py for conversation history
- [x] T017 [US1] Implement RAG service in src/services/rag_service.py with Cohere integration (depends on T015)
- [x] T018 [US1] Implement chat endpoint in src/api/chat.py with retrieval and generation logic
- [x] T019 [US1] Add validation and error handling for hallucination prevention
- [x] T020 [US1] Add logging for user story 1 operations with context tracking

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Selected-text-only Q&A (Priority: P2)

**Goal**: Allow students to select text and ask questions only about that selected text, so that they can get focused answers without interference from other book content

**Independent Test**: Can be fully tested by selecting text, asking questions about it, and verifying that answers only come from the selected text without external retrieval

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T021 [P] [US2] Contract test for POST /api/chat with selected-text-only mode in tests/contract/test_chat_api.py
- [ ] T022 [P] [US2] Integration test for selected-text-only Q&A user journey in tests/integration/test_selected_text_qa.py

### Implementation for User Story 2

- [x] T023 [P] [US2] Create SelectedText model in src/models/selected_text.py for user-selected content
- [x] T024 [US2] Implement selected-text-only Q&A service in src/services/selected_text_service.py
- [x] T025 [US2] Implement selected-text chat endpoint in src/api/chat.py with isolation enforcement
- [x] T026 [US2] Integrate with User Story 1 components (if needed)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Chat History Persistence (Priority: P3)

**Goal**: Preserve conversation history between user and chatbot so students can continue discussions about book content across different study sessions

**Independent Test**: Can be fully tested by having a conversation, closing the browser, returning later, and verifying that the conversation history is preserved

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T027 [P] [US3] Contract test for session persistence in tests/contract/test_session_api.py
- [ ] T028 [P] [US3] Integration test for chat history persistence in tests/integration/test_chat_history.py

### Implementation for User Story 3

- [x] T029 [P] [US3] Create EmbeddingMetadata model in src/models/embedding_metadata.py for vector storage tracking
- [x] T030 [US3] Implement vector search service in src/services/vector_search_service.py with Qdrant integration
- [x] T031 [US3] Implement advanced retrieval endpoint in src/api/retrieval.py with score thresholding

**Checkpoint**: All user stories should now be independently functional

---

[Add more user story phases as needed, following the same pattern]

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T032 [P] Documentation updates in docs/ including architecture diagram and RAG pipeline explanation
- [ ] T033 Code cleanup and refactoring
- [ ] T034 Performance optimization across all stories with token limit enforcement
- [x] T035 [P] Additional unit tests (if requested) in tests/unit/
- [ ] T036 Security hardening with additional input sanitization
- [x] T037 Run quickstart.md validation
- [ ] T038 [P] End-to-end adversarial testing to ensure no hallucinations
- [x] T039 API endpoint documentation with full examples
- [ ] T040 Environment setup documentation with reproducible end-to-end instructions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for POST /api/chat in tests/contract/test_chat_api.py"
Task: "Integration test for book-wide Q&A user journey in tests/integration/test_book_wide_qa.py"

# Launch all models for User Story 1 together:
Task: "Create Document model in src/models/document.py for book content storage"
Task: "Create ChatSession model in src/models/chat_session.py for conversation history"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence