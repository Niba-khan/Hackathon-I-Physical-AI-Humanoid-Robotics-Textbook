# Data Model: Integrated RAG Chatbot for AI-Native Textbook

## Entity: Document
**Description**: Represents book content chunks for RAG retrieval, with content, embedding vector, and metadata
**Fields**:
- id: UUID (primary key)
- content: Text (the actual text content of the chunk)
- document_id: String (identifier for the original document/chapter)
- section_ref: String (reference to the section, page, or heading)
- chunk_index: Integer (position of this chunk in the document sequence)
- embedding_vector: Vector (the vector representation of the content)
- created_at: DateTime (timestamp of creation)
- updated_at: DateTime (timestamp of last update)

## Entity: ChatSession
**Description**: Stores conversation history between user and chatbot with query-response pairs
**Fields**:
- id: UUID (primary key)
- session_token: String (identifier for the session, tied to user if applicable)
- created_at: DateTime (timestamp of session creation)
- updated_at: DateTime (timestamp of last activity)
- is_active: Boolean (whether the session is currently active)

## Entity: ChatMessage
**Description**: Represents individual messages in a chat session
**Fields**:
- id: UUID (primary key)
- session_id: UUID (foreign key to ChatSession)
- role: String (either 'user' or 'assistant')
- content: Text (the message content)
- query_mode: String (either 'book-wide' or 'selected-text-only')
- selected_text: Text (the selected text if in selected-text-only mode, null otherwise)
- retrieved_chunks: JSON (references to chunks retrieved during this query)
- created_at: DateTime (timestamp of message creation)

## Entity: EmbeddingMetadata
**Description**: Tracks vector storage information including source document, chunk position, and embedding timestamp
**Fields**:
- id: UUID (primary key)
- document_id: String (identifier for the original document)
- chunk_id: String (identifier for the specific chunk)
- qdrant_point_id: String (identifier in the Qdrant vector store)
- embedding_model: String (the model used for embedding)
- embedding_timestamp: DateTime (when the embedding was created)
- source_url: String (URL of the source document)

## Entity: SelectedText
**Description**: Represents user-selected text for the selected-text-only Q&A mode
**Fields**:
- id: UUID (primary key)
- session_id: UUID (foreign key to ChatSession)
- text_content: Text (the selected text)
- char_start_pos: Integer (starting character position in the source)
- char_end_pos: Integer (ending character position in the source)
- source_url: String (URL where the text was selected from)
- created_at: DateTime (timestamp of selection)