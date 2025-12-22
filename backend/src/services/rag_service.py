from typing import List, Dict, Any, Optional
from src.services.cohere_service import CohereService
from src.services.vector_search_service import VectorSearchService, DocumentChunk
from src.core.config import settings
from src.core.error_handler import NoContextException, RAGException
import logging

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.cohere_service = CohereService()
        self.vector_service = VectorSearchService()

    def answer_from_corpus(self, query: str, top_k: int = None, threshold: float = None) -> Dict[str, Any]:
        """
        Answer a question using the full book corpus via RAG
        """
        if top_k is None:
            top_k = settings.top_k
        if threshold is None:
            threshold = settings.score_threshold

        # Validate query length
        if len(query) > settings.max_query_length:
            raise RAGException(
                f"Query exceeds maximum length of {settings.max_query_length} characters",
                "QUERY_TOO_LONG"
            )

        # Generate embedding for the query
        query_embedding = self.cohere_service.embed_query(query)

        # Search for relevant chunks in the vector store
        similar_chunks = self.vector_service.search_similar(
            query_embedding, 
            top_k=top_k, 
            threshold=threshold
        )

        if not similar_chunks:
            raise NoContextException("No relevant context found in the book corpus to answer this question.")

        # Prepare context from retrieved chunks
        context_parts = []
        sources = []
        total_length = 0

        for chunk in similar_chunks:
            chunk_text = chunk["content"]
            # Check if adding this chunk would exceed the max context length
            if total_length + len(chunk_text) > settings.max_context_length:
                logger.info(f"Context length limit reached. Using {len(context_parts)} out of {len(similar_chunks)} chunks.")
                break
            
            context_parts.append(chunk_text)
            sources.append({
                "document_id": chunk["document_id"],
                "section_ref": chunk["section_ref"],
                "content": chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],  # Truncate for display
                "score": chunk["score"]
            })
            total_length += len(chunk_text)

        context = "\n\n".join(context_parts)

        # Generate response using Cohere
        response = self.cohere_service.generate_response(query, context)

        return {
            "response": response,
            "sources": sources,
            "query_mode": "book-wide"
        }

    def answer_from_selected_text(self, query: str, selected_text: str) -> Dict[str, Any]:
        """
        Answer a question using only the provided selected text
        """
        # Validate query length
        if len(query) > settings.max_query_length:
            raise RAGException(
                f"Query exceeds maximum length of {settings.max_query_length} characters",
                "QUERY_TOO_LONG"
            )

        # Generate response using only the selected text as context
        response = self.cohere_service.generate_response(query, selected_text)

        # For selected-text mode, we return the selected text as the "source"
        sources = [{
            "document_id": "selected_text",
            "section_ref": "user_selection",
            "content": selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
            "score": 1.0  # Perfect relevance since it's the exact provided context
        }]

        return {
            "response": response,
            "sources": sources,
            "query_mode": "selected-text-only"
        }