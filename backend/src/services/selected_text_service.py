from typing import Dict, Any
from src.services.cohere_service import CohereService
from src.core.config import settings
from src.core.error_handler import RAGException
import logging

logger = logging.getLogger(__name__)

class SelectedTextService:
    def __init__(self):
        self.cohere_service = CohereService()

    def answer_from_selected_text(self, query: str, selected_text: str) -> Dict[str, Any]:
        """
        Answer a question using only the provided selected text
        This is a simplified version that focuses specifically on the selected-text-only functionality
        """
        # Validate query length
        if len(query) > settings.max_query_length:
            raise RAGException(
                f"Query exceeds maximum length of {settings.max_query_length} characters",
                "QUERY_TOO_LONG"
            )

        # Validate selected text is provided
        if not selected_text or not selected_text.strip():
            raise RAGException(
                "Selected text is required for selected-text-only mode",
                "MISSING_SELECTED_TEXT"
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