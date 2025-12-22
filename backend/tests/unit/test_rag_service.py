import pytest
from src.services.rag_service import RAGService
from src.services.cohere_service import CohereService
from unittest.mock import Mock, patch

class TestRAGService:
    def setup_method(self):
        # Create a mock CohereService and VectorSearchService to avoid external dependencies in tests
        self.mock_cohere_service = Mock(spec=CohereService)
        self.mock_cohere_service.embed_query.return_value = [0.1, 0.2, 0.3]
        self.mock_cohere_service.generate_response.return_value = "Test response"
        
        # Create a mock vector service
        self.mock_vector_service = Mock()
        self.mock_vector_service.search_similar.return_value = [
            {
                "id": "test_id",
                "content": "Test content",
                "document_id": "test_doc",
                "section_ref": "1.1",
                "score": 0.8,
                "metadata": {}
            }
        ]
        
        # Create RAGService with mocked dependencies
        self.rag_service = RAGService()
        self.rag_service.cohere_service = self.mock_cohere_service
        self.rag_service.vector_service = self.mock_vector_service

    def test_answer_from_corpus(self):
        query = "What is RAG?"
        
        result = self.rag_service.answer_from_corpus(query)
        
        # Verify that the correct methods were called
        self.mock_cohere_service.embed_query.assert_called_once_with(query)
        self.mock_vector_service.search_similar.assert_called_once()
        self.mock_cohere_service.generate_response.assert_called_once()
        
        # Verify the result structure
        assert "response" in result
        assert "sources" in result
        assert "query_mode" in result
        assert result["query_mode"] == "book-wide"
        
    def test_answer_from_selected_text(self):
        query = "What does this text mean?"
        selected_text = "This is the selected text for testing."
        
        result = self.rag_service.answer_from_selected_text(query, selected_text)
        
        # Verify that the correct methods were called
        self.mock_cohere_service.generate_response.assert_called_once()
        
        # Verify the result structure
        assert "response" in result
        assert "sources" in result
        assert "query_mode" in result
        assert result["query_mode"] == "selected-text-only"