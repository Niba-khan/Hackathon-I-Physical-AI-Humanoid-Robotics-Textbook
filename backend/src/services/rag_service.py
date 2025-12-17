from typing import List, Dict, Any, Optional
from services.vector_service import VectorService
from services.embedding_service import EmbeddingService
from services.llm_service import llm_service
from utils import logger
from config import settings


class RAGService:
    def __init__(self):
        self.vector_service = VectorService()
        self.embedding_service = EmbeddingService()
        self.llm_service = llm_service
    
    def query_full_book(self, question: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Query the full book using semantic search.
        
        Args:
            question: The question to answer
            top_k: Number of top results to retrieve
            
        Returns:
            Dictionary with answer and metadata
        """
        try:
            # Generate embedding for the question
            question_embedding = self.embedding_service.generate_embedding(question)
            
            # Search for relevant chunks
            relevant_chunks = self.vector_service.search_chunks(
                query_vector=question_embedding,
                top_k=top_k
            )
            
            if not relevant_chunks:
                return {
                    "response": "This information is not available in the book.",
                    "retrieved_chunks": [],
                    "query_mode": "full-book"
                }
            
            # Construct context from retrieved chunks
            context = "\n\n".join([chunk["content"] for chunk in relevant_chunks])
            
            # Generate answer using LLM
            answer = self.llm_service.generate_response(question, context)

            return {
                "response": answer,
                "retrieved_chunks": relevant_chunks,
                "query_mode": "full-book"
            }
        except Exception as e:
            logger.error(f"Error in full book query: {e}")
            return {
                "response": "Error processing your query. Please try again.",
                "retrieved_chunks": [],
                "query_mode": "full-book"
            }
    
    def query_selected_text(self, question: str, selected_text: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Query specifically about selected text only.
        
        Args:
            question: The question to answer
            selected_text: The text selected by the user
            top_k: Number of top results to retrieve (not used for selected text mode)
            
        Returns:
            Dictionary with answer and metadata
        """
        try:
            if not selected_text or not selected_text.strip():
                return {
                    "response": "This information is not available in the book.",
                    "retrieved_chunks": [],
                    "query_mode": "selected-text"
                }
            
            # For selected text mode, we use the selected text directly as context
            relevant_chunks = self.vector_service.search_chunks_by_content(selected_text, top_k)
            
            # Construct context from selected text
            context = selected_text

            # Generate answer using LLM
            answer = self.llm_service.generate_response(question, context)

            return {
                "response": answer,
                "retrieved_chunks": relevant_chunks,  # Using the chunks found for the selected text
                "query_mode": "selected-text"
            }
        except Exception as e:
            logger.error(f"Error in selected text query: {e}")
            return {
                "response": "Error processing your query. Please try again.",
                "retrieved_chunks": [],
                "query_mode": "selected-text"
            }
    
    
    def ingest_content(self, content_items: List[Dict[str, Any]]) -> int:
        """
        Process and store book content for retrieval.
        
        Args:
            content_items: List of content items with source_page, content, section_title
            
        Returns:
            Number of processed chunks
        """
        total_chunks = 0
        all_chunks_to_store = []
        
        for item in content_items:
            source_page = item.get('source_page')
            content = item.get('content', '')
            section_title = item.get('section_title', 'Unknown')
            
            # Generate embedding for the content
            if content.strip():
                try:
                    embedding = self.embedding_service.generate_embedding(content)
                    
                    chunk_to_store = {
                        "chunk_id": f"{source_page.replace('/', '_')}_1",
                        "source_page": source_page,
                        "section_title": section_title,
                        "content": content,
                        "embedding": embedding
                    }
                    
                    all_chunks_to_store.append(chunk_to_store)
                    total_chunks += 1
                    
                except Exception as e:
                    logger.error(f"Error generating embedding for {source_page}: {e}")
        
        if all_chunks_to_store:
            # Store all chunks in vector database
            self.vector_service.add_chunks(all_chunks_to_store)
        
        logger.info(f"Successfully ingested content: {total_chunks} chunks")
        return total_chunks