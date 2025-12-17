from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional
from config import settings
import logging


class VectorService:
    def __init__(self):
        # Try to connect to remote Qdrant first
        try:
            if settings.qdrant_endpoint and "your-cluster-url" not in settings.qdrant_endpoint:
                self.client = QdrantClient(
                    url=settings.qdrant_endpoint,
                    api_key=settings.qdrant_api_key,
                    # timeout=10  # Adjust as needed
                )
                logging.info(f"Connected to Qdrant at {settings.qdrant_endpoint}")
            else:
                # Use in-memory Qdrant for local development
                logging.info("Using in-memory Qdrant for local development")
                self.client = QdrantClient(":memory:")
        except Exception as e:
            logging.error(f"Failed to initialize Qdrant client: {e}")
            # Fall back to in-memory instance
            logging.info("Falling back to in-memory Qdrant")
            self.client = QdrantClient(":memory:")

        # Define collection name
        self.collection_name = "book_chunks"

        # Create collection if it doesn't exist
        self._create_collection()
    
    def _create_collection(self):
        """Create the Qdrant collection if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # If it doesn't exist, create it
            # Using a default embedding size of 384 for sentence-transformers/all-MiniLM-L6-v2
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
            )
    
    def add_chunks(self, chunks: List[dict]):
        """Add book chunks to the vector database"""
        points = []
        for chunk in chunks:
            point = models.PointStruct(
                id=chunk["chunk_id"],
                vector=chunk["embedding"],  # This should be a list of floats
                payload={
                    "source_page": chunk["source_page"],
                    "section_title": chunk["section_title"],
                    "content": chunk["content"],
                    "metadata": chunk.get("metadata", {})
                }
            )
            points.append(point)
        
        self.client.upsert(collection_name=self.collection_name, points=points)
    
    def search_chunks(self, query_vector: List[float], top_k: int = 5) -> List[dict]:
        """Search for relevant chunks based on query vector"""
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=top_k
        )
        
        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append({
                "chunk_id": result.id,
                "source_page": result.payload.get("source_page"),
                "section_title": result.payload.get("section_title"),
                "content": result.payload.get("content"),
                "metadata": result.payload.get("metadata"),
                "score": result.score
            })
        
        return formatted_results
    
    def search_chunks_by_content(self, selected_text: str, top_k: int = 5) -> List[dict]:
        """For selected-text mode, return the selected text as a pseudo-chunk"""
        # This is a placeholder implementation for when we have embeddings for the selected text
        # For now, we'll just return the selected text as a chunk without vector search
        return [{
            "chunk_id": "selected-text",
            "source_page": "selected-text",
            "section_title": "Selected Text",
            "content": selected_text,
            "metadata": {},
            "score": 1.0
        }]