from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from dotenv import load_dotenv
import os

# Import Cohere service to handle embeddings
from .cohere_service import CohereService

load_dotenv()

class VectorSearchConfig:
    url = os.getenv("QDRANT_URL")
    api_key = os.getenv("QDRANT_API_KEY")
    collection_name = "book_chunks"

class DocumentChunk(BaseModel):
    id: str
    content: str
    document_id: str
    section_ref: str
    chunk_index: int
    metadata: Dict[str, Any]

class VectorSearchService:
    def __init__(self):
        if not VectorSearchConfig.url or not VectorSearchConfig.api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

        self.client = QdrantClient(
            url=VectorSearchConfig.url,
            api_key=VectorSearchConfig.api_key,
        )
        self.collection_name = VectorSearchConfig.collection_name
        self.cohere_service = CohereService()  # Add Cohere service for embeddings
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """
        Create the collection if it doesn't exist
        """
        try:
            self.client.get_collection(self.collection_name)
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
            )

    def store_chunk(self, chunk: DocumentChunk, embedding: List[float]) -> bool:
        """
        Store a document chunk with its embedding in Qdrant
        """
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=chunk.id,
                        vector=embedding,
                        payload={
                            "content": chunk.content,
                            "document_id": chunk.document_id,
                            "section_ref": chunk.section_ref,
                            "chunk_index": chunk.chunk_index,
                            "metadata": chunk.metadata
                        }
                    )
                ]
            )
            return True
        except Exception as e:
            print(f"Error storing chunk: {e}")
            return False

    def search_similar(self, query_embedding: List[float], top_k: int = 5, threshold: float = 0.7) -> List[Dict[str, Any]]:
        """
        Search for similar chunks based on embedding similarity
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=threshold
            )

            return [
                {
                    "id": result.id,
                    "content": result.payload["content"],
                    "document_id": result.payload["document_id"],
                    "section_ref": result.payload["section_ref"],
                    "score": result.score,
                    "metadata": result.payload["metadata"]
                }
                for result in results
                if result.score >= threshold
            ]
        except Exception as e:
            print(f"Error searching for similar chunks: {e}")
            return []

    def delete_document(self, document_id: str) -> bool:
        """
        Delete all chunks associated with a document
        """
        try:
            # Find all points with this document_id
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="payload.document_id",
                            match=models.MatchValue(value=document_id)
                        )
                    ]
                ),
                limit=10000  # Adjust as needed
            )

            # Extract point IDs
            point_ids = [point.id for point in results[0]]

            if point_ids:
                # Delete the points
                self.client.delete(
                    collection_name=self.collection_name,
                    points_selector=models.PointIdsList(
                        points=point_ids
                    )
                )

            return True
        except Exception as e:
            print(f"Error deleting document: {e}")
            return False