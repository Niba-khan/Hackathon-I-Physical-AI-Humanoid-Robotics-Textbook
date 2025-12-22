from fastapi import APIRouter, Depends
from pydantic import BaseModel
from typing import List, Dict, Any
from sqlalchemy.orm import Session
from src.core.database import get_db
from src.services.vector_search_service import VectorSearchService
from src.core.security import sanitize_input
import logging

logger = logging.getLogger(__name__)
router = APIRouter()

# Request/Response models
class SearchRequest(BaseModel):
    query: str
    top_k: int = 5
    threshold: float = 0.7

class SearchResult(BaseModel):
    id: str
    content: str
    document_id: str
    section_ref: str
    score: float
    metadata: Dict[str, Any]

class SearchResponse(BaseModel):
    results: List[SearchResult]

@router.post("/retrieval/search", response_model=SearchResponse)
async def search_endpoint(
    search_request: SearchRequest,
    db: Session = Depends(get_db)
):
    """
    Advanced retrieval endpoint with score thresholding
    """
    # Sanitize the query
    sanitized_query = sanitize_input(search_request.query)
    
    # Initialize vector search service
    vector_service = VectorSearchService()
    
    # Generate embedding for the query
    query_embedding = vector_service.cohere_service.embed_query(sanitized_query)
    
    # Search for similar chunks
    results = vector_service.search_similar(
        query_embedding,
        top_k=search_request.top_k,
        threshold=search_request.threshold
    )
    
    # Convert to response format
    search_results = [
        SearchResult(
            id=result["id"],
            content=result["content"],
            document_id=result["document_id"],
            section_ref=result["section_ref"],
            score=result["score"],
            metadata=result["metadata"]
        )
        for result in results
    ]
    
    logger.info(f"Retrieved {len(search_results)} results for query: {sanitized_query[:50]}...")
    
    return SearchResponse(results=search_results)