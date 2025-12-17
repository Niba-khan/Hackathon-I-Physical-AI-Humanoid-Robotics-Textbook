from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List
from pydantic import BaseModel
from models.database import get_db
from services.rag_service import RAGService
from utils import logger
import uuid


class IngestRequest(BaseModel):
    content: str
    source_page: str
    section_title: str


router = APIRouter()

# Initialize services
rag_service = RAGService()


@router.post("/ingest")
async def ingest_endpoint(
    request: IngestRequest,
    db: Session = Depends(get_db)
):
    """
    Process and store book content for retrieval.
    
    Args:
        request: Ingest request with content, source_page, and section_title
        db: Database session dependency
    """
    try:
        # Validate input
        if not request.content or not request.content.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Content cannot be empty"
            )
        
        if not request.source_page or not request.source_page.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Source page cannot be empty"
            )
        
        if not request.section_title or not request.section_title.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Section title cannot be empty"
            )
        
        # Process and store the content
        content_items = [{
            'source_page': request.source_page,
            'content': request.content,
            'section_title': request.section_title
        }]
        
        # Ingest the content using the RAG service
        processed_chunks = rag_service.ingest_content(content_items)
        
        return {
            "message": "Content successfully ingested",
            "chunks_processed": processed_chunks
        }
    
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in ingest endpoint: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while ingesting content"
        )


@router.post("/ingest-batch")
async def ingest_batch_endpoint(
    requests: List[IngestRequest],
    db: Session = Depends(get_db)
):
    """
    Process and store multiple book content items for retrieval.
    
    Args:
        requests: List of ingest requests with content, source_page, and section_title
        db: Database session dependency
    """
    try:
        # Validate input
        if not requests or len(requests) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="At least one content item is required"
            )
        
        # Validate each request
        for request in requests:
            if not request.content or not request.content.strip():
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Content cannot be empty"
                )
            
            if not request.source_page or not request.source_page.strip():
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Source page cannot be empty"
                )
            
            if not request.section_title or not request.section_title.strip():
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Section title cannot be empty"
                )
        
        # Process and store the content items
        content_items = []
        for req in requests:
            content_items.append({
                'source_page': req.source_page,
                'content': req.content,
                'section_title': req.section_title
            })
        
        # Ingest the content using the RAG service
        processed_chunks = rag_service.ingest_content(content_items)
        
        return {
            "message": f"Successfully ingested {len(content_items)} content items",
            "chunks_processed": processed_chunks
        }
    
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in batch ingest endpoint: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while ingesting content"
        )