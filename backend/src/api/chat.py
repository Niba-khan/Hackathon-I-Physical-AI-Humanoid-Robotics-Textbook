from fastapi import APIRouter, Depends, Request
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
from sqlalchemy.orm import Session
from src.core.database import get_db
from src.services.rag_service import RAGService
from src.core.security import check_rate_limit, sanitize_input, validate_query_length
from src.core.config import settings
import uuid
import logging

logger = logging.getLogger(__name__)
router = APIRouter()

# Request/Response models
class ChatRequest(BaseModel):
    query: str
    mode: str  # "book-wide" or "selected-text-only"
    selected_text: Optional[str] = None
    session_id: Optional[str] = None

class Source(BaseModel):
    document_id: str
    section_ref: str
    content: str
    score: float

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    session_id: str
    query_mode: str

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: Request,
    chat_request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    Main chat endpoint that handles both book-wide and selected-text-only modes
    """
    # Rate limiting
    check_rate_limit(request)
    
    # Sanitize inputs
    sanitized_query = sanitize_input(chat_request.query)
    sanitized_selected_text = sanitize_input(chat_request.selected_text) if chat_request.selected_text else None
    
    # Validate query length
    validate_query_length(sanitized_query)
    
    # Validate mode
    if chat_request.mode not in ["book-wide", "selected-text-only"]:
        from src.core.error_handler import RAGException
        raise RAGException("Invalid mode. Use 'book-wide' or 'selected-text-only'", "INVALID_INPUT")
    
    # Validate selected text for selected-text-only mode
    if chat_request.mode == "selected-text-only" and not sanitized_selected_text:
        from src.core.error_handler import RAGException
        raise RAGException("Selected text is required for selected-text-only mode", "MISSING_SELECTED_TEXT")
    
    # Generate or use session ID
    session_id = chat_request.session_id or str(uuid.uuid4())
    
    # Initialize RAG service
    rag_service = RAGService()
    
    try:
        if chat_request.mode == "book-wide":
            result = rag_service.answer_from_corpus(sanitized_query)
        else:  # selected-text-only mode
            result = rag_service.answer_from_selected_text(sanitized_query, sanitized_selected_text)
        
        # Add session ID to the result
        result["session_id"] = session_id
        
        # Log the successful query
        logger.info(f"Chat query processed for session {session_id}, mode: {chat_request.mode}")
        
        return ChatResponse(**result)
    
    except Exception as e:
        logger.error(f"Error processing chat request: {e}")
        raise


@router.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {
        "status": "healthy",
        "timestamp": "2025-12-19T10:00:00Z",  # This would be dynamic in a real implementation
        "version": settings.app_version
    }