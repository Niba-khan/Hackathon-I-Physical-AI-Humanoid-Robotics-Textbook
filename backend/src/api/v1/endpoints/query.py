from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional
from pydantic import BaseModel, Field, validator
from models.session import QueryResultCreate
from models.database import get_db
from services.rag_service import RAGService
from services.content_service import content_service
from utils import logger
import uuid
import re


class QueryRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=1000, description="The question to be answered")
    session_id: Optional[str] = Field(None, description="The session identifier (optional, creates new if not provided)")
    query_mode: str = Field("full-book", pattern=r"^(full-book|selected-text)$", description="Either 'full-book' or 'selected-text'")
    selected_text: Optional[str] = Field(None, max_length=10000, description="Text selected by user when query_mode is 'selected-text'")

    @validator('question')
    def validate_question(cls, v):
        # Remove any potentially harmful characters or patterns
        if re.search(r'[<>\\"\'\(\)]', v):
            raise ValueError('Question contains invalid characters')
        return v.strip()

    @validator('session_id')
    def validate_session_id(cls, v):
        if v is not None:
            # Validate session ID format (alphanumeric and some special chars)
            if not re.match(r'^[a-zA-Z0-9_-]+$', v):
                raise ValueError('Invalid session ID format')
        return v

    @validator('selected_text')
    def validate_selected_text(cls, v):
        if v is not None and len(v.strip()) == 0:
            raise ValueError('Selected text cannot be empty')
        # Remove potentially harmful patterns
        if v and re.search(r'[<>\\"\'\(\)]', v):
            raise ValueError('Selected text contains invalid characters')
        return v.strip() if v else v


router = APIRouter()

# Initialize services
rag_service = RAGService()


@router.post("/query")
async def query_endpoint(
    request: QueryRequest,
    db: Session = Depends(get_db)
):
    """
    Query the RAG system with a question.

    Args:
        request: Query request with question, session_id, query_mode, selected_text
        db: Database session dependency
    """
    try:
        # Generate a new session ID if not provided
        session_id = request.session_id or str(uuid.uuid4())

        # Validate selected_text when in selected-text mode
        if request.query_mode == "selected-text" and (not request.selected_text or not request.selected_text.strip()):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="selected_text is required when query_mode is 'selected-text'"
            )

        # Sanitize inputs to prevent injection attacks
        question = request.question.strip()
        selected_text = request.selected_text.strip() if request.selected_text else None

        # Process the query based on mode
        if request.query_mode == "full-book":
            result = rag_service.query_full_book(question)
        else:  # selected-text mode
            result = rag_service.query_selected_text(question, selected_text)

        # Additional validation: check if result contains content from retrieval
        if not result.get("retrieved_chunks") and "not available in the book" not in result.get("response", "").lower():
            logger.warning(f"Query returned no retrieved chunks: {request.question}")

        # Create a new query result record
        result_data = QueryResultCreate(
            result_id=str(uuid.uuid4()),
            session_id=session_id,
            query_text=request.question,
            response_text=result["response"],
            retrieved_chunks=result["retrieved_chunks"],
            query_mode=request.query_mode,
            selected_text=request.selected_text if request.query_mode == "selected-text" else None,
            metadata={"mode": request.query_mode}
        )

        content_service.create_query_result(db, result_data)

        # Return the response
        result["session_id"] = session_id
        return result

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except ValueError as ve:
        logger.error(f"Value error in query endpoint: {ve}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid request: {str(ve)}"
        )
    except Exception as e:
        logger.error(f"Error in query endpoint: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while processing the query"
        )


@router.get("/sessions/{session_id}")
async def get_session(session_id: str, db: Session = Depends(get_db)):
    """
    Retrieve a specific chat session.
    """
    try:
        # Get query results for the session
        query_results = content_service.get_query_results_by_session(db, session_id)
        
        if not query_results:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Session not found"
            )
        
        # Format the response
        formatted_results = []
        for result in query_results:
            formatted_results.append({
                "query_text": result.query_text,
                "response_text": result.response_text,
                "created_at": result.created_at.isoformat() if result.created_at else None
            })
        
        return {
            "session_id": session_id,
            "created_at": query_results[0].created_at.isoformat() if query_results and query_results[0].created_at else None,
            "query_results": formatted_results
        }
    
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in get session endpoint: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while retrieving the session"
        )