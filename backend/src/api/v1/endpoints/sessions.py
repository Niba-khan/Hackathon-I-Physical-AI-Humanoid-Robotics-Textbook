from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional
from pydantic import BaseModel, Field
from models.session import ChatSessionCreate, ChatSessionUpdate
from models.database import get_db
from services.content_service import content_service
from utils import logger
import uuid


class CreateSessionRequest(BaseModel):
    user_id: Optional[str] = Field(None, description="Identifier for the user (optional)")


class UpdateSessionRequest(BaseModel):
    active: Optional[bool] = Field(None, description="Whether the session is currently active")


router = APIRouter()


@router.post("/sessions")
async def create_session(
    request: CreateSessionRequest,
    db: Session = Depends(get_db)
):
    """
    Create a new chat session.
    """
    try:
        # Generate a new session ID
        session_id = str(uuid.uuid4())

        # Create session data
        session_data = ChatSessionCreate(
            session_id=session_id,
            user_id=request.user_id,
            active=True,
            metadata={"created_via": "api"}
        )

        # Create the session in the database
        session = content_service.create_session(db, session_data)

        return {
            "session_id": session.session_id,
            "user_id": session.user_id,
            "active": session.active,
            "created_at": session.created_at.isoformat() if session.created_at else None,
            "updated_at": session.updated_at.isoformat() if session.updated_at else None
        }

    except Exception as e:
        logger.error(f"Error in create session endpoint: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while creating the session"
        )


@router.get("/sessions/{session_id}")
async def get_session(
    session_id: str,
    db: Session = Depends(get_db)
):
    """
    Retrieve a specific chat session.
    """
    try:
        # Get the session from the database
        session = content_service.get_session(db, session_id)

        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Session not found"
            )

        # Get query results for the session
        query_results = content_service.get_query_results_by_session(db, session_id)

        # Format the response
        formatted_results = []
        for result in query_results:
            formatted_results.append({
                "result_id": result.result_id,
                "query_text": result.query_text,
                "response_text": result.response_text,
                "query_mode": result.query_mode,
                "created_at": result.created_at.isoformat() if result.created_at else None
            })

        return {
            "session_id": session.session_id,
            "user_id": session.user_id,
            "active": session.active,
            "created_at": session.created_at.isoformat() if session.created_at else None,
            "updated_at": session.updated_at.isoformat() if session.updated_at else None,
            "metadata": session.metadata,
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


@router.patch("/sessions/{session_id}")
async def update_session(
    session_id: str,
    request: UpdateSessionRequest,
    db: Session = Depends(get_db)
):
    """
    Update an existing chat session.
    """
    try:
        # Check if the session exists
        existing_session = content_service.get_session(db, session_id)
        if not existing_session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Session not found"
            )

        # Prepare update data
        session_update_data = ChatSessionUpdate(
            session_id=session_id,
            active=request.active
        )

        # Update the session in the database
        updated_session = content_service.update_session(db, session_update_data)

        return {
            "session_id": updated_session.session_id,
            "user_id": updated_session.user_id,
            "active": updated_session.active,
            "created_at": updated_session.created_at.isoformat() if updated_session.created_at else None,
            "updated_at": updated_session.updated_at.isoformat() if updated_session.updated_at else None
        }

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in update session endpoint: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while updating the session"
        )


@router.delete("/sessions/{session_id}")
async def delete_session(
    session_id: str,
    db: Session = Depends(get_db)
):
    """
    Deactivate a chat session (soft delete).
    """
    try:
        # Check if the session exists
        existing_session = content_service.get_session(db, session_id)
        if not existing_session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Session not found"
            )

        # Update the session to mark as inactive
        session_update_data = ChatSessionUpdate(
            session_id=session_id,
            active=False
        )

        updated_session = content_service.update_session(db, session_update_data)

        return {
            "session_id": updated_session.session_id,
            "active": updated_session.active,
            "message": "Session deactivated successfully"
        }

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in delete session endpoint: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while deactivating the session"
        )