from typing import Dict, Any, List
from sqlalchemy.orm import Session
from models.database import ChatSession as DBChatSession, QueryResult as DBQueryResult
from models.session import ChatSessionCreate, QueryResultCreate
from utils import logger


class ContentService:
    def __init__(self):
        # This service will handle content-related operations
        # including session management and query result storage
        pass
    
    def create_session(self, db: Session, session_data: ChatSessionCreate) -> DBChatSession:
        """
        Create a new chat session.
        
        Args:
            db: Database session
            session_data: Session creation data
            
        Returns:
            Created ChatSession object
        """
        try:
            db_session = DBChatSession(
                session_id=session_data.session_id,
                user_id=session_data.user_id,
                active=session_data.active,
                metadata_=session_data.metadata
            )
            db.add(db_session)
            db.commit()
            db.refresh(db_session)
            return db_session
        except Exception as e:
            logger.error(f"Error creating session: {e}")
            db.rollback()
            raise
    
    def get_session(self, db: Session, session_id: str) -> DBChatSession:
        """
        Get a chat session by ID.
        
        Args:
            db: Database session
            session_id: Session identifier
            
        Returns:
            ChatSession object or None
        """
        try:
            return db.query(DBChatSession).filter(DBChatSession.session_id == session_id).first()
        except Exception as e:
            logger.error(f"Error getting session {session_id}: {e}")
            return None
    
    def create_query_result(self, db: Session, result_data: QueryResultCreate) -> DBQueryResult:
        """
        Create a new query result.
        
        Args:
            db: Database session
            result_data: Query result creation data
            
        Returns:
            Created QueryResult object
        """
        try:
            db_result = DBQueryResult(
                result_id=result_data.result_id,
                session_id=result_data.session_id,
                query_text=result_data.query_text,
                response_text=result_data.response_text,
                retrieved_chunks=result_data.retrieved_chunks,
                query_mode=result_data.query_mode,
                selected_text=result_data.selected_text,
                metadata_=result_data.metadata
            )
            db.add(db_result)
            db.commit()
            db.refresh(db_result)
            return db_result
        except Exception as e:
            logger.error(f"Error creating query result: {e}")
            db.rollback()
            raise
    
    def get_query_results_by_session(self, db: Session, session_id: str) -> List[DBQueryResult]:
        """
        Get all query results for a session.
        
        Args:
            db: Database session
            session_id: Session identifier
            
        Returns:
            List of QueryResult objects
        """
        try:
            return db.query(DBQueryResult).filter(DBQueryResult.session_id == session_id).all()
        except Exception as e:
            logger.error(f"Error getting query results for session {session_id}: {e}")
            return []


# Singleton instance
content_service = ContentService()