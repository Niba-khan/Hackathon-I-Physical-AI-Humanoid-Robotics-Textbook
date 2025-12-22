from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, UUID, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from src.core.database import Base

class ChatSession(Base):
    """
    Stores conversation history between user and chatbot with query-response pairs
    """
    __tablename__ = "chat_sessions"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_token = Column(String, nullable=True)  # identifier for the session, tied to user if applicable
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    is_active = Column(Boolean, default=True)  # whether the session is currently active

    # Relationship
    messages = relationship("ChatMessage", back_populates="session")