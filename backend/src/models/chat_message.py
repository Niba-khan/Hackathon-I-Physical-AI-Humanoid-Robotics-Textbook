from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, UUID, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from src.core.database import Base

class ChatMessage(Base):
    """
    Represents individual messages in a chat session
    """
    __tablename__ = "chat_messages"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PG_UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    role = Column(String, nullable=False)  # either 'user' or 'assistant'
    content = Column(Text, nullable=False)  # the message content
    query_mode = Column(String, nullable=False)  # either 'book-wide' or 'selected-text-only'
    selected_text = Column(Text, nullable=True)  # the selected text if in selected-text-only mode, null otherwise
    retrieved_chunks = Column(JSON, nullable=True)  # references to chunks retrieved during this query
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    # Relationship
    session = relationship("ChatSession", back_populates="messages")