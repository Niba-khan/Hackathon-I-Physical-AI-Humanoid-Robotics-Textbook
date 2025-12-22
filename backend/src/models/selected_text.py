from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, UUID, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from src.core.database import Base

class SelectedText(Base):
    """
    Represents user-selected text for the selected-text-only Q&A mode
    """
    __tablename__ = "selected_texts"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PG_UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    text_content = Column(Text, nullable=False)  # the selected text
    char_start_pos = Column(Integer, nullable=True)  # starting character position in the source
    char_end_pos = Column(Integer, nullable=True)  # ending character position in the source
    source_url = Column(String, nullable=True)  # URL where the text was selected from
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    # Relationship
    session = relationship("ChatSession")