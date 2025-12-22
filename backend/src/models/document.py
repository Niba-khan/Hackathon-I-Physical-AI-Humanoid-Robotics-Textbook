from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, UUID, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from src.core.database import Base

class Document(Base):
    """
    Represents book content chunks for RAG retrieval, with content, embedding vector, and metadata
    """
    __tablename__ = "documents"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    content = Column(Text, nullable=False)  # the actual text content of the chunk
    document_id = Column(String, nullable=False)  # identifier for the original document/chapter
    section_ref = Column(String, nullable=True)  # reference to the section, page, or heading
    chunk_index = Column(Integer, nullable=True)  # position of this chunk in the document sequence
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())