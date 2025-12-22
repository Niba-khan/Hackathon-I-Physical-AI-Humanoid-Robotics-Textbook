from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, UUID, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from src.core.database import Base

class EmbeddingMetadata(Base):
    """
    Tracks vector storage information including source document, chunk position, and embedding timestamp
    """
    __tablename__ = "embedding_metadata"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    document_id = Column(String, nullable=False)  # identifier for the original document
    chunk_id = Column(String, nullable=False)  # identifier for the specific chunk
    qdrant_point_id = Column(String, nullable=False)  # identifier in the Qdrant vector store
    embedding_model = Column(String, nullable=False)  # the model used for embedding
    embedding_timestamp = Column(DateTime(timezone=True), server_default=func.now())  # when the embedding was created
    source_url = Column(String, nullable=True)  # URL of the source document

    # Relationship
    document = relationship("Document", back_populates="embedding_metadata")