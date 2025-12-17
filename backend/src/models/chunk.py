from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime


class BookChunkBase(BaseModel):
    chunk_id: str
    source_page: str
    section_title: str
    content: str
    embedding: Optional[list] = None  # Vector embedding
    metadata: Optional[Dict[str, Any]] = {}


class BookChunkCreate(BookChunkBase):
    pass


class BookChunk(BookChunkBase):
    id: int

    class Config:
        from_attributes = True