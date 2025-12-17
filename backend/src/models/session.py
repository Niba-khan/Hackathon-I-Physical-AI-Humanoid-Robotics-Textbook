from pydantic import BaseModel
from typing import Optional, Dict, Any, List
from datetime import datetime
from .chunk import BookChunk


class ChatSessionBase(BaseModel):
    session_id: str
    user_id: Optional[str] = None
    active: bool = True
    metadata: Optional[Dict[str, Any]] = {}


class ChatSessionCreate(ChatSessionBase):
    pass


class ChatSession(ChatSessionBase):
    id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class ChatSessionUpdate(BaseModel):
    session_id: str
    active: Optional[bool] = None
    user_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None


class QueryResultBase(BaseModel):
    result_id: str
    session_id: str
    query_text: str
    response_text: str
    retrieved_chunks: List[Dict[str, Any]]  # List of chunk info
    query_mode: str  # "full-book" or "selected-text"
    selected_text: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = {}


class QueryResultCreate(QueryResultBase):
    pass


class QueryResult(QueryResultBase):
    id: int
    created_at: datetime

    class Config:
        from_attributes = True