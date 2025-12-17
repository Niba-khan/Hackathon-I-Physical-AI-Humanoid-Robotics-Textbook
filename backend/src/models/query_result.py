from pydantic import BaseModel
from typing import Optional, Dict, Any, List
from datetime import datetime


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