from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text, Boolean, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
from config import settings


# Database setup
SQLALCHEMY_DATABASE_URL = settings.neon_database_url

engine = create_engine(SQLALCHEMY_DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()


# Database Models
class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, unique=True, index=True)
    user_id = Column(String, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    active = Column(Boolean, default=True)
    metadata_ = Column("metadata", JSON, default={})


class QueryResult(Base):
    __tablename__ = "query_results"

    id = Column(Integer, primary_key=True, index=True)
    result_id = Column(String, unique=True, index=True)
    session_id = Column(String, index=True)  # Foreign key reference to chat_sessions
    query_text = Column(Text)
    response_text = Column(Text)
    retrieved_chunks = Column(JSON)  # Store as JSON array
    query_mode = Column(String)  # "full-book" or "selected-text"
    selected_text = Column(Text, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    metadata_ = Column("metadata", JSON, default={})


# Create tables
def create_tables():
    Base.metadata.create_all(bind=engine)


# Dependency to get DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()