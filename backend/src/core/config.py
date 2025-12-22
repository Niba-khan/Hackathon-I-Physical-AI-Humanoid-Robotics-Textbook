from pydantic_settings import BaseSettings
from typing import Optional
import os

class Settings(BaseSettings):
    # API Keys and External Services
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")
    
    # Application Settings
    app_name: str = "RAG Chatbot API"
    app_version: str = "1.0.0"
    secret_key: str = os.getenv("SECRET_KEY", "dev-secret-key-change-in-production")
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30
    
    # RAG Configuration
    top_k: int = 5  # Number of top results to retrieve
    score_threshold: float = 0.7  # Minimum similarity score for retrieval
    max_context_length: int = 2000  # Maximum length of context to send to LLM
    max_query_length: int = 500  # Maximum length of user query
    
    # Rate Limiting
    rate_limit_requests: int = 100  # Number of requests allowed
    rate_limit_window: int = 60  # Time window in seconds
    
    # Model Configuration
    llm_model: str = "command-r-plus"
    embedding_model: str = "embed-english-v3.0"
    
    class Config:
        env_file = ".env"

settings = Settings()