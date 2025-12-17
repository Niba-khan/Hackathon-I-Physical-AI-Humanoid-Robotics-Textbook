from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # Database settings
    neon_database_url: str
    
    # Qdrant settings
    qdrant_api_key: str
    qdrant_endpoint: str
    qdrant_cluster_id: str
    
    # LLM provider settings
    llm_provider_api_key: str
    
    # Optional settings with defaults
    app_name: str = "RAG Chatbot API"
    debug: bool = False
    max_chunk_length: int = 10000  # Maximum length for selected text
    secret_key: Optional[str] = "your-secret-key-change-in-production"
    jwt_algorithm: Optional[str] = "HS256"
    api_key: Optional[str] = "your-api-key"

    class Config:
        env_file = ".env"


settings = Settings()