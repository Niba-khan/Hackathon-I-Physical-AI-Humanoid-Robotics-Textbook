from fastapi import FastAPI
from src.api import chat
from fastapi.middleware.cors import CORSMiddleware
from src.api import retrieval
from src.core.error_handler import setup_error_handlers
from src.core.config import settings
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)

# Create FastAPI app instance
app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    description="RAG Chatbot API for AI-Native Textbook"
)

# Set up error handlers
setup_error_handlers(app)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000/Hackathon-I-Physical-AI-Humanoid-Robotics-Textbook/"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Include API routes
app.include_router(chat.router, prefix="/api", tags=["chat"])
app.include_router(retrieval.router, prefix="/api", tags=["retrieval"])

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API for AI-Native Textbook", "version": settings.app_version}