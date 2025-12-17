from fastapi import FastAPI 
from api.v1.endpoints import query, ingest, sessions
from config import settings
from api.middleware.rate_limit import rate_limit_middleware
from api.middleware.error_handler import ErrorLoggingMiddleware
import uvicorn


# Create the FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the embedded RAG chatbot that answers questions based on book content",
    version="1.0.0"
)

# Add middleware
app.middleware("http")(rate_limit_middleware)
app.add_middleware(ErrorLoggingMiddleware)


# Include API routes
app.include_router(query.router, prefix="/api/v1", tags=["query"])
app.include_router(ingest.router, prefix="/api/v1", tags=["ingest"])
app.include_router(sessions.router, prefix="/api/v1", tags=["sessions"])


@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}


@app.get("/health")
def health_check():
    return {"status": "healthy", "message": "API is operational"}


# For development, you can run this file directly
if __name__ == "__main__":
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )