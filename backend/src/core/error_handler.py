import logging
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from typing import Dict, Any
import traceback

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class RAGException(Exception):
    """Base exception for RAG-related errors"""
    def __init__(self, message: str, error_code: str = "RAG_ERROR"):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

class NoContextException(RAGException):
    """Raised when no relevant context is found to answer a question"""
    def __init__(self, message: str = "No relevant context found to answer the question"):
        super().__init__(message, "NO_CONTEXT_AVAILABLE")

class ServiceUnavailableException(RAGException):
    """Raised when an external service is unavailable"""
    def __init__(self, service_name: str):
        message = f"{service_name} is currently unavailable"
        super().__init__(message, "SERVICE_UNAVAILABLE")

def setup_error_handlers(app: FastAPI):
    """Set up global error handlers for the application"""
    
    @app.exception_handler(StarletteHTTPException)
    async def http_exception_handler(request: Request, exc: StarletteHTTPException):
        logger.error(f"HTTP error: {exc.status_code} - {exc.detail}")
        return JSONResponse(
            status_code=exc.status_code,
            content={
                "error": exc.detail,
                "error_code": "HTTP_ERROR",
                "timestamp": request.state.start_time if hasattr(request.state, 'start_time') else None
            }
        )
    
    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        logger.error(f"Validation error: {exc}")
        return JSONResponse(
            status_code=422,
            content={
                "error": "Invalid input parameters",
                "details": exc.errors(),
                "error_code": "INVALID_INPUT",
                "timestamp": request.state.start_time if hasattr(request.state, 'start_time') else None
            }
        )
    
    @app.exception_handler(RAGException)
    async def rag_exception_handler(request: Request, exc: RAGException):
        logger.error(f"RAG error: {exc.error_code} - {exc.message}")
        status_code = 400 if exc.error_code == "NO_CONTEXT_AVAILABLE" else 500
        return JSONResponse(
            status_code=status_code,
            content={
                "error": exc.message,
                "error_code": exc.error_code,
                "timestamp": request.state.start_time if hasattr(request.state, 'start_time') else None
            }
        )
    
    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        logger.error(f"Unexpected error: {exc}\n{traceback.format_exc()}")
        return JSONResponse(
            status_code=500,
            content={
                "error": "An unexpected error occurred",
                "error_code": "INTERNAL_ERROR",
                "timestamp": request.state.start_time if hasattr(request.state, 'start_time') else None
            }
        )