from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import JSONResponse
from utils import logger
import traceback
from typing import Callable, Awaitable
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response


class ErrorLoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware to log errors and exceptions that occur during request processing.
    """
    
    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        try:
            response = await call_next(request)
            return response
        except HTTPException as e:
            # Log HTTP exceptions
            logger.error(f"HTTPException: {e.status_code} - {e.detail}")
            # Re-raise to be handled by FastAPI's exception handler
            raise
        except Exception as e:
            # Log all other exceptions
            error_id = f"err_{id(e)}"
            logger.error(f"ServerError: {error_id} - {str(e)}")
            logger.error(f"Path: {request.url.path}")
            logger.error(f"Method: {request.method}")
            logger.error(f"Headers: {dict(request.headers)}")
            logger.error(f"Traceback: {traceback.format_exc()}")
            
            # Return a generic error response
            return JSONResponse(
                status_code=500,
                content={
                    "detail": "An internal server error occurred",
                    "error_id": error_id
                }
            )


async def log_errors_middleware(request: Request, call_next):
    """
    Alternative function-based middleware for error logging.
    """
    try:
        response = await call_next(request)
        # Log successful requests if needed
        if response.status_code >= 400:
            logger.warning(f"Request to {request.url.path} returned status {response.status_code}")
        return response
    except Exception as e:
        # Log the exception with details
        logger.error(f"Unhandled exception in request: {str(e)}")
        logger.error(f"URL: {request.url}")
        logger.error(f"Method: {request.method}")
        logger.error(f"Headers: {request.headers}")
        logger.error(f"Traceback: {traceback.format_exc()}")
        
        # Re-raise the exception to be handled by other exception handlers
        raise