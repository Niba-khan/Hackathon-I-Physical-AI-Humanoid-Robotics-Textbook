from fastapi import FastAPI, Request, HTTPException, status
from fastapi.responses import JSONResponse
from collections import defaultdict
import time
import hashlib
from typing import Dict
from utils import logger


class RateLimiter:
    def __init__(self, max_requests: int = 10, window_size: int = 60):
        """
        Initialize the rate limiter.
        
        Args:
            max_requests: Maximum number of requests allowed in the window
            window_size: Time window in seconds
        """
        self.max_requests = max_requests
        self.window_size = window_size
        self.requests: Dict[str, list] = defaultdict(list)

    def _get_client_identifier(self, request: Request) -> str:
        """
        Get a unique identifier for the client.
        Prioritizes X-Forwarded-For header for clients behind proxies,
        falls back to request.client.host.
        """
        forwarded = request.headers.get("X-Forwarded-For")
        if forwarded:
            # In production, you might want to be more careful about IP spoofing
            client_ip = forwarded.split(",")[0].strip()
        else:
            client_ip = request.client.host
        
        # Create a hash to avoid storing actual IPs
        return hashlib.sha256(client_ip.encode()).hexdigest()

    def is_allowed(self, request: Request) -> bool:
        """
        Check if the request is allowed based on rate limits.
        
        Args:
            request: The incoming request
            
        Returns:
            True if request is allowed, False otherwise
        """
        client_id = self._get_client_identifier(request)
        current_time = time.time()
        
        # Remove old requests outside the time window
        self.requests[client_id] = [
            req_time for req_time in self.requests[client_id]
            if current_time - req_time < self.window_size
        ]
        
        # Check if client has exceeded the limit
        if len(self.requests[client_id]) >= self.max_requests:
            return False
        
        # Add current request to the list
        self.requests[client_id].append(current_time)
        return True


# Global rate limiter instance
rate_limiter = RateLimiter(max_requests=30, window_size=60)  # 30 requests per minute


async def rate_limit_middleware(request: Request, call_next):
    """
    Middleware to enforce rate limiting.
    
    Args:
        request: The incoming request
        call_next: The next function in the middleware chain
        
    Returns:
        The response from the next function, or a 429 response if rate limited
    """
    # Skip rate limiting for certain paths (like health checks)
    if request.url.path in ["/health", "/"]:
        return await call_next(request)
    
    # Check if the request is allowed
    if not rate_limiter.is_allowed(request):
        logger.warning(f"Rate limit exceeded for client: {request.client.host}")
        return JSONResponse(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            content={"detail": "Rate limit exceeded. Please try again later."}
        )
    
    response = await call_next(request)
    return response