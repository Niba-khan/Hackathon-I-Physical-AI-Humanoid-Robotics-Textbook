import time
import html
import re
from collections import defaultdict
from fastapi import Request
from src.core.config import settings
from src.core.error_handler import RAGException

class RateLimiter:
    def __init__(self):
        self.requests = defaultdict(list)

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if the identifier is allowed to make a request based on rate limits
        """
        now = time.time()
        # Remove requests older than the time window
        self.requests[identifier] = [
            req_time for req_time in self.requests[identifier]
            if now - req_time < settings.rate_limit_window
        ]

        # Check if the number of requests is within the limit
        if len(self.requests[identifier]) >= settings.rate_limit_requests:
            return False

        # Add the current request
        self.requests[identifier].append(now)
        return True

# Global rate limiter instance
rate_limiter = RateLimiter()

def check_rate_limit(request: Request) -> None:
    """
    Check if the request is within rate limits
    """
    # Use client IP as identifier (in production, consider using user ID or API key)
    client_ip = request.client.host
    if not rate_limiter.is_allowed(client_ip):
        raise RAGException(
            f"Rate limit exceeded. Maximum {settings.rate_limit_requests} requests per {settings.rate_limit_window} seconds.",
            "RATE_LIMIT_EXCEEDED"
        )

def sanitize_input(text: str) -> str:
    """
    Sanitize user input to prevent injection attacks
    """
    if not text:
        return text

    # Remove potentially dangerous characters/sequences
    # Remove any script tags (case insensitive)
    text = re.sub(r'<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>', '', text, flags=re.IGNORECASE)

    # Remove javascript: urls
    text = re.sub(r'javascript:', '', text, flags=re.IGNORECASE)

    # HTML encode potentially dangerous characters
    text = html.escape(text)

    # Remove any control characters except common whitespace
    text = ''.join(char for char in text if ord(char) >= 32 or char in '\n\r\t')

    return text.strip()

def validate_query_length(query: str) -> None:
    """
    Validate that the query length is within acceptable limits
    """
    if len(query) > settings.max_query_length:
        raise RAGException(
            f"Query exceeds maximum length of {settings.max_query_length} characters",
            "QUERY_TOO_LONG"
        )