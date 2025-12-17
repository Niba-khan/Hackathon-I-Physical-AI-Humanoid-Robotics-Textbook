from fastapi import Request, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from config import settings
import jwt
from utils import logger


class JWTAuth:
    """
    JWT-based authentication for API endpoints.
    """
    def __init__(self):
        self.secret_key = settings.secret_key or "your-default-secret-key-change-in-production"
        self.algorithm = settings.jwt_algorithm or "HS256"
        self.security = HTTPBearer(auto_error=False)

    async def authenticate(self, request: Request):
        """
        Authenticate the request using JWT token.
        
        Args:
            request: The incoming request
            
        Returns:
            User data if authenticated, None otherwise
        """
        # Extract the token from the Authorization header
        credentials: HTTPAuthorizationCredentials = await self.security(request)
        
        if not credentials or not credentials.credentials:
            return None
        
        try:
            # Decode the JWT token
            payload = jwt.decode(
                credentials.credentials,
                self.secret_key,
                algorithms=[self.algorithm]
            )
            
            # Return user information from the token
            return {
                "user_id": payload.get("user_id"),
                "username": payload.get("username"),
                "role": payload.get("role")
            }
        except jwt.ExpiredSignatureError:
            logger.warning("Expired token received")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Token has expired"
            )
        except jwt.InvalidTokenError:
            logger.warning("Invalid token received")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )


# API Key-based authentication as an alternative
class APIKeyAuth:
    """
    API Key-based authentication for API endpoints.
    """
    def __init__(self):
        self.api_key = settings.api_key

    async def authenticate(self, request: Request):
        """
        Authenticate the request using API key.
        
        Args:
            request: The incoming request
            
        Returns:
            True if authenticated, False otherwise
        """
        api_key = request.headers.get("X-API-Key") or request.query_params.get("api_key")
        
        if not api_key or api_key != self.api_key:
            return False
        
        return True


# Initialize authenticators
jwt_auth = JWTAuth()
api_key_auth = APIKeyAuth()


# Middleware for request validation
async def security_middleware(request: Request, call_next):
    """
    Middleware to add security validations to requests.
    
    Args:
        request: The incoming request
        call_next: The next function in the middleware chain
        
    Returns:
        The response from the next function, or an error response if validation fails
    """
    # Add security headers to the response
    response = await call_next(request)
    
    # Add security headers
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["X-XSS-Protection"] = "1; mode=block"
    response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
    
    return response