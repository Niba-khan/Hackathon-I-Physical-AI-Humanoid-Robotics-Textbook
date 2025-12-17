import logging
from datetime import datetime
import os
from logging.handlers import RotatingFileHandler


def setup_logging():
    # Create logs directory if it doesn't exist
    if not os.path.exists("logs"):
        os.makedirs("logs")
    
    # Configure root logger
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            RotatingFileHandler("logs/app.log", maxBytes=10000000, backupCount=5),
            logging.StreamHandler()  # Also log to console
        ]
    )
    
    return logging.getLogger(__name__)


# Initialize the logger
logger = setup_logging()


class AppException(Exception):
    """Base application exception"""
    def __init__(self, message: str, error_code: str = "APP_ERROR"):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)
        logger.error(f"{error_code}: {message}")


class RAGException(AppException):
    """Exception for RAG-related errors"""
    def __init__(self, message: str):
        super().__init__(message, "RAG_ERROR")


class ConfigurationError(AppException):
    """Exception for configuration-related errors"""
    def __init__(self, message: str):
        super().__init__(message, "CONFIG_ERROR")


class ValidationError(AppException):
    """Exception for validation-related errors"""
    def __init__(self, message: str):
        super().__init__(message, "VALIDATION_ERROR")