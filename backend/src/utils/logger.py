"""
Structured logging setup for the RAG chatbot backend.
"""

import logging
import sys
from typing import Optional
from .config import settings


def setup_logger(name: Optional[str] = None) -> logging.Logger:
    """
    Configure and return a logger instance.

    Args:
        name: Logger name (defaults to root logger if None)

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name or __name__)

    # Only configure if not already configured
    if not logger.handlers:
        # Set log level from config
        log_level = getattr(logging, settings.log_level.upper(), logging.INFO)
        logger.setLevel(log_level)

        # Create console handler
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(log_level)

        # Create formatter
        formatter = logging.Formatter(
            fmt="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        console_handler.setFormatter(formatter)

        # Add handler to logger
        logger.addHandler(console_handler)

        # Prevent propagation to root logger
        logger.propagate = False

    return logger


# Global logger instance for the application
logger = setup_logger("rag_chatbot")
