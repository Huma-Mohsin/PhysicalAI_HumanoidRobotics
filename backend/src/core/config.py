"""
Core Configuration Module

Centralized application settings with validation and environment variable loading.
"""

import os
from typing import Optional
from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    """Application configuration settings."""

    # Application
    APP_NAME: str = "Physical AI & Humanoid Robotics RAG Platform API"
    APP_VERSION: str = "1.0.0"
    DEBUG: bool = False
    API_HOST: str = "0.0.0.0"
    API_PORT: int = 8000

    # Environment
    ENVIRONMENT: str = "development"  # development, staging, production

    # Database (Neon Postgres)
    DATABASE_URL: str
    DB_POOL_SIZE: int = 10
    DB_MAX_OVERFLOW: int = 20

    # Vector Database (Qdrant)
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION_EN: str = "content_embeddings_en"
    QDRANT_COLLECTION_UR: str = "content_embeddings_ur"

    # OpenAI
    OPENAI_API_KEY: str
    OPENAI_MODEL: str = "gpt-4"
    OPENAI_EMBEDDING_MODEL: str = "text-embedding-3-small"
    OPENAI_MAX_TOKENS: int = 1500
    OPENAI_TEMPERATURE: float = 0.7

    # Security
    SECRET_KEY: str
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 10080  # 7 days

    # CORS
    CORS_ORIGINS: list[str] = [
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
    ]

    # Rate Limiting
    RATE_LIMIT_ENABLED: bool = True
    RATE_LIMIT_REQUESTS: int = 100  # requests per minute
    RATE_LIMIT_WINDOW: int = 60  # seconds

    # Logging
    LOG_LEVEL: str = "INFO"  # DEBUG, INFO, WARNING, ERROR, CRITICAL
    LOG_FORMAT: str = "json"  # json or text

    # RAG Configuration
    RAG_TOP_K: int = 5  # Number of chunks to retrieve
    RAG_SCORE_THRESHOLD: float = 0.7  # Minimum similarity score

    # Performance
    CACHE_ENABLED: bool = True
    CACHE_TTL: int = 3600  # Cache time-to-live in seconds

    class Config:
        env_file = ".env"
        case_sensitive = True
        extra = "allow"


@lru_cache()
def get_settings() -> Settings:
    """
    Get cached settings instance.

    Returns:
        Settings: Application configuration
    """
    return Settings()


# Export singleton instance
settings = get_settings()
