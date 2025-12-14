"""
Configuration management using Pydantic Settings.
Loads environment variables from .env file and validates them.
"""

from pydantic_settings import BaseSettings
from pydantic import Field
from typing import List


class Settings(BaseSettings):
    """Application configuration loaded from environment variables."""

    # LLM Provider Selection
    llm_provider: str = Field(default="cohere", env="LLM_PROVIDER")  # options: openai, cohere

    # Cohere Configuration
    cohere_api_key: str = Field(default="", env="COHERE_API_KEY")
    cohere_embedding_model: str = Field(default="embed-english-v3.0", env="COHERE_EMBEDDING_MODEL")
    cohere_chat_model: str = Field(default="command-r-08-2024", env="COHERE_CHAT_MODEL")

    # OpenAI Configuration (backup)
    openai_api_key: str = Field(default="", env="OPENAI_API_KEY")
    openai_embedding_model: str = Field(default="text-embedding-3-small", env="OPENAI_EMBEDDING_MODEL")
    openai_chat_model: str = Field(default="gpt-4o-mini", env="OPENAI_CHAT_MODEL")

    # Qdrant Vector Store
    qdrant_url: str = Field(..., env="QDRANT_URL")
    qdrant_api_key: str = Field(..., env="QDRANT_API_KEY")
    qdrant_collection: str = Field(default="humanoid_robotics_book", env="QDRANT_COLLECTION")

    # Neon Serverless Postgres
    neon_database_url: str = Field(..., env="NEON_DATABASE_URL")

    # Better-Auth Integration (Optional for MVP)
    better_auth_api_url: str = Field(default="http://localhost:3000/api/auth", env="BETTER_AUTH_API_URL")
    better_auth_secret: str = Field(default="", env="BETTER_AUTH_SECRET")

    # Application Settings
    app_env: str = Field(default="development", env="APP_ENV")
    log_level: str = Field(default="INFO", env="LOG_LEVEL")
    cors_origins: str = Field(default="http://localhost:3000", env="CORS_ORIGINS")

    # Rate Limiting
    rate_limit_requests: int = Field(default=10, env="RATE_LIMIT_REQUESTS")
    rate_limit_window: int = Field(default=60, env="RATE_LIMIT_WINDOW")

    # RAG Configuration
    rag_top_k: int = Field(default=5, env="RAG_TOP_K")
    rag_similarity_threshold: float = Field(default=0.7, env="RAG_SIMILARITY_THRESHOLD")
    rag_max_context_tokens: int = Field(default=2000, env="RAG_MAX_CONTEXT_TOKENS")
    rag_chunk_size: int = Field(default=600, env="RAG_CHUNK_SIZE")
    rag_chunk_overlap: int = Field(default=100, env="RAG_CHUNK_OVERLAP")

    class Config:
        from pathlib import Path
        env_file = Path(__file__).parent.parent.parent / ".env"  # Points to backend/.env
        env_file_encoding = "utf-8"
        case_sensitive = False

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]


# Global settings instance
settings = Settings()
