"""
FastAPI application entry point for RAG Chatbot backend.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.utils.config import settings
from src.utils.logger import logger
from src.services.database_service import db_service
from src.services.auth_service import init_auth_service
from src.api import chat
from src.api.auth import router as auth_router

# Initialize FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI & Humanoid Robotics book",
    version="1.0.0",
    docs_url="/docs" if settings.app_env == "development" else None,
    redoc_url="/redoc" if settings.app_env == "development" else None,
)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)

# Register routers
app.include_router(chat.router)
app.include_router(auth_router)


# Serverless-compatible: Initialize services lazily, not on startup
# Database and auth services are initialized on first use

@app.on_event("startup")
async def run_migrations():
    """Run database migrations on startup (for Vercel deployment)."""
    try:
        from pathlib import Path
        import asyncpg

        logger.info("Running database migrations...")

        # Read migration file
        migration_path = Path(__file__).parent.parent / "database" / "migrations" / "004_add_background_fields.sql"
        if migration_path.exists():
            migration_sql = migration_path.read_text()

            # Connect and run migration
            conn = await asyncpg.connect(settings.neon_database_url)
            try:
                await conn.execute(migration_sql)
                logger.info("✅ Migration 004 completed successfully!")
            finally:
                await conn.close()
        else:
            logger.warning(f"Migration file not found: {migration_path}")
    except Exception as e:
        # Don't fail startup if migration fails
        logger.error(f"Migration error (non-fatal): {e}")


@app.get("/")
async def root():
    """Root endpoint - health check."""
    return {
        "service": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "running",
        "environment": settings.app_env
    }


@app.get("/health")
async def health_check():
    """Health check endpoint for monitoring."""
    return {
        "status": "healthy",
        "environment": settings.app_env,
        "timestamp": "2025-12-11T00:00:00Z"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.app_env == "development",
        log_level=settings.log_level.lower()
    )
