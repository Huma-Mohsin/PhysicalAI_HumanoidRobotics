"""
FastAPI application entry point for RAG Chatbot backend.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from utils.config import settings
from utils.logger import logger
from services.database_service import db_service
from api import chat

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


@app.on_event("startup")
async def startup_event():
    """Execute on application startup."""
    logger.info(f"Starting RAG Chatbot API in {settings.app_env} mode")
    logger.info(f"CORS origins: {settings.cors_origins_list}")
    logger.info(f"OpenAI model: {settings.openai_chat_model}")
    logger.info(f"Qdrant collection: {settings.qdrant_collection}")

    # Initialize database connection pool
    await db_service.connect()
    logger.info("Database service initialized")


@app.on_event("shutdown")
async def shutdown_event():
    """Execute on application shutdown."""
    # Close database connection pool
    await db_service.disconnect()
    logger.info("Shutting down RAG Chatbot API")


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
