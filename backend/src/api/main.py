"""
FastAPI Main Application

Entry point for the Physical AI RAG Platform backend API.
"""

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager
from dotenv import load_dotenv

# Load environment variables first
load_dotenv()

# Import configuration and logging
from src.core.config import settings
from src.core.logging import setup_logging, get_logger

# Setup logging before anything else
setup_logging()
logger = get_logger(__name__)

# Import middleware and routes
from src.api.middleware.cors import add_cors_middleware
from src.api.middleware.rate_limit import add_rate_limit_middleware
from src.api.middleware.request_validation import add_request_validation_middleware
from src.api.routes import api_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan context manager.

    Handles startup and shutdown events.
    """
    # Startup
    logger.info(f"🚀 Starting {settings.APP_NAME} v{settings.APP_VERSION}")
    logger.info(f"Environment: {settings.ENVIRONMENT}")

    # Initialize database (create tables if they don't exist)
    try:
        from src.db.neon import init_db, check_db_connection
        if check_db_connection():
            # init_db()  # Uncomment when ready to create tables
            logger.info("✅ Database connection verified")
        else:
            logger.warning("⚠️  Database connection failed - check DATABASE_URL")
    except Exception as e:
        logger.error(f"Database initialization error: {e}")

    # Check Qdrant connection
    try:
        from src.db.qdrant_client import get_qdrant_service
        qdrant = get_qdrant_service()
        if qdrant.check_connection():
            logger.info("✅ Qdrant vector DB connection verified")
        else:
            logger.warning("⚠️  Qdrant connection failed - check QDRANT_URL and QDRANT_API_KEY")
    except Exception as e:
        logger.error(f"Qdrant initialization error: {e}")

    logger.info("✅ API server ready")

    yield

    # Shutdown
    logger.info("👋 Shutting down API server...")


# Create FastAPI application
app = FastAPI(
    title=settings.APP_NAME,
    description="Backend API for the Physical AI educational platform with RAG chatbot",
    version=settings.APP_VERSION,
    lifespan=lifespan,
    debug=settings.DEBUG,
)

# Add middleware (order matters - CORS should be last)
add_request_validation_middleware(app)
add_rate_limit_middleware(app)
add_cors_middleware(app)

# Register API routes
app.include_router(api_router)


@app.get("/")
async def root():
    """Root endpoint - API information."""
    return {
        "message": settings.APP_NAME,
        "version": settings.APP_VERSION,
        "environment": settings.ENVIRONMENT,
        "docs": "/docs",
        "health": "/health",
    }


@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """Global exception handler for unhandled errors."""
    logger.error(
        f"Unhandled exception: {exc}",
        extra={"path": str(request.url), "method": request.method},
        exc_info=True
    )
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error"},
    )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "src.api.main:app",
        host=settings.API_HOST,
        port=settings.API_PORT,
        reload=settings.DEBUG,
        log_level=settings.LOG_LEVEL.lower(),
    )
