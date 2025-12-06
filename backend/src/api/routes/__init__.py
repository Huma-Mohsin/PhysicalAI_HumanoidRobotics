"""
API Routes Module

Registers all route blueprints for the FastAPI application.
"""

from fastapi import APIRouter
from .health import router as health_router
from .chat import router as chat_router
from .auth import router as auth_router
from .feedback import router as feedback_router

# Create main API router
api_router = APIRouter()

# Register route modules
api_router.include_router(health_router, tags=["health"])
api_router.include_router(chat_router, prefix="/chat", tags=["chat"])
api_router.include_router(auth_router, prefix="/auth", tags=["auth"])
api_router.include_router(feedback_router, prefix="/feedback", tags=["feedback"])
