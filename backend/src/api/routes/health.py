"""
Health Check Endpoint

Provides system health status for monitoring and deployment validation.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Dict, Any

router = APIRouter()


class HealthResponse(BaseModel):
    """Health check response model."""
    status: str
    database: str
    vector_db: str
    message: str


@router.get("/health", response_model=HealthResponse)
async def health_check() -> Dict[str, Any]:
    """
    Health check endpoint.

    Returns system status including database and vector database connections.

    Returns:
        HealthResponse: Health status of all services
    """
    database_status = "unknown"
    vector_db_status = "unknown"

    # Check database connection
    try:
        from src.db.neon import check_db_connection
        if check_db_connection():
            database_status = "connected"
        else:
            database_status = "disconnected"
    except Exception as e:
        print(f"Database health check failed: {e}")
        database_status = "error"

    # Check vector database connection
    try:
        from src.db.qdrant_client import get_qdrant_service
        qdrant = get_qdrant_service()
        if qdrant.check_connection():
            vector_db_status = "connected"
        else:
            vector_db_status = "disconnected"
    except Exception as e:
        print(f"Vector DB health check failed: {e}")
        vector_db_status = "error"

    # Determine overall status
    if database_status == "connected" and vector_db_status == "connected":
        overall_status = "healthy"
        message = "All systems operational"
    elif database_status == "error" or vector_db_status == "error":
        overall_status = "unhealthy"
        message = "System errors detected"
    else:
        overall_status = "degraded"
        message = "Some services unavailable"

    return {
        "status": overall_status,
        "database": database_status,
        "vector_db": vector_db_status,
        "message": message,
    }
