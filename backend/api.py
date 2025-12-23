"""
API entry point for the FastAPI application.
This file serves as the entry point for Vercel serverless deployment.
"""

from src.main import app

# Export the FastAPI app for Vercel
# Vercel will automatically detect and serve the ASGI app
__all__ = ["app"]