"""
CORS Middleware Configuration

Configures Cross-Origin Resource Sharing for the frontend application.
"""

from fastapi.middleware.cors import CORSMiddleware
from fastapi import FastAPI
import os
from dotenv import load_dotenv

load_dotenv()


def add_cors_middleware(app: FastAPI) -> None:
    """
    Add CORS middleware to FastAPI application.

    Args:
        app: FastAPI application instance
    """
    # Get allowed origins from environment (comma-separated)
    cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=cors_origins,
        allow_credentials=True,
        allow_methods=["*"],  # Allow all HTTP methods
        allow_headers=["*"],  # Allow all headers
        expose_headers=["*"],
    )

    print(f"CORS enabled for origins: {cors_origins}")
