"""
Vercel entry point for the FastAPI application.
This file serves as the entry point for Vercel deployments.
"""

from src.main import app

# This ensures Vercel can properly import and run the application
application = app

# For Vercel Python runtime compatibility
handler = app