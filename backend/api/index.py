"""
API entry point for FastAPI application.
This file sits at the root level to properly handle Python imports.
"""

import sys
from pathlib import Path

# Add src directory to Python path
src_path = Path(__file__).parent.parent / "src"
sys.path.insert(0, str(src_path))

# Now import the FastAPI app
from main import app
