"""
Pytest configuration and shared fixtures.
"""

import pytest
import os
from typing import Generator
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

# Set test environment variables
os.environ["ENVIRONMENT"] = "test"
os.environ["DATABASE_URL"] = "sqlite:///:memory:"
os.environ["OPENAI_API_KEY"] = "test-key"
os.environ["QDRANT_URL"] = "http://localhost:6333"
os.environ["QDRANT_API_KEY"] = "test-key"
os.environ["SECRET_KEY"] = "test-secret-key-for-testing-only"
os.environ["RATE_LIMIT_ENABLED"] = "false"  # Disable rate limiting in tests

from src.api.main import app
from src.db.neon import Base, get_async_session


# ============================================================================
# Database Fixtures
# ============================================================================

@pytest.fixture(scope="function")
def test_db():
    """
    Create a test database for each test function.

    Uses in-memory SQLite database that's recreated for each test.
    """
    engine = create_engine(
        "sqlite:///:memory:",
        connect_args={"check_same_thread": False},
        poolclass=StaticPool,
    )

    # Create all tables
    Base.metadata.create_all(bind=engine)

    # Create session maker
    TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

    # Create session
    db = TestingSessionLocal()

    try:
        yield db
    finally:
        db.close()
        Base.metadata.drop_all(bind=engine)


# ============================================================================
# API Client Fixtures
# ============================================================================

@pytest.fixture(scope="module")
def client() -> Generator:
    """
    Create a test client for the FastAPI application.

    Yields:
        TestClient: FastAPI test client
    """
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture(scope="function")
def authenticated_client(client: TestClient) -> Generator:
    """
    Create an authenticated test client with valid JWT token.

    Args:
        client: Base test client

    Yields:
        TestClient: Authenticated test client
    """
    # Create a test user and get token
    signup_data = {
        "email": "test@example.com",
        "password": "testpassword123",
        "os_type": "Ubuntu 22.04",
        "gpu_model": "RTX 4090",
    }

    response = client.post("/auth/signup", json=signup_data)

    if response.status_code == 201:
        token = response.json()["access_token"]

        # Add authorization header to client
        client.headers.update({"Authorization": f"Bearer {token}"})

        yield client

        # Cleanup
        client.headers.pop("Authorization", None)
    else:
        pytest.skip("Failed to create test user for authentication")


# ============================================================================
# Mock Data Fixtures
# ============================================================================

@pytest.fixture
def mock_user_data():
    """Mock user registration data."""
    return {
        "email": "testuser@example.com",
        "password": "securepassword123",
        "os_type": "Ubuntu 22.04",
        "gpu_model": "RTX 4080",
        "environment_preference": "workstation",
        "language_preference": "en"
    }


@pytest.fixture
def mock_chat_query():
    """Mock chat query data."""
    return {
        "query": "How do I create a ROS 2 node?",
        "language": "en",
        "chat_history": []
    }


@pytest.fixture
def mock_feedback_data():
    """Mock feedback submission data."""
    return {
        "message_id": "550e8400-e29b-41d4-a716-446655440000",
        "rating": 1,
        "comment": "Very helpful response!"
    }


# ============================================================================
# Environment Fixtures
# ============================================================================

@pytest.fixture(autouse=True)
def setup_test_environment(monkeypatch):
    """
    Set up test environment variables for all tests.

    Args:
        monkeypatch: Pytest monkeypatch fixture
    """
    test_env_vars = {
        "ENVIRONMENT": "test",
        "DEBUG": "false",
        "LOG_LEVEL": "ERROR",  # Reduce logging noise in tests
        "RATE_LIMIT_ENABLED": "false",
    }

    for key, value in test_env_vars.items():
        monkeypatch.setenv(key, value)
