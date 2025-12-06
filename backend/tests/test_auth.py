"""
Unit and integration tests for authentication endpoints.
"""

import pytest
from fastapi.testclient import TestClient
from src.core.security import hash_password, verify_password, create_access_token, decode_access_token


# ============================================================================
# Unit Tests - Security Functions
# ============================================================================

@pytest.mark.unit
@pytest.mark.auth
class TestSecurityFunctions:
    """Test password hashing and JWT token functions."""

    def test_password_hashing(self):
        """Test password hashing and verification."""
        password = "testpassword123"

        # Hash password
        hashed = hash_password(password)

        # Verify correct password
        assert verify_password(password, hashed) is True

        # Verify incorrect password
        assert verify_password("wrongpassword", hashed) is False

    def test_jwt_token_creation(self):
        """Test JWT token creation and decoding."""
        payload = {"sub": "user-123", "email": "test@example.com"}

        # Create token
        token = create_access_token(payload)
        assert isinstance(token, str)
        assert len(token) > 0

        # Decode token
        decoded = decode_access_token(token)
        assert decoded["sub"] == "user-123"
        assert decoded["email"] == "test@example.com"
        assert "exp" in decoded  # Expiration timestamp


# ============================================================================
# Integration Tests - Auth Endpoints
# ============================================================================

@pytest.mark.integration
@pytest.mark.auth
class TestAuthEndpoints:
    """Test authentication API endpoints."""

    def test_signup_success(self, client: TestClient, mock_user_data):
        """Test successful user registration."""
        response = client.post("/auth/signup", json=mock_user_data)

        assert response.status_code == 201
        data = response.json()

        # Check response structure
        assert "access_token" in data
        assert "token_type" in data
        assert data["token_type"] == "bearer"
        assert "user" in data

        # Check user data
        user = data["user"]
        assert user["email"] == mock_user_data["email"]
        assert "id" in user
        assert "hardware_profile" in user

        # Check hardware profile
        profile = user["hardware_profile"]
        assert profile["os_type"] == mock_user_data["os_type"]
        assert profile["gpu_model"] == mock_user_data["gpu_model"]
        assert profile["environment_preference"] == mock_user_data["environment_preference"]
        assert "is_workstation_capable" in profile

    def test_signup_duplicate_email(self, client: TestClient, mock_user_data):
        """Test that duplicate email signup fails."""
        # First signup
        client.post("/auth/signup", json=mock_user_data)

        # Duplicate signup
        response = client.post("/auth/signup", json=mock_user_data)

        assert response.status_code == 400
        assert "already registered" in response.json()["detail"].lower()

    def test_signup_invalid_email(self, client: TestClient):
        """Test signup with invalid email format."""
        invalid_data = {
            "email": "not-an-email",
            "password": "testpassword123"
        }

        response = client.post("/auth/signup", json=invalid_data)

        assert response.status_code == 422  # Validation error

    def test_signup_weak_password(self, client: TestClient):
        """Test signup with password too short."""
        weak_data = {
            "email": "test@example.com",
            "password": "123"  # Too short
        }

        response = client.post("/auth/signup", json=weak_data)

        assert response.status_code == 422  # Validation error

    def test_signin_success(self, client: TestClient, mock_user_data):
        """Test successful user sign in."""
        # First create user
        client.post("/auth/signup", json=mock_user_data)

        # Sign in
        signin_data = {
            "email": mock_user_data["email"],
            "password": mock_user_data["password"]
        }
        response = client.post("/auth/signin", json=signin_data)

        assert response.status_code == 200
        data = response.json()

        assert "access_token" in data
        assert "user" in data
        assert data["user"]["email"] == mock_user_data["email"]

    def test_signin_wrong_password(self, client: TestClient, mock_user_data):
        """Test sign in with incorrect password."""
        # Create user
        client.post("/auth/signup", json=mock_user_data)

        # Try to sign in with wrong password
        signin_data = {
            "email": mock_user_data["email"],
            "password": "wrongpassword"
        }
        response = client.post("/auth/signin", json=signin_data)

        assert response.status_code == 401
        assert "incorrect" in response.json()["detail"].lower()

    def test_signin_nonexistent_user(self, client: TestClient):
        """Test sign in with nonexistent user."""
        signin_data = {
            "email": "nonexistent@example.com",
            "password": "password123"
        }
        response = client.post("/auth/signin", json=signin_data)

        assert response.status_code == 401

    def test_get_current_user(self, authenticated_client: TestClient):
        """Test getting current authenticated user profile."""
        response = authenticated_client.get("/auth/me")

        assert response.status_code == 200
        data = response.json()

        assert "id" in data
        assert "email" in data
        assert "hardware_profile" in data

    def test_get_current_user_unauthorized(self, client: TestClient):
        """Test getting user profile without authentication."""
        response = client.get("/auth/me")

        assert response.status_code == 403  # Forbidden (no auth header)

    def test_update_hardware_profile(self, authenticated_client: TestClient):
        """Test updating user's hardware profile."""
        update_data = {
            "gpu_model": "RTX 4090",
            "environment_preference": "cloud"
        }

        response = authenticated_client.put("/auth/hardware-profile", json=update_data)

        assert response.status_code == 200
        data = response.json()

        assert data["gpu_model"] == "RTX 4090"
        assert data["environment_preference"] == "cloud"


# ============================================================================
# Integration Tests - Hardware Profile Logic
# ============================================================================

@pytest.mark.integration
@pytest.mark.auth
class TestHardwareProfile:
    """Test hardware profile capabilities detection."""

    @pytest.mark.parametrize("gpu_model,expected_capability", [
        ("RTX 4090", True),
        ("RTX 4080", True),
        ("RTX 4070 Ti", True),
        ("RTX 5090", True),
        ("RTX 3090", False),
        ("GTX 1080", False),
        ("No GPU", False),
    ])
    def test_workstation_capability_detection(
        self,
        client: TestClient,
        gpu_model: str,
        expected_capability: bool
    ):
        """Test workstation capability based on GPU model."""
        user_data = {
            "email": f"test_{gpu_model.replace(' ', '_')}@example.com",
            "password": "testpassword123",
            "gpu_model": gpu_model
        }

        response = client.post("/auth/signup", json=user_data)

        assert response.status_code == 201
        profile = response.json()["user"]["hardware_profile"]
        assert profile["is_workstation_capable"] == expected_capability
