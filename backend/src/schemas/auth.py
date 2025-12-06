"""
Authentication request/response schemas
"""

from pydantic import BaseModel, Field, EmailStr
from typing import Optional
from datetime import datetime
from enum import Enum


class EnvironmentPreference(str, Enum):
    """Hardware environment preferences"""
    WORKSTATION = "workstation"
    CLOUD = "cloud"
    MAC = "mac"


class LanguagePreference(str, Enum):
    """Content language preferences"""
    EN = "en"
    UR = "ur"


class SignUpRequest(BaseModel):
    """Sign up request schema"""
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, max_length=100, description="User password (min 8 characters)")

    # Hardware profile fields (optional during signup)
    os_type: Optional[str] = Field(None, max_length=50, description="Operating system (e.g., Ubuntu 22.04, Windows 11, macOS)")
    gpu_model: Optional[str] = Field(None, max_length=100, description="GPU model (e.g., RTX 4070 Ti, RTX 3060)")
    environment_preference: Optional[EnvironmentPreference] = Field(EnvironmentPreference.CLOUD, description="Preferred environment mode")
    language_preference: Optional[LanguagePreference] = Field(LanguagePreference.EN, description="Content language")

    class Config:
        json_schema_extra = {
            "example": {
                "email": "student@example.com",
                "password": "SecurePass123!",
                "os_type": "Ubuntu 22.04",
                "gpu_model": "RTX 4070 Ti",
                "environment_preference": "workstation",
                "language_preference": "en"
            }
        }


class SignInRequest(BaseModel):
    """Sign in request schema"""
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="User password")

    class Config:
        json_schema_extra = {
            "example": {
                "email": "student@example.com",
                "password": "SecurePass123!"
            }
        }


class HardwareProfileResponse(BaseModel):
    """Hardware profile response schema"""
    id: str = Field(..., description="Profile UUID")
    user_id: str = Field(..., description="User UUID")
    os_type: Optional[str] = Field(None, description="Operating system")
    gpu_model: Optional[str] = Field(None, description="GPU model")
    environment_preference: EnvironmentPreference = Field(..., description="Environment mode")
    language_preference: LanguagePreference = Field(..., description="Content language")
    is_workstation_capable: bool = Field(..., description="Whether GPU supports workstation mode")
    created_at: datetime = Field(..., description="Profile creation timestamp")
    updated_at: Optional[datetime] = Field(None, description="Last update timestamp")

    class Config:
        from_attributes = True


class UserResponse(BaseModel):
    """User response schema (excludes password)"""
    id: str = Field(..., description="User UUID")
    email: str = Field(..., description="User email")
    created_at: datetime = Field(..., description="Account creation timestamp")
    hardware_profile: Optional[HardwareProfileResponse] = Field(None, description="User's hardware profile")

    class Config:
        from_attributes = True


class AuthResponse(BaseModel):
    """Authentication response with token and user data"""
    access_token: str = Field(..., description="JWT access token")
    token_type: str = Field(default="bearer", description="Token type")
    user: UserResponse = Field(..., description="User data")

    class Config:
        json_schema_extra = {
            "example": {
                "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
                "token_type": "bearer",
                "user": {
                    "id": "123e4567-e89b-12d3-a456-426614174000",
                    "email": "student@example.com",
                    "created_at": "2025-12-06T10:00:00Z",
                    "hardware_profile": {
                        "id": "123e4567-e89b-12d3-a456-426614174001",
                        "user_id": "123e4567-e89b-12d3-a456-426614174000",
                        "os_type": "Ubuntu 22.04",
                        "gpu_model": "RTX 4070 Ti",
                        "environment_preference": "workstation",
                        "language_preference": "en",
                        "is_workstation_capable": True,
                        "created_at": "2025-12-06T10:00:00Z",
                        "updated_at": None
                    }
                }
            }
        }


class UpdateHardwareProfileRequest(BaseModel):
    """Update hardware profile request"""
    os_type: Optional[str] = Field(None, max_length=50, description="Operating system")
    gpu_model: Optional[str] = Field(None, max_length=100, description="GPU model")
    environment_preference: Optional[EnvironmentPreference] = Field(None, description="Environment mode")
    language_preference: Optional[LanguagePreference] = Field(None, description="Content language")

    class Config:
        json_schema_extra = {
            "example": {
                "os_type": "Ubuntu 22.04",
                "gpu_model": "RTX 4090",
                "environment_preference": "workstation",
                "language_preference": "ur"
            }
        }
