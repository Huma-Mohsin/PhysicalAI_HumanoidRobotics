"""
User Session model for tracking user sessions (anonymous and authenticated).
"""

from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime
from uuid import UUID


class HardwareProfile(BaseModel):
    """Hardware profile from Better-Auth."""
    type: str = Field(..., description="Hardware type: gpu_workstation, edge_device, or cloud_mac")
    details: Optional[dict] = Field(default=None, description="Additional hardware details")


class UserSession(BaseModel):
    """User session model."""
    session_id: UUID = Field(..., description="Unique session identifier")
    user_id: Optional[str] = Field(default=None, description="User ID from Better-Auth (null for anonymous)")
    hardware_profile: Optional[HardwareProfile] = Field(default=None, description="User's hardware profile")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Session creation timestamp")
    last_active: datetime = Field(default_factory=datetime.utcnow, description="Last activity timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
                "user_id": "550e8400-e29b-41d4-a716-446655440000",
                "hardware_profile": {
                    "type": "GPU",
                    "details": {"gpu_model": "RTX 4090", "vram": "24GB"}
                },
                "created_at": "2025-12-11T10:00:00Z",
                "last_active": "2025-12-11T10:30:00Z"
            }
        }


class UserSessionCreate(BaseModel):
    """Schema for creating a new user session."""
    session_id: Optional[UUID] = None  # Optional: if provided, use it; otherwise generate new UUID
    user_id: Optional[str] = None  # Changed to str to match database VARCHAR(255)
    hardware_profile: Optional[HardwareProfile] = None
