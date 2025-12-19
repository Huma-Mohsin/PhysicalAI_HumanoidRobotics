"""
User Profile model for Better-Auth integration.
Feature 008: Added software/hardware background fields for personalization.
"""

from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List
from datetime import datetime


class HardwareDetails(BaseModel):
    """Detailed hardware specifications."""
    gpu_model: Optional[str] = Field(default=None, description="GPU model (e.g., RTX 4090)")
    cpu_model: Optional[str] = Field(default=None, description="CPU model (e.g., Intel i9-13900K)")
    ram_size: Optional[int] = Field(default=None, description="RAM size in GB")
    os_type: Optional[str] = Field(default=None, description="Operating system (windows, linux, macos)")
    additional_notes: Optional[str] = Field(default=None, description="Additional hardware notes")


class UserProfile(BaseModel):
    """User profile model with software/hardware background for personalization."""
    user_id: str = Field(..., description="Unique user identifier from Better-Auth")
    email: str = Field(..., description="User's email address")
    name: Optional[str] = Field(default=None, description="User's full name")
    password: Optional[str] = Field(default=None, description="User password (TECH DEBT: plaintext)")

    # Software background (Feature 008)
    software_experience: Optional[str] = Field(
        default=None,
        description="Software experience level: beginner, intermediate, expert"
    )
    programming_languages: Optional[List[str]] = Field(
        default=None,
        description="Programming languages known: Python, JavaScript, C++, ROS 2, etc."
    )

    # Hardware background (Feature 005/008)
    hardware_type: Optional[str] = Field(
        default=None,
        description="Type of hardware: gpu_workstation, edge_device, cloud_mac"
    )
    hardware_details: Optional[HardwareDetails] = Field(
        default=None,
        description="Detailed hardware specifications"
    )
    hardware_experience: Optional[bool] = Field(
        default=False,
        description="Has experience with robotics hardware"
    )

    created_at: datetime = Field(default_factory=datetime.utcnow, description="Profile creation timestamp")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last profile update timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "user_id": "user_2j2h34jkh234jkh234",
                "email": "user@example.com",
                "name": "John Doe",
                "hardware_type": "gpu_workstation",
                "hardware_details": {
                    "gpu_model": "RTX 4090",
                    "cpu_model": "Intel i9-13900K",
                    "ram_size": 128,
                    "os_type": "linux",
                    "additional_notes": "Multiple GPUs configured"
                },
                "created_at": "2025-12-16T10:00:00Z",
                "updated_at": "2025-12-16T10:30:00Z"
            }
        }


class UserProfileCreate(BaseModel):
    """Schema for creating a new user profile."""
    user_id: str
    email: str
    password: str  # TECH DEBT: plaintext
    name: Optional[str] = None

    # Software background
    software_experience: Optional[str] = None
    programming_languages: Optional[List[str]] = None

    # Hardware background
    hardware_type: Optional[str] = None
    hardware_details: Optional[HardwareDetails] = None
    hardware_experience: Optional[bool] = False


class UserProfileUpdate(BaseModel):
    """Schema for updating a user profile."""
    name: Optional[str] = None

    # Software background
    software_experience: Optional[str] = None
    programming_languages: Optional[List[str]] = None

    # Hardware background
    hardware_type: Optional[str] = None
    hardware_details: Optional[HardwareDetails] = None
    hardware_experience: Optional[bool] = None