"""
Conversation model for grouping related messages.
"""

from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime
from uuid import UUID


class Conversation(BaseModel):
    """Conversation model."""
    conv_id: UUID = Field(..., description="Unique conversation identifier")
    session_id: UUID = Field(..., description="Session ID this conversation belongs to")
    title: Optional[str] = Field(default=None, max_length=200, description="Auto-generated conversation title")
    summary: Optional[str] = Field(default=None, description="LLM-generated summary after 20+ messages")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Conversation creation timestamp")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last update timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "conv_id": "550e8400-e29b-41d4-a716-446655440000",
                "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
                "title": "ROS 2 Components and Setup",
                "summary": None,
                "created_at": "2025-12-11T10:00:00Z",
                "updated_at": "2025-12-11T10:30:00Z"
            }
        }


class ConversationCreate(BaseModel):
    """Schema for creating a new conversation."""
    session_id: UUID
    title: Optional[str] = None
