"""
Message model for individual user and assistant messages.
"""

from pydantic import BaseModel, Field
from typing import Optional, Literal
from datetime import datetime
from uuid import UUID


class TextSelection(BaseModel):
    """Text selection metadata."""
    text: str = Field(..., description="Highlighted text content")
    chapter_id: str = Field(..., description="Chapter file name")
    start_offset: int = Field(..., description="Character position where selection starts")
    end_offset: int = Field(..., description="Character position where selection ends")
    context_before: Optional[str] = Field(default=None, description="50 chars before selection")
    context_after: Optional[str] = Field(default=None, description="50 chars after selection")


class MessageMetadata(BaseModel):
    """Performance and debugging metadata."""
    tokens_used: Optional[int] = Field(default=None, description="Total tokens (prompt + completion)")
    latency_ms: Optional[int] = Field(default=None, description="Response time in milliseconds")
    model: Optional[str] = Field(default=None, description="OpenAI model used")
    retrieved_chunks: Optional[int] = Field(default=None, description="Number of chunks retrieved")
    qdrant_query_ms: Optional[int] = Field(default=None, description="Qdrant search latency")
    embedding_model: Optional[str] = Field(default=None, description="Embedding model used")


class Message(BaseModel):
    """Message model."""
    message_id: UUID = Field(..., description="Unique message identifier")
    conv_id: UUID = Field(..., description="Conversation ID this message belongs to")
    role: Literal["user", "assistant"] = Field(..., description="Message role")
    content: str = Field(..., description="Message content")
    text_selection: Optional[TextSelection] = Field(default=None, description="Text selection context (if applicable)")
    metadata: Optional[MessageMetadata] = Field(default=None, description="Performance metadata")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Message creation timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "message_id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
                "conv_id": "550e8400-e29b-41d4-a716-446655440000",
                "role": "assistant",
                "content": "ROS 2 has three main components: Nodes, Topics, and Services...",
                "text_selection": None,
                "metadata": {
                    "tokens_used": 856,
                    "latency_ms": 2340,
                    "model": "gpt-4o-mini",
                    "retrieved_chunks": 5,
                    "qdrant_query_ms": 120,
                    "embedding_model": "text-embedding-3-small"
                },
                "created_at": "2025-12-11T10:00:00Z"
            }
        }


class MessageCreate(BaseModel):
    """Schema for creating a new message."""
    conv_id: UUID
    role: Literal["user", "assistant"]
    content: str
    text_selection: Optional[TextSelection] = None
    metadata: Optional[MessageMetadata] = None
