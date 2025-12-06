"""
Feedback request/response schemas
"""

from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class FeedbackRequest(BaseModel):
    """Submit feedback for a chat message"""
    message_id: str = Field(..., description="Chat message UUID")
    rating: int = Field(..., ge=-1, le=1, description="Rating: -1 (thumbs down), 0 (neutral), 1 (thumbs up)")
    comment: Optional[str] = Field(None, max_length=500, description="Optional feedback comment")

    class Config:
        json_schema_extra = {
            "example": {
                "message_id": "123e4567-e89b-12d3-a456-426614174000",
                "rating": 1,
                "comment": "Very helpful explanation of ROS 2 nodes!"
            }
        }


class FeedbackResponse(BaseModel):
    """Feedback submission response"""
    id: str = Field(..., description="Feedback UUID")
    message_id: str = Field(..., description="Chat message UUID")
    rating: int = Field(..., description="Rating value")
    comment: Optional[str] = Field(None, description="Feedback comment")
    created_at: datetime = Field(..., description="Feedback timestamp")

    class Config:
        from_attributes = True


class FeedbackStats(BaseModel):
    """Aggregated feedback statistics"""
    total_feedback: int = Field(..., description="Total feedback submissions")
    positive_count: int = Field(..., description="Thumbs up count")
    negative_count: int = Field(..., description="Thumbs down count")
    positive_percentage: float = Field(..., description="Percentage of positive feedback")
    average_rating: float = Field(..., description="Average rating (-1 to 1)")
    recent_comments: list[str] = Field(default_factory=list, description="Recent feedback comments")

    class Config:
        json_schema_extra = {
            "example": {
                "total_feedback": 150,
                "positive_count": 120,
                "negative_count": 30,
                "positive_percentage": 80.0,
                "average_rating": 0.6,
                "recent_comments": [
                    "Great explanation!",
                    "Code examples were very helpful"
                ]
            }
        }
