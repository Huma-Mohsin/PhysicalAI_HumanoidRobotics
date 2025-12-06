"""
Chat API request/response schemas
"""

from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class ChatMessage(BaseModel):
    """Single chat message"""
    role: str = Field(..., description="Message role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")


class ChatRequest(BaseModel):
    """Chat endpoint request"""
    query: str = Field(..., min_length=1, max_length=1000, description="User's question")
    language: str = Field(default="en", pattern="^(en|ur)$", description="Content language")
    selected_text: Optional[str] = Field(None, max_length=5000, description="Selected text for contextual query")
    chat_history: Optional[List[ChatMessage]] = Field(None, max_length=10, description="Previous conversation messages")
    user_id: Optional[str] = Field(None, description="User ID for authenticated requests")

    class Config:
        json_schema_extra = {
            "example": {
                "query": "How do I create a ROS 2 node that uses GPT-4 for robot control?",
                "language": "en",
                "selected_text": None,
                "chat_history": [],
                "user_id": None
            }
        }


class RetrievedChunk(BaseModel):
    """Retrieved context chunk from vector DB"""
    id: str = Field(..., description="Chunk ID")
    score: float = Field(..., description="Similarity score (0.0 to 1.0)")
    content: str = Field(..., description="Chunk content")
    metadata: dict = Field(default_factory=dict, description="Additional metadata")


class ChatResponse(BaseModel):
    """Chat endpoint response"""
    response: str = Field(..., description="AI-generated response")
    retrieved_chunks: List[RetrievedChunk] = Field(..., description="Context chunks used")
    num_chunks: int = Field(..., description="Number of chunks retrieved")
    language: str = Field(..., description="Response language")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Response timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "response": "To create a ROS 2 node with GPT-4 integration, you'll use the `rclpy` library and the OpenAI SDK...",
                "retrieved_chunks": [
                    {
                        "id": "chunk_123",
                        "score": 0.89,
                        "content": "class LLMRobotController(Node): ...",
                        "metadata": {"module": "ros2", "chapter": "nodes"}
                    }
                ],
                "num_chunks": 3,
                "language": "en",
                "timestamp": "2025-12-06T10:30:00Z"
            }
        }
