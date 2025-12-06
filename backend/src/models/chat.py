"""
Chat Models

Represents chatbot interaction messages and feedback.
"""

from sqlalchemy import Column, String, Text, Enum, DateTime, ForeignKey, Integer
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
import enum

from src.db.neon import Base


class FeedbackRating(str, enum.Enum):
    """Feedback rating enum."""
    THUMBS_UP = "thumbs_up"
    THUMBS_DOWN = "thumbs_down"


class ChatMessage(Base):
    """
    Chat message table for storing query-response pairs.
    """

    __tablename__ = "chat_messages"

    # Primary key
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)

    # Foreign key to users table (nullable for anonymous users)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=True, index=True)

    # Message content
    query = Column(Text, nullable=False)  # User's question
    response = Column(Text, nullable=False)  # Chatbot's answer
    selected_text = Column(Text, nullable=True)  # Context from text selection (if applicable)

    # Metadata
    language = Column(String(2), default="en", nullable=False)  # 'en' or 'ur'
    session_id = Column(String(255), nullable=True)  # For conversation continuity
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)

    # Relationships
    user = relationship("User", back_populates="chat_messages")
    feedback = relationship("Feedback", back_populates="message", uselist=False, cascade="all, delete-orphan")

    def __repr__(self):
        return f"<ChatMessage(id={self.id}, user_id={self.user_id}, language={self.language})>"


class Feedback(Base):
    """
    Feedback table for user ratings on chatbot responses.
    """

    __tablename__ = "feedback"

    # Primary key
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)

    # Foreign key to chat_messages table
    message_id = Column(UUID(as_uuid=True), ForeignKey("chat_messages.id", ondelete="CASCADE"), unique=True, nullable=False, index=True)

    # Feedback data
    rating = Column(Enum(FeedbackRating), nullable=False)  # thumbs_up or thumbs_down
    optional_text = Column(Text, nullable=True)  # Additional comments (shown when thumbs_down)

    # Metadata
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)

    # Relationship
    message = relationship("ChatMessage", back_populates="feedback")

    def __repr__(self):
        return f"<Feedback(message_id={self.message_id}, rating={self.rating})>"
