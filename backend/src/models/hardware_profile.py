"""
Hardware Profile Model

Stores user's hardware and software environment for personalized content delivery.
"""

from sqlalchemy import Column, String, Enum, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
import enum

from src.db.neon import Base


class EnvironmentPreference(str, enum.Enum):
    """Environment preference enum."""
    WORKSTATION = "workstation"
    CLOUD = "cloud"
    MAC = "mac"


class LanguagePreference(str, enum.Enum):
    """Language preference enum."""
    EN = "en"
    UR = "ur"


class HardwareProfile(Base):
    """
    Hardware profile table for user environment data.
    """

    __tablename__ = "hardware_profiles"

    # Primary key
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)

    # Foreign key to users table
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), unique=True, nullable=False, index=True)

    # Hardware/Software specifications
    os_type = Column(String(50), nullable=False)  # e.g., "Ubuntu 22.04", "macOS", "Windows"
    gpu_model = Column(String(100), nullable=True)  # e.g., "RTX 4070 Ti", "Integrated Graphics", None

    # User preferences
    environment_preference = Column(
        Enum(EnvironmentPreference),
        default=EnvironmentPreference.CLOUD,
        nullable=False
    )
    language_preference = Column(
        Enum(LanguagePreference),
        default=LanguagePreference.EN,
        nullable=False
    )

    # Metadata
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now(), nullable=False)

    # Relationship
    user = relationship("User", back_populates="hardware_profile")

    def __repr__(self):
        return f"<HardwareProfile(user_id={self.user_id}, os={self.os_type}, gpu={self.gpu_model})>"

    def is_workstation_capable(self) -> bool:
        """
        Check if user's GPU is capable of running workstation mode (Isaac Sim locally).

        Returns:
            bool: True if GPU model indicates high-end NVIDIA card
        """
        if not self.gpu_model:
            return False

        gpu_lower = self.gpu_model.lower()
        workstation_gpus = ["rtx 4070 ti", "rtx 4080", "rtx 4090", "rtx 50"]

        return any(gpu_name in gpu_lower for gpu_name in workstation_gpus)
