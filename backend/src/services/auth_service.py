"""
Authentication service for Better-Auth integration.
Handles user profile management and hardware profile association.
"""

from typing import Optional, Dict, Any
from datetime import datetime
from uuid import UUID
import asyncpg
from pydantic import BaseModel, Field

from ..models.user_profile import HardwareDetails
from ..utils.logger import logger
from ..utils.config import settings


class UserProfile(BaseModel):
    """User profile with hardware information."""
    user_id: str = Field(..., description="User ID from Better-Auth")
    email: str = Field(..., description="User email")
    name: Optional[str] = Field(default=None, description="User's full name")
    hardware_type: Optional[str] = Field(
        default=None,
        description="Type of hardware: gpu_workstation, edge_device, cloud_mac"
    )
    hardware_details: Optional[HardwareDetails] = Field(
        default=None,
        description="Detailed hardware specifications"
    )
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Profile creation timestamp")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last profile update timestamp")


class UserProfileCreate(BaseModel):
    """Schema for creating a new user profile."""
    user_id: str
    email: str
    name: Optional[str] = None
    hardware_type: Optional[str] = None
    hardware_details: Optional[HardwareDetails] = None


class UserProfileUpdate(BaseModel):
    """Schema for updating a user profile."""
    name: Optional[str] = None
    hardware_type: Optional[str] = None
    hardware_details: Optional[HardwareDetails] = None


class AuthService:
    """Service class for handling authentication and user profile operations."""

    def __init__(self, db_pool: asyncpg.Pool):
        self.db_pool = db_pool

    async def create_user_profile(self, profile_data: UserProfileCreate) -> UserProfile:
        """Create a new user profile with hardware information."""
        try:
            async with self.db_pool.acquire() as conn:
                async with conn.transaction():
                    # Insert user profile
                    query = """
                        INSERT INTO user_profiles (
                            user_id,
                            email,
                            name,
                            hardware_type,
                            hardware_details,
                            created_at,
                            updated_at
                        )
                        VALUES ($1, $2, $3, $4, $5, $6, $7)
                        ON CONFLICT (user_id)
                        DO UPDATE SET
                            email = EXCLUDED.email,
                            name = EXCLUDED.name,
                            hardware_type = EXCLUDED.hardware_type,
                            hardware_details = EXCLUDED.hardware_details,
                            updated_at = EXCLUDED.updated_at
                        RETURNING user_id, email, name, hardware_type, hardware_details, created_at, updated_at
                    """

                    # Prepare hardware details for JSONB storage
                    hardware_details_json = None
                    if profile_data.hardware_details:
                        hardware_details_json = {
                            "gpu_model": profile_data.hardware_details.gpu_model,
                            "cpu_model": profile_data.hardware_details.cpu_model,
                            "ram_size": profile_data.hardware_details.ram_size,
                            "os_type": profile_data.hardware_details.os_type,
                            "additional_notes": profile_data.hardware_details.additional_notes
                        }

                    result = await conn.fetchrow(
                        query,
                        profile_data.user_id,
                        profile_data.email,
                        profile_data.name,
                        profile_data.hardware_type,
                        hardware_details_json,
                        profile_data.created_at if hasattr(profile_data, 'created_at') else datetime.utcnow(),
                        datetime.utcnow()
                    )

                    # Create HardwareDetails object from result
                    hardware_details = None
                    if result['hardware_details']:
                        hardware_details = HardwareDetails(
                            gpu_model=result['hardware_details'].get('gpu_model'),
                            cpu_model=result['hardware_details'].get('cpu_model'),
                            ram_size=result['hardware_details'].get('ram_size'),
                            os_type=result['hardware_details'].get('os_type'),
                            additional_notes=result['hardware_details'].get('additional_notes')
                        )

                    return UserProfile(
                        user_id=result['user_id'],
                        email=result['email'],
                        name=result['name'],
                        hardware_type=result['hardware_type'],
                        hardware_details=hardware_details,
                        created_at=result['created_at'],
                        updated_at=result['updated_at']
                    )
        except Exception as e:
            logger.error(f"Error creating user profile: {str(e)}")
            raise

    async def get_user_profile(self, user_id: str) -> Optional[UserProfile]:
        """Retrieve a user profile by user ID."""
        try:
            async with self.db_pool.acquire() as conn:
                query = """
                    SELECT user_id, email, name, hardware_type, hardware_details, created_at, updated_at
                    FROM user_profiles
                    WHERE user_id = $1
                """

                result = await conn.fetchrow(query, user_id)

                if not result:
                    return None

                # Create HardwareDetails object from result
                hardware_details = None
                if result['hardware_details']:
                    hardware_details = HardwareDetails(
                        gpu_model=result['hardware_details'].get('gpu_model'),
                        cpu_model=result['hardware_details'].get('cpu_model'),
                        ram_size=result['hardware_details'].get('ram_size'),
                        os_type=result['hardware_details'].get('os_type'),
                        additional_notes=result['hardware_details'].get('additional_notes')
                    )

                return UserProfile(
                    user_id=result['user_id'],
                    email=result['email'],
                    name=result['name'],
                    hardware_type=result['hardware_type'],
                    hardware_details=hardware_details,
                    created_at=result['created_at'],
                    updated_at=result['updated_at']
                )
        except Exception as e:
            logger.error(f"Error retrieving user profile: {str(e)}")
            raise

    async def update_user_profile(self, user_id: str, profile_update: UserProfileUpdate) -> Optional[UserProfile]:
        """Update a user profile with new information."""
        try:
            async with self.db_pool.acquire() as conn:
                # Build dynamic query based on what fields are provided
                update_fields = []
                update_values = []

                if profile_update.name is not None:
                    update_fields.append("name = $2")
                    update_values.append(profile_update.name)

                if profile_update.hardware_type is not None or profile_update.hardware_details is not None:
                    # Prepare hardware details for JSONB storage
                    hardware_details_json = None
                    if profile_update.hardware_details:
                        hardware_details_json = {
                            "gpu_model": profile_update.hardware_details.gpu_model,
                            "cpu_model": profile_update.hardware_details.cpu_model,
                            "ram_size": profile_update.hardware_details.ram_size,
                            "os_type": profile_update.hardware_details.os_type,
                            "additional_notes": profile_update.hardware_details.additional_notes
                        }

                    update_fields.append("hardware_type = $3, hardware_details = $4")
                    update_values.append(profile_update.hardware_type)
                    update_values.append(hardware_details_json)

                update_fields.append("updated_at = $5")
                update_values.extend([user_id, datetime.utcnow()])

                if not update_fields:
                    # Nothing to update, just return current profile
                    return await self.get_user_profile(user_id)

                query = f"""
                    UPDATE user_profiles
                    SET {', '.join(update_fields)}
                    WHERE user_id = $1
                    RETURNING user_id, email, name, hardware_type, hardware_details, created_at, updated_at
                """

                result = await conn.fetchrow(query, *update_values)

                if not result:
                    return None

                # Create HardwareDetails object from result
                hardware_details = None
                if result['hardware_details']:
                    hardware_details = HardwareDetails(
                        gpu_model=result['hardware_details'].get('gpu_model'),
                        cpu_model=result['hardware_details'].get('cpu_model'),
                        ram_size=result['hardware_details'].get('ram_size'),
                        os_type=result['hardware_details'].get('os_type'),
                        additional_notes=result['hardware_details'].get('additional_notes')
                    )

                return UserProfile(
                    user_id=result['user_id'],
                    email=result['email'],
                    name=result['name'],
                    hardware_type=result['hardware_type'],
                    hardware_details=hardware_details,
                    created_at=result['created_at'],
                    updated_at=result['updated_at']
                )
        except Exception as e:
            logger.error(f"Error updating user profile: {str(e)}")
            raise

    async def get_hardware_profile(self, user_id: str) -> Optional[dict]:
        """Get only the hardware profile for a user as a dictionary."""
        try:
            profile = await self.get_user_profile(user_id)
            if not profile:
                return None

            # Return a simplified dict structure for compatibility with the RAG service
            return {
                "type": profile.hardware_type,
                "details": profile.hardware_details.dict() if profile.hardware_details else None
            }
        except Exception as e:
            logger.error(f"Error retrieving hardware profile: {str(e)}")
            raise


# Global instance - will be initialized when the app starts
auth_service: Optional[AuthService] = None


def init_auth_service(db_pool: asyncpg.Pool) -> AuthService:
    """Initialize the auth service with database connection."""
    global auth_service
    auth_service = AuthService(db_pool)
    return auth_service


def get_auth_service() -> AuthService:
    """Get the global auth service instance."""
    global auth_service
    if not auth_service:
        raise RuntimeError("Auth service not initialized. Call init_auth_service first.")
    return auth_service