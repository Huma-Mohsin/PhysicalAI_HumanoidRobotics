"""
Authentication API endpoints
"""

from fastapi import APIRouter, HTTPException, status, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
import logging
from datetime import timedelta

from ...schemas.auth import (
    SignUpRequest,
    SignInRequest,
    AuthResponse,
    UserResponse,
    HardwareProfileResponse,
    UpdateHardwareProfileRequest
)
from ...models.user import User
from ...models.hardware_profile import HardwareProfile, EnvironmentPreference, LanguagePreference
from ...db.neon import get_async_session
from ...core.security import (
    hash_password,
    verify_password,
    create_access_token,
    get_current_user_id,
    ACCESS_TOKEN_EXPIRE_MINUTES
)

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    request: SignUpRequest,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Register a new user with optional hardware profile

    Workflow:
    1. Validate email uniqueness
    2. Hash password
    3. Create user record
    4. Create hardware profile (if data provided)
    5. Generate JWT token
    6. Return user data and token

    Args:
        request: SignUpRequest with email, password, and optional hardware info

    Returns:
        AuthResponse with access token and user data
    """
    # Check if email already exists
    result = await session.execute(
        select(User).where(User.email == request.email)
    )
    existing_user = result.scalar_one_or_none()

    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    try:
        # Create user
        user = User(
            email=request.email,
            password_hash=hash_password(request.password)
        )
        session.add(user)
        await session.flush()  # Get user.id before creating profile

        # Create hardware profile
        hardware_profile = HardwareProfile(
            user_id=user.id,
            os_type=request.os_type,
            gpu_model=request.gpu_model,
            environment_preference=request.environment_preference or EnvironmentPreference.CLOUD,
            language_preference=request.language_preference or LanguagePreference.EN
        )
        session.add(hardware_profile)
        await session.commit()
        await session.refresh(user)
        await session.refresh(hardware_profile)

        # Generate access token
        access_token = create_access_token(
            data={"sub": str(user.id), "email": user.email},
            expires_delta=timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        )

        # Construct response
        user_response = UserResponse(
            id=str(user.id),
            email=user.email,
            created_at=user.created_at,
            hardware_profile=HardwareProfileResponse(
                id=str(hardware_profile.id),
                user_id=str(hardware_profile.user_id),
                os_type=hardware_profile.os_type,
                gpu_model=hardware_profile.gpu_model,
                environment_preference=hardware_profile.environment_preference,
                language_preference=hardware_profile.language_preference,
                is_workstation_capable=hardware_profile.is_workstation_capable(),
                created_at=hardware_profile.created_at,
                updated_at=hardware_profile.updated_at
            )
        )

        return AuthResponse(
            access_token=access_token,
            token_type="bearer",
            user=user_response
        )

    except Exception as e:
        await session.rollback()
        logger.error(f"Signup error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create user account"
        )


@router.post("/signin", response_model=AuthResponse)
async def signin(
    request: SignInRequest,
    session: AsyncSession = Depends(get_async_session)
):
    """
    Authenticate user and return access token

    Args:
        request: SignInRequest with email and password

    Returns:
        AuthResponse with access token and user data
    """
    # Find user by email
    result = await session.execute(
        select(User).where(User.email == request.email)
    )
    user = result.scalar_one_or_none()

    if not user or not verify_password(request.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    # Load hardware profile
    await session.refresh(user, ["hardware_profile"])

    # Generate access token
    access_token = create_access_token(
        data={"sub": str(user.id), "email": user.email}
    )

    # Construct response
    hardware_profile_response = None
    if user.hardware_profile:
        hardware_profile_response = HardwareProfileResponse(
            id=str(user.hardware_profile.id),
            user_id=str(user.hardware_profile.user_id),
            os_type=user.hardware_profile.os_type,
            gpu_model=user.hardware_profile.gpu_model,
            environment_preference=user.hardware_profile.environment_preference,
            language_preference=user.hardware_profile.language_preference,
            is_workstation_capable=user.hardware_profile.is_workstation_capable(),
            created_at=user.hardware_profile.created_at,
            updated_at=user.hardware_profile.updated_at
        )

    user_response = UserResponse(
        id=str(user.id),
        email=user.email,
        created_at=user.created_at,
        hardware_profile=hardware_profile_response
    )

    return AuthResponse(
        access_token=access_token,
        token_type="bearer",
        user=user_response
    )


@router.get("/me", response_model=UserResponse)
async def get_current_user(
    user_id: str = Depends(get_current_user_id),
    session: AsyncSession = Depends(get_async_session)
):
    """
    Get current authenticated user's profile

    Args:
        user_id: Extracted from JWT token

    Returns:
        UserResponse with hardware profile
    """
    result = await session.execute(
        select(User).where(User.id == user_id)
    )
    user = result.scalar_one_or_none()

    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    await session.refresh(user, ["hardware_profile"])

    hardware_profile_response = None
    if user.hardware_profile:
        hardware_profile_response = HardwareProfileResponse(
            id=str(user.hardware_profile.id),
            user_id=str(user.hardware_profile.user_id),
            os_type=user.hardware_profile.os_type,
            gpu_model=user.hardware_profile.gpu_model,
            environment_preference=user.hardware_profile.environment_preference,
            language_preference=user.hardware_profile.language_preference,
            is_workstation_capable=user.hardware_profile.is_workstation_capable(),
            created_at=user.hardware_profile.created_at,
            updated_at=user.hardware_profile.updated_at
        )

    return UserResponse(
        id=str(user.id),
        email=user.email,
        created_at=user.created_at,
        hardware_profile=hardware_profile_response
    )


@router.put("/hardware-profile", response_model=HardwareProfileResponse)
async def update_hardware_profile(
    request: UpdateHardwareProfileRequest,
    user_id: str = Depends(get_current_user_id),
    session: AsyncSession = Depends(get_async_session)
):
    """
    Update current user's hardware profile

    Args:
        request: UpdateHardwareProfileRequest with new values
        user_id: Extracted from JWT token

    Returns:
        Updated HardwareProfileResponse
    """
    # Find user's hardware profile
    result = await session.execute(
        select(HardwareProfile).where(HardwareProfile.user_id == user_id)
    )
    profile = result.scalar_one_or_none()

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Hardware profile not found"
        )

    # Update fields (only if provided)
    if request.os_type is not None:
        profile.os_type = request.os_type
    if request.gpu_model is not None:
        profile.gpu_model = request.gpu_model
    if request.environment_preference is not None:
        profile.environment_preference = request.environment_preference
    if request.language_preference is not None:
        profile.language_preference = request.language_preference

    try:
        await session.commit()
        await session.refresh(profile)

        return HardwareProfileResponse(
            id=str(profile.id),
            user_id=str(profile.user_id),
            os_type=profile.os_type,
            gpu_model=profile.gpu_model,
            environment_preference=profile.environment_preference,
            language_preference=profile.language_preference,
            is_workstation_capable=profile.is_workstation_capable(),
            created_at=profile.created_at,
            updated_at=profile.updated_at
        )

    except Exception as e:
        await session.rollback()
        logger.error(f"Profile update error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update hardware profile"
        )
