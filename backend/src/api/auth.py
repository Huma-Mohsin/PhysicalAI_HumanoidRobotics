"""
Better-Auth API endpoints for the RAG Chatbot backend.
This module provides authentication endpoints that integrate with the frontend Better-Auth client.
"""
from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import JSONResponse
from typing import Optional
from datetime import datetime
from uuid import UUID, uuid4
from pydantic import BaseModel
from ..utils.logger import logger
from ..utils.config import settings
from ..services.auth_service import get_auth_service, UserProfileCreate, UserProfileUpdate
from ..services.database_service import db_service
from ..models.user_session import UserSessionCreate

router = APIRouter(prefix="/api/auth", tags=["auth"])


# Request models
class SignupRequest(BaseModel):
    """Request body for signup endpoint with software/hardware background (Feature 008)."""
    email: str
    password: str
    name: str = ""

    # Software background (Feature 008)
    software_experience: Optional[str] = None  # beginner, intermediate, expert
    programming_languages: Optional[list] = None  # ["Python", "JavaScript", etc.]

    # Hardware background (Feature 005/008)
    hardware_type: Optional[str] = None  # gpu_workstation, edge_device, cloud_mac
    hardware_details: Optional[dict] = None
    hardware_experience: Optional[bool] = False


class LoginRequest(BaseModel):
    """Request body for login endpoint."""
    email: str
    password: str


@router.post("/signup")
async def signup(request: Request, data: SignupRequest):
    """
    Handle user signup requests from Better-Auth frontend.
    Creates a user profile with hardware information AND creates a session.
    """
    try:
        # Get the auth service instance (lazy initialization)
        auth_service = await get_auth_service()

        # Generate user_id
        user_id = str(uuid4())

        # Create user profile with software/hardware background (Feature 008)
        profile_data = UserProfileCreate(
            user_id=user_id,
            email=data.email,
            password=data.password,  # TECH DEBT: Plain text password
            name=data.name,
            # Software background
            software_experience=data.software_experience,
            programming_languages=data.programming_languages,
            # Hardware background
            hardware_type=data.hardware_type,
            hardware_details=data.hardware_details,
            hardware_experience=data.hardware_experience
        )

        # Create the user profile in the database
        user_profile = await auth_service.create_user_profile(profile_data)

        # Create session for this user
        await db_service.ensure_connected()
        session = await db_service.create_session(
            UserSessionCreate(user_id=user_profile.user_id)
        )

        # Return success response with session cookie (Feature 008: include background)
        response = JSONResponse(
            status_code=201,
            content={
                "success": True,
                "message": "User created successfully",
                "user": {
                    "id": user_profile.user_id,
                    "email": user_profile.email,
                    "name": user_profile.name,
                    # Software background
                    "softwareExperience": user_profile.software_experience,
                    "programmingLanguages": user_profile.programming_languages or [],
                    # Hardware background
                    "hardwareType": user_profile.hardware_type,
                    "hardwareExperience": user_profile.hardware_experience
                }
            }
        )

        # Set httpOnly session cookie
        response.set_cookie(
            key="session_token",
            value=str(session.session_id),
            httponly=True,
            secure=False,  # Set to True in production (HTTPS only)
            samesite="lax",
            max_age=30 * 24 * 60 * 60  # 30 days
        )

        return response
    except Exception as e:
        logger.error(f"Signup error: {str(e)}")
        raise HTTPException(status_code=400, detail=f"Signup failed: {str(e)}")


@router.post("/login")
async def login(request: Request, data: LoginRequest):
    """
    Handle user login requests from Better-Auth frontend.
    Validates credentials and returns user information with session cookie.
    """
    try:
        auth_service = await get_auth_service()

        # Query user by email
        user_profile = await auth_service.get_user_by_email(data.email)

        if not user_profile:
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "Invalid email or password"}
            )

        # Validate password using bcrypt
        if not auth_service.verify_password(data.password, user_profile.password_hash):
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "Invalid email or password"}
            )

        # Create session for this user
        await db_service.ensure_connected()
        session = await db_service.create_session(
            UserSessionCreate(user_id=user_profile.user_id)
        )

        # Return success response with session cookie (include background data!)
        response = JSONResponse(
            status_code=200,
            content={
                "success": True,
                "message": "Login successful",
                "user": {
                    "id": user_profile.user_id,
                    "email": user_profile.email,
                    "name": user_profile.name,
                    # Software background
                    "softwareExperience": user_profile.software_experience,
                    "programmingLanguages": user_profile.programming_languages or [],
                    # Hardware background
                    "hardwareType": user_profile.hardware_type,
                    "hardwareExperience": user_profile.hardware_experience
                }
            }
        )

        # Set httpOnly session cookie
        response.set_cookie(
            key="session_token",
            value=str(session.session_id),
            httponly=True,
            secure=False,  # Set to True in production
            samesite="lax",
            max_age=30 * 24 * 60 * 60  # 30 days
        )

        return response
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Login error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={"error": "InternalServerError", "message": "Login failed"}
        )


@router.get("/get-session")
async def get_session(request: Request):
    """
    Get current user session from cookie.
    Returns user profile + hardware details.
    FIXES 404 ERROR - this endpoint was missing!
    """
    try:
        # Extract session_token from cookie
        session_token = request.cookies.get("session_token")

        if not session_token:
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "No session token provided"}
            )

        # Parse UUID
        try:
            session_id = UUID(session_token)
        except ValueError:
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "Invalid session token"}
            )

        # Get session from database
        await db_service.ensure_connected()
        session = await db_service.get_session(session_id)

        if not session or not session.user_id:
            raise HTTPException(
                status_code=401,
                detail={"error": "Unauthorized", "message": "Session not found or not authenticated"}
            )

        # Get user profile
        auth_service = await get_auth_service()
        user_profile = await auth_service.get_user_profile(session.user_id)

        if not user_profile:
            raise HTTPException(
                status_code=404,
                detail={"error": "NotFound", "message": "User profile not found"}
            )

        # Return user + software/hardware profile (Feature 008) - do NOT include password!
        return JSONResponse(
            status_code=200,
            content={
                "success": True,
                "user": {
                    "id": user_profile.user_id,
                    "email": user_profile.email,
                    "name": user_profile.name,
                    # Software background
                    "softwareExperience": user_profile.software_experience,
                    "programmingLanguages": user_profile.programming_languages or [],
                    # Hardware background
                    "hardwareType": user_profile.hardware_type,
                    "hardwareExperience": user_profile.hardware_experience
                },
                "hardware_profile": {
                    "type": user_profile.hardware_type,
                    "details": user_profile.hardware_details.dict() if user_profile.hardware_details else None
                }
            }
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get session error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={"error": "InternalServerError", "message": "Failed to get session"}
        )


@router.get("/user/{user_id}")
async def get_user_profile(user_id: str):
    """
    Get user profile information by user ID.
    """
    try:
        auth_service = await get_auth_service()
        user_profile = await auth_service.get_user_profile(user_id)

        if not user_profile:
            raise HTTPException(status_code=404, detail="User not found")

        return JSONResponse(
            status_code=200,
            content={
                "user": {
                    "id": user_profile.user_id,
                    "email": user_profile.email,
                    "name": user_profile.name,
                    "hardware_type": user_profile.hardware_type,
                    "hardware_details": user_profile.hardware_details.dict() if user_profile.hardware_details else None,
                    "created_at": user_profile.created_at.isoformat(),
                    "updated_at": user_profile.updated_at.isoformat()
                }
            }
        )
    except Exception as e:
        logger.error(f"Get user profile error: {str(e)}")
        raise HTTPException(status_code=400, detail=f"Failed to get user profile: {str(e)}")


@router.put("/user/{user_id}")
async def update_user_profile(
    user_id: str,
    name: Optional[str] = None,
    hardware_type: Optional[str] = None,
    hardware_details: Optional[dict] = None
):
    """
    Update user profile information.
    """
    try:
        auth_service = await get_auth_service()

        profile_update = UserProfileUpdate(
            name=name,
            hardware_type=hardware_type,
            hardware_details=hardware_details
        )

        updated_profile = await auth_service.update_user_profile(user_id, profile_update)

        if not updated_profile:
            raise HTTPException(status_code=404, detail="User not found")

        return JSONResponse(
            status_code=200,
            content={
                "success": True,
                "message": "Profile updated successfully",
                "user": {
                    "id": updated_profile.user_id,
                    "email": updated_profile.email,
                    "name": updated_profile.name,
                    "hardware_type": updated_profile.hardware_type,
                    "hardware_details": updated_profile.hardware_details.dict() if updated_profile.hardware_details else None,
                    "updated_at": updated_profile.updated_at.isoformat()
                }
            }
        )
    except Exception as e:
        logger.error(f"Update user profile error: {str(e)}")
        raise HTTPException(status_code=400, detail=f"Failed to update user profile: {str(e)}")


@router.post("/logout")
async def logout(request: Request):
    """
    Handle user logout requests.
    Deletes session from database and clears cookie.
    """
    try:
        # Extract session_token from cookie
        session_token = request.cookies.get("session_token")

        if session_token:
            try:
                session_id = UUID(session_token)

                # Delete session from database
                await db_service.ensure_connected()
                await db_service.delete_session(session_id)

                logger.info(f"Session {session_id} deleted successfully")
            except ValueError:
                # Invalid UUID, just clear cookie
                pass
            except Exception as e:
                logger.error(f"Error deleting session: {str(e)}")
                # Continue to clear cookie even if delete fails

        # Return response with cleared cookie
        response = JSONResponse(
            status_code=200,
            content={
                "success": True,
                "message": "Logout successful"
            }
        )

        # Clear session cookie
        response.delete_cookie(key="session_token")

        return response
    except Exception as e:
        logger.error(f"Logout error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={"error": "InternalServerError", "message": "Logout failed"}
        )
