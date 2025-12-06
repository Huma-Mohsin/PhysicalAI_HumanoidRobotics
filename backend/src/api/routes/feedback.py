"""
Feedback API endpoints
"""

from fastapi import APIRouter, HTTPException, status, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, func
import logging
from typing import Optional

from ...schemas.feedback import FeedbackRequest, FeedbackResponse, FeedbackStats
from ...models.feedback import Feedback
from ...models.chat import ChatMessage
from ...db.neon import get_async_session
from ...core.security import get_current_user_id_optional

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post("", response_model=FeedbackResponse, status_code=status.HTTP_201_CREATED)
async def submit_feedback(
    request: FeedbackRequest,
    session: AsyncSession = Depends(get_async_session),
    user_id: Optional[str] = Depends(get_current_user_id_optional)
):
    """
    Submit feedback for a chat message

    Args:
        request: FeedbackRequest with message_id, rating, and optional comment

    Returns:
        FeedbackResponse with created feedback
    """
    # Verify message exists
    result = await session.execute(
        select(ChatMessage).where(ChatMessage.id == request.message_id)
    )
    message = result.scalar_one_or_none()

    if not message:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Chat message not found"
        )

    # Check if feedback already exists for this message
    existing_feedback = await session.execute(
        select(Feedback).where(Feedback.message_id == request.message_id)
    )
    feedback = existing_feedback.scalar_one_or_none()

    try:
        if feedback:
            # Update existing feedback
            feedback.rating = request.rating
            feedback.comment = request.comment
        else:
            # Create new feedback
            feedback = Feedback(
                message_id=request.message_id,
                rating=request.rating,
                comment=request.comment
            )
            session.add(feedback)

        await session.commit()
        await session.refresh(feedback)

        return FeedbackResponse(
            id=str(feedback.id),
            message_id=str(feedback.message_id),
            rating=feedback.rating,
            comment=feedback.comment,
            created_at=feedback.created_at
        )

    except Exception as e:
        await session.rollback()
        logger.error(f"Feedback submission error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to submit feedback"
        )


@router.get("/stats", response_model=FeedbackStats)
async def get_feedback_stats(
    session: AsyncSession = Depends(get_async_session)
):
    """
    Get aggregated feedback statistics

    Returns:
        FeedbackStats with overall metrics
    """
    try:
        # Total feedback count
        total_result = await session.execute(
            select(func.count(Feedback.id))
        )
        total_feedback = total_result.scalar() or 0

        # Positive feedback count (rating = 1)
        positive_result = await session.execute(
            select(func.count(Feedback.id)).where(Feedback.rating == 1)
        )
        positive_count = positive_result.scalar() or 0

        # Negative feedback count (rating = -1)
        negative_result = await session.execute(
            select(func.count(Feedback.id)).where(Feedback.rating == -1)
        )
        negative_count = negative_result.scalar() or 0

        # Average rating
        avg_result = await session.execute(
            select(func.avg(Feedback.rating))
        )
        average_rating = float(avg_result.scalar() or 0.0)

        # Recent comments (last 5)
        comments_result = await session.execute(
            select(Feedback.comment)
            .where(Feedback.comment.isnot(None))
            .order_by(Feedback.created_at.desc())
            .limit(5)
        )
        recent_comments = [c for c in comments_result.scalars() if c]

        # Calculate percentage
        positive_percentage = (
            (positive_count / total_feedback * 100) if total_feedback > 0 else 0.0
        )

        return FeedbackStats(
            total_feedback=total_feedback,
            positive_count=positive_count,
            negative_count=negative_count,
            positive_percentage=round(positive_percentage, 1),
            average_rating=round(average_rating, 2),
            recent_comments=recent_comments
        )

    except Exception as e:
        logger.error(f"Error fetching feedback stats: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to fetch feedback statistics"
        )


@router.delete("/{message_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_feedback(
    message_id: str,
    session: AsyncSession = Depends(get_async_session),
    user_id: str = Depends(get_current_user_id_optional)
):
    """
    Delete feedback for a message

    Args:
        message_id: Chat message UUID

    Returns:
        204 No Content on success
    """
    result = await session.execute(
        select(Feedback).where(Feedback.message_id == message_id)
    )
    feedback = result.scalar_one_or_none()

    if not feedback:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Feedback not found"
        )

    try:
        await session.delete(feedback)
        await session.commit()
    except Exception as e:
        await session.rollback()
        logger.error(f"Error deleting feedback: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete feedback"
        )
