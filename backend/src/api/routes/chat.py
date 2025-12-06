"""
Chat API endpoint for RAG queries
"""

from fastapi import APIRouter, HTTPException, status
from fastapi.responses import StreamingResponse
from typing import Optional
import logging
from datetime import datetime

from ...schemas.chat import ChatRequest, ChatResponse, RetrievedChunk
from ...services.rag_service import RAGService
from ...db.neon import get_async_session
from ...models.chat import ChatMessage as ChatMessageModel

logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize RAG service
try:
    rag_service = RAGService()
except Exception as e:
    logger.error(f"Failed to initialize RAG service: {e}")
    rag_service = None


@router.post("", response_model=ChatResponse, status_code=status.HTTP_200_OK)
async def chat_query(request: ChatRequest):
    """
    Process a chat query using RAG pipeline

    Workflow:
    1. Retrieve relevant content chunks from Qdrant
    2. Generate AI response with OpenAI
    3. Optionally persist chat history to database
    4. Return response with context metadata

    Args:
        request: ChatRequest with query, language, optional context

    Returns:
        ChatResponse with AI answer and retrieved chunks
    """
    if not rag_service:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="RAG service is not available. Check OpenAI API key configuration."
        )

    try:
        # Convert Pydantic chat history to dict format
        chat_history = None
        if request.chat_history:
            chat_history = [
                {"role": msg.role, "content": msg.content}
                for msg in request.chat_history
            ]

        # Process query through RAG pipeline
        result = await rag_service.process_query(
            query=request.query,
            language=request.language,
            selected_text=request.selected_text,
            chat_history=chat_history,
            stream=False  # Non-streaming for now
        )

        # Transform retrieved chunks to response format
        retrieved_chunks = [
            RetrievedChunk(
                id=str(chunk.get("id", "unknown")),
                score=chunk.get("score", 0.0),
                content=chunk.get("payload", {}).get("content", ""),
                metadata=chunk.get("payload", {}).get("metadata", {})
            )
            for chunk in result["retrieved_chunks"]
        ]

        # Optionally persist to database if user is authenticated
        if request.user_id:
            try:
                async with get_async_session() as session:
                    chat_message = ChatMessageModel(
                        user_id=request.user_id,
                        query=request.query,
                        response=result["response"],
                        selected_text=request.selected_text,
                        language=request.language
                    )
                    session.add(chat_message)
                    await session.commit()
            except Exception as db_error:
                logger.warning(f"Failed to persist chat message: {db_error}")
                # Don't fail the request if persistence fails

        return ChatResponse(
            response=result["response"],
            retrieved_chunks=retrieved_chunks,
            num_chunks=result["num_chunks"],
            language=request.language,
            timestamp=datetime.utcnow()
        )

    except ValueError as ve:
        logger.error(f"Validation error in chat query: {ve}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(ve)
        )
    except Exception as e:
        logger.error(f"Error processing chat query: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to process chat query. Please try again later."
        )


@router.get("/health", status_code=status.HTTP_200_OK)
async def chat_health():
    """
    Health check for chat service

    Returns:
        Service status and availability
    """
    return {
        "status": "healthy" if rag_service else "degraded",
        "rag_service": rag_service is not None,
        "timestamp": datetime.utcnow().isoformat()
    }
