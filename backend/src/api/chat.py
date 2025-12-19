"""
Chat API endpoints for RAG chatbot.
Implements POST /api/chat/query endpoint.
"""

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import Optional
from uuid import UUID
from datetime import datetime

from src.services.database_service import db_service
from src.services.rag_service import rag_service
from src.services.auth_service import get_auth_service
from src.models.user_session import UserSessionCreate
from src.models.conversation import ConversationCreate
from src.models.message import MessageCreate, Message, MessageMetadata, TextSelection
from src.utils.logger import logger


# API Router
router = APIRouter(prefix="/api/chat", tags=["chat"])


# Request/Response Models
class ChatQueryRequest(BaseModel):
    """Request body for chat query."""
    question: str = Field(..., min_length=1, max_length=2000, description="User's question")
    conversation_id: Optional[UUID] = Field(default=None, description="Existing conversation ID")
    session_id: Optional[UUID] = Field(default=None, description="User session ID")
    user_id: Optional[str] = Field(default=None, description="Authenticated user ID from Better-Auth")
    text_selection: Optional[TextSelection] = Field(default=None, description="User-highlighted text context")


class ChunkMetadata(BaseModel):
    """Metadata for retrieved chunks."""
    chunk_id: str
    chapter_id: str
    chapter_title: str
    section_title: str
    similarity_score: float
    excerpt: str


class ResponseMetadata(BaseModel):
    """Response metadata."""
    latency_ms: int
    tokens_used: int
    model: str
    qdrant_query_ms: int
    retrieved_count: int
    embedding_model: str


class ChatQueryResponse(BaseModel):
    """Response body for chat query."""
    message_id: UUID
    conversation_id: UUID
    session_id: UUID
    response: str
    retrieved_chunks: list[ChunkMetadata]
    metadata: ResponseMetadata
    timestamp: datetime


@router.post("/query", response_model=ChatQueryResponse)
async def chat_query(request: ChatQueryRequest):
    """
    Handle chat query (general Q&A about book content).

    Implements the RAG pipeline:
    1. Session resolution (create or fetch)
    2. Conversation resolution (create or fetch + history)
    3. RAG execution (embed, search, generate)
    4. Message storage (user question + assistant response)
    """
    try:
        # Input validation (already handled by Pydantic, but add sanitization)
        question = request.question.strip()

        if not question:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={"error": "ValidationError", "message": "Question cannot be empty", "field": "question"}
            )

        logger.info(f"Chat query: {question[:100]}...")

        # ====================
        # T021: Session Resolution
        # ====================
        session_id = request.session_id

        if session_id:
            # Fetch existing session
            session = await db_service.get_session(session_id)
            if not session:
                # Session doesn't exist - create it with the provided session_id
                # This handles anonymous users who generate session_id in browser
                session = await db_service.create_session(
                    UserSessionCreate(session_id=session_id)
                )
                logger.info(f"Created new session with provided ID: {session_id}")
            else:
                # Update last_active
                await db_service.update_session_activity(session_id)

        else:
            # Create new anonymous session
            session = await db_service.create_session(UserSessionCreate())
            session_id = session.session_id
            logger.info(f"Created new session: {session_id}")

        # ====================
        # T022: Conversation Resolution
        # ====================
        conversation_id = request.conversation_id
        conversation_history = []

        if conversation_id:
            # Fetch existing conversation
            conversation = await db_service.get_conversation(conversation_id)
            if not conversation:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail={"error": "NotFoundError", "message": f"Conversation ID '{conversation_id}' not found", "field": "conversation_id"}
                )

            # Fetch last 5 messages for context
            messages = await db_service.get_recent_messages(conversation_id, limit=5)
            conversation_history = [
                {"role": msg.role, "content": msg.content}
                for msg in messages
            ]

        else:
            # Create new conversation
            conversation = await db_service.create_conversation(
                ConversationCreate(session_id=session_id)
            )
            conversation_id = conversation.conv_id
            logger.info(f"Created new conversation: {conversation_id}")

        # ====================
        # T023: RAG Pipeline Execution (Feature 008: Software + Hardware Personalization)
        # ====================
        # Get personalization data from authenticated user
        hardware_profile = None
        software_experience = None
        programming_languages = None

        if request.user_id:
            try:
                auth_service = await get_auth_service()
                # Get full user profile with software + hardware background
                user_profile = await auth_service.get_user_profile(request.user_id)
                if user_profile:
                    # Hardware personalization
                    hardware_profile = user_profile.hardware_type
                    # Software personalization (Feature 008)
                    software_experience = user_profile.software_experience
                    programming_languages = user_profile.programming_languages
                    logger.info(f"Using personalization from authenticated user: hardware={hardware_profile}, experience={software_experience}")
            except Exception as e:
                logger.error(f"Error fetching user profile for personalization: {str(e)}")
                # Fallback to session hardware profile if available
                hardware_profile = session.hardware_profile.type if session.hardware_profile else None
        else:
            # For anonymous users, use session hardware profile if available
            hardware_profile = session.hardware_profile.type if session.hardware_profile else None

        rag_result = rag_service.query_book(
            question=question,
            conversation_history=conversation_history,
            hardware_profile=hardware_profile,
            software_experience=software_experience,
            programming_languages=programming_languages
        )

        response_text = rag_result["response"]
        retrieved_chunks = rag_result["retrieved_chunks"]
        rag_metadata = rag_result["metadata"]

        # ====================
        # Save Messages
        # ====================
        # Save user message (with optional text selection)
        user_message = await db_service.create_message(
            MessageCreate(
                conv_id=conversation_id,
                role="user",
                content=question,
                text_selection=request.text_selection  # Include highlighted text context
            )
        )

        # Save assistant response
        assistant_message = await db_service.create_message(
            MessageCreate(
                conv_id=conversation_id,
                role="assistant",
                content=response_text,
                metadata=rag_metadata
            )
        )

        # Auto-generate conversation title from first question
        if not conversation.title and len(conversation_history) == 0:
            # First message - generate title
            title = question[:200] if len(question) <= 200 else question[:197] + "..."
            await db_service.update_conversation_title(conversation_id, title)

        # ====================
        # Build Response
        # ====================
        chunks_response = [
            ChunkMetadata(
                chunk_id=chunk["chunk_id"],
                chapter_id=chunk["chapter_id"],
                chapter_title=chunk["chapter_title"],
                section_title=chunk.get("section_title", ""),
                similarity_score=chunk["similarity_score"],
                excerpt=chunk["excerpt"]
            )
            for chunk in retrieved_chunks
        ]

        metadata_response = ResponseMetadata(
            latency_ms=rag_metadata.latency_ms,
            tokens_used=rag_metadata.tokens_used,
            model=rag_metadata.model,
            qdrant_query_ms=rag_metadata.qdrant_query_ms,
            retrieved_count=rag_metadata.retrieved_chunks,
            embedding_model=rag_metadata.embedding_model
        )

        return ChatQueryResponse(
            message_id=assistant_message.message_id,
            conversation_id=conversation_id,
            session_id=session_id,
            response=response_text,
            retrieved_chunks=chunks_response,
            metadata=metadata_response,
            timestamp=datetime.utcnow()
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Chat query failed: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "InternalServerError", "message": "Failed to generate response. Please try again later"}
        )
