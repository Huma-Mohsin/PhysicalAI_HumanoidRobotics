"""
Database service for CRUD operations on user_sessions, conversations, and messages.
Uses asyncpg for async PostgreSQL operations.
"""

import asyncpg
import json
from typing import List, Optional
from uuid import UUID
from datetime import datetime

from utils.config import settings
from utils.logger import logger
from models.user_session import UserSession, UserSessionCreate, HardwareProfile
from models.conversation import Conversation, ConversationCreate
from models.message import Message, MessageCreate


class DatabaseService:
    """Database service for RAG chatbot."""

    def __init__(self):
        """Initialize database service."""
        self.pool: Optional[asyncpg.Pool] = None

    async def connect(self):
        """Create connection pool to Neon Postgres."""
        if self.pool is None:
            try:
                self.pool = await asyncpg.create_pool(
                    settings.neon_database_url,
                    min_size=2,
                    max_size=10,
                    command_timeout=60
                )
                logger.info("✅ Database connection pool created")
            except Exception as e:
                logger.error(f"❌ Failed to connect to database: {str(e)}")
                raise

    async def disconnect(self):
        """Close connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("Database connection pool closed")

    # User Session operations

    async def create_session(self, session_data: UserSessionCreate) -> UserSession:
        """Create a new user session."""
        async with self.pool.acquire() as conn:
            hardware_profile_json = json.dumps(session_data.hardware_profile.dict()) if session_data.hardware_profile else None

            if session_data.session_id:
                # Use provided session_id
                row = await conn.fetchrow(
                    """
                    INSERT INTO user_sessions (session_id, user_id, hardware_profile, created_at, last_active)
                    VALUES ($1, $2, $3, NOW(), NOW())
                    RETURNING session_id, user_id, hardware_profile, created_at, last_active
                    """,
                    session_data.session_id,
                    session_data.user_id,
                    hardware_profile_json
                )
            else:
                # Let database generate session_id
                row = await conn.fetchrow(
                    """
                    INSERT INTO user_sessions (user_id, hardware_profile, created_at, last_active)
                    VALUES ($1, $2, NOW(), NOW())
                    RETURNING session_id, user_id, hardware_profile, created_at, last_active
                    """,
                    session_data.user_id,
                    hardware_profile_json
                )

            return self._row_to_user_session(row)

    async def get_session(self, session_id: UUID) -> Optional[UserSession]:
        """Fetch a user session by ID."""
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM user_sessions WHERE session_id = $1",
                session_id
            )

            return self._row_to_user_session(row) if row else None

    async def update_session_activity(self, session_id: UUID):
        """Update last_active timestamp for a session."""
        async with self.pool.acquire() as conn:
            await conn.execute(
                "UPDATE user_sessions SET last_active = NOW() WHERE session_id = $1",
                session_id
            )

    # Conversation operations

    async def create_conversation(self, conv_data: ConversationCreate) -> Conversation:
        """Create a new conversation."""
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                INSERT INTO conversations (session_id, title, created_at, updated_at)
                VALUES ($1, $2, NOW(), NOW())
                RETURNING conv_id, session_id, title, summary, created_at, updated_at
                """,
                conv_data.session_id,
                conv_data.title
            )

            return self._row_to_conversation(row)

    async def get_conversation(self, conv_id: UUID) -> Optional[Conversation]:
        """Fetch a conversation by ID."""
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM conversations WHERE conv_id = $1",
                conv_id
            )

            return self._row_to_conversation(row) if row else None

    async def get_conversations_by_session(
        self, session_id: UUID, limit: int = 20, offset: int = 0
    ) -> List[Conversation]:
        """Fetch conversations for a session."""
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT * FROM conversations
                WHERE session_id = $1
                ORDER BY updated_at DESC
                LIMIT $2 OFFSET $3
                """,
                session_id, limit, offset
            )

            return [self._row_to_conversation(row) for row in rows]

    async def update_conversation_title(self, conv_id: UUID, title: str):
        """Update conversation title."""
        async with self.pool.acquire() as conn:
            await conn.execute(
                "UPDATE conversations SET title = $1 WHERE conv_id = $2",
                title, conv_id
            )

    # Message operations

    async def create_message(self, message_data: MessageCreate) -> Message:
        """Create a new message."""
        async with self.pool.acquire() as conn:
            text_selection_json = json.dumps(message_data.text_selection.dict()) if message_data.text_selection else None
            metadata_json = json.dumps(message_data.metadata.dict()) if message_data.metadata else None

            row = await conn.fetchrow(
                """
                INSERT INTO messages (conv_id, role, content, text_selection, metadata, created_at)
                VALUES ($1, $2, $3, $4, $5, NOW())
                RETURNING message_id, conv_id, role, content, text_selection, metadata, created_at
                """,
                message_data.conv_id,
                message_data.role,
                message_data.content,
                text_selection_json,
                metadata_json
            )

            return self._row_to_message(row)

    async def get_messages_by_conversation(
        self, conv_id: UUID, limit: int = 50
    ) -> List[Message]:
        """Fetch messages for a conversation."""
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT * FROM messages
                WHERE conv_id = $1
                ORDER BY created_at ASC
                LIMIT $2
                """,
                conv_id, limit
            )

            return [self._row_to_message(row) for row in rows]

    async def get_recent_messages(self, conv_id: UUID, limit: int = 5) -> List[Message]:
        """Fetch recent messages for conversation context."""
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT * FROM messages
                WHERE conv_id = $1
                ORDER BY created_at DESC
                LIMIT $2
                """,
                conv_id, limit
            )

            # Reverse to get chronological order
            return [self._row_to_message(row) for row in reversed(rows)]

    # Helper methods to convert database rows to Pydantic models

    def _row_to_user_session(self, row) -> UserSession:
        """Convert database row to UserSession model."""
        hardware_profile = None
        if row["hardware_profile"]:
            hardware_profile = HardwareProfile(**row["hardware_profile"])

        return UserSession(
            session_id=row["session_id"],
            user_id=row["user_id"],
            hardware_profile=hardware_profile,
            created_at=row["created_at"],
            last_active=row["last_active"]
        )

    def _row_to_conversation(self, row) -> Conversation:
        """Convert database row to Conversation model."""
        return Conversation(
            conv_id=row["conv_id"],
            session_id=row["session_id"],
            title=row["title"],
            summary=row["summary"],
            created_at=row["created_at"],
            updated_at=row["updated_at"]
        )

    def _row_to_message(self, row) -> Message:
        """Convert database row to Message model."""
        from models.message import TextSelection, MessageMetadata

        text_selection = None
        if row["text_selection"]:
            # Parse JSON if it's a string
            text_selection_data = row["text_selection"]
            if isinstance(text_selection_data, str):
                import json
                text_selection_data = json.loads(text_selection_data)
            text_selection = TextSelection(**text_selection_data)

        metadata = None
        if row["metadata"]:
            # Parse JSON if it's a string
            metadata_data = row["metadata"]
            if isinstance(metadata_data, str):
                import json
                metadata_data = json.loads(metadata_data)
            metadata = MessageMetadata(**metadata_data)

        return Message(
            message_id=row["message_id"],
            conv_id=row["conv_id"],
            role=row["role"],
            content=row["content"],
            text_selection=text_selection,
            metadata=metadata,
            created_at=row["created_at"]
        )


# Global database service instance
db_service = DatabaseService()
