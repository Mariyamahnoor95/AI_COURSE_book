"""
Chat History Service

CRUD operations for chat_sessions and chat_messages tables (Neon Postgres).

Maps to: FR-018, FR-022
"""

import os
import uuid
from typing import List, Dict, Any, Optional
from datetime import datetime, timedelta
from dataclasses import dataclass
import psycopg2
from psycopg2.extras import RealDictCursor, Json
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


@dataclass
class ContextSelection:
    """Represents user's selected retrieval scope"""
    mode: str  # "full_textbook" | "module" | "chapter"
    selected_module_ids: List[str]
    selected_chapter_ids: List[str]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mode": self.mode,
            "selected_module_ids": self.selected_module_ids,
            "selected_chapter_ids": self.selected_chapter_ids
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ContextSelection":
        return cls(
            mode=data.get("mode", "full_textbook"),
            selected_module_ids=data.get("selected_module_ids", []),
            selected_chapter_ids=data.get("selected_chapter_ids", [])
        )


@dataclass
class ChatSession:
    """Represents a chat session"""
    session_id: str
    user_id: Optional[str]
    created_at: datetime
    updated_at: datetime
    expires_at: datetime
    context_selection: ContextSelection


@dataclass
class Citation:
    """Represents a citation in an assistant response"""
    chunk_id: str
    chapter_title: str
    heading: str
    url: str


@dataclass
class ChatMessage:
    """Represents a single chat message"""
    message_id: str
    session_id: str
    role: str  # "user" | "assistant"
    content: str
    timestamp: datetime
    citations: Optional[List[Citation]] = None
    response_time_ms: Optional[int] = None


class ChatHistoryService:
    """Service for managing chat sessions and messages in Neon Postgres"""

    def __init__(self, database_url: str | None = None):
        """
        Initialize chat history service

        Args:
            database_url: Postgres connection string (defaults to NEON_DATABASE_URL env var)
        """
        self.database_url = database_url or os.getenv("NEON_DATABASE_URL")
        if not self.database_url:
            raise ValueError(
                "Database URL required. Set NEON_DATABASE_URL environment variable "
                "or pass database_url parameter."
            )

    def _get_connection(self):
        """Get database connection"""
        return psycopg2.connect(self.database_url, cursor_factory=RealDictCursor)

    def create_session(
        self,
        context_selection: ContextSelection,
        user_id: Optional[str] = None
    ) -> ChatSession:
        """
        Create new chat session

        Args:
            context_selection: User's retrieval context settings
            user_id: Optional user identifier

        Returns:
            Created ChatSession object
        """
        session_id = str(uuid.uuid4())
        now = datetime.utcnow()
        expires_at = now + timedelta(hours=24)

        conn = self._get_connection()
        try:
            with conn.cursor() as cursor:
                cursor.execute("""
                    INSERT INTO chat_sessions (
                        session_id, user_id, created_at, updated_at,
                        expires_at, context_selection
                    ) VALUES (%s, %s, %s, %s, %s, %s)
                    RETURNING *
                """, (
                    session_id,
                    user_id,
                    now,
                    now,
                    expires_at,
                    Json(context_selection.to_dict())
                ))
                row = cursor.fetchone()
                conn.commit()

            return ChatSession(
                session_id=row['session_id'],
                user_id=row['user_id'],
                created_at=row['created_at'],
                updated_at=row['updated_at'],
                expires_at=row['expires_at'],
                context_selection=ContextSelection.from_dict(row['context_selection'])
            )
        finally:
            conn.close()

    def get_session(self, session_id: str) -> Optional[ChatSession]:
        """
        Get session by ID

        Args:
            session_id: Session UUID

        Returns:
            ChatSession or None if not found or expired
        """
        conn = self._get_connection()
        try:
            with conn.cursor() as cursor:
                cursor.execute("""
                    SELECT * FROM chat_sessions
                    WHERE session_id = %s
                    AND expires_at > NOW()
                """, (session_id,))
                row = cursor.fetchone()

            if not row:
                return None

            return ChatSession(
                session_id=row['session_id'],
                user_id=row['user_id'],
                created_at=row['created_at'],
                updated_at=row['updated_at'],
                expires_at=row['expires_at'],
                context_selection=ContextSelection.from_dict(row['context_selection'])
            )
        finally:
            conn.close()

    def update_session_context(
        self,
        session_id: str,
        context_selection: ContextSelection
    ) -> None:
        """
        Update session's context selection

        Args:
            session_id: Session UUID
            context_selection: New context settings
        """
        conn = self._get_connection()
        try:
            with conn.cursor() as cursor:
                cursor.execute("""
                    UPDATE chat_sessions
                    SET context_selection = %s
                    WHERE session_id = %s
                """, (Json(context_selection.to_dict()), session_id))
                conn.commit()
        finally:
            conn.close()

    def add_message(
        self,
        session_id: str,
        role: str,
        content: str,
        citations: Optional[List[Citation]] = None,
        response_time_ms: Optional[int] = None
    ) -> ChatMessage:
        """
        Add message to session

        Args:
            session_id: Session UUID
            role: "user" or "assistant"
            content: Message text
            citations: Citations for assistant messages
            response_time_ms: Response time for assistant messages

        Returns:
            Created ChatMessage

        Raises:
            ValueError: If role is invalid or content length exceeds limits
        """
        if role not in ("user", "assistant"):
            raise ValueError(f"Invalid role: {role}. Must be 'user' or 'assistant'")

        # Validate content length
        if role == "user" and len(content) > 500:
            raise ValueError(f"User message exceeds 500 character limit: {len(content)}")
        if role == "assistant" and len(content) > 10000:
            raise ValueError(f"Assistant message exceeds 10000 character limit: {len(content)}")

        message_id = str(uuid.uuid4())
        timestamp = datetime.utcnow()

        # Convert citations to JSON
        # For assistant messages, always use a JSON array (empty if no citations)
        # For user messages, use NULL
        if role == "assistant":
            if citations:
                citations_json = Json([
                    {
                        "chunk_id": c.chunk_id,
                        "chapter_title": c.chapter_title,
                        "heading": c.heading,
                        "url": c.url
                    }
                    for c in citations
                ])
            else:
                citations_json = Json([])  # Empty array instead of NULL
        else:
            citations_json = None  # NULL for user messages

        conn = self._get_connection()
        try:
            with conn.cursor() as cursor:
                cursor.execute("""
                    INSERT INTO chat_messages (
                        message_id, session_id, role, content,
                        timestamp, citations, response_time_ms
                    ) VALUES (%s, %s, %s, %s, %s, %s, %s)
                    RETURNING *
                """, (
                    message_id,
                    session_id,
                    role,
                    content,
                    timestamp,
                    citations_json,
                    response_time_ms
                ))
                row = cursor.fetchone()
                conn.commit()

            # Parse citations
            parsed_citations = None
            if row['citations']:
                parsed_citations = [
                    Citation(**c) for c in row['citations']
                ]

            return ChatMessage(
                message_id=row['message_id'],
                session_id=row['session_id'],
                role=row['role'],
                content=row['content'],
                timestamp=row['timestamp'],
                citations=parsed_citations,
                response_time_ms=row['response_time_ms']
            )
        finally:
            conn.close()

    def get_session_history(
        self,
        session_id: str,
        limit: int = 10
    ) -> List[ChatMessage]:
        """
        Get recent messages for session

        Args:
            session_id: Session UUID
            limit: Number of recent messages to return (default: 10)

        Returns:
            List of ChatMessage objects, ordered by timestamp (oldest first)
        """
        conn = self._get_connection()
        try:
            with conn.cursor() as cursor:
                cursor.execute("""
                    SELECT * FROM chat_messages
                    WHERE session_id = %s
                    ORDER BY timestamp DESC
                    LIMIT %s
                """, (session_id, limit))
                rows = cursor.fetchall()

            # Reverse to get oldest first
            rows = list(reversed(rows))

            messages = []
            for row in rows:
                parsed_citations = None
                if row['citations']:
                    parsed_citations = [
                        Citation(**c) for c in row['citations']
                    ]

                messages.append(ChatMessage(
                    message_id=row['message_id'],
                    session_id=row['session_id'],
                    role=row['role'],
                    content=row['content'],
                    timestamp=row['timestamp'],
                    citations=parsed_citations,
                    response_time_ms=row['response_time_ms']
                ))

            return messages
        finally:
            conn.close()

    def cleanup_expired_sessions(self) -> int:
        """
        Delete expired sessions and their messages

        Returns:
            Number of sessions deleted
        """
        conn = self._get_connection()
        try:
            with conn.cursor() as cursor:
                cursor.execute("""
                    DELETE FROM chat_sessions
                    WHERE expires_at < NOW()
                    RETURNING session_id
                """)
                deleted = cursor.fetchall()
                conn.commit()
            return len(deleted)
        finally:
            conn.close()


# Convenience functions for direct usage
_default_service = None


def get_chat_history_service() -> ChatHistoryService:
    """Get or create default chat history service instance"""
    global _default_service
    if _default_service is None:
        _default_service = ChatHistoryService()
    return _default_service
