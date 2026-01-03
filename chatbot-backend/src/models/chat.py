"""
Chat session and message data models

Maps to: FR-018, FR-022
Per: specs/001-physical-ai-robotics-textbook/data-model.md
"""

from datetime import datetime, timedelta
from typing import List, Optional, Literal
from pydantic import BaseModel, Field
import uuid


class Citation(BaseModel):
    """
    Source citation for assistant responses

    Links chatbot answers back to specific textbook sections
    """
    chunk_id: str = Field(..., description="UUID of source content chunk")
    chapter_title: str = Field(..., description="Human-readable chapter name")
    heading: str = Field(..., description="Section heading within chapter")
    url: str = Field(..., description="Full URL with anchor (e.g., /docs/module-01/ch01#section)")
    chunk_preview: Optional[str] = Field(None, description="Brief excerpt from chunk (max 200 chars)")


class ContextSelection(BaseModel):
    """
    User's selected retrieval scope for chatbot queries

    Determines which textbook content is searched during RAG
    """
    mode: Literal["full_textbook", "module", "chapter"] = Field(
        ...,
        description="Retrieval scope: full textbook, specific module(s), or specific chapter(s)"
    )
    selected_module_ids: List[str] = Field(
        default_factory=list,
        description="Module IDs for filtering (e.g., ['module-01-ros2'])"
    )
    selected_chapter_ids: List[str] = Field(
        default_factory=list,
        description="Chapter IDs for filtering (e.g., ['module-01-ros2/week-03/ch01-nodes-topics'])"
    )

    def validate_selection(self) -> bool:
        """Ensure context selection is consistent with mode"""
        if self.mode == "full_textbook":
            return len(self.selected_module_ids) == 0 and len(self.selected_chapter_ids) == 0
        elif self.mode == "module":
            return len(self.selected_module_ids) > 0 and len(self.selected_chapter_ids) == 0
        elif self.mode == "chapter":
            return len(self.selected_module_ids) > 0 and len(self.selected_chapter_ids) > 0
        return False


class ChatSession(BaseModel):
    """
    User conversation session with the chatbot

    Manages:
    - Session lifecycle (24-hour expiration)
    - Context selection (which textbook sections to query)
    - Anonymous or authenticated users
    """
    session_id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Unique session identifier"
    )
    user_id: Optional[str] = Field(
        None,
        description="User identifier if authenticated (null for anonymous)"
    )
    created_at: datetime = Field(
        default_factory=datetime.utcnow,
        description="Session creation timestamp (UTC)"
    )
    updated_at: datetime = Field(
        default_factory=datetime.utcnow,
        description="Last activity timestamp (UTC)"
    )
    expires_at: datetime = Field(
        default_factory=lambda: datetime.utcnow() + timedelta(hours=24),
        description="Expiration timestamp (24 hours after updated_at)"
    )
    context_selection: ContextSelection = Field(
        default_factory=lambda: ContextSelection(mode="full_textbook"),
        description="User's selected retrieval scope"
    )

    def is_expired(self) -> bool:
        """Check if session has expired"""
        return datetime.utcnow() > self.expires_at

    def update_activity(self):
        """Update timestamps when user interacts"""
        self.updated_at = datetime.utcnow()
        self.expires_at = self.updated_at + timedelta(hours=24)


class ChatMessage(BaseModel):
    """
    Single message in a chat conversation

    Can be:
    - User query: role="user", no citations
    - Assistant response: role="assistant", includes citations and response_time
    """
    message_id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Unique message identifier"
    )
    session_id: str = Field(..., description="Parent session reference")
    role: Literal["user", "assistant"] = Field(..., description="Message origin")
    content: str = Field(
        ...,
        description="Message text (max 500 chars for user, 10000 for assistant)",
        max_length=10000
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Message creation time (UTC)"
    )
    citations: Optional[List[Citation]] = Field(
        None,
        description="Source citations (only for assistant messages)"
    )
    response_time_ms: Optional[int] = Field(
        None,
        description="Time to generate response in milliseconds (only for assistant)"
    )

    def validate_message(self) -> bool:
        """Ensure message fields are consistent with role"""
        if self.role == "user":
            # User messages should not have citations or response time
            return self.citations is None and self.response_time_ms is None and len(self.content) <= 500
        elif self.role == "assistant":
            # Assistant messages should have citations and response time
            return self.citations is not None and self.response_time_ms is not None
        return False
