"""
Content chunk data models for RAG system

Maps to: FR-017
Per: specs/001-physical-ai-robotics-textbook/data-model.md
"""

from typing import List, Dict, Any
from pydantic import BaseModel, Field
import uuid


class ChunkMetadata(BaseModel):
    """
    Structured metadata for content chunk retrieval and citation

    Enables:
    - Filtering by module/week/chapter during RAG queries
    - Generating clickable citations with chapter title + heading + URL
    """
    module: str = Field(..., description="Module ID (e.g., 'module-01-ros2')")
    week: int = Field(..., description="Week number (1-13)", ge=1, le=13)
    chapter_title: str = Field(..., description="Human-readable chapter name")
    heading: str = Field(..., description="Section heading (H2/H3 level)")
    page_url: str = Field(
        ...,
        description="Full URL path (e.g., '/docs/module-01-ros2/week-03/ch01-nodes-topics')"
    )
    heading_anchor: str = Field(
        ...,
        description="Docusaurus auto-generated anchor (e.g., '#creating-a-publisher-node')"
    )

    def get_full_url(self, base_url: str = "") -> str:
        """Generate full clickable URL with anchor"""
        return f"{base_url}{self.page_url}{self.heading_anchor}"


class ContentChunk(BaseModel):
    """
    Semantic chunk of textbook content for RAG retrieval

    Generated during content ingestion from Markdown files
    Stored in Qdrant vector database with embeddings
    """
    id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Unique chunk identifier (UUID v4)"
    )
    chapter_id: str = Field(
        ...,
        description="Parent chapter reference (e.g., 'module-01-ros2/week-03/ch01-nodes-topics')"
    )
    content_text: str = Field(
        ...,
        description="Raw text content (400-600 tokens typically)",
        min_length=50,
        max_length=10000
    )
    embedding_vector: List[float] = Field(
        ...,
        description="OpenAI text-embedding-3-small vector (1536 dimensions)"
    )
    token_count: int = Field(
        ...,
        description="Number of tokens in content_text",
        ge=100,
        le=800
    )
    chunk_order: int = Field(
        ...,
        description="Position within chapter (0-indexed)",
        ge=0
    )
    metadata: ChunkMetadata = Field(..., description="Structured metadata for retrieval")

    def validate_embedding(self) -> bool:
        """Ensure embedding vector has correct dimensions"""
        return len(self.embedding_vector) == 1536

    def to_dict_for_qdrant(self) -> Dict[str, Any]:
        """Convert to Qdrant storage format"""
        return {
            "id": self.id,
            "vector": self.embedding_vector,
            "payload": {
                "chapter_id": self.chapter_id,
                "content_text": self.content_text,
                "token_count": self.token_count,
                "chunk_order": self.chunk_order,
                "metadata": self.metadata.dict()
            }
        }

    @classmethod
    def from_qdrant_result(cls, qdrant_point) -> 'ContentChunk':
        """Create ContentChunk from Qdrant search result"""
        return cls(
            id=qdrant_point.id,
            chapter_id=qdrant_point.payload["chapter_id"],
            content_text=qdrant_point.payload["content_text"],
            embedding_vector=qdrant_point.vector if hasattr(qdrant_point, 'vector') else [],
            token_count=qdrant_point.payload["token_count"],
            chunk_order=qdrant_point.payload["chunk_order"],
            metadata=ChunkMetadata(**qdrant_point.payload["metadata"])
        )
