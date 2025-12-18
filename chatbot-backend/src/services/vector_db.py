"""
Vector Database Client (Qdrant)

Implements insert, search, and delete operations for Qdrant collection.
Collection name: "textbook_chunks" with 1536-dimensional vectors.

Maps to: FR-017
"""

import os
import uuid
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
    SearchRequest
)
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


@dataclass
class SearchResult:
    """Represents a vector search result"""
    chunk_id: str
    score: float
    content_text: str
    metadata: Dict[str, Any]


class VectorDBClient:
    """Qdrant vector database client for textbook content chunks"""

    COLLECTION_NAME = "textbook_chunks"
    VECTOR_SIZE = 1536
    DISTANCE_METRIC = Distance.COSINE

    def __init__(
        self,
        url: str | None = None,
        api_key: str | None = None
    ):
        """
        Initialize Qdrant client

        Args:
            url: Qdrant server URL (defaults to QDRANT_URL env var)
            api_key: Qdrant API key (defaults to QDRANT_API_KEY env var)
        """
        self.url = url or os.getenv("QDRANT_URL")
        self.api_key = api_key or os.getenv("QDRANT_API_KEY")

        if not self.url:
            raise ValueError(
                "Qdrant URL required. Set QDRANT_URL environment variable "
                "or pass url parameter."
            )

        self.client = QdrantClient(
            url=self.url,
            api_key=self.api_key
        )

    def create_collection(self) -> None:
        """
        Create textbook_chunks collection if it doesn't exist

        Collection configuration:
        - Vector size: 1536 (text-embedding-3-small)
        - Distance metric: Cosine similarity
        - On-disk payload storage for efficiency
        """
        # Check if collection exists
        collections = self.client.get_collections().collections
        collection_names = [c.name for c in collections]

        if self.COLLECTION_NAME in collection_names:
            print(f"Collection '{self.COLLECTION_NAME}' already exists")
            return

        # Create collection
        self.client.create_collection(
            collection_name=self.COLLECTION_NAME,
            vectors_config=VectorParams(
                size=self.VECTOR_SIZE,
                distance=self.DISTANCE_METRIC
            )
        )
        print(f"Created collection '{self.COLLECTION_NAME}'")

    def insert_chunk(
        self,
        chunk_id: str,
        embedding: List[float],
        content_text: str,
        metadata: Dict[str, Any]
    ) -> None:
        """
        Insert single content chunk into Qdrant

        Args:
            chunk_id: Unique chunk identifier (UUID)
            embedding: 1536-dimensional vector
            content_text: Raw text content
            metadata: Metadata dict with keys: module, week, chapter_title,
                      heading, page_url, heading_anchor

        Raises:
            ValueError: If embedding size doesn't match VECTOR_SIZE
        """
        if len(embedding) != self.VECTOR_SIZE:
            raise ValueError(
                f"Embedding size {len(embedding)} doesn't match "
                f"expected size {self.VECTOR_SIZE}"
            )

        # Combine content and metadata into payload
        payload = {
            "content_text": content_text,
            **metadata
        }

        point = PointStruct(
            id=chunk_id,
            vector=embedding,
            payload=payload
        )

        self.client.upsert(
            collection_name=self.COLLECTION_NAME,
            points=[point]
        )

    def insert_batch(
        self,
        chunk_ids: List[str],
        embeddings: List[List[float]],
        content_texts: List[str],
        metadatas: List[Dict[str, Any]]
    ) -> None:
        """
        Insert multiple chunks in a single batch operation

        Args:
            chunk_ids: List of unique chunk IDs
            embeddings: List of 1536-dimensional vectors
            content_texts: List of content texts
            metadatas: List of metadata dicts

        Raises:
            ValueError: If lists have different lengths or embedding sizes mismatch
        """
        if not (len(chunk_ids) == len(embeddings) == len(content_texts) == len(metadatas)):
            raise ValueError("All input lists must have the same length")

        # Validate embedding sizes
        for i, emb in enumerate(embeddings):
            if len(emb) != self.VECTOR_SIZE:
                raise ValueError(
                    f"Embedding {i} size {len(emb)} doesn't match "
                    f"expected size {self.VECTOR_SIZE}"
                )

        # Create points
        points = []
        for chunk_id, embedding, content_text, metadata in zip(
            chunk_ids, embeddings, content_texts, metadatas
        ):
            payload = {
                "content_text": content_text,
                **metadata
            }
            point = PointStruct(
                id=chunk_id,
                vector=embedding,
                payload=payload
            )
            points.append(point)

        # Batch upsert
        self.client.upsert(
            collection_name=self.COLLECTION_NAME,
            points=points
        )

    def search(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        score_threshold: float = 0.7,
        filter_conditions: Optional[Dict[str, Any]] = None
    ) -> List[SearchResult]:
        """
        Search for similar chunks using vector similarity

        Args:
            query_embedding: 1536-dimensional query vector
            top_k: Number of results to return (default: 5)
            score_threshold: Minimum similarity score (0-1, default: 0.7)
            filter_conditions: Optional metadata filters (e.g., {"module": "module-01-ros2"})

        Returns:
            List of SearchResult objects sorted by similarity score

        Example:
            >>> results = client.search(
            ...     query_embedding=embed_query("What is ROS 2?"),
            ...     top_k=5,
            ...     filter_conditions={"module": "module-01-ros2"}
            ... )
        """
        if len(query_embedding) != self.VECTOR_SIZE:
            raise ValueError(
                f"Query embedding size {len(query_embedding)} doesn't match "
                f"expected size {self.VECTOR_SIZE}"
            )

        # Build filter
        query_filter = None
        if filter_conditions:
            field_conditions = []
            for key, value in filter_conditions.items():
                field_conditions.append(
                    FieldCondition(
                        key=key,
                        match=MatchValue(value=value)
                    )
                )
            query_filter = Filter(must=field_conditions)

        # Search
        search_result = self.client.search(
            collection_name=self.COLLECTION_NAME,
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=score_threshold,
            query_filter=query_filter
        )

        # Convert to SearchResult objects
        results = []
        for hit in search_result:
            results.append(SearchResult(
                chunk_id=hit.id,
                score=hit.score,
                content_text=hit.payload.get("content_text", ""),
                metadata={
                    k: v for k, v in hit.payload.items()
                    if k != "content_text"
                }
            ))

        return results

    def delete_chunk(self, chunk_id: str) -> None:
        """
        Delete single chunk by ID

        Args:
            chunk_id: Chunk ID to delete
        """
        self.client.delete(
            collection_name=self.COLLECTION_NAME,
            points_selector=[chunk_id]
        )

    def delete_by_chapter(self, chapter_id: str) -> None:
        """
        Delete all chunks belonging to a specific chapter

        Args:
            chapter_id: Chapter ID (e.g., "module-01-ros2/week-03/ch01-nodes-topics")
        """
        # Note: Qdrant doesn't support direct payload-based deletion
        # We need to search and then delete by IDs
        # For now, this is a placeholder - implement if needed
        raise NotImplementedError(
            "Delete by chapter not yet implemented. "
            "Use re-ingestion to update chapter content."
        )

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get collection statistics

        Returns:
            Dict with collection info (point count, vector size, etc.)
        """
        info = self.client.get_collection(self.COLLECTION_NAME)
        return {
            "collection_name": self.COLLECTION_NAME,
            "points_count": info.points_count,
            "vector_size": info.config.params.vectors.size,
            "distance_metric": info.config.params.vectors.distance.name,
            "status": info.status.name
        }


# Convenience functions for direct usage
_default_client = None


def get_vector_db_client() -> VectorDBClient:
    """Get or create default vector DB client instance"""
    global _default_client
    if _default_client is None:
        _default_client = VectorDBClient()
    return _default_client


def search_similar_chunks(
    query_embedding: List[float],
    top_k: int = 5,
    module_filter: Optional[str] = None,
    chapter_filter: Optional[str] = None
) -> List[SearchResult]:
    """
    Search for similar chunks with optional module/chapter filtering

    Args:
        query_embedding: 1536-dimensional query vector
        top_k: Number of results
        module_filter: Filter by module ID (e.g., "module-01-ros2")
        chapter_filter: Filter by chapter title

    Returns:
        List of SearchResult objects

    Example:
        >>> from services.embeddings import embed_query
        >>> embedding = embed_query("How do ROS 2 nodes communicate?")
        >>> results = search_similar_chunks(
        ...     embedding,
        ...     top_k=5,
        ...     module_filter="module-01-ros2"
        ... )
    """
    client = get_vector_db_client()

    filter_conditions = {}
    if module_filter:
        filter_conditions["module"] = module_filter
    if chapter_filter:
        filter_conditions["chapter_title"] = chapter_filter

    return client.search(
        query_embedding=query_embedding,
        top_k=top_k,
        filter_conditions=filter_conditions if filter_conditions else None
    )
