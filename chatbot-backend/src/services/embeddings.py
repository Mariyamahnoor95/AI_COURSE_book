"""
Embedding Service

Generates 1536-dimensional vectors using OpenAI text-embedding-3-small
per research.md decisions.

Maps to: FR-016, FR-017
"""

import os
from typing import List
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


class EmbeddingService:
    """Service for generating text embeddings using OpenAI API"""

    def __init__(self, api_key: str | None = None):
        """
        Initialize embedding service

        Args:
            api_key: OpenAI API key (defaults to OPENAI_API_KEY env var)
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError(
                "OpenAI API key required. Set OPENAI_API_KEY environment variable "
                "or pass api_key parameter."
            )

        self.client = OpenAI(api_key=self.api_key)
        self.model = "text-embedding-3-small"  # 1536 dimensions
        self.dimensions = 1536

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding vector for single text

        Args:
            text: Text to embed

        Returns:
            List of 1536 float values representing the embedding

        Raises:
            ValueError: If text is empty
            OpenAI API errors: If API request fails
        """
        if not text or not text.strip():
            raise ValueError("Text cannot be empty")

        response = self.client.embeddings.create(
            model=self.model,
            input=text.strip()
        )

        embedding = response.data[0].embedding
        assert len(embedding) == self.dimensions, (
            f"Expected {self.dimensions} dimensions, got {len(embedding)}"
        )

        return embedding

    def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in a single API call

        Args:
            texts: List of texts to embed (max 2048 texts per batch)

        Returns:
            List of embedding vectors, one per input text

        Raises:
            ValueError: If texts list is empty or contains empty strings
            OpenAI API errors: If API request fails
        """
        if not texts:
            raise ValueError("Texts list cannot be empty")

        # Filter out empty texts and track indices
        non_empty_texts = [(i, t.strip()) for i, t in enumerate(texts) if t.strip()]
        if not non_empty_texts:
            raise ValueError("All texts are empty")

        # Check batch size limit
        if len(non_empty_texts) > 2048:
            raise ValueError(
                f"Batch size {len(non_empty_texts)} exceeds OpenAI limit of 2048. "
                "Split into smaller batches."
            )

        # Get embeddings
        response = self.client.embeddings.create(
            model=self.model,
            input=[t for _, t in non_empty_texts]
        )

        # Verify dimensions
        embeddings = [item.embedding for item in response.data]
        for emb in embeddings:
            assert len(emb) == self.dimensions, (
                f"Expected {self.dimensions} dimensions, got {len(emb)}"
            )

        # Map embeddings back to original indices (handle empty texts)
        result = [None] * len(texts)
        for (original_idx, _), embedding in zip(non_empty_texts, embeddings):
            result[original_idx] = embedding

        # Replace None with zero vectors for empty texts
        zero_vector = [0.0] * self.dimensions
        result = [emb if emb is not None else zero_vector for emb in result]

        return result

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for user query (alias for embed_text)

        Args:
            query: User query text

        Returns:
            List of 1536 float values
        """
        return self.embed_text(query)


# Convenience functions for direct usage
_default_service = None


def get_embedding_service() -> EmbeddingService:
    """Get or create default embedding service instance"""
    global _default_service
    if _default_service is None:
        _default_service = EmbeddingService()
    return _default_service


def embed_text(text: str) -> List[float]:
    """
    Generate embedding for text using default service

    Args:
        text: Text to embed

    Returns:
        1536-dimensional embedding vector

    Example:
        >>> embedding = embed_text("ROS 2 uses a publish-subscribe pattern")
        >>> len(embedding)
        1536
    """
    service = get_embedding_service()
    return service.embed_text(text)


def embed_batch(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts using default service

    Args:
        texts: List of texts to embed

    Returns:
        List of 1536-dimensional embedding vectors

    Example:
        >>> texts = ["ROS 2 basics", "Digital twin simulation"]
        >>> embeddings = embed_batch(texts)
        >>> len(embeddings)
        2
        >>> len(embeddings[0])
        1536
    """
    service = get_embedding_service()
    return service.embed_batch(texts)


def embed_query(query: str) -> List[float]:
    """
    Generate embedding for user query using default service

    Args:
        query: User query text

    Returns:
        1536-dimensional embedding vector
    """
    service = get_embedding_service()
    return service.embed_query(query)
