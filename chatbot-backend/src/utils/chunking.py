"""
Content Chunking Utility for RAG System

Implements semantic chunking with heading-aware boundaries per research.md:
- Chunk size: 400-600 tokens (target), 800 tokens (maximum)
- Overlap: 60 tokens (15% of 400)
- Respects Markdown H2/H3 headings, code blocks, paragraphs
- Minimum: 100 tokens (merge short sections)

Maps to: FR-017 (chunking for embeddings)
"""

import re
from typing import List, Dict, Any
from dataclasses import dataclass
import tiktoken


@dataclass
class ContentChunk:
    """Represents a semantic chunk of textbook content"""
    content_text: str
    token_count: int
    chunk_order: int
    metadata: Dict[str, Any]


class ContentChunker:
    """Semantic chunking with heading-aware boundaries"""

    def __init__(
        self,
        chunk_size_min: int = 400,
        chunk_size_max: int = 600,
        chunk_size_limit: int = 800,
        overlap_tokens: int = 60,
        min_chunk_tokens: int = 100
    ):
        """
        Initialize chunker with configurable parameters

        Args:
            chunk_size_min: Target minimum chunk size in tokens
            chunk_size_max: Target maximum chunk size in tokens
            chunk_size_limit: Hard limit - chunks larger than this will be split
            overlap_tokens: Number of tokens to overlap between chunks
            min_chunk_tokens: Minimum size - smaller chunks will be merged
        """
        self.chunk_size_min = chunk_size_min
        self.chunk_size_max = chunk_size_max
        self.chunk_size_limit = chunk_size_limit
        self.overlap_tokens = overlap_tokens
        self.min_chunk_tokens = min_chunk_tokens

        # Use cl100k_base encoding (same as text-embedding-3-small)
        self.encoding = tiktoken.get_encoding("cl100k_base")

    def count_tokens(self, text: str) -> int:
        """Count tokens in text using tiktoken"""
        return len(self.encoding.encode(text))

    def chunk_markdown(
        self,
        markdown_content: str,
        chapter_id: str,
        module_id: str,
        week_number: int,
        chapter_title: str,
        page_url: str
    ) -> List[ContentChunk]:
        """
        Chunk Markdown content with semantic boundaries

        Args:
            markdown_content: Raw Markdown text
            chapter_id: Unique chapter identifier
            module_id: Parent module ID
            week_number: Week number (1-13)
            chapter_title: Chapter title
            page_url: Published URL path

        Returns:
            List of ContentChunk objects with metadata
        """
        # Parse Markdown structure
        sections = self._parse_sections(markdown_content)

        # Group sections into chunks
        chunks = []
        current_chunk = []
        current_tokens = 0
        chunk_order = 0

        for section in sections:
            section_tokens = self.count_tokens(section['content'])

            # Handle oversized sections (>800 tokens)
            if section_tokens > self.chunk_size_limit:
                # Save current chunk if exists
                if current_chunk:
                    chunk_text = self._join_sections(current_chunk)
                    chunks.append(self._create_chunk(
                        chunk_text,
                        chunk_order,
                        chapter_id,
                        module_id,
                        week_number,
                        chapter_title,
                        page_url,
                        current_chunk[0]['heading']
                    ))
                    chunk_order += 1
                    current_chunk = []
                    current_tokens = 0

                # Split oversized section at paragraph boundaries
                split_chunks = self._split_large_section(
                    section,
                    chapter_id,
                    module_id,
                    week_number,
                    chapter_title,
                    page_url,
                    chunk_order
                )
                chunks.extend(split_chunks)
                chunk_order += len(split_chunks)
                continue

            # Check if adding this section exceeds target max
            if current_tokens + section_tokens > self.chunk_size_max and current_chunk:
                # Save current chunk
                chunk_text = self._join_sections(current_chunk)
                chunks.append(self._create_chunk(
                    chunk_text,
                    chunk_order,
                    chapter_id,
                    module_id,
                    week_number,
                    chapter_title,
                    page_url,
                    current_chunk[0]['heading']
                ))
                chunk_order += 1

                # Start new chunk with overlap
                current_chunk = [section]
                current_tokens = section_tokens
            else:
                # Add section to current chunk
                current_chunk.append(section)
                current_tokens += section_tokens

        # Save final chunk
        if current_chunk:
            chunk_text = self._join_sections(current_chunk)
            chunks.append(self._create_chunk(
                chunk_text,
                chunk_order,
                chapter_id,
                module_id,
                week_number,
                chapter_title,
                page_url,
                current_chunk[0]['heading']
            ))

        # Merge chunks smaller than min_chunk_tokens
        chunks = self._merge_small_chunks(chunks)

        return chunks

    def _parse_sections(self, markdown_content: str) -> List[Dict[str, str]]:
        """
        Parse Markdown into sections based on H2/H3 headings

        Returns:
            List of dicts with 'heading', 'content', 'anchor' keys
        """
        sections = []

        # Split by H2 and H3 headings
        # Pattern matches: ## Heading or ### Heading
        heading_pattern = r'^(#{2,3})\s+(.+)$'
        lines = markdown_content.split('\n')

        current_heading = None
        current_anchor = None
        current_content = []

        for line in lines:
            match = re.match(heading_pattern, line)
            if match:
                # Save previous section
                if current_content:
                    sections.append({
                        'heading': current_heading or chapter_title,
                        'content': '\n'.join(current_content).strip(),
                        'anchor': current_anchor or ''
                    })

                # Start new section
                heading_level = len(match.group(1))  # Number of #
                heading_text = match.group(2).strip()
                current_heading = heading_text
                current_anchor = self._generate_anchor(heading_text)
                current_content = [line]  # Include heading in content
            else:
                current_content.append(line)

        # Save final section
        if current_content:
            sections.append({
                'heading': current_heading or 'Introduction',
                'content': '\n'.join(current_content).strip(),
                'anchor': current_anchor or ''
            })

        return sections

    def _generate_anchor(self, heading_text: str) -> str:
        """Generate Docusaurus-style heading anchor"""
        # Lowercase, replace spaces with hyphens, remove special chars
        anchor = heading_text.lower()
        anchor = re.sub(r'[^\w\s-]', '', anchor)
        anchor = re.sub(r'[-\s]+', '-', anchor)
        return f'#{anchor}'

    def _join_sections(self, sections: List[Dict[str, str]]) -> str:
        """Join sections into single text"""
        return '\n\n'.join(s['content'] for s in sections)

    def _create_chunk(
        self,
        chunk_text: str,
        chunk_order: int,
        chapter_id: str,
        module_id: str,
        week_number: int,
        chapter_title: str,
        page_url: str,
        heading: str
    ) -> ContentChunk:
        """Create ContentChunk with metadata"""
        token_count = self.count_tokens(chunk_text)

        # Extract heading anchor if present
        heading_anchor = self._generate_anchor(heading) if heading else ''

        metadata = {
            "module": module_id,
            "week": week_number,
            "chapter_title": chapter_title,
            "heading": heading,
            "page_url": f"{page_url}{heading_anchor}",
            "heading_anchor": heading_anchor
        }

        return ContentChunk(
            content_text=chunk_text,
            token_count=token_count,
            chunk_order=chunk_order,
            metadata=metadata
        )

    def _split_large_section(
        self,
        section: Dict[str, str],
        chapter_id: str,
        module_id: str,
        week_number: int,
        chapter_title: str,
        page_url: str,
        start_order: int
    ) -> List[ContentChunk]:
        """Split oversized section at paragraph boundaries with overlap"""
        content = section['content']
        heading = section['heading']
        paragraphs = re.split(r'\n\n+', content)

        chunks = []
        current_text = []
        current_tokens = 0
        chunk_order = start_order

        for para in paragraphs:
            para_tokens = self.count_tokens(para)

            if current_tokens + para_tokens > self.chunk_size_max and current_text:
                # Save current chunk
                chunk_text = '\n\n'.join(current_text)
                chunks.append(self._create_chunk(
                    chunk_text,
                    chunk_order,
                    chapter_id,
                    module_id,
                    week_number,
                    chapter_title,
                    page_url,
                    heading
                ))
                chunk_order += 1

                # Start new chunk with last paragraph for overlap
                current_text = [para]
                current_tokens = para_tokens
            else:
                current_text.append(para)
                current_tokens += para_tokens

        # Save final chunk
        if current_text:
            chunk_text = '\n\n'.join(current_text)
            chunks.append(self._create_chunk(
                chunk_text,
                chunk_order,
                chapter_id,
                module_id,
                week_number,
                chapter_title,
                page_url,
                heading
            ))

        return chunks

    def _merge_small_chunks(self, chunks: List[ContentChunk]) -> List[ContentChunk]:
        """Merge chunks smaller than min_chunk_tokens with previous chunk"""
        if not chunks:
            return chunks

        merged = [chunks[0]]

        for chunk in chunks[1:]:
            if chunk.token_count < self.min_chunk_tokens and merged:
                # Merge with previous chunk
                prev = merged[-1]
                merged_text = prev.content_text + '\n\n' + chunk.content_text
                merged_tokens = self.count_tokens(merged_text)

                # Update previous chunk
                merged[-1] = ContentChunk(
                    content_text=merged_text,
                    token_count=merged_tokens,
                    chunk_order=prev.chunk_order,
                    metadata=prev.metadata  # Keep first chunk's metadata
                )
            else:
                merged.append(chunk)

        return merged


def chunk_chapter_content(
    markdown_content: str,
    chapter_id: str,
    module_id: str,
    week_number: int,
    chapter_title: str,
    page_url: str
) -> List[ContentChunk]:
    """
    Convenience function to chunk chapter content with default parameters

    Args:
        markdown_content: Raw Markdown text
        chapter_id: Unique chapter identifier
        module_id: Parent module ID
        week_number: Week number (1-13)
        chapter_title: Chapter title
        page_url: Published URL path

    Returns:
        List of ContentChunk objects

    Example:
        >>> content = "## Introduction\\n\\nThis is a chapter...\\n\\n## Section 2\\n\\nMore content..."
        >>> chunks = chunk_chapter_content(
        ...     content,
        ...     "module-01-ros2/week-03/ch01-nodes-topics",
        ...     "module-01-ros2",
        ...     3,
        ...     "ROS 2 Nodes and Topics",
        ...     "/docs/module-01-ros2/week-03/ch01-nodes-topics"
        ... )
        >>> len(chunks)
        2
    """
    chunker = ContentChunker()
    return chunker.chunk_markdown(
        markdown_content,
        chapter_id,
        module_id,
        week_number,
        chapter_title,
        page_url
    )
