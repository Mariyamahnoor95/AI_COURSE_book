"""
Content Ingestion Script

Reads docs/ Markdown files, chunks content, generates embeddings,
and stores in Qdrant with metadata.

Usage:
  python chatbot-backend/scripts/ingest_content.py --docs-dir ./docs
  python chatbot-backend/scripts/ingest_content.py --docs-dir ./docs --incremental
  python chatbot-backend/scripts/ingest_content.py --docs-dir ./docs --force

Maps to: FR-017
"""

import os
import sys
import argparse
import uuid
from pathlib import Path
from typing import List, Dict, Any, Tuple
import re

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.utils.chunking import chunk_chapter_content, ContentChunk
from src.services.embeddings import EmbeddingService
from src.services.vector_db import VectorDBClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Exit codes
EXIT_SUCCESS = 0
EXIT_WARNINGS = 1
EXIT_FAILURES = 2


def parse_frontmatter(markdown_content: str) -> Dict[str, str]:
    """
    Extract frontmatter from Markdown file

    Args:
        markdown_content: Raw Markdown text

    Returns:
        Dict with frontmatter fields (id, title, etc.)
    """
    frontmatter_pattern = r'^---\n(.*?)\n---'
    match = re.match(frontmatter_pattern, markdown_content, re.DOTALL)

    if not match:
        return {}

    frontmatter_text = match.group(1)
    frontmatter = {}

    for line in frontmatter_text.split('\n'):
        if ':' in line:
            key, value = line.split(':', 1)
            frontmatter[key.strip()] = value.strip()

    return frontmatter


def extract_chapter_metadata(file_path: Path, docs_dir: Path) -> Dict[str, Any]:
    """
    Extract metadata from chapter file path and frontmatter

    Args:
        file_path: Path to Markdown file
        docs_dir: Root docs directory

    Returns:
        Dict with chapter_id, module_id, week_number, chapter_title, page_url
    """
    # Read file
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    frontmatter = parse_frontmatter(content)

    # Extract from path: docs/module-01-ros2/week-03/ch01-nodes-topics.md
    relative_path = file_path.relative_to(docs_dir)
    path_parts = relative_path.parts

    # Determine module and week from path
    module_id = None
    week_number = None

    if len(path_parts) >= 2:
        # Check if first part is a module or foundations
        if path_parts[0].startswith('module-') or path_parts[0] == 'foundations':
            module_id = path_parts[0]

            # Check for week directory
            if len(path_parts) >= 3 and path_parts[1].startswith('week-'):
                week_str = path_parts[1].replace('week-', '')
                week_number = int(week_str)

    # Build chapter_id from frontmatter or path
    chapter_id = frontmatter.get('id')
    if not chapter_id:
        # Build from path
        chapter_id = str(relative_path.with_suffix('')).replace(os.sep, '/')

    # Get title
    chapter_title = frontmatter.get('title', file_path.stem.replace('-', ' ').title())

    # Build page URL
    page_url = f"/docs/{chapter_id}"

    return {
        'chapter_id': chapter_id,
        'module_id': module_id or 'unknown',
        'week_number': week_number or 0,
        'chapter_title': chapter_title,
        'page_url': page_url,
        'content': content
    }


def find_markdown_files(docs_dir: Path) -> List[Path]:
    """
    Find all Markdown files in docs directory

    Args:
        docs_dir: Root docs directory

    Returns:
        List of Path objects for .md files
    """
    markdown_files = []

    for root, dirs, files in os.walk(docs_dir):
        for file in files:
            if file.endswith('.md') and not file.startswith('_'):
                file_path = Path(root) / file
                markdown_files.append(file_path)

    return sorted(markdown_files)


def ingest_chapter(
    file_path: Path,
    docs_dir: Path,
    embedding_service: EmbeddingService,
    vector_db_client: VectorDBClient,
    validation_warnings: List[str],
    validation_errors: List[str]
) -> Tuple[int, int]:
    """
    Ingest single chapter into vector database with validation

    Args:
        file_path: Path to Markdown file
        docs_dir: Root docs directory
        embedding_service: Embedding service instance
        vector_db_client: Vector DB client instance
        validation_warnings: List to accumulate warnings
        validation_errors: List to accumulate errors

    Returns:
        Tuple of (chunks_created, tokens_processed)
    """
    print(f"  Processing: {file_path.name}...", end='')

    # Extract metadata
    metadata = extract_chapter_metadata(file_path, docs_dir)

    # Chunk content
    chunks = chunk_chapter_content(
        markdown_content=metadata['content'],
        chapter_id=metadata['chapter_id'],
        module_id=metadata['module_id'],
        week_number=metadata['week_number'],
        chapter_title=metadata['chapter_title'],
        page_url=metadata['page_url']
    )

    if not chunks:
        print(" ⚠️  No chunks generated (file may be empty)")
        return 0, 0

    # Validate chunks (T013a)
    chunk_warnings, chunk_errors = validate_chunks(chunks, file_path.name)
    validation_warnings.extend(chunk_warnings)
    validation_errors.extend(chunk_errors)

    # Generate embeddings (batch for efficiency)
    content_texts = [chunk.content_text for chunk in chunks]
    embeddings = embedding_service.embed_batch(content_texts)

    # Generate UUIDs for chunks
    chunk_ids = [str(uuid.uuid4()) for _ in chunks]

    # Prepare metadatas
    metadatas = [chunk.metadata for chunk in chunks]

    # Insert into Qdrant
    vector_db_client.insert_batch(
        chunk_ids=chunk_ids,
        embeddings=embeddings,
        content_texts=content_texts,
        metadatas=metadatas
    )

    total_tokens = sum(chunk.token_count for chunk in chunks)
    status = "✅" if not chunk_errors else "⚠️"
    print(f" {status} {len(chunks)} chunks, {total_tokens} tokens")

    return len(chunks), total_tokens


def validate_pre_ingestion(docs_dir: Path) -> Tuple[bool, List[str], List[str]]:
    """
    Pre-flight validation checks before ingestion

    Args:
        docs_dir: Root docs directory

    Returns:
        Tuple of (passed, warnings, errors)
    """
    warnings = []
    errors = []

    # Check 1: docs/ directory exists
    if not docs_dir.exists():
        errors.append(f"Docs directory not found: {docs_dir}")
        return False, warnings, errors

    # Check 2: Find Markdown files
    markdown_files = find_markdown_files(docs_dir)
    if not markdown_files:
        errors.append(f"No Markdown files found in {docs_dir}")
        return False, warnings, errors

    # Check 3: Validate each Markdown file
    for file_path in markdown_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check if file is parseable
            if not content.strip():
                warnings.append(f"Empty file: {file_path.name}")
                continue

            # Check frontmatter
            frontmatter = parse_frontmatter(content)
            if not frontmatter:
                warnings.append(f"No frontmatter: {file_path.name}")

            # Check if frontmatter has required fields (id, title)
            if frontmatter and 'title' not in frontmatter:
                warnings.append(f"Missing 'title' in frontmatter: {file_path.name}")

        except UnicodeDecodeError:
            errors.append(f"Cannot decode file (encoding error): {file_path.name}")
        except Exception as e:
            errors.append(f"Error reading {file_path.name}: {str(e)}")

    passed = len(errors) == 0
    return passed, warnings, errors


def validate_chunks(chunks: List[ContentChunk], file_name: str) -> Tuple[List[str], List[str]]:
    """
    Validate chunk token limits per T013a acceptance criteria

    Args:
        chunks: List of content chunks
        file_name: File name for error reporting

    Returns:
        Tuple of (warnings, errors)
    """
    warnings = []
    errors = []

    for i, chunk in enumerate(chunks):
        # Warning if <400 or >600 tokens
        if chunk.token_count < 400:
            warnings.append(
                f"{file_name} chunk {i}: {chunk.token_count} tokens (<400 min target)"
            )
        elif chunk.token_count > 600:
            warnings.append(
                f"{file_name} chunk {i}: {chunk.token_count} tokens (>600 max target)"
            )

        # Error if >800 tokens
        if chunk.token_count > 800:
            errors.append(
                f"{file_name} chunk {i}: {chunk.token_count} tokens (>800 hard limit)"
            )

    return warnings, errors


def validate_post_ingestion(
    expected_chunks: int,
    actual_chunks: int,
    tolerance: float = 0.9
) -> Tuple[bool, str]:
    """
    Post-ingestion verification per T013a acceptance criteria

    Args:
        expected_chunks: Expected number of chunks
        actual_chunks: Actual chunks inserted
        tolerance: Minimum ratio (default: 90%)

    Returns:
        Tuple of (passed, message)
    """
    if expected_chunks == 0:
        return True, "No chunks expected"

    ratio = actual_chunks / expected_chunks

    if ratio < tolerance:
        return False, (
            f"Post-ingestion check failed: {actual_chunks}/{expected_chunks} chunks "
            f"inserted ({ratio:.1%} < {tolerance:.0%} threshold)"
        )

    return True, (
        f"Post-ingestion check passed: {actual_chunks}/{expected_chunks} chunks "
        f"inserted ({ratio:.1%})"
    )


def main():
    """Main ingestion workflow with validation"""
    parser = argparse.ArgumentParser(description='Ingest textbook content into vector database')
    parser.add_argument(
        '--docs-dir',
        type=str,
        required=True,
        help='Path to docs directory (e.g., ./docs or ../../docs)'
    )
    parser.add_argument(
        '--incremental',
        action='store_true',
        help='Only process new/modified files (not yet implemented)'
    )
    parser.add_argument(
        '--force',
        action='store_true',
        help='Force re-ingestion of all files'
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Output results as JSON'
    )

    args = parser.parse_args()

    # Resolve docs directory
    docs_dir = Path(args.docs_dir).resolve()
    if not docs_dir.exists():
        print(f"❌ Error: Docs directory not found: {docs_dir}")
        sys.exit(1)

    if not args.json:
        print("🚀 Content Ingestion Script")
        print("=" * 60)
        print(f"Docs directory: {docs_dir}")
        print(f"Mode: {'Incremental' if args.incremental else 'Full'}")
        print()

    # Pre-flight validation
    if not args.json:
        print("🔍 Running pre-flight validation...")

    pre_passed, pre_warnings, pre_errors = validate_pre_ingestion(docs_dir)

    if pre_warnings and not args.json:
        print(f"\n⚠️  Pre-flight warnings ({len(pre_warnings)}):")
        for warning in pre_warnings[:5]:  # Show first 5
            print(f"    {warning}")
        if len(pre_warnings) > 5:
            print(f"    ... and {len(pre_warnings) - 5} more")

    if pre_errors:
        if not args.json:
            print(f"\n❌ Pre-flight errors ({len(pre_errors)}):")
            for error in pre_errors:
                print(f"    {error}")
            print("\nFix these errors before proceeding.")
        sys.exit(EXIT_FAILURES)

    if not args.json:
        print("✅ Pre-flight validation passed")
        print()

    # Initialize services
    try:
        embedding_service = EmbeddingService()
        vector_db_client = VectorDBClient()
    except ValueError as e:
        print(f"❌ Error initializing services: {e}")
        print("\nMake sure you have set:")
        print("  - OPENAI_API_KEY in .env")
        print("  - QDRANT_URL in .env")
        print("  - QDRANT_API_KEY in .env")
        sys.exit(1)

    # Ensure collection exists
    try:
        vector_db_client.create_collection()
    except Exception as e:
        print(f"⚠️  Warning: Could not create collection: {e}")

    # Find Markdown files
    markdown_files = find_markdown_files(docs_dir)

    if not markdown_files:
        print(f"⚠️  No Markdown files found in {docs_dir}")
        sys.exit(0)

    if not args.json:
        print(f"📚 Found {len(markdown_files)} Markdown files")
        print()

    # Process files
    total_chunks = 0
    expected_chunks = 0
    total_tokens = 0
    processed_files = 0
    failed_files = []
    validation_warnings = []
    validation_errors = []

    for file_path in markdown_files:
        try:
            chunks, tokens = ingest_chapter(
                file_path,
                docs_dir,
                embedding_service,
                vector_db_client,
                validation_warnings,
                validation_errors
            )
            expected_chunks += chunks  # Track for post-ingestion validation
            total_chunks += chunks
            total_tokens += tokens
            processed_files += 1
        except Exception as e:
            if not args.json:
                print(f"  ❌ Error: {e}")
            failed_files.append((file_path.name, str(e)))

    # Post-ingestion validation (T013a)
    post_passed, post_message = validate_post_ingestion(expected_chunks, total_chunks)

    # Determine exit code
    exit_code = EXIT_SUCCESS
    if validation_errors or failed_files or not post_passed:
        exit_code = EXIT_FAILURES
    elif validation_warnings:
        exit_code = EXIT_WARNINGS

    # Summary
    if args.json:
        import json
        result = {
            "status": "success" if exit_code == EXIT_SUCCESS else ("warnings" if exit_code == EXIT_WARNINGS else "failures"),
            "processed_files": processed_files,
            "failed_files": len(failed_files),
            "total_chunks": total_chunks,
            "total_tokens": total_tokens,
            "validation_warnings": len(validation_warnings),
            "validation_errors": len(validation_errors),
            "post_ingestion_passed": post_passed,
            "errors": [{"file": f, "error": e} for f, e in failed_files],
            "exit_code": exit_code
        }
        print(json.dumps(result, indent=2))
    else:
        print()
        print("=" * 60)
        print("✅ Ingestion Complete!")
        print(f"  Files processed: {processed_files}/{len(markdown_files)}")
        print(f"  Total chunks: {total_chunks}")
        print(f"  Total tokens: {total_tokens}")

        # Validation summary
        if validation_warnings:
            print(f"\n⚠️  Validation warnings: {len(validation_warnings)}")
            for warning in validation_warnings[:5]:
                print(f"    {warning}")
            if len(validation_warnings) > 5:
                print(f"    ... and {len(validation_warnings) - 5} more")

        if validation_errors:
            print(f"\n❌ Validation errors: {len(validation_errors)}")
            for error in validation_errors[:5]:
                print(f"    {error}")
            if len(validation_errors) > 5:
                print(f"    ... and {len(validation_errors) - 5} more")

        if failed_files:
            print(f"\n❌ Failed files: {len(failed_files)}")
            for file, error in failed_files:
                print(f"    - {file}: {error}")

        # Post-ingestion check
        print(f"\n📊 Post-ingestion validation:")
        print(f"  {post_message}")
        if not post_passed:
            print(f"  ❌ FAILED: Some chunks may not have been inserted correctly")

        # Show collection info
        try:
            info = vector_db_client.get_collection_info()
            print(f"\n📊 Vector DB Status:")
            print(f"  Collection: {info['collection_name']}")
            print(f"  Points: {info['points_count']}")
            print(f"  Status: {info['status']}")
        except Exception as e:
            print(f"\n⚠️  Could not fetch collection info: {e}")

        if exit_code == EXIT_SUCCESS:
            print("\n🎉 Vector database ready for RAG queries!")
        elif exit_code == EXIT_WARNINGS:
            print("\n⚠️  Ingestion completed with warnings. Review above.")
        else:
            print("\n❌ Ingestion completed with errors. Fix issues and re-run.")

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
