# Data Model Design

**Feature**: Physical AI & Humanoid Robotics Interactive Textbook
**Date**: 2025-12-17
**Phase**: Phase 1 - Design & Contracts
**Prerequisites**: Phase 0 research complete

---

## Overview

This document defines the data models for the textbook content structure and chatbot system. Models are designed to support:
- Hierarchical textbook organization (Modules → Weeks → Chapters)
- RAG-based chatbot with context-aware retrieval
- Chat session management with history persistence
- Citation generation linking responses to source content

---

## Content Structure Models

### 1. Module

Represents a major topic grouping in the curriculum (e.g., "ROS 2", "Digital Twin").

**Attributes:**
- `id` (string, required): Unique identifier, format: `module-{number:02d}-{slug}` (e.g., `module-01-ros2`)
- `order` (integer, required): Display sequence (1-4)
- `name` (string, required): Human-readable title (e.g., "ROS 2 - Robotic Nervous System")
- `description` (string, optional): Brief overview (max 500 characters)
- `week_range` (string, required): Week span, format: `"{start}-{end}"` (e.g., `"3-5"`)
- `chapter_count` (integer, required): Number of chapters in module
- `assessment_id` (string, optional): Reference to assessment document

**Relationships:**
- Has many `Week` (1 module → 2-3 weeks)
- Has many `Chapter` (1 module → 4-6 chapters)
- Has one `Assessment` (optional)

**Constraints:**
- `order` must be unique and sequential (1, 2, 3, 4)
- `week_range` must not overlap with other modules
- `chapter_count` must match actual chapter count in module

**Example:**
```json
{
  "id": "module-01-ros2",
  "order": 1,
  "name": "ROS 2 - Robotic Nervous System",
  "description": "Master ROS 2 for robotic control including nodes, topics, services, and Python integration",
  "week_range": "3-5",
  "chapter_count": 6,
  "assessment_id": "module-01-ros2/assessment-ros2-project"
}
```

---

### 2. Week

Represents a single week in the 13-week course schedule.

**Attributes:**
- `number` (integer, required): Week number (1-13)
- `title` (string, required): Descriptive label (e.g., "Week 3: ROS 2 Nodes and Topics")
- `module_id` (string, required): Parent module reference
- `chapter_ids` (array of strings, required): Ordered list of chapter IDs for this week

**Relationships:**
- Belongs to one `Module`
- Has many `Chapter` (1 week → 2 chapters typically)

**Constraints:**
- `number` must be unique (1-13)
- `chapter_ids` length typically 2 (can vary for foundations or capstone weeks)
- Referenced chapters must belong to same module

**Example:**
```json
{
  "number": 3,
  "title": "Week 3: ROS 2 Nodes and Topics",
  "module_id": "module-01-ros2",
  "chapter_ids": [
    "module-01-ros2/week-03/ch01-nodes-topics",
    "module-01-ros2/week-03/ch02-services-actions"
  ]
}
```

---

### 3. Chapter

Represents a single textbook chapter (Markdown document).

**Attributes:**
- `id` (string, required): Unique identifier, format: `{module_id}/week-{number:02d}/ch{number:02d}-{slug}` (e.g., `module-01-ros2/week-03/ch01-nodes-topics`)
- `module_id` (string, required): Parent module reference
- `week_number` (integer, required): Parent week number (1-13)
- `order` (integer, required): Display sequence within module (1-6)
- `title` (string, required): Chapter heading (e.g., "ROS 2 Nodes and Topics")
- `file_path` (string, required): Relative path from docs root (e.g., `docs/module-01-ros2/week-03/ch01-nodes-topics.md`)
- `page_url` (string, required): Published URL path (e.g., `/docs/module-01-ros2/week-03/ch01-nodes-topics`)

**Relationships:**
- Belongs to one `Module`
- Belongs to one `Week`
- Has many `ContentChunk` (1 chapter → 8-12 chunks)

**Constraints:**
- `id` must be unique across all chapters
- `file_path` must exist in repository
- `page_url` must match Docusaurus routing convention
- `order` must be unique within module

**Example:**
```json
{
  "id": "module-01-ros2/week-03/ch01-nodes-topics",
  "module_id": "module-01-ros2",
  "week_number": 3,
  "order": 1,
  "title": "ROS 2 Nodes and Topics",
  "file_path": "docs/module-01-ros2/week-03/ch01-nodes-topics.md",
  "page_url": "/docs/module-01-ros2/week-03/ch01-nodes-topics"
}
```

---

## RAG System Models

### 4. ContentChunk

Represents a semantic chunk of textbook content embedded for RAG retrieval.

**Attributes:**
- `id` (string, required): Unique identifier (UUID v4)
- `chapter_id` (string, required): Parent chapter reference
- `content_text` (string, required): Raw text content (400-600 tokens typically)
- `embedding_vector` (array of float32, required): OpenAI text-embedding-3-small vector (1536 dimensions)
- `token_count` (integer, required): Number of tokens in `content_text`
- `chunk_order` (integer, required): Position within chapter (0-indexed)
- `metadata` (object, required): Structured metadata for retrieval and citation

**Metadata Schema:**
```json
{
  "module": "string (module_id)",
  "week": "integer (1-13)",
  "chapter_title": "string",
  "heading": "string (H2/H3 section title)",
  "page_url": "string (full URL path)",
  "heading_anchor": "string (e.g., '#creating-a-publisher-node')"
}
```

**Relationships:**
- Belongs to one `Chapter`

**Constraints:**
- `token_count` must be between 100-800 tokens
- `embedding_vector` must have exactly 1536 dimensions
- `chunk_order` must be unique within chapter
- `metadata.heading_anchor` must match Docusaurus auto-generated anchor

**Example:**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "chapter_id": "module-01-ros2/week-03/ch01-nodes-topics",
  "content_text": "ROS 2 uses a publish-subscribe pattern where publisher nodes send messages to topics, and subscriber nodes receive them asynchronously. This decouples communication and enables flexible robotic architectures. To create a publisher, use the rclpy.create_publisher() method...",
  "embedding_vector": [0.023, -0.112, 0.045, ...],  // 1536 dimensions
  "token_count": 450,
  "chunk_order": 2,
  "metadata": {
    "module": "module-01-ros2",
    "week": 3,
    "chapter_title": "ROS 2 Nodes and Topics",
    "heading": "Creating a Publisher Node",
    "page_url": "/docs/module-01-ros2/week-03/ch01-nodes-topics",
    "heading_anchor": "#creating-a-publisher-node"
  }
}
```

---

## Chatbot Session Models

### 5. ChatSession

Represents a user's conversation session with the chatbot.

**Attributes:**
- `session_id` (string, required): Unique identifier (UUID v4)
- `user_id` (string, optional): User identifier if authentication enabled (null for anonymous)
- `created_at` (timestamp, required): Session creation time (ISO 8601)
- `updated_at` (timestamp, required): Last activity time (ISO 8601)
- `expires_at` (timestamp, required): Expiration time (24 hours after `updated_at`)
- `context_selection` (object, required): User's selected retrieval scope

**Context Selection Schema:**
```json
{
  "mode": "full_textbook | module | chapter",
  "selected_module_ids": ["string array, empty if mode=full_textbook"],
  "selected_chapter_ids": ["string array, empty unless mode=chapter"]
}
```

**Relationships:**
- Has many `ChatMessage` (1 session → unbounded messages, last 10 retained for context)

**Constraints:**
- `expires_at` must be exactly 24 hours after `updated_at`
- `context_selection.mode` determines which IDs must be populated:
  - `full_textbook`: both arrays empty
  - `module`: `selected_module_ids` non-empty, `selected_chapter_ids` empty
  - `chapter`: both arrays non-empty
- Session automatically expires and messages archived after 24 hours idle

**Example:**
```json
{
  "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "user_id": null,
  "created_at": "2025-12-17T10:30:00Z",
  "updated_at": "2025-12-17T11:45:00Z",
  "expires_at": "2025-12-18T11:45:00Z",
  "context_selection": {
    "mode": "module",
    "selected_module_ids": ["module-01-ros2"],
    "selected_chapter_ids": []
  }
}
```

---

### 6. ChatMessage

Represents a single message in a chat conversation (user query or assistant response).

**Attributes:**
- `message_id` (string, required): Unique identifier (UUID v4)
- `session_id` (string, required): Parent session reference
- `role` (string, required): Message origin (`"user"` or `"assistant"`)
- `content` (string, required): Message text (max 10,000 characters for assistant, 500 for user)
- `timestamp` (timestamp, required): Message creation time (ISO 8601)
- `citations` (array of objects, optional): Source citations (only for assistant messages)
- `response_time_ms` (integer, optional): Time to generate response (only for assistant messages)

**Citation Schema:**
```json
{
  "chunk_id": "string (UUID)",
  "chapter_title": "string",
  "heading": "string",
  "url": "string (full page URL with anchor)"
}
```

**Relationships:**
- Belongs to one `ChatSession`

**Constraints:**
- `role` must be either `"user"` or `"assistant"`
- `content` max length: 500 chars (user), 10,000 chars (assistant)
- `citations` only present if `role == "assistant"`
- `response_time_ms` only present if `role == "assistant"`
- Messages ordered by `timestamp` within session

**Example (User Query):**
```json
{
  "message_id": "a3f7b2c1-9e4d-4a18-b6c8-5d9f3e2a1b0c",
  "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "role": "user",
  "content": "How do I create a ROS 2 publisher in Python?",
  "timestamp": "2025-12-17T11:45:32Z",
  "citations": null,
  "response_time_ms": null
}
```

**Example (Assistant Response):**
```json
{
  "message_id": "b4e8c3d2-0f5e-5b29-c7d9-6e0g4f3b2c1d",
  "session_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "role": "assistant",
  "content": "To create a ROS 2 publisher in Python, use rclpy.create_publisher()...\n\n📚 **Sources:**\n• [ROS 2 Nodes and Topics: Creating a Publisher Node](/docs/module-01-ros2/week-03/ch01-nodes-topics#creating-a-publisher-node)",
  "timestamp": "2025-12-17T11:45:35Z",
  "citations": [
    {
      "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
      "chapter_title": "ROS 2 Nodes and Topics",
      "heading": "Creating a Publisher Node",
      "url": "/docs/module-01-ros2/week-03/ch01-nodes-topics#creating-a-publisher-node"
    }
  ],
  "response_time_ms": 2850
}
```

---

## Entity Relationship Diagram

```
Module (1) ──< Week (many)
  │
  └──< Chapter (many) ──< ContentChunk (many)

ChatSession (1) ──< ChatMessage (many)

ChatMessage.citations[].chunk_id → ContentChunk.id (reference)
```

---

## Validation Rules

### Content Structure
1. **Module ordering**: `order` values must be sequential (1, 2, 3, 4) with no gaps
2. **Week coverage**: All weeks 1-13 must be assigned to exactly one module or foundations
3. **Chapter naming**: Chapter IDs must follow convention `{module_id}/week-{number:02d}/ch{number:02d}-{slug}`
4. **File existence**: All `file_path` references must exist in repository before ingestion

### RAG System
5. **Chunk token limits**: `token_count` must be 100-800 (reject if outside range)
6. **Embedding dimensions**: `embedding_vector` must have exactly 1536 floats (OpenAI text-embedding-3-small)
7. **Metadata completeness**: All metadata fields required (no null values except `user_id` in ChatSession)
8. **Chunk ordering**: `chunk_order` within chapter must be sequential starting from 0

### Chat Sessions
9. **Session expiry**: Expire sessions automatically 24 hours after `updated_at`
10. **Message history**: Retain only last 10 messages per session for context (archive older messages)
11. **Content length**: User messages max 500 chars, assistant messages max 10,000 chars
12. **Citation validity**: All `citations[].chunk_id` must reference existing ContentChunk

---

## State Transitions

### ChatSession Lifecycle
```
created → active → expired
   ↓         ↓
   └─────────┴──→ (24h idle) → archived
```

**States:**
- `created`: Session initialized, no messages yet
- `active`: At least one message, `updated_at` within last 24 hours
- `expired`: `updated_at` older than 24 hours
- `archived`: Messages moved to cold storage, session_id remains for reference

### ContentChunk Ingestion
```
raw_markdown → parsed → chunked → embedded → indexed
```

**Steps:**
1. **raw_markdown**: Read Markdown file from `file_path`
2. **parsed**: Extract AST, identify H2/H3 headings
3. **chunked**: Split into 400-600 token segments respecting boundaries
4. **embedded**: Generate 1536-dim vector via OpenAI API
5. **indexed**: Store in Qdrant with metadata

---

## Storage Estimates

### Content Structure (Static)
- **Modules**: 4 records × ~200 bytes = 0.8 KB
- **Weeks**: 13 records × ~150 bytes = 2 KB
- **Chapters**: 23 records × ~300 bytes = 7 KB
- **Total static metadata**: ~10 KB (negligible)

### RAG System (Qdrant)
- **ContentChunks**: 230 chunks × 6,500 bytes = ~1.5 MB (0.15% of 1GB limit)
- **Headroom**: 650x capacity remaining (can scale to 150,000 chunks if needed)

### Chat Sessions (Neon Postgres)
- **Sessions**: 1,000 active sessions × 500 bytes = 0.5 MB
- **Messages**: 1,000 sessions × 10 messages × 1 KB = 10 MB
- **Total**: ~10.5 MB (2% of 0.5GB free tier)
- **Scalability**: Can handle 10,000 active sessions within free tier

---

## Implementation Notes

### Technology-Agnostic Design
- Models defined with logical structure, not database-specific schema
- Supports multiple storage backends (Postgres, MongoDB, etc.)
- Embedding vector can be stored in Qdrant, Postgres (pgvector), or other vector DBs

### Docusaurus Integration
- `page_url` and `heading_anchor` follow Docusaurus conventions
- Chapter `id` matches Docusaurus document ID
- `file_path` relative to `docs/` directory

### OpenAI Agents SDK Integration
- `ChatSession` and `ChatMessage` align with Agents SDK conversation history format
- `ContentChunk.embedding_vector` compatible with OpenAI text-embedding-3-small output
- `citations` schema supports Agents SDK response metadata

---

## References

- [spec.md](./spec.md) - Feature specification (FR-001 to FR-025)
- [plan.md](./plan.md) - Implementation plan (Phase 1.1)
- [research.md](./research.md) - Phase 0 research decisions
- [ADR-0002](../../history/adr/0002-content-organization-and-pedagogical-structure.md) - Content structure
- [ADR-0003](../../history/adr/0003-rag-system-architecture-and-content-chunking.md) - RAG design
- [ADR-0005](../../history/adr/0005-dual-sdk-architecture-openai-agents-and-chatkit-integration.md) - SDK integration
