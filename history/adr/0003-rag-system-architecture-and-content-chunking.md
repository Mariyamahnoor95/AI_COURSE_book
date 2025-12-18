# ADR-0003: RAG System Architecture and Content Chunking

> **Scope**: RAG pipeline design including content chunking strategy, embedding generation, metadata schema, and citation implementation, influencing chatbot accuracy, response time, and free tier compliance.

- **Status:** Accepted (with Phase 0 research pending for specific parameters)
- **Date:** 2025-12-16
- **Feature:** 001-physical-ai-robotics-textbook
- **Context:** The chatbot must answer questions exclusively from textbook content (FR-012, FR-019) with <3s response time (SC-002) and cite sources (FR-020), while fitting within Qdrant Free Tier 1GB limit. This requires strategic decisions about content chunking, embedding models, metadata schema, and retrieval strategy.

## Decision

**Adopt a semantic chunking RAG architecture with metadata-rich embeddings:**

**Content Chunking Strategy:**
- **Approach**: Semantic chunking (to be refined in Phase 0 research)
- **Target chunk size**: 200-800 tokens (preliminary estimate)
- **Overlap**: 50-100 tokens (preliminary estimate)
- **Chunking boundaries**: Respect Markdown headings, code blocks, and paragraph boundaries
- **Rationale**: Balance between context completeness and retrieval precision

**Embedding and Storage:**
- **Model**: OpenAI text-embedding-3-small (1536 dimensions)
- **Vector DB**: Qdrant Cloud Free Tier (1GB limit)
- **Estimated capacity**: ~20-26 chapters with 4-6 chunks per chapter (to be validated in Phase 0)
- **Metadata schema**:
  ```json
  {
    "chunk_id": "string (UUID)",
    "chapter_id": "string",
    "module": "string (e.g., 'module-01-ros2')",
    "week": "number (1-13)",
    "chapter_title": "string",
    "heading": "string (section heading within chapter)",
    "page_url": "string (Docusaurus page URL for citation)",
    "chunk_order": "number (position within chapter)"
  }
  ```

**RAG Pipeline:**
1. User query → OpenAI embedding generation
2. Qdrant vector similarity search (top-k=5-10, to be tuned in Phase 0)
3. Context filtering by user's context selection (module/chapter/full textbook)
4. Retrieved chunks + metadata → OpenAI Agents/ChatKit SDK for response generation
5. Response includes citations with chapter title, heading, and URL (FR-020)

**Chat History:**
- **Storage**: Neon Serverless Postgres (Free Tier, 0.5GB)
- **Retention**: Last 10 messages per session for conversational context
- **Session timeout**: 24 hours of inactivity

**Citation Implementation:**
- **Format**: [Chapter Title: Section Heading](URL) with optional chunk preview
- **Validation**: Phase 0 research task 6 will define specific format

## Consequences

### Positive

- **Semantic accuracy**: Chunking respects document structure, preserving context and reducing fragmented answers
- **Verifiability**: Citations enable students to verify chatbot responses against source material (SC-005)
- **Context-aware retrieval**: Metadata filtering supports module/chapter-specific queries (FR-016)
- **Free tier compliance**: Estimated 1GB vector storage fits 20-26 chapters within Qdrant limit
- **Fast retrieval**: Vector similarity search + metadata filtering achieves <1s retrieval (supports <3s total response time goal)
- **Conversational continuity**: 10-message history enables multi-turn dialogue without full textbook re-retrieval
- **Scalability**: Chunk-based approach allows incremental content updates without full re-embedding

### Negative

- **Chunking complexity**: Semantic chunking requires sophisticated parsing (heading detection, code block preservation)
- **Storage uncertainty**: Preliminary estimates need Phase 0 validation; may require chunk size reduction if 1GB exceeded
- **Citation overhead**: Including metadata in every chunk increases storage footprint (~10-15% overhead)
- **Retrieval tuning required**: Top-k parameter, similarity threshold, and context window need empirical optimization
- **Embedding costs**: ~$0.02 per 1M tokens; full textbook re-embedding on structural changes costs ~$0.50-$1.00
- **Out-of-scope detection challenge**: RAG may retrieve marginally relevant chunks for off-topic queries; requires validation logic (SC-003)

## Alternatives Considered

**Alternative A: Fixed-Size Chunking (500 tokens, 50-token overlap)**
- **Approach**: Split Markdown into 500-token chunks regardless of structure
- **Rejected because:**
  - Breaks mid-sentence or mid-code-block, reducing answer quality
  - Poor citation UX (citations may point to arbitrary text fragments, not semantic sections)
  - Harder to implement context filtering by heading/section
  - No clear advantage over semantic chunking for technical documentation

**Alternative B: Heading-Based Chunking (One chunk per H2/H3 section)**
- **Approach**: Chunk at Markdown heading boundaries
- **Rejected because:**
  - Variable chunk sizes (some headings may cover 2000+ tokens, exceeding optimal embedding context)
  - Long sections reduce retrieval precision (may retrieve entire section when only subsection is relevant)
  - May exceed Qdrant 1GB limit if many chapters have large sections
  - Doesn't solve overlap problem for cross-section concepts

**Alternative C: Paragraph-Level Chunking**
- **Approach**: One chunk per Markdown paragraph
- **Rejected because:**
  - Too granular; loses contextual coherence (e.g., multi-paragraph explanations)
  - Increases chunk count, risking Qdrant storage limit
  - Requires aggressive top-k retrieval (10-20 chunks) to assemble coherent answers, slowing response time
  - Poor citation UX (many micro-citations instead of section-level references)

**Alternative D: No Chunking (Embed Full Chapters)**
- **Approach**: Store one embedding per chapter
- **Rejected because:**
  - OpenAI embedding models have 8191 token limit; full chapters may exceed this
  - Retrieval returns entire chapter context, reducing answer specificity
  - Doesn't support citation at section/heading level (FR-020)
  - Harder to fit within Qdrant 1GB limit (full-chapter embeddings are larger)

**Alternative E: Hybrid Approach (Heading-based with size cap)**
- **Approach**: Chunk at headings, but split large sections at paragraph boundaries if >800 tokens
- **Considered viable but deferred**: This is a refinement of semantic chunking; Phase 0 research (Task 2) will validate whether this complexity is needed

## References

- Feature Spec: [specs/001-physical-ai-robotics-textbook/spec.md](../../specs/001-physical-ai-robotics-textbook/spec.md) (FR-012, FR-016, FR-019, FR-020, SC-002, SC-003, SC-005)
- Implementation Plan: [specs/001-physical-ai-robotics-textbook/plan.md](../../specs/001-physical-ai-robotics-textbook/plan.md) (Phase 0 Task 2, Phase 0 Task 6, Phase 1.1 Data Model)
- Related ADRs: ADR-0002 (Content Organization - impacts metadata schema), ADR-0004 (Zero-Cost Infrastructure Strategy)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Principle 3: Mandated tech stack including OpenAI embeddings, Qdrant Cloud)
- **Pending Research**: Phase 0 Task 2 will finalize chunking parameters (size, overlap, boundary rules); Phase 0 Task 6 will finalize citation format
