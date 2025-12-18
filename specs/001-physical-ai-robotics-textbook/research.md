# Phase 0 Research: Technical Decisions

**Feature**: Physical AI & Humanoid Robotics Interactive Textbook
**Date**: 2025-12-17
**Status**: Complete

---

## Decision 1: SDK Selection (OpenAI Agents vs ChatKit)

### Findings Summary

**OpenAI Agents SDK:**
- Production-ready framework for agentic workflows
- Built-in RAG optimizations (query reranking, file search tool)
- Pydantic-powered tool schema generation
- Python support (stable), works with Responses API and Chat Completions API

**ChatKit SDK:**
- UI toolkit for embedding chat experiences in products
- React components (ChatKit.js) + Python backend SDK
- Integrates WITH Agents SDK (provides event streaming, widget rendering)
- Generally available with AgentKit release

**Key Insight:** Not mutually exclusive - ChatKit provides frontend/UI layer, Agents SDK provides backend/RAG logic.

### Final Choice

**Use BOTH: OpenAI Agents SDK (backend) + ChatKit SDK (frontend integration)**

**Architecture:**
```
Docusaurus + ChatKit.js → FastAPI + ChatKit Python SDK → OpenAI Agents SDK → Qdrant/OpenAI API
```

**Rationale:**
- Agents SDK: Production-ready RAG pipeline with built-in optimizations
- ChatKit SDK: Pre-built UI components save frontend development time
- Constitution compliance: Both are OpenAI official SDKs
- Reduces custom code: Leverage OpenAI's RAG and UI best practices

### Risks & Trade-offs

**Risks:**
- Dual SDK dependency increases complexity and learning curve
- Version conflicts possible between Agents SDK and ChatKit SDK

**Mitigation:**
- Both officially supported by OpenAI with aligned release cycles
- Start with basic integration, add advanced features incrementally

**Trade-offs:**
- More dependencies vs. full control with custom implementation
- **Accepted**: Built-in optimizations and UI components justify complexity

### Follow-up ADR Required?
**YES** - Create ADR-0005 documenting dual SDK architecture decision (significant architectural choice affecting entire chatbot stack)

---

## Decision 2: Deployment Platform

### Findings Summary

Platform comparison for FastAPI hosting:

| Platform | Free Tier | FastAPI Support | Cold Start | Cost |
|----------|-----------|----------------|------------|------|
| Vercel | 100GB bandwidth, 100k invocations | Serverless only (NOT full FastAPI) | ~1-3s | $0 |
| Railway | No free tier | Excellent, templates available | ~10s (sleep after 10min) | $5/month minimum |
| Render | Free tier, sleeps after 15min | Excellent, managed DBs | ~30-60s | $0 (free) / $7 (starter) |
| Fly.io | Soft $5/month free tier | Excellent, Docker containers | ~5-10s | ~$3-5/month |

**Key Finding:** Vercel unsuitable for full FastAPI apps (serverless functions only, not persistent services).

### Final Choice

**Render Free Tier for pilot deployment**

**Configuration:**
- Free tier with 15-minute sleep mode
- 512MB RAM, shared CPU
- Git-based auto-deploy
- Upgrade path: $7/month Starter (no sleep)

**Rationale:**
- Zero infrastructure cost for pilot validation
- Straightforward deployment (Git push → auto-deploy)
- 30-60s cold start acceptable for pilot testing
- Clear upgrade path if sleep mode becomes problematic

### Risks & Trade-offs

**Risks:**
- 30-60s cold starts after 15min idle may frustrate users
- Free tier performance variability under load

**Mitigation:**
- Add "Waking up chatbot..." loading state in UI
- Consider scheduled pings during peak hours to keep warm
- Monitor usage patterns for upgrade decision

**Trade-offs:**
- Free tier with sleep vs. $5-7/month for always-on service
- **Accepted**: Pilot budget prioritizes zero cost; upgrade based on real usage data

### Follow-up ADR Required?
**NO** - Infrastructure implementation detail aligned with ADR-0004 (Zero-Cost Strategy). May revisit post-pilot.

---

## Decision 3: Content Chunking Parameters

### Findings Summary

**Industry consensus (2025):**
- Recommended chunk size: **400-512 tokens**
- Overlap: **10-20% of chunk size** (50-100 tokens)
- NVIDIA research: 15% overlap optimal with semantic chunking
- Semantic chunking (respecting headings/paragraphs) outperforms fixed-size for technical docs

**Storage calculation:**
- OpenAI text-embedding-3-small: 1536 dims × 4 bytes = 6,144 bytes/vector
- Metadata overhead: ~300-500 bytes/chunk
- Total: ~6,500 bytes/chunk
- Qdrant 1GB limit = ~150,000 chunks capacity
- Target: 23 chapters × 10 chunks = 230 chunks (0.15% of limit)

### Final Choice

**Semantic chunking with heading-aware boundaries**

**Parameters:**
- **Chunk size**: 400-600 tokens (target), 800 tokens (maximum)
- **Overlap**: 60 tokens (15% of 400)
- **Boundaries**: Respect Markdown H2/H3 headings, code blocks, paragraphs
- **Minimum**: 100 tokens (merge short sections with context)

**Algorithm:**
1. Parse Markdown AST to identify structural boundaries
2. Group content between headings into candidate chunks
3. If >800 tokens, split at paragraph boundaries with 60-token overlap
4. If <100 tokens, merge with previous chunk
5. Attach metadata: module, week, chapter, heading, URL, order

**Metadata schema:**
```json
{
  "chunk_id": "uuid",
  "chapter_id": "module-01-ros2/week-03/ch01-nodes-topics",
  "module": "module-01-ros2",
  "week": 3,
  "chapter_title": "ROS 2 Nodes and Topics",
  "heading": "Creating a Publisher Node",
  "page_url": "/docs/module-01-ros2/week-03/ch01-nodes-topics#creating-a-publisher-node",
  "chunk_order": 2,
  "token_count": 450
}
```

**Rationale:**
- 400-600 tokens balances context completeness with retrieval precision
- Heading-aware boundaries enable precise section-level citations
- Well within Qdrant 1GB limit (using <1% for 23 chapters)
- 15% overlap captures cross-boundary concepts

### Risks & Trade-offs

**Risks:**
- Variable chunk sizes (100-800 tokens) may affect retrieval consistency
- Semantic chunking complexity (AST parsing) vs. simple fixed-size

**Mitigation:**
- Normalize by token count in metadata
- Tune top-k retrieval parameter based on empirical testing

**Trade-offs:**
- Implementation complexity vs. citation accuracy
- **Accepted**: Citation quality and context preservation justify semantic approach

### Follow-up ADR Required?
**NO** - Refines ADR-0003 (RAG System Architecture) with specific parameters. No new architectural decision.

---

## Decision 4: GitHub Pages & Docusaurus Configuration

### Findings Summary

**Docusaurus sidebar capabilities:**
- Hierarchical categories (`type: 'category'`)
- Collapsible sections with `collapsible: true`
- Generated index pages with category links
- Automatic doc ID resolution from file paths

**GitHub Pages:**
- 1GB storage limit, 100GB/month bandwidth
- Free for public repositories
- Custom domain support
- Automatic deployment via GitHub Actions

### Final Choice

**Nested sidebar with generated index pages**

**Sidebar structure:**
```javascript
// sidebars.js
export default {
  textbookSidebar: [
    {type: 'doc', id: 'intro', label: 'Course Overview'},
    {
      type: 'category',
      label: 'Foundations (Weeks 1-2)',
      collapsible: true,
      collapsed: false,  // Initially expanded
      link: {
        type: 'generated-index',
        title: 'Physical AI Foundations',
        slug: '/category/foundations',
      },
      items: [
        {type: 'category', label: 'Week 1', items: ['foundations/week-01/ch00-intro', '...']},
        {type: 'category', label: 'Week 2', items: ['foundations/week-02/ch02-sensors', '...']}
      ]
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 (Weeks 3-5)',
      collapsible: true,
      collapsed: true,  // Modules collapsed by default
      link: {type: 'generated-index', title: 'ROS 2', slug: '/category/module-01-ros2'},
      items: [
        {type: 'category', label: 'Week 3', items: ['...']},
        {type: 'category', label: 'Week 4', items: ['...']},
        {type: 'category', label: 'Week 5', items: ['...']},
        {type: 'doc', id: 'module-01-ros2/assessment-ros2-project', label: '📝 Assessment'}
      ]
    }
    // Repeat for Modules 2-4
  ]
};
```

**GitHub Pages deployment:**
- Use Docusaurus built-in deployment: `npm run deploy`
- GitHub Actions workflow for automated deployment on push to main
- Build output to `gh-pages` branch

**Rationale:**
- Three-level hierarchy (Module → Week → Chapter) matches course structure
- Generated index pages provide module overviews without extra files
- Collapsible categories reduce visual clutter (26+ chapters)
- GitHub Pages free tier sufficient for textbook (~200MB build)

### Risks & Trade-offs

**Risks:**
- Three-level nesting creates deep URLs
- Manual sidebar config requires updates when adding chapters

**Mitigation:**
- Docusaurus handles deep paths well with breadcrumbs
- Use consistent naming conventions to minimize config changes

**Trade-offs:**
- Manual config vs. auto-generation (more control but more maintenance)
- **Accepted**: Precise control over labels and order justifies manual approach

### Follow-up ADR Required?
**NO** - Implementation detail aligned with ADR-0002 (Content Organization).

---

## Decision 5: Citation Format

### Findings Summary

**Requirements (FR-020):**
- Chatbot must cite specific textbook sections
- Enable verification against source material
- Include chapter title, section heading, URL

**Format options:**
- Inline citations (academic): `[1]` with footnotes
- Embedded links (conversational): hyperlinked phrases
- Structured citations (clear): end-of-response list
- Hover tooltips (UI-based): requires custom component

### Final Choice

**Structured end-of-response citations with clickable links**

**Format:**
```markdown
[Chatbot response content, 2-4 paragraphs]

📚 **Sources:**
• [Chapter Title: Section Heading](/docs/path/to/page#heading-anchor)
• [Chapter Title: Section Heading](/docs/path/to/page#heading-anchor)
```

**Example:**
```
ROS 2 uses a publish-subscribe pattern where publisher nodes send messages
to topics, and subscriber nodes receive them asynchronously. To create a
publisher, use rclpy.create_publisher() with a message type and topic name.

📚 **Sources:**
• [ROS 2 Nodes and Topics: Creating a Publisher Node](/docs/module-01-ros2/week-03/ch01-nodes-topics#creating-a-publisher-node)
• [Python Integration: Using rclpy](/docs/module-01-ros2/week-04/ch03-python-rclpy#create-publisher-method)
```

**Implementation (FastAPI):**
- Retrieve metadata from RAG chunks (chapter_title, heading, page_url)
- Deduplicate citations (same section cited once)
- Format as Markdown list with hyperlinks
- ChatKit MessageList renders as clickable links

**Rationale:**
- End-of-response placement doesn't interrupt reading flow
- Clickable links enable one-click verification (SC-005)
- 📚 emoji provides visual indicator (consistent with 📝 for assessments)
- Docusaurus auto-generates heading anchors (no manual maintenance)

### Risks & Trade-offs

**Risks:**
- Citations may become verbose with 5-10 sources
- End-of-response may be overlooked by users

**Mitigation:**
- Limit to top 3-5 most relevant chunks
- Use visual separator and emoji for visibility

**Trade-offs:**
- End-of-response vs. inline citations (academic rigor)
- **Accepted**: Conversational UX prioritizes readability

### Follow-up ADR Required?
**NO** - Implementation detail aligned with ADR-0003 (RAG System Architecture).

---

## Summary Table

| Decision | Final Choice | Follow-up ADR? |
|----------|--------------|----------------|
| **SDK Selection** | OpenAI Agents SDK (backend) + ChatKit SDK (frontend) | **YES (ADR-0005)** |
| **Deployment Platform** | Render Free Tier (upgrade to $7/month Starter if needed) | NO |
| **Content Chunking** | 400-600 tokens, 60-token overlap (15%), semantic boundaries | NO |
| **Docusaurus Config** | Nested categories with generated index pages, GitHub Pages | NO |
| **Citation Format** | Structured end-of-response with clickable links | NO |

## Next Steps

1. ✅ **Phase 0 Complete** - All technical decisions finalized
2. **Create ADR-0005**: Dual SDK Architecture (Agents + ChatKit)
3. **Proceed to Phase 1**: Generate data-model.md, contracts/chatbot-api.yaml, quickstart.md

## References

### Internal Documents
- [spec.md](./spec.md) - Feature specification
- [plan.md](./plan.md) - Implementation plan
- [ADR-0001](../../history/adr/0001-static-frontend-and-dynamic-backend-separation.md)
- [ADR-0002](../../history/adr/0002-content-organization-and-pedagogical-structure.md)
- [ADR-0003](../../history/adr/0003-rag-system-architecture-and-content-chunking.md)
- [ADR-0004](../../history/adr/0004-zero-cost-infrastructure-strategy-and-free-tier-optimization.md)

### External Sources

**OpenAI SDKs:**
- [OpenAI Agents SDK](https://openai.github.io/openai-agents-python/)
- [ChatKit SDK Documentation](https://platform.openai.com/docs/guides/chatkit)
- [New Tools for Building Agents | OpenAI](https://openai.com/index/new-tools-for-building-agents/)

**RAG Chunking:**
- [Best Chunking Strategies for RAG 2025 | Firecrawl](https://www.firecrawl.dev/blog/best-chunking-strategies-rag-2025)
- [Chunking Best Practices | Unstructured](https://unstructured.io/blog/chunking-for-rag-best-practices)
- [Finding the Best Chunking Strategy | NVIDIA](https://developer.nvidia.com/blog/finding-the-best-chunking-strategy-for-accurate-ai-responses/)

**Deployment Platforms:**
- [Python Hosting Comparison 2025 | Nandann](https://www.nandann.com/blog/python-hosting-options-comparison)
- [Platform Comparison | Jason Sy](https://www.jasonsy.dev/blog/comparing-deployment-platforms-2025)
- [Heroku vs Render vs Vercel vs Fly.io vs Railway | BoltOps](https://blog.boltops.com/2025/05/01/heroku-vs-render-vs-vercel-vs-fly-io-vs-railway-meet-blossom-an-alternative/)

**Docusaurus:**
- [Docusaurus GitHub Repository](https://github.com/facebook/docusaurus)
- [Docusaurus Sidebar Documentation](https://docusaurus.io/docs/sidebar)
