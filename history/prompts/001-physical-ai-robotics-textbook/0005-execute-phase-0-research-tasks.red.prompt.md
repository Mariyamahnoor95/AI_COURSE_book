---
id: 0005
title: Execute Phase 0 Research Tasks
stage: red
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-robotics-textbook
branch: 001-physical-ai-robotics-textbook
user: noori
command: /sp.execute
labels: ["phase-0", "research", "technical-decisions", "sdk-selection", "deployment", "chunking", "citations"]
links:
  spec: specs/001-physical-ai-robotics-textbook/spec.md
  ticket: null
  adr: history/adr/0005-dual-sdk-architecture-openai-agents-and-chatkit-integration.md
  pr: null
files:
  - specs/001-physical-ai-robotics-textbook/research.md
  - history/adr/0005-dual-sdk-architecture-openai-agents-and-chatkit-integration.md
tests:
  - Constitution compliance validation: All decisions trace to mandates (PASS)
  - Free tier budget validation: All choices within Qdrant 1GB, Neon 0.5GB, GitHub Pages 1GB limits (PASS)
  - ADR significance test: ADR-0005 passed (Impact, Alternatives, Scope criteria met)
---

## Prompt

/sp.execute

Execute Phase 0 research tasks only.

Focus on:
1. Finalizing SDK and tool selections mandated by the constitution
2. Confirming deployment platform and GitHub Pages configuration
3. Defining content chunking parameters for RAG and search
4. Finalizing citation and reference format for the textbook

For each decision:
- Summarize findings
- State the final choice
- Note risks and trade-offs
- Indicate whether a follow-up ADR is required

## Response snapshot

**Phase 0 Research Complete - 5 Critical Decisions Finalized**

### Decision 1: SDK Selection
**Final Choice:** Dual SDK architecture - OpenAI Agents SDK (backend RAG logic) + ChatKit SDK (frontend UI/integration)
**Rationale:** Not mutually exclusive; Agents provides intelligence layer, ChatKit provides UX layer
**Key Finding:** ChatKit integrates WITH Agents SDK, leveraging built-in RAG optimizations + pre-built UI components
**Follow-up ADR:** YES (ADR-0005 created)

### Decision 2: Deployment Platform
**Final Choice:** Render Free Tier (FastAPI backend)
**Configuration:** 512MB RAM, 15-minute sleep mode, 30-60s cold start, $0/month
**Upgrade Path:** $7/month Starter plan (no sleep) if cold starts problematic
**Key Trade-off:** Free tier with sleep vs. always-on service ($5-7/month)
**Follow-up ADR:** NO (aligned with ADR-0004 Zero-Cost Strategy)

### Decision 3: Content Chunking
**Final Choice:** Semantic chunking with heading-aware boundaries
**Parameters:** 400-600 tokens (target), 60-token overlap (15%), 800-token maximum
**Algorithm:** Parse Markdown AST, respect H2/H3 headings, split long sections at paragraphs
**Storage Validation:** 23 chapters × 10 chunks = 230 chunks (0.15% of Qdrant 1GB limit)
**Follow-up ADR:** NO (refines ADR-0003 with specific parameters)

### Decision 4: Docusaurus & GitHub Pages
**Final Choice:** Nested sidebar categories with generated index pages
**Structure:** Module → Week → Chapter (3 levels), collapsible categories, emoji markers for assessments
**GitHub Pages:** Free tier (1GB storage, 100GB bandwidth), automated deployment via GitHub Actions
**Follow-up ADR:** NO (implementation detail aligned with ADR-0002)

### Decision 5: Citation Format
**Final Choice:** Structured end-of-response citations with clickable links
**Format:** `📚 **Sources:**` section with `[Chapter: Heading](URL)` list
**Implementation:** Deduplicate citations, limit to top 3-5 sources, Docusaurus auto-generates heading anchors
**Follow-up ADR:** NO (implementation detail aligned with ADR-0003)

**Architecture Summary:**
```
Frontend: Docusaurus + ChatKit.js → REST/WebSocket
Backend: FastAPI + ChatKit Python SDK → OpenAI Agents SDK
RAG: Agents SDK → Qdrant (vector search) + OpenAI API (embeddings/chat)
Storage: Neon Postgres (chat history), Qdrant Cloud (embeddings)
Deployment: GitHub Pages (textbook), Render Free Tier (chatbot backend)
```

**Key Validations:**
- ✅ Constitution compliance (all decisions trace to mandated technologies)
- ✅ Free tier budget (Qdrant <1% utilization, Render $0/month, GitHub Pages free)
- ✅ Performance goals (chunking supports <3s response time, <1s retrieval)

## Outcome

- ✅ Impact: Phase 0 complete with all 5 technical unknowns resolved and documented
- 🧪 Tests: Constitution compliance PASS, free tier budget validation PASS, ADR significance test PASS
- 📁 Files: Created research.md (comprehensive decision documentation) and ADR-0005 (dual SDK architecture)
- 🔁 Next prompts: Proceed to Phase 1 (generate data-model.md, contracts/chatbot-api.yaml, quickstart.md)
- 🧠 Reflection: Research phase successfully balanced constitution mandates with free tier constraints. Dual SDK decision (Agents + ChatKit) was non-obvious but emerged from understanding that SDKs complement rather than compete. Semantic chunking parameters (400-600 tokens, 15% overlap) align with 2025 industry best practices while staying well within Qdrant 1GB limit. Render Free Tier chosen over Railway/Fly.io due to clearest zero-cost pilot path despite 30-60s cold starts. All decisions documented with explicit risks, trade-offs, and mitigation strategies.

## Evaluation notes (flywheel)

- Failure modes observed: None - all 5 decisions finalized with clear rationale and validation
- Graders run and results (PASS/FAIL): Constitution compliance PASS (5/5 decisions trace to mandates), Free tier budget PASS (all within limits), ADR significance PASS (ADR-0005 meets all 3 criteria)
- Prompt variant (if applicable): Standard /sp.execute Phase 0 workflow with focused decision criteria (findings, choice, risks, ADR requirement)
- Next experiment (smallest change to try): Execute Phase 1 design tasks (data models, API contracts, quickstart guide) using finalized Phase 0 decisions as input
