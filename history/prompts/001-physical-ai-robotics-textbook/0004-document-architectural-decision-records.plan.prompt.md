---
id: 0004
title: Document Architectural Decision Records
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-robotics-textbook
branch: 001-physical-ai-robotics-textbook
user: noori
command: /sp.adr
labels: ["adr", "architectural-decisions", "infrastructure", "content-structure", "rag-system", "free-tier-optimization"]
links:
  spec: specs/001-physical-ai-robotics-textbook/spec.md
  ticket: null
  adr: history/adr/0001-static-frontend-and-dynamic-backend-separation.md
  pr: null
files:
  - history/adr/0001-static-frontend-and-dynamic-backend-separation.md
  - history/adr/0002-content-organization-and-pedagogical-structure.md
  - history/adr/0003-rag-system-architecture-and-content-chunking.md
  - history/adr/0004-zero-cost-infrastructure-strategy-and-free-tier-optimization.md
tests:
  - ADR significance test (3-part): All 4 ADRs passed (Impact, Alternatives, Scope)
  - Constitution compliance validation: All ADRs trace to constitution mandates
---

## Prompt

/sp.adr Review the approved plan and document the key Architectural Decision Records (ADRs). Focus on decisions that: 1. Impact multiple phases or major sections of the textbook 2. Were driven by constitution-mandated constraints 3. Had viable alternatives that were considered and rejected 4. Influence content structure, authoring workflow, or deployment strategy also research.md, data-models.md and more you need

## Response snapshot

Created 4 clustered ADRs documenting architecturally significant decisions from the approved implementation plan:

**ADR-0001: Static Frontend and Dynamic Backend Separation**
- Decision: Docusaurus + GitHub Pages (static textbook) + FastAPI + Cloud (dynamic chatbot backend)
- Alternatives rejected: Monolithic Next.js, Full-stack Docusaurus, Cloudflare Workers edge functions
- Impact: Deployment strategy, infrastructure costs, development parallelization, performance optimization
- Rationale: Zero-cost GitHub Pages hosting, independent scaling, CDN performance, clear separation of concerns

**ADR-0002: Content Organization and Pedagogical Structure**
- Decision: Hybrid directory structure (foundations/ + module-XX/week-YY/chXX-topic.md)
- Alternatives rejected: Flat module structure, week-only structure, completely flat, topic-based only
- Impact: Navigation UX, chatbot context selection, authoring workflow, Docusaurus sidebar generation
- Rationale: Supports both sequential learning (week-by-week) and modular reference (topic-based), enables chatbot context filtering

**ADR-0003: RAG System Architecture and Content Chunking**
- Decision: Semantic chunking (200-800 tokens, 50-100 overlap), metadata-rich embeddings, citation-required responses
- Alternatives rejected: Fixed-size chunking, heading-only chunking, paragraph-level chunking, full-chapter embeddings, hybrid approach (deferred to Phase 0)
- Impact: Chatbot accuracy, response time, storage efficiency, citation UX, free tier compliance
- Rationale: Preserves document structure, enables precise citations, fits Qdrant 1GB limit, <3s response time
- Pending: Phase 0 Task 2 (finalize chunking parameters), Phase 0 Task 6 (citation format)

**ADR-0004: Zero-Cost Infrastructure Strategy and Free Tier Optimization**
- Decision: Optimize for free tiers (GitHub Pages 1GB, Qdrant 1GB, Neon 0.5GB) with explicit resource budgets and monitoring
- Alternatives rejected: Paid tiers from start, self-hosted infrastructure, AWS Free Tier, hybrid free/paid
- Impact: Content scope (max 26 chapters), session management, rate limiting, monitoring overhead, upgrade path
- Rationale: Zero upfront cost for pilot, predictable scaling ($44/month for paid tiers), forces efficient design, low financial risk
- Pending: Phase 0 Task 5 (backend deployment platform selection)

**Note on Missing Documents**: User requested documentation based on research.md and data-model.md, but these documents don't exist yet (Phase 0 and Phase 1 not executed). ADRs created exclusively from approved plan.md decisions. Additional ADRs may be generated after Phase 0 research completes (SDK selection, deployment platform, chunking parameters, citation format).

## Outcome

- ✅ Impact: 4 comprehensive ADRs document all cross-cutting architectural decisions from approved plan with alternatives analysis and traceability
- 🧪 Tests: Significance test applied to all decisions (Impact, Alternatives, Scope criteria), constitution compliance validated
- 📁 Files: Created history/adr/ directory with ADR-0001 through ADR-0004, all cross-referenced with spec.md and plan.md
- 🔁 Next prompts: Execute Phase 0 research tasks (6 decisions needed), then generate ADRs for SDK selection, deployment platform, and finalized chunking strategy
- 🧠 Reflection: ADR creation successfully extracted 4 decision clusters from plan.md covering architecture (static/dynamic separation), content structure (hybrid directory), RAG design (semantic chunking), and infrastructure (free tier optimization). Each ADR documents 3-5 alternatives with explicit rejection rationale. Status field notes pending Phase 0 decisions for ADR-0003 and ADR-0004. Cross-references ensure traceability to constitution mandates and specification requirements.

## Evaluation notes (flywheel)

- Failure modes observed: None - all 4 ADRs created successfully with complete alternatives analysis
- Graders run and results (PASS/FAIL): Significance test PASS (4/4 ADRs meet all 3 criteria), Constitution compliance PASS (all decisions trace to mandates)
- Prompt variant (if applicable): Standard /sp.adr workflow with plan-only input (research.md and data-model.md not yet available)
- Next experiment (smallest change to try): Execute Phase 0 research.md generation, then create follow-up ADRs for deferred decisions (SDK selection, backend platform, chunking parameters)
