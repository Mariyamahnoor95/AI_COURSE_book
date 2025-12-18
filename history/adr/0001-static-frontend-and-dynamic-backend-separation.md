# ADR-0001: Static Frontend and Dynamic Backend Separation

> **Scope**: Architectural decision to separate textbook presentation (static) from chatbot functionality (dynamic), influencing deployment strategy, infrastructure costs, and system scalability.

- **Status:** Accepted
- **Date:** 2025-12-16
- **Feature:** 001-physical-ai-robotics-textbook
- **Context:** The textbook must serve static Markdown content to students while also providing an interactive RAG chatbot. This creates a fundamental architectural choice: build a monolithic application or separate concerns into static frontend and dynamic backend components.

## Decision

**Adopt a separated architecture with static frontend and dynamic backend:**

**Frontend Stack (Static):**
- Framework: Docusaurus 3.x for static site generation
- Content: Markdown files organized by module/week structure
- Deployment: GitHub Pages (free, CDN-backed)
- UI Components: React 18+ for chatbot widget integration

**Backend Stack (Dynamic):**
- Framework: FastAPI 0.104+ for REST API
- RAG System: OpenAI Agents/ChatKit SDK + text-embedding-3-small
- Vector DB: Qdrant Cloud (Free Tier, 1GB limit)
- Relational DB: Neon Serverless Postgres (Free Tier, 0.5GB)
- Deployment: Cloud service (Vercel/Railway/Render - to be selected in Phase 0)

**Integration:** React chatbot component embedded in Docusaurus pages communicates with FastAPI backend via REST API.

## Consequences

### Positive

- **Zero-cost static hosting**: GitHub Pages provides free CDN-backed hosting for textbook content (1GB limit, 100GB/month bandwidth)
- **Independent scaling**: Textbook content delivery and chatbot API scale separately; heavy chatbot usage doesn't impact textbook access
- **Performance optimization**: Static pages load instantly from CDN; chatbot API can be optimized independently for <3s response time (SC-002)
- **Deployment independence**: Textbook updates don't require backend redeployment and vice versa
- **Development parallelization**: Content authoring and chatbot development can proceed simultaneously
- **Cost predictability**: Free tier infrastructure for pilot (Qdrant 1GB, Neon 0.5GB, GitHub Pages) with clear upgrade path

### Negative

- **Additional complexity**: Requires managing two deployment pipelines (GitHub Actions for Pages + cloud platform for backend)
- **Cross-origin coordination**: Requires proper CORS configuration and API authentication
- **Session management overhead**: Must implement stateful sessions across stateless static pages
- **Local development setup**: Developers need to run both Docusaurus dev server and FastAPI backend
- **Network dependency**: Chatbot functionality requires external API calls; offline textbook access doesn't include chatbot

## Alternatives Considered

**Alternative A: Monolithic Next.js Application**
- Stack: Next.js 14 with App Router, server-side rendering, API routes for chatbot
- Deployment: Vercel (integrated frontend + backend)
- **Rejected because:**
  - Vercel free tier limits (100GB/month bandwidth, 100 hours compute) insufficient for 50+ concurrent users
  - Couples textbook updates with backend deployments
  - Higher complexity for content authoring (requires React knowledge vs. Markdown)
  - No cost advantage over separated approach

**Alternative B: Full-stack Docusaurus with Backend Plugin**
- Stack: Docusaurus + custom plugin with server-side components for chatbot
- Deployment: Single deployment to cloud platform supporting Node.js + Python
- **Rejected because:**
  - Docusaurus primarily designed for static generation; server-side plugin support limited
  - Loses GitHub Pages free hosting advantage
  - Complicates Docusaurus build process with backend dependencies
  - Increases deployment complexity without clear benefit

**Alternative C: Static Site with Edge Functions (Cloudflare Workers)**
- Stack: Docusaurus + GitHub Pages + Cloudflare Workers for chatbot API
- **Rejected because:**
  - Cloudflare Workers free tier (100k requests/day) may be insufficient for pilot
  - Limited Python support (requires Wasm or JavaScript rewrite)
  - Increases vendor lock-in beyond constitution-mandated technologies
  - Adds learning curve for edge computing paradigm

## References

- Feature Spec: [specs/001-physical-ai-robotics-textbook/spec.md](../../specs/001-physical-ai-robotics-textbook/spec.md)
- Implementation Plan: [specs/001-physical-ai-robotics-textbook/plan.md](../../specs/001-physical-ai-robotics-textbook/plan.md) (Structure Decision, Phase 0 Task 5)
- Related ADRs: ADR-0003 (RAG System Architecture), ADR-0004 (Zero-Cost Infrastructure Strategy)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Principle 3: Spec-Driven Development & Defined Tech Stack)
