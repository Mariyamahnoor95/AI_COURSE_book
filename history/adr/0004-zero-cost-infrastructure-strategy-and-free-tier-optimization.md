# ADR-0004: Zero-Cost Infrastructure Strategy and Free Tier Optimization

> **Scope**: Infrastructure cost optimization strategy leveraging free tiers for pilot deployment, influencing architecture design, content chunking, deployment platform selection, and future scalability path.

- **Status:** Accepted (with Phase 0 backend deployment platform selection pending)
- **Date:** 2025-12-16
- **Feature:** 001-physical-ai-robotics-textbook
- **Context:** The project targets a pilot deployment for 50-100 concurrent students without initial funding. This requires strategic use of free tier services while maintaining performance goals (<3s chatbot response, <2s page load, 50+ concurrent users). The decision directly impacts technical architecture, content design, and operational monitoring.

## Decision

**Adopt a zero-cost infrastructure strategy using free tier services with explicit resource budgets:**

**Infrastructure Stack and Limits:**

| Service | Free Tier Limit | Projected Usage | Mitigation Strategy |
|---------|----------------|-----------------|---------------------|
| **GitHub Pages** | 1GB storage, 100GB/month bandwidth | ~200MB textbook (Docusaurus build), ~10GB/month bandwidth | Static site optimization (image compression, code splitting) |
| **Qdrant Cloud** | 1GB vector storage | ~600-800MB (20-26 chapters, 4-6 chunks/chapter, 1536-dim embeddings) | Semantic chunking optimization (Phase 0 Task 2), aggressive deduplication |
| **Neon Postgres** | 0.5GB storage, compute limits | ~50MB (chat history, 10 messages/session × 1000 sessions) | 24-hour session expiry, message retention cap |
| **OpenAI API** | Pay-per-use (no free tier) | ~$20-50/month (embeddings $0.02/1M tokens, chat $0.50/1M tokens) | Rate limiting (10 queries/user/hour), caching frequent queries |
| **Backend Deployment** | Platform-specific (to be selected in Phase 0 Task 5) | FastAPI service with cold start tolerance | Candidates: Vercel (serverless), Railway (512MB RAM), Render (512MB RAM) |

**Resource Budget Enforcement:**

- **Content constraint**: Maximum 26 chapters to stay within Qdrant 1GB limit
- **Session management**: Automatic cleanup of expired sessions (>24h inactive)
- **Rate limiting**: Per-user query limits (10/hour) with clear messaging
- **Monitoring**: Weekly usage checks for Qdrant storage, Neon storage, API costs
- **Upgrade path**: If pilot exceeds limits, paid tier costs are predictable (Qdrant $25/month for 4GB, Neon $19/month for compute)

**Cost Prediction (Monthly):**

- **Free infrastructure**: $0 (GitHub Pages, Qdrant Free, Neon Free, backend free tier)
- **OpenAI API**: $20-50 (variable based on user activity)
- **Total pilot cost**: $20-50/month

## Consequences

### Positive

- **Zero upfront cost**: Pilot deployment requires no infrastructure funding, only OpenAI API usage
- **Predictable scaling costs**: Exceeding limits has clear upgrade path ($44/month for paid Qdrant + Neon tiers)
- **Forces efficient design**: Storage and compute constraints drive optimization (semantic chunking, session cleanup, rate limiting)
- **Rapid deployment**: Free tier signup and deployment faster than procurement processes for paid services
- **Low financial risk**: If pilot fails, no infrastructure costs to recoup
- **Educational alignment**: Resource constraints teach students about real-world cost optimization

### Negative

- **Hard storage limits**: Qdrant 1GB is non-negotiable; exceeding requires immediate paid upgrade or content reduction
- **Content scope risk**: If textbook expands beyond 26 chapters, must either compress chunks or upgrade infrastructure
- **Performance variability**: Free tier compute may have cold starts (backend), rate limits, or degraded performance under load
- **Rate limiting friction**: 10 queries/hour per user may frustrate power users during study sessions
- **Monitoring overhead**: Manual weekly checks required to prevent hitting limits without warning
- **OpenAI API cost variability**: High user activity spikes could exceed $50/month budget
- **Migration complexity**: Moving from free to paid tiers requires careful data migration (Qdrant, Neon)

## Alternatives Considered

**Alternative A: Paid Tiers from Start**
- Stack: Qdrant Standard ($25/month, 4GB), Neon Pro ($19/month), Vercel Pro ($20/month)
- **Total monthly cost**: $64/month + OpenAI API costs
- **Rejected because:**
  - Requires funding commitment before validating pilot success
  - No significant performance advantage for 50-100 concurrent users
  - Free tier constraints can be validated during pilot; premature optimization
  - Doesn't teach resource-constrained design principles

**Alternative B: Self-Hosted Infrastructure**
- Stack: DigitalOcean Droplet ($6/month, 1GB RAM), self-hosted Qdrant, PostgreSQL
- **Rejected because:**
  - Requires DevOps overhead (server management, backups, security patches)
  - 1GB RAM insufficient for FastAPI + Qdrant + Postgres on single instance
  - No auto-scaling for traffic spikes
  - Higher total cost when accounting for engineering time

**Alternative C: AWS Free Tier (EC2 + RDS + S3)**
- Stack: t2.micro EC2, RDS PostgreSQL (free tier), S3 for static hosting
- **Rejected because:**
  - Free tier limited to 12 months; not sustainable for pilot extension
  - More complex setup than managed services (Qdrant Cloud, Neon)
  - No native vector database support (would need pgvector or external Qdrant)
  - S3 + CloudFront setup more complex than GitHub Pages for static hosting

**Alternative D: Hybrid Free/Paid (Free frontend, paid backend)**
- Stack: GitHub Pages (free) + Railway Pro ($5/month) + Qdrant Standard ($25/month)
- **Rejected because:**
  - Partial paid commitment without validating full free tier viability
  - Railway Pro unnecessary if free tier (512MB RAM) handles FastAPI workload
  - Better to start fully free and upgrade components as needed

## References

- Feature Spec: [specs/001-physical-ai-robotics-textbook/spec.md](../../specs/001-physical-ai-robotics-textbook/spec.md) (SC-006: 50+ concurrent users)
- Implementation Plan: [specs/001-physical-ai-robotics-textbook/plan.md](../../specs/001-physical-ai-robotics-textbook/plan.md) (Technical Context Constraints, Phase 0 Task 2, Phase 0 Task 5)
- Related ADRs: ADR-0001 (Static Frontend and Dynamic Backend Separation), ADR-0003 (RAG System Architecture - chunking optimization for storage limit)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Principle 3: Mandated tech stack including Qdrant Cloud Free Tier, Neon Postgres Free Tier)
- **Pending Decision**: Phase 0 Task 5 will select specific backend deployment platform (Vercel/Railway/Render) based on free tier limits and cold start benchmarks
