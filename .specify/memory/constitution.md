<!--
Sync Impact Report:
Version change:  → 1.0.0
Modified principles:
  -  → Audience-Centric Design
  -  → Unified Textbook & Integrated RAG Chatbot
  -  → Spec-Driven Development & Defined Tech Stack
  -  → Markdown & Scope Adherence
Added sections:
  - Project Scope & Constraints
Removed sections:
  - Unused Principle sections
  - Unused Section 3
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .specify/templates/commands/*.md: ✅ updated
  - README.md: ⚠ pending (manual follow-up)
  - docs/quickstart.md: ⚠ pending (manual follow-up)
Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Original adoption date for the constitution.
-->
# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### Audience-Centric Design
All content MUST be tailored to undergraduate to early graduate students learning Physical AI and Humanoid Robotics. Technical depth and pedagogical approach MUST align with this demographic.
Rationale: Ensures the book and chatbot are effective learning tools for the target demographic, optimizing clarity and relevance.

### Unified Textbook & Integrated RAG Chatbot
The project MUST deliver a comprehensive course textbook on Physical AI & Humanoid Robotics. An embedded RAG chatbot MUST be integrated, with its knowledge base strictly limited to the book's content.
Rationale: Provides a cohesive learning experience and a focused, reliable conversational agent directly supporting the curriculum.

### Spec-Driven Development & Defined Tech Stack
All development MUST adhere strictly to Spec-Driven Development (SDD) principles. The project MUST utilize the specified tech stack: Spec-Kit Plus + Claude Code, Docusaurus + GitHub Pages, OpenAI Agents / ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud (Free Tier).
Rationale: Ensures structured development, maintains consistency, leverages chosen technologies effectively, and prevents scope creep.

### Markdown & Scope Adherence
All textual output MUST be in Markdown format. Content MUST strictly remain within the defined scope of Physical AI & Humanoid Robotics; out-of-scope content is NOT permitted.
Rationale: Guarantees consistent formatting, facilitates documentation, and prevents dilution of the core subject matter.

## Project Scope & Constraints

## Target Audience
Undergraduate to early graduate students learning Physical AI and Humanoid Robotics.

## Scope
Create a unified course textbook on Physical AI & Humanoid Robotics and an embedded RAG chatbot that answers questions from the book content only.

## Tech Stack
- Spec-Kit Plus + Claude Code
- Docusaurus + GitHub Pages
- OpenAI Agents / ChatKit SDKs
- FastAPI
- Neon Serverless Postgres
- Qdrant Cloud (Free Tier)

## Constraints
- Spec-driven development only
- Markdown output
- No out-of-scope content

## Governance
This constitution supersedes all other project practices. Amendments require a documented proposal, team approval, and a clear migration plan. All Pull Requests and code reviews MUST verify compliance with these principles. Unjustified complexity is not permitted.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date for the constitution. | **Last Amended**: 2025-12-15
