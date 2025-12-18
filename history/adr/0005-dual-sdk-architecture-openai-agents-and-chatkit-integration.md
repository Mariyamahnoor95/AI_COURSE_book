# ADR-0005: Dual SDK Architecture - OpenAI Agents and ChatKit Integration

> **Scope**: SDK selection for RAG chatbot implementation, affecting frontend-backend integration, conversation management, and overall system architecture.

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** 001-physical-ai-robotics-textbook
- **Context:** The RAG chatbot requires both backend intelligence (RAG pipeline, LLM orchestration, vector search) and frontend user experience (chat UI, message rendering, context selection). This decision determines which OpenAI SDKs to use and how they integrate with FastAPI and Docusaurus.

## Decision

**Adopt dual SDK architecture combining OpenAI Agents SDK (backend) with ChatKit SDK (frontend/integration):**

**Backend Stack (RAG Logic):**
- **OpenAI Agents SDK** (`openai-agents-python`): RAG orchestration, LLM calls, conversation management
- **ChatKit Python SDK** (`openai-chatkit`): Event streaming, response formatting for frontend
- **FastAPI**: REST API endpoints (`/chat`, `/context`, `/sessions`)
- **Qdrant Client**: Vector similarity search
- **Psycopg3**: Chat history persistence (Neon Postgres)

**Frontend Stack (User Interface):**
- **ChatKit.js**: React chat UI components (message list, input, context selector)
- **Custom Docusaurus integration**: Swizzle Root component to embed ChatbotWidget globally
- **WebSocket/REST**: Real-time communication with FastAPI backend

**Integration Flow:**
```
User Query (Docusaurus + ChatKit.js)
  → WebSocket/REST → FastAPI endpoint
    → ChatKit Python SDK (format request)
      → OpenAI Agents SDK (RAG pipeline)
        → Qdrant (vector search for relevant chunks)
        → OpenAI API (embedding + chat completion)
        → Return response with citations
      → ChatKit Python SDK (stream events)
    → WebSocket/REST → ChatKit.js (render messages)
  → User sees response with clickable citations
```

**Component Responsibilities:**

| Component | Responsibility |
|-----------|----------------|
| **Agents SDK** | RAG retrieval, query optimization, LLM orchestration, tool calling |
| **ChatKit Python** | Event streaming, response formatting, session management |
| **ChatKit.js** | Message rendering, input handling, context UI, WebSocket client |
| **FastAPI** | API routing, request validation, database queries, CORS handling |

**Rationale:**
- **Agents SDK**: Production-ready RAG with built-in optimizations (query reranking, automatic tool generation)
- **ChatKit SDK**: Pre-built UI components and event streaming reduce custom code
- **Not mutually exclusive**: ChatKit provides UI/integration layer; Agents provides intelligence layer
- **Constitution compliance**: Both are OpenAI official SDKs (mandated by constitution)
- **Reduced development time**: Leverage OpenAI's RAG and UI best practices instead of building from scratch

## Consequences

### Positive

- **Production-ready RAG**: Agents SDK includes query optimization, reranking, and conversation history management out-of-box
- **Reduced frontend complexity**: ChatKit.js provides pre-built message rendering, input validation, and loading states
- **Official OpenAI support**: Both SDKs maintained by OpenAI with aligned release cycles and documentation
- **Event streaming built-in**: ChatKit Python SDK handles streaming responses without custom WebSocket implementation
- **Constitution compliance**: Uses mandated technologies (OpenAI Agents/ChatKit SDKs per constitution Principle 3)
- **Separation of concerns**: Agents SDK focuses on intelligence, ChatKit focuses on UX/presentation
- **Easier testing**: Test RAG logic (Agents) separately from UI rendering (ChatKit)

### Negative

- **Dual SDK learning curve**: Developers must learn both Agents SDK and ChatKit SDK APIs
- **Version coordination**: Must ensure compatible versions of Agents SDK, ChatKit Python, and ChatKit.js
- **Increased dependencies**: Two additional SDKs vs. custom RAG implementation with OpenAI API directly
- **Abstraction overhead**: May obscure underlying OpenAI API calls, harder to debug edge cases
- **Lock-in risk**: Tighter coupling to OpenAI's SDK evolution (breaking changes require updates to both SDKs)
- **Bundle size**: ChatKit.js adds ~50-100KB to frontend bundle

## Alternatives Considered

**Alternative A: OpenAI Agents SDK Only (Custom Frontend)**
- Stack: Agents SDK (backend) + custom React chat UI + raw WebSockets
- **Rejected because:**
  - Requires building chat UI from scratch (message rendering, input validation, loading states)
  - No pre-built event streaming (must implement WebSocket protocol manually)
  - 2-3 weeks additional development time for custom UI
  - ChatKit.js provides professional UX patterns (typing indicators, error handling) out-of-box

**Alternative B: ChatKit SDK Only (Custom RAG)**
- Stack: ChatKit Python + ChatKit.js + custom RAG implementation with OpenAI API
- **Rejected because:**
  - Loses Agents SDK's RAG optimizations (query reranking, tool generation)
  - Must implement conversation history management manually
  - Agents SDK provides built-in tracing and evaluation tools
  - ChatKit is designed to integrate WITH Agents SDK, not replace it

**Alternative C: Direct OpenAI API (No SDKs)**
- Stack: Raw OpenAI API calls + custom chat UI + custom RAG pipeline
- **Rejected because:**
  - Maximum flexibility but highest implementation complexity
  - Requires 4-6 weeks to build equivalent RAG quality and UI polish
  - Misses constitution mandate for OpenAI Agents/ChatKit SDKs
  - No access to Agents SDK's query optimization and reranking features
  - Custom implementations unlikely to match OpenAI's battle-tested patterns

**Alternative D: LangChain + Custom UI**
- Stack: LangChain (RAG framework) + custom React chat UI
- **Rejected because:**
  - Not constitution-compliant (constitution mandates OpenAI Agents/ChatKit SDKs)
  - LangChain adds another abstraction layer (more complexity, not less)
  - OpenAI Agents SDK specifically designed for OpenAI API (tighter integration)
  - LangChain's RAG abstractions may conflict with custom chunking strategy (ADR-0003)

## References

- Feature Spec: [specs/001-physical-ai-robotics-textbook/spec.md](../../specs/001-physical-ai-robotics-textbook/spec.md) (FR-012, FR-013, FR-019)
- Implementation Plan: [specs/001-physical-ai-robotics-textbook/plan.md](../../specs/001-physical-ai-robotics-textbook/plan.md) (Phase 0 Task 3, Technical Context)
- Research Document: [specs/001-physical-ai-robotics-textbook/research.md](../../specs/001-physical-ai-robotics-textbook/research.md) (Decision 1)
- Related ADRs:
  - ADR-0001 (Static Frontend and Dynamic Backend Separation)
  - ADR-0003 (RAG System Architecture and Content Chunking)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Principle 3: Mandated tech stack including OpenAI Agents/ChatKit SDKs)
- External Documentation:
  - [OpenAI Agents SDK](https://openai.github.io/openai-agents-python/)
  - [ChatKit SDK Documentation](https://platform.openai.com/docs/guides/chatkit)
  - [ChatKit Python SDK GitHub](https://github.com/openai/chatkit-python)
