# Implementation Plan: Physical AI & Humanoid Robotics Interactive Textbook

**Branch**: `001-physical-ai-robotics-textbook` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-robotics-textbook/spec.md`

## Summary

Deliver a university-level interactive textbook on Physical AI & Humanoid Robotics with an embedded RAG chatbot. The textbook covers 4 modules (ROS 2, Digital Twin, AI Robot Brain, Vision-Language-Action) across 13 weeks using Docusaurus for content delivery and GitHub Pages for hosting. The RAG chatbot, built with FastAPI, OpenAI Agents/ChatKit, Qdrant Cloud, and Neon Postgres, provides context-aware Q&A grounded exclusively in textbook content.

## Technical Context

**Language/Version**: Python 3.10+, Node.js 18+ (Docusaurus), Markdown (content authoring)
**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18+
- Backend: FastAPI 0.104+, OpenAI Python SDK 1.x, Qdrant Client, Psycopg3
- RAG: OpenAI Agents SDK or ChatKit SDK, text-embedding-3-small

**Storage**:
- Vector DB: Qdrant Cloud (Free Tier) for content embeddings
- Relational DB: Neon Serverless Postgres (Free Tier) for chat history and user sessions
- Static Content: GitHub Pages (Docusaurus build output)

**Testing**:
- Content validation: Markdown linting, link checking
- Chatbot: pytest for API endpoints, RAG accuracy testing with known Q&A pairs
- End-to-end: Playwright for textbook navigation and chatbot interaction

**Target Platform**:
- Textbook: Static website (GitHub Pages, desktop/laptop browsers)
- Chatbot Backend: Cloud-hosted FastAPI service (Vercel, Railway, or similar)

**Project Type**: Web application (static frontend + API backend)

**Performance Goals**:
- Chatbot response time: <3 seconds (SC-002)
- Page load time: <2 seconds for textbook pages
- Concurrent users: 50+ without degradation (SC-006)
- RAG retrieval: <1 second for top-k similar chunks

**Constraints**:
- Qdrant Free Tier: 1GB storage limit (requires efficient chunking strategy)
- Neon Free Tier: 0.5GB storage, compute limits
- OpenAI API budget: Requires query rate limiting and cost monitoring
- GitHub Pages: 1GB size limit, 100GB/month bandwidth soft limit
- Markdown-only content (no proprietary formats)

**Scale/Scope**:
- Content: 4 modules, ~20-26 chapters (4-6 per module), ~40-52 textbook pages
- Weekly structure: 13 weeks of content
- Assessments: 4 (ROS 2, Gazebo, Isaac, Capstone)
- Code examples: 30-50 executable snippets across modules
- Diagrams: 15-25 architectural/flow diagrams
- Expected students: 50-100 concurrent during pilot

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Audience-Centric Design**: Content tailored to undergraduate/early graduate students with basic programming and AI knowledge (per spec assumptions)

✅ **Unified Textbook & Integrated RAG Chatbot**: Plan delivers both textbook (Docusaurus) and integrated chatbot (embedded UI + FastAPI backend)

✅ **Spec-Driven Development & Defined Tech Stack**: Plan uses mandated technologies:
- Spec-Kit Plus + Claude Code (development workflow) ✅
- Docusaurus + GitHub Pages (textbook hosting) ✅
- OpenAI Agents / ChatKit SDKs (chatbot interface) ✅
- FastAPI (backend API) ✅
- Neon Serverless Postgres (chat history storage) ✅
- Qdrant Cloud Free Tier (vector embeddings) ✅

✅ **Markdown & Scope Adherence**: All textbook content authored in Markdown; scope limited to Physical AI & Humanoid Robotics per spec

**Constitution Compliance**: PASS - All principles satisfied

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-robotics-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output: Technology research and decisions
├── data-model.md        # Phase 1 output: Content and chatbot data models
├── quickstart.md        # Phase 1 output: Development setup guide
├── contracts/           # Phase 1 output: API contracts
│   └── chatbot-api.yaml # OpenAPI spec for chatbot backend
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus Static Site (Textbook)
docs/
├── intro.md                          # Homepage/course overview
├── module-01-ros2/                   # Module 1: ROS 2
│   ├── week-03/                      # Weeks 3-5 content
│   │   ├── ch01-nodes-topics.md
│   │   └── ch02-services-actions.md
│   ├── week-04/
│   │   ├── ch03-python-rclpy.md
│   │   └── ch04-tf2-transforms.md
│   ├── week-05/
│   │   ├── ch05-urdf-models.md
│   │   └── ch06-nav2-basics.md
│   └── assessment-ros2-project.md
├── module-02-digital-twin/           # Module 2: Simulation
│   ├── week-06/
│   │   ├── ch07-gazebo-physics.md
│   │   └── ch08-sensor-modeling.md
│   ├── week-07/
│   │   ├── ch09-unity-viz.md
│   │   └── ch10-digital-twin.md
│   └── assessment-gazebo-sim.md
├── module-03-isaac/                  # Module 3: NVIDIA Isaac
│   ├── week-08/
│   │   ├── ch11-isaac-sim.md
│   │   └── ch12-isaac-ros.md
│   ├── week-09/
│   │   ├── ch13-vslam-nav2.md
│   │   └── ch14-perception.md
│   ├── week-10/
│   │   ├── ch15-rl-sim2real.md
│   │   └── ch16-sensor-fusion.md
│   └── assessment-isaac-pipeline.md
├── module-04-vla/                    # Module 4: Vision-Language-Action
│   ├── week-11/
│   │   ├── ch17-humanoid-urdf.md
│   │   └── ch18-joint-control.md
│   ├── week-12/
│   │   ├── ch19-grasping.md
│   │   └── ch20-walking-gaits.md
│   ├── week-13/
│   │   ├── ch21-whisper-voice.md
│   │   ├── ch22-llm-planning.md
│   │   └── ch23-vla-integration.md
│   └── assessment-capstone.md
└── foundations/                      # Weeks 1-2 content
    ├── week-01/
    │   ├── ch00-intro-physical-ai.md
    │   └── ch01-embodied-intelligence.md
    └── week-02/
        ├── ch02-sensors.md
        └── ch03-actuators-robotics-arch.md

static/
├── img/                              # Diagrams and images
└── code/                             # Downloadable code examples

docusaurus.config.js                  # Docusaurus configuration
sidebars.js                           # Navigation structure
package.json                          # Node dependencies

# Chatbot Backend (FastAPI)
chatbot-backend/
├── src/
│   ├── main.py                       # FastAPI app entry point
│   ├── models/
│   │   ├── chat.py                   # Chat session, message models
│   │   ├── content.py                # Content chunk, embedding models
│   │   └── context.py                # Context selection models
│   ├── services/
│   │   ├── embeddings.py             # OpenAI embedding generation
│   │   ├── rag.py                    # RAG retrieval and response
│   │   ├── vector_db.py              # Qdrant client wrapper
│   │   └── chat_history.py           # Neon Postgres chat persistence
│   ├── api/
│   │   ├── chatbot.py                # /chat endpoint
│   │   └── context.py                # /context endpoint
│   └── utils/
│       ├── chunking.py               # Markdown content chunking
│       └── validators.py             # Input validation
├── scripts/
│   ├── ingest_content.py             # Batch embed textbook content
│   └── setup_db.py                   # Initialize Postgres schema
└── tests/
    ├── test_rag.py                   # RAG accuracy tests
    ├── test_api.py                   # API endpoint tests
    └── test_known_qa.py              # Known Q&A pair validation

requirements.txt                      # Python dependencies
pyproject.toml                        # Python project config

# Chatbot Frontend (Docusaurus Plugin/Component)
src/
└── components/
    └── ChatbotWidget/
        ├── ChatbotWidget.tsx         # React chatbot UI component
        ├── ChatInput.tsx             # Input field component
        ├── MessageList.tsx           # Chat history display
        ├── ContextSelector.tsx       # Chapter/module context picker
        └── styles.module.css         # Component styles

# Deployment
.github/
└── workflows/
    ├── deploy-docs.yml               # GitHub Pages deployment
    └── deploy-chatbot.yml            # Chatbot backend deployment

# Configuration
.env.example                          # Environment variable template
README.md                             # Project overview and setup
```

**Structure Decision**: Web application architecture with static frontend (Docusaurus) and separate API backend (FastAPI). Docusaurus serves textbook content via GitHub Pages. Chatbot backend is independently deployed as a cloud service. React component embedded in Docusaurus pages communicates with FastAPI backend via REST API.

## Complexity Tracking

> No constitution violations detected. All complexity is justified and minimal.

---

## Phase 0: Outline & Research

**Goal**: Resolve technical unknowns and establish implementation patterns for Docusaurus, RAG system, and deployment.

**Prerequisites**: Specification complete and constitution check passed

**Research Tasks**:

1. **Docusaurus Configuration for Course Structure**
   - Decision Needed: How to organize 13-week, 4-module content in Docusaurus sidebar
   - Research: Docusaurus docs categories, collapsible sections, custom sidebar config
   - Output: Sidebar structure mapping modules → weeks → chapters

2. **Content Chunking Strategy for RAG**
   - Decision Needed: Optimal chunk size and overlap for textbook embeddings
   - Research: Best practices for semantic chunking of technical documentation (heading-based vs. fixed-size vs. semantic)
   - Constraint: Must fit within Qdrant Free Tier 1GB limit (~20-26 chapters)
   - Output: Chunking parameters (size, overlap, metadata schema)

3. **OpenAI Agents vs. ChatKit SDK Selection**
   - Decision Needed: Which SDK to use for chatbot conversational interface
   - Research: Feature comparison, RAG integration ease, cost implications, documentation quality
   - Output: Selected SDK with rationale

4. **Chatbot Embedding in Docusaurus**
   - Decision Needed: How to embed React chatbot component in Docusaurus
   - Research: Docusaurus swizzling, custom plugins, global component injection
   - Output: Integration approach (e.g., custom theme component, plugin)

5. **Deployment Platforms**
   - Decision Needed: Where to host FastAPI chatbot backend (Vercel, Railway, Render, Fly.io)
   - Research: Free tier limits, Python support, Neon/Qdrant connectivity, cold start times
   - Output: Selected platform with cost/performance justification

6. **Citation Implementation**
   - Decision Needed: How to implement FR-020 (chatbot must cite textbook sections)
   - Research: Metadata embedding strategies, citation formatting, linking back to Docusaurus pages
   - Output: Citation metadata schema and display format

**Deliverable**: `research.md` containing all decisions with rationale, alternatives considered, and implementation patterns selected

**Completion Criteria**:
- All "NEEDS CLARIFICATION" items from Technical Context resolved
- Technology choices documented with specific versions
- Integration patterns validated with proof-of-concept code snippets where applicable
- No open technical questions blocking Phase 1 design

---

## Phase 1: Design & Contracts

**Goal**: Define data models, API contracts, and development quickstart for textbook and chatbot systems.

**Prerequisites**: `research.md` complete with all decisions documented

**Tasks**:

### 1.1 Data Model Design (`data-model.md`)

**Content Structure Models**:
- **Module**: id, name, description, order (1-4), week_range (e.g., "3-5"), chapter_count
- **Chapter**: id, module_id, week, order, title, file_path, markdown_content
- **Week**: number (1-13), title, module_id, chapter_ids[]

**Chatbot Data Models**:
- **ContentChunk**: id, chapter_id, content_text, embedding_vector (1536 dims), metadata {module, week, chapter_title, heading, page_url}
- **ChatSession**: session_id (UUID), user_id (optional), created_at, updated_at, context_selection {mode: "full" | "chapter" | "module", selected_ids: []}
- **ChatMessage**: message_id, session_id, role ("user" | "assistant"), content, citations [{chunk_id, chapter_title, url}], timestamp
- **ContextSelection**: session_id, selection_mode ("full_textbook" | "chapter" | "module"), selected_chapter_ids[], selected_module_ids[]

**Validation Rules**:
- Chunk size: 200-800 tokens (based on Phase 0 research)
- Chunk overlap: 50-100 tokens
- Embedding model: text-embedding-3-small (1536 dimensions)
- Session timeout: 24 hours of inactivity
- Message history retention: Last 10 messages per session for context

**State Transitions**:
- ChatSession: created → active → expired
- Context selection can be updated during active session

### 1.2 API Contracts (`contracts/chatbot-api.yaml`)

**Endpoints**:

```yaml
POST /api/v1/chat
  Request:
    session_id: string (optional, create if null)
    message: string (max 500 chars)
    context: {mode: "full"|"chapter"|"module", ids: string[]}
  Response:
    session_id: string
    message: string
    citations: [{chapter_title, url, chunk_preview}]
    response_time_ms: number

GET /api/v1/context/modules
  Response:
    modules: [{id, name, chapter_count}]

GET /api/v1/context/chapters?module_id=X
  Response:
    chapters: [{id, title, module_id, week}]

POST /api/v1/sessions
  Request:
    context: {mode, ids}
  Response:
    session_id: string
    created_at: timestamp

GET /api/v1/sessions/{session_id}/history
  Response:
    messages: [{role, content, citations, timestamp}]
```

**Error Responses**:
- 400: Invalid input (message too long, invalid context IDs)
- 429: Rate limit exceeded (per-user daily quota)
- 500: RAG retrieval failure or OpenAI API error

### 1.3 Quickstart Guide (`quickstart.md`)

**Local Development Setup**:
1. Prerequisites: Python 3.10+, Node.js 18+, Docker (optional for local Qdrant)
2. Clone repository and install dependencies
3. Configure `.env` with API keys (OpenAI, Qdrant Cloud, Neon Postgres)
4. Run content ingestion script to populate vector DB
5. Start FastAPI backend (`uvicorn src.main:app --reload`)
6. Start Docusaurus dev server (`npm start`)
7. Test chatbot integration at `http://localhost:3000`

**Content Authoring Workflow**:
1. Create Markdown file in appropriate module/week directory
2. Follow naming convention: `chXX-topic-name.md`
3. Include frontmatter metadata (title, week, module)
4. Run Markdown linter and link checker
5. Re-run content ingestion script to update embeddings
6. Verify chatbot can answer questions about new content

**Deployment Process**:
1. GitHub Actions triggers on push to main
2. Build Docusaurus static site
3. Deploy to GitHub Pages
4. Deploy FastAPI backend to selected platform
5. Run smoke tests (page loads, chatbot responds)

### 1.4 Agent Context Update

Run `.specify/scripts/bash/update-agent-context.sh claude` to add:
- Docusaurus 3.x patterns
- FastAPI best practices
- RAG implementation patterns
- Qdrant and Neon client usage

**Deliverables**:
- `data-model.md`: Complete entity definitions with validation rules
- `contracts/chatbot-api.yaml`: OpenAPI spec for chatbot backend
- `quickstart.md`: Development setup and authoring workflow
- Updated agent context file with project-specific patterns

**Completion Criteria**:
- All entities from spec mapped to data models
- API contracts cover all user stories (P1-P4)
- Quickstart guide tested by fresh developer setup
- No ambiguous data relationships or missing validations
- Re-run constitution check: PASS

---

## Phase 2: Task Breakdown

**Note**: This phase is executed by `/sp.tasks` command, NOT `/sp.plan`. The plan ends here.

**Goal**: Generate `tasks.md` with testable, dependency-ordered implementation tasks.

**Prerequisites**: Phase 1 complete (data-model.md, contracts/, quickstart.md)

**Expected Task Categories** (to be generated by `/sp.tasks`):
1. **Infrastructure Setup**: Docusaurus init, FastAPI project structure, DB schemas, CI/CD pipelines
2. **Content Authoring**: Write chapters for Modules 1-4, create diagrams, code examples
3. **Chatbot Backend**: Implement RAG service, API endpoints, embedding ingestion, rate limiting
4. **Chatbot Frontend**: Build React component, context selector UI, message rendering with citations
5. **Integration**: Embed chatbot in Docusaurus, connect to backend API, test E2E flows
6. **Assessment Content**: Write project descriptions, rubrics, starter code references
7. **Testing & Validation**: RAG accuracy tests, API tests, navigation tests, accessibility tests
8. **Deployment**: GitHub Pages setup, backend deployment, environment configuration
9. **Documentation**: Update README, write instructor guide, student onboarding docs

**Completion Criteria for `/sp.tasks`**:
- Each task testable and independently completable
- Dependency order: Infrastructure → Content + Backend → Frontend → Integration → Validation
- All functional requirements (FR-001 to FR-025) mapped to tasks
- All success criteria (SC-001 to SC-013) have corresponding validation tasks

---

## Next Steps

1. **Execute Phase 0**: Generate `research.md` by researching the 6 identified technical unknowns
2. **Execute Phase 1**: Create data models, API contracts, and quickstart guide based on research decisions
3. **Run `/sp.tasks`**: Generate detailed task breakdown for implementation
4. **Begin Implementation**: Follow tasks.md in dependency order, starting with infrastructure setup

**Validation Gate**: Before proceeding to `/sp.tasks`, verify:
- ✅ Constitution check still passes
- ✅ All research decisions documented and reviewed
- ✅ Data models cover all entities from spec
- ✅ API contracts support all user stories
- ✅ Quickstart guide is actionable and complete
