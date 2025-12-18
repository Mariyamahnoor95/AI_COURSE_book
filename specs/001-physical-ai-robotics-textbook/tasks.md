---
description: "Implementation tasks for Physical AI & Humanoid Robotics Interactive Textbook"
---

# Tasks: Physical AI & Humanoid Robotics Interactive Textbook

**Input**: Design documents from `/specs/001-physical-ai-robotics-textbook/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/chatbot-api.yaml

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend (Docusaurus)**: `docs/`, `src/`, `static/`, root-level config files
- **Backend (FastAPI)**: `chatbot-backend/src/`, `chatbot-backend/tests/`
- **Deployment**: `.github/workflows/`, `chatbot-backend/scripts/`

---

## Phase 1: Infrastructure Setup

**Purpose**: Project initialization, environment configuration, and deployment scaffolding

**Dependencies**: None (can start immediately)

- [ ] T001 Initialize Docusaurus project with configuration in docusaurus.config.js
  - **Acceptance**: `npm start` runs successfully, homepage accessible at http://localhost:3000
  - **Maps to**: FR-013, FR-019

- [ ] T002 Configure Docusaurus sidebar structure in sidebars.js per ADR-0002
  - **Acceptance**: Sidebar shows 4 modules (Foundations, Module 1-4) with nested week/chapter structure
  - **Maps to**: FR-001, FR-002, FR-023

- [ ] T003 [P] Initialize FastAPI backend project structure in chatbot-backend/
  - **Acceptance**: Directory structure created (src/, tests/, scripts/), requirements.txt exists
  - **Maps to**: FR-015

- [ ] T004 [P] Create .env.example with required API keys in chatbot-backend/
  - **Acceptance**: Template includes OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL
  - **Maps to**: FR-016, FR-017, FR-018

- [ ] T005 [P] Set up Qdrant Cloud Free Tier collection for content embeddings
  - **Acceptance**: Collection "textbook_chunks" created with 1536-dim vectors, accessible via API key
  - **Maps to**: FR-017

- [ ] T006 [P] Set up Neon Postgres database schema in chatbot-backend/scripts/setup_db.py
  - **Acceptance**: Tables created: chat_sessions, chat_messages (per data-model.md)
  - **Maps to**: FR-018

- [ ] T007 [P] Create GitHub Actions workflow for GitHub Pages deployment in .github/workflows/deploy-docs.yml
  - **Acceptance**: Workflow triggers on push to main, builds Docusaurus, deploys to gh-pages branch
  - **Maps to**: FR-013, SC-013

- [ ] T008 [P] Configure Render web service for FastAPI backend deployment
  - **Acceptance**: Service created, environment variables set, deploys on push to main
  - **Maps to**: FR-015

---

## Phase 2: Foundational Components

**Purpose**: Core backend infrastructure and content ingestion pipeline (MUST complete before user stories)

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

**Dependencies**: Phase 1 complete

- [X] T009 Implement content chunking utility in chatbot-backend/src/utils/chunking.py
  - **Acceptance**: Semantic chunking with 400-600 tokens, 60-token overlap, respects H2/H3 boundaries per research.md
  - **Maps to**: FR-017 (chunking for embeddings)

- [X] T010 Implement embedding service in chatbot-backend/src/services/embeddings.py
  - **Acceptance**: Generates 1536-dim vectors using OpenAI text-embedding-3-small per research.md
  - **Maps to**: FR-016, FR-017

- [X] T011 Implement vector database client in chatbot-backend/src/services/vector_db.py
  - **Acceptance**: Insert, search, and delete operations work with Qdrant collection
  - **Maps to**: FR-017

- [X] T012 Implement chat history service in chatbot-backend/src/services/chat_history.py
  - **Acceptance**: CRUD operations for chat_sessions and chat_messages tables (Neon Postgres)
  - **Maps to**: FR-018, FR-022

- [X] T013 Create content ingestion script in chatbot-backend/scripts/ingest_content.py
  - **Acceptance**: Reads docs/ Markdown files, chunks content, generates embeddings, stores in Qdrant with metadata
  - **Maps to**: FR-017

- [X] T013a Add pre/post-ingestion validation to content ingestion script
  - **File**: chatbot-backend/scripts/ingest_content.py
  - **Acceptance**: Pre-flight checks (docs/ exists, Markdown parseable, valid frontmatter), token limit validation (warn if <400 or >600, fail if >800), post-ingestion verification (expected vs. actual chunk count, fail if <90%), exit codes (0=success, 1=warnings, 2=failures)
  - **Dependencies**: T013
  - **Maps to**: FR-017
  - **Rationale**: Catches content errors before deployment

- [X] T014 Implement FastAPI main app with CORS and health check in chatbot-backend/src/main.py
  - **Acceptance**: `uvicorn src.main:app` starts server, /health endpoint returns 200 OK
  - **Maps to**: FR-015

- [ ] T014a Add OpenAPI contract validation to FastAPI backend
  - **File**: chatbot-backend/src/main.py
  - **Acceptance**: FastAPI automatically validates request/response schemas against contracts/chatbot-api.yaml using openapi-spec-validator, invalid requests return 422 with schema errors
  - **Dependencies**: T014
  - **Maps to**: FR-015
  - **Rationale**: Ensures frontend/backend contract compliance at runtime

- [ ] T014b Add health check polling and cold-start UX in ChatbotWidget
  - **Files**: src/components/ChatbotWidget/ChatbotWidget.tsx, src/components/ChatbotWidget/LoadingState.tsx
  - **Acceptance**: Before first message, widget polls GET /health every 5s until 200 OK, displays "⏳ Waking up chatbot (30-60s)..." with animated spinner, after 90s timeout shows "Backend unavailable - try again in 2 minutes"
  - **Dependencies**: T014 (health endpoint), T031 (ChatbotWidget)
  - **Maps to**: FR-007, SC-002
  - **Rationale**: Addresses Render Free Tier 30-60s cold start without scope change

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Core Textbook Content Access (Priority: P1) 🎯 MVP

**Goal**: Deliver complete textbook content covering 4 modules across 13 weeks with accessible navigation

**Independent Test**: Navigate through all 4 modules, verify 23+ chapters exist with proper structure, confirm all learning objectives covered

**Dependencies**: Phase 2 complete

### Content Authoring for User Story 1

- [ ] T015 [P] [US1] Create Foundations Week 1-2 content in docs/foundations/
  - **Files**: week-01/ch00-intro-physical-ai.md, ch01-embodied-intelligence.md, week-02/ch02-sensors.md, ch03-actuators-robotics-arch.md
  - **Acceptance**: 4 chapters (1,500-2,500 words each) covering embodied AI, sensors, actuators, robotics architectures per FR-002a
  - **Maps to**: FR-001, FR-002a, SC-009

- [ ] T016 [P] [US1] Create Module 1 (ROS 2) content in docs/module-01-ros2/
  - **Files**: week-03/ch01-nodes-topics.md, ch02-services-actions.md; week-04/ch03-python-rclpy.md, ch04-tf2-transforms.md; week-05/ch05-urdf-models.md, ch06-nav2-basics.md
  - **Acceptance**: 6 chapters covering ROS 2 nodes, topics, services, Python (rclpy, tf2, sensor_msgs, geometry_msgs, nav2_simple_commander), URDF per FR-003
  - **Maps to**: FR-003, FR-024, SC-009

- [ ] T017 [P] [US1] Create Module 2 (Digital Twin) content in docs/module-02-digital-twin/
  - **Files**: week-06/ch07-gazebo-physics.md, ch08-sensor-modeling.md; week-07/ch09-unity-viz.md, ch10-digital-twin.md
  - **Acceptance**: 4 chapters covering Gazebo physics, Unity visualization, sensor modeling (LiDAR, depth, IMU), digital twin concepts per FR-004
  - **Maps to**: FR-004, SC-009

- [ ] T018 [P] [US1] Create Module 3 (NVIDIA Isaac) content in docs/module-03-isaac/
  - **Files**: week-08/ch11-isaac-sim.md, ch12-isaac-ros.md; week-09/ch13-vslam-nav2.md, ch14-perception.md; week-10/ch15-rl-sim2real.md, ch16-sensor-fusion.md
  - **Acceptance**: 6 chapters covering Isaac Sim, Isaac ROS, VSLAM, Nav2, RL, sim-to-real per FR-005
  - **Maps to**: FR-005, SC-009

- [ ] T019 [P] [US1] Create Module 4 (VLA) content in docs/module-04-vla/
  - **Files**: week-11/ch17-humanoid-urdf.md, ch18-joint-control.md; week-12/ch19-grasping.md, ch20-walking-gaits.md; week-13/ch21-whisper-voice.md, ch22-llm-planning.md, ch23-vla-integration.md
  - **Acceptance**: 6 chapters covering humanoid URDF, joint control, grasping, walking gaits, Whisper, LLM, VLA integration per FR-006, FR-006a
  - **Maps to**: FR-006, FR-006a, SC-009

- [ ] T020 [P] [US1] Create code examples for ROS 2 chapters in static/code/
  - **Acceptance**: 10-15 Python code examples using rclpy, executable and well-commented per FR-024
  - **Maps to**: FR-024, SC-010

- [ ] T021 [P] [US1] Create visual diagrams for textbook in static/img/
  - **Acceptance**: 15-25 diagrams (ROS 2 architecture, sensor data flow, Isaac components, VLA integration) per FR-025
  - **Maps to**: FR-025

- [ ] T022 [US1] Run content ingestion to populate vector database
  - **Acceptance**: 230+ chunks embedded and indexed in Qdrant, metadata includes module/week/chapter/heading/URL
  - **Dependencies**: T015-T019 (content must exist), T013 (ingestion script)
  - **Maps to**: FR-017

- [ ] T023 [US1] Validate Markdown linting and link checking
  - **Acceptance**: All Markdown files pass linting, no broken internal links
  - **Dependencies**: T015-T019
  - **Maps to**: FR-014

- [ ] T024 [US1] Deploy textbook to GitHub Pages
  - **Acceptance**: Site accessible at https://[username].github.io/[repo], all 23+ chapters navigable within 3 clicks per SC-001
  - **Dependencies**: T023
  - **Maps to**: FR-013, SC-001, SC-013

**Checkpoint**: User Story 1 complete - Full textbook content deployed and accessible

---

## Phase 4: User Story 2 - RAG Chatbot Query Interface (Priority: P2)

**Goal**: Embed conversational chatbot that answers questions from textbook content with citations

**Independent Test**: Ask chatbot 10 diverse questions about textbook topics, verify responses cite textbook sections and refuse out-of-scope queries

**Dependencies**: Phase 3 complete (textbook content must exist and be ingested)

### Backend Implementation for User Story 2

- [ ] T025 [P] [US2] Create data models in chatbot-backend/src/models/
  - **Files**: chat.py (ChatSession, ChatMessage), content.py (ContentChunk)
  - **Acceptance**: Models match data-model.md schema (session_id, role, content, citations, metadata)
  - **Maps to**: FR-018, FR-022

- [ ] T026 [P] [US2] Implement RAG service in chatbot-backend/src/services/rag.py
  - **Acceptance**: Query embedding → Qdrant search (top-k=5-10) → OpenAI Agents SDK → Response with citations
  - **Dependencies**: T010 (embeddings), T011 (vector DB)
  - **Maps to**: FR-008, FR-016, FR-020, SC-002

- [ ] T027 [US2] Implement POST /chat endpoint in chatbot-backend/src/api/chatbot.py
  - **Acceptance**: Accepts request per contracts/chatbot-api.yaml, returns response with citations, <3s response time per SC-002
  - **Dependencies**: T026 (RAG service), T012 (chat history)
  - **Maps to**: FR-007, FR-008, FR-020, FR-022, SC-002, SC-005

- [ ] T028 [P] [US2] Implement out-of-scope query rejection in chatbot-backend/src/services/rag.py
  - **Acceptance**: Queries unrelated to Physical AI/Humanoid Robotics return refusal message per FR-008, 100% accuracy per SC-003
  - **Dependencies**: T026
  - **Maps to**: FR-008, SC-003

- [ ] T029 [P] [US2] Implement citation formatting in chatbot-backend/src/services/rag.py
  - **Acceptance**: Citations formatted as "📚 **Sources:** [Chapter: Heading](URL)" per research.md, 90% of responses include citations per SC-005
  - **Dependencies**: T026
  - **Maps to**: FR-020, SC-005

- [ ] T030 [P] [US2] Implement rate limiting (10 queries/hour/user) in chatbot-backend/src/middleware/
  - **Acceptance**: Exceeding limit returns 429 error per contracts/chatbot-api.yaml
  - **Maps to**: FR-021, SC-006

### Frontend Implementation for User Story 2

- [ ] T031 [P] [US2] Create ChatbotWidget component in src/components/ChatbotWidget/
  - **Files**: ChatbotWidget.tsx, ChatInput.tsx, MessageList.tsx, styles.module.css
  - **Acceptance**: Fixed bottom-right floating widget, collapsible, message input/display with citations
  - **Maps to**: FR-007

- [ ] T032 [US2] Integrate ChatKit.js for message rendering in src/components/ChatbotWidget/
  - **Acceptance**: ChatKit.js renders assistant messages with clickable citation links per ADR-0005
  - **Dependencies**: T031
  - **Maps to**: FR-016

- [ ] T033 [US2] Swizzle Docusaurus Root component to embed ChatbotWidget in src/theme/Root.tsx
  - **Acceptance**: ChatbotWidget appears on all pages, connects to FastAPI backend per research.md
  - **Dependencies**: T031
  - **Maps to**: FR-007

- [ ] T034 [US2] Connect ChatbotWidget to FastAPI backend via REST API
  - **Acceptance**: User sends message → POST /chat → Displays response with clickable citations
  - **Dependencies**: T027, T033
  - **Maps to**: FR-007, SC-002

- [ ] T035 [US2] Deploy backend to Render and update frontend API URL
  - **Acceptance**: Production chatbot connects to Render backend, responds to queries in <3s per SC-002
  - **Dependencies**: T027, T034
  - **Maps to**: FR-015, SC-002

**Checkpoint**: User Story 2 complete - Chatbot functional with full-textbook context and citations

---

## Phase 5: User Story 3 - Context-Specific Chatbot Queries (Priority: P3)

**Goal**: Enable students to select specific modules or chapters for focused chatbot responses

**Independent Test**: Select "Module 1: ROS 2" context, ask question, verify response only cites Module 1 content with 95% accuracy per SC-004

**Dependencies**: Phase 4 complete (chatbot must exist)

### Backend Implementation for User Story 3

- [ ] T036 [P] [US3] Implement GET /context/modules endpoint in chatbot-backend/src/api/context.py
  - **Acceptance**: Returns list of 4 modules with id, name, chapter_count, week_range per contracts/chatbot-api.yaml
  - **Maps to**: FR-010

- [ ] T037 [P] [US3] Implement GET /context/chapters endpoint in chatbot-backend/src/api/context.py
  - **Acceptance**: Returns chapters optionally filtered by module_id per contracts/chatbot-api.yaml
  - **Maps to**: FR-010

- [ ] T038 [P] [US3] Implement POST /sessions endpoint in chatbot-backend/src/api/chatbot.py
  - **Acceptance**: Creates session with context_selection (mode: full_textbook|module|chapter, selected IDs) per contracts/chatbot-api.yaml
  - **Dependencies**: T025 (data models), T012 (chat history)
  - **Maps to**: FR-010, FR-018

- [ ] T039 [US3] Update RAG service to filter by context_selection in chatbot-backend/src/services/rag.py
  - **Acceptance**: Qdrant search filtered by selected module/chapter IDs when mode != full_textbook per data-model.md
  - **Dependencies**: T026 (RAG service), T038 (sessions)
  - **Maps to**: FR-010, SC-004

- [ ] T040 [P] [US3] Implement GET /sessions/{session_id}/history endpoint in chatbot-backend/src/api/chatbot.py
  - **Acceptance**: Returns last 10 messages with role/content/citations per contracts/chatbot-api.yaml
  - **Dependencies**: T012 (chat history), T038 (sessions)
  - **Maps to**: FR-022, SC-012

### Frontend Implementation for User Story 3

- [ ] T041 [P] [US3] Create ContextSelector component in src/components/ChatbotWidget/
  - **Files**: ContextSelector.tsx
  - **Acceptance**: Dropdown showing "Full Textbook", 4 modules, and chapters (filtered by module)
  - **Maps to**: FR-010, SC-011

- [ ] T042 [US3] Integrate ContextSelector into ChatbotWidget
  - **Acceptance**: User selects context → Creates session with selected IDs → Chatbot queries respect context, switchable within 2 clicks per SC-011
  - **Dependencies**: T036, T037, T038, T039, T041
  - **Maps to**: FR-010, SC-004, SC-011

**Checkpoint**: User Story 3 complete - Context-specific querying functional with 95% accuracy

---

## Phase 6: User Story 4 - Progressive Capstone Project Guidance (Priority: P4)

**Goal**: Provide structured assessment descriptions and rubrics for 4 projects (ROS 2, Gazebo, Isaac, Capstone)

**Independent Test**: Access each of 4 assessment pages, verify clear requirements, starter code references, and success criteria exist

**Dependencies**: Phase 3 complete (textbook content must exist)

### Content Authoring for User Story 4

- [ ] T043 [P] [US4] Create ROS 2 project assessment in docs/module-01-ros2/assessment-ros2-project.md
  - **Acceptance**: Clear requirements for multi-node ROS 2 system, starter code references, success criteria, due Week 5 per FR-011
  - **Maps to**: FR-011, SC-007

- [ ] T044 [P] [US4] Create Gazebo simulation assessment in docs/module-02-digital-twin/assessment-gazebo-sim.md
  - **Acceptance**: Instructions for digital twin with sensor integration, physics simulation, due Week 7 per FR-011
  - **Maps to**: FR-011

- [ ] T045 [P] [US4] Create Isaac perception pipeline assessment in docs/module-03-isaac/assessment-isaac-pipeline.md
  - **Acceptance**: Guidance on VSLAM, Nav2 integration, sensor fusion, due Week 10 per FR-011
  - **Maps to**: FR-011

- [ ] T046 [P] [US4] Create capstone project assessment in docs/module-04-vla/assessment-capstone.md
  - **Acceptance**: Comprehensive checklist with voice understanding (Whisper), task planning (LLM), navigation, perception, manipulation, due Week 13 per FR-011, FR-012
  - **Maps to**: FR-011, FR-012, SC-008

- [ ] T047 [US4] Update sidebar.js to include assessment links
  - **Acceptance**: Each module shows 📝 Assessment link at end of chapter list per research.md sidebar structure
  - **Dependencies**: T043-T046
  - **Maps to**: FR-023

- [ ] T048 [US4] Re-run content ingestion for assessment pages
  - **Acceptance**: Assessment content embedded in Qdrant, chatbot can answer questions about projects
  - **Dependencies**: T043-T046, T013 (ingestion script)
  - **Maps to**: FR-017

**Checkpoint**: User Story 4 complete - All 4 assessments accessible with clear guidance

---

## Phase 7: Testing & Validation

**Purpose**: End-to-end testing, performance validation, and quality assurance

**Dependencies**: Phases 3-6 complete (all user stories implemented)

- [ ] T049 [P] Create RAG accuracy tests in chatbot-backend/tests/test_rag.py
  - **Acceptance**: 20+ known Q&A pairs tested, 90%+ accuracy, responses include citations per SC-005
  - **Maps to**: SC-003, SC-004, SC-005

- [ ] T050 [P] Create API endpoint tests in chatbot-backend/tests/test_api.py
  - **Acceptance**: All 5 endpoints tested (POST /chat, GET /context/modules, GET /context/chapters, POST /sessions, GET /sessions/{id}/history)
  - **Maps to**: FR-015

- [ ] T050a [P] Add contract compliance tests for frontend/backend integration
  - **File**: tests/e2e/test_contract_compliance.py
  - **Acceptance**: Use schemathesis or openapi-core to generate test requests from contracts/chatbot-api.yaml, validate all 5 endpoints return responses matching OpenAPI schemas, test error cases (400, 429, 500), 100% schema compliance
  - **Dependencies**: T050, contracts/chatbot-api.yaml
  - **Maps to**: FR-015
  - **Rationale**: Prevents frontend/backend drift over time

- [ ] T051 [P] Create Playwright end-to-end tests in tests/e2e/
  - **Acceptance**: Tests for navigation (SC-001), chatbot interaction (SC-002), context switching (SC-011), concurrent users (SC-006)
  - **Maps to**: SC-001, SC-002, SC-006, SC-011

- [ ] T052 Test chatbot response time performance
  - **Acceptance**: 95% of responses <3 seconds per SC-002
  - **Maps to**: SC-002

- [ ] T053 Test concurrent user load (50+ users)
  - **Acceptance**: System supports 50+ concurrent users without degradation >5 seconds per SC-006
  - **Maps to**: SC-006

- [ ] T054 Validate out-of-scope query rejection
  - **Acceptance**: 100% of off-topic queries refused per SC-003
  - **Maps to**: SC-003

- [ ] T055 Validate code example executability
  - **Acceptance**: All ROS 2 code examples run successfully in specified environments per SC-010
  - **Maps to**: SC-010

- [ ] T056 Validate conversation context retention
  - **Acceptance**: Follow-up questions answered correctly 90% of the time per SC-012
  - **Maps to**: SC-012

---

## Phase 8: Documentation & Polish

**Purpose**: Developer documentation, deployment guides, and final refinements

**Dependencies**: Phase 7 complete

- [ ] T057 [P] Update README.md with project overview and setup instructions
  - **Acceptance**: README includes project description, tech stack, local setup (7 steps per quickstart.md), deployment instructions
  - **Maps to**: FR-014

- [ ] T058 [P] Create instructor guide for course delivery
  - **Acceptance**: Guide explains weekly content structure, assessment timing, chatbot usage tips
  - **Maps to**: FR-002, FR-011

- [ ] T059 [P] Create student onboarding documentation
  - **Acceptance**: Doc explains textbook navigation, chatbot features (full-textbook vs. context-specific), assessment deadlines
  - **Maps to**: SC-001, SC-011

- [ ] T060 [P] Add usage monitoring dashboard for Qdrant/Neon/OpenAI
  - **Acceptance**: Script or dashboard shows storage usage (Qdrant, Neon), API costs (OpenAI)
  - **Maps to**: FR-017, FR-018

- [ ] T061 Perform final accessibility audit (WCAG compliance)
  - **Acceptance**: Docusaurus pages meet WCAG 2.1 Level AA standards (keyboard navigation, alt text, ARIA labels)
  - **Maps to**: SC-013

- [ ] T062 Optimize images and diagrams for performance
  - **Acceptance**: All images <500KB, page load time <2 seconds per plan.md performance goals
  - **Maps to**: FR-025

---

## Phase 9: Deployment & Launch Readiness

**Purpose**: Final deployment, smoke tests, and production validation

**Dependencies**: Phase 8 complete

- [ ] T063 Deploy final textbook build to GitHub Pages
  - **Acceptance**: Production site accessible, all 23+ chapters load correctly
  - **Maps to**: FR-013, SC-013

- [ ] T064 Deploy final backend build to Render
  - **Acceptance**: Production API responds to /health, /chat endpoints functional
  - **Maps to**: FR-015

- [ ] T065 Run production smoke tests
  - **Acceptance**: Navigate to 10 random chapters (load time <2s), ask chatbot 5 questions (response <3s, citations included)
  - **Maps to**: SC-001, SC-002, SC-005

- [ ] T066 Verify GitHub Actions deployment pipeline
  - **Acceptance**: Push to main triggers auto-deploy, textbook updates live within 5 minutes
  - **Maps to**: FR-013

- [ ] T067 Verify Render auto-deploy for backend
  - **Acceptance**: Push to main triggers backend redeploy, API available within 10 minutes
  - **Maps to**: FR-015

- [ ] T068 Create backup and recovery documentation
  - **Acceptance**: Documented procedures for backing up Qdrant snapshots, Neon database, restoring from backup
  - **Maps to**: FR-017, FR-018

---

## Coverage Validation

### Functional Requirements Coverage (25 FRs)

| FR | Tasks | Status |
|----|-------|--------|
| FR-001 | T015-T019 | ✅ 4 modules content |
| FR-002 | T015-T019 | ✅ 13-week structure |
| FR-002a | T015 | ✅ Weeks 1-2 foundations |
| FR-003 | T016 | ✅ ROS 2 coverage |
| FR-004 | T017 | ✅ Simulation coverage |
| FR-005 | T018 | ✅ Isaac coverage |
| FR-006 | T019 | ✅ VLA coverage |
| FR-006a | T019 | ✅ Weeks 11-12 humanoid |
| FR-007 | T031, T033 | ✅ Embedded chatbot |
| FR-008 | T027, T028 | ✅ Textbook-only RAG |
| FR-009 | T027, T039 | ✅ Full-textbook mode |
| FR-010 | T036-T039, T042 | ✅ Context selection |
| FR-011 | T043-T046 | ✅ 4 assessments |
| FR-012 | T046 | ✅ Capstone requirements |
| FR-013 | T007, T024, T063 | ✅ GitHub Pages |
| FR-014 | T015-T019 | ✅ Markdown content |
| FR-015 | T003, T014, T064 | ✅ FastAPI backend |
| FR-016 | T026, T032 | ✅ Agents/ChatKit SDK |
| FR-017 | T005, T009-T011, T013 | ✅ Qdrant embeddings |
| FR-018 | T006, T012, T025 | ✅ Neon chat history |
| FR-019 | T001 | ✅ Docusaurus rendering |
| FR-020 | T027, T029 | ✅ Citation requirement |
| FR-021 | T030, T053 | ✅ Concurrent users |
| FR-022 | T012, T027, T040 | ✅ Conversation context |
| FR-023 | T002, T047 | ✅ Navigation structure |
| FR-024 | T020 | ✅ ROS 2 code examples |
| FR-025 | T021 | ✅ Visual diagrams |

**Coverage**: 25/25 FRs = 100% ✅

### Success Criteria Coverage (13 SCs)

| SC | Tasks | Status |
|----|-------|--------|
| SC-001 | T002, T024, T065 | ✅ 3-click navigation |
| SC-002 | T027, T052, T065 | ✅ <3s response time |
| SC-003 | T028, T054 | ✅ 100% out-of-scope refusal |
| SC-004 | T039, T042 | ✅ 95% context accuracy |
| SC-005 | T029, T049 | ✅ 90% citation rate |
| SC-006 | T030, T053 | ✅ 50+ concurrent users |
| SC-007 | T043 | ✅ ROS 2 project completion |
| SC-008 | T046 | ✅ Capstone completion |
| SC-009 | T015-T019 | ✅ Learning outcomes coverage |
| SC-010 | T020, T055 | ✅ Executable code examples |
| SC-011 | T041, T042 | ✅ Context switching <2 clicks |
| SC-012 | T040, T056 | ✅ 90% conversation context |
| SC-013 | T007, T024, T063 | ✅ GitHub Pages deployment |

**Coverage**: 13/13 SCs = 100% ✅

---

## Dependency Graph

```
Phase 1 (Infrastructure) → Phase 2 (Foundational)
                                  ↓
                     ┌────────────┼────────────┐
                     ↓            ↓            ↓
              Phase 3 (US1) → Phase 4 (US2) → Phase 5 (US3)
                     ↓
              Phase 6 (US4)
                     ↓
              Phase 7 (Testing)
                     ↓
              Phase 8 (Documentation)
                     ↓
              Phase 9 (Deployment)
```

**Critical Path**: Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 5 → Phase 7 → Phase 8 → Phase 9

**Parallel Opportunities**:
- Phase 2: Tasks T009-T014 can run in parallel
- Phase 3: Tasks T015-T021 (content authoring) can run in parallel
- Phase 4: Backend tasks (T025-T030) and Frontend tasks (T031-T033) can run in parallel
- Phase 5: Backend tasks (T036-T040) and Frontend task (T041) can run in parallel
- Phase 6: Tasks T043-T046 (assessment authoring) can run in parallel
- Phase 7: All test tasks (T049-T056) can run in parallel
- Phase 8: Documentation tasks (T057-T062) can run in parallel

---

## Implementation Strategy

### MVP Scope (Phase 3 Only)

**Minimum Viable Product** = User Story 1 (Core Textbook Content Access)

**Deliverable**: Static textbook with 23+ chapters, full navigation, no chatbot

**Validation**: Student can read all content, navigate modules, access learning materials

**Timeline Estimate**: Phase 1 (1 week) + Phase 2 (1 week) + Phase 3 (4-6 weeks content authoring)

### Incremental Delivery

1. **Sprint 1-2**: Phases 1-2 (Infrastructure + Foundation) → 2 weeks
2. **Sprint 3-6**: Phase 3 (US1 - Textbook content) → 4-6 weeks
3. **Sprint 7-8**: Phase 4 (US2 - RAG Chatbot) → 2 weeks
4. **Sprint 9**: Phase 5 (US3 - Context Selection) → 1 week
5. **Sprint 10**: Phase 6 (US4 - Assessments) → 1 week
6. **Sprint 11**: Phase 7 (Testing & Validation) → 1 week
7. **Sprint 12**: Phases 8-9 (Documentation & Deployment) → 1 week

**Total Estimated Timeline**: 12-14 weeks

### Parallel Execution Examples

**Phase 3 Parallelization** (4 developers):
- Dev 1: T015 (Foundations content)
- Dev 2: T016 (Module 1 content)
- Dev 3: T017 (Module 2 content)
- Dev 4: T018 (Module 3 content)
- All: T019 (Module 4 content) → Sequential after others complete

**Phase 4 Parallelization** (2 developers):
- Dev 1 (Backend): T025 → T026 → T027 → T028 → T029 → T030
- Dev 2 (Frontend): T031 → T032 → T033
- Together: T034 (integration), T035 (deployment)

---

## Notes

- **Content authoring** (T015-T021) is the longest phase; consider hiring technical writers or subject matter experts
- **Test tasks** (Phase 7) can be written incrementally as features complete, not required upfront
- **Free tier limits**: Monitor Qdrant (1GB), Neon (0.5GB), OpenAI API costs ($20-50/month estimated) per research.md
- **Cold start mitigation**: T014b implements health check polling and "Waking up chatbot..." UX for Render Free Tier 30-60s cold starts
- **Content ingestion**: T013a adds validation checks; run T013 (ingest_content.py) after any Markdown content changes to update RAG embeddings
- **Contract validation**: T014a (runtime validation) and T050a (compliance tests) ensure frontend/backend API compatibility
- **Deployment automation**: GitHub Actions (T007) and Render auto-deploy (T008) enable continuous delivery

---

## References

- [spec.md](./spec.md) - User stories and functional requirements
- [plan.md](./plan.md) - Technical stack and project structure
- [data-model.md](./data-model.md) - Entity definitions and relationships
- [contracts/chatbot-api.yaml](./contracts/chatbot-api.yaml) - OpenAPI specification
- [research.md](./research.md) - Phase 0 technical decisions
- [quickstart.md](./quickstart.md) - Developer setup and workflows
