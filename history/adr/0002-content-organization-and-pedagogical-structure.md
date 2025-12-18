# ADR-0002: Content Organization and Pedagogical Structure

> **Scope**: Directory structure and chapter organization for the textbook, impacting content authoring workflow, navigation design, and RAG context selection capabilities.

- **Status:** Accepted
- **Date:** 2025-12-16
- **Feature:** 001-physical-ai-robotics-textbook
- **Context:** The textbook must organize 23+ chapters across 4 modules and 13 weeks. The structure must support both pedagogical progression (week-by-week learning) and modular reference (topic-based lookup), while also enabling the chatbot to provide context-specific answers when users limit queries to specific modules or chapters.

## Decision

**Adopt a hybrid directory structure with foundations + module-based organization:**

```
docs/
├── intro.md                          # Course overview
├── foundations/                      # Weeks 1-2 (foundational concepts)
│   ├── week-01/
│   │   ├── ch00-intro-physical-ai.md
│   │   └── ch01-embodied-intelligence.md
│   └── week-02/
│       ├── ch02-sensors.md
│       └── ch03-actuators-robotics-arch.md
├── module-01-ros2/                   # Module 1 (Weeks 3-5)
│   ├── week-03/
│   │   ├── ch01-nodes-topics.md
│   │   └── ch02-services-actions.md
│   ├── week-04/
│   │   ├── ch03-python-rclpy.md
│   │   └── ch04-tf2-transforms.md
│   ├── week-05/
│   │   ├── ch05-urdf-models.md
│   │   └── ch06-nav2-basics.md
│   └── assessment-ros2-project.md
├── module-02-digital-twin/           # Module 2 (Weeks 6-7)
│   ├── week-06/
│   │   ├── ch07-gazebo-physics.md
│   │   └── ch08-sensor-modeling.md
│   ├── week-07/
│   │   ├── ch09-unity-viz.md
│   │   └── ch10-digital-twin.md
│   └── assessment-gazebo-sim.md
├── module-03-isaac/                  # Module 3 (Weeks 8-10)
│   ├── week-08/, week-09/, week-10/  # 6 chapters total
│   └── assessment-isaac-pipeline.md
└── module-04-vla/                    # Module 4 (Weeks 11-13)
    ├── week-11/, week-12/, week-13/  # 6 chapters total
    └── assessment-capstone.md
```

**Naming Conventions:**
- Directories: `module-XX-topic` and `week-XX`
- Chapter files: `chXX-topic-name.md` (sequential numbering within module)
- Assessment files: `assessment-topic.md` at module root

**Mapping Rules:**
- Foundations: 4 chapters (2 per week)
- Modules 1-4: 4-6 chapters each, approximately 2 chapters per week
- Each module contains 2-3 week subdirectories aligned with course schedule

## Consequences

### Positive

- **Clear pedagogical flow**: Students follow week-by-week directories matching course schedule
- **Modular reference access**: Advanced students or instructors can jump directly to specific modules (e.g., "I just need ROS 2 content")
- **Chatbot context mapping**: Directory structure directly maps to context selection modes (full textbook, module, or chapter-specific queries per FR-016)
- **Maintainability**: Assessments colocated with module content; easy to update module-specific material
- **Scalability**: New modules or weeks can be added without restructuring existing content
- **Docusaurus integration**: Sidebar automatically organizes by modules and weeks using directory structure
- **Metadata extraction**: File paths provide automatic metadata for RAG embeddings (module, week, chapter)

### Negative

- **Nested depth**: Three levels of nesting (module → week → chapter) may complicate URL structure
- **Chapter renumbering**: If chapters are added mid-module, subsequent chapter numbers may need updates
- **Foundations separation**: Splitting foundations from modules creates inconsistency (foundations/ vs. module-XX/)
- **Assessment placement**: Module-level assessments at root may be harder to discover than week-level placement
- **Weekly alignment rigidity**: If course schedule changes, directory structure requires refactoring

## Alternatives Considered

**Alternative A: Flat Module Structure (No Week Subdirectories)**
```
docs/
├── module-01-ros2/
│   ├── ch01-nodes-topics.md
│   ├── ch02-services-actions.md
│   ├── ...
│   └── assessment-ros2-project.md
```
- **Rejected because:**
  - Loses week-by-week pedagogical flow
  - Harder to map chapters to specific course weeks (FR-002)
  - Students can't easily see "Week 3 content" without consulting external mapping
  - Less intuitive for sequential learning progression

**Alternative B: Week-Only Structure (No Module Grouping)**
```
docs/
├── week-01/, week-02/, ..., week-13/
```
- **Rejected because:**
  - Difficult to reference by topic (e.g., "all ROS 2 content" spans multiple weeks)
  - Chatbot context selection by module (FR-016) requires manual week-to-module mapping
  - Assessment placement unclear (belongs to week or topic?)
  - Breaks topical coherence for advanced reference use

**Alternative C: Completely Flat Structure**
```
docs/
├── ch01-intro-physical-ai.md
├── ch02-embodied-intelligence.md
├── ...
```
- **Rejected because:**
  - No visual organization for 23+ chapters
  - Impossible to implement chatbot context selection by module/chapter
  - Violates SC-001 (navigate within 3 clicks) for deeply nested content
  - Poor Docusaurus sidebar UX (single long list)

**Alternative D: Topic-Based Structure (No Week Alignment)**
```
docs/
├── fundamentals/, ros2/, simulation/, isaac/, vla/
```
- **Rejected because:**
  - Doesn't align with 13-week course structure (FR-002)
  - Students can't follow weekly schedule without external syllabus
  - Assessment timing (Week 5, 7, 10, 13 per FR-011) requires extra mapping

## References

- Feature Spec: [specs/001-physical-ai-robotics-textbook/spec.md](../../specs/001-physical-ai-robotics-textbook/spec.md) (FR-002, FR-002a, FR-006a, FR-016)
- Implementation Plan: [specs/001-physical-ai-robotics-textbook/plan.md](../../specs/001-physical-ai-robotics-textbook/plan.md) (Project Structure section, Phase 0 Task 1)
- Related ADRs: ADR-0003 (RAG System Architecture - impacts metadata schema)
- Clarification Session: [history/prompts/001-physical-ai-robotics-textbook/0002-clarify-learning-outcomes-and-course-structure.spec.prompt.md](../prompts/001-physical-ai-robotics-textbook/0002-clarify-learning-outcomes-and-course-structure.spec.prompt.md) (Q5: Chapter structure decision)
