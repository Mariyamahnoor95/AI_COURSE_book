"""
FastAPI endpoints for chatbot interactions

Maps to: FR-007, FR-008, FR-016, FR-018, FR-019, FR-020
Per: specs/001-physical-ai-robotics-textbook/contracts/chatbot-api.yaml
"""

from typing import Optional, List, Dict, Any
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field, validator
import uuid

from ..models.chat import Citation, ContextSelection
from ..services.rag import RAGService
from ..services.chat_history import ChatHistoryService, ChatSession as DBChatSession

router = APIRouter(prefix="/api", tags=["chatbot"])


# Request/Response Models (per OpenAPI contract)

class ChatRequest(BaseModel):
    """
    Request schema for POST /chat
    Per: chatbot-api.yaml ChatRequest schema
    """
    session_id: Optional[str] = Field(
        None,
        description="Existing session UUID (null to create new session)"
    )
    message: str = Field(
        ...,
        description="User query text",
        min_length=1,
        max_length=500
    )
    context: Optional[ContextSelection] = Field(
        None,
        description="Retrieval scope (defaults to full_textbook)"
    )

    @validator('session_id')
    def validate_session_id(cls, v):
        """Ensure session_id is valid UUID format if provided"""
        if v is not None:
            try:
                uuid.UUID(v)
            except ValueError:
                raise ValueError("session_id must be a valid UUID")
        return v


class ChatResponse(BaseModel):
    """
    Response schema for POST /chat
    Per: chatbot-api.yaml ChatResponse schema
    """
    session_id: str = Field(..., description="Session UUID")
    message: str = Field(..., description="AI response with embedded citations")
    citations: List[Citation] = Field(..., description="Source citations")
    response_time_ms: int = Field(..., description="Generation time in milliseconds")


# Initialize services (singleton pattern)
rag_service = RAGService(top_k=5)
chat_history_service = ChatHistoryService()


@router.post("/chat", response_model=ChatResponse, status_code=status.HTTP_200_OK)
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Process user message and return AI response with citations

    Maps to:
    - FR-007: Real-time chatbot responses
    - FR-008: Semantic search with top-k retrieval
    - FR-016: Context-specific retrieval (module/chapter filtering)
    - FR-018: Chat history persistence
    - FR-019: Session management with 24h expiration
    - FR-020: Citation formatting

    Success Criteria:
    - SC-002: Response time <3 seconds (95th percentile)
    - SC-003: 100% out-of-scope rejection accuracy
    - SC-005: 90% of responses include citations

    Args:
        request: ChatRequest with user message and optional session/context

    Returns:
        ChatResponse with AI response, citations, and timing

    Raises:
        HTTPException 400: Invalid session_id or malformed request
        HTTPException 429: Rate limit exceeded (handled by middleware)
        HTTPException 500: Internal error during response generation
    """
    try:
        # Step 1: Handle session creation or retrieval
        if request.session_id:
            # Retrieve existing session
            session = chat_history_service.get_session(request.session_id)

            if not session:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Session {request.session_id} not found or expired"
                )

            # Update context if provided in request
            if request.context:
                # Convert Pydantic model to dataclass for service
                from ..services.chat_history import ContextSelection as DBContextSelection
                db_context = DBContextSelection(
                    mode=request.context.mode,
                    selected_module_ids=request.context.selected_module_ids,
                    selected_chapter_ids=request.context.selected_chapter_ids
                )
                chat_history_service.update_session_context(
                    session_id=session.session_id,
                    context_selection=db_context
                )
                # Update local session object
                session.context_selection = db_context
        else:
            # Create new session
            from ..services.chat_history import ContextSelection as DBContextSelection
            context = request.context or ContextSelection(mode="full_textbook")
            db_context = DBContextSelection(
                mode=context.mode,
                selected_module_ids=context.selected_module_ids,
                selected_chapter_ids=context.selected_chapter_ids
            )
            session = chat_history_service.create_session(
                context_selection=db_context,
                user_id=None  # Anonymous users for now
            )

        # Step 2: Retrieve conversation history for context (last 5-10 messages)
        conversation_history = chat_history_service.get_session_history(
            session_id=session.session_id,
            limit=5
        )

        # Convert to OpenAI format: List[Dict[str, str]]
        formatted_history = []
        for msg in conversation_history:
            formatted_history.append({
                "role": msg.role,
                "content": msg.content
            })

        # Step 3: Save user message to database
        chat_history_service.add_message(
            session_id=session.session_id,
            role="user",
            content=request.message,
            citations=None,
            response_time_ms=None
        )

        # Step 4: Generate RAG response
        # Convert DB context selection back to Pydantic model for RAG service
        pydantic_context = ContextSelection(
            mode=session.context_selection.mode,
            selected_module_ids=session.context_selection.selected_module_ids,
            selected_chapter_ids=session.context_selection.selected_chapter_ids
        )
        rag_result = rag_service.query(
            user_query=request.message,
            context_selection=pydantic_context,
            conversation_history=formatted_history
        )

        # Step 5: Save assistant message to database
        # Convert Pydantic Citation objects to dataclass Citations
        from ..services.chat_history import Citation as DBCitation
        db_citations = []
        for citation in rag_result["citations"]:
            db_citations.append(DBCitation(
                chunk_id=citation.chunk_id,
                chapter_title=citation.chapter_title,
                heading=citation.heading,
                url=citation.url
            ))

        chat_history_service.add_message(
            session_id=session.session_id,
            role="assistant",
            content=rag_result["response"],
            citations=db_citations,
            response_time_ms=rag_result["response_time_ms"]
        )

        # Step 6: Return response
        return ChatResponse(
            session_id=session.session_id,
            message=rag_result["response"],
            citations=rag_result["citations"],
            response_time_ms=rag_result["response_time_ms"]
        )

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise

    except Exception as e:
        # Log error and return generic 500 response
        print(f"Error processing chat request: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while processing your request. Please try again."
        )


@router.get("/context/modules", response_model=List[Dict[str, str]])
async def get_modules() -> List[Dict[str, str]]:
    """
    List available modules for context filtering

    Maps to: FR-016 (Context-specific retrieval)

    Returns:
        List of {module_id, title} dicts
    """
    # This would typically query the database for available modules
    # For now, return static list based on textbook structure
    return [
        {"module_id": "module-01-ros2", "title": "Module 1: ROS 2 Fundamentals"},
        {"module_id": "module-02-digital-twin", "title": "Module 2: Digital Twin & Simulation"},
        {"module_id": "module-03-isaac", "title": "Module 3: NVIDIA Isaac Platform"},
        {"module_id": "module-04-vla", "title": "Module 4: Vision-Language-Action Models"}
    ]


@router.get("/context/chapters", response_model=List[Dict[str, Any]])
async def get_chapters(module_id: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    List available chapters, optionally filtered by module

    Maps to: FR-016 (Context-specific retrieval)

    Args:
        module_id: Optional module filter (e.g., 'module-01-ros2')

    Returns:
        List of {chapter_id, title, module_id, week} dicts
    """
    # This would typically query the database for available chapters
    # For now, return static list based on textbook structure

    all_chapters = [
        # Module 1 chapters
        {"chapter_id": "module-01-ros2/week-03/ch01-nodes-topics", "title": "Chapter 1: Nodes & Topics", "module_id": "module-01-ros2", "week": 3},
        {"chapter_id": "module-01-ros2/week-03/ch02-services-actions", "title": "Chapter 2: Services & Actions", "module_id": "module-01-ros2", "week": 3},
        {"chapter_id": "module-01-ros2/week-04/ch03-parameters-launch", "title": "Chapter 3: Parameters & Launch", "module_id": "module-01-ros2", "week": 4},
        {"chapter_id": "module-01-ros2/week-04/ch04-tf2-transforms", "title": "Chapter 4: TF2 Transforms", "module_id": "module-01-ros2", "week": 4},
        {"chapter_id": "module-01-ros2/week-05/ch05-urdf-models", "title": "Chapter 5: URDF Models", "module_id": "module-01-ros2", "week": 5},
        {"chapter_id": "module-01-ros2/week-05/ch06-nav2-basics", "title": "Chapter 6: Nav2 Basics", "module_id": "module-01-ros2", "week": 5},

        # Module 2 chapters
        {"chapter_id": "module-02-digital-twin/week-06/ch07-gazebo-sim", "title": "Chapter 7: Gazebo Simulation", "module_id": "module-02-digital-twin", "week": 6},
        {"chapter_id": "module-02-digital-twin/week-06/ch08-sensor-modeling", "title": "Chapter 8: Sensor Modeling", "module_id": "module-02-digital-twin", "week": 6},
        {"chapter_id": "module-02-digital-twin/week-07/ch09-unity-viz", "title": "Chapter 9: Unity Visualization", "module_id": "module-02-digital-twin", "week": 7},
        {"chapter_id": "module-02-digital-twin/week-07/ch10-digital-twin", "title": "Chapter 10: Digital Twin", "module_id": "module-02-digital-twin", "week": 7},

        # Module 3 chapters
        {"chapter_id": "module-03-isaac/week-08/ch11-isaac-sim", "title": "Chapter 11: Isaac Sim", "module_id": "module-03-isaac", "week": 8},
        {"chapter_id": "module-03-isaac/week-08/ch12-isaac-ros", "title": "Chapter 12: Isaac ROS", "module_id": "module-03-isaac", "week": 8},
        {"chapter_id": "module-03-isaac/week-09/ch13-vslam-nav2", "title": "Chapter 13: Visual SLAM & Nav2", "module_id": "module-03-isaac", "week": 9},
        {"chapter_id": "module-03-isaac/week-09/ch14-perception", "title": "Chapter 14: Perception", "module_id": "module-03-isaac", "week": 9},
        {"chapter_id": "module-03-isaac/week-10/ch15-rl-sim2real", "title": "Chapter 15: RL & Sim2Real", "module_id": "module-03-isaac", "week": 10},
        {"chapter_id": "module-03-isaac/week-10/ch16-sensor-fusion", "title": "Chapter 16: Sensor Fusion", "module_id": "module-03-isaac", "week": 10},

        # Module 4 chapters
        {"chapter_id": "module-04-vla/week-11/ch17-vla-foundations", "title": "Chapter 17: VLA Foundations", "module_id": "module-04-vla", "week": 11},
        {"chapter_id": "module-04-vla/week-11/ch18-joint-control", "title": "Chapter 18: Joint Control", "module_id": "module-04-vla", "week": 11},
        {"chapter_id": "module-04-vla/week-12/ch19-object-detection", "title": "Chapter 19: Object Detection", "module_id": "module-04-vla", "week": 12},
        {"chapter_id": "module-04-vla/week-12/ch20-llm-integration", "title": "Chapter 20: LLM Integration", "module_id": "module-04-vla", "week": 12},
    ]

    # Filter by module_id if provided
    if module_id:
        filtered_chapters = [ch for ch in all_chapters if ch["module_id"] == module_id]
        return filtered_chapters

    return all_chapters
