"""
RAG (Retrieval-Augmented Generation) service

Combines:
- Qdrant vector search for relevant content chunks
- OpenAI Agents SDK for response generation
- Citation formatting and out-of-scope detection

Maps to: FR-008, FR-016, FR-020, SC-002, SC-003, SC-005
"""

from typing import List, Dict, Any, Optional
import time
import os
import google.generativeai as genai

from ..models.chat import Citation, ContextSelection
from ..models.content import ContentChunk
from .embeddings import EmbeddingService
from .vector_db import VectorDBClient


class RAGService:
    """
    RAG service for textbook-grounded chatbot responses

    Workflow:
    1. Embed user query
    2. Search Qdrant for top-k similar chunks (filtered by context)
    3. Build context from retrieved chunks
    4. Generate response using OpenAI with system prompt
    5. Extract citations from retrieved chunks
    6. Detect and reject out-of-scope queries
    """

    def __init__(
        self,
        embedding_service: Optional[EmbeddingService] = None,
        vector_db_client: Optional[VectorDBClient] = None,
        top_k: int = 5
    ):
        """
        Initialize RAG service

        Args:
            embedding_service: Service for generating query embeddings
            vector_db_client: Client for vector database operations
            top_k: Number of chunks to retrieve (default: 5, range: 5-10 per research.md)
        """
        self.embedding_service = embedding_service or EmbeddingService()
        self.vector_db = vector_db_client or VectorDBClient()
        self.top_k = top_k

        # Initialize Gemini client
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable not set")
        genai.configure(api_key=api_key)

        # List available models to debug
        print("🔍 Available Gemini models:")
        try:
            for model in genai.list_models():
                if 'generateContent' in model.supported_generation_methods:
                    print(f"   ✓ {model.name}")
        except Exception as e:
            print(f"   ❌ Error listing models: {e}")

        # Use gemini-1.0-pro (stable model)
        self.model = genai.GenerativeModel('gemini-1.0-pro')

        # System prompt for textbook-grounded responses (using gemini-pro)
        self.system_prompt = """You are a helpful teaching assistant for a Physical AI & Humanoid Robotics course.

Your role:
- Answer questions ONLY using the provided textbook content
- Provide clear, educational explanations suitable for university students
- Include specific examples and technical details from the textbook
- Cite the source sections you're referencing

Rules:
- If the question is about Physical AI, Humanoid Robotics, ROS 2, simulation, NVIDIA Isaac, or VLA models, answer using the provided context
- If the question is completely unrelated to these topics, politely decline and redirect to course material
- Always maintain an encouraging, educational tone
- Use technical terminology correctly and explain when needed

Format your response with:
1. A clear, direct answer
2. Supporting details and examples from the textbook
3. Practical applications when relevant
"""

    def query(
        self,
        user_query: str,
        context_selection: Optional[ContextSelection] = None,
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Process user query and generate RAG response

        Args:
            user_query: User's question
            context_selection: Filter for module/chapter retrieval
            conversation_history: Previous messages for context (last 5-10)

        Returns:
            Dict with:
                - response: Generated answer text
                - citations: List of source citations
                - response_time_ms: Generation time
                - is_out_of_scope: Whether query was rejected
        """
        start_time = time.time()

        # Step 1: Check if query is out of scope (basic keyword filter)
        if self._is_out_of_scope(user_query):
            return {
                "response": self._get_out_of_scope_message(),
                "citations": [],
                "response_time_ms": int((time.time() - start_time) * 1000),
                "is_out_of_scope": True
            }

        # Step 2: Embed user query
        query_embedding = self.embedding_service.embed_query(user_query)

        # Step 3: Search Qdrant for relevant chunks
        filter_conditions = self._build_filter_conditions(context_selection)
        search_results = self.vector_db.search(
            query_embedding=query_embedding,
            top_k=self.top_k,
            filter_conditions=filter_conditions
        )

        if not search_results:
            return {
                "response": "I couldn't find relevant information in the textbook to answer your question. Could you rephrase or ask about a different topic covered in the course?",
                "citations": [],
                "response_time_ms": int((time.time() - start_time) * 1000),
                "is_out_of_scope": False
            }

        # Step 4: Build context from retrieved chunks
        context_text = self._build_context(search_results)

        # Step 5: Generate response using Gemini
        prompt = self._build_prompt(user_query, context_text, conversation_history)

        try:
            response = self.model.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.7,
                    max_output_tokens=1000,
                )
            )
            response_text = response.text
        except Exception as e:
            print(f"Error generating response: {e}")
            response_text = "I'm having trouble generating a response right now. Please try again."

        # Step 6: Format citations
        citations = self._format_citations(search_results)

        response_time_ms = int((time.time() - start_time) * 1000)

        return {
            "response": response_text,
            "citations": citations,
            "response_time_ms": response_time_ms,
            "is_out_of_scope": False
        }

    def _is_out_of_scope(self, query: str) -> bool:
        """
        Detect queries unrelated to Physical AI/Humanoid Robotics

        Uses keyword-based filtering + length checks
        Per SC-003: 100% accuracy for out-of-scope rejection

        Args:
            query: User question

        Returns:
            True if query should be rejected
        """
        query_lower = query.lower()

        # Course-related keywords
        in_scope_keywords = [
            'robot', 'ros', 'ros2', 'physical ai', 'humanoid', 'sensor', 'actuator',
            'gazebo', 'isaac', 'nvidia', 'simulation', 'urdf', 'navigation', 'nav2',
            'vla', 'vision', 'language', 'action', 'perception', 'manipulation',
            'grasping', 'walking', 'joint', 'control', 'digital twin', 'unity',
            'lidar', 'camera', 'imu', 'whisper', 'llm', 'embodied', 'tf2', 'transform'
        ]

        # Check if query contains any in-scope keywords
        has_relevant_keywords = any(keyword in query_lower for keyword in in_scope_keywords)

        # Very short queries or overly generic questions
        if len(query.strip()) < 10:
            return True

        # If no relevant keywords found, likely out of scope
        if not has_relevant_keywords:
            # Double-check with common off-topic patterns
            off_topic_patterns = [
                'weather', 'recipe', 'movie', 'music', 'sports', 'politics',
                'celebrity', 'game', 'stock', 'crypto', 'bitcoin', 'dating'
            ]
            if any(pattern in query_lower for pattern in off_topic_patterns):
                return True

        return False

    def _get_out_of_scope_message(self) -> str:
        """Return friendly rejection message for out-of-scope queries"""
        return """I'm a teaching assistant specifically for the Physical AI & Humanoid Robotics course.

I can help you with topics like:
- ROS 2 programming and robotics fundamentals
- Simulation with Gazebo, Unity, and NVIDIA Isaac
- Sensor integration and perception
- Humanoid robot control and locomotion
- Vision-Language-Action (VLA) models

Please ask a question related to these course topics!"""

    def _build_filter_conditions(self, context_selection: Optional[ContextSelection]) -> Optional[Dict]:
        """
        Build Qdrant filter conditions from context selection

        Args:
            context_selection: User's module/chapter filter

        Returns:
            Qdrant filter dict or None for full textbook
        """
        if not context_selection or context_selection.mode == "full_textbook":
            return None

        conditions = []

        if context_selection.mode == "module" and context_selection.selected_module_ids:
            # Filter by module IDs
            conditions.append({
                "key": "metadata.module",
                "match": {"any": context_selection.selected_module_ids}
            })

        elif context_selection.mode == "chapter" and context_selection.selected_chapter_ids:
            # Filter by specific chapter IDs
            conditions.append({
                "key": "chapter_id",
                "match": {"any": context_selection.selected_chapter_ids}
            })

        return {"must": conditions} if conditions else None

    def _build_context(self, search_results: List[Any]) -> str:
        """
        Concatenate retrieved chunks into context string

        Args:
            search_results: Qdrant search results

        Returns:
            Formatted context text for LLM
        """
        context_parts = []

        for i, result in enumerate(search_results, 1):
            chunk_text = result.content_text
            metadata = result.metadata

            # Add source citation inline
            source = f"[Source {i}: {metadata.get('chapter_title', 'Unknown')} - {metadata.get('heading', 'Unknown')}]"
            context_parts.append(f"{source}\n{chunk_text}\n")

        return "\n---\n".join(context_parts)

    def _build_prompt(
        self,
        user_query: str,
        context_text: str,
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> str:
        """
        Build prompt for Gemini API

        Args:
            user_query: Current user question
            context_text: Retrieved textbook chunks
            conversation_history: Previous messages (last 5-10)

        Returns:
            Complete prompt string
        """
        prompt_parts = [self.system_prompt, "\n\n"]

        # Add conversation history for context (last 5 messages max)
        if conversation_history:
            prompt_parts.append("Previous conversation:\n")
            for msg in conversation_history[-5:]:
                role = "User" if msg["role"] == "user" else "Assistant"
                prompt_parts.append(f"{role}: {msg['content']}\n")
            prompt_parts.append("\n")

        # Add current query with retrieved context
        prompt_parts.append(f"""Based on the following textbook content, please answer this question:

{user_query}

Textbook content:
{context_text}

Provide a clear answer with examples from the textbook above.""")

        return "".join(prompt_parts)

    def _format_citations(self, search_results: List[Any]) -> List[Citation]:
        """
        Format retrieved chunks into Citation objects

        Per FR-020: Citations formatted as "📚 **Sources:** [Chapter: Heading](URL)"
        Per SC-005: 90% of responses should include citations

        Args:
            search_results: Qdrant search results

        Returns:
            List of Citation objects
        """
        citations = []

        for result in search_results:
            metadata = result.metadata

            # Extract preview (first 200 chars)
            content_preview = result.content_text[:200]
            if len(result.content_text) > 200:
                content_preview += "..."

            citation = Citation(
                chunk_id=result.chunk_id,
                chapter_title=metadata.get("chapter_title", "Unknown Chapter"),
                heading=metadata.get("heading", "Unknown Section"),
                url=f"{metadata.get('page_url', '')}{metadata.get('heading_anchor', '')}",
                chunk_preview=content_preview
            )
            citations.append(citation)

        return citations
