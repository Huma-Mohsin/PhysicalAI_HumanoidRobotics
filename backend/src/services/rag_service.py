"""
RAG (Retrieval-Augmented Generation) service.
Orchestrates the full RAG pipeline: embed → search → generate.
"""

from typing import List, Dict, Any, Optional
import cohere
import time

from utils.config import settings
from utils.logger import logger
from services.embedding_service import embedding_service
from services.qdrant_service import qdrant_service
from models.message import MessageMetadata


class RAGService:
    """Service for RAG pipeline operations."""

    def __init__(self):
        """Initialize chat client based on LLM provider."""
        self.provider = settings.llm_provider

        if self.provider == "cohere":
            self.client = cohere.ClientV2(api_key=settings.cohere_api_key)
            self.chat_model = settings.cohere_chat_model
        else:  # openai
            # Import OpenAI only when needed (conditional import to reduce deployment size)
            from openai import OpenAI
            self.client = OpenAI(api_key=settings.openai_api_key)
            self.chat_model = settings.openai_chat_model

        logger.info(f"Initialized {self.provider} RAG service with model: {self.chat_model}")

    def query_book(
        self,
        question: str,
        conversation_history: Optional[List[Dict[str, str]]] = None,
        hardware_profile: Optional[str] = None,
        chapter_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Execute full RAG pipeline for a question.

        Args:
            question: User's question
            conversation_history: Previous messages for context (list of {role, content})
            hardware_profile: User's hardware type (GPU/Edge/Cloud)
            chapter_id: Filter by specific chapter (for text selection queries)

        Returns:
            Dictionary with response, retrieved chunks, and metadata
        """
        start_time = time.time()

        logger.info(f"RAG query: '{question[:50]}...'")

        # Step 1: Vector Search
        qdrant_start = time.time()

        if chapter_id:
            # Text selection query: search within chapter
            retrieved_chunks = qdrant_service.query_by_chapter(
                query_text=question,
                chapter_id=chapter_id,
                top_k=3,
                similarity_threshold=0.65
            )
        else:
            # General Q&A query: search across all chapters
            retrieved_chunks = qdrant_service.query_similar_chunks(
                query_text=question,
                top_k=settings.rag_top_k,
                similarity_threshold=settings.rag_similarity_threshold,
                hardware_profile=hardware_profile
            )

        qdrant_time = int((time.time() - qdrant_start) * 1000)

        logger.info(f"Retrieved {len(retrieved_chunks)} chunks in {qdrant_time}ms")

        # Step 2: Assemble Context
        context = self._assemble_context(
            retrieved_chunks=retrieved_chunks,
            conversation_history=conversation_history or [],
            hardware_profile=hardware_profile
        )

        # Step 3: Generate Response
        llm_start = time.time()

        response, tokens_used = self._generate_response(
            question=question,
            context=context,
            conversation_history=conversation_history or []
        )

        llm_time = int((time.time() - llm_start) * 1000)

        total_time = int((time.time() - start_time) * 1000)

        logger.info(f"Generated response in {llm_time}ms (total: {total_time}ms, {tokens_used} tokens)")

        # Step 4: Build metadata
        embedding_model = settings.cohere_embedding_model if self.provider == "cohere" else settings.openai_embedding_model

        metadata = MessageMetadata(
            tokens_used=tokens_used,
            latency_ms=total_time,
            model=self.chat_model,
            retrieved_chunks=len(retrieved_chunks),
            qdrant_query_ms=qdrant_time,
            embedding_model=embedding_model
        )

        return {
            "response": response,
            "retrieved_chunks": retrieved_chunks[:3],  # Return top 3 for client
            "metadata": metadata
        }

    def _assemble_context(
        self,
        retrieved_chunks: List[Dict[str, Any]],
        conversation_history: List[Dict[str, str]],
        hardware_profile: Optional[str]
    ) -> str:
        """
        Assemble context from retrieved chunks and conversation history.

        Args:
            retrieved_chunks: Chunks from Qdrant
            conversation_history: Previous messages
            hardware_profile: User's hardware type

        Returns:
            Formatted context string
        """
        context_parts = []

        # Add retrieved book content
        if retrieved_chunks:
            context_parts.append("# Relevant Book Content\n")
            for i, chunk in enumerate(retrieved_chunks, 1):
                context_parts.append(
                    f"## Source {i}: {chunk['chapter_title']} (Similarity: {chunk['similarity_score']:.2f})\n"
                    f"{chunk['content']}\n"
                )

        # Add conversation history (last 5 messages for context)
        if conversation_history:
            recent_history = conversation_history[-5:]
            context_parts.append("\n# Conversation History\n")
            for msg in recent_history:
                role = msg.get("role", "unknown").capitalize()
                content = msg.get("content", "")
                context_parts.append(f"**{role}**: {content}\n")

        # Add hardware profile context
        if hardware_profile:
            context_parts.append(f"\n# User Hardware Profile: {hardware_profile}\n")

        return "\n".join(context_parts)

    def _generate_response(
        self,
        question: str,
        context: str,
        conversation_history: List[Dict[str, str]]
    ) -> tuple[str, int]:
        """
        Generate response using Cohere or OpenAI.

        Args:
            question: User's question
            context: Assembled context
            conversation_history: Previous messages

        Returns:
            Tuple of (response_text, tokens_used)
        """
        # Build system prompt
        system_prompt = """You are an AI assistant for the "Physical AI & Humanoid Robotics" book. Your role is to:

1. Answer questions based on the book content provided in the context
2. Provide accurate, detailed explanations about ROS 2, Gazebo, NVIDIA Isaac, and humanoid robotics
3. Adapt responses based on the user's hardware setup (GPU Workstation, Edge Device, or Cloud/Mac)
4. Use markdown formatting for code blocks, lists, and emphasis
5. If the book doesn't cover a topic, politely say so and suggest related content

When answering:
- Quote specific sections from the book when relevant
- Provide practical examples and code snippets if applicable
- Tailor hardware recommendations to the user's profile
- Be concise but thorough
"""

        try:
            if self.provider == "cohere":
                # Build messages for Cohere V2 (system message goes in messages array)
                messages = []

                # Add system prompt as first message
                messages.append({
                    "role": "system",
                    "content": system_prompt
                })

                # Add conversation history (last 5 messages)
                for msg in conversation_history[-5:]:
                    messages.append({
                        "role": msg["role"],
                        "content": msg["content"]
                    })

                # Add current question
                user_message = f"{question}\n\nContext:\n{context}" if context else question
                messages.append({
                    "role": "user",
                    "content": user_message
                })

                # Call Cohere V2 API (no preamble parameter)
                response = self.client.chat(
                    model=self.chat_model,
                    messages=messages,
                    temperature=0.7,
                    max_tokens=1000
                )

                response_text = response.message.content[0].text
                tokens_used = response.usage.tokens.input_tokens + response.usage.tokens.output_tokens

                return response_text, tokens_used

            else:  # openai
                # Build messages for OpenAI
                messages = [
                    {"role": "system", "content": system_prompt}
                ]

                # Add context as a system message
                if context:
                    messages.append({
                        "role": "system",
                        "content": f"Context for answering the user's question:\n\n{context}"
                    })

                # Add conversation history (last 5 messages)
                for msg in conversation_history[-5:]:
                    messages.append({
                        "role": msg["role"],
                        "content": msg["content"]
                    })

                # Add current question
                messages.append({
                    "role": "user",
                    "content": question
                })

                # Call OpenAI API
                response = self.client.chat.completions.create(
                    model=self.chat_model,
                    messages=messages,
                    temperature=0.7,
                    max_tokens=1000
                )

                response_text = response.choices[0].message.content
                tokens_used = response.usage.total_tokens

                return response_text, tokens_used

        except Exception as e:
            logger.error(f"{self.provider.upper()} API error: {str(e)}")
            raise


# Global RAG service instance
rag_service = RAGService()
