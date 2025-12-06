"""
RAG Service: Retrieval-Augmented Generation pipeline
"""

from typing import List, Dict, Optional
from .openai_service import OpenAIService
from ..db.qdrant_client import QdrantService


class RAGService:
    """
    RAG pipeline for Physical AI educational content

    Workflow:
    1. User query → Generate embedding
    2. Search Qdrant for similar content chunks
    3. Construct prompt with retrieved context
    4. Generate response with OpenAI
    """

    def __init__(self):
        self.openai_service = OpenAIService()
        self.qdrant_service = QdrantService()

        # System prompt for Physical AI domain
        self.system_prompt = """You are an expert Physical AI instructor specializing in robotics, ROS 2, simulation, and embodied intelligence.

Your role:
- Answer questions about Physical AI concepts, ROS 2, Gazebo, Isaac Sim, and Vision-Language-Action models
- Provide practical code examples when relevant
- Explain how AI reasoning integrates with physical robot execution
- Reference specific tutorials, hardware requirements, and best practices

Guidelines:
- Use retrieved context to provide accurate, specific answers
- If the context doesn't contain the answer, acknowledge limitations
- Prioritize hands-on, executable examples over theory
- Always connect digital AI concepts to physical robotics applications
"""

    def retrieve_context(
        self,
        query: str,
        language: str = "en",
        top_k: int = 5,
        score_threshold: float = 0.7,
        selected_text: Optional[str] = None
    ) -> List[Dict]:
        """
        Retrieve relevant content chunks from Qdrant

        Args:
            query: User's question
            language: Content language ('en' or 'ur')
            top_k: Number of results to retrieve
            score_threshold: Minimum similarity score (0.0 to 1.0)
            selected_text: Optional context-scoped text from docs

        Returns:
            List of retrieved chunks with metadata
        """
        # Use selected text as additional context if provided
        search_text = f"{selected_text}\n\n{query}" if selected_text else query

        # Generate embedding for query
        query_embedding = self.openai_service.generate_embedding(search_text)

        # Search appropriate collection based on language
        collection_name = f"content_embeddings_{language}"

        results = self.qdrant_service.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=score_threshold
        )

        return results

    async def retrieve_context_async(
        self,
        query: str,
        language: str = "en",
        top_k: int = 5,
        score_threshold: float = 0.7,
        selected_text: Optional[str] = None
    ) -> List[Dict]:
        """Async version of retrieve_context"""
        search_text = f"{selected_text}\n\n{query}" if selected_text else query
        query_embedding = await self.openai_service.generate_embedding_async(search_text)

        collection_name = f"content_embeddings_{language}"

        results = self.qdrant_service.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=score_threshold
        )

        return results

    def generate_response(
        self,
        query: str,
        retrieved_chunks: List[Dict],
        chat_history: Optional[List[Dict]] = None
    ) -> str:
        """
        Generate AI response using retrieved context

        Args:
            query: User's question
            retrieved_chunks: Context chunks from Qdrant
            chat_history: Previous messages for continuity

        Returns:
            AI-generated response string
        """
        # Construct context from retrieved chunks
        context_text = "\n\n---\n\n".join([
            f"[Score: {chunk['score']:.2f}]\n{chunk['payload'].get('content', '')}"
            for chunk in retrieved_chunks
        ])

        # Build messages for chat completion
        messages = [{"role": "system", "content": self.system_prompt}]

        # Add chat history if provided (last 5 messages)
        if chat_history:
            messages.extend(chat_history[-5:])

        # Add user query with retrieved context
        user_message = f"""Context from Physical AI documentation:

{context_text}

---

Question: {query}

Answer based on the provided context. If the context doesn't contain enough information, acknowledge this and provide general guidance."""

        messages.append({"role": "user", "content": user_message})

        # Generate completion
        response = self.openai_service.chat_completion(
            messages=messages,
            temperature=0.7,
            max_tokens=800
        )

        return response.choices[0].message.content

    async def generate_response_async(
        self,
        query: str,
        retrieved_chunks: List[Dict],
        chat_history: Optional[List[Dict]] = None,
        stream: bool = False
    ):
        """
        Async response generation with optional streaming

        Args:
            query: User's question
            retrieved_chunks: Context chunks
            chat_history: Previous messages
            stream: Enable streaming responses

        Returns:
            Response string or AsyncStream
        """
        context_text = "\n\n---\n\n".join([
            f"[Score: {chunk['score']:.2f}]\n{chunk['payload'].get('content', '')}"
            for chunk in retrieved_chunks
        ])

        messages = [{"role": "system", "content": self.system_prompt}]

        if chat_history:
            messages.extend(chat_history[-5:])

        user_message = f"""Context from Physical AI documentation:

{context_text}

---

Question: {query}

Answer based on the provided context. If the context doesn't contain enough information, acknowledge this and provide general guidance."""

        messages.append({"role": "user", "content": user_message})

        response = await self.openai_service.chat_completion_async(
            messages=messages,
            temperature=0.7,
            max_tokens=800,
            stream=stream
        )

        if stream:
            return response
        else:
            return response.choices[0].message.content

    async def process_query(
        self,
        query: str,
        language: str = "en",
        selected_text: Optional[str] = None,
        chat_history: Optional[List[Dict]] = None,
        stream: bool = False
    ):
        """
        End-to-end RAG pipeline

        Args:
            query: User's question
            language: Content language
            selected_text: Optional scoped context
            chat_history: Previous conversation
            stream: Enable streaming

        Returns:
            Dict with response and metadata
        """
        # Retrieve relevant context
        retrieved_chunks = await self.retrieve_context_async(
            query=query,
            language=language,
            selected_text=selected_text
        )

        # Generate response
        response = await self.generate_response_async(
            query=query,
            retrieved_chunks=retrieved_chunks,
            chat_history=chat_history,
            stream=stream
        )

        if stream:
            # Return stream directly for streaming responses
            return {
                "stream": response,
                "retrieved_chunks": retrieved_chunks
            }
        else:
            return {
                "response": response,
                "retrieved_chunks": retrieved_chunks,
                "num_chunks": len(retrieved_chunks)
            }
