"""
OpenAI Service for embeddings and chat completions
"""

import os
from typing import List, Optional
from openai import OpenAI, AsyncOpenAI
from openai.types.chat import ChatCompletion


class OpenAIService:
    """Service for OpenAI API interactions"""

    def __init__(self):
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")

        self.client = OpenAI(api_key=api_key)
        self.async_client = AsyncOpenAI(api_key=api_key)
        self.embedding_model = "text-embedding-3-small"  # 1536 dimensions
        self.chat_model = "gpt-4o-mini"  # Cost-effective for RAG

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding vector for text

        Args:
            text: Input text to embed

        Returns:
            List of 1536 floats (embedding vector)
        """
        response = self.client.embeddings.create(
            model=self.embedding_model,
            input=text
        )
        return response.data[0].embedding

    async def generate_embedding_async(self, text: str) -> List[float]:
        """Async version of generate_embedding"""
        response = await self.async_client.embeddings.create(
            model=self.embedding_model,
            input=text
        )
        return response.data[0].embedding

    def chat_completion(
        self,
        messages: List[dict],
        temperature: float = 0.7,
        max_tokens: Optional[int] = None
    ) -> ChatCompletion:
        """
        Generate chat completion

        Args:
            messages: List of message dicts with 'role' and 'content'
            temperature: Sampling temperature (0.0 to 2.0)
            max_tokens: Maximum tokens to generate

        Returns:
            ChatCompletion object
        """
        return self.client.chat.completions.create(
            model=self.chat_model,
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens
        )

    async def chat_completion_async(
        self,
        messages: List[dict],
        temperature: float = 0.7,
        max_tokens: Optional[int] = None,
        stream: bool = False
    ):
        """
        Async chat completion with optional streaming

        Args:
            messages: List of message dicts
            temperature: Sampling temperature
            max_tokens: Maximum tokens to generate
            stream: Enable streaming responses

        Returns:
            ChatCompletion or AsyncStream
        """
        return await self.async_client.chat.completions.create(
            model=self.chat_model,
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens,
            stream=stream
        )
