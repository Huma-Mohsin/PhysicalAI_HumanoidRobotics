"""
Embedding service using OpenAI or Cohere.
Handles text chunking and embedding generation.
"""

import cohere
from typing import List, Dict, Any
import tiktoken

from utils.config import settings
from utils.logger import logger


class EmbeddingService:
    """Service for generating text embeddings."""

    def __init__(self):
        """Initialize embedding client based on LLM provider."""
        self.provider = settings.llm_provider

        if self.provider == "cohere":
            self.client = cohere.ClientV2(api_key=settings.cohere_api_key)
            self.model = settings.cohere_embedding_model
            self.embedding_dim = 1024  # Cohere embed-english-v3.0
        else:  # openai
            # Import OpenAI only when needed (conditional import to reduce deployment size)
            from openai import OpenAI
            self.client = OpenAI(api_key=settings.openai_api_key)
            self.model = settings.openai_embedding_model
            self.embedding_dim = 1536  # OpenAI text-embedding-3-small

        self.encoding = tiktoken.encoding_for_model("gpt-4")  # Use GPT-4 encoding for token counting
        logger.info(f"Initialized {self.provider} embedding service with model: {self.model}")

    def count_tokens(self, text: str) -> int:
        """Count tokens in text."""
        return len(self.encoding.encode(text))

    def chunk_text(
        self,
        text: str,
        chunk_size: int = None,
        chunk_overlap: int = None
    ) -> List[Dict[str, Any]]:
        """
        Split text into chunks with overlap.

        Args:
            text: Text to chunk
            chunk_size: Maximum tokens per chunk (default from config)
            chunk_overlap: Token overlap between chunks (default from config)

        Returns:
            List of chunks with metadata
        """
        chunk_size = chunk_size or settings.rag_chunk_size
        chunk_overlap = chunk_overlap or settings.rag_chunk_overlap

        # Split by paragraphs first (better semantic boundaries)
        paragraphs = text.split('\n\n')
        chunks = []
        current_chunk = ""
        current_tokens = 0
        chunk_index = 0

        for paragraph in paragraphs:
            paragraph = paragraph.strip()
            if not paragraph:
                continue

            para_tokens = self.count_tokens(paragraph)

            # If paragraph alone exceeds chunk size, split it
            if para_tokens > chunk_size:
                # Split by sentences
                sentences = paragraph.split('. ')
                for sentence in sentences:
                    sentence = sentence.strip() + '. '
                    sentence_tokens = self.count_tokens(sentence)

                    if current_tokens + sentence_tokens > chunk_size:
                        # Save current chunk
                        if current_chunk:
                            chunks.append({
                                "text": current_chunk.strip(),
                                "chunk_index": chunk_index,
                                "token_count": current_tokens
                            })
                            chunk_index += 1

                        # Start new chunk with overlap
                        if chunks and chunk_overlap > 0:
                            # Get last N tokens from previous chunk
                            prev_text = chunks[-1]["text"]
                            prev_tokens = self.encoding.encode(prev_text)
                            overlap_tokens = prev_tokens[-chunk_overlap:]
                            current_chunk = self.encoding.decode(overlap_tokens) + " "
                            current_tokens = len(overlap_tokens)
                        else:
                            current_chunk = ""
                            current_tokens = 0

                    current_chunk += sentence + " "
                    current_tokens += sentence_tokens

            # Paragraph fits in chunk
            elif current_tokens + para_tokens <= chunk_size:
                current_chunk += paragraph + "\n\n"
                current_tokens += para_tokens

            # Paragraph would exceed chunk, save current and start new
            else:
                if current_chunk:
                    chunks.append({
                        "text": current_chunk.strip(),
                        "chunk_index": chunk_index,
                        "token_count": current_tokens
                    })
                    chunk_index += 1

                # Start new chunk with overlap
                if chunks and chunk_overlap > 0:
                    prev_text = chunks[-1]["text"]
                    prev_tokens = self.encoding.encode(prev_text)
                    overlap_tokens = prev_tokens[-chunk_overlap:]
                    current_chunk = self.encoding.decode(overlap_tokens) + " " + paragraph + "\n\n"
                    current_tokens = len(overlap_tokens) + para_tokens
                else:
                    current_chunk = paragraph + "\n\n"
                    current_tokens = para_tokens

        # Save final chunk
        if current_chunk:
            chunks.append({
                "text": current_chunk.strip(),
                "chunk_index": chunk_index,
                "token_count": current_tokens
            })

        logger.info(f"Created {len(chunks)} chunks from text ({len(text)} chars)")
        return chunks

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed

        Returns:
            Embedding vector (1024-dim for Cohere, 1536-dim for OpenAI)
        """
        try:
            if self.provider == "cohere":
                response = self.client.embed(
                    texts=[text],
                    model=self.model,
                    input_type="search_document",
                    embedding_types=["float"]
                )
                return response.embeddings.float_[0]
            else:  # openai
                response = self.client.embeddings.create(
                    input=text,
                    model=self.model
                )
                return response.data[0].embedding

        except Exception as e:
            logger.error(f"Failed to generate embedding: {str(e)}")
            raise

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts (batch processing).

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        try:
            if self.provider == "cohere":
                response = self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type="search_document",
                    embedding_types=["float"]
                )
                return response.embeddings.float_
            else:  # openai
                response = self.client.embeddings.create(
                    input=texts,
                    model=self.model
                )
                return [item.embedding for item in response.data]

        except Exception as e:
            logger.error(f"Failed to generate batch embeddings: {str(e)}")
            raise

    def embed_chunks(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for a list of chunks.

        Args:
            chunks: List of chunk dictionaries from chunk_text()

        Returns:
            Chunks with embedding vectors added
        """
        texts = [chunk["text"] for chunk in chunks]
        embeddings = self.generate_embeddings_batch(texts)

        for i, chunk in enumerate(chunks):
            chunk["embedding"] = embeddings[i]

        logger.info(f"Generated {len(embeddings)} embeddings")
        return chunks


# Global embedding service instance
embedding_service = EmbeddingService()
