"""
Qdrant service for semantic search operations.
Provides methods to query similar chunks with filtering.
"""

from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue, MatchAny

from src.utils.config import settings
from src.utils.logger import logger
from src.services.embedding_service import embedding_service


class QdrantService:
    """Service for querying Qdrant vector store."""

    def __init__(self):
        """Initialize Qdrant client."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = settings.qdrant_collection

    def query_similar_chunks(
        self,
        query_text: str,
        top_k: int = None,
        similarity_threshold: float = None,
        chapter_id: Optional[str] = None,
        hardware_profile: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar chunks using semantic search.

        Args:
            query_text: User's question
            top_k: Number of results to return (default from config)
            similarity_threshold: Minimum similarity score (default from config)
            chapter_id: Filter by specific chapter (optional)
            hardware_profile: Filter by hardware type: GPU, Edge, Cloud (optional)

        Returns:
            List of similar chunks with metadata and similarity scores
        """
        top_k = top_k or settings.rag_top_k
        similarity_threshold = similarity_threshold or settings.rag_similarity_threshold

        # Generate query embedding
        logger.info(f"Generating embedding for query: '{query_text[:50]}...'")
        query_embedding = embedding_service.generate_embedding(query_text)

        # Build filter
        query_filter = self._build_filter(chapter_id, hardware_profile)

        # Search Qdrant
        try:
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                score_threshold=similarity_threshold,
                query_filter=query_filter
            ).points

            logger.info(f"Found {len(search_results)} chunks with similarity > {similarity_threshold}")

            # Convert to dict format
            chunks = []
            for result in search_results:
                chunk = {
                    "chunk_id": result.payload.get("chunk_id"),
                    "chapter_id": result.payload.get("chapter_id"),
                    "chapter_title": result.payload.get("chapter_title"),
                    "section_title": result.payload.get("section_title", ""),
                    "content": result.payload.get("content"),
                    "content_type": result.payload.get("content_type"),
                    "hardware_relevance": result.payload.get("hardware_relevance", []),
                    "chunk_index": result.payload.get("chunk_index"),
                    "similarity_score": result.score,
                    "excerpt": result.payload.get("content", "")[:200]  # First 200 chars
                }
                chunks.append(chunk)

            return chunks

        except Exception as e:
            logger.error(f"Qdrant search failed: {str(e)}")
            raise

    def query_by_chapter(
        self,
        query_text: str,
        chapter_id: str,
        top_k: int = 3,
        similarity_threshold: float = 0.65
    ) -> List[Dict[str, Any]]:
        """
        Search within a specific chapter (for text selection queries).

        Args:
            query_text: User's question
            chapter_id: Chapter to search within
            top_k: Number of results (default 3 for text selection)
            similarity_threshold: Minimum similarity (lower threshold for focused queries)

        Returns:
            List of similar chunks from the specified chapter
        """
        logger.info(f"Searching in chapter '{chapter_id}' for: '{query_text[:50]}...'")

        return self.query_similar_chunks(
            query_text=query_text,
            top_k=top_k,
            similarity_threshold=similarity_threshold,
            chapter_id=chapter_id
        )

    def _build_filter(
        self,
        chapter_id: Optional[str] = None,
        hardware_profile: Optional[str] = None
    ) -> Optional[Filter]:
        """
        Build Qdrant filter for query.

        Args:
            chapter_id: Filter by chapter
            hardware_profile: Filter by hardware type

        Returns:
            Qdrant Filter object or None
        """
        conditions = []

        # Chapter filter (hard filter - must match)
        if chapter_id:
            conditions.append(
                FieldCondition(
                    key="chapter_id",
                    match=MatchValue(value=chapter_id)
                )
            )

        # Hardware filter (soft filter - boost relevant content)
        # Note: Qdrant doesn't support "should" directly in search,
        # so we use "must" with MatchAny to include both specific and "all"
        if hardware_profile:
            conditions.append(
                FieldCondition(
                    key="hardware_relevance",
                    match=MatchAny(any=[hardware_profile, "all"])
                )
            )

        if conditions:
            return Filter(must=conditions)

        return None

    def get_collection_stats(self) -> Dict[str, Any]:
        """
        Get collection statistics.

        Returns:
            Dictionary with collection stats
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)

            return {
                "collection_name": self.collection_name,
                "points_count": collection_info.points_count,
                "vector_size": collection_info.config.params.vectors.size,
                "distance_metric": str(collection_info.config.params.vectors.distance),
                "status": collection_info.status
            }

        except Exception as e:
            logger.error(f"Failed to get collection stats: {str(e)}")
            raise


# Global Qdrant service instance
qdrant_service = QdrantService()
