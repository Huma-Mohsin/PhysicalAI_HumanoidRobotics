"""
Qdrant Cloud Vector Database Client Module

This module provides connection and operations for the Qdrant vector database
used for RAG semantic search.
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from typing import List, Dict, Any, Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Qdrant configuration from environment
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Collection names for English and Urdu content
COLLECTION_EN = "content_embeddings_en"
COLLECTION_UR = "content_embeddings_ur"

# Vector configuration (OpenAI text-embedding-3-small)
VECTOR_SIZE = 1536
DISTANCE_METRIC = Distance.COSINE


class QdrantService:
    """
    Service class for Qdrant vector database operations.
    """

    def __init__(self):
        """Initialize Qdrant client with credentials from environment."""
        if not QDRANT_URL or not QDRANT_API_KEY:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment")

        self.client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            timeout=30,
        )

    def check_connection(self) -> bool:
        """
        Check if Qdrant connection is healthy.

        Returns:
            bool: True if connection is successful, False otherwise
        """
        try:
            collections = self.client.get_collections()
            return True
        except Exception as e:
            print(f"Qdrant connection failed: {e}")
            return False

    def create_collection(self, collection_name: str) -> bool:
        """
        Create a collection for content embeddings if it doesn't exist.

        Args:
            collection_name: Name of the collection (COLLECTION_EN or COLLECTION_UR)

        Returns:
            bool: True if collection created or already exists
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            exists = any(col.name == collection_name for col in collections)

            if exists:
                print(f"Collection '{collection_name}' already exists")
                return True

            # Create collection
            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=VECTOR_SIZE,
                    distance=DISTANCE_METRIC,
                    on_disk=True,  # Free tier optimization: store vectors on disk
                ),
            )
            print(f"Collection '{collection_name}' created successfully")
            return True

        except Exception as e:
            print(f"Failed to create collection '{collection_name}': {e}")
            return False

    def upsert_embeddings(
        self,
        collection_name: str,
        points: List[PointStruct]
    ) -> bool:
        """
        Insert or update embeddings in the collection.

        Args:
            collection_name: Target collection name
            points: List of PointStruct objects with id, vector, and payload

        Returns:
            bool: True if upsert successful
        """
        try:
            self.client.upsert(
                collection_name=collection_name,
                points=points,
            )
            print(f"Upserted {len(points)} points to '{collection_name}'")
            return True
        except Exception as e:
            print(f"Failed to upsert embeddings: {e}")
            return False

    def search(
        self,
        collection_name: str,
        query_vector: List[float],
        limit: int = 5,
        score_threshold: float = 0.7,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection.

        Args:
            collection_name: Collection to search
            query_vector: Query embedding vector
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score (0-1)

        Returns:
            List of search results with payload and score
        """
        try:
            results = self.client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=limit,
                score_threshold=score_threshold,
            )

            return [
                {
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                }
                for result in results
            ]

        except Exception as e:
            print(f"Search failed: {e}")
            return []

    def get_collection_info(self, collection_name: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a collection.

        Args:
            collection_name: Collection name

        Returns:
            Dictionary with collection info or None if not found
        """
        try:
            info = self.client.get_collection(collection_name=collection_name)
            return {
                "name": collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status,
            }
        except Exception as e:
            print(f"Failed to get collection info: {e}")
            return None


# Global instance (singleton pattern)
_qdrant_service: Optional[QdrantService] = None


def get_qdrant_service() -> QdrantService:
    """
    Get or create the global Qdrant service instance.

    Returns:
        QdrantService: Global Qdrant service instance
    """
    global _qdrant_service
    if _qdrant_service is None:
        _qdrant_service = QdrantService()
    return _qdrant_service
