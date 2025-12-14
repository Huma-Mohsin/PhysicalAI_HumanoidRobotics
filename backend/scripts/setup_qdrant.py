"""
Qdrant Cloud setup script.
Creates the vector collection for the RAG chatbot.
"""

import sys
from pathlib import Path

# Add parent directory to path to import utils
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from utils.config import settings
from utils.logger import logger
from services.embedding_service import embedding_service


def setup_qdrant_collection():
    """
    Create Qdrant collection for humanoid robotics book embeddings.

    Collection specs:
    - Vector size: 1024 (Cohere embed-english-v3.0) or 1536 (OpenAI text-embedding-3-small)
    - Distance metric: Cosine similarity
    - On-disk storage: Required for free tier
    """
    logger.info(f"Using {settings.llm_provider} embeddings with {embedding_service.embedding_dim} dimensions")
    logger.info("Connecting to Qdrant Cloud...")

    # Initialize Qdrant client
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )

    collection_name = settings.qdrant_collection

    # Check if collection already exists
    try:
        collections = client.get_collections().collections
        existing_collections = [col.name for col in collections]

        if collection_name in existing_collections:
            logger.warning(f"Collection '{collection_name}' already exists")
            response = input("Do you want to recreate it? (yes/no): ")
            if response.lower() != "yes":
                logger.info("Skipping collection creation")
                return

            # Delete existing collection
            client.delete_collection(collection_name)
            logger.info(f"Deleted existing collection '{collection_name}'")

    except Exception as e:
        logger.error(f"Error checking existing collections: {str(e)}")
        raise

    # Create new collection
    logger.info(f"Creating collection '{collection_name}'...")

    try:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=embedding_service.embedding_dim,  # Dynamic: 1024 (Cohere) or 1536 (OpenAI)
                distance=Distance.COSINE,  # Cosine similarity
                on_disk=True  # Required for free tier (1GB limit)
            )
        )
        logger.info(f"✅ Collection '{collection_name}' created successfully")

    except Exception as e:
        logger.error(f"❌ Failed to create collection: {str(e)}")
        raise

    # Create payload indexes for faster filtering
    logger.info("Creating payload indexes...")

    try:
        # Index chapter_id for filtering by chapter
        client.create_payload_index(
            collection_name=collection_name,
            field_name="chapter_id",
            field_schema="keyword"
        )
        logger.info("✅ Created index on 'chapter_id'")

        # Index hardware_relevance for filtering by user profile
        client.create_payload_index(
            collection_name=collection_name,
            field_name="hardware_relevance",
            field_schema="keyword"
        )
        logger.info("✅ Created index on 'hardware_relevance'")

        # Index content_type for filtering by text/code/mixed
        client.create_payload_index(
            collection_name=collection_name,
            field_name="content_type",
            field_schema="keyword"
        )
        logger.info("✅ Created index on 'content_type'")

    except Exception as e:
        logger.error(f"⚠️ Failed to create indexes: {str(e)}")
        # Don't raise - indexes are optional optimizations

    # Verify collection
    collection_info = client.get_collection(collection_name)
    logger.info(f"Collection info: {collection_info}")
    logger.info("🎉 Qdrant setup complete!")


if __name__ == "__main__":
    setup_qdrant_collection()
