"""
Validation script to verify Qdrant collection has correct embeddings.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from qdrant_client import QdrantClient
from utils.config import settings
from utils.logger import logger


def validate_embeddings(collection_name: str = None):
    """
    Validate Qdrant collection.

    Args:
        collection_name: Qdrant collection name (default from config)
    """
    collection_name = collection_name or settings.qdrant_collection

    logger.info(f"Validating Qdrant collection: {collection_name}")

    # Connect to Qdrant
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )

    # Get collection info
    try:
        collection_info = client.get_collection(collection_name)
    except Exception as e:
        logger.error(f"❌ Collection '{collection_name}' not found: {str(e)}")
        return False

    logger.info(f"Collection: {collection_name}")
    logger.info(f"Points count: {collection_info.points_count}")
    logger.info(f"Vector size: {collection_info.config.params.vectors.size}")
    logger.info(f"Distance: {collection_info.config.params.vectors.distance}")

    # Validate vector size
    if collection_info.config.params.vectors.size != 1536:
        logger.error(f"❌ Vector size mismatch: expected 1536, got {collection_info.config.params.vectors.size}")
        return False

    logger.info("✅ Vector size correct (1536)")

    # Validate distance metric
    expected_distance = "Cosine"
    if str(collection_info.config.params.vectors.distance) != expected_distance:
        logger.error(f"❌ Distance metric mismatch: expected {expected_distance}, got {collection_info.config.params.vectors.distance}")
        return False

    logger.info(f"✅ Distance metric correct ({expected_distance})")

    # Check if collection has points
    if collection_info.points_count == 0:
        logger.warning("⚠️ Collection is empty (0 points)")
        return False

    logger.info(f"✅ Collection has {collection_info.points_count} points")

    # Sample a few points to verify payload structure
    scroll_result = client.scroll(
        collection_name=collection_name,
        limit=5
    )

    if not scroll_result[0]:
        logger.warning("⚠️ Could not retrieve sample points")
        return False

    logger.info(f"Validating {len(scroll_result[0])} sample points...")

    required_fields = ["chunk_id", "chapter_id", "chapter_title", "content", "chunk_index"]

    for i, point in enumerate(scroll_result[0]):
        payload = point.payload

        # Check required fields
        missing_fields = [field for field in required_fields if field not in payload]

        if missing_fields:
            logger.error(f"❌ Point {i+1} missing fields: {missing_fields}")
            return False

        # Validate content_type
        if "content_type" in payload and payload["content_type"] not in ["text", "code", "mixed"]:
            logger.error(f"❌ Point {i+1} has invalid content_type: {payload['content_type']}")
            return False

        # Validate hardware_relevance
        if "hardware_relevance" in payload:
            if not isinstance(payload["hardware_relevance"], list):
                logger.error(f"❌ Point {i+1} hardware_relevance is not a list")
                return False

        logger.info(f"✅ Point {i+1} valid: chapter={payload.get('chapter_id')}, tokens={payload.get('token_count')}")

    logger.info("🎉 Validation passed! Collection is ready for RAG queries")
    return True


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Validate Qdrant embeddings")
    parser.add_argument(
        "--collection",
        type=str,
        default=None,
        help="Qdrant collection name (default from config)"
    )

    args = parser.parse_args()

    success = validate_embeddings(args.collection)
    sys.exit(0 if success else 1)
