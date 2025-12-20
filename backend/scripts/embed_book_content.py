"""
Script to embed book content (MDX files) into Qdrant vector store.
Parses docs/*.mdx, chunks content, generates embeddings, and uploads to Qdrant.
"""

import sys
import re
from pathlib import Path
from typing import List, Dict, Any
from uuid import uuid4

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
from src.services.embedding_service import embedding_service
from src.utils.config import settings
from src.utils.logger import logger


def parse_mdx_file(file_path: Path) -> Dict[str, Any]:
    """
    Parse an MDX file and extract content.

    Args:
        file_path: Path to MDX file

    Returns:
        Dictionary with chapter metadata and content
    """
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Extract chapter ID from filename (e.g., "01-introduction.mdx" -> "01-introduction")
    chapter_id = file_path.stem

    # Extract title from first H1 heading
    title_match = re.search(r"^#\s+(.+)$", content, re.MULTILINE)
    chapter_title = title_match.group(1) if title_match else chapter_id

    # Remove frontmatter (YAML between ---) if present
    content = re.sub(r"^---\n.*?\n---\n", "", content, flags=re.DOTALL)

    # Extract sections (H2 headings)
    sections = re.split(r"\n##\s+", content)

    logger.info(f"Parsed {file_path.name}: {len(sections)} sections, {len(content)} chars")

    return {
        "chapter_id": chapter_id,
        "chapter_title": chapter_title,
        "content": content,
        "file_path": str(file_path)
    }


def detect_content_type(text: str) -> str:
    """
    Detect if chunk is text, code, or mixed.

    Args:
        text: Chunk text

    Returns:
        "text", "code", or "mixed"
    """
    # Simple heuristic: if >30% of lines start with code patterns, it's code
    lines = text.split('\n')
    code_lines = sum(
        1 for line in lines
        if line.strip().startswith(('import ', 'from ', 'def ', 'class ', 'const ', 'let ', 'var ', '```'))
    )

    code_ratio = code_lines / len(lines) if lines else 0

    if code_ratio > 0.3:
        return "code" if code_ratio > 0.7 else "mixed"
    return "text"


def detect_hardware_relevance(text: str) -> List[str]:
    """
    Detect which hardware types this content is relevant to.

    Args:
        text: Chunk text

    Returns:
        List of hardware types: ["GPU", "Edge", "Cloud", "all"]
    """
    text_lower = text.lower()

    hardware_tags = []

    # GPU keywords
    if any(keyword in text_lower for keyword in [
        "rtx", "gpu", "cuda", "isaac sim", "omniverse", "local", "workstation"
    ]):
        hardware_tags.append("GPU")

    # Edge keywords
    if any(keyword in text_lower for keyword in [
        "jetson", "edge", "embedded", "orin", "nano"
    ]):
        hardware_tags.append("Edge")

    # Cloud keywords
    if any(keyword in text_lower for keyword in [
        "cloud", "aws", "azure", "robomaker", "omniverse cloud", "streaming"
    ]):
        hardware_tags.append("Cloud")

    # If no specific hardware mentioned, mark as "all"
    if not hardware_tags:
        hardware_tags.append("all")

    return hardware_tags


def embed_book_content(docs_dir: Path, collection_name: str = None):
    """
    Main function to embed all book content.

    Args:
        docs_dir: Path to docs directory containing MDX files
        collection_name: Qdrant collection name (default from config)
    """
    collection_name = collection_name or settings.qdrant_collection

    # Initialize Qdrant client
    logger.info(f"Connecting to Qdrant: {settings.qdrant_url}")
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )

    # Get all MDX files
    mdx_files = sorted(docs_dir.glob("*.mdx"))

    if not mdx_files:
        logger.warning(f"No MDX files found in {docs_dir}")
        return

    logger.info(f"Found {len(mdx_files)} MDX files to process")

    all_points = []

    # Process each file
    for mdx_file in mdx_files:
        logger.info(f"Processing: {mdx_file.name}")

        # Parse MDX file
        chapter_data = parse_mdx_file(mdx_file)

        # Chunk content
        chunks = embedding_service.chunk_text(
            chapter_data["content"],
            chunk_size=settings.rag_chunk_size,
            chunk_overlap=settings.rag_chunk_overlap
        )

        # Generate embeddings
        chunks_with_embeddings = embedding_service.embed_chunks(chunks)

        # Create Qdrant points
        for chunk in chunks_with_embeddings:
            point_id = str(uuid4())

            # Detect content type and hardware relevance
            content_type = detect_content_type(chunk["text"])
            hardware_relevance = detect_hardware_relevance(chunk["text"])

            # Create payload
            payload = {
                "chunk_id": point_id,
                "chapter_id": chapter_data["chapter_id"],
                "chapter_title": chapter_data["chapter_title"],
                "chunk_index": chunk["chunk_index"],
                "content": chunk["text"],
                "content_type": content_type,
                "hardware_relevance": hardware_relevance,
                "token_count": chunk["token_count"],
                "embedding_model": embedding_service.model
            }

            # Create point
            point = PointStruct(
                id=point_id,
                vector=chunk["embedding"],
                payload=payload
            )

            all_points.append(point)

        logger.info(f"✅ Processed {mdx_file.name}: {len(chunks_with_embeddings)} chunks")

    # Upload to Qdrant in batches
    batch_size = 100
    total_uploaded = 0

    for i in range(0, len(all_points), batch_size):
        batch = all_points[i:i + batch_size]

        try:
            client.upsert(
                collection_name=collection_name,
                points=batch
            )
            total_uploaded += len(batch)
            logger.info(f"Uploaded batch {i // batch_size + 1}: {total_uploaded}/{len(all_points)} points")

        except Exception as e:
            logger.error(f"Failed to upload batch: {str(e)}")
            raise

    logger.info(f"🎉 Successfully embedded {total_uploaded} chunks from {len(mdx_files)} chapters")

    # Verify collection
    collection_info = client.get_collection(collection_name)
    logger.info(f"Collection '{collection_name}' now has {collection_info.points_count} points")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Embed book content into Qdrant")
    parser.add_argument(
        "--docs-dir",
        type=Path,
        default=Path(__file__).parent.parent.parent / "humanoid_robot_book" / "docs",
        help="Path to docs directory (default: ../humanoid_robot_book/docs)"
    )
    parser.add_argument(
        "--collection",
        type=str,
        default=None,
        help="Qdrant collection name (default from config)"
    )

    args = parser.parse_args()

    if not args.docs_dir.exists():
        logger.error(f"Docs directory not found: {args.docs_dir}")
        sys.exit(1)

    embed_book_content(args.docs_dir, args.collection)
