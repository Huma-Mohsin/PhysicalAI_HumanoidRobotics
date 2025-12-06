"""
Content Embedding Script

Processes all Markdown files from frontend/docs/ and creates embeddings
for the RAG chatbot.

Usage:
    python backend/scripts/embed_content.py

Requirements:
    - OPENAI_API_KEY in .env
    - QDRANT_URL and QDRANT_API_KEY in .env
    - Qdrant collections initialized (run init_qdrant.py first)
"""

import os
import sys
from pathlib import Path
from typing import List, Dict
import re
from dotenv import load_dotenv

# Add parent directory to path to import from src
sys.path.append(str(Path(__file__).parent.parent))

load_dotenv()

from src.services.openai_service import OpenAIService
from src.db.qdrant_client import QdrantService


def extract_text_from_markdown(file_path: Path) -> str:
    """
    Extract plain text content from Markdown file

    Args:
        file_path: Path to markdown file

    Returns:
        Cleaned text content
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Remove YAML frontmatter
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

    # Remove code blocks but keep the code content
    content = re.sub(r'```[\w]*\n', '\n', content)
    content = re.sub(r'```', '', content)

    # Remove markdown formatting
    content = re.sub(r'#{1,6}\s', '', content)  # Headers
    content = re.sub(r'\*\*(.*?)\*\*', r'\1', content)  # Bold
    content = re.sub(r'\*(.*?)\*', r'\1', content)  # Italic
    content = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', content)  # Links

    return content.strip()


def chunk_text(text: str, max_chunk_size: int = 1000) -> List[str]:
    """
    Split text into chunks for embedding

    Args:
        text: Input text
        max_chunk_size: Maximum characters per chunk

    Returns:
        List of text chunks
    """
    # Split by double newlines (paragraphs)
    paragraphs = text.split('\n\n')

    chunks = []
    current_chunk = ""

    for para in paragraphs:
        if len(current_chunk) + len(para) < max_chunk_size:
            current_chunk += para + "\n\n"
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = para + "\n\n"

    if current_chunk:
        chunks.append(current_chunk.strip())

    return chunks


def process_docs_directory(docs_path: Path, language: str = 'en') -> List[Dict]:
    """
    Process all markdown files in docs directory

    Args:
        docs_path: Path to frontend/docs directory
        language: Language code ('en' or 'ur')

    Returns:
        List of document chunks with metadata
    """
    documents = []

    # Filter files by language suffix
    pattern = "*-ur.md" if language == 'ur' else "*.md"

    for md_file in docs_path.rglob(pattern):
        # Skip Urdu files when processing English
        if language == 'en' and md_file.name.endswith('-ur.md'):
            continue

        print(f"Processing: {md_file.name}")

        text = extract_text_from_markdown(md_file)
        chunks = chunk_text(text)

        # Extract metadata from path
        relative_path = md_file.relative_to(docs_path)
        module_name = str(relative_path.parent) if relative_path.parent != Path('.') else 'intro'
        file_name = md_file.stem.replace('-ur', '')  # Remove -ur suffix

        for i, chunk in enumerate(chunks):
            documents.append({
                "content": chunk,
                "metadata": {
                    "file": str(relative_path),
                    "module": module_name,
                    "chapter": file_name,
                    "chunk_index": i,
                    "total_chunks": len(chunks),
                    "language": language
                }
            })

    return documents


def embed_documents(
    documents: List[Dict],
    openai_service: OpenAIService,
    qdrant_service: QdrantService,
    collection_name: str = "content_embeddings_en"
):
    """
    Generate embeddings and upload to Qdrant

    Args:
        documents: List of document chunks
        openai_service: OpenAI service instance
        qdrant_service: Qdrant service instance
        collection_name: Target collection name
    """
    from qdrant_client.models import PointStruct
    import uuid

    print(f"\nEmbedding {len(documents)} chunks...")

    points = []

    for i, doc in enumerate(documents):
        if i % 10 == 0:
            print(f"Progress: {i}/{len(documents)}")

        # Generate embedding
        embedding = openai_service.generate_embedding(doc["content"])

        # Create point for Qdrant
        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "content": doc["content"],
                "metadata": doc["metadata"]
            }
        )

        points.append(point)

    # Upload to Qdrant in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        qdrant_service.client.upsert(
            collection_name=collection_name,
            points=batch
        )
        print(f"Uploaded batch {i // batch_size + 1}")

    print(f"\n✅ Successfully embedded {len(documents)} chunks to {collection_name}")


def main():
    """Main execution"""
    print("🚀 Content Embedding Script for Physical AI RAG Platform\n")

    # Check environment variables
    required_vars = ["OPENAI_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"❌ Missing environment variables: {', '.join(missing_vars)}")
        print("Please set them in backend/.env")
        sys.exit(1)

    # Initialize services
    print("Initializing services...")
    openai_service = OpenAIService()
    qdrant_service = QdrantService()

    # Check Qdrant connection
    if not qdrant_service.check_connection():
        print("❌ Failed to connect to Qdrant. Run init_qdrant.py first.")
        sys.exit(1)

    # Find docs directory
    project_root = Path(__file__).parent.parent.parent
    docs_path = project_root / "frontend" / "docs"

    if not docs_path.exists():
        print(f"❌ Docs directory not found: {docs_path}")
        sys.exit(1)

    print(f"📁 Processing docs from: {docs_path}\n")

    # Process and embed English content
    print("=" * 60)
    print("EMBEDDING ENGLISH CONTENT")
    print("=" * 60)

    collection_en = "content_embeddings_en"
    try:
        qdrant_service.client.get_collection(collection_en)
        print(f"✅ Collection '{collection_en}' found")
    except Exception:
        print(f"❌ Collection '{collection_en}' not found. Run init_qdrant.py first.")
        sys.exit(1)

    documents_en = process_docs_directory(docs_path, language='en')
    print(f"\n📊 Extracted {len(documents_en)} English chunks")
    embed_documents(documents_en, openai_service, qdrant_service, collection_en)

    # Process and embed Urdu content
    print("\n" + "=" * 60)
    print("EMBEDDING URDU CONTENT")
    print("=" * 60)

    collection_ur = "content_embeddings_ur"
    try:
        qdrant_service.client.get_collection(collection_ur)
        print(f"✅ Collection '{collection_ur}' found")
    except Exception:
        print(f"⚠️  Collection '{collection_ur}' not found. Skipping Urdu embedding.")
        print("Run init_qdrant.py to create Urdu collection.")
        collection_ur = None

    if collection_ur:
        documents_ur = process_docs_directory(docs_path, language='ur')
        print(f"\n📊 Extracted {len(documents_ur)} Urdu chunks")

        if len(documents_ur) > 0:
            embed_documents(documents_ur, openai_service, qdrant_service, collection_ur)
        else:
            print("⚠️  No Urdu content found. Create *-ur.md files to enable Urdu search.")

    print("\n🎉 Content embedding complete!")
    print(f"\nYou can now query the RAG chatbot with:")
    print("POST http://localhost:8000/chat")
    print('{"query": "How do I create a ROS 2 node with GPT-4?", "language": "en"}')
    print('{"query": "ROS 2 node کیسے بنائیں؟", "language": "ur"}')


if __name__ == "__main__":
    main()
