"""
Automated setup script for RAG Chatbot backend.
Runs all setup steps in sequence: migrations → Qdrant → embeddings.
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from utils.logger import logger
from utils.config import settings


async def main():
    """Run complete setup sequence."""

    print("=" * 60)
    print("RAG Chatbot Backend - Automated Setup")
    print("=" * 60)
    print()

    # Check environment variables
    logger.info("Step 1/4: Checking environment configuration...")

    required_vars = [
        ("OPENAI_API_KEY", settings.openai_api_key),
        ("QDRANT_URL", settings.qdrant_url),
        ("QDRANT_API_KEY", settings.qdrant_api_key),
        ("NEON_DATABASE_URL", settings.neon_database_url)
    ]

    missing_vars = []
    for var_name, var_value in required_vars:
        if not var_value or var_value.startswith("your-") or var_value.startswith("sk-proj-your"):
            missing_vars.append(var_name)
            logger.error(f"❌ {var_name} not configured")
        else:
            # Mask sensitive values
            masked = var_value[:10] + "..." + var_value[-5:] if len(var_value) > 15 else "***"
            logger.info(f"✅ {var_name}: {masked}")

    if missing_vars:
        logger.error(f"\n❌ Missing required environment variables: {', '.join(missing_vars)}")
        logger.error("Please configure your .env file with valid credentials.")
        logger.error("See .env.example for reference.")
        return False

    logger.info("✅ All required environment variables configured\n")

    # Step 2: Database migrations
    logger.info("Step 2/4: Running database migrations...")
    try:
        from run_migrations import run_all_migrations
        await run_all_migrations()
        logger.info("✅ Database migrations complete\n")
    except Exception as e:
        logger.error(f"❌ Database migration failed: {str(e)}")
        logger.error("Please check your NEON_DATABASE_URL and network connectivity.")
        return False

    # Step 3: Qdrant setup
    logger.info("Step 3/4: Setting up Qdrant collection...")
    try:
        from setup_qdrant import setup_qdrant_collection
        setup_qdrant_collection()
        logger.info("✅ Qdrant collection created\n")
    except Exception as e:
        logger.error(f"❌ Qdrant setup failed: {str(e)}")
        logger.error("Please check your QDRANT_URL and QDRANT_API_KEY.")
        return False

    # Step 4: Embed book content
    logger.info("Step 4/4: Embedding book content...")

    docs_dir = Path(__file__).parent.parent.parent / "humanoid_robot_book" / "docs"

    if not docs_dir.exists():
        logger.error(f"❌ Docs directory not found: {docs_dir}")
        logger.error("Please ensure your book content is in humanoid_robot_book/docs/")
        return False

    mdx_files = list(docs_dir.glob("*.mdx"))
    if not mdx_files:
        logger.warning(f"⚠️ No MDX files found in {docs_dir}")
        logger.warning("Skipping embedding step (no content to embed)")
        return True

    logger.info(f"Found {len(mdx_files)} MDX files to embed")

    # Confirm before embedding (costs API credits)
    print()
    print("⚠️  IMPORTANT: Embedding will use OpenAI API credits (~$0.01-0.05)")
    print(f"   Files to process: {len(mdx_files)}")
    print(f"   Model: {settings.openai_embedding_model}")
    print()

    response = input("Continue with embedding? (yes/no): ").strip().lower()

    if response != "yes":
        logger.info("Embedding cancelled by user")
        logger.info("You can run it later with: python scripts/embed_book_content.py")
        return True

    try:
        from embed_book_content import embed_book_content
        embed_book_content(docs_dir)
        logger.info("✅ Book content embedded successfully\n")
    except Exception as e:
        logger.error(f"❌ Embedding failed: {str(e)}")
        logger.error("You can retry with: python scripts/embed_book_content.py")
        return False

    # Validate embeddings
    logger.info("Validating embeddings...")
    try:
        from validate_embeddings import validate_embeddings
        if validate_embeddings():
            logger.info("✅ Embeddings validated successfully\n")
        else:
            logger.warning("⚠️ Embedding validation failed")
            return False
    except Exception as e:
        logger.error(f"❌ Validation failed: {str(e)}")
        return False

    # Success!
    print()
    print("=" * 60)
    print("🎉 Setup Complete!")
    print("=" * 60)
    print()
    print("Your RAG Chatbot backend is ready to use!")
    print()
    print("Next steps:")
    print("  1. Start the API server:")
    print("     cd src && python main.py")
    print()
    print("  2. Test the API:")
    print("     Open http://localhost:8000/docs")
    print()
    print("  3. Try a query:")
    print("     curl -X POST http://localhost:8000/api/chat/query \\")
    print("       -H 'Content-Type: application/json' \\")
    print("       -d '{\"question\": \"What is ROS 2?\"}'")
    print()

    return True


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
