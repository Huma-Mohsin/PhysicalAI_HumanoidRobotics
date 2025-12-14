"""
Test script to verify RAG pipeline is working end-to-end.
Tests: embeddings → Qdrant search → LLM generation
"""

import sys
import asyncio
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from services.database_service import db_service
from services.rag_service import rag_service
from utils.logger import logger
from utils.config import settings


async def test_rag_pipeline():
    """Test complete RAG pipeline."""

    print("=" * 60)
    print("RAG Pipeline Test")
    print("=" * 60)
    print()

    # Test 1: Database connection
    logger.info("Test 1/4: Database connection...")
    try:
        await db_service.connect()
        logger.info("✅ Database connection successful\n")
    except Exception as e:
        logger.error(f"❌ Database connection failed: {str(e)}")
        return False

    # Test 2: Qdrant collection stats
    logger.info("Test 2/4: Qdrant collection stats...")
    try:
        from services.qdrant_service import qdrant_service
        stats = qdrant_service.get_collection_stats()

        logger.info(f"Collection: {stats['collection_name']}")
        logger.info(f"Points: {stats['points_count']}")
        logger.info(f"Vector size: {stats['vector_size']}")
        logger.info(f"Distance: {stats['distance_metric']}")

        if stats['points_count'] == 0:
            logger.error("❌ Collection is empty! Run embed_book_content.py first.")
            return False

        logger.info(f"✅ Qdrant ready with {stats['points_count']} chunks\n")

    except Exception as e:
        logger.error(f"❌ Qdrant check failed: {str(e)}")
        return False

    # Test 3: RAG query (no conversation history)
    logger.info("Test 3/4: RAG query (general Q&A)...")

    test_questions = [
        "What is ROS 2?",
        "What are the main components of ROS 2?",
        "How do I set up NVIDIA Isaac Sim?"
    ]

    print("\nSelect a test question:")
    for i, q in enumerate(test_questions, 1):
        print(f"  {i}. {q}")
    print(f"  {len(test_questions) + 1}. Custom question")

    try:
        choice = int(input("\nEnter choice (1-4): ").strip())
        if choice == len(test_questions) + 1:
            question = input("Enter your question: ").strip()
        else:
            question = test_questions[choice - 1]
    except (ValueError, IndexError):
        question = test_questions[0]

    logger.info(f"\nQuestion: {question}")

    try:
        result = rag_service.query_book(
            question=question,
            conversation_history=[],
            hardware_profile=None
        )

        logger.info(f"\n✅ RAG query successful!")
        logger.info(f"Latency: {result['metadata'].latency_ms}ms")
        logger.info(f"Tokens: {result['metadata'].tokens_used}")
        logger.info(f"Retrieved chunks: {result['metadata'].retrieved_chunks}")

        print("\n" + "=" * 60)
        print("RESPONSE:")
        print("=" * 60)
        print(result['response'])
        print("=" * 60)

        print("\nRETRIEVED CHUNKS:")
        for i, chunk in enumerate(result['retrieved_chunks'], 1):
            print(f"\n{i}. {chunk['chapter_title']} (similarity: {chunk['similarity_score']:.3f})")
            print(f"   {chunk['excerpt'][:150]}...")

        logger.info("\n✅ RAG pipeline working correctly!\n")

    except Exception as e:
        logger.error(f"❌ RAG query failed: {str(e)}", exc_info=True)
        return False

    # Test 4: Session & conversation storage
    logger.info("Test 4/4: Database storage (session + conversation + messages)...")

    try:
        from models.user_session import UserSessionCreate
        from models.conversation import ConversationCreate
        from models.message import MessageCreate

        # Create session
        session = await db_service.create_session(UserSessionCreate())
        logger.info(f"✅ Created session: {session.session_id}")

        # Create conversation
        conversation = await db_service.create_conversation(
            ConversationCreate(session_id=session.session_id, title="Test Conversation")
        )
        logger.info(f"✅ Created conversation: {conversation.conv_id}")

        # Save user message
        user_msg = await db_service.create_message(
            MessageCreate(
                conv_id=conversation.conv_id,
                role="user",
                content=question
            )
        )
        logger.info(f"✅ Saved user message: {user_msg.message_id}")

        # Save assistant response
        assistant_msg = await db_service.create_message(
            MessageCreate(
                conv_id=conversation.conv_id,
                role="assistant",
                content=result['response'],
                metadata=result['metadata']
            )
        )
        logger.info(f"✅ Saved assistant message: {assistant_msg.message_id}")

        # Fetch messages
        messages = await db_service.get_messages_by_conversation(conversation.conv_id)
        logger.info(f"✅ Retrieved {len(messages)} messages from conversation")

        logger.info("\n✅ Database storage working correctly!\n")

    except Exception as e:
        logger.error(f"❌ Database storage test failed: {str(e)}")
        return False

    # Cleanup
    await db_service.disconnect()

    # All tests passed!
    print("\n" + "=" * 60)
    print("🎉 All Tests Passed!")
    print("=" * 60)
    print("\nYour RAG chatbot backend is fully functional!")
    print("\nNext steps:")
    print("  1. Start the API server: python src/main.py")
    print("  2. Test the API: http://localhost:8000/docs")
    print("  3. Build the frontend React UI")
    print()

    return True


if __name__ == "__main__":
    success = asyncio.run(test_rag_pipeline())
    sys.exit(0 if success else 1)
