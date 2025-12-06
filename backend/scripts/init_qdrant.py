"""
Qdrant Initialization Script

Creates content embedding collections for English and Urdu content.

Usage:
    python scripts/init_qdrant.py
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from src.db.qdrant_client import get_qdrant_service, COLLECTION_EN, COLLECTION_UR


def main():
    """
    Initialize Qdrant collections for content embeddings.
    """
    print("🔧 Initializing Qdrant Vector Database...")
    print("=" * 60)

    try:
        # Get Qdrant service instance
        qdrant = get_qdrant_service()

        # Test connection
        print("\n1. Testing Qdrant connection...")
        if not qdrant.check_connection():
            print("❌ Qdrant connection failed. Please check your credentials.")
            sys.exit(1)
        print("✅ Connection successful")

        # Create English collection
        print(f"\n2. Creating collection: {COLLECTION_EN}")
        if qdrant.create_collection(COLLECTION_EN):
            print(f"✅ Collection '{COLLECTION_EN}' ready")
        else:
            print(f"❌ Failed to create collection '{COLLECTION_EN}'")
            sys.exit(1)

        # Create Urdu collection
        print(f"\n3. Creating collection: {COLLECTION_UR}")
        if qdrant.create_collection(COLLECTION_UR):
            print(f"✅ Collection '{COLLECTION_UR}' ready")
        else:
            print(f"❌ Failed to create collection '{COLLECTION_UR}'")
            sys.exit(1)

        # Display collection info
        print("\n4. Collection Information:")
        print("-" * 60)

        for collection_name in [COLLECTION_EN, COLLECTION_UR]:
            info = qdrant.get_collection_info(collection_name)
            if info:
                print(f"\n📊 {collection_name}:")
                print(f"   Status: {info['status']}")
                print(f"   Vectors: {info['vectors_count']}")
                print(f"   Points: {info['points_count']}")

        print("\n" + "=" * 60)
        print("✅ Qdrant initialization complete!")
        print("\nNext steps:")
        print("  1. Run content indexing script to populate collections")
        print("  2. Start the FastAPI backend server")

    except Exception as e:
        print(f"\n❌ Initialization failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
