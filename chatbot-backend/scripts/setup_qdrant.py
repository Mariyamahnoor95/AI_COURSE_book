"""
Setup script for Qdrant Cloud collection

This script creates the 'textbook_chunks' collection for content embeddings.

Prerequisites:
1. Sign up for Qdrant Cloud Free Tier: https://cloud.qdrant.io/
2. Create a cluster (Free tier: 1GB storage)
3. Get API key and cluster URL from Qdrant console
4. Add credentials to .env file:
   QDRANT_URL=https://your-cluster.qdrant.cloud:6333
   QDRANT_API_KEY=your-api-key

Usage:
  python chatbot-backend/scripts/setup_qdrant.py
"""

import os
import sys
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

# Load environment variables
load_dotenv()

def setup_qdrant_collection():
    """Create Qdrant collection for textbook content embeddings"""

    # Get credentials from environment
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        print("❌ Error: QDRANT_URL and QDRANT_API_KEY must be set in .env file")
        print("\nManual setup instructions:")
        print("1. Go to https://cloud.qdrant.io/")
        print("2. Sign up and create a cluster (Free tier: 1GB)")
        print("3. Copy cluster URL and API key")
        print("4. Create .env file from .env.example")
        print("5. Add your credentials to .env")
        sys.exit(1)

    try:
        # Connect to Qdrant Cloud
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
        )

        collection_name = "textbook_chunks"

        # Check if collection already exists
        collections = client.get_collections().collections
        if any(col.name == collection_name for col in collections):
            print(f"⚠️  Collection '{collection_name}' already exists")
            response = input("Delete and recreate? (y/n): ")
            if response.lower() == 'y':
                client.delete_collection(collection_name)
                print(f"🗑️  Deleted existing collection '{collection_name}'")
            else:
                print("✅ Using existing collection")
                return

        # Create collection
        # - 1536 dimensions (OpenAI text-embedding-3-small per research.md)
        # - Cosine distance (standard for semantic search)
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1536,  # OpenAI text-embedding-3-small dimension
                distance=Distance.COSINE,
            ),
        )

        print(f"✅ Created collection '{collection_name}'")
        print(f"   - Vectors: 1536 dimensions")
        print(f"   - Distance metric: Cosine")
        print(f"   - Storage capacity: 1GB (Free tier)")
        print(f"   - Expected usage: ~1.5MB for 230 chunks (0.15% of limit)")

        # Verify collection
        collection_info = client.get_collection(collection_name)
        print(f"\n📊 Collection info:")
        print(f"   - Points count: {collection_info.points_count}")
        print(f"   - Status: {collection_info.status}")

    except Exception as e:
        print(f"❌ Error setting up Qdrant collection: {e}")
        sys.exit(1)

if __name__ == "__main__":
    print("🚀 Setting up Qdrant Cloud collection...")
    setup_qdrant_collection()
    print("\n✅ Qdrant setup complete!")
    print("\nNext steps:")
    print("1. Run content ingestion: python chatbot-backend/scripts/ingest_content.py")
    print("2. Verify embeddings were created")
