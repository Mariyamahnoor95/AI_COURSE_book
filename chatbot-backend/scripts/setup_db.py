"""
Setup script for Neon Postgres database schema

This script creates the chat_sessions and chat_messages tables for chat history.

Prerequisites:
1. Sign up for Neon Free Tier: https://neon.tech/
2. Create a project (Free tier: 0.5GB storage)
3. Get connection string from Neon console
4. Add to .env file:
   NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/dbname

Usage:
  python chatbot-backend/scripts/setup_db.py
"""

import os
import sys
from dotenv import load_dotenv
import psycopg2
from psycopg2.extensions import ISOLATION_LEVEL_AUTOCOMMIT

# Load environment variables
load_dotenv()

def setup_database_schema():
    """Create database tables for chat sessions and messages"""

    # Get connection string from environment
    database_url = os.getenv("NEON_DATABASE_URL")

    if not database_url:
        print("❌ Error: NEON_DATABASE_URL must be set in .env file")
        print("\nManual setup instructions:")
        print("1. Go to https://neon.tech/")
        print("2. Sign up and create a project (Free tier: 0.5GB)")
        print("3. Copy connection string from console")
        print("4. Create .env file from .env.example")
        print("5. Add your connection string to .env")
        sys.exit(1)

    try:
        # Connect to Neon Postgres
        conn = psycopg2.connect(database_url)
        conn.set_isolation_level(ISOLATION_LEVEL_AUTOCOMMIT)
        cursor = conn.cursor()

        print("🚀 Creating database schema...")

        # Create chat_sessions table (per data-model.md)
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chat_sessions (
                session_id UUID PRIMARY KEY,
                user_id VARCHAR(255),
                created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
                updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
                expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
                context_selection JSONB NOT NULL,
                CONSTRAINT valid_context_mode CHECK (
                    context_selection->>'mode' IN ('full_textbook', 'module', 'chapter')
                )
            );
        """)
        print("✅ Created table: chat_sessions")

        # Create chat_messages table (per data-model.md)
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chat_messages (
                message_id UUID PRIMARY KEY,
                session_id UUID NOT NULL REFERENCES chat_sessions(session_id) ON DELETE CASCADE,
                role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
                content TEXT NOT NULL,
                timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
                citations JSONB,
                response_time_ms INTEGER,
                CONSTRAINT user_content_length CHECK (
                    (role = 'user' AND LENGTH(content) <= 500) OR
                    (role = 'assistant' AND LENGTH(content) <= 10000)
                ),
                CONSTRAINT assistant_citations CHECK (
                    (role = 'assistant' AND citations IS NOT NULL) OR
                    (role = 'user' AND citations IS NULL)
                ),
                CONSTRAINT assistant_response_time CHECK (
                    (role = 'assistant' AND response_time_ms IS NOT NULL) OR
                    (role = 'user' AND response_time_ms IS NULL)
                )
            );
        """)
        print("✅ Created table: chat_messages")

        # Create indexes for performance
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_chat_sessions_expires_at
            ON chat_sessions(expires_at);
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id
            ON chat_messages(session_id);
        """)
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_chat_messages_timestamp
            ON chat_messages(timestamp);
        """)
        print("✅ Created indexes for query optimization")

        # Create function to auto-update updated_at timestamp
        cursor.execute("""
            CREATE OR REPLACE FUNCTION update_updated_at_column()
            RETURNS TRIGGER AS $$
            BEGIN
                NEW.updated_at = NOW();
                NEW.expires_at = NOW() + INTERVAL '24 hours';
                RETURN NEW;
            END;
            $$ LANGUAGE plpgsql;
        """)

        cursor.execute("""
            DROP TRIGGER IF EXISTS update_chat_sessions_updated_at ON chat_sessions;
        """)

        cursor.execute("""
            CREATE TRIGGER update_chat_sessions_updated_at
            BEFORE UPDATE ON chat_sessions
            FOR EACH ROW
            EXECUTE FUNCTION update_updated_at_column();
        """)
        print("✅ Created trigger for automatic timestamp updates")

        # Verify tables
        cursor.execute("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_name IN ('chat_sessions', 'chat_messages');
        """)
        tables = cursor.fetchall()
        print(f"\n📊 Verified tables: {[t[0] for t in tables]}")

        # Estimate storage
        print(f"\n💾 Storage estimates (per data-model.md):")
        print(f"   - 1,000 sessions × 10 messages = ~10.5MB")
        print(f"   - Free tier: 0.5GB (500MB)")
        print(f"   - Expected usage: ~2% of limit")

        cursor.close()
        conn.close()

    except Exception as e:
        print(f"❌ Error setting up database: {e}")
        sys.exit(1)

if __name__ == "__main__":
    print("🚀 Setting up Neon Postgres database schema...")
    setup_database_schema()
    print("\n✅ Database setup complete!")
    print("\nNext steps:")
    print("1. Tables ready for chat history storage")
    print("2. Implement chat history service (T012)")
