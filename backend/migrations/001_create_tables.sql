-- Migration 001: Create tables for RAG Chatbot
-- Date: 2025-12-11
-- Description: Initial database schema for user sessions, conversations, and messages

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Table: user_sessions
-- Purpose: Track user sessions (anonymous or authenticated)
CREATE TABLE IF NOT EXISTS user_sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NULL,  -- References Better-Auth users table (nullable for anonymous users)
    hardware_profile JSONB NULL,  -- {type: "GPU"|"Edge"|"Cloud", details: {...}}
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_active TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes for user_sessions
CREATE INDEX IF NOT EXISTS idx_user_sessions_user_id ON user_sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_user_sessions_last_active ON user_sessions(last_active);

-- Comments for user_sessions
COMMENT ON TABLE user_sessions IS 'Tracks user sessions for conversation context (anonymous or authenticated)';
COMMENT ON COLUMN user_sessions.hardware_profile IS 'JSON object containing hardware type and configuration from Better-Auth profile';

-- Table: conversations
-- Purpose: Group related messages into conversation threads
CREATE TABLE IF NOT EXISTS conversations (
    conv_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES user_sessions(session_id) ON DELETE CASCADE,
    title VARCHAR(200) NULL,  -- Auto-generated from first user message
    summary TEXT NULL,  -- LLM-generated summary after 20+ messages
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes for conversations
CREATE INDEX IF NOT EXISTS idx_conversations_session_id ON conversations(session_id);
CREATE INDEX IF NOT EXISTS idx_conversations_updated_at ON conversations(updated_at DESC);

-- Comments for conversations
COMMENT ON TABLE conversations IS 'Chat threads grouping related Q&A exchanges';
COMMENT ON COLUMN conversations.summary IS 'Compressed summary of messages 1-10 when conversation exceeds 20 messages';

-- Table: messages
-- Purpose: Store individual user and assistant messages
DO $$ BEGIN
    CREATE TYPE message_role AS ENUM ('user', 'assistant');
EXCEPTION
    WHEN duplicate_object THEN null;
END $$;

CREATE TABLE IF NOT EXISTS messages (
    message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conv_id UUID NOT NULL REFERENCES conversations(conv_id) ON DELETE CASCADE,
    role message_role NOT NULL,
    content TEXT NOT NULL,  -- User question or assistant response
    text_selection JSONB NULL,  -- {text: string, chapter_id: string, start: int, end: int}
    metadata JSONB NULL,  -- {tokens: int, latency_ms: int, model: string, retrieved_chunks: int}
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes for messages
CREATE INDEX IF NOT EXISTS idx_messages_conv_id_created_at ON messages(conv_id, created_at DESC);
CREATE INDEX IF NOT EXISTS idx_messages_role ON messages(role);

-- Comments for messages
COMMENT ON TABLE messages IS 'Individual messages (user questions + assistant responses) in conversations';
COMMENT ON COLUMN messages.text_selection IS 'If user highlighted text, stores selection context for retrieval';
COMMENT ON COLUMN messages.metadata IS 'Performance and debugging metadata (token counts, latency, model used)';

-- Trigger: Auto-update conversations.updated_at on new message
CREATE OR REPLACE FUNCTION update_conversation_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    UPDATE conversations SET updated_at = NOW() WHERE conv_id = NEW.conv_id;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Drop trigger if exists (for idempotency)
DROP TRIGGER IF EXISTS trg_update_conversation_timestamp ON messages;

CREATE TRIGGER trg_update_conversation_timestamp
AFTER INSERT ON messages
FOR EACH ROW
EXECUTE FUNCTION update_conversation_timestamp();

-- Grant permissions (adjust based on your database user)
-- GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO your_db_user;
-- GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO your_db_user;
