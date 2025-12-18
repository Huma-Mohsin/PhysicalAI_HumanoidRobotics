-- Migration 003: Add password column to user_profiles for Better-Auth
-- Created: 2025-12-18
-- Purpose: Add password field for authentication (user_profiles table already exists from migration 002)

-- Add password column to existing user_profiles table
ALTER TABLE user_profiles
ADD COLUMN IF NOT EXISTS password VARCHAR(255);

-- Check if user_sessions.user_id column exists and drop it if wrong type
DO $$
BEGIN
    -- Drop the column if it exists with wrong type (UUID)
    IF EXISTS (
        SELECT 1 FROM information_schema.columns
        WHERE table_name = 'user_sessions'
        AND column_name = 'user_id'
        AND data_type = 'uuid'
    ) THEN
        ALTER TABLE user_sessions DROP COLUMN user_id;
    END IF;
END $$;

-- Add user_id foreign key to user_sessions table (links sessions to users)
-- Note: user_profiles.user_id is VARCHAR(255), not UUID
ALTER TABLE user_sessions
ADD COLUMN IF NOT EXISTS user_id VARCHAR(255) REFERENCES user_profiles(user_id);

-- Create index on user_id for efficient session lookups
CREATE INDEX IF NOT EXISTS idx_user_sessions_user_id ON user_sessions(user_id);

-- Add comment documenting tech debt
COMMENT ON COLUMN user_profiles.password IS 'TECH DEBT: Currently stores plaintext passwords. TODO: Implement bcrypt hashing post-hackathon.';
