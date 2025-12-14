-- Rollback Migration 001
-- Date: 2025-12-11
-- Description: Rollback schema changes from 001_create_tables.sql

-- Drop trigger first
DROP TRIGGER IF EXISTS trg_update_conversation_timestamp ON messages;

-- Drop function
DROP FUNCTION IF EXISTS update_conversation_timestamp();

-- Drop tables in reverse order (respecting foreign key constraints)
DROP TABLE IF EXISTS messages CASCADE;
DROP TYPE IF EXISTS message_role;
DROP TABLE IF EXISTS conversations CASCADE;
DROP TABLE IF EXISTS user_sessions CASCADE;

-- Note: We don't drop the uuid-ossp extension as it might be used by other tables
-- Uncomment the line below if you want to drop it:
-- DROP EXTENSION IF EXISTS "uuid-ossp";
