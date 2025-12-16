-- Rollback Migration 002: Remove user profiles table
-- Date: 2025-12-16
-- Description: Remove user_profiles table and related triggers

-- Drop trigger
DROP TRIGGER IF EXISTS trg_update_user_profile_timestamp ON user_profiles;

-- Drop function
DROP FUNCTION IF EXISTS update_user_profile_timestamp();

-- Drop indexes
DROP INDEX IF EXISTS idx_user_profiles_email;
DROP INDEX IF EXISTS idx_user_profiles_hardware_type;

-- Drop table
DROP TABLE IF EXISTS user_profiles;