-- Migration 004: Add Software and Hardware Background Fields
-- Feature 008: Auth + Personalization Integration
-- Created: 2025-12-19

-- Add software background fields
ALTER TABLE user_profiles ADD COLUMN IF NOT EXISTS software_experience VARCHAR(20);
ALTER TABLE user_profiles ADD COLUMN IF NOT EXISTS programming_languages JSONB;

-- Add hardware background fields
ALTER TABLE user_profiles ADD COLUMN IF NOT EXISTS hardware_type VARCHAR(30);
ALTER TABLE user_profiles ADD COLUMN IF NOT EXISTS hardware_experience BOOLEAN DEFAULT false;

-- Add indexes for performance
CREATE INDEX IF NOT EXISTS idx_user_profiles_hardware_type ON user_profiles(hardware_type);
CREATE INDEX IF NOT EXISTS idx_user_profiles_software_experience ON user_profiles(software_experience);

-- Verify migration
SELECT column_name, data_type, is_nullable
FROM information_schema.columns
WHERE table_name = 'user_profiles'
AND column_name IN ('software_experience', 'programming_languages', 'hardware_type', 'hardware_experience');
