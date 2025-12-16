-- Migration 002: Create user profiles table for Better-Auth integration
-- Date: 2025-12-16
-- Description: Add user_profiles table to store Better-Auth user data with hardware profiles

-- Table: user_profiles
-- Purpose: Store user profiles from Better-Auth with detailed hardware information
CREATE TABLE IF NOT EXISTS user_profiles (
    user_id VARCHAR(255) PRIMARY KEY,  -- Better-Auth user ID
    email VARCHAR(255) NOT NULL UNIQUE,
    name VARCHAR(255),
    hardware_type VARCHAR(50) NULL,     -- 'gpu_workstation', 'edge_device', 'cloud_mac'
    hardware_details JSONB NULL,        -- Detailed hardware specs
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes for user_profiles
CREATE INDEX IF NOT EXISTS idx_user_profiles_email ON user_profiles(email);
CREATE INDEX IF NOT EXISTS idx_user_profiles_hardware_type ON user_profiles(hardware_type);

-- Comments for user_profiles
COMMENT ON TABLE user_profiles IS 'User profiles from Better-Auth with hardware configuration details';
COMMENT ON COLUMN user_profiles.hardware_type IS 'Type of hardware user has: gpu_workstation, edge_device, or cloud_mac';
COMMENT ON COLUMN user_profiles.hardware_details IS 'JSON object containing detailed hardware specs (gpu_model, cpu_model, ram_size, os_type, etc.)';

-- Trigger: Auto-update updated_at timestamp
CREATE OR REPLACE FUNCTION update_user_profile_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Drop trigger if exists (for idempotency)
DROP TRIGGER IF EXISTS trg_update_user_profile_timestamp ON user_profiles;

CREATE TRIGGER trg_update_user_profile_timestamp
BEFORE UPDATE ON user_profiles
FOR EACH ROW
EXECUTE FUNCTION update_user_profile_timestamp();