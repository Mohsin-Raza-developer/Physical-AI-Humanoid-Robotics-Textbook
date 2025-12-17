-- Migration: Add first_name and last_name to users table
-- Purpose: Store user's full name for personalized experience

-- Add first_name column
ALTER TABLE users 
ADD COLUMN IF NOT EXISTS first_name VARCHAR(100);

-- Add last_name column
ALTER TABLE users 
ADD COLUMN IF NOT EXISTS last_name VARCHAR(100);

-- Update existing users with placeholder names (extracted from email)
UPDATE users 
SET first_name = SPLIT_PART(email, '@', 1),
    last_name = 'User'
WHERE first_name IS NULL;

-- Make columns NOT NULL after updating existing records
ALTER TABLE users 
ALTER COLUMN first_name SET NOT NULL,
ALTER COLUMN last_name SET NOT NULL;

-- Add index for faster name searches
CREATE INDEX IF NOT EXISTS idx_users_first_name ON users(first_name);
CREATE INDEX IF NOT EXISTS idx_users_last_name ON users(last_name);

-- Verify migration
SELECT COUNT(*) as total_users, 
       COUNT(first_name) as users_with_first_name,
       COUNT(last_name) as users_with_last_name
FROM users;
