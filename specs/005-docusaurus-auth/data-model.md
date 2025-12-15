# Data Model: Docusaurus Authentication System

**Feature**: 005-docusaurus-auth  
**Date**: 2025-12-13  
**Status**: Complete

## Overview

This document defines the data models for the authentication system based on the feature specification requirements. The models are designed to support user registration, authentication, profile management, and GDPR-compliant data deletion.

## Entity Definitions

### User Entity

**Description**: Core entity representing a registered user of the system

**Fields**:
- `id` (UUID, Primary Key, Auto-generated)
  - Purpose: Unique identifier for the user
  - Constraints: Required, Unique, Indexed
- `email` (VARCHAR(255), Unique, Not Null)
  - Purpose: User's email address for login
  - Constraints: Required, Unique, Valid email format, Indexed
- `password_hash` (VARCHAR(255), Not Null)
  - Purpose: Securely hashed password (handled by Better Auth)
  - Constraints: Required, Minimum 60 chars (bcrypt standard)
- `software_level` (ENUM: 'Beginner', 'Intermediate', 'Advanced', Not Null)
  - Purpose: User's software background from onboarding questions
  - Constraints: Required, One of the three values
- `hardware_access` (ENUM: 'Laptop/Cloud', 'RTX GPU', 'Jetson Edge', 'Physical Robot', Not Null)
  - Purpose: User's hardware access from onboarding questions
  - Constraints: Required, One of the four values
- `created_at` (TIMESTAMP, Default: NOW())
  - Purpose: Account creation timestamp
  - Constraints: Auto-generated on creation
- `updated_at` (TIMESTAMP, Default: NOW(), Auto-update on modification)
  - Purpose: Last modification timestamp
  - Constraints: Auto-updated on changes
- `last_login_at` (TIMESTAMP, Nullable)
  - Purpose: Last time user logged in successfully
  - Constraints: Nullable, Updated on login
- `email_verified` (BOOLEAN, Default: false)
  - Purpose: Whether the user's email has been verified
  - Constraints: Boolean, Default to false
- `verification_token` (VARCHAR(255), Nullable)
  - Purpose: Token for email verification
  - Constraints: Nullable, Unique, For verification process
- `is_active` (BOOLEAN, Default: true)
  - Purpose: Whether the account is active
  - Constraints: Boolean, Default to true, Used for soft deletion

**Relationships**:
- One-to-Many: User to PasswordResetTokens (user can request multiple resets)
- One-to-Many: User to Sessions (user can have multiple concurrent sessions)

**Validation Rules**:
- Email must be in valid format
- Software_level must be one of: 'Beginner', 'Intermediate', 'Advanced'
- Hardware_access must be one of: 'Laptop/Cloud', 'RTX GPU', 'Jetson Edge', 'Physical Robot'
- Password must meet requirements: 8+ chars, mixed case, number, special character

### Password Reset Token Entity

**Description**: Temporary token for password reset functionality

**Fields**:
- `id` (UUID, Primary Key, Auto-generated)
  - Purpose: Unique identifier for the reset token
  - Constraints: Required, Unique, Indexed
- `user_id` (UUID, Foreign Key to users.id, Not Null)
  - Purpose: Reference to the user requesting reset
  - Constraints: Required, References users table
- `token` (VARCHAR(255), Unique, Not Null)
  - Purpose: Secure token string for password reset
  - Constraints: Required, Unique, Indexed
- `expires_at` (TIMESTAMP, Not Null)
  - Purpose: When the reset token expires (1 hour from creation)
  - Constraints: Required, Future timestamp
- `used` (BOOLEAN, Default: false)
  - Purpose: Whether the token has been used
  - Constraints: Boolean, Default to false
- `created_at` (TIMESTAMP, Default: NOW())
  - Purpose: When the token was created
  - Constraints: Auto-generated on creation

**Relationships**:
- Many-to-One: PasswordResetTokens to User (many reset tokens per user)

**Validation Rules**:
- Token must be unique
- expires_at must be in the future
- Once used, token cannot be reused

### Session Entity (Managed by Better Auth)

**Description**: User session information (delegated to Better Auth)

**Note**: Better Auth manages sessions internally. The schema is implementation-dependent and may not require explicit definition in our custom tables.

## Data Model Constraints

### Primary Business Rules
1. Email uniqueness: No two users can have the same email address
2. Required onboarding: Both software_level and hardware_access must be provided during registration
3. GDPR compliance: User data must be completely removable upon account deletion
4. Password requirements: Passwords must meet specified security requirements
5. Session management: Sessions must expire after 30 days of inactivity

### Data Integrity Rules
1. Foreign key constraints must be maintained between related tables
2. User profile data must be consistent and valid
3. Password reset tokens must expire after 1 hour
4. Used password reset tokens cannot be reused
5. Email verification tokens must be removed after verification

## State Transitions

### User State Transitions
- `Unverified` → `Verified`: When email verification is completed
- `Active` → `Inactive`: When account is deleted by user
- `Inactive` → `Active`: Not allowed (GDPR compliance - deleted data cannot be restored)

## Indexes for Performance

### Required Indexes
1. `users.email` - For fast login lookup
2. `users.verification_token` - For email verification lookup
3. `password_reset_tokens.token` - For password reset lookup
4. `password_reset_tokens.expires_at` - For cleaning up expired tokens
5. `users.created_at` - For data retention queries

## GDPR Compliance Considerations

### Data Retention
- User profile data is retained until account deletion
- Password reset tokens are automatically cleaned up after expiration
- Session data is managed by Better Auth with appropriate timeouts

### Complete Deletion Requirements
- When a user deletes their account:
  1. All user data in the `users` table must be permanently removed
  2. All password reset tokens associated with the user must be removed
  3. Better Auth should invalidate all associated sessions
  4. No residual data should remain in the system