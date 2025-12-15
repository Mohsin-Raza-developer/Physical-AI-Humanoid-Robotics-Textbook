# Feature Specification: Docusaurus Authentication System

**Feature Branch**: `005-docusaurus-auth`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "I want to create an Authentication System for the Physical AI & Humanoid Robotics textbook (built with Docusaurus). Before writing the formal specification, help me think through the requirements: **Core Functionality:** 1. User Registration (Email + Password only, no OAuth) 2. Two onboarding questions AFTER email/password entry: - "What's your software background?" → Options: Beginner, Intermediate, Advanced(user can select only one) - "What hardware do you have access to?" → Options: Laptop/Cloud, RTX GPU, Jetson Edge, Physical Robot (User can select only one) 3. Login/Logout with persistent session (user stays logged in until manual logout) 4. Password reset functionality 5. User can delete their account (GDPR compliance) **Success Criteria (Must be testable):** - User can complete registration in 6-8 seconds max - Session persists indefinitely (until user manually logs out) - Password reset email delivered within 7-10 seconds - User profile stores: Email, Software Level, Hardware Access - Account deletion permanently removes all user data from database - Integration with existing Docusaurus site (zero breaking changes to current book) **Technical Constraints:** - Platform: Docusaurus (React-based static site) for book content - Auth Provider: Better Auth (https://www.better-auth.com/) - Backend API: Lightweight Next.js API routes (separate deployment for Better Auth server-side logic) - Database: Neon Serverless Postgres (to be set up fresh for this project) - Styling: Docusaurus built-in theming + custom CSS (dark and light mode friendly) - Mobile-responsive (must work on 320px+ screens) - GDPR compliant (user can permanently delete account and all data) - Architecture: Docusaurus frontend → Next.js API (Better Auth) → Neon Postgres **Integration Requirements:** - Must integrate seamlessly with existing Docusaurus book - No changes to current book content or navigation - Auth state should be accessible to future features (RAG chatbot, content personalization, translation) - Login/Signup UI should match Docusaurus theme(dark and light mode friendly) **Non-Goals (What we're NOT building now):** - RAG Chatbot integration — Future phase - OAuth login (Google/GitHub) — Future phase - Two-Factor Authentication (2FA) — Future phase - Admin dashboard — Separate spec - Content personalization buttons — Future phase - Translation feature — Future phase **Dependencies:** - Existing Docusaurus book deployment on GitHub Pages must continue working - Fresh Neon Postgres database setup required - Auth should be modular (can be reused for chatbot/personalization in future) Now, based on this conversation: 1. Create a clear specification with Intent, Success Criteria, Constraints, and Non-Goals 2. Focus on WHAT we're building, not HOW 3. Make all success criteria SMART (Specific, Measurable, Achievable, Relevant, Time-bound) 4. Explicitly state integration approach with Docusaurus 5. Include database schema requirements for user profiles create new branch with named "005-auth-system" then write the specification at: specs/005-auth-system/spec.md"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Registration (Priority: P1)

A new visitor to the Physical AI & Humanoid Robotics textbook wants to create an account to access personalized content and features. The user enters their email and password, then answers two onboarding questions about their software background and hardware access.

**Why this priority**: This is the foundational feature that enables all other authenticated experiences. Without registration, users cannot access personalized content or track their progress.

**Independent Test**: A visitor can successfully create a new account by providing email, password, and completing the two onboarding questions, then be redirected to a welcome page.

**Acceptance Scenarios**:

1. **Given** a visitor is on the registration page, **When** they enter a valid email, password, and select both onboarding question options, **Then** they receive a confirmation that their account is created and can access authenticated features
2. **Given** a visitor enters invalid email format, **When** they submit the registration form, **Then** they receive a clear error message indicating the email format is invalid
3. **Given** a visitor enters a password that doesn't meet minimum requirements, **When** they submit the registration form, **Then** they receive a clear error message indicating the password requirements

---

### User Story 2 - User Login & Session Persistence (Priority: P1)

A registered user wants to access the textbook platform with their credentials and stay logged in across browser sessions until they manually log out.

**Why this priority**: Without reliable login and persistent sessions, users cannot access their personalized content or maintain their learning progress. Session persistence is critical for creating a seamless user experience.

**Independent Test**: A user can log in with their email and password, close the browser, reopen it later, and remain authenticated without having to log in again.

**Acceptance Scenarios**:

1. **Given** a user enters valid credentials, **When** they submit the login form, **Then** they are authenticated and redirected to their last viewed content or dashboard
2. **Given** a user is logged in, **When** they close and reopen their browser after some time, **Then** they remain logged in and can access authenticated features without re-authentication
3. **Given** a user wants to log out, **When** they click the logout button, **Then** their session is terminated and they are redirected to the public content

---

### User Story 3 - Password Reset (Priority: P2)

A user who has forgotten their password needs to regain access to their account through a secure password reset process.

**Password Reset Flow:**
1. User clicks "Forgot Password" or "Reset Password" button on login page
2. System displays email input form
3. User enters their email address
4. System validates:
   - If email is registered in database: Send password reset email (within 7-10 seconds)
   - If email is NOT registered or invalid: Display clear alert message: "This email is not registered. Please check your email address or sign up for a new account."
5. User receives email with secure reset link (valid for 1 hour)
6. User clicks the link in email
7. System displays "Set New Password" form
8. User enters new password and confirms it
9. System validates password requirements and updates the password
10. User is redirected to login page with success message

**Why this priority**: Password recovery is essential for account accessibility and user retention. Without this feature, users would be locked out of their accounts permanently.

**Independent Test**: A user can initiate password reset from the login page, receive an email with reset instructions, follow the link, and successfully set a new password.

**Acceptance Scenarios**:

1. **Given** a user clicks the "Forgot Password" link, **When** they enter their registered email address, **Then** they receive an email with password reset instructions within 7-10 seconds
2. **Given** a user follows the reset link in their email, **When** they enter a new password and confirm it, **Then** their password is updated and they can log in with the new credentials
3. **Given** a user enters an unregistered email address, **When** they submit the reset request, **Then** they see an alert message indicating the email is not registered
4. **Given** a user enters an invalid email format, **When** they submit the reset request, **Then** they see an error message indicating the email format is invalid
5. **Given** a user clicks an expired reset link (older than 1 hour), **When** they try to reset password, **Then** they see an error message indicating the link has expired and are prompted to request a new one

---

### User Story 4 - Account Deletion (Priority: P2)

A user who no longer wants to use the service needs to permanently delete their account and all associated data to comply with GDPR requirements.

**Why this priority**: GDPR compliance is a legal requirement for handling user data in the EU. Users must be able to completely remove their personal information from our systems.

**Independent Test**: A user can access their account settings, initiate account deletion, confirm the action, and have all their personal data permanently removed from the database.

**Acceptance Scenarios**:

1. **Given** a logged-in user navigates to account settings, **When** they select "Delete Account" and confirm the action, **Then** their account and all associated data are permanently removed from the database
2. **Given** a user has deleted their account, **When** they try to log in with previous credentials, **Then** they receive an error indicating the account doesn't exist

---

### Edge Cases

- What happens when a user tries to register with an email that is already in use? System should display a clear error message indicating the email is already registered.
- How does the system handle invalid onboarding question selections? System should require both questions to be answered before allowing registration to proceed.
- What if the password reset email delivery fails? System should display an appropriate error message and allow the user to retry the reset request.
- How does the system handle concurrent sessions on multiple devices? System should maintain consistent states across devices, and logging out on one device should not affect other sessions (unless specified otherwise).
- What happens when a user enters an invalid email format during password reset? System should display an appropriate error message indicating the email format is invalid.
- What happens when a user attempts to use an expired password reset link? System should indicate that the link has expired and prompt the user to request a new reset email.
- What happens when a user tries to use a password reset link that has already been used? System should indicate that the link is no longer valid and prompt the user to request a new reset email.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to register with email address and password only (no OAuth)
- **FR-002**: System MUST present two onboarding questions after email/password entry during registration:
  - "What's your software background?" with options: Beginner, Intermediate, Advanced (single selection)
  - "What hardware do you have access to?" with options: Laptop/Cloud, RTX GPU, Jetson Edge, Physical Robot (single selection)
- **FR-003**: System MUST authenticate users via email and password
- **FR-004**: System MUST maintain persistent sessions with 30-day inactivity timeout until manual logout
- **FR-005**: System MUST provide password reset functionality via email with reset links
- **FR-006**: System MUST allow users to permanently delete their accounts and all associated data
- **FR-007**: System MUST store user profile data: Email, Software Level, Hardware Access
- **FR-008**: System MUST be GDPR compliant by providing complete account deletion
- **FR-009**: System MUST integrate seamlessly with the existing Docusaurus site without breaking changes
- **FR-010**: System MUST provide mobile-responsive UI that works on screens 320px and larger
- **FR-011**: System MUST support both light and dark mode themes consistent with Docusaurus
- **FR-012**: System MUST provide access to auth state for future features (RAG chatbot, content personalization, translation)
- **FR-013**: System MUST validate email format during registration and password reset
- **FR-014**: System MUST send password reset emails within 7-10 seconds of request
- **FR-015**: System MUST validate that password reset tokens are valid and not expired (1 hour validity)
- **FR-016**: System MUST provide clear error messaging for unregistered email addresses during password reset
- **FR-017**: System MUST prevent reuse of password reset tokens once used
- **FR-018**: System MUST enforce password requirements (8 characters, mixed case, number, special char) during registration and password reset
- **FR-019**: System MUST implement 30-day inactivity timeout for persistent sessions (sessions expire after 30 days of inactivity or on manual logout)

### Key Entities

- **User Profile**: Core entity representing a registered user
  - Attributes: Email (unique), Encrypted Password, Software Level (Beginner/Intermediate/Advanced), Hardware Access (Laptop/Cloud/RTX GPU/Jetson Edge/Physical Robot), Creation Date, Last Login Date
  - Related to: Authentication sessions, future personalized content features

## Database Schema

### Table: `users`
- `id` (UUID, Primary Key, Auto-generated)
- `email` (VARCHAR(255), Unique, Not Null)
- `password_hash` (VARCHAR(255), Not Null) - Encrypted by Better Auth
- `software_level` (ENUM: 'Beginner', 'Intermediate', 'Advanced', Not Null)
- `hardware_access` (ENUM: 'Laptop/Cloud', 'RTX GPU', 'Jetson Edge', 'Physical Robot', Not Null)
- `created_at` (TIMESTAMP, Default: NOW())
- `updated_at` (TIMESTAMP, Default: NOW(), Auto-update on modification)
- `last_login_at` (TIMESTAMP, Nullable)

### Table: `password_reset_tokens` (For tracking reset links)
- `id` (UUID, Primary Key, Auto-generated)
- `user_id` (UUID, Foreign Key to users.id)
- `token` (VARCHAR(255), Unique, Not Null)
- `expires_at` (TIMESTAMP, Not Null) - Set to 1 hour from creation
- `used` (BOOLEAN, Default: false)
- `created_at` (TIMESTAMP, Default: NOW())

### Table: `sessions` (Managed by Better Auth)
- Defer to Better Auth's internal session management schema

## Constraints

- **Technical Stack**: Better Auth for authentication, Next.js API routes for backend, Neon Postgres for database
- **Integration**: Must work alongside existing Docusaurus deployment without replacing it
- **Architecture**: Docusaurus frontend → Next.js API (Better Auth) → Neon Postgres
- **Performance**: Registration in 6-8 seconds, password reset email in 7-10 seconds
- **Scalability**: System must support 1000 concurrent users
- **Reliability**: System must maintain 99.5% uptime
- **Security**: System must implement standard auth security practices (password policies, secure session handling, brute force protection)
- **Observability**: System must implement basic logging and metrics
- **UI/UX**: Must support Docusaurus dark/light theme modes
- **Mobile**: Minimum screen width support: 320px
- **Compliance**: GDPR-compliant account deletion
- **Timeline**: [To be determined based on project roadmap]

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: New users can complete the entire registration process (email, password, both onboarding questions) in 6-8 seconds maximum
- **SC-002**: User sessions persist indefinitely until manual logout, with authenticated state maintained across browser restarts and device reboots
- **SC-003**: Password reset emails are delivered to users within 7-10 seconds of requesting the reset
- **SC-004**: User profile information (Email, Software Level, Hardware Access) is accurately stored and retrievable after registration
- **SC-005**: When a user deletes their account, all personal data is permanently removed from the database with no residual information remaining
- **SC-006**: Integration with the existing Docusaurus site occurs without breaking changes to current book content or navigation
- **SC-007**: The authentication system UI is responsive and functional on screen sizes down to 320px wide
- **SC-008**: The authentication UI supports both light and dark modes consistent with the Docusaurus theme

## Non-Goals

- OAuth login (Google/GitHub) — Future phase
- Two-Factor Authentication (2FA) — Future phase
- RAG Chatbot integration — Separate feature
- Content personalization UI — Separate feature
- Translation feature — Separate feature
- Admin dashboard — Separate feature
- Course progress tracking — Separate feature
- Payment/subscription system — Not applicable

## Clarifications

### Session 2025-12-13

- Q: What is the expected scale for concurrent users? → A: Support 1000 concurrent users
- Q: What are the observability requirements? → A: Basic logging and metrics
- Q: What are the security requirements beyond GDPR? → A: Standard auth security practices
- Q: What are the session management requirements? → A: Persistent sessions with 30-day inactivity timeout
- Q: What are the password requirements? → A: 8 characters, mixed case, number, special char
- Q: What is the required system uptime? → A: 99.5% uptime