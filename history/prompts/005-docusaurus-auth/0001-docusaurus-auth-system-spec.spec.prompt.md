---
id: 0001
title: docusaurus-auth-system-spec
stage: spec
date: 2025-12-13
surface: agent
model: Qwen
feature: 005-docusaurus-auth
branch: 005-docusaurus-auth
user: user
command: /sp.specify
labels: ["auth", "docusaurus", "specification"]
links:
  spec: /specs/005-docusaurus-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - /specs/005-docusaurus-auth/spec.md
 - /specs/005-docusaurus-auth/checklists/requirements.md
tests:
 - specs_validated
---

## Prompt

I want to create an Authentication System for the Physical AI & Humanoid Robotics textbook (built with Docusaurus). Before writing the formal specification, help me think through the requirements: **Core Functionality:** 1. User Registration (Email + Password only, no OAuth) 2. Two onboarding questions AFTER email/password entry: - "What's your software background?" → Options: Beginner, Intermediate, Advanced(user can select only one) - "What hardware do you have access to?" → Options: Laptop/Cloud, RTX GPU, Jetson Edge, Physical Robot (User can select only one) 3. Login/Logout with persistent session (user stays logged in until manual logout) 4. Password reset functionality 5. User can delete their account (GDPR compliance) **Success Criteria (Must be testable):** - User can complete registration in 6-8 seconds max - Session persists indefinitely (until user manually logs out) - Password reset email delivered within 7-10 seconds - User profile stores: Email, Software Level, Hardware Access - Account deletion permanently removes all user data from database - Integration with existing Docusaurus site (zero breaking changes to current book) **Technical Constraints:** - Platform: Docusaurus (React-based static site) for book content - Auth Provider: Better Auth (https://www.better-auth.com/) - Backend API: Lightweight Next.js API routes (separate deployment for Better Auth server-side logic) - Database: Neon Serverless Postgres (to be set up fresh for this project) - Styling: Docusaurus built-in theming + custom CSS (dark and light mode friendly) - Mobile-responsive (must work on 320px+ screens) - GDPR compliant (user can permanently delete account and all data) - Architecture: Docusaurus frontend → Next.js API (Better Auth) → Neon Postgres **Integration Requirements:** - Must integrate seamlessly with existing Docusaurus book - No changes to current book content or navigation - Auth state should be accessible to future features (RAG chatbot, content personalization, translation) - Login/Signup UI should match Docusaurus theme(dark and light mode friendly) **Non-Goals (What we're NOT building now):** - RAG Chatbot integration — Future phase - OAuth login (Google/GitHub) — Future phase - Two-Factor Authentication (2FA) — Future phase - Admin dashboard — Separate spec - Content personalization buttons — Future phase - Translation feature — Future phase **Dependencies:** - Existing Docusaurus book deployment on GitHub Pages must continue working - Fresh Neon Postgres database setup required - Auth should be modular (can be reused for chatbot/personalization in future) Now, based on this conversation: 1. Create a clear specification with Intent, Success Criteria, Constraints, and Non-Goals 2. Focus on WHAT we're building, not HOW 3. Make all success criteria SMART (Specific, Measurable, Achievable, Relevant, Time-bound) 4. Explicitly state integration approach with Docusaurus 5. Include database schema requirements for user profiles create new branch with named "005-auth-system" then write the specification at: specs/005-auth-system/spec.md

## Response snapshot

# Feature Specification: Docusaurus Authentication System

**Feature Branch**: `005-docusaurus-auth`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "I want to create an Authentication System for the Physical AI & Humanoid Robotics textbook (built with Docusaurus). Before writing the formal specification, help me think through the requirements: **Core Functionality:** 1. User Registration (Email + Password only, no OAuth) 2. Two onboarding questions AFTER email/password entry: - "What's your software background?" → Options: Beginner, Intermediate, Advanced(user can select only one) - "What hardware do you have access to?" → Options: Laptop/Cloud, RTX GPU, Jetson Edge, Physical Robot (User can select only one) 3. Login/Logout with persistent session (user stays logged in until manual logout) 4. Password reset functionality 5. User can delete their account (GDPR compliance) **Success Criteria (Must be testable):** - User can complete registration in 6-8 seconds max - Session persists indefinitely (until user manual logs out) - Password reset email delivered within 7-10 seconds - User profile stores: Email, Software Level, Hardware Access - Account deletion permanently removes all user data from database - Integration with existing Docusaurus site (zero breaking changes to current book) **Technical Constraints:** - Platform: Docusaurus (React-based static site) for book content - Auth Provider: Better Auth (https://www.better-auth.com/) - Backend API: Lightweight Next.js API routes (separate deployment for Better Auth server-side logic) - Database: Neon Serverless Postgres (to be set up fresh for this project) - Styling: Docusaurus built-in theming + custom CSS (dark and light mode friendly) - Mobile-responsive (must work on 320px+ screens) - GDPR compliant (user can permanently delete account and all data) - Architecture: Docusaurus frontend → Next.js API (Better Auth) → Neon Postgres **Integration Requirements:** - Must integrate seamlessly with existing Docusaurus book - No changes to current book content or navigation - Auth state should be accessible to future features (RAG chatbot, content personalization, translation) - Login/Signup UI should match Docusaurus theme(dark and light mode friendly) **Non-Goals (What we're NOT building now):** - RAG Chatbot integration — Future phase - OAuth login (Google/GitHub) — Future phase - Two-Factor Authentication (2FA) — Future phase - Admin dashboard — Separate spec - Content personalization buttons — Future phase - Translation feature — Future phase **Dependencies:** - Existing Docusaurus book deployment on GitHub Pages must continue working - Fresh Neon Postgres database setup required - Auth should be modular (can be reused for chatbot/personalization in future) Now, based on this conversation: 1. Create a clear specification with Intent, Success Criteria, Constraints, and Non-Goals 2. Focus on WHAT we're building, not HOW 3. Make all success criteria SMART (Specific, Measurable, Achievable, Relevant, Time-bound) 4. Explicitly state integration approach with Docusaurus 5. Include database schema requirements for user profiles create new branch with named "005-auth-system" then write the specification at: specs/005-auth-system/spec.md"

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

**Why this priority**: Password recovery is essential for account accessibility and user retention. Without this feature, users would be locked out of their accounts permanently.

**Independent Test**: A user can initiate password reset from the login page, receive an email with reset instructions, follow the link, and successfully set a new password.

**Acceptance Scenarios**:

1. **Given** a user clicks the "Forgot Password" link, **When** they enter their registered email address, **Then** they receive an email with password reset instructions within 7-10 seconds
2. **Given** a user follows the reset link in their email, **When** they enter a new password and confirm it, **Then** their password is updated and they can log in with the new credentials

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

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to register with email address and password only (no OAuth)
- **FR-002**: System MUST present two onboarding questions after email/password entry during registration:
  - "What's your software background?" with options: Beginner, Intermediate, Advanced (single selection)
  - "What hardware do you have access to?" with options: Laptop/Cloud, RTX GPU, Jetson Edge, Physical Robot (single selection)
- **FR-003**: System MUST authenticate users via email and password
- **FR-004**: System MUST maintain persistent sessions until manual logout
- **FR-005**: System MUST provide password reset functionality via email with reset links
- **FR-006**: System MUST allow users to permanently delete their accounts and all associated data
- **FR-007**: System MUST store user profile data: Email, Software Level, Hardware Access
- **FR-008**: System MUST be GDPR compliant by providing complete account deletion
- **FR-009**: System MUST integrate seamlessly with the existing Docusaurus site without breaking changes
- **FR-010**: System MUST provide mobile-responsive UI that works on screens 320px and larger
- **FR-011**: System MUST support both light and dark mode themes consistent with Docusaurus
- **FR-012**: System MUST provide access to auth state for future features (RAG chatbot, content personalization, translation)

### Key Entities

- **User Profile**: Core entity representing a registered user
  - Attributes: Email (unique), Encrypted Password, Software Level (Beginner/Intermediate/Advanced), Hardware Access (Laptop/Cloud/RTX GPU/Jetson Edge/Physical Robot), Creation Date, Last Login Date
  - Related to: Authentication sessions, future personalized content features

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

## Outcome

- ✅ Impact: Created complete specification for Docusaurus authentication system
- 🧪 Tests: Specification validated against quality checklist
- 📁 Files: spec.md and requirements checklist created
- 🔁 Next prompts: /sp.plan to create implementation plan
- 🧠 Reflection: Specification addresses all core requirements with clear user stories and success criteria

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Plan implementation of authentication system
